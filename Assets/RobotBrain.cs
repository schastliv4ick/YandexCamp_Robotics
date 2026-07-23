using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;

/// <summary>
/// Архитектура Артёма (deathlydh), портированная на наши компоненты.
///
/// Скопировано целиком:
///  - 15 наблюдений в его порядке (4 сенсора + 5 зрение + 1 клешня + 2 смещение + heading + speed + timeSinceSeen)
///  - Фиксированные награды-константы (НЕ континуальные коэффициенты)
///  - Клешня автономна (НЕ действие сети): ИК + окно "мяч был виден недавно" + анти-дребезг
///  - Задержка действий (8..13) + задержка сенсоров (2 шага)
///  - Раздельный шум зрения: дистанция в 3x шумнее угла
///  - Burst dropout, усиленный вращением корпуса и резким поворотом камеры
///  - Retry-машина: слепой подъезд -> отъезд назад -> до 2 повторов
///  - Anti-stuck (200 тиков / 0.5 м) + лимит эпизода из config
///  - 360-градусный спавн мяча с проверкой стен и пола
///  - Camera Step Limit 15 градусов/тик (реальное серво)
///  - DR из config: ball_max_distance, ball_scale, ball_mass, vision_noise, vision_dropout, episode_length
///
/// ВАЖНО для Inspector:
///  - Space Size = 15, Stacked Vectors = 4
///  - Continuous Actions = 3, Discrete Branches = 0 (клешня больше НЕ действие сети!)
/// </summary>
[RequireComponent(typeof(TrackController))]
[RequireComponent(typeof(VirtualSensors))]
public class RobotBrain : Agent
{
    [Header("References")]
    [SerializeField] private TrackController trackController;
    [SerializeField] private GripperController gripperController;
    [SerializeField] private VirtualSensors virtualSensors;
    [SerializeField] private SimulatedYoloCamera yoloCamera;
    [SerializeField] private Transform cameraPivot;

    [Header("Target")]
    [SerializeField] private Rigidbody targetBall;

    [Header("Latency Simulation (Sim-to-Real)")]
    [Tooltip("Минимальная задержка действий (шаги FixedUpdate). 1 шаг ~ 20мс при 50Hz.")]
    public int minActionLatency = 8;
    [Tooltip("Максимальная задержка действий (рандомизируется каждый эпизод).")]
    public int maxActionLatency = 13;
    [Tooltip("Задержка сенсоров (шаги). Имитирует запаздывание ROS-топиков.")]
    public int sensorLatency = 2;

    [Header("Camera servo")]
    [SerializeField] private float cameraPivotMaxAngle = 90f;

    private TrackController track;
    private VirtualSensors sensors;
    private Rigidbody rb;
    private DiagnosticLogger diagLogger;

    // --- Задержки ---
    private int currentActionLatency = 3;
    private Queue<float[]> actionBuffer = new Queue<float[]>();
    private Queue<float[]> sensorBuffer = new Queue<float[]>();
    private float[] delayedSensors = new float[4]; // UZ, L_IR, R_IR, CLAW_IR

    private Vector3 startPosition;
    private Quaternion startRotation;

    // --- Награды/состояние ---
    private float lastDistance = 1f;
    private bool wasSeeingBallLastStep = false;
    private int holdTicks = 0;

    private float prevGas = 0f;
    private float prevSteering = 0f;
    private float prevCameraYaw = 0f;

    // --- Слепой захват ---
    private bool wasCloseToBall = false;
    private int blindApproachTicks = 0;
    private const int BLIND_APPROACH_MAX = 80;

    // --- Retry ---
    private bool isRetrying = false;
    private int retryBackupTicks = 0;
    private int retryCount = 0;
    private const int MAX_RETRIES = 2;
    private const int RETRY_BACKUP_DURATION = 80;

    // --- Anti-stuck ---
    private Vector3 lastPosition;
    private int stuckTimer = 0;

    // --- Burst dropout ---
    private int burstDropoutRemaining = 0;
    private float lastCamDelta = 0f;

    // --- Камера/серво ---
    private float currentCameraYaw = 0f;
    private const float MAX_CAMERA_STEP_NORMALIZED = 15f / 90f; // 15 градусов из 90

    // --- Зрение: память ---
    private float lastKnownBallDirection = 0f;
    private float lastDetectionTime = 0f;

    // --- Клешня: окно "мяч был виден недавно" + анти-дребезг ---
    private int lastBallSeenStep = -999;
    private const int BALL_SEEN_WINDOW = 50;
    private int holdWithoutIR = 0;
    private const int HOLD_WITHOUT_IR_MAX = 100;

    private float startBallMass = 0.1f;

    private bool episodeOutcomeLogged = false;
    private bool isFirstEpisode = true;

    private bool IsTraining => Academy.Instance.IsCommunicatorOn;

    public override void Initialize()
    {
        track = trackController != null ? trackController : GetComponent<TrackController>();
        sensors = virtualSensors != null ? virtualSensors : GetComponent<VirtualSensors>();
        rb = GetComponent<Rigidbody>();
        diagLogger = GetComponent<DiagnosticLogger>();

        startPosition = transform.position;
        startRotation = transform.rotation;
        lastPosition = transform.position;

        if (targetBall != null) startBallMass = targetBall.mass;

        if (targetBall == null) Debug.LogWarning("[RobotBrain] targetBall не назначен!");
        if (yoloCamera == null) Debug.LogWarning("[RobotBrain] yoloCamera не назначен!");
        if (gripperController == null) Debug.LogWarning("[RobotBrain] gripperController не назначен!");
    }

    public override void OnEpisodeBegin()
    {
        if (!isFirstEpisode && !episodeOutcomeLogged) LogEpisodeOutcome("timeout");
        isFirstEpisode = false;
        episodeOutcomeLogged = false;

        // Физическая рандомизация робота
        if (IsTraining)
        {
            if (rb != null) rb.mass = UnityEngine.Random.Range(1.0f, 4.0f);
            if (track != null)
            {
                track.moveSpeed = UnityEngine.Random.Range(0.3f, 0.7f);
                track.turnSpeed = UnityEngine.Random.Range(80f, 160f);
            }
        }

        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }
        transform.SetPositionAndRotation(startPosition, startRotation);

        ResetBall();

        // Сброс состояния
        lastDistance = 1f;
        wasSeeingBallLastStep = false;
        holdTicks = 0;
        prevGas = 0f;
        prevSteering = 0f;
        prevCameraYaw = 0f;
        wasCloseToBall = false;
        blindApproachTicks = 0;
        isRetrying = false;
        retryBackupTicks = 0;
        retryCount = 0;
        stuckTimer = 0;
        lastPosition = transform.position;
        burstDropoutRemaining = 0;
        lastCamDelta = 0f;
        currentCameraYaw = 0f;
        lastKnownBallDirection = 0f;
        lastDetectionTime = Time.time;
        lastBallSeenStep = -999;
        holdWithoutIR = 0;
        if (cameraPivot != null) cameraPivot.localRotation = Quaternion.identity;

        // Задержка действий: рандом каждый эпизод
        currentActionLatency = IsTraining
            ? UnityEngine.Random.Range(minActionLatency, maxActionLatency + 1) : 0;
        actionBuffer.Clear();
        for (int i = 0; i < currentActionLatency; i++)
            actionBuffer.Enqueue(new float[] { 0f, 0f, 0f });

        sensorBuffer.Clear();
        delayedSensors = new float[] { 1f, 0f, 0f, 0f };
    }

    /// <summary>15 наблюдений в порядке Артёма.</summary>
    public override void CollectObservations(VectorSensor sensor)
    {
        var envParams = Academy.Instance.EnvironmentParameters;

        // --- Шум сенсоров ---
        float noiseUS = IsTraining ? UnityEngine.Random.Range(-0.05f, 0.05f) : 0f;

        // Раздельный шум зрения: угол точный (bbox center), дистанция шумная (bbox height)
        float noiseAmp = IsTraining ? envParams.GetWithDefault("vision_noise", 0.02f) : 0f;
        float noiseVisAngle = IsTraining ? UnityEngine.Random.Range(-noiseAmp, noiseAmp) : 0f;
        float noiseVisDist = IsTraining ? UnityEngine.Random.Range(-noiseAmp * 3f, noiseAmp * 3f) : 0f;

        // --- Burst dropout: усиливается вращением корпуса и резким поворотом камеры ---
        float dropoutRate = IsTraining ? envParams.GetWithDefault("vision_dropout", 0f) : 0f;
        float bodyRotDropout = (rb != null && rb.angularVelocity.magnitude > 0.5f) ? 0.12f : 0f;
        float effectiveDropout = dropoutRate + (lastCamDelta > 0.3f ? 0.15f : 0f) + bodyRotDropout;

        if (burstDropoutRemaining > 0) burstDropoutRemaining--;
        else if (IsTraining && UnityEngine.Random.value < effectiveDropout)
            burstDropoutRemaining = UnityEngine.Random.Range(5, 16);
        bool yoloDropout = burstDropoutRemaining > 0;

        // --- 1-4: сенсоры с задержкой ---
        if (IsTraining && sensorLatency > 0 && sensors != null)
        {
            sensorBuffer.Enqueue(new float[] {
                Mathf.Clamp01(sensors.USNormalizedDistance + noiseUS),
                sensors.LeftIRObstacle,
                sensors.RightIRObstacle,
                sensors.GripperIRBallDetected
            });
            if (sensorBuffer.Count > sensorLatency) delayedSensors = sensorBuffer.Dequeue();

            sensor.AddObservation(delayedSensors[0]);
            sensor.AddObservation(delayedSensors[1]);
            sensor.AddObservation(delayedSensors[2]);
            sensor.AddObservation(delayedSensors[3]);
        }
        else if (sensors != null)
        {
            sensor.AddObservation(Mathf.Clamp01(sensors.USNormalizedDistance + noiseUS));
            sensor.AddObservation(sensors.LeftIRObstacle);
            sensor.AddObservation(sensors.RightIRObstacle);
            sensor.AddObservation(sensors.GripperIRBallDetected);
        }
        else
        {
            sensor.AddObservation(1f);
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
        }

        // --- 5-9: зрение ---
        bool ballVisible = yoloCamera != null && yoloCamera.IsBallVisible && !yoloDropout;
        if (ballVisible)
        {
            lastKnownBallDirection = yoloCamera.RelativeAngle;
            lastDetectionTime = Time.time;
            lastBallSeenStep = StepCount;
        }

        if (yoloCamera != null)
        {
            sensor.AddObservation(ballVisible ? Mathf.Clamp(yoloCamera.RelativeAngle + noiseVisAngle, -1f, 1f) : 0f);
            sensor.AddObservation(ballVisible ? Mathf.Clamp01(yoloCamera.NormalizedDistance + noiseVisDist) : 1f);
            sensor.AddObservation(lastKnownBallDirection);
            sensor.AddObservation(ballVisible ? 1f : 0f);
            sensor.AddObservation(currentCameraYaw);
        }
        else
        {
            sensor.AddObservation(0f);
            sensor.AddObservation(1f);
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
        }

        // --- 10: клешня ---
        sensor.AddObservation(gripperController != null && gripperController.IsHolding ? 1f : 0f);

        // --- 11-12: смещение от старта ---
        Vector3 displacement = transform.position - startPosition;
        sensor.AddObservation(Mathf.Clamp(displacement.x / 3f, -1f, 1f));
        sensor.AddObservation(Mathf.Clamp(displacement.z / 3f, -1f, 1f));

        // --- 13: курс ---
        sensor.AddObservation(transform.eulerAngles.y / 360f);

        // --- 14: скорость ---
        sensor.AddObservation(rb != null ? Mathf.Clamp01(rb.linearVelocity.magnitude / 0.5f) : 0f);

        // --- 15: время с последней детекции ---
        sensor.AddObservation(Mathf.Clamp01((Time.time - lastDetectionTime) / 5f));
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        var envParams = Academy.Instance.EnvironmentParameters;

        float gas, steering, cameraYawInput;

        // --- Задержка действий (FIFO) ---
        if (IsTraining && currentActionLatency > 0)
        {
            actionBuffer.Enqueue(new float[] {
                Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f),
                Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f),
                Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f)
            });
            float[] delayed = actionBuffer.Dequeue();
            gas = delayed[0]; steering = delayed[1]; cameraYawInput = delayed[2];
        }
        else
        {
            gas = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
            steering = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
            cameraYawInput = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);
        }

        // ============================================================
        // ФАЗА 0: МЯЧ СХВАЧЕН — проверяем самым первым
        // ============================================================
        if (gripperController != null && gripperController.IsHolding)
        {
            holdTicks++;
            stuckTimer = 0;
            AddReward(0.02f);

            if (track != null) track.Move(0f, 0f);

            if (IsTraining && holdTicks >= 50)
            {
                AddReward(5.0f);
                Academy.Instance.StatsRecorder.Add("Custom/GrabSuccess", 1.0f);
                LogEpisodeOutcome("success");
                EndEpisode();
                return;
            }
            return;
        }
        holdTicks = 0;

        // === ЛИМИТ ЭПИЗОДА (из config) ===
        int episodeLimit = Mathf.RoundToInt(envParams.GetWithDefault("episode_length", 800f));
        if (IsTraining && StepCount >= episodeLimit)
        {
            AddReward(-0.05f);
            Academy.Instance.StatsRecorder.Add("Custom/GrabSuccess", 0.0f);
            LogEpisodeOutcome("timeout");
            EndEpisode();
            return;
        }

        // === АНТИ-ЗАСТРЕВАНИЕ ===
        if (Mathf.Abs(gas) > 0.1f || Mathf.Abs(steering) > 0.1f)
        {
            stuckTimer++;
            if (stuckTimer >= 200)
            {
                float distanceTravelled = Vector3.Distance(transform.position, lastPosition);
                if (distanceTravelled < 0.5f)
                {
                    AddReward(-0.5f);
                    if (IsTraining) Academy.Instance.StatsRecorder.Add("Custom/GrabSuccess", 0.0f);
                    LogEpisodeOutcome("timeout");
                    EndEpisode();
                    return;
                }
                stuckTimer = 0;
                lastPosition = transform.position;
            }
        }
        else
        {
            stuckTimer = 0;
            lastPosition = transform.position;
        }

        // === CAMERA STEP LIMIT: реальное серво макс 15 градусов/тик ===
        float targetCamYaw = Mathf.Clamp(cameraYawInput, -1f, 1f);
        float camDelta = targetCamYaw - currentCameraYaw;
        if (Mathf.Abs(camDelta) > MAX_CAMERA_STEP_NORMALIZED)
            targetCamYaw = currentCameraYaw + Mathf.Sign(camDelta) * MAX_CAMERA_STEP_NORMALIZED;
        lastCamDelta = Mathf.Abs(targetCamYaw - currentCameraYaw);
        currentCameraYaw = targetCamYaw;
        if (cameraPivot != null)
            cameraPivot.localRotation = Quaternion.Euler(0f, currentCameraYaw * cameraPivotMaxAngle, 0f);

        // === ДВИЖЕНИЕ ===
        if (track != null) track.Move(gas, steering);

        // === КЛЕШНЯ: автономна, по ИК + окно "мяч был виден недавно" ===
        bool gripperSensorActive = sensors != null && sensors.GripperIRBallDetected > 0.5f;
        if (gripperController != null)
        {
            bool ballRecentlySeen = (StepCount - lastBallSeenStep) < BALL_SEEN_WINDOW;
            bool allowGrip = IsTraining || ballRecentlySeen;

            if (!gripperController.IsHolding && gripperSensorActive && allowGrip)
            {
                gripperController.GripperCloseCommand = true;
            }
            else if (gripperController.IsHolding && !gripperSensorActive)
            {
                // Анти-дребезг: не разжимаем мгновенно, ждём 2 секунды без ИК
                holdWithoutIR++;
                if (holdWithoutIR >= HOLD_WITHOUT_IR_MAX)
                {
                    gripperController.GripperCloseCommand = false;
                    holdWithoutIR = 0;
                }
            }
            else if (gripperController.IsHolding && gripperSensorActive)
            {
                holdWithoutIR = 0;
            }
        }

        // === REWARD #4: SENSOR PROXIMITY (штраф по датчикам, не по коллизиям) ===
        if (IsTraining && sensors != null)
        {
            if (sensors.USNormalizedDistance < 0.12f)
            {
                float sonarProx = 1f - (sensors.USNormalizedDistance / 0.12f);
                AddReward(-0.03f * sonarProx);
            }
            if (sensors.LeftIRObstacle > 0.5f || sensors.RightIRObstacle > 0.5f)
            {
                AddReward(-0.01f);
            }
        }

        // === REWARD #5: ACTION RATE PENALTY (квадратичный) ===
        if (IsTraining)
        {
            float actionRate = Mathf.Pow(gas - prevGas, 2)
                             + Mathf.Pow(steering - prevSteering, 2)
                             + Mathf.Pow(cameraYawInput - prevCameraYaw, 2);
            AddReward(-0.05f * actionRate);

            // === REWARD #6: MILD REVERSE PENALTY (с исключениями) ===
            if (gas < -0.1f && !isRetrying)
            {
                bool nearWall = sensors != null && sensors.USNormalizedDistance < 0.12f;
                bool nearSideWall = sensors != null
                    && (sensors.LeftIRObstacle > 0.5f || sensors.RightIRObstacle > 0.5f);
                if (!nearWall && !nearSideWall) AddReward(-0.005f);
            }
        }
        prevGas = gas;
        prevSteering = steering;
        prevCameraYaw = cameraYawInput;

        // === ДИАГНОСТИКА ===
        if (IsTraining)
        {
            var stats = Academy.Instance.StatsRecorder;
            stats.Add("Custom/Gas", gas);
            stats.Add("Custom/IsReverse", gas < -0.1f ? 1f : 0f);
            stats.Add("Custom/BlindTicks", blindApproachTicks);
        }

        // === СЧИТЫВАЕМ ЗРЕНИЕ ===
        bool hasSeenBall = yoloCamera != null && yoloCamera.IsBallVisible;
        float currentAngle = hasSeenBall ? yoloCamera.RelativeAngle : 0f;
        float currentDist = hasSeenBall ? yoloCamera.NormalizedDistance : 0f;

        bool gripperSeesBall = sensors != null && sensors.GripperIRBallDetected > 0.5f;
        if (gripperSeesBall) isRetrying = false;

        // === ФАЗА RETRY ===
        if (isRetrying && hasSeenBall)
        {
            isRetrying = false;
            wasCloseToBall = false;
            retryBackupTicks = 0;
            blindApproachTicks = 0;
        }

        if (isRetrying)
        {
            retryBackupTicks++;
            if (retryBackupTicks >= RETRY_BACKUP_DURATION)
            {
                isRetrying = false;
                wasCloseToBall = false;
                retryBackupTicks = 0;
                blindApproachTicks = 0;
                lastDistance = 1f;
            }
            LogStepToDiagnostics(gas, steering);
            return;
        }

        if (hasSeenBall)
        {
            // Убираем спайк-награду при первом появлении мяча
            if (!wasSeeingBallLastStep) lastDistance = currentDist;

            blindApproachTicks = 0;
            isRetrying = false;

            // === REWARD #1: DISTANCE DELTA (proximity-scaled) ===
            // dist=0.8 -> 2.8x, dist=0.3 -> 4.8x, dist=0.1 -> 5.6x
            if (wasSeeingBallLastStep)
            {
                float distanceDelta = lastDistance - currentDist;
                if (Mathf.Abs(distanceDelta) < 0.5f) // фильтр спайков
                {
                    float proximityMultiplier = 2.0f + 4.0f * (1.0f - Mathf.Clamp01(currentDist));
                    AddReward(distanceDelta * proximityMultiplier);
                }
            }

            // === REWARD #7: PROXIMITY SLOW-DOWN BONUS ===
            if (currentDist < 0.3f && gas > 0.01f && gas < 0.3f) AddReward(0.005f);

            // === REWARD #8: SPEED PENALTY NEAR BALL ===
            if (currentDist < 0.25f && Mathf.Abs(gas) > 0.4f) AddReward(-0.01f);

            // === REWARD #9: ALIGNMENT BONUS ===
            if (currentDist < 0.4f && Mathf.Abs(currentAngle) < 0.15f) AddReward(0.005f);

            wasCloseToBall = currentDist <= 0.35f;

            lastDistance = currentDist;
            wasSeeingBallLastStep = true;
        }
        else
        {
            // === МЯЧ НЕ ВИДЕН ===
            if (wasCloseToBall && !gripperSeesBall)
            {
                // ФАЗА 3: мяч в слепой зоне (под роботом/в клешне)
                blindApproachTicks++;

                // Бонус за медленное движение вперёд (ползи к мячу, не отъезжай)
                if (gas > 0.01f && gas < 0.3f) AddReward(0.003f);

                if (blindApproachTicks >= BLIND_APPROACH_MAX)
                {
                    if (retryCount < MAX_RETRIES)
                    {
                        isRetrying = true;
                        retryBackupTicks = 0;
                        retryCount++;
                        blindApproachTicks = 0;
                    }
                    else
                    {
                        wasCloseToBall = false;
                        blindApproachTicks = 0;
                    }
                }
            }
            // Фаза поиска: без отдельных наград/штрафов

            lastDistance = 1f;
            wasSeeingBallLastStep = false;
        }

        LogStepToDiagnostics(gas, steering);
    }

    private void LogStepToDiagnostics(float gas, float steering)
    {
        if (diagLogger == null || sensors == null) return;

        bool bs = yoloCamera != null && yoloCamera.IsBallVisible;
        float ba = bs ? yoloCamera.RelativeAngle : 0f;
        float bd = bs ? yoloCamera.NormalizedDistance : 0f;

        diagLogger.LogStep(
            StepCount, bs, ba, bd,
            sensors.USNormalizedDistance,
            sensors.LeftIRObstacle > 0.5f ? 1 : 0,
            sensors.RightIRObstacle > 0.5f ? 1 : 0,
            sensors.GripperIRBallDetected > 0.5f ? 1 : 0,
            currentCameraYaw, gas, steering,
            gripperController != null && gripperController.IsHolding,
            holdTicks, isRetrying,
            transform.position.x - startPosition.x,
            transform.position.z - startPosition.z,
            transform.eulerAngles.y / 360f,
            rb != null ? rb.linearVelocity.magnitude : 0f
        );
    }

    private void LogEpisodeOutcome(string outcome)
    {
        if (diagLogger == null) return;
        float robotMass = rb != null ? rb.mass : 0f;
        float ballMassMultiplier = (targetBall != null && startBallMass > 0.0001f)
            ? targetBall.mass / startBallMass : 1f;
        float ballScaleMultiplier = targetBall != null ? targetBall.transform.localScale.x : 1f;
        diagLogger.LogEpisodeEnd(outcome, StepCount, robotMass,
            ballMassMultiplier, ballScaleMultiplier, currentActionLatency);
        episodeOutcomeLogged = true;
    }

    /// <summary>360-градусный спавн мяча с проверкой стен и пола + DR из config.</summary>
    private void ResetBall()
    {
        if (targetBall == null) return;

        var envParams = Academy.Instance.EnvironmentParameters;

        // Вернуть мяч из клешни
        if (gripperController != null && gripperController.IsHolding)
        {
            gripperController.GripperCloseCommand = false;
            gripperController.ReleaseBall();
        }

        targetBall.isKinematic = false;
        targetBall.linearVelocity = Vector3.zero;
        targetBall.angularVelocity = Vector3.zero;

        Vector3 randomPos = transform.position;
        bool validPos = false;
        int attempts = 0;

        while (!validPos && attempts < 30)
        {
            float maxDist = envParams.GetWithDefault("ball_max_distance", 1.5f);

            // 360 градусов: мяч в ЛЮБОМ направлении вокруг робота
            float spawnAngle = UnityEngine.Random.Range(0f, 360f);
            float spawnDist = UnityEngine.Random.Range(0.5f, maxDist);
            Vector3 direction = Quaternion.Euler(0f, spawnAngle, 0f) * Vector3.forward;
            randomPos = transform.position + direction * spawnDist;
            randomPos.y = transform.position.y + 0.2f;

            // Проверка: не в стене/препятствии
            Collider[] colliders = Physics.OverlapSphere(randomPos, 0.15f);
            bool hasObstacle = false;
            foreach (var c in colliders)
            {
                if (c.CompareTag("Wall") || c.CompareTag("Obstacle")) { hasObstacle = true; break; }
            }

            // Проверка: под мячом есть пол (не за границей арены)
            if (!hasObstacle)
            {
                bool hasFloor = Physics.Raycast(randomPos + Vector3.up * 0.5f, Vector3.down, 2f);
                if (!hasFloor) hasObstacle = true;
            }

            if (!hasObstacle) validPos = true;
            attempts++;
        }

        if (!validPos)
        {
            // Fallback: прямо перед роботом
            randomPos = transform.position + transform.forward * 0.7f;
            randomPos.y = transform.position.y + 0.2f;
        }

        targetBall.transform.position = randomPos;

        // DR мяча из config
        targetBall.mass = envParams.GetWithDefault("ball_mass", 0.1f)
            * UnityEngine.Random.Range(0.5f, 2.0f);

        float ballScale = envParams.GetWithDefault("ball_scale", 0.12f)
            * UnityEngine.Random.Range(0.8f, 1.2f);
        targetBall.transform.localScale = Vector3.one * ballScale;

        Collider ballCollider = targetBall.GetComponent<Collider>();
        if (ballCollider != null) ballCollider.enabled = true;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var ca = actionsOut.ContinuousActions;
        ca[0] = Input.GetAxis("Vertical");
        ca[1] = Input.GetAxis("Horizontal");
        ca[2] = 0f;
    }
}
