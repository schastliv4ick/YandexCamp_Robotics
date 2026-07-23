using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;

[RequireComponent(typeof(Rigidbody))]
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

    [Header("Settings")]
    [SerializeField] private float fallHeightThreshold = -1f;
    [SerializeField] private float cameraPivotMaxAngle = 45f;
    [SerializeField] private float cameraPivotSpeed = 60f;

    [Header("Rewards")]
    [Header("1. Сближение с мячом (Distance Delta)")]
    [SerializeField] private float distanceRewardFar = 0.05f;   // награда за 1 метр сближения, если мяч далеко
    [SerializeField] private float distanceRewardNear = 0.15f;  // награда за 1 метр сближения, если мяч близко
    [SerializeField] private float nearDistanceThreshold = 0.5f; // порог "близко" в метрах

    [Header("2. Центрирование мяча перед захватом")]
    [SerializeField] private float centeringRewardScale = 0.02f;

    [Header("3. Плавность управления")]
    [SerializeField] private float actionRatePenalty = 0.01f;

    [Header("4. Дистанция до стен (УЗ, континуальный штраф)")]
    [SerializeField] private float obstaclePenaltyScale = 0.05f;
    [SerializeField] private float obstacleSafeDistance = 0.07f;

    [Header("5. Реальный контакт со стеной (ИК, ближний диапазон)")]
    [SerializeField] private float irCollisionPenalty = 0.02f;

    [Header("6. Задний ход")]
    [SerializeField] private float backwardPenalty = 0.01f;
    [SerializeField] private float backwardGasThreshold = -0.1f;

    [Header("7-8. Терминальные условия")]
    [SerializeField] private float successReward = 30.0f;
    [SerializeField] private float fallPenalty = 1.0f;

    [Header("9. Награда за тик удержания мяча")]
    [SerializeField] private float holdingBallReward = 0.2f;

    [Header("10. Настройки спавна мяча (Curriculum Z-Zones)")]
    [SerializeField] private float spawnMinX = -2.2f;
    [SerializeField] private float spawnMaxX = 2.2f;
    [SerializeField] private float spawnMinZ_Easy = -0.3f;
    [SerializeField] private float spawnMaxZ_Easy = 0.5f;
    [SerializeField] private float spawnMinZ_Medium = 0.5f;
    [SerializeField] private float spawnMaxZ_Medium = 1.8f;
    [SerializeField] private float spawnMinZ_Hard = 1.8f;
    [SerializeField] private float spawnMaxZ_Hard = 2.5f;
    [SerializeField] private LayerMask spawnCheckLayerMask; 
    [SerializeField] private float ballClearanceRadius = 0.15f;

    [Header("11. Настройки рандомизации препятствий")]
    [Tooltip("Перетащите сюда ваши кубы-препятствия из Арены")]
    [SerializeField] private Transform[] obstacles;
    [Tooltip("Максимальный радиус случайного сдвига препятствия (м)")]
    [SerializeField] private float obstacleOffsetRange = 0.3f;

    private Queue<float[]> actionBuffer = new Queue<float[]>();
    private int currentActionLatency = 5;
    private int holdTicks = 0;
    private int burstDropoutRemaining = 0;
    private float lastDetectionTime = 0;

    private Rigidbody rb;
    private Vector3 startPosition;
    private Quaternion startRotation;
    private float prevDistanceToBall;
    private float prevZPosition; // для награды за продвижение по мировой оси +Z (полоса препятствий)
    private float prevGas;
    private float prevSteer;
    private float lastKnownBallAngle;
    private float cameraPivotAngle;

    private Vector3 startBallScale;
    private float startBallMass;
    private Vector3 startBallLocalPosition;
    private Vector3[] originalObstaclePositions;

    private DiagnosticLogger diagLogger; // это всегда null, если не включен в сцене

    private bool lastBallVisible;
    private float lastBallAngle;
    private float lastBallDist;

    private bool episodeOutcomeLogged = false;
    private bool isFirstEpisode = true;
    private bool rewardConfigLoaded = false;

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        diagLogger = GetComponent<DiagnosticLogger>();
        startPosition = transform.position;
        startRotation = transform.rotation;

        LoadRewardConfigFromEnvironmentParameters();

        if (targetBall != null)
        {
            startBallScale = targetBall.transform.localScale;
            startBallMass = targetBall.mass;
            startBallLocalPosition = targetBall.transform.localPosition;
        }

        // Запоминаем исходные координаты препятствий для корректного сдвига без накопления дрейфа
        if (obstacles != null && obstacles.Length > 0)
        {
            originalObstaclePositions = new Vector3[obstacles.Length];
            for (int i = 0; i < obstacles.Length; i++)
            {
                if (obstacles[i] != null)
                {
                    originalObstaclePositions[i] = obstacles[i].localPosition;
                }
            }
        }
    }

    // считываем веса из конфига
    // GetWithDefault возвращает переданное значение, иначе возвращает дефолтное из кода
    private void LoadRewardConfigFromEnvironmentParameters()
    {
        var p = Academy.Instance.EnvironmentParameters;

        distanceRewardFar = p.GetWithDefault("distance_reward_far", distanceRewardFar);
        distanceRewardNear = p.GetWithDefault("distance_reward_near", distanceRewardNear);
        nearDistanceThreshold = p.GetWithDefault("near_distance_threshold", nearDistanceThreshold);
        centeringRewardScale = p.GetWithDefault("centering_reward_scale", centeringRewardScale);
        actionRatePenalty = p.GetWithDefault("action_rate_penalty", actionRatePenalty);
        obstaclePenaltyScale = p.GetWithDefault("obstacle_penalty_scale", obstaclePenaltyScale);
        obstacleSafeDistance = p.GetWithDefault("obstacle_safe_distance", obstacleSafeDistance);
        irCollisionPenalty = p.GetWithDefault("ir_collision_penalty", irCollisionPenalty);
        backwardPenalty = p.GetWithDefault("backward_penalty", backwardPenalty);
        backwardGasThreshold = p.GetWithDefault("backward_gas_threshold", backwardGasThreshold);
        successReward = p.GetWithDefault("success_reward", successReward);
        fallPenalty = p.GetWithDefault("fall_penalty", fallPenalty);

        Debug.Log($"[RobotBrain] Reward config: success={successReward:F2} fall={fallPenalty:F2} " +
            $"distNear={distanceRewardNear:F3} distFar={distanceRewardFar:F3} " +
            $"obstaclePenalty={obstaclePenaltyScale:F3} actionRate={actionRatePenalty:F3}");
    }

    private void RandomizeObstacles()
    {
        if (obstacles == null || originalObstaclePositions == null) return;
        for (int i = 0; i < obstacles.Length; i++)
        {
            if (obstacles[i] != null)
            {
                float randX = UnityEngine.Random.Range(-obstacleOffsetRange, obstacleOffsetRange);
                float randZ = UnityEngine.Random.Range(-obstacleOffsetRange, obstacleOffsetRange);
                Vector3 offset = new Vector3(randX, 0f, randZ);
                obstacles[i].localPosition = originalObstaclePositions[i] + offset;
            }
        }
    }

    public void ResetBall()
    {
        if (targetBall == null) return;

        // --- КОСТЫЛЬ: Жестко ставим мяч в конец арены (в зону финиша) ---
        // X = 0 (по центру), Z = 2.5 (далеко впереди). Высота остается стартовой.
        Vector3 finishLinePos = new Vector3(startBallLocalPosition.x, startBallLocalPosition.y, (spawnMinZ_Hard + spawnMaxZ_Hard) / 2);
        targetBall.transform.localPosition = finishLinePos;

        targetBall.mass = startBallMass * (1.0f + UnityEngine.Random.Range(0.0f, 1.0f));
        targetBall.transform.localScale = startBallScale * (1.0f + UnityEngine.Random.Range(-0.2f, 0.2f));
        targetBall.linearVelocity = Vector3.zero;
        targetBall.angularVelocity = Vector3.zero;
    }

    public override void OnEpisodeBegin()
    {
        if (!isFirstEpisode && !episodeOutcomeLogged)
            LogEpisodeOutcome("timeout");
        isFirstEpisode = false;
        episodeOutcomeLogged = false;

        if (!rewardConfigLoaded && Academy.Instance.IsCommunicatorOn)
        {
            LoadRewardConfigFromEnvironmentParameters();
            rewardConfigLoaded = true;
        }

        RandomizeObstacles();

        if (gripperController != null && gripperController.IsHolding)
        {
            gripperController.GripperCloseCommand = false;
            gripperController.ReleaseBall();
        }

        ResetBall();

        if (Academy.Instance.IsCommunicatorOn)
        {
            if (rb != null)
            {
                rb.mass = UnityEngine.Random.Range(2.2f, 2.8f);
            }

            if (trackController != null)
            {
                // Меняем динамические характеристики двигателей
                trackController.moveSpeed = UnityEngine.Random.Range(0.4f, 0.6f);
                trackController.turnSpeed = UnityEngine.Random.Range(108f, 132f); // скорость вращения
                // TODO Будем реализовывать сглаживание внутри trackcontroller?
                // trackController.smoothing = UnityEngine.Random.Range(0.01f, 0.25f); // инерция привода
            }
        }

        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        float randomAngle = UnityEngine.Random.Range(-15, 15f);
        Quaternion randomRotation = Quaternion.Euler(0f, startRotation.eulerAngles.y + randomAngle, 0f);
        transform.SetPositionAndRotation(startPosition, randomRotation);
        prevGas = 0f;
        prevSteer = 0f;
        lastKnownBallAngle = 0f;
        lastDetectionTime = Time.time;
        cameraPivotAngle = 0f;

        if (targetBall != null)
            prevDistanceToBall = Vector3.Distance(transform.position, targetBall.position);

        prevZPosition = transform.localPosition.z; 

        holdTicks = 0;

        currentActionLatency = Academy.Instance.IsCommunicatorOn ? UnityEngine.Random.Range(8, 13) : 0;
        actionBuffer.Clear();
        for (int i = 0; i < currentActionLatency; i++)
        {
            actionBuffer.Enqueue(new float[] { 0f, 0f, 0f });
        }
    }

    private void LogEpisodeOutcome(string outcome)
    {
        if (diagLogger == null) return;

        float robotMass = rb != null ? rb.mass : 0f;
        float ballMassMultiplier = (targetBall != null && startBallMass > 0.0001f) ? targetBall.mass / startBallMass : 1f;
        float ballScaleMultiplier = (targetBall != null && startBallScale.x > 0.0001f) ? targetBall.transform.localScale.x / startBallScale.x : 1f;

        diagLogger.LogEpisodeEnd(outcome, StepCount, robotMass, ballMassMultiplier, ballScaleMultiplier, currentActionLatency);
        episodeOutcomeLogged = true;
    }

    private void CalculateRewards(float gas, float steer)
    {
        // Локальные переменные для фиксации значений на этом шаге
         if (targetBall == null || yoloCamera == null || virtualSensors == null) return;
         
        float rewardDist = 0f;
        float rewardCentering = 0f;
        float penaltyAction = 0f;
        float penaltyObstacle = 0f;
        float penaltyCollision = 0f;
        float penaltyBackward = 0f;
        float penaltySpeedNearBall = 0f;
        float penaltyTime = -0.0005f;
        float rewardSearch = 0f;

        if (transform.position.y < fallHeightThreshold)
        {
            AddReward(-fallPenalty);
            LogEpisodeOutcome("fall");
            EndEpisode();
            return;
        }
        
        bool spottedBallWithYolo = yoloCamera != null && yoloCamera.IsBallVisible;
        bool reachedFinishLine = targetBall != null && transform.localPosition.z > spawnMinZ_Hard;

        if (reachedFinishLine || spottedBallWithYolo)
        {
            AddReward(successReward);
            LogEpisodeOutcome("success");
            EndEpisode();
            return;
        }

        // --- ГЛАВНЫЙ ДВИГАТЕЛЬ МОДЕЛИ 1: Награда за продвижение сквозь лабиринт ---
        if (targetBall != null)
        {
            float deltaZ = transform.localPosition.z - prevZPosition;
            
            // Награждаем строго за честно пройденные метры вперед!
            if (deltaZ > 0f && deltaZ < 0.5f) 
            {
                rewardSearch = deltaZ * 0.5f; 
                AddReward(rewardSearch);
            }

            prevZPosition = transform.localPosition.z;
        }

        // --- ШТРАФЫ ЗА ДИНАМИКУ И СТЕНЫ ---
        float actionMagnitude = Mathf.Abs(gas - prevGas) + Mathf.Abs(steer - prevSteer);
        penaltyAction = -actionRatePenalty * actionMagnitude;
        AddReward(penaltyAction);

        if (virtualSensors != null)
        {
            float us = virtualSensors.USNormalizedDistance;
            if (us < obstacleSafeDistance)
            {
                float danger = (obstacleSafeDistance - us) / obstacleSafeDistance;
                penaltyObstacle = -obstaclePenaltyScale * danger;
                AddReward(penaltyObstacle);
            }

            if (virtualSensors.LeftIRObstacle > 0.5f || virtualSensors.RightIRObstacle > 0.5f)
            {
                penaltyCollision = -irCollisionPenalty;
                AddReward(penaltyCollision);
            }
        }

        if (gas < backwardGasThreshold)
        {
            penaltyBackward = -backwardPenalty;
            AddReward(penaltyBackward);
        }

        AddReward(penaltyTime);

        var stats = Academy.Instance.StatsRecorder;
        stats.Add("ComponentRewards/1_Distance", rewardDist);
        stats.Add("ComponentRewards/2_Centering", rewardCentering);
        stats.Add("ComponentRewards/3_ActionRatePenalty", penaltyAction);
        stats.Add("ComponentRewards/4_ObstaclePenalty", penaltyObstacle);
        stats.Add("ComponentRewards/5_CollisionPenalty", penaltyCollision);
        stats.Add("ComponentRewards/6_BackwardPenalty", penaltyBackward);
        stats.Add("ComponentRewards/7_TimePenalty", penaltyTime);
        stats.Add("ComponentRewards/8_SearchBonus", rewardSearch);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if (virtualSensors == null || yoloCamera == null || gripperController == null) return;

        // 1. УЛЬТРАЗВУК С ШУМОМ
        // Подмешиваем случайную погрешность в пределах +-5% к нормализованной дистанции
        float noiseUS = Academy.Instance.IsCommunicatorOn ? UnityEngine.Random.Range(-0.05f, 0.05f) : 0f;
        float noisyDistance = Mathf.Clamp01(virtualSensors.USNormalizedDistance + noiseUS);
        sensor.AddObservation(noisyDistance); // 0 (УЗ дальномер с шумом)

        // Боковые ИК датчики оставляем бинарными (1 или 0)
        sensor.AddObservation(virtualSensors.LeftIRObstacle); // 1
        sensor.AddObservation(virtualSensors.RightIRObstacle); // 2
        sensor.AddObservation(virtualSensors.GripperIRBallDetected); // 3

        // 2. СИМУЛЯЦИЯ ПОТЕРЬ КАДРОВ YOLO (Burst Dropout)
        // Если счетчик активен — уменьшаем его на 1 за каждый физический тик
        if (burstDropoutRemaining > 0) burstDropoutRemaining--;
        // Если робот крутится на месте быстрее 0.5 рад/с — с шансом 15% активируем слепую зону на 5-15 шагов
        else if (Academy.Instance.IsCommunicatorOn && rb != null && rb.angularVelocity.magnitude > 0.5f)
        {
            if (UnityEngine.Random.value < 0.15f) burstDropoutRemaining = UnityEngine.Random.Range(5, 16);
        }

        // Переопределяем видимость мяча с учетом симулированного лага камеры
        bool ballVisible = yoloCamera.IsBallVisible && !(burstDropoutRemaining > 0);
        if (ballVisible)
        {
            lastDetectionTime = Time.time;
            lastKnownBallAngle = ballVisible ? yoloCamera.RelativeAngle : 0f;
        }

        lastBallVisible = ballVisible;
        float ballAngleToChassis = cameraPivotAngle + (ballVisible ? yoloCamera.RelativeAngle * cameraPivotMaxAngle : 0f);
        lastBallDist = ballVisible ? yoloCamera.NormalizedDistance : 1f;
        lastBallAngle = Mathf.Clamp(ballAngleToChassis / cameraPivotMaxAngle, -1f, 1f);

        // Отправляем данные камеры в нейросеть
        sensor.AddObservation(ballVisible ? yoloCamera.RelativeAngle : 0f); // 4 (угол до мяча)
        sensor.AddObservation(ballVisible ? yoloCamera.NormalizedDistance : 1f); // 5 (дистанция до мяча)
        sensor.AddObservation(lastKnownBallAngle); // 6
        sensor.AddObservation(ballVisible ? 1.0f : 0.0f); // 7
        sensor.AddObservation(cameraPivotMaxAngle > 0f ? cameraPivotAngle / cameraPivotMaxAngle : 0f); // 8

        // Оставшиеся наблюдения (состояние клешни, смещение, одометрия) отправляем без изменений
        sensor.AddObservation(gripperController.IsHolding ? 1f : 0f); // 9
        sensor.AddObservation(Mathf.DeltaAngle(startRotation.eulerAngles.y, transform.eulerAngles.y) / 180f); // 12
        sensor.AddObservation(Mathf.Clamp01(rb.linearVelocity.magnitude / 0.5f)); // 13
        float timeSinceLastDetection = Time.time - lastDetectionTime;
        sensor.AddObservation(Mathf.Clamp(timeSinceLastDetection, 0f, 10f) / 10f); // 14
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        if(gripperController == null || virtualSensors == null || trackController == null || cameraPivot == null) return;
        
        // Автозахват оставляем для совместимости, но он больше не триггерит конец эпизода
        if (virtualSensors.GripperIRBallDetected > 0.5f)
        {
            gripperController.GripperCloseCommand = true;
        }

        float gas, steer, cameraSignal;

        if (Academy.Instance.IsCommunicatorOn && currentActionLatency > 0)
        {
            // Кладем свежее действие нейросети в конец очереди
            float[] newActions = new float[] {
                Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f),
                Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f),
                Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f)
            };
            actionBuffer.Enqueue(newActions);

            // Достаем устаревшее на N шагов действие из начала очереди
            float[] delayed = actionBuffer.Dequeue();
            gas = delayed[0];
            steer = delayed[1];
            cameraSignal = delayed[2];
        }
        else
        {
            // Без задержки (для ручного управления или инференса на реальном роботе)
            gas = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
            steer = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
            cameraSignal = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);
        }

        trackController.GasInput = gas;
        trackController.SteerInput = steer;

        cameraPivotAngle = Mathf.Clamp(cameraPivotAngle + cameraSignal * cameraPivotSpeed * Time.fixedDeltaTime,
            -cameraPivotMaxAngle, cameraPivotMaxAngle);
        cameraPivot.localRotation = Quaternion.Euler(15f, cameraPivotAngle, 0f);

        CalculateRewards(gas, steer);

        prevGas = gas;
        prevSteer = steer;

        if (diagLogger != null)
        {
            // Эвристика IsRetrying оставлена чисто для логера
            bool isRetryingHeuristic = gas < backwardGasThreshold
                && lastBallDist < nearDistanceThreshold
                && (Time.time - lastDetectionTime) < 1f;

            diagLogger.LogStep(
                StepCount,
                lastBallVisible, lastBallAngle, lastBallDist,
                virtualSensors.USNormalizedDistance,
                virtualSensors.LeftIRObstacle > 0.5f ? 1 : 0,
                virtualSensors.RightIRObstacle > 0.5f ? 1 : 0,
                virtualSensors.GripperIRBallDetected > 0.5f ? 1 : 0,
                cameraPivotAngle, gas, steer,
                gripperController.IsHolding, holdTicks, isRetryingHeuristic,
                transform.position.x - startPosition.x, transform.position.z - startPosition.z,
                transform.eulerAngles.y / 360f,
                rb.linearVelocity.magnitude
            );
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var ca = actionsOut.ContinuousActions;
        ca[0] = Input.GetAxis("Vertical");
        ca[1] = Input.GetAxis("Horizontal");
        ca[2] = 0f;

        var da = actionsOut.DiscreteActions;
        da[0] = Input.GetKey(KeyCode.Space) ? 1 : (Input.GetKey(KeyCode.LeftShift) ? 2 : 0);
    }
}