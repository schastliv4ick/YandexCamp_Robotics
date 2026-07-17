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
    [SerializeField] private float obstacleSafeDistance = 0.3f;

    [Header("5. Реальный контакт со стеной (ИК, ближний диапазон)")]
    [SerializeField] private float irCollisionPenalty = 0.02f;

    [Header("6. Задний ход")]
    [SerializeField] private float backwardPenalty = 0.01f;
    [SerializeField] private float backwardGasThreshold = -0.1f;

    [Header("7-8. Терминальные условия")]
    [SerializeField] private float successReward = 5.0f;
    [SerializeField] private float fallPenalty = 1.0f;

    private Queue<float[]> actionBuffer = new Queue<float[]>();
    private int currentActionLatency = 5;
    private int holdTicks = 0;

    private int burstDropoutRemaining = 0;
    private float lastDetectionTime = 0;

    private Rigidbody rb;
    private Vector3 startPosition;
    private Quaternion startRotation;

    private float prevDistanceToBall;
    private float prevGas;
    private float prevSteer;
    private float lastKnownBallAngle;
    private float cameraPivotAngle;

    private Vector3 startBallScale;
    private float startBallMass;
    private Vector3 startBallLocalPosition; 

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
        if (targetBall == null) Debug.LogWarning("[RobotBrain] targetBall не назначен — награда за сближение всегда будет 0!");
        if (virtualSensors == null) Debug.LogWarning("[RobotBrain] virtualSensors не назначен — штрафы за препятствия работать не будут!");
        if (yoloCamera == null) Debug.LogWarning("[RobotBrain] yoloCamera не назначен — награда за центрирование всегда будет 0!");
        if (gripperController == null) Debug.LogWarning("[RobotBrain] gripperController не назначен — терминальная награда за захват не сработает!");
    
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
        backwardGasThreshold  = p.GetWithDefault("backward_gas_threshold", backwardGasThreshold);
        successReward = p.GetWithDefault("success_reward", successReward);
        fallPenalty = p.GetWithDefault("fall_penalty", fallPenalty);

        Debug.Log($"[RobotBrain] Reward config: success={successReward:F2} fall={fallPenalty:F2} " +
                  $"distNear={distanceRewardNear:F3} distFar={distanceRewardFar:F3} " +
                  $"obstaclePenalty={obstaclePenaltyScale:F3} actionRate={actionRatePenalty:F3}");
    }

    public void ResetBall()
    {
        if (targetBall == null) return;
        
        float randomX = UnityEngine.Random.Range(-0.5f, 0.5f);
        float randomZ = UnityEngine.Random.Range(-0.5f, 0.5f);
        Vector3 randomOffset = new Vector3(randomX, 0f, randomZ);

        targetBall.transform.localPosition = startBallLocalPosition + randomOffset;
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

        ResetBall();
        if (Academy.Instance.IsCommunicatorOn)
        {
            if (rb != null)
            {
                // Рандомизируем массу робота от 1.0кг до 4.0кг (базовый вес 2.5кг)
                rb.mass = UnityEngine.Random.Range(1.0f, 4.0f);
            }
            if (trackController != null)
            {
                // Меняем динамические характеристики двигателей
                trackController.moveSpeed = UnityEngine.Random.Range(0.3f, 0.7f);   // базовый м/с +-40%
                trackController.turnSpeed = UnityEngine.Random.Range(80f, 160f);    // скорость вращения
                // TODO Будем реализовывать сглаживание внутри trackcontroller?
                // trackController.smoothing = UnityEngine.Random.Range(0.01f, 0.25f); // инерция привода
            }
        }

        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        float randomAngle = UnityEngine.Random.Range(-180f, 180f);
        Quaternion randomRotation = Quaternion.Euler(0f, startRotation.eulerAngles.y + randomAngle, 0f);
        transform.SetPositionAndRotation(startPosition, randomRotation);

        if (gripperController != null && gripperController.IsHolding)
        {
            gripperController.GripperCloseCommand = false;
            gripperController.ReleaseBall();
        }

        prevGas = 0f;
        prevSteer = 0f;
        lastKnownBallAngle = 0f;
        lastDetectionTime = Time.time;
        cameraPivotAngle = 0f;

        if (targetBall != null)
            prevDistanceToBall = Vector3.Distance(transform.position, targetBall.position);
                    
        holdTicks = 0;
        currentActionLatency = Academy.Instance.IsCommunicatorOn ? UnityEngine.Random.Range(1, 3) : 0;
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
        if (transform.position.y < fallHeightThreshold)
        {
            AddReward(-fallPenalty);
            LogEpisodeOutcome("fall");
            EndEpisode();
            return;
        }

        if (targetBall != null)
        {
            float currentDistance = Vector3.Distance(transform.position, targetBall.position);
            float delta = prevDistanceToBall - currentDistance; // > 0, если стали ближе

            float rewardScale = currentDistance < nearDistanceThreshold ? distanceRewardNear : distanceRewardFar;
            AddReward(delta * rewardScale);

            prevDistanceToBall = currentDistance;
        }

        if (yoloCamera != null && yoloCamera.IsBallVisible)
        {
            AddReward(centeringRewardScale * (1f - Mathf.Abs(yoloCamera.RelativeAngle))* (1f - yoloCamera.NormalizedDistance));
        }

        float actionMagnitude = Mathf.Abs(gas - prevGas) + Mathf.Abs(steer - prevSteer);
        AddReward(-actionRatePenalty * actionMagnitude);

        if (virtualSensors != null)
        {
            float us = virtualSensors.USNormalizedDistance; // 0 = вплотную, 1 = чисто
            if (us < obstacleSafeDistance)
            {
                float danger = (obstacleSafeDistance - us) / obstacleSafeDistance; // 0..1
                AddReward(-obstaclePenaltyScale * danger);
            }
        }

        if (virtualSensors != null && (virtualSensors.LeftIRObstacle > 0.5f || virtualSensors.RightIRObstacle > 0.5f))
        {
            AddReward(-irCollisionPenalty);
        }

        if (gas < backwardGasThreshold)
        {
            AddReward(-backwardPenalty);
        }
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
        sensor.AddObservation(virtualSensors.LeftIRObstacle);  // 1
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
            lastKnownBallAngle = yoloCamera.RelativeAngle;
        }
        
        lastBallVisible = ballVisible;
        lastBallAngle = ballVisible ? yoloCamera.RelativeAngle : 0f;
        lastBallDist = ballVisible ? yoloCamera.NormalizedDistance : 1f;

        // Отправляем данные камеры в нейросеть
        sensor.AddObservation(ballVisible ? yoloCamera.RelativeAngle : 0f);  // 4 (угол до мяча)
        sensor.AddObservation(ballVisible ? yoloCamera.NormalizedDistance : 1f); // 5 (дистанция до мяча)
        sensor.AddObservation(lastKnownBallAngle);                       // 6
        sensor.AddObservation(ballVisible ? 1.0f : 0.0f);                       // 7
        sensor.AddObservation(cameraPivotMaxAngle > 0f ? cameraPivotAngle / cameraPivotMaxAngle : 0f); // 8
        
        // Оставшиеся наблюдения (состояние клешни, смещение, одометрия) отправляем без изменений
        sensor.AddObservation(gripperController.IsHolding ? 1f : 0f); // 9
        
        Vector3 worldDisplacement = transform.position - startPosition;
        Vector3 localDisplacement = Quaternion.Inverse(startRotation) * worldDisplacement;
        // TODO верно что дальше 5и метров не уедем?
        const float maxOffset = 5f;
        sensor.AddObservation(Mathf.Clamp(localDisplacement.x / maxOffset, -1f, 1f));               // 10
        sensor.AddObservation(Mathf.Clamp(localDisplacement.z / maxOffset, -1f, 1f));               // 11
        sensor.AddObservation(transform.eulerAngles.y / 360f);                          // 12
        sensor.AddObservation(Mathf.Clamp01(rb.linearVelocity.magnitude / 0.5f));        // 13
        
        float timeSinceLastDetection = Time.time - lastDetectionTime;
        sensor.AddObservation(Mathf.Clamp(timeSinceLastDetection, 0f, 10f) / 10f);                                           // 14
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        if (gripperController != null && gripperController.IsHolding)
        {
            if(trackController != null)
            {
                trackController.GasInput = 0;
                trackController.SteerInput = 0;
            }
            holdTicks++;
            AddReward(0.02f);
            if (holdTicks >= 20)
            {
                AddReward(successReward);
                LogEpisodeOutcome("success");
                EndEpisode();
            }
            return;
        }
        else holdTicks = 0;

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

        if (trackController != null)
        {
            trackController.GasInput = gas;
            trackController.SteerInput = steer;
        }


        cameraPivotAngle = Mathf.Clamp(cameraPivotAngle + cameraSignal * cameraPivotSpeed * Time.fixedDeltaTime,
            -cameraPivotMaxAngle, cameraPivotMaxAngle);
        if (cameraPivot != null)
            cameraPivot.localRotation = Quaternion.Euler(0f, cameraPivotAngle, 0f);

        int gripperCommand = actions.DiscreteActions[0];
        if (gripperController != null)
        {
            if (gripperCommand == 1) gripperController.GripperCloseCommand = true;
            else if (gripperCommand == 2) gripperController.GripperCloseCommand = false;
        }

        CalculateRewards(gas, steer);
        prevGas = gas;
        prevSteer = steer;

        if (diagLogger != null && virtualSensors != null && gripperController != null)
        {
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