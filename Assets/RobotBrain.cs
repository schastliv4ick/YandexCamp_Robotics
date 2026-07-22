using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;
using RosMessageTypes.Geometry;

[RequireComponent(typeof(Rigidbody))]
public class RobotBrain : Agent
{
    [Header("Mode")]
    [Tooltip("true = управление реальным роботом через ROSBridge, false = симуляция")]
    [SerializeField] private bool useRealRobot = false;

    [Header("Real Robot References")]
    [SerializeField] private ROSBridge rosBridge;
    [SerializeField] private RealVision realVision;

    [Header("YOLO Camera Calibration")]
    [Tooltip("Разрешение по ширине входного кадра (пиксели)")]
    [SerializeField] private float imageWidthPixels = 640f;
    [Tooltip("Фокусное расстояние в пикселях. Подбирается калибровкой.")]
    [SerializeField] private float focalLengthPx = 500f;
    [Tooltip("Реальный диаметр мяча в метрах")]
    [SerializeField] private float realBallDiameterMeters = 0.065f;
    [Tooltip("Высота камеры над полом в метрах")]
    [SerializeField] private float cameraHeightMeters = 0.15f;
    [Tooltip("Максимальное расстояние для нормализации наблюдений")]
    [SerializeField] private float maxDetectionDistance = 3.0f;
    [Tooltip("Тиков подряд ИК клешни для подтверждения захвата")]
    [SerializeField] private int requiredGripperDetectionTicks = 3;

    [Header("Simulation References")]
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
    [Header("1. Distance Delta")]
    [SerializeField] private float distanceRewardFar = 0.05f;
    [SerializeField] private float distanceRewardNear = 0.15f;
    [SerializeField] private float nearDistanceThreshold = 0.5f;

    [Header("2. Centering")]
    [SerializeField] private float centeringRewardScale = 0.02f;

    [Header("3. Action Smoothness")]
    [SerializeField] private float actionRatePenalty = 0.01f;

    [Header("4. Obstacle Distance (US)")]
    [SerializeField] private float obstaclePenaltyScale = 0.05f;
    [SerializeField] private float obstacleSafeDistance = 0.07f;

    [Header("5. Wall Contact (IR)")]
    [SerializeField] private float irCollisionPenalty = 0.02f;

    [Header("6. Backward")]
    [SerializeField] private float backwardPenalty = 0.01f;
    [SerializeField] private float backwardGasThreshold = -0.1f;

    [Header("7-8. Terminal")]
    [SerializeField] private float successReward = 5.0f;
    [SerializeField] private float fallPenalty = 1.0f;

    [Header("9. Holding Tick")]
    [SerializeField] private float holdingBallReward = 0.2f;

    [Header("10. Ball Spawn (Curriculum)")]
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

    [Header("11. Obstacle Randomization")]
    [SerializeField] private Transform[] obstacles;
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
    private float prevZPosition;
    private float prevGas;
    private float prevSteer;
    private float lastKnownBallAngle;
    private float cameraPivotAngle;

    private Vector3 startBallScale;
    private float startBallMass;
    private Vector3 startBallLocalPosition;
    private Vector3[] originalObstaclePositions;

    private DiagnosticLogger diagLogger;

    private bool lastBallVisible;
    private float lastBallAngle;
    private float lastBallDist;

    private bool episodeOutcomeLogged = false;
    private bool isFirstEpisode = true;
    private bool rewardConfigLoaded = false;

    private bool realGripperHolding = false;
    private int realGripperDetectionTicks = 0;

    [Header("Debug (read only)")]
    [SerializeField] private float lastComputedDistance = -1f;

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

        if (obstacles != null && obstacles.Length > 0)
        {
            originalObstaclePositions = new Vector3[obstacles.Length];
            for (int i = 0; i < obstacles.Length; i++)
            {
                if (obstacles[i] != null)
                    originalObstaclePositions[i] = obstacles[i].localPosition;
            }
        }

        if (!useRealRobot)
        {
            if (targetBall == null) Debug.LogWarning("[RobotBrain] targetBall not assigned!");
            if (virtualSensors == null) Debug.LogWarning("[RobotBrain] virtualSensors not assigned!");
            if (yoloCamera == null) Debug.LogWarning("[RobotBrain] yoloCamera not assigned!");
            if (gripperController == null) Debug.LogWarning("[RobotBrain] gripperController not assigned!");
        }
        else
        {
            if (rosBridge == null) Debug.LogWarning("[RobotBrain] rosBridge not assigned — real robot commands will not be sent!");
            if (realVision == null) Debug.LogWarning("[RobotBrain] realVision not assigned — YOLO data unavailable!");
        }
    }

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
                obstacles[i].localPosition = originalObstaclePositions[i] + new Vector3(randX, 0f, randZ);
            }
        }
    }

    public void ResetBall()
    {
        if (targetBall == null) return;
        Vector3 finalLocalPos = startBallLocalPosition;
        bool positionValid = false;
        int attempts = 0;
        int maxAttempts = 100;
        float checkRadius = (startBallScale.x * 0.5f) + ballClearanceRadius;

        while (!positionValid && attempts < maxAttempts)
        {
            attempts++;
            float randomX = UnityEngine.Random.Range(spawnMinX, spawnMaxX);
            float roll = UnityEngine.Random.value;
            float randomZ = startBallLocalPosition.z;
            if (roll < 0.15f)
                randomZ = UnityEngine.Random.Range(spawnMinZ_Easy, spawnMaxZ_Easy);
            else if (roll < 0.75f)
                randomZ = UnityEngine.Random.Range(spawnMinZ_Medium, spawnMaxZ_Medium);
            else
                randomZ = UnityEngine.Random.Range(spawnMinZ_Hard, spawnMaxZ_Hard);

            Vector3 proposedLocalPos = new Vector3(randomX, startBallLocalPosition.y, randomZ);
            Vector3 proposedWorldPos = targetBall.transform.parent.TransformPoint(proposedLocalPos);
            Collider[] colliders = Physics.OverlapSphere(proposedWorldPos, checkRadius, spawnCheckLayerMask);
            if (colliders.Length == 0)
            {
                finalLocalPos = proposedLocalPos;
                positionValid = true;
            }
        }

        if (!positionValid)
        {
            Debug.LogWarning($"[RobotBrain] Could not find safe ball spawn in {maxAttempts} attempts. Using default.");
            finalLocalPos = startBallLocalPosition;
        }

        targetBall.transform.localPosition = finalLocalPos;
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

        if (!useRealRobot)
        {
            RandomizeObstacles();
            if (gripperController != null && gripperController.IsHolding)
            {
                gripperController.GripperCloseCommand = false;
                gripperController.ReleaseBall();
            }
            ResetBall();

            if (Academy.Instance.IsCommunicatorOn)
            {
                if (rb != null) rb.mass = UnityEngine.Random.Range(2.2f, 2.8f);
                if (trackController != null)
                {
                    trackController.moveSpeed = UnityEngine.Random.Range(0.4f, 0.6f);
                    trackController.turnSpeed = UnityEngine.Random.Range(108f, 132f);
                }
            }

            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;

            float randomAngle = UnityEngine.Random.Range(-180f, 180f);
            Quaternion randomRotation = Quaternion.Euler(0f, startRotation.eulerAngles.y + randomAngle, 0f);
            transform.SetPositionAndRotation(startPosition, randomRotation);
        }
        else
        {
            if (rosBridge != null)
            {
                rosBridge.PublishDrive(0f, 0f);
                rosBridge.PublishGripper(1);
            }
            realGripperHolding = false;
            realGripperDetectionTicks = 0;
        }

        prevGas = 0f;
        prevSteer = 0f;
        lastKnownBallAngle = 0f;
        lastDetectionTime = Time.time;
        cameraPivotAngle = 0f;

        float initDist = GetCurrentDistanceToBall();
        prevDistanceToBall = initDist >= 0f ? initDist : 999f;
        prevZPosition = transform.position.z;
        holdTicks = 0;

        currentActionLatency = Academy.Instance.IsCommunicatorOn ? UnityEngine.Random.Range(8, 13) : 0;
        actionBuffer.Clear();
        for (int i = 0; i < currentActionLatency; i++)
            actionBuffer.Enqueue(new float[] { 0f, 0f, 0f });
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

    // ==================== SENSOR GETTERS ====================

    private float GetUSNormalizedDistance()
    {
        if (useRealRobot && rosBridge != null)
        {
            var data = rosBridge.GetSensorData();
            if (data != null) return Mathf.Clamp01((float)data.x);
        }
        return virtualSensors != null ? virtualSensors.USNormalizedDistance : 1f;
    }

    private float GetLeftIRObstacle()
    {
        if (useRealRobot && rosBridge != null)
        {
            var data = rosBridge.GetSensorData();
            if (data != null) return (float)data.y;
        }
        return virtualSensors != null ? virtualSensors.LeftIRObstacle : 0f;
    }

    private float GetRightIRObstacle()
    {
        if (useRealRobot && rosBridge != null)
        {
            var data = rosBridge.GetSensorData();
            if (data != null) return (float)data.z;
        }
        return virtualSensors != null ? virtualSensors.RightIRObstacle : 0f;
    }

    private float GetGripperIRBallDetected()
    {
        if (useRealRobot && rosBridge != null)
        {
            var data = rosBridge.GetSensorData();
            if (data != null) return (float)data.w;
        }
        return virtualSensors != null ? virtualSensors.GripperIRBallDetected : 0f;
    }

    // ==================== YOLO GETTERS ====================

    private bool IsBallVisible()
    {
        if (useRealRobot && realVision != null)
            return realVision.useYOLO && realVision.seesBall;
        return yoloCamera != null && yoloCamera.IsBallVisible;
    }

    private float GetBallRelativeAngle()
    {
        if (useRealRobot && realVision != null)
            return realVision.normalizedAngle;
        return yoloCamera != null ? yoloCamera.RelativeAngle : 0f;
    }

    public float ComputeDistanceToBall(YoloDataPacket data)
    {
        if (data == null || data.sees < 0.5f || data.w < 0.001f)
            return -1f;

        float bboxWidthPx = data.w * imageWidthPixels;
        if (bboxWidthPx < 1f) return -1f;

        float distCam = (realBallDiameterMeters * focalLengthPx) / bboxWidthPx;
        float sq = distCam * distCam - cameraHeightMeters * cameraHeightMeters;
        if (sq < 0f) return -1f;

        return Mathf.Sqrt(sq);
    }

    private float GetComputedDistanceToBall()
    {
        if (realVision == null) return -1f;
        var packet = new YoloDataPacket
        {
            angle = realVision.normalizedAngle,
            distance = realVision.normalizedDistance,
            sees = realVision.seesBall ? 1f : 0f,
            conf = realVision.confidence,
            w = realVision.ballWidth,
            h = realVision.ballHeight,
            cy = realVision.normalizedCenterY
        };
        float dist = ComputeDistanceToBall(packet);
        lastComputedDistance = dist;
        return dist;
    }

    private float GetBallNormalizedDistance()
    {
        if (useRealRobot && realVision != null)
        {
            float dist = GetComputedDistanceToBall();
            if (dist < 0f) return 1f;
            return Mathf.Clamp01(dist / maxDetectionDistance);
        }
        return yoloCamera != null ? yoloCamera.NormalizedDistance : 1f;
    }

    private float GetCurrentDistanceToBall()
    {
        if (useRealRobot)
        {
            float dist = GetComputedDistanceToBall();
            if (dist >= 0f) return dist;
            return prevDistanceToBall;
        }
        if (targetBall != null)
            return Vector3.Distance(transform.position, targetBall.position);
        return 999f;
    }

    private bool GetIsHolding()
    {
        if (useRealRobot)
            return realGripperHolding;
        return gripperController != null && gripperController.IsHolding;
    }

    // ==================== REWARDS ====================

    private void CalculateRewards(float gas, float steer)
    {
        float rewardDist = 0f;
        float rewardCentering = 0f;
        float penaltyAction = 0f;
        float penaltyObstacle = 0f;
        float penaltyCollision = 0f;
        float penaltyBackward = 0f;
        float penaltyTime = -0.0005f;
        float rewardSearch = 0f;

        if (!useRealRobot && transform.position.y < fallHeightThreshold)
        {
            AddReward(-fallPenalty);
            LogEpisodeOutcome("fall");
            EndEpisode();
            return;
        }

        bool hasBallReference = targetBall != null || useRealRobot;
        if (hasBallReference)
        {
            float currentDistance = GetCurrentDistanceToBall();
            float delta = prevDistanceToBall - currentDistance;
            bool canSeeBall = IsBallVisible();
            bool isCloseBlindZone = currentDistance < 0.6f;

            if (canSeeBall || isCloseBlindZone)
            {
                float rewardScale = currentDistance < nearDistanceThreshold ? distanceRewardNear : distanceRewardFar;
                rewardDist = delta * rewardScale;
                AddReward(rewardDist);
            }

            if (!canSeeBall && !isCloseBlindZone)
            {
                float deltaZ = transform.position.z - prevZPosition;
                if (deltaZ > 0f && deltaZ < 0.5f)
                {
                    rewardSearch = deltaZ * 0.5f;
                    AddReward(rewardSearch);
                }
            }

            prevZPosition = transform.position.z;
        }

        if (IsBallVisible())
        {
            float chassisAlignment = 1f - (Mathf.Abs(cameraPivotAngle) / cameraPivotMaxAngle);
            rewardCentering = centeringRewardScale * (1f - Mathf.Abs(GetBallRelativeAngle())) * chassisAlignment;
            AddReward(rewardCentering);
        }

        float actionMagnitude = Mathf.Abs(gas - prevGas) + Mathf.Abs(steer - prevSteer);
        penaltyAction = -actionRatePenalty * actionMagnitude;
        AddReward(penaltyAction);

        float us = GetUSNormalizedDistance();
        if (us < obstacleSafeDistance)
        {
            float danger = (obstacleSafeDistance - us) / obstacleSafeDistance;
            penaltyObstacle = -obstaclePenaltyScale * danger;
            AddReward(penaltyObstacle);
        }

        if (GetLeftIRObstacle() > 0.5f || GetRightIRObstacle() > 0.5f)
        {
            penaltyCollision = -irCollisionPenalty;
            AddReward(penaltyCollision);
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

    // ==================== OBSERVATIONS ====================

    public override void CollectObservations(VectorSensor sensor)
    {
        if (!useRealRobot)
        {
            if (virtualSensors == null || yoloCamera == null || gripperController == null) return;
        }
        else
        {
            if (rosBridge == null) return;
        }

        float noiseUS = Academy.Instance.IsCommunicatorOn ? UnityEngine.Random.Range(-0.05f, 0.05f) : 0f;
        float noisyDistance = Mathf.Clamp01(GetUSNormalizedDistance() + noiseUS);
        sensor.AddObservation(noisyDistance);

        sensor.AddObservation(GetLeftIRObstacle());
        sensor.AddObservation(GetRightIRObstacle());
        sensor.AddObservation(GetGripperIRBallDetected());

        if (!useRealRobot)
        {
            if (burstDropoutRemaining > 0) burstDropoutRemaining--;
            else if (Academy.Instance.IsCommunicatorOn && rb != null && rb.angularVelocity.magnitude > 0.5f)
            {
                if (UnityEngine.Random.value < 0.15f) burstDropoutRemaining = UnityEngine.Random.Range(5, 16);
            }
        }

        bool ballVisible = IsBallVisible() && !(burstDropoutRemaining > 0);
        if (ballVisible)
        {
            lastDetectionTime = Time.time;
            lastKnownBallAngle = GetBallRelativeAngle();
        }

        lastBallVisible = ballVisible;
        float ballAngleToChassis = cameraPivotAngle + (ballVisible ? GetBallRelativeAngle() * cameraPivotMaxAngle : 0f);
        lastBallDist = ballVisible ? GetBallNormalizedDistance() : 1f;
        lastBallAngle = Mathf.Clamp(ballAngleToChassis / cameraPivotMaxAngle, -1f, 1f);

        sensor.AddObservation(ballVisible ? GetBallRelativeAngle() : 0f);
        sensor.AddObservation(ballVisible ? GetBallNormalizedDistance() : 1f);
        sensor.AddObservation(lastKnownBallAngle);
        sensor.AddObservation(ballVisible ? 1.0f : 0.0f);
        sensor.AddObservation(cameraPivotMaxAngle > 0f ? cameraPivotAngle / cameraPivotMaxAngle : 0f);

        sensor.AddObservation(GetIsHolding() ? 1f : 0f);
        sensor.AddObservation(Mathf.DeltaAngle(startRotation.eulerAngles.y, transform.eulerAngles.y) / 180f);
        sensor.AddObservation(Mathf.Clamp01(rb.linearVelocity.magnitude / 0.5f));
        float timeSinceLastDetection = Time.time - lastDetectionTime;
        sensor.AddObservation(Mathf.Clamp(timeSinceLastDetection, 0f, 10f) / 10f);
    }

    // ==================== ACTIONS ====================

    public override void OnActionReceived(ActionBuffers actions)
    {
        bool holding = GetIsHolding();

        if (holding)
        {
            if (!useRealRobot)
            {
                if (trackController != null)
                {
                    trackController.GasInput = 0;
                    trackController.SteerInput = 0;
                }
            }
            else
            {
                if (rosBridge != null) rosBridge.PublishDrive(0f, 0f);
            }

            holdTicks++;
            AddReward(holdingBallReward);
            if (holdTicks >= 20)
            {
                AddReward(successReward);
                LogEpisodeOutcome("success");
                EndEpisode();
            }
            return;
        }
        else
        {
            holdTicks = 0;
        }

        float gas, steer, cameraSignal;

        if (Academy.Instance.IsCommunicatorOn && currentActionLatency > 0)
        {
            float[] newActions = new float[] {
                Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f),
                Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f),
                Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f)
            };
            actionBuffer.Enqueue(newActions);
            float[] delayed = actionBuffer.Dequeue();
            gas = delayed[0];
            steer = delayed[1];
            cameraSignal = delayed[2];
        }
        else
        {
            gas = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
            steer = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
            cameraSignal = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);
        }

        // Drive
        if (!useRealRobot)
        {
            if (trackController != null)
            {
                trackController.GasInput = gas;
                trackController.SteerInput = steer;
            }
        }
        else
        {
            if (rosBridge != null)
                rosBridge.PublishDrive(gas, steer);
        }

        // Camera pivot
        cameraPivotAngle = Mathf.Clamp(cameraPivotAngle + cameraSignal * cameraPivotSpeed * Time.fixedDeltaTime,
            -cameraPivotMaxAngle, cameraPivotMaxAngle);

        if (!useRealRobot && cameraPivot != null)
            cameraPivot.localRotation = Quaternion.Euler(15f, cameraPivotAngle, 0f);

        if (useRealRobot && rosBridge != null)
        {
            float normAngle = cameraPivotMaxAngle > 0f ? cameraPivotAngle / cameraPivotMaxAngle : 0f;
            rosBridge.PublishCamera(normAngle);
        }

        // Gripper (auto)
        if (!useRealRobot)
        {
            if (gripperController != null && virtualSensors != null)
            {
                if (virtualSensors.GripperIRBallDetected > 0.5f)
                    gripperController.GripperCloseCommand = true;
            }
        }
        else
        {
            if (GetGripperIRBallDetected() > 0.5f)
            {
                if (rosBridge != null) rosBridge.PublishGripper(2);
                realGripperDetectionTicks++;
                if (realGripperDetectionTicks >= requiredGripperDetectionTicks)
                    realGripperHolding = true;
            }
            else
            {
                realGripperDetectionTicks = 0;
            }
        }

        CalculateRewards(gas, steer);

        prevGas = gas;
        prevSteer = steer;
        prevZPosition = transform.position.z;

        // Diagnostics
        if (diagLogger != null)
        {
            bool isRetryingHeuristic = gas < backwardGasThreshold
                && lastBallDist < nearDistanceThreshold
                && (Time.time - lastDetectionTime) < 1f;

            diagLogger.LogStep(
                StepCount,
                lastBallVisible, lastBallAngle, lastBallDist,
                GetUSNormalizedDistance(),
                GetLeftIRObstacle() > 0.5f ? 1 : 0,
                GetRightIRObstacle() > 0.5f ? 1 : 0,
                GetGripperIRBallDetected() > 0.5f ? 1 : 0,
                cameraPivotAngle, gas, steer,
                holding, holdTicks, isRetryingHeuristic,
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
