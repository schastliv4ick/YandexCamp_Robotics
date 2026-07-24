using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;

public enum RobotTaskState
{
    Navigation,
    ScriptedPickup,
    Grasping,
    HoldingConfirmation,
    Completed,
    // Appended (not inserted) so any previously-serialized state values keep their meaning.
    Idle
}

[RequireComponent(typeof(Rigidbody))]
public class RobotBrain : Agent
{
    [Header("Mode")]
    [Tooltip("true = управление реальным роботом через ROSBridge, false = симуляция")]
    [SerializeField] private bool useRealRobot = false;

    [Header("M2M Activation (Real Robot)")]
    [Tooltip("If true, the real tracked (gusenichny) rover stays idle with motors off until M2MListener " +
        "passes through a validated gfsx_robot activate/start command sent by the Yandex rover's search relay " +
        "(rover_dispatcher.py) after it finds the ball. Ignored in simulation.")]
    [SerializeField] private bool requireM2MActivation = true;

    [Header("Real Robot References")]
    [SerializeField] private ROSBridge rosBridge;
    [SerializeField] private RealVision realVision;

    [Header("YOLO Camera Calibration")]
    [Tooltip("Fallback frame width. The actual width is read from every UDP packet.")]
    [SerializeField, Min(1f)] private float imageWidthPixels = 640f;
    [Tooltip("Horizontal focal length in pixels. Calibrate it with a ball at a measured distance.")]
    [SerializeField, Min(1f)] private float focalLengthPx = 500f;
    [Tooltip("Real ball diameter in metres")]
    [SerializeField, Min(0.001f)] private float realBallDiameterMeters = 0.065f;
    [Tooltip("Camera optical-centre height above the floor in metres")]
    [SerializeField, Min(0f)] private float cameraHeightMeters = 0.15f;
    [Tooltip("Empirical multiplier applied after geometric distance calculation")]
    [SerializeField, Min(0.0001f)] private float distanceScale = 1f;
    [Tooltip("Empirical offset in metres applied after distanceScale")]
    [SerializeField] private float distanceOffsetMeters = 0f;
    [Tooltip("Maximum metric distance used to normalize the agent observation")]
    [SerializeField, Min(0.01f)] private float maxDetectionDistance = 3.0f;
    [Tooltip("Minimum YOLO confidence accepted by RobotBrain")]
    [SerializeField, Range(0f, 1f)] private float minimumVisionConfidence = 0.20f;
    [Tooltip("EMA weight of each new distance measurement: 1 = no smoothing")]
    [SerializeField, Range(0.01f, 1f)] private float distanceSmoothingFactor = 0.35f;
    [Tooltip("Reset distance smoothing after this many seconds without a valid measurement")]
    [SerializeField, Min(0.05f)] private float distanceFilterResetSeconds = 0.75f;
    [Tooltip("Measured horizontal ball distance used by the calibration context action")]
    [SerializeField, Min(0.01f)] private float calibrationKnownDistanceMeters = 1.0f;
    [Tooltip("Consecutive gripper IR ticks required to confirm a real grasp")]
    [SerializeField, Min(1)] private int requiredGripperDetectionTicks = 3;

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

    [Header("Task Flow: Navigation -> Scripted Pickup")]
    [Tooltip("The navigation model keeps control until the ball is visible continuously for this long.")]
    [SerializeField, Min(0.05f)] private float ballVisibleConfirmationSeconds = 1f;
    [Tooltip("Reward issued when control passes from the navigation model to the pickup script.")]
    [SerializeField] private float navigationHandoffReward = 1f;
    [Tooltip("Return control to navigation when the pickup script loses the ball for this long.")]
    [SerializeField, Min(0.1f)] private float pickupLostBallTimeoutSeconds = 1.5f;

    [Header("Scripted Pickup: Fine Centering")]
    [Tooltip("Desired horizontal ball position in the centered camera. Positive moves the target point to the right.")]
    [SerializeField, Range(-0.30f, 0.30f)] private float pickupAimOffsetNormalized = 0f;
    [Tooltip("Error smaller than this is treated as perfectly centered.")]
    [SerializeField, Range(0f, 0.20f)] private float pickupCenterDeadZone = 0.02f;
    [Tooltip("Error at which scripted steering reaches pickupMaximumSteer.")]
    [SerializeField, Range(0.03f, 1f)] private float pickupFullSteerError = 0.15f;
    [Tooltip("Minimum steering command outside the dead zone. Compensates motor/track static friction.")]
    [SerializeField, Range(0f, 1f)] private float pickupMinimumSteer = 0.07f;
    [Tooltip("Invert this if positive steering turns away from a ball on the right.")]
    [SerializeField] private float pickupSteerDirection = 1f;
    [SerializeField, Range(0f, 1f)] private float pickupMaximumSteer = 0.35f;
    [Tooltip("Low-pass time constant for the visual horizontal error. Lower reacts faster; higher rejects jitter.")]
    [SerializeField, Min(0f)] private float pickupErrorFilterTime = 0.06f;

    [Header("Scripted Pickup: Approach Gate")]
    [Tooltip("The ball must remain within this error before forward motion is enabled.")]
    [SerializeField, Range(0.005f, 0.30f)] private float pickupApproachEnterError = 0.045f;
    [Tooltip("Forward motion is disabled again when error grows beyond this value.")]
    [SerializeField, Range(0.01f, 0.50f)] private float pickupApproachExitError = 0.085f;
    [Tooltip("How long alignment must remain valid before the robot starts moving forward.")]
    [SerializeField, Min(0f)] private float pickupCenteredHoldSeconds = 0.25f;
    [Tooltip("Camera must be this close to its zero position before image-space approach control is used.")]
    [SerializeField, Min(0f)] private float pickupCameraReadyToleranceDegrees = 2f;
    [Tooltip("Camera zero must remain stable for this long before approach is enabled.")]
    [SerializeField, Min(0f)] private float pickupCameraReadyHoldSeconds = 0.15f;

    [Header("Scripted Pickup: Forward Motion")]
    [SerializeField, Range(0f, 1f)] private float pickupMaximumForward = 0.18f;
    [SerializeField, Range(0f, 1f)] private float pickupMinimumForward = 0.035f;
    [SerializeField, Min(0.02f)] private float pickupSlowDistanceMeters = 0.50f;
    [SerializeField, Min(0.01f)] private float pickupCloseDistanceMeters = 0.11f;
    [SerializeField] private bool allowDistanceTriggeredGrasp = false;
    [Tooltip("Acceleration/deceleration smoothing. Higher is softer but increases stopping distance.")]
    [SerializeField, Min(0.01f)] private float pickupGasSmoothTime = 0.30f;
    [Tooltip("Steering smoothing. Keep much lower than gas smoothing to correct lateral error quickly.")]
    [SerializeField, Min(0.01f)] private float pickupSteerSmoothTime = 0.08f;
    [SerializeField, Min(0f)] private float pickupCameraCenterSpeedDegrees = 30f;
    [Tooltip("Optional safety stop. Keep disabled if the ultrasonic sensor sees the ball itself.")]
    [SerializeField] private bool pickupUseUltrasonicEmergencyStop = false;
    [SerializeField, Range(0f, 1f)] private float pickupUltrasonicStopDistance = 0.03f;

    [Header("Scripted Pickup: Gripper and Completion")]
    [SerializeField, Range(0f, 1f)] private float gripperDetectionThreshold = 0.5f;
    [SerializeField, Min(0.1f)] private float graspTimeoutSeconds = 2f;
    [SerializeField, Min(0.1f)] private float holdingConfirmationSeconds = 3f;
    [SerializeField, Min(0f)] private float holdingLossGraceSeconds = 0.2f;
    [SerializeField] private int gripperOpenCommand = 1;
    [SerializeField] private int gripperCloseCommand = 2;
    [Tooltip("Usually false for the real robot. Enable only in a training scene that should reset after success.")]
    [SerializeField] private bool endEpisodeOnPickupComplete = false;

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
    private bool gripperCloseRequested = false;

    [Header("Task State (read only)")]
    [SerializeField] private RobotTaskState currentTaskState = RobotTaskState.Navigation;
    [SerializeField] private float ballVisibleDuration = 0f;
    [SerializeField] private float pickupBallLostDuration = 0f;
    [SerializeField] private float graspDuration = 0f;
    [SerializeField] private float holdingDuration = 0f;
    [SerializeField] private float holdingLostDuration = 0f;
    [SerializeField] private float pickupRawHorizontalError = 0f;
    [SerializeField] private float pickupFilteredHorizontalError = 0f;
    [SerializeField] private float pickupAlignedDuration = 0f;
    [SerializeField] private float pickupCameraReadyDuration = 0f;
    [SerializeField] private bool pickupCameraReady = false;
    [SerializeField] private bool pickupApproachEnabled = false;
    private bool pickupErrorFilterInitialized = false;
    private float scriptedGas = 0f;
    private float scriptedSteer = 0f;
    private float scriptedGasVelocity = 0f;
    private float scriptedSteerVelocity = 0f;
    private bool completionHandled = false;

    [Header("Debug (read only)")]
    [SerializeField] private float rawComputedDistance = -1f;
    [SerializeField] private float lastComputedDistance = -1f;
    private float filteredRealDistance = -1f;
    private float lastValidDistanceTime = float.NegativeInfinity;
    private int lastProcessedVisionSequence = -1;
    private double lastProcessedVisionTimestamp = double.NaN;

    public bool IsUsingRealRobot => useRealRobot;
    public ROSBridge RosBridgeSource => rosBridge;
    public RealVision RealVisionSource => realVision;
    public RobotTaskState CurrentTaskState => currentTaskState;
    public float DebugBallVisibleDuration => ballVisibleDuration;
    public float DebugBallVisibleConfirmationSeconds => ballVisibleConfirmationSeconds;
    public float DebugPickupBallLostDuration => pickupBallLostDuration;
    public float DebugGraspDuration => graspDuration;
    public float DebugHoldingDuration => holdingDuration;
    public float DebugHoldingConfirmationSeconds => holdingConfirmationSeconds;
    public float DebugPickupRawHorizontalError => pickupRawHorizontalError;
    public float DebugPickupFilteredHorizontalError => pickupFilteredHorizontalError;
    public float DebugPickupAimOffsetNormalized => pickupAimOffsetNormalized;
    public float DebugPickupAlignedDuration => pickupAlignedDuration;
    public float DebugPickupCenteredHoldSeconds => pickupCenteredHoldSeconds;
    public bool DebugPickupCameraReady => pickupCameraReady;
    public bool DebugPickupApproachEnabled => pickupApproachEnabled;
    public bool DebugBallVisible => IsBallVisible();
    public float DebugBallRelativeAngle => GetBallRelativeAngle();
    public float DebugBallBearingDegrees => GetBallRelativeBearingDegrees();
    public float DebugBallNormalizedDistance => GetBallNormalizedDistance();
    public float DebugBallDistanceMeters => GetCurrentDistanceToBall();
    public float DebugRawBallDistanceMeters => rawComputedDistance;
    public float DebugEffectiveCameraPivotAngle => GetEffectiveCameraPivotAngle();
    public float DebugCommandedCameraPivotAngle => cameraPivotAngle;
    public float DebugCameraHalfFovDegrees => GetCameraHalfFovDegrees();
    public float DebugFocalLengthPx => focalLengthPx;
    public float DebugUSNormalizedDistance => GetUSNormalizedDistance();
    public float DebugLeftIR => GetLeftIRObstacle();
    public float DebugRightIR => GetRightIRObstacle();
    public float DebugGripperIR => GetGripperIRBallDetected();
    public bool DebugIsHolding => GetIsHolding();
    public float DebugCurrentGas => prevGas;
    public float DebugCurrentSteer => prevSteer;
    public float DebugLastKnownBallAngle => lastKnownBallAngle;
    public float DebugTimeSinceLastDetection => Mathf.Max(0f, Time.time - lastDetectionTime);
    public bool DebugHasTrackController => trackController != null;
    public float DebugMoveSpeed => trackController != null ? trackController.moveSpeed : 0f;
    public float DebugTurnSpeed => trackController != null ? trackController.turnSpeed : 0f;
    public bool DebugHasGripperController => gripperController != null;
    public bool DebugGripperCloseCommand => gripperController != null && gripperController.GripperCloseCommand;
    public bool DebugHasTargetBall => targetBall != null;
    public float DebugTargetBallMass => targetBall != null ? targetBall.mass : 0f;
    public float DebugTargetBallScale => targetBall != null ? targetBall.transform.localScale.x : 0f;

    private void OnValidate()
    {
        pickupFullSteerError = Mathf.Max(pickupCenterDeadZone + 0.005f, pickupFullSteerError);
        pickupApproachEnterError = Mathf.Max(pickupCenterDeadZone, pickupApproachEnterError);
        pickupApproachExitError = Mathf.Max(pickupApproachEnterError + 0.005f, pickupApproachExitError);
        pickupMinimumSteer = Mathf.Min(pickupMinimumSteer, pickupMaximumSteer);
        pickupMinimumForward = Mathf.Min(pickupMinimumForward, pickupMaximumForward);
    }

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        diagLogger = GetComponent<DiagnosticLogger>();
        startPosition = transform.position;
        startRotation = transform.rotation;

        LoadRewardConfigFromEnvironmentParameters();
        ValidateVisionCalibration();

        // A real robot must remain stopped after completion instead of being reset by Agent.MaxStep.
        if (useRealRobot && !endEpisodeOnPickupComplete)
            MaxStep = 0;

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

    private void ValidateVisionCalibration()
    {
        if (imageWidthPixels <= 0f)
            Debug.LogError("[RobotBrain] imageWidthPixels must be positive.");
        if (focalLengthPx <= 0f)
            Debug.LogError("[RobotBrain] focalLengthPx must be positive.");
        if (realBallDiameterMeters <= 0f)
            Debug.LogError("[RobotBrain] realBallDiameterMeters must be positive.");
        if (maxDetectionDistance <= 0f)
            Debug.LogError("[RobotBrain] maxDetectionDistance must be positive.");

        float ballRadius = 0.5f * realBallDiameterMeters;
        if (cameraHeightMeters < ballRadius)
            Debug.LogWarning("[RobotBrain] Camera height is below the ball centre; check calibration values.");
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

        // The real robot waits for an M2M activate signal; simulation always starts navigating.
        currentTaskState = (useRealRobot && requireM2MActivation)
            ? RobotTaskState.Idle
            : RobotTaskState.Navigation;
        ballVisibleDuration = 0f;
        pickupBallLostDuration = 0f;
        graspDuration = 0f;
        holdingDuration = 0f;
        holdingLostDuration = 0f;
        scriptedGas = 0f;
        scriptedSteer = 0f;
        scriptedGasVelocity = 0f;
        scriptedSteerVelocity = 0f;
        ResetPickupCenteringState();
        completionHandled = false;
        gripperCloseRequested = false;
        OpenGripper();

        prevGas = 0f;
        prevSteer = 0f;
        lastKnownBallAngle = 0f;
        lastDetectionTime = Time.time - 10f;
        cameraPivotAngle = 0f;

        filteredRealDistance = -1f;
        rawComputedDistance = -1f;
        lastComputedDistance = -1f;
        lastValidDistanceTime = float.NegativeInfinity;
        lastProcessedVisionSequence = -1;
        lastProcessedVisionTimestamp = double.NaN;

        float initDist = GetCurrentDistanceToBall();
        prevDistanceToBall = initDist;
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
        if (useRealRobot)
        {
            YoloDataPacket packet;
            return realVision != null
                && realVision.TryGetLatestPacket(out packet)
                && packet.sees > 0.5f
                && packet.conf >= minimumVisionConfidence;
        }

        return yoloCamera != null && yoloCamera.IsBallVisible;
    }

    private float GetBallRelativeAngle()
    {
        if (useRealRobot)
        {
            if (realVision == null || !IsBallVisible())
                return 0f;

            return realVision.GetCalibratedNormalizedAngle(focalLengthPx, imageWidthPixels);
        }

        return yoloCamera != null ? yoloCamera.RelativeAngle : 0f;
    }

    private float GetBallRelativeBearingDegrees()
    {
        if (useRealRobot)
        {
            if (realVision == null || !IsBallVisible())
                return 0f;

            return realVision.GetHorizontalBearingDegrees(focalLengthPx, imageWidthPixels);
        }

        return GetBallRelativeAngle() * cameraPivotMaxAngle;
    }

    private float GetCameraHalfFovDegrees()
    {
        float frameWidth = imageWidthPixels;
        if (useRealRobot && realVision != null && realVision.frameWidthPixels > 0)
            frameWidth = realVision.frameWidthPixels;

        if (frameWidth <= 0f || focalLengthPx <= 0f)
            return cameraPivotMaxAngle;

        return Mathf.Atan(0.5f * frameWidth / focalLengthPx) * Mathf.Rad2Deg;
    }

    private float GetEffectiveCameraPivotAngle()
    {
        if (useRealRobot && rosBridge != null)
            return rosBridge.CurrentCameraNormalizedAngle * cameraPivotMaxAngle;

        return cameraPivotAngle;
    }

    public float ComputeDistanceToBall(YoloDataPacket data)
    {
        return RealVision.ComputeHorizontalDistanceToBall(
            data,
            imageWidthPixels,
            focalLengthPx,
            realBallDiameterMeters,
            cameraHeightMeters,
            distanceScale,
            distanceOffsetMeters);
    }

    private float GetComputedDistanceToBall()
    {
        if (realVision == null)
            return -1f;

        YoloDataPacket packet;
        if (!realVision.TryGetLatestPacket(out packet)
            || packet.sees <= 0.5f
            || packet.conf < minimumVisionConfidence
            || packet.clipped)
        {
            rawComputedDistance = -1f;
            lastComputedDistance = -1f;
            if (Time.time - lastValidDistanceTime > distanceFilterResetSeconds)
                filteredRealDistance = -1f;
            return -1f;
        }

        bool alreadyProcessed = packet.seq == lastProcessedVisionSequence
            && packet.timestamp.Equals(lastProcessedVisionTimestamp);
        if (alreadyProcessed)
            return lastComputedDistance;

        lastProcessedVisionSequence = packet.seq;
        lastProcessedVisionTimestamp = packet.timestamp;
        rawComputedDistance = ComputeDistanceToBall(packet);

        if (rawComputedDistance < 0f)
        {
            lastComputedDistance = -1f;
            return -1f;
        }

        bool filterExpired = Time.time - lastValidDistanceTime > distanceFilterResetSeconds;
        if (filteredRealDistance < 0f || filterExpired)
            filteredRealDistance = rawComputedDistance;
        else
            filteredRealDistance = Mathf.Lerp(
                filteredRealDistance,
                rawComputedDistance,
                distanceSmoothingFactor);

        lastValidDistanceTime = Time.time;
        lastComputedDistance = filteredRealDistance;
        return lastComputedDistance;
    }

    private float GetBallNormalizedDistance()
    {
        if (useRealRobot)
        {
            float distanceMeters = GetComputedDistanceToBall();
            if (distanceMeters < 0f)
                return 1f;

            return Mathf.Clamp01(distanceMeters / maxDetectionDistance);
        }

        return yoloCamera != null ? yoloCamera.NormalizedDistance : 1f;
    }

    private float GetCurrentDistanceToBall()
    {
        if (useRealRobot)
            return GetComputedDistanceToBall();

        if (targetBall != null)
            return Vector3.Distance(transform.position, targetBall.position);

        return -1f;
    }

    public bool CalibrateVisionAtKnownDistance(float knownHorizontalDistanceMeters)
    {
        if (realVision == null)
        {
            Debug.LogError("[RobotBrain] Cannot calibrate: RealVision is not assigned.");
            return false;
        }

        YoloDataPacket packet;
        if (!realVision.TryGetLatestPacket(out packet))
        {
            string age = float.IsInfinity(realVision.PacketAgeSeconds)
                ? "N/A"
                : $"{realVision.PacketAgeSeconds:F2}s";
            Debug.LogError(
                $"[RobotBrain] Cannot calibrate: no fresh UDP packet. "
                + $"useYOLO={realVision.useYOLO}, seq={realVision.lastPacketSequence}, "
                + $"age={age}, frame={realVision.frameWidthPixels}x{realVision.frameHeightPixels}. "
                + "Run calibration in Play Mode with the updated Python node sending protocol v1.");
            return false;
        }

        if (packet.sees <= 0.5f)
        {
            Debug.LogError(
                $"[RobotBrain] Cannot calibrate: the latest packet contains no ball detection "
                + $"(seq={packet.seq}, conf={packet.conf:F2}). Keep the ball fully visible in the camera.");
            return false;
        }

        if (packet.conf < minimumVisionConfidence)
        {
            Debug.LogError(
                $"[RobotBrain] Cannot calibrate: YOLO confidence {packet.conf:F2} is below "
                + $"minimumVisionConfidence {minimumVisionConfidence:F2}.");
            return false;
        }

        if (packet.clipped)
        {
            Debug.LogError(
                "[RobotBrain] Cannot calibrate: the ball bounding box touches the image edge. "
                + "Move the entire ball inside the frame.");
            return false;
        }

        float calibratedFocalLength;
        bool success = RealVision.TryCalibrateFocalLength(
            packet,
            imageWidthPixels,
            realBallDiameterMeters,
            cameraHeightMeters,
            knownHorizontalDistanceMeters,
            out calibratedFocalLength);

        if (!success)
        {
            Debug.LogError("[RobotBrain] Calibration failed. Keep the entire ball inside the frame and verify dimensions.");
            return false;
        }

        focalLengthPx = calibratedFocalLength;
        filteredRealDistance = -1f;
        lastProcessedVisionSequence = -1;
        lastProcessedVisionTimestamp = double.NaN;
        Debug.Log($"[RobotBrain] YOLO focal length calibrated: {focalLengthPx:F2} px "
            + $"at {knownHorizontalDistanceMeters:F3} m. Copy this value to the inspector if calibration ran in Play Mode.");
        return true;
    }

    [ContextMenu("Calibrate YOLO focal length from current detection")]
    private void CalibrateVisionFromInspectorDistance()
    {
        CalibrateVisionAtKnownDistance(calibrationKnownDistanceMeters);
    }

    private bool GetIsHolding()
    {
        if (useRealRobot)
            return realGripperHolding;
        return gripperController != null && gripperController.IsHolding;
    }

    private void FixedUpdate()
    {
        if (!Application.isPlaying)
            return;

        UpdateRealGripperHoldingState();
        UpdateBallTrackingMemory();

        switch (currentTaskState)
        {
            case RobotTaskState.Idle:
                UpdateIdleWaitingForActivation();
                break;
            case RobotTaskState.Navigation:
                UpdateNavigationHandoff(Time.fixedDeltaTime);
                break;
            case RobotTaskState.ScriptedPickup:
                UpdateScriptedPickup(Time.fixedDeltaTime);
                break;
            case RobotTaskState.Grasping:
                UpdateGrasping(Time.fixedDeltaTime);
                break;
            case RobotTaskState.HoldingConfirmation:
                UpdateHoldingConfirmation(Time.fixedDeltaTime);
                break;
            case RobotTaskState.Completed:
                KeepRobotStoppedWithBall();
                break;
        }
    }

    private void UpdateRealGripperHoldingState()
    {
        if (!useRealRobot)
            return;

        if (!gripperCloseRequested)
        {
            realGripperDetectionTicks = 0;
            realGripperHolding = false;
            return;
        }

        if (GetGripperIRBallDetected() > gripperDetectionThreshold)
            realGripperDetectionTicks++;
        else
            realGripperDetectionTicks = 0;

        realGripperHolding = realGripperDetectionTicks >= requiredGripperDetectionTicks;
    }

    private void UpdateBallTrackingMemory()
    {
        bool visible = IsBallVisible();
        lastBallVisible = visible;

        if (!visible)
            return;

        lastDetectionTime = Time.time;
        lastBallDist = GetBallNormalizedDistance();
        lastBallAngle = GetBallAngleToChassisNormalized();
        lastKnownBallAngle = lastBallAngle;
    }

    private float GetBallAngleToChassisNormalized()
    {
        if (!IsBallVisible())
            return lastKnownBallAngle;

        float effectivePivotAngle = GetEffectiveCameraPivotAngle();
        float relativeBearingDegrees = GetBallRelativeBearingDegrees();
        float chassisBearingDegrees = effectivePivotAngle + relativeBearingDegrees;
        float maxObservableAngle = cameraPivotMaxAngle
            + (useRealRobot ? GetCameraHalfFovDegrees() : cameraPivotMaxAngle);

        if (maxObservableAngle <= 0.001f)
            return 0f;

        return Mathf.Clamp(chassisBearingDegrees / maxObservableAngle, -1f, 1f);
    }

    private void UpdateIdleWaitingForActivation()
    {
        // Keep motors at zero every tick, not just once on entry: any stray external
        // /cmd_vel publish while waiting must not move the rover.
        StopDriveImmediately();

        if (!useRealRobot || !requireM2MActivation)
        {
            TransitionTo(RobotTaskState.Navigation, "M2M activation not required");
            return;
        }

        if (M2MListener.TryConsumePendingActivation(out string targetClassName))
        {
            string reason = string.IsNullOrEmpty(targetClassName)
                ? "M2M activate/start received"
                : $"M2M activate/start received (target class: {targetClassName})";
            TransitionTo(RobotTaskState.Navigation, reason);
        }
    }

    private void UpdateNavigationHandoff(float deltaTime)
    {
        if (IsBallVisible())
        {
            ballVisibleDuration += deltaTime;
            if (ballVisibleDuration >= ballVisibleConfirmationSeconds)
            {
                AddReward(navigationHandoffReward);
                TransitionTo(RobotTaskState.ScriptedPickup, "ball visible continuously");
            }
        }
        else
        {
            ballVisibleDuration = 0f;
        }
    }

    private void UpdateScriptedPickup(float deltaTime)
    {
        CenterCameraForPickup(deltaTime);

        bool visible = IsBallVisible();
        float targetGas = 0f;
        float targetSteer = 0f;

        if (!visible)
        {
            pickupBallLostDuration += deltaTime;
            pickupApproachEnabled = false;
            pickupAlignedDuration = 0f;
            pickupCameraReadyDuration = 0f;
            pickupCameraReady = false;

            targetSteer = ComputePickupSteer(lastKnownBallAngle);

            if (pickupBallLostDuration >= pickupLostBallTimeoutSeconds)
            {
                OpenGripper();
                TransitionTo(RobotTaskState.Navigation, "ball lost during pickup");
                return;
            }
        }
        else
        {
            pickupBallLostDuration = 0f;

            float cameraAngleDegrees = Mathf.Abs(GetEffectiveCameraPivotAngle());
            if (cameraAngleDegrees <= pickupCameraReadyToleranceDegrees)
                pickupCameraReadyDuration += deltaTime;
            else
                pickupCameraReadyDuration = 0f;

            pickupCameraReady = cameraAngleDegrees <= pickupCameraReadyToleranceDegrees
                && (pickupCameraReadyHoldSeconds <= 0f
                    || pickupCameraReadyDuration >= pickupCameraReadyHoldSeconds);

            // While the camera is returning to zero, use chassis-frame error and do not move forward.
            // Once stable, use direct image error so the target offset maps exactly to the gripper axis.
            float rawError = pickupCameraReady
                ? GetBallRelativeAngle() - pickupAimOffsetNormalized
                : GetBallAngleToChassisNormalized();

            pickupRawHorizontalError = Mathf.Clamp(rawError, -1f, 1f);
            pickupFilteredHorizontalError = FilterPickupError(pickupRawHorizontalError, deltaTime);
            float absoluteError = Mathf.Abs(pickupFilteredHorizontalError);

            targetSteer = ComputePickupSteer(pickupFilteredHorizontalError);

            if (pickupCameraReady && absoluteError <= pickupApproachEnterError)
                pickupAlignedDuration += deltaTime;
            else
                pickupAlignedDuration = 0f;

            if (!pickupApproachEnabled
                && pickupCameraReady
                && pickupAlignedDuration >= pickupCenteredHoldSeconds)
            {
                pickupApproachEnabled = true;
            }

            if (pickupApproachEnabled
                && (!pickupCameraReady || absoluteError >= pickupApproachExitError))
            {
                pickupApproachEnabled = false;
                pickupAlignedDuration = 0f;
            }

            float distanceMeters = GetCurrentDistanceToBall();
            bool gripperSeesBall = GetGripperIRBallDetected() > gripperDetectionThreshold;
            bool closeByDistance = allowDistanceTriggeredGrasp
                && distanceMeters >= 0f
                && distanceMeters <= pickupCloseDistanceMeters
                && absoluteError <= pickupApproachEnterError;

            if (gripperSeesBall || closeByDistance)
            {
                StopDriveImmediately();
                CloseGripper();
                TransitionTo(RobotTaskState.Grasping,
                    gripperSeesBall ? "gripper IR detected ball" : "ball reached grasp distance");
                return;
            }

            if (pickupApproachEnabled)
            {
                float baseGas;
                if (distanceMeters < 0f)
                {
                    baseGas = pickupMinimumForward;
                }
                else
                {
                    float distanceFactor = Mathf.InverseLerp(
                        pickupCloseDistanceMeters,
                        Mathf.Max(pickupCloseDistanceMeters + 0.01f, pickupSlowDistanceMeters),
                        distanceMeters);
                    baseGas = Mathf.Lerp(pickupMinimumForward, pickupMaximumForward, distanceFactor);
                }

                // Continuously remove forward speed as lateral error approaches the exit threshold.
                float alignmentFactor = 1f - Mathf.InverseLerp(
                    pickupApproachEnterError,
                    Mathf.Max(pickupApproachEnterError + 0.001f, pickupApproachExitError),
                    absoluteError);
                alignmentFactor = Mathf.Clamp01(alignmentFactor);
                alignmentFactor *= alignmentFactor;
                targetGas = baseGas * alignmentFactor;
            }

            if (pickupUseUltrasonicEmergencyStop
                && GetUSNormalizedDistance() < pickupUltrasonicStopDistance
                && (distanceMeters < 0f || distanceMeters > pickupCloseDistanceMeters * 1.5f))
            {
                targetGas = 0f;
            }
        }

        ApplySmoothedScriptedDrive(targetGas, targetSteer, deltaTime);
    }

    private float FilterPickupError(float rawError, float deltaTime)
    {
        if (!pickupErrorFilterInitialized || pickupErrorFilterTime <= 0f)
        {
            pickupFilteredHorizontalError = rawError;
            pickupErrorFilterInitialized = true;
            return pickupFilteredHorizontalError;
        }

        float alpha = 1f - Mathf.Exp(-deltaTime / Mathf.Max(0.001f, pickupErrorFilterTime));
        pickupFilteredHorizontalError = Mathf.Lerp(
            pickupFilteredHorizontalError,
            rawError,
            alpha);
        return pickupFilteredHorizontalError;
    }

    private float ComputePickupSteer(float horizontalError)
    {
        float absoluteError = Mathf.Abs(horizontalError);
        if (absoluteError <= pickupCenterDeadZone)
            return 0f;

        float response = Mathf.InverseLerp(
            pickupCenterDeadZone,
            Mathf.Max(pickupCenterDeadZone + 0.001f, pickupFullSteerError),
            absoluteError);
        float magnitude = Mathf.Lerp(pickupMinimumSteer, pickupMaximumSteer, response);
        return Mathf.Sign(horizontalError)
            * magnitude
            * Mathf.Sign(Mathf.Approximately(pickupSteerDirection, 0f) ? 1f : pickupSteerDirection);
    }

    private void UpdateGrasping(float deltaTime)
    {
        StopDriveImmediately();
        CloseGripper();
        graspDuration += deltaTime;

        if (GetIsHolding())
        {
            holdingDuration = 0f;
            holdingLostDuration = 0f;
            TransitionTo(RobotTaskState.HoldingConfirmation, "grasp detected");
            return;
        }

        if (graspDuration >= graspTimeoutSeconds)
        {
            OpenGripper();
            TransitionTo(IsBallVisible() ? RobotTaskState.ScriptedPickup : RobotTaskState.Navigation,
                "grasp timeout");
        }
    }

    private void UpdateHoldingConfirmation(float deltaTime)
    {
        StopDriveImmediately();
        CloseGripper();

        if (GetIsHolding())
        {
            holdingLostDuration = 0f;
            holdingDuration += deltaTime;
            AddReward(holdingBallReward * deltaTime);

            if (holdingDuration >= holdingConfirmationSeconds)
                TransitionTo(RobotTaskState.Completed, "ball held continuously");
        }
        else
        {
            holdingLostDuration += deltaTime;
            if (holdingLostDuration >= holdingLossGraceSeconds)
            {
                holdingDuration = 0f;
                TransitionTo(RobotTaskState.Grasping, "holding sensor lost");
            }
        }
    }

    private void KeepRobotStoppedWithBall()
    {
        StopDriveImmediately();
        CloseGripper();
    }

    private void TransitionTo(RobotTaskState nextState, string reason)
    {
        if (currentTaskState == nextState)
            return;

        RobotTaskState previousState = currentTaskState;
        currentTaskState = nextState;

        if (nextState == RobotTaskState.Navigation)
        {
            StopDriveImmediately();
            ballVisibleDuration = 0f;
            pickupBallLostDuration = 0f;
            graspDuration = 0f;
            holdingDuration = 0f;
            holdingLostDuration = 0f;
            scriptedGas = 0f;
            scriptedSteer = 0f;
            scriptedGasVelocity = 0f;
            scriptedSteerVelocity = 0f;
            ResetPickupCenteringState();
            actionBuffer.Clear();
            for (int i = 0; i < currentActionLatency; i++)
                actionBuffer.Enqueue(new float[] { 0f, 0f, 0f });
        }
        else if (nextState == RobotTaskState.ScriptedPickup)
        {
            pickupBallLostDuration = 0f;
            graspDuration = 0f;
            holdingDuration = 0f;
            holdingLostDuration = 0f;
            ResetPickupCenteringState();
            StopDriveImmediately();
            OpenGripper();
        }
        else if (nextState == RobotTaskState.Grasping)
        {
            graspDuration = 0f;
            holdingDuration = 0f;
            holdingLostDuration = 0f;
        }
        else if (nextState == RobotTaskState.HoldingConfirmation)
        {
            holdingDuration = 0f;
            holdingLostDuration = 0f;
        }
        else if (nextState == RobotTaskState.Completed)
        {
            StopDriveImmediately();
            CloseGripper();

            if (!completionHandled)
            {
                completionHandled = true;
                AddReward(successReward);
                LogEpisodeOutcome("success");
                Debug.Log($"[RobotBrain] Task complete: ball held for {holdingConfirmationSeconds:F1} seconds.");

                if (endEpisodeOnPickupComplete)
                    EndEpisode();
            }
        }

        Debug.Log($"[RobotBrain] State {previousState} -> {nextState}: {reason}");
    }

    private void ApplySmoothedScriptedDrive(float targetGas, float targetSteer, float deltaTime)
    {
        scriptedGas = Mathf.SmoothDamp(
            scriptedGas,
            targetGas,
            ref scriptedGasVelocity,
            Mathf.Max(0.01f, pickupGasSmoothTime),
            Mathf.Infinity,
            deltaTime);
        scriptedSteer = Mathf.SmoothDamp(
            scriptedSteer,
            targetSteer,
            ref scriptedSteerVelocity,
            Mathf.Max(0.01f, pickupSteerSmoothTime),
            Mathf.Infinity,
            deltaTime);

        scriptedGas = Mathf.Clamp(scriptedGas, -1f, 1f);
        scriptedSteer = Mathf.Clamp(scriptedSteer, -1f, 1f);
        SendDriveCommand(scriptedGas, scriptedSteer);
        prevGas = scriptedGas;
        prevSteer = scriptedSteer;
    }

    private void ResetPickupCenteringState()
    {
        pickupRawHorizontalError = 0f;
        pickupFilteredHorizontalError = 0f;
        pickupAlignedDuration = 0f;
        pickupCameraReadyDuration = 0f;
        pickupCameraReady = false;
        pickupApproachEnabled = false;
        pickupErrorFilterInitialized = false;
    }

    private void StopDriveImmediately()
    {
        scriptedGas = 0f;
        scriptedSteer = 0f;
        scriptedGasVelocity = 0f;
        scriptedSteerVelocity = 0f;
        SendDriveCommand(0f, 0f);
        prevGas = 0f;
        prevSteer = 0f;
    }

    private void SendDriveCommand(float gas, float steer)
    {
        gas = Mathf.Clamp(gas, -1f, 1f);
        steer = Mathf.Clamp(steer, -1f, 1f);

        if (useRealRobot)
        {
            rosBridge?.PublishDrive(gas, steer);
        }
        else if (trackController != null)
        {
            trackController.GasInput = gas;
            trackController.SteerInput = steer;
        }
    }

    private void CenterCameraForPickup(float deltaTime)
    {
        cameraPivotAngle = Mathf.MoveTowards(
            cameraPivotAngle,
            0f,
            pickupCameraCenterSpeedDegrees * deltaTime);
        ApplyCameraCommand();
    }

    private void ApplyCameraCommand()
    {
        if (!useRealRobot && cameraPivot != null)
            cameraPivot.localRotation = Quaternion.Euler(15f, cameraPivotAngle, 0f);

        if (useRealRobot && rosBridge != null)
        {
            float normalized = cameraPivotMaxAngle > 0f
                ? cameraPivotAngle / cameraPivotMaxAngle
                : 0f;
            rosBridge.PublishCamera(normalized);
        }
    }

    private void OpenGripper()
    {
        gripperCloseRequested = false;
        realGripperHolding = false;
        realGripperDetectionTicks = 0;

        if (useRealRobot)
            rosBridge?.PublishGripper(gripperOpenCommand);
        else if (gripperController != null)
            gripperController.GripperCloseCommand = false;
    }

    private void CloseGripper()
    {
        gripperCloseRequested = true;

        if (useRealRobot)
            rosBridge?.PublishGripper(gripperCloseCommand);
        else if (gripperController != null)
            gripperController.GripperCloseCommand = true;
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
            bool hasCurrentDistance = currentDistance >= 0f;
            bool hadPreviousDistance = prevDistanceToBall >= 0f;
            bool canSeeBall = IsBallVisible();
            bool isCloseBlindZone = hasCurrentDistance && currentDistance < 0.6f;

            if (hasCurrentDistance && hadPreviousDistance && (canSeeBall || isCloseBlindZone))
            {
                float delta = prevDistanceToBall - currentDistance;
                float rewardScale = currentDistance < nearDistanceThreshold
                    ? distanceRewardNear
                    : distanceRewardFar;
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

            if (hasCurrentDistance)
                prevDistanceToBall = currentDistance;
            else if (useRealRobot)
                prevDistanceToBall = -1f;

            prevZPosition = transform.position.z;
        }

        if (IsBallVisible())
        {
            float effectivePivotAngle = GetEffectiveCameraPivotAngle();
            float chassisAlignment = cameraPivotMaxAngle > 0f
                ? 1f - Mathf.Clamp01(Mathf.Abs(effectivePivotAngle) / cameraPivotMaxAngle)
                : 1f;
            rewardCentering = centeringRewardScale
                * (1f - Mathf.Abs(GetBallRelativeAngle()))
                * chassisAlignment;
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
        float effectivePivotAngle = GetEffectiveCameraPivotAngle();
        float relativeBallAngle = ballVisible ? GetBallRelativeAngle() : 0f;
        float chassisBallAngleNormalized = ballVisible ? GetBallAngleToChassisNormalized() : lastKnownBallAngle;

        if (ballVisible)
        {
            lastDetectionTime = Time.time;
            // Store the last location in the chassis frame, not in the moving camera frame.
            lastKnownBallAngle = chassisBallAngleNormalized;
        }

        lastBallVisible = ballVisible;
        lastBallDist = ballVisible ? GetBallNormalizedDistance() : 1f;
        lastBallAngle = chassisBallAngleNormalized;

        sensor.AddObservation(relativeBallAngle);
        sensor.AddObservation(ballVisible ? GetBallNormalizedDistance() : 1f);
        sensor.AddObservation(lastKnownBallAngle);
        sensor.AddObservation(ballVisible ? 1.0f : 0.0f);
        sensor.AddObservation(cameraPivotMaxAngle > 0f ? effectivePivotAngle / cameraPivotMaxAngle : 0f);

        sensor.AddObservation(GetIsHolding() ? 1f : 0f);
        sensor.AddObservation(Mathf.DeltaAngle(startRotation.eulerAngles.y, transform.eulerAngles.y) / 180f);
        sensor.AddObservation(Mathf.Clamp01(rb.linearVelocity.magnitude / 0.5f));
        float timeSinceLastDetection = Time.time - lastDetectionTime;
        sensor.AddObservation(Mathf.Clamp(timeSinceLastDetection, 0f, 10f) / 10f);
    }

    // ==================== ACTIONS ====================

    public override void OnActionReceived(ActionBuffers actions)
    {
        // The navigation model owns the actuators only in Navigation state.
        if (currentTaskState != RobotTaskState.Navigation)
            return;

        float gas;
        float steer;
        float cameraSignal;

        if (Academy.Instance.IsCommunicatorOn && currentActionLatency > 0)
        {
            float[] newActions = new float[]
            {
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

        SendDriveCommand(gas, steer);

        cameraPivotAngle = Mathf.Clamp(
            cameraPivotAngle + cameraSignal * cameraPivotSpeed * Time.fixedDeltaTime,
            -cameraPivotMaxAngle,
            cameraPivotMaxAngle);
        ApplyCameraCommand();

        CalculateRewards(gas, steer);

        prevGas = gas;
        prevSteer = steer;
        prevZPosition = transform.position.z;

        if (diagLogger != null)
        {
            bool holding = GetIsHolding();
            bool isRetryingHeuristic = gas < backwardGasThreshold
                && lastBallDist < nearDistanceThreshold
                && (Time.time - lastDetectionTime) < 1f;

            diagLogger.LogStep(
                StepCount,
                lastBallVisible, lastBallAngle, lastBallDist,
                GetUSNormalizedDistance(),
                GetLeftIRObstacle() > 0.5f ? 1 : 0,
                GetRightIRObstacle() > 0.5f ? 1 : 0,
                GetGripperIRBallDetected() > gripperDetectionThreshold ? 1 : 0,
                GetEffectiveCameraPivotAngle(), gas, steer,
                holding, Mathf.RoundToInt(holdingDuration / Mathf.Max(Time.fixedDeltaTime, 0.001f)),
                isRetryingHeuristic,
                transform.position.x - startPosition.x,
                transform.position.z - startPosition.z,
                transform.eulerAngles.y / 360f,
                rb != null ? rb.linearVelocity.magnitude : 0f);
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
