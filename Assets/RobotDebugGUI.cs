using UnityEngine;
using System.Text;

/// <summary>
/// Runtime diagnostics for RobotBrain. Uses an explicit debug API instead of reflection,
/// so renamed private fields cannot silently break the overlay.
/// </summary>
public class RobotDebugGUI : MonoBehaviour
{
    [Header("References")]
    [Tooltip("If empty, RobotBrain is searched on the same GameObject.")]
    [SerializeField] private RobotBrain robotBrain;

    [Header("Display Settings")]
    [SerializeField] private bool showOnStart = true;
    [SerializeField] private KeyCode toggleKey = KeyCode.F12;
    [SerializeField] private int fontSize = 14;
    [SerializeField] private float windowWidth = 520f;
    [SerializeField] private float windowHeight = 720f;
    [SerializeField, Min(0.05f)] private float displayUpdateInterval = 0.2f;

    [Header("Sensor Display")]
    [Tooltip("Threshold used only to display digital ON/OFF states for IR sensors.")]
    [SerializeField, Range(0f, 1f)] private float sensorActiveThreshold = 0.5f;
    [SerializeField] private bool showRawRosSensorPacket = true;

    [Header("Optional Console Output")]
    [SerializeField] private bool logSensorsToConsole = false;
    [SerializeField, Min(0.1f)] private float consoleLogInterval = 1f;

    private bool isVisible;
    private string infoText = string.Empty;
    private float displayTimer;
    private float consoleLogTimer;
    private Vector2 scrollPosition;

    private void Awake()
    {
        if (robotBrain == null)
            robotBrain = GetComponent<RobotBrain>();

        if (robotBrain == null)
        {
            Debug.LogError("[RobotDebugGUI] RobotBrain was not found. Overlay disabled.");
            enabled = false;
            return;
        }

        isVisible = showOnStart;
        UpdateInfoText();
    }

    private void Update()
    {
        if (Input.GetKeyDown(toggleKey))
            isVisible = !isVisible;

        displayTimer += Time.unscaledDeltaTime;
        if (displayTimer >= displayUpdateInterval)
        {
            displayTimer = 0f;
            UpdateInfoText();
        }

        if (logSensorsToConsole)
        {
            consoleLogTimer += Time.unscaledDeltaTime;
            if (consoleLogTimer >= consoleLogInterval)
            {
                consoleLogTimer = 0f;
                LogSensorReadings();
            }
        }
    }

    private void UpdateInfoText()
    {
        if (robotBrain == null)
            return;

        bool useReal = robotBrain.IsUsingRealRobot;
        RealVision vision = robotBrain.RealVisionSource;
        ROSBridge ros = robotBrain.RosBridgeSource;

        string rosStatus = ros == null
            ? "MISSING"
            : (ros.IsConnected ? "CONFIGURED" : "NOT CONFIGURED");
        string visionStatus = vision == null
            ? "MISSING"
            : (vision.HasFreshPacket ? "UDP FRESH" : "WAITING / STALE");

        float ultrasonic = robotBrain.DebugUSNormalizedDistance;
        float leftIr = robotBrain.DebugLeftIR;
        float rightIr = robotBrain.DebugRightIR;
        float gripperIr = robotBrain.DebugGripperIR;

        StringBuilder text = new StringBuilder(2200);
        text.AppendLine("<b>Robot Debug Info</b>");
        text.AppendLine($"Mode: {(useReal ? "REAL ROBOT" : "SIMULATION")}");
        text.AppendLine($"Task state: <b>{robotBrain.CurrentTaskState}</b>");
        text.AppendLine($"Steps: {robotBrain.StepCount}");
        text.AppendLine($"Cumulative Reward: {robotBrain.GetCumulativeReward():F3}");

        text.AppendLine("----------------------------------------");
        text.AppendLine("<b>Task flow</b>");
        text.AppendLine($"Visible confirmation: {robotBrain.DebugBallVisibleDuration:F2} / "
            + $"{robotBrain.DebugBallVisibleConfirmationSeconds:F2} s");
        text.AppendLine($"Pickup lost-ball timer: {robotBrain.DebugPickupBallLostDuration:F2} s");
        text.AppendLine($"Pickup camera ready: {robotBrain.DebugPickupCameraReady}");
        text.AppendLine($"Pickup error: raw={robotBrain.DebugPickupRawHorizontalError:+0.000;-0.000;0.000}, "
            + $"filtered={robotBrain.DebugPickupFilteredHorizontalError:+0.000;-0.000;0.000}");
        text.AppendLine($"Pickup aim offset: {robotBrain.DebugPickupAimOffsetNormalized:+0.000;-0.000;0.000}");
        text.AppendLine($"Aligned: {robotBrain.DebugPickupAlignedDuration:F2} / "
            + $"{robotBrain.DebugPickupCenteredHoldSeconds:F2} s, approach={robotBrain.DebugPickupApproachEnabled}");
        text.AppendLine($"Grasp timer: {robotBrain.DebugGraspDuration:F2} s");
        text.AppendLine($"Holding confirmation: {robotBrain.DebugHoldingDuration:F2} / "
            + $"{robotBrain.DebugHoldingConfirmationSeconds:F2} s");

        text.AppendLine("----------------------------------------");
        text.AppendLine("<b>YOLO / Vision</b>");
        text.AppendLine($"Ball visible: {robotBrain.DebugBallVisible}");
        text.AppendLine($"Ball angle: {robotBrain.DebugBallRelativeAngle:F3} norm / "
            + $"{robotBrain.DebugBallBearingDegrees:F2} deg");
        text.AppendLine($"Ball distance (norm): {robotBrain.DebugBallNormalizedDistance:F3}");
        text.AppendLine($"Ball distance: {FormatMetres(robotBrain.DebugBallDistanceMeters)}");
        text.AppendLine($"Raw distance: {FormatMetres(robotBrain.DebugRawBallDistanceMeters)}");
        text.AppendLine($"Calibration: f={robotBrain.DebugFocalLengthPx:F1}px, "
            + $"half-FOV={robotBrain.DebugCameraHalfFovDegrees:F1} deg");

        if (vision != null)
        {
            string packetAge = float.IsInfinity(vision.PacketAgeSeconds)
                ? "N/A"
                : $"{vision.PacketAgeSeconds:F2}s";
            text.AppendLine($"Packet: seq={vision.lastPacketSequence}, age={packetAge}, "
                + $"frame={vision.frameWidthPixels}x{vision.frameHeightPixels}");
            text.AppendLine($"Box: cx={vision.normalizedCenterX:F3}, cy={vision.normalizedCenterY:F3}, "
                + $"w={vision.ballWidth:F3}, h={vision.ballHeight:F3}, "
                + $"conf={vision.confidence:F2}, clipped={vision.boundingBoxClipped}");
        }

        text.AppendLine($"Last chassis angle: {robotBrain.DebugLastKnownBallAngle:F3}");
        text.AppendLine($"Time since detection: {robotBrain.DebugTimeSinceLastDetection:F2}s");

        AppendSensorSection(text, useReal, ros, ultrasonic, leftIr, rightIr, gripperIr);

        text.AppendLine("----------------------------------------");
        text.AppendLine("<b>Actuators</b>");
        text.AppendLine($"Gas: {robotBrain.DebugCurrentGas:F3}  Steer: {robotBrain.DebugCurrentSteer:F3}");
        text.AppendLine($"Camera pivot: command={robotBrain.DebugCommandedCameraPivotAngle:F2} deg, "
            + $"smoothed={robotBrain.DebugEffectiveCameraPivotAngle:F2} deg");
        text.AppendLine($"Is holding: {robotBrain.DebugIsHolding}");

        if (useReal)
        {
            text.AppendLine($"Gripper command: {(ros != null ? FormatGripperCommand(ros.LastGripperCommand) : "N/A")}");
        }
        else
        {
            text.AppendLine($"Gripper closed: "
                + (robotBrain.DebugHasGripperController
                    ? robotBrain.DebugGripperCloseCommand.ToString()
                    : "N/A"));
        }

        text.AppendLine("----------------------------------------");

        if (!useReal)
        {
            text.AppendLine("<b>Simulation params</b>");
            text.AppendLine(robotBrain.DebugHasTrackController
                ? $"Move speed: {robotBrain.DebugMoveSpeed:F3}  Turn speed: {robotBrain.DebugTurnSpeed:F3}"
                : "TrackController: N/A");
            text.AppendLine(robotBrain.DebugHasTargetBall
                ? $"Ball mass: {robotBrain.DebugTargetBallMass:F3}  Scale: {robotBrain.DebugTargetBallScale:F3}"
                : "Target ball: N/A");
        }
        else
        {
            text.AppendLine("<b>Real robot</b>");
            text.AppendLine($"ROS Bridge: {rosStatus}");
            text.AppendLine($"RealVision: {visionStatus}");
        }

        text.AppendLine("----------------------------------------");
        text.AppendLine($"<i>Press {toggleKey} to toggle this overlay</i>");
        infoText = text.ToString();
    }

    private void AppendSensorSection(
        StringBuilder text,
        bool useReal,
        ROSBridge ros,
        float ultrasonic,
        float leftIr,
        float rightIr,
        float gripperIr)
    {
        text.AppendLine("----------------------------------------");
        text.AppendLine("<b>Sensors</b>");
        text.AppendLine($"Source: {(useReal ? "ROS / REAL" : "VIRTUAL / SIMULATION")}");

        if (useReal)
        {
            bool hasRosSensorData = ros != null && ros.HasSensorData && ros.GetSensorData() != null;
            text.AppendLine($"ROS sensor packet: {(hasRosSensorData ? "RECEIVED" : "NO DATA")}");

            if (hasRosSensorData && showRawRosSensorPacket)
            {
                var raw = ros.GetSensorData();
                text.AppendLine($"Raw QuaternionMsg: x={raw.x:F4}, y={raw.y:F4}, "
                    + $"z={raw.z:F4}, w={raw.w:F4}");
            }
        }

        // In the current ROS contract x is the ultrasonic distance in metres.
        // RobotBrain clamps it to [0,1] for the policy observation, so both forms are displayed.
        text.AppendLine($"Ultrasonic: {ultrasonic:F4} m / {ultrasonic * 100f:F1} cm");
        text.AppendLine($"Ultrasonic policy value: {Mathf.Clamp01(ultrasonic):F3}");
        text.AppendLine($"IR Left: {leftIr:F3}  [{FormatDigitalSensor(leftIr)}]");
        text.AppendLine($"IR Right: {rightIr:F3}  [{FormatDigitalSensor(rightIr)}]");
        text.AppendLine($"Gripper IR: {gripperIr:F3}  [{FormatDigitalSensor(gripperIr)}]");
    }

    private void LogSensorReadings()
    {
        if (robotBrain == null)
            return;

        ROSBridge ros = robotBrain.RosBridgeSource;
        float ultrasonic = robotBrain.DebugUSNormalizedDistance;
        float leftIr = robotBrain.DebugLeftIR;
        float rightIr = robotBrain.DebugRightIR;
        float gripperIr = robotBrain.DebugGripperIR;

        string rawRos = "raw=N/A";
        if (robotBrain.IsUsingRealRobot && ros != null && ros.HasSensorData && ros.GetSensorData() != null)
        {
            var raw = ros.GetSensorData();
            rawRos = $"raw(x={raw.x:F4}, y={raw.y:F4}, z={raw.z:F4}, w={raw.w:F4})";
        }

        Debug.Log(
            $"[RobotSensors] state={robotBrain.CurrentTaskState} "
            + $"pickupErr={robotBrain.DebugPickupFilteredHorizontalError:+0.000;-0.000;0.000}, "
            + $"approach={robotBrain.DebugPickupApproachEnabled}, "
            + $"US={ultrasonic:F4}m ({ultrasonic * 100f:F1}cm), "
            + $"IR_L={leftIr:F3}/{FormatDigitalSensor(leftIr)}, "
            + $"IR_R={rightIr:F3}/{FormatDigitalSensor(rightIr)}, "
            + $"GRIP_IR={gripperIr:F3}/{FormatDigitalSensor(gripperIr)}, "
            + rawRos);
    }

    private string FormatDigitalSensor(float value)
    {
        return value >= sensorActiveThreshold ? "ACTIVE" : "CLEAR";
    }

    private static string FormatMetres(float value)
    {
        return value >= 0f ? $"{value:F3} m" : "N/A";
    }

    private static string FormatGripperCommand(int command)
    {
        if (command == int.MinValue)
            return "NOT SENT";
        if (command == 1)
            return "1 (OPEN)";
        if (command == 2)
            return "2 (CLOSE)";
        return command.ToString();
    }

    private void OnGUI()
    {
        if (!isVisible || robotBrain == null)
            return;

        GUIStyle windowStyle = new GUIStyle(GUI.skin.window)
        {
            fontSize = fontSize
        };

        GUIStyle labelStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = fontSize,
            richText = true,
            wordWrap = true
        };

        float visibleWidth = Mathf.Min(windowWidth, Mathf.Max(200f, Screen.width - 20f));
        float visibleHeight = Mathf.Min(windowHeight, Mathf.Max(200f, Screen.height - 20f));
        Rect windowRect = new Rect(10f, 10f, visibleWidth, visibleHeight);

        GUI.Window(0, windowRect, id =>
        {
            scrollPosition = GUILayout.BeginScrollView(scrollPosition);
            GUILayout.Label(infoText, labelStyle);
            GUILayout.EndScrollView();
            GUI.DragWindow(new Rect(0f, 0f, visibleWidth, 22f));
        }, "Robot Debug", windowStyle);
    }
}
