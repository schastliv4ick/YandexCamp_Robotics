using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

/// <summary>
/// ROS transport for drive, gripper, camera pan and sensor feedback.
/// RobotBrain owns task logic; this component only transports commands and data.
/// </summary>
public class ROSBridge : MonoBehaviour
{
    [Header("ROS Connection")]
    [SerializeField] private string rosIPAddress = "192.168.2.156";
    [SerializeField] private int rosPort = 10000;

    [Header("Topics")]
    [SerializeField] private string cmdVelTopic = "/cmd_vel";
    [SerializeField] private string gripperTopic = "/cmd_gripper";
    [SerializeField] private string cameraTopic = "/cmd_camera_pan";
    [SerializeField] private string sensorTopic = "/sensor/data";

    [Header("Speed Limits")]
    [SerializeField, Min(0f)] private float maxLinearSpeed = 0.5f;
    [SerializeField, Min(0f)] private float maxAngularSpeed = 1.0f;

    [Header("Camera Smoothing")]
    [SerializeField, Min(0.01f)] private float cameraSmoothTime = 0.25f;

    private ROSConnection ros;
    private bool isConnected;
    private bool hasSensorData;

    private float currentCameraAngle;
    private float targetCameraAngle;
    private float cameraVelocity;
    private int lastGripperValue = int.MinValue;
    private QuaternionMsg lastSensorData;

    public bool IsConnected => isConnected;
    public bool HasSensorData => hasSensorData;
    public float CurrentCameraNormalizedAngle => currentCameraAngle;
    public float TargetCameraNormalizedAngle => targetCameraAngle;
    public int LastGripperCommand => lastGripperValue;

    private void Start()
    {
        currentCameraAngle = 0f;
        targetCameraAngle = 0f;
        InitializeROS();
    }

    private void Update()
    {
        float previousAngle = currentCameraAngle;
        currentCameraAngle = Mathf.SmoothDamp(
            currentCameraAngle,
            targetCameraAngle,
            ref cameraVelocity,
            Mathf.Max(0.01f, cameraSmoothTime),
            Mathf.Infinity,
            Time.deltaTime);

        currentCameraAngle = Mathf.Clamp(currentCameraAngle, -1f, 1f);
        if (Mathf.Abs(currentCameraAngle - previousAngle) > 0.0001f)
            PublishCameraRaw(currentCameraAngle);
    }

    private void InitializeROS()
    {
        try
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.RosIPAddress = rosIPAddress;
            ros.RosPort = rosPort;

            ros.RegisterPublisher<TwistMsg>(cmdVelTopic);
            ros.RegisterPublisher<Int32Msg>(gripperTopic);
            ros.RegisterPublisher<Float32Msg>(cameraTopic);
            ros.Subscribe<QuaternionMsg>(sensorTopic, SensorCallback);

            isConnected = true;
            Debug.Log($"[ROSBridge] ROS transport initialized for {rosIPAddress}:{rosPort}.");
        }
        catch (System.Exception exception)
        {
            isConnected = false;
            Debug.LogError($"[ROSBridge] Initialization failed: {exception.Message}");
        }
    }

    /// <summary>
    /// Publishes normalized drive commands. linear and angular are clamped to [-1, 1].
    /// </summary>
    public void PublishDrive(float linear, float angular)
    {
        if (!isConnected || ros == null)
            return;

        linear = Mathf.Clamp(linear, -1f, 1f);
        angular = Mathf.Clamp(angular, -1f, 1f);

        TwistMsg message = new TwistMsg
        {
            linear = new Vector3Msg { x = linear * maxLinearSpeed },
            angular = new Vector3Msg { z = angular * maxAngularSpeed }
        };

        try
        {
            ros.Publish(cmdVelTopic, message);
        }
        catch (System.Exception exception)
        {
            Debug.LogError($"[ROSBridge] cmd_vel publish failed: {exception.Message}");
        }
    }

    /// <summary>
    /// Publishes a gripper command only when its value changes.
    /// </summary>
    public void PublishGripper(int value)
    {
        if (!isConnected || ros == null)
            return;
        if (value == lastGripperValue)
            return;

        try
        {
            ros.Publish(gripperTopic, new Int32Msg { data = value });
            lastGripperValue = value;
            Debug.Log($"[ROSBridge] Gripper command: {value}");
        }
        catch (System.Exception exception)
        {
            Debug.LogError($"[ROSBridge] Gripper publish failed: {exception.Message}");
        }
    }

    /// <summary>
    /// Sets a normalized camera-pan target. Smoothing is applied in Update().
    /// </summary>
    public void PublishCamera(float normalizedAngle)
    {
        targetCameraAngle = Mathf.Clamp(normalizedAngle, -1f, 1f);
    }

    public QuaternionMsg GetSensorData()
    {
        return lastSensorData;
    }

    public void EmergencyStop()
    {
        PublishDrive(0f, 0f);
        targetCameraAngle = currentCameraAngle;
        Debug.LogWarning("[ROSBridge] Emergency drive stop published.");
    }

    private void PublishCameraRaw(float normalizedAngle)
    {
        if (!isConnected || ros == null)
            return;

        try
        {
            ros.Publish(cameraTopic, new Float32Msg { data = normalizedAngle });
        }
        catch (System.Exception exception)
        {
            Debug.LogError($"[ROSBridge] Camera publish failed: {exception.Message}");
        }
    }

    private void SensorCallback(QuaternionMsg message)
    {
        lastSensorData = message;
        hasSensorData = true;
    }

    private void OnDestroy()
    {
        if (isConnected)
            PublishDrive(0f, 0f);
    }
}
