using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public class ROSBridge : MonoBehaviour
{
    [Header("ROS Topics")]
    public string cmdVelTopic = "/cmd_vel";
    public string gripperTopic = "/cmd_gripper";
    public string cameraPanTopic = "/cmd_camera_pan";

    [Header("Robot Speed Limits (m/s and rad/s)")]
    public float maxLinearSpeed = 0.5f;
    public float maxAngularSpeed = 1.0f;

    [Header("EMA Filter")]
    [Range(0.1f, 1f)]
    public float emaAlpha = 0.8f;
    public bool enableFilter = true;

    private ROSConnection ros;
    private float smoothGas = 0f;
    private float smoothSteering = 0f;
    private bool isInitialized = false;

    void Start()
    {
        try
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<TwistMsg>(cmdVelTopic);
            ros.RegisterPublisher<Int32Msg>(gripperTopic);
            ros.RegisterPublisher<Float32Msg>(cameraPanTopic);
            isInitialized = true;
            Debug.Log($"[ROSBridge] Successfully registered publishers for topics: {cmdVelTopic}, {gripperTopic}, {cameraPanTopic}");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[ROSBridge] Failed to initialize ROS connection: {e.Message}");
        }
    }

    public void PublishCommand(float gas, float steering)
    {
        if (!isInitialized) return;

        if (Mathf.Approximately(gas, 0f) && Mathf.Approximately(steering, 0f))
        {
            smoothGas = 0f;
            smoothSteering = 0f;
        }
        else if (enableFilter)
        {
            smoothGas = emaAlpha * gas + (1f - emaAlpha) * smoothGas;
            smoothSteering = emaAlpha * steering + (1f - emaAlpha) * smoothSteering;
        }
        else
        {
            smoothGas = gas;
            smoothSteering = steering;
        }

        float clampedGas = Mathf.Clamp(smoothGas, -1f, 1f);
        float clampedSteering = Mathf.Clamp(smoothSteering, -1f, 1f);

        TwistMsg cmd = new TwistMsg();
        cmd.linear.x = clampedGas * maxLinearSpeed;
        cmd.angular.z = clampedSteering * maxAngularSpeed;

        try
        {
            ros.Publish(cmdVelTopic, cmd);
        }
        catch (System.Exception e)
        {
            Debug.LogWarning($"[ROSBridge] Failed to publish to {cmdVelTopic}: {e.Message}");
        }
    }

    public void PublishGripperCmd(int cmd)
    {
        if (!isInitialized) return;
        try
        {
            Int32Msg msg = new Int32Msg();
            msg.data = cmd;
            ros.Publish(gripperTopic, msg);
        }
        catch (System.Exception e)
        {
            Debug.LogWarning($"[ROSBridge] Failed to publish to {gripperTopic}: {e.Message}");
        }
    }

    public void PublishCameraCmd(float yaw)
    {
        if (!isInitialized) return;
        try
        {
            Float32Msg msg = new Float32Msg();
            msg.data = yaw;
            ros.Publish(cameraPanTopic, msg);
        }
        catch (System.Exception e)
        {
            Debug.LogWarning($"[ROSBridge] Failed to publish to {cameraPanTopic}: {e.Message}");
        }
    }
}