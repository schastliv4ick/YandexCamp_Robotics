using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public class ROSBridge : MonoBehaviour
{
    // ==========================================
    // НАСТРОЙКИ (видны в инспекторе)
    // ==========================================
    [Header("ROS Connection")]
    [SerializeField] private string rosIPAddress = "127.0.0.1";
    [SerializeField] private int rosPort = 10000;

    [Header("Topics")]
    [SerializeField] private string cmdVelTopic = "/cmd_vel";
    [SerializeField] private string manipulatorTopic = "/manipulator_angles";

    [Header("Speed Limits (для движения)")]
    [SerializeField] private float maxLinearSpeed = 0.5f;
    [SerializeField] private float maxAngularSpeed = 1.0f;

    // ==========================================
    // ПРИВАТНЫЕ ПЕРЕМЕННЫЕ
    // ==========================================
    private ROSConnection ros;
    private TwistMsg twistMsg;
    private Float32MultiArrayMsg manipulatorMsg;
    private bool isConnected = false;

    // ==========================================
    // СВОЙСТВА
    // ==========================================
    public bool IsConnected => isConnected;

    // ==========================================
    void Start()
    {
        InitializeROS();
    }

    void OnDestroy()
    {
        if (isConnected)
        {
            PublishCommand(0f, 0f);
        }
    }

    // ==========================================
    // ИНИЦИАЛИЗАЦИЯ
    // ==========================================
    private void InitializeROS()
    {
        try
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.RosIPAddress = rosIPAddress;
            ros.RosPort = rosPort;

            // Регистрируем оба издателя
            ros.RegisterPublisher<TwistMsg>(cmdVelTopic);
            ros.RegisterPublisher<Float32MultiArrayMsg>(manipulatorTopic);

            twistMsg = new TwistMsg();
            manipulatorMsg = new Float32MultiArrayMsg();

            isConnected = true;
            Debug.Log($"[ROSBridge] Подключено к ROS {rosIPAddress}:{rosPort}");
        }
        catch (System.Exception e)
        {
            isConnected = false;
            Debug.LogError($"[ROSBridge] Ошибка: {e.Message}");
        }
    }

    // ==========================================
    // ПУБЛИКАЦИЯ КОМАНДЫ ДВИЖЕНИЯ
    // ==========================================
    public void PublishCommand(float linear, float angular)
    {
        if (!isConnected || ros == null) return;

        linear = Mathf.Clamp(linear, -1f, 1f);
        angular = Mathf.Clamp(angular, -1f, 1f);

        twistMsg.linear = new Vector3Msg();
        twistMsg.angular = new Vector3Msg();
        twistMsg.linear.x = linear * maxLinearSpeed;
        twistMsg.angular.z = angular * maxAngularSpeed;

        try
        {
            ros.Publish(cmdVelTopic, twistMsg);
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[ROSBridge] Ошибка публикации cmd_vel: {e.Message}");
        }
    }

    // ==========================================
    // ПУБЛИКАЦИЯ УГЛОВ МАНИПУЛЯТОРА
    // ==========================================
    public void PublishManipulator(float[] angles)
    {
        if (!isConnected || ros == null || angles.Length < 4) return;

        manipulatorMsg.data = new float[4];
        // Копируем первые 4 значения
        for (int i = 0; i < 4 && i < angles.Length; i++)
        {
            manipulatorMsg.data[i] = angles[i];
        }
        // Если меньше 4 – заполняем нулями
        if (angles.Length < 4)
        {
            for (int i = angles.Length; i < 4; i++)
                manipulatorMsg.data[i] = 0f;
        }

        try
        {
            ros.Publish(manipulatorTopic, manipulatorMsg);
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[ROSBridge] Ошибка публикации manipulator_angles: {e.Message}");
        }
    }

    // ==========================================
    // АВАРИЙНАЯ ОСТАНОВКА
    //==========================================
    public void EmergencyStop()
    {
        PublishCommand(0f, 0f);
        Debug.LogWarning("[ROSBridge] ⚠️ АВАРИЙНАЯ ОСТАНОВКА");
    }
}