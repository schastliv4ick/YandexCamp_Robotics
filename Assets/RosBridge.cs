using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

/// <summary>
/// ROS-мост для связи с роботом. Предоставляет API для публикации команд 
/// и получения данных с датчиков. Всё управление вынесено в отдельные скрипты.
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
    [SerializeField] private float maxLinearSpeed = 0.5f;
    [SerializeField] private float maxAngularSpeed = 1.0f;

    [Header("Camera Smoothing")]
    [SerializeField] private float cameraSmoothSpeed = 3.0f;

    private ROSConnection ros;
    private bool isConnected = false;

    // Текущее и целевое положение камеры (для плавного движения)
    private float currentCameraAngle = 0f;
    private float targetCameraAngle = 0f;

    // Последние полученные данные с датчиков
    private QuaternionMsg lastSensorData;

    // Флаг для предотвращения повторной публикации одной и той же команды клешни
    private int lastGripperValue = -1;

    public bool IsConnected => isConnected;

    void Start()
    {
        InitializeROS();
        currentCameraAngle = 0f;
        targetCameraAngle = 0f;
    }

    void Update()
    {
        // Плавное движение камеры к заданной цели
        if (Mathf.Abs(currentCameraAngle - targetCameraAngle) > 0.001f)
        {
            currentCameraAngle = Mathf.Lerp(currentCameraAngle, targetCameraAngle, Time.deltaTime * cameraSmoothSpeed);
            PublishCameraRaw(currentCameraAngle);
        }
    }

    void OnDestroy()
    {
        // При необходимости – экстренная остановка
        // EmergencyStop();
    }

    private void InitializeROS()
    {
        try
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.RosIPAddress = rosIPAddress;
            ros.RosPort = rosPort;

            // Регистрируем издателей
            ros.RegisterPublisher<TwistMsg>(cmdVelTopic);
            ros.RegisterPublisher<Int32Msg>(gripperTopic);
            ros.RegisterPublisher<Float32Msg>(cameraTopic);

            // Подписываемся на датчики
            ros.Subscribe<QuaternionMsg>(sensorTopic, SensorCallback);

            isConnected = true;
            Debug.Log($"[ROSBridge] Подключено к ROS {rosIPAddress}:{rosPort}");
        }
        catch (System.Exception e)
        {
            isConnected = false;
            Debug.LogError($"[ROSBridge] Ошибка инициализации: {e.Message}");
        }
    }

    // ===== Публичные методы для управления =====

    /// <summary>
    /// Публикует команду движения (линейная и угловая скорость).
    /// Значения нормализуются в диапазоне [-1; 1].
    /// </summary>
    public void PublishDrive(float linear, float angular)
    {
        if (!isConnected || ros == null) return;

        linear = Mathf.Clamp(linear, -1f, 1f);
        angular = Mathf.Clamp(angular, -1f, 1f);

        TwistMsg msg = new TwistMsg
        {
            linear = new Vector3Msg { x = linear * maxLinearSpeed },
            angular = new Vector3Msg { z = angular * maxAngularSpeed }
        };

        try
        {
            ros.Publish(cmdVelTopic, msg);
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[ROSBridge] Ошибка публикации cmd_vel: {e.Message}");
        }
    }

    /// <summary>
    /// Публикует команду для клешни.
    /// </summary>
    public void PublishGripper(int value)
    {
        if (!isConnected || ros == null) return;
        if (value == lastGripperValue) return; // предотвращаем дублирование

        lastGripperValue = value;
        try
        {
            ros.Publish(gripperTopic, new Int32Msg { data = value });
            Debug.Log($"[ROSBridge] Gripper: {value}");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[ROSBridge] Ошибка публикации gripper: {e.Message}");
        }
    }

    /// <summary>
    /// Устанавливает целевой угол камеры (диапазон -1..1).
    /// Движение будет плавным за счёт Update().
    /// </summary>
    public void PublishCamera(float angle)
    {
        if (!isConnected || ros == null) return;
        targetCameraAngle = Mathf.Clamp(angle, -1f, 1f);
    }

    /// <summary>
    /// Возвращает последние полученные данные с датчиков.
    /// Если данных ещё нет, возвращает null.
    /// </summary>
    public QuaternionMsg GetSensorData()
    {
        return lastSensorData;
    }

    /// <summary>
    /// Экстренная остановка всех движений.
    /// </summary>
    public void EmergencyStop()
    {
        PublishDrive(0f, 0f);
        PublishGripper(0);
        Debug.LogWarning("[ROSBridge] ⚠️ АВАРИЙНАЯ ОСТАНОВКА");
    }

    // ===== Приватные методы =====

    /// <summary>
    /// Прямая публикация текущего угла камеры (используется внутри для плавного движения).
    /// </summary>
    private void PublishCameraRaw(float angle)
    {
        if (!isConnected || ros == null) return;
        try
        {
            ros.Publish(cameraTopic, new Float32Msg(angle));
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[ROSBridge] Ошибка публикации camera: {e.Message}");
        }
    }

    /// <summary>
    /// Колбэк для получения данных с датчиков.
    /// </summary>
    private void SensorCallback(QuaternionMsg msg)
    {
        lastSensorData = msg;
        // Для отладки можно раскомментировать:
        // Debug.Log($"UZ: {msg.x * 100:F1} см, IR_L: {msg.y}, IR_R: {msg.z}, GripperIR: {msg.w}");
    }
}