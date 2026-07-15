using UnityEngine;

public class VirtualSensors : MonoBehaviour
{
    [Header("Sensor Anchor Points")]
    [SerializeField] private Transform centerPoint;      // Точка УЗ-датчика (вперед)
    [SerializeField] private Transform leftIRPoint;       // Точка левого ИК
    [SerializeField] private Transform rightIRPoint;      // Точка правого ИК
    [SerializeField] private Transform gripperIRPoint;    // Точка ИК клешни

    [Header("Ultrasonic (US) Settings")]
    [Tooltip("Максимальное расстояние измерения УЗ-датчика (в метрах)")]
    [SerializeField] private float maxUSDistance = 5.0f;
    [Tooltip("Количество лучей в веере")]
    [SerializeField] private int usRayCount = 5;
    [Tooltip("Угол веера (конус обзора) в градусах")]
    [SerializeField] private float usFovDegrees = 30f;
    [Tooltip("Маска слоев препятствий (исключая робота и мяч)")]
    [SerializeField] private LayerMask obstacleLayerMask;

    [Header("IR Obstacle Settings")]
    [Tooltip("Дистанция обнаружения стен ИК-датчиками (0.15f = 15 см)")]
    [SerializeField] private float irObstacleDistance = 0.15f;

    [Header("IR Gripper Settings")]
    [Tooltip("Дистанция обнаружения мяча в клешне (0.08f = 8 см)")]
    [SerializeField] private float irGripperDistance = 0.08f;
    [Tooltip("Маска слоя мяча")]
    [SerializeField] private LayerMask ballLayerMask;

    // Публичные свойства для чтения результатов другими скриптами / ИИ
    public float USNormalizedDistance { get; private set; } = 1.0f; // 0 (вплотную) .. 1 (чисто)
    public float LeftIRObstacle { get; private set; } = 0f;         // 1 (стена) или 0 (чисто)
    public float RightIRObstacle { get; private set; } = 0f;        // 1 (стена) или 0 (чисто)
    public float GripperIRBallDetected { get; private set; } = 0f;  // 1 (мяч обнаружен) или 0 (пусто)

    private void FixedUpdate()
    {
        UpdateUltrasonicSensor();
        UpdateIRObstacleSensors();
        UpdateGripperSensor();
    }

    private void UpdateUltrasonicSensor()
    {
        if (centerPoint == null) return;

        float shortestDistance = maxUSDistance;
        float startAngle = -usFovDegrees / 2f;
        float angleStep = usFovDegrees / (usRayCount - 1);

        for (int i = 0; i < usRayCount; i++)
        {
            float currentAngle = startAngle + (i * angleStep);
            // Поворачиваем направление луча по оси Y относительно направления датчика
            Vector3 rayDirection = Quaternion.Euler(0, currentAngle, 0) * centerPoint.forward;

            if (Physics.Raycast(centerPoint.position, rayDirection, out RaycastHit hit, maxUSDistance, obstacleLayerMask))
            {
                // Игнорируем попадания в мяч (на всякий случай, если слой не отфильтровал)
                if (!hit.collider.CompareTag("TargetBall"))
                {
                    if (hit.distance < shortestDistance)
                    {
                        shortestDistance = hit.distance;
                    }
                }
            }

            // Визуализация лучей в редакторе (зеленый - пусто, красный - попадание)
            Debug.DrawRay(centerPoint.position, rayDirection * (hit.collider != null ? hit.distance : maxUSDistance), 
                hit.collider != null && !hit.collider.CompareTag("TargetBall") ? Color.red : Color.green);
        }

        // Нормализация: 0 (вплотную) до 1 (чисто)
        USNormalizedDistance = Mathf.Clamp01(shortestDistance / maxUSDistance);
    }

    private void UpdateIRObstacleSensors()
    {
        // Левый датчик
        LeftIRObstacle = CheckIRObstacle(leftIRPoint) ? 1f : 0f;

        // Правый датчик
        RightIRObstacle = CheckIRObstacle(rightIRPoint) ? 1f : 0f;
    }

    private bool CheckIRObstacle(Transform irPoint)
    {
        if (irPoint == null) return false;

        bool hitDetected = Physics.Raycast(irPoint.position, irPoint.forward, out RaycastHit hit, irObstacleDistance, obstacleLayerMask);

        // Визуализация
        Debug.DrawRay(irPoint.position, irPoint.forward * irObstacleDistance, hitDetected ? Color.red : Color.blue);

        return hitDetected;
    }

    private void UpdateGripperSensor()
    {
        if (gripperIRPoint == null) return;

        // Ищем только объекты на слое Ball в пределах заданной дистанции
        bool ballHit = Physics.Raycast(gripperIRPoint.position, gripperIRPoint.forward, out RaycastHit hit, irGripperDistance, ballLayerMask);
        
        if (ballHit && hit.collider.CompareTag("TargetBall"))
        {
            GripperIRBallDetected = 1f;
        }
        else
        {
            GripperIRBallDetected = 0f;
        }

        // Визуализация
        Debug.DrawRay(gripperIRPoint.position, gripperIRPoint.forward * irGripperDistance, GripperIRBallDetected > 0.5f ? Color.magenta : Color.yellow);
    }
}