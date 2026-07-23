using UnityEngine;

/// <summary>
/// P2, Step 5: virtual sensors for the GFS-X.
///
/// Anchor points (empty GameObjects) on the robot:
///   CenterPoint     - forward, ultrasonic
///   LeftIRPoint     - left
///   RightIRPoint    - right
///   GripperIRPoint  - inside the gripper
/// </summary>
public class VirtualSensors : MonoBehaviour
{
    [Header("Anchor points (P2, Step 5)")]
    [SerializeField] private Transform centerPoint;
    [SerializeField] private Transform leftIRPoint;
    [SerializeField] private Transform rightIRPoint;
    [SerializeField] private Transform gripperIRPoint;

    [Header("Ultrasonic (HC-SR04)")]
    [Tooltip("Real sensor cone is about 30 degrees.")]
    [SerializeField] private float ultrasonicConeAngle = 30f;
    [Tooltip("Number of rays fanned across the cone.")]
    [SerializeField] private int ultrasonicRayCount = 5;
    [Tooltip("Максимальная дальность УЗ в метрах. Реальный датчик при отсутствии препятствий " +
             "выдаёт 5 м. Используется для нормализации в 0..1.")]
    [SerializeField] private float ultrasonicMaxDistance = 5f;
    [Tooltip("Дальность достоверных показаний (м). Дальше этого реальный датчик сильно шумит, " +
             "поэтому в симуляции за этим порогом показания считаем как 'чисто' (5 м).")]
    [SerializeField] private float ultrasonicReliableDistance = 2.5f;

    [Header("Obstacle IR (walls only, ~15 cm)")]
    [SerializeField] private float irObstacleDistance = 0.15f;

    [Header("Gripper IR (ball, ~7-8 cm)")]
    [SerializeField] private float gripperIRDistance = 0.08f;
    [SerializeField] private string targetBallTag = "TargetBall";

    // 0 = touching an obstacle, 1 = clear
    public float USNormalizedDistance { get; private set; } = 1f;
    public float LeftIRObstacle { get; private set; }
    public float RightIRObstacle { get; private set; }
    public float GripperIRBallDetected { get; private set; }

    void Awake()
    {
        if (centerPoint == null) Debug.LogWarning("[VirtualSensors] centerPoint is not assigned - the ultrasonic will always read 'clear'!");
        if (gripperIRPoint == null) Debug.LogWarning("[VirtualSensors] gripperIRPoint is not assigned - the gripper will never detect the ball!");
    }

    void FixedUpdate()
    {
        USNormalizedDistance = ReadUltrasonic();
        LeftIRObstacle = ReadObstacleIR(leftIRPoint, -transform.right);
        RightIRObstacle = ReadObstacleIR(rightIRPoint, transform.right);
        GripperIRBallDetected = ReadGripperIR();
    }

    /// <summary>
    /// Fans several rays across the ~30 degree cone and keeps the shortest hit.
    /// The ball is deliberately ignored: it is far too small for ultrasonic ranging,
    /// and treating it as an obstacle would make the agent flee its own target.
    /// </summary>
    private float ReadUltrasonic()
    {
        if (centerPoint == null) return 1f;

        float shortest = ultrasonicMaxDistance;
        float half = ultrasonicConeAngle * 0.5f;
        int rays = Mathf.Max(1, ultrasonicRayCount);

        for (int i = 0; i < rays; i++)
        {
            // Spread rays evenly from -half to +half.
            float t = rays == 1 ? 0.5f : (float)i / (rays - 1);
            float angle = Mathf.Lerp(-half, half, t);
            Vector3 dir = Quaternion.AngleAxis(angle, centerPoint.up) * centerPoint.forward;

            if (Physics.Raycast(centerPoint.position, dir, out RaycastHit hit,
                                ultrasonicMaxDistance, ~0, QueryTriggerInteraction.Ignore))
            {
                // Skip the ball - too small to echo.
                if (hit.collider.CompareTag(targetBallTag)) continue;
                if (hit.distance < shortest) shortest = hit.distance;
            }
        }

        // Реальный датчик достоверен только до ultrasonicReliableDistance. Дальше показания
        // тонут в помехах, и робот фактически видит "чисто" = максимальные 5 м.
        if (shortest > ultrasonicReliableDistance) shortest = ultrasonicMaxDistance;

        return Mathf.Clamp01(shortest / ultrasonicMaxDistance);
    }

    /// <summary>Single short ray. Returns 1 when a wall is found, 0 when the path is clear.</summary>
    private float ReadObstacleIR(Transform point, Vector3 direction)
    {
        if (point == null) return 0f;

        if (Physics.Raycast(point.position, direction, out RaycastHit hit,
                            irObstacleDistance, ~0, QueryTriggerInteraction.Ignore))
        {
            // Reacts to walls only, not to the ball.
            if (hit.collider.CompareTag(targetBallTag)) return 0f;
            return 1f;
        }
        return 0f;
    }

    /// <summary>Points into the gripper. Detects the ball at close range.</summary>
    private float ReadGripperIR()
    {
        if (gripperIRPoint == null) return 0f;

        if (Physics.Raycast(gripperIRPoint.position, gripperIRPoint.forward, out RaycastHit hit,
                            gripperIRDistance, ~0, QueryTriggerInteraction.Ignore))
        {
            if (hit.collider.CompareTag(targetBallTag)) return 1f;
        }
        return 0f;
    }

    void OnDrawGizmosSelected()
    {
        if (centerPoint != null)
        {
            Gizmos.color = Color.cyan;
            float half = ultrasonicConeAngle * 0.5f;
            int rays = Mathf.Max(1, ultrasonicRayCount);
            for (int i = 0; i < rays; i++)
            {
                float t = rays == 1 ? 0.5f : (float)i / (rays - 1);
                Vector3 dir = Quaternion.AngleAxis(Mathf.Lerp(-half, half, t), centerPoint.up) * centerPoint.forward;
                Gizmos.DrawRay(centerPoint.position, dir * ultrasonicMaxDistance);
            }
        }
        Gizmos.color = Color.red;
        if (leftIRPoint != null) Gizmos.DrawRay(leftIRPoint.position, -transform.right * irObstacleDistance);
        if (rightIRPoint != null) Gizmos.DrawRay(rightIRPoint.position, transform.right * irObstacleDistance);
        Gizmos.color = Color.green;
        if (gripperIRPoint != null) Gizmos.DrawRay(gripperIRPoint.position, gripperIRPoint.forward * gripperIRDistance);
    }
}
