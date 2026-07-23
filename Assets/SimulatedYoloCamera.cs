using UnityEngine;

/// <summary>
/// P3, Step 2: Simulated YOLO detector.
///
/// The real robot finds the ball with an onboard camera + YOLO, which returns a
/// bounding box in image space. Rendering a real image in the simulator would
/// waste CPU, so instead we project the ball's 3D position into 2D viewport
/// coordinates with Camera.WorldToViewportPoint().
///
/// Outputs (consumed by RobotBrain.CollectObservations):
///   IsBallVisible      - ball inside hFOV, within range, not occluded by a wall
///   RelativeAngle      - horizontal offset from frame centre, -1 (left) .. +1 (right)
///   NormalizedDistance - 0 (touching) .. 1 (at maxVisionDistance)
///   CameraYaw          - current servo angle, normalized -1 .. +1
/// </summary>
[RequireComponent(typeof(Camera))]
public class SimulatedYoloCamera : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private Transform targetBall;

    [Header("Vision settings (P3, Step 2)")]
    [Tooltip("Horizontal field of view of the real camera, degrees.")]
    [SerializeField] private float horizontalFOV = 40f;
    [Tooltip("Максимальная дальность распознавания мяча, м. Реальная камера видит 2-3 м, " +
             "дальше начинаются сильные помехи. Используется для нормализации в 0..1.")]
    [SerializeField] private float maxVisionDistance = 2.5f;
    [Tooltip("Layers that block line of sight (walls). The ball itself must NOT be on these layers.")]
    [SerializeField] private LayerMask occlusionMask = ~0;

    [Header("Servo (camera pivot)")]
    [SerializeField] private float cameraYawMaxAngle = 45f;

    private Camera cam;

    // --- Public read-only outputs ---
    public bool IsBallVisible { get; private set; }
    public float RelativeAngle { get; private set; }
    public float NormalizedDistance { get; private set; } = 1f;
    public float CameraYaw
    {
        get
        {
            if (cameraYawMaxAngle <= 0f) return 0f;
            float yaw = transform.localEulerAngles.y;
            if (yaw > 180f) yaw -= 360f;
            return Mathf.Clamp(yaw / cameraYawMaxAngle, -1f, 1f);
        }
    }

    void Awake()
    {
        cam = GetComponent<Camera>();
        // The projection math below assumes the camera matches the real hFOV.
        cam.fieldOfView = Camera.HorizontalToVerticalFieldOfView(horizontalFOV, cam.aspect);

        if (targetBall == null)
            Debug.LogWarning("[SimulatedYoloCamera] targetBall is not assigned - the ball will never be visible!");
    }

    void FixedUpdate()
    {
        Detect();
    }

    private void Detect()
    {
        IsBallVisible = false;
        RelativeAngle = 0f;
        NormalizedDistance = 1f;

        if (targetBall == null || cam == null) return;

        // 1. Project the 3D ball position into 2D viewport coordinates (0..1 range).
        Vector3 viewport = cam.WorldToViewportPoint(targetBall.position);

        // z <= 0 means the ball is behind the camera plane.
        if (viewport.z <= 0f) return;

        // Outside the frame horizontally or vertically -> YOLO would not see it.
        if (viewport.x < 0f || viewport.x > 1f || viewport.y < 0f || viewport.y > 1f) return;

        // 2. Range check + normalization: 0 = touching, 1 = at max vision distance.
        float distance = Vector3.Distance(cam.transform.position, targetBall.position);
        if (distance > maxVisionDistance) return;

        // 3. Occlusion check: no wall between the camera and the ball.
        Vector3 direction = targetBall.position - cam.transform.position;
        if (Physics.Raycast(cam.transform.position, direction.normalized, out RaycastHit hit,
                            distance, occlusionMask, QueryTriggerInteraction.Ignore))
        {
            // Something was hit before the ball - line of sight is blocked.
            if (hit.transform != targetBall && !hit.transform.IsChildOf(targetBall))
                return;
        }

        // Ball confirmed visible - publish the detection.
        IsBallVisible = true;
        // Viewport x is 0..1 across the frame; remap so 0 = centre, -1 = left edge, +1 = right edge.
        RelativeAngle = Mathf.Clamp((viewport.x - 0.5f) * 2f, -1f, 1f);
        NormalizedDistance = Mathf.Clamp01(distance / maxVisionDistance);
    }
}
