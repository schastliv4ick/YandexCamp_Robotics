using UnityEngine;

public class SimulatedYoloCamera : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private Camera targetCamera;
    [SerializeField] private Transform targetBall;

    [Header("Settings")]
    [SerializeField] private float maxViewDistance = 2f;
    [SerializeField] private float hFov = 40f;
    [SerializeField] private LayerMask obstacleLayerMask;
    [SerializeField] private string targetBallTag = "TargetBall";

    public float RelativeAngle { get; private set; }
    public float NormalizedDistance { get; private set; } = 1f;
    public bool IsBallVisible { get; private set; }

    private void Awake()
    {
        if (targetCamera == null) targetCamera = GetComponent<Camera>();
    }

    private void FixedUpdate()
    {
        if (targetBall == null || targetCamera == null)
        {
            IsBallVisible = false;
            RelativeAngle = 0f;
            NormalizedDistance = 1f;
            return;
        }

        Vector3 viewportPoint = targetCamera.WorldToViewportPoint(targetBall.position);
        float distance = Vector3.Distance(targetCamera.transform.position, targetBall.position);

        bool inFront = viewportPoint.z > 0f;
        bool inViewport = viewportPoint.x >= 0f && viewportPoint.x <= 1f && viewportPoint.y >= 0f && viewportPoint.y <= 1f;
        bool inRange = distance <= maxViewDistance;

        Vector3 dirToBall = targetBall.position - targetCamera.transform.position;
        Vector3 flatDir = Vector3.ProjectOnPlane(dirToBall, targetCamera.transform.up);
        bool inFov = Vector3.Angle(targetCamera.transform.forward, flatDir) <= hFov * 0.5f;

        bool notObstructed = true;
        if (Physics.Raycast(targetCamera.transform.position, dirToBall.normalized, out RaycastHit hit, distance, obstacleLayerMask))
            if (!hit.collider.CompareTag(targetBallTag)) notObstructed = false;

        IsBallVisible = inFront && inViewport && inRange && inFov && notObstructed;

        if (IsBallVisible)
        {
            RelativeAngle = (viewportPoint.x - 0.5f) * 2f;
            NormalizedDistance = Mathf.Clamp01(distance / maxViewDistance);
        }
        else
        {
            RelativeAngle = 0f;
            NormalizedDistance = 1f;
        }
    }
}
