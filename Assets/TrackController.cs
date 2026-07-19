using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class TrackController : MonoBehaviour
{
    [SerializeField] public float moveSpeed = 5f;
    [SerializeField] public float turnSpeed = 120f;
    [SerializeField] private float turnK = 0.30f;
    [SerializeField] private float maxLinearCmd = 0.25f;
    [SerializeField] private float motorDeadzone = 10f;
    [SerializeField] private float minMotorPwm = 35f;
    [SerializeField] private float maxPwmStep = 15f;
    [SerializeField] private float pwmScale = 200f;
    [SerializeField] private float acceleration = 4f;
    [SerializeField] private float deceleration = 6f;
    [SerializeField] private float turnAcceleration = 4f;
    [SerializeField] private float turnDeceleration = 6f;

    private Rigidbody rb;
    private float currentLinearVelocity;
    private float currentAngularVelocity;

    // ИИ или Heuristic будут записывать команды напрямую сюда
    public float GasInput { get; set; }
    public float SteerInput { get; set; }

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.linearDamping = 9f;
        rb.angularDamping = 9f;
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
    }

    private void FixedUpdate()
    {
        float steer = Mathf.Clamp(SteerInput, -1f, 1f);
        float gas = Mathf.Clamp(GasInput, -1f, 1f);

        float targetLinear = Mathf.Clamp(gas * maxLinearCmd, -maxLinearCmd, maxLinearCmd);
        float targetAngular = Mathf.Clamp(steer * turnK, -1f, 1f);

        float baselineMass = 2.5f;
        float massFactor = rb != null ? (baselineMass / rb.mass) : 1f;

        float effectiveAcceleration = acceleration * massFactor;
        float effectiveDeceleration = deceleration * massFactor;
        float effectiveTurnAcceleration = turnAcceleration * massFactor;
        float effectiveTurnDeceleration = turnDeceleration * massFactor;

        float linearAccelRate = Mathf.Abs(targetLinear) > 0.0001f ? effectiveAcceleration : effectiveDeceleration;
        float angularAccelRate = Mathf.Abs(targetAngular) > 0.0001f ? effectiveTurnAcceleration : effectiveTurnDeceleration;

        currentLinearVelocity = Mathf.MoveTowards(currentLinearVelocity, targetLinear, linearAccelRate * Time.fixedDeltaTime);
        currentAngularVelocity = Mathf.MoveTowards(currentAngularVelocity, targetAngular, angularAccelRate * Time.fixedDeltaTime);

        float leftCmd = Mathf.Clamp(currentLinearVelocity + currentAngularVelocity, -maxLinearCmd, maxLinearCmd);
        float rightCmd = Mathf.Clamp(currentLinearVelocity - currentAngularVelocity, -maxLinearCmd, maxLinearCmd);

        float leftPwm = ConvertCmdToPwm(leftCmd);
        float rightPwm = ConvertCmdToPwm(rightCmd);

        float leftSpeed = PwmToSpeed(leftPwm);
        float rightSpeed = PwmToSpeed(rightPwm);

        float linearSpeed = (leftSpeed + rightSpeed) * 0.5f;
        float angularSpeed = (leftSpeed - rightSpeed) * 0.5f;


        if (rb != null)
        {
            rb.linearVelocity = transform.forward * linearSpeed;
            float rotationDegreesPerSecond = angularSpeed * turnSpeed;
            rb.angularVelocity = transform.up * (rotationDegreesPerSecond * Mathf.Deg2Rad);
        }
        else
        {
            Vector3 movement = transform.forward * linearSpeed * Time.fixedDeltaTime;
            Quaternion rotationDelta = Quaternion.Euler(0f, angularSpeed * turnSpeed * Time.fixedDeltaTime, 0f);
            transform.position += movement;
            transform.rotation *= rotationDelta;
        }
    }

    private float ConvertCmdToPwm(float command)
    {
        if (Mathf.Abs(command) < 0.0001f) return 0f;
        float pwm = Mathf.Abs(command) * pwmScale;
        if (pwm < motorDeadzone) return 0f;
        if (pwm < minMotorPwm) pwm = minMotorPwm;
        return Mathf.Clamp(pwm, 0f, 100f) * Mathf.Sign(command);
    }

    private float PwmToSpeed(float pwm)
    {
        return (pwm / pwmScale) * moveSpeed;
    }
}