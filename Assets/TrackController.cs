using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class TrackController : MonoBehaviour
{
    [Header("Настройки движения")]
    [Tooltip("Скорость вперед/назад (м/с)")]
    public float moveSpeed = 0.57f;

    [Tooltip("Скорость поворота (градусов/с)")]
    public float turnSpeed = 120f;

    [Header("Плавность")]
    [Range(0f, 0.95f)]
    [Tooltip("Инерция разгона (0 = мгновенно, 1 = бесконечно)")]
    public float smoothing = 0.05f;

    [Header("Sim-to-Real: Motor Pipeline")]
    [Tooltip("Коэффициент смешивания угловой скорости (0.30 - реальный робот)")]
    public float turnK = 0.30f;

    [Tooltip("Ограничение линейной скорости cmd (0.25 - реальный робот)")]
    public float maxLinearCmd = 0.25f;

    [Tooltip("Мертвая зона мотора в PWM (10% - реальный робот)")]
    public float motorDeadzone = 10f;

    [Tooltip("Минимальный PWM буст (35% - реальный робот)")]
    public float minMotorPwm = 35f;

    [Tooltip("Максимальный шаг PWM за тик 20мс (15 - реальный робот)")]
    public float maxPwmStep = 15f;

    [Header("Трение гусениц о пол")]
    [Tooltip("Настраивать физический материал коллайдера (трение гусениц). " +
             "linearDamping/angularDamping — это сопротивление среды, а НЕ трение о поверхность!")]
    public bool applyTrackFriction = true;
    [Tooltip("Статическое трение (сцепление при старте с места). Резиновые гусеницы ~0.9")]
    public float trackStaticFriction = 0.9f;
    [Tooltip("Динамическое трение (при движении). Обычно чуть ниже статического.")]
    public float trackDynamicFriction = 0.7f;
    [Tooltip("Упругость (отскок). Для гусениц практически нулевая.")]
    public float trackBounciness = 0.0f;

    private Rigidbody rb;
    private float targetLinear = 0f;
    private float targetAngular = 0f;
    
    // PWM значения левого/правого бортов (0-100)
    [HideInInspector] public float pwmLeft = 0f;
    [HideInInspector] public float pwmRight = 0f;

    void Start()
    {
        rb = GetComponent<Rigidbody>();

        // 1. Заморозка вращений по X и Z (оставляем Y для поворотов)
        rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
        
        // Рекомендуемые настройки для стабильности
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
        rb.linearDamping = 8f;
        rb.angularDamping = 10f;

        SetupTrackFriction();
    }

    /// <summary>
    /// Трение гусениц о пол. linearDamping/angularDamping гасят движение "как в воздухе"
    /// и НЕ моделируют сцепление с поверхностью — для этого нужен PhysicsMaterial на коллайдере.
    /// Без него робот проскальзывает при разгоне и заносит на поворотах не так, как реальный.
    /// </summary>
    private void SetupTrackFriction()
    {
        if (!applyTrackFriction) return;

        Collider col = GetComponent<Collider>();
        if (col == null) col = GetComponentInChildren<Collider>();
        if (col == null)
        {
            Debug.LogWarning("[TrackController] Коллайдер не найден — трение гусениц не настроено.");
            return;
        }

#if UNITY_6000_0_OR_NEWER
        var mat = new PhysicsMaterial("TrackFriction");
        mat.frictionCombine = PhysicsMaterialCombine.Average;
        mat.bounceCombine = PhysicsMaterialCombine.Minimum;
#else
        var mat = new PhysicMaterial("TrackFriction");
        mat.frictionCombine = PhysicMaterialCombine.Average;
        mat.bounceCombine = PhysicMaterialCombine.Minimum;
#endif
        mat.staticFriction = trackStaticFriction;
        mat.dynamicFriction = trackDynamicFriction;
        mat.bounciness = trackBounciness;

        col.material = mat;
    }

    /// <summary>
    /// Задает целевую скорость движения (от -1 до 1).
    /// </summary>
    public void Move(float linearInput, float angularInput)
    {
        targetLinear = linearInput;
        targetAngular = angularInput;
    }

    void FixedUpdate()
    {
        // 1. Масштабируем входы в физические скорости cmd_vel (как в ROSBridge)
        float lin_x = Mathf.Clamp(targetLinear * 0.5f, -maxLinearCmd, maxLinearCmd);
        float ang_z = targetAngular * 1.0f; // max angular velocity = 1.0 rad/s

        // 2. Смешивание скоростей гусениц
        float v_left = lin_x + (ang_z * turnK);
        float v_right = lin_x - (ang_z * turnK);

        // 3. Конвертация в PWM (коэффициент 200)
        float targetPwmL = v_left * 200f;
        float targetPwmR = v_right * 200f;

        // 4. Мягкий старт (maxPwmStep за тик 20мс)
        float deltaL = targetPwmL - pwmLeft;
        if (Mathf.Abs(deltaL) > maxPwmStep)
            pwmLeft += Mathf.Sign(deltaL) * maxPwmStep;
        else
            pwmLeft = targetPwmL;

        float deltaR = targetPwmR - pwmRight;
        if (Mathf.Abs(deltaR) > maxPwmStep)
            pwmRight += Mathf.Sign(deltaR) * maxPwmStep;
        else
            pwmRight = targetPwmR;

        // 5. Двухступенчатая мертвая зона (Deadzone и Boost)
        float absL = Mathf.Abs(pwmLeft);
        float effectiveL = 0f;
        if (absL >= motorDeadzone)
            effectiveL = absL < minMotorPwm ? minMotorPwm : absL;
        effectiveL *= Mathf.Sign(pwmLeft);

        float absR = Mathf.Abs(pwmRight);
        float effectiveR = 0f;
        if (absR >= motorDeadzone)
            effectiveR = absR < minMotorPwm ? minMotorPwm : absR;
        effectiveR *= Mathf.Sign(pwmRight);

        // 6. Совмещенная кинематика (вращение + смещение вперед)
        float physicalLinearSpeed = ((effectiveL + effectiveR) / 2f) / 100f * moveSpeed;
        float physicalAngularSpeed = ((effectiveL - effectiveR) / 2f) / 100f * turnSpeed;

        // Двигаем Rigidbody
        float yawDelta = physicalAngularSpeed * Time.fixedDeltaTime;
        Quaternion newRot = rb.rotation * Quaternion.Euler(0f, yawDelta, 0f);
        rb.MoveRotation(newRot);

        Vector3 move = transform.forward * physicalLinearSpeed;
        rb.MovePosition(rb.position + move * Time.fixedDeltaTime);
    }
}
