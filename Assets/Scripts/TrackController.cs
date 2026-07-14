using UnityEngine;

public class TrackController : MonoBehaviour
{
    [Header("Movement Calibration")]
    [Tooltip("Базовая линейная скорость (м/с)")]
    [SerializeField] private float moveSpeed = 0.57f;
    
    [Tooltip("Базовая скорость поворота (град/с или безразмерный коэффициент)")]
    [SerializeField] private float turnSpeed = 120f;
    
    [Tooltip("Коэффициент влияния разворота на скорость гусениц")]
    [SerializeField] private float turnK = 0.30f;
    
    [Tooltip("Лимит поступательной скорости (м/с)")]
    [SerializeField] private float maxLinearCmd = 0.25f;

    [Header("Motor Settings (PWM)")]
    [Tooltip("Порог мертвой зоны в % PWM")]
    [SerializeField] private float motorDeadzone = 10f;
    
    [Tooltip("Минимальный порог старта моторов в % PWM")]
    [SerializeField] private float minMotorPwm = 35f;
    
    [Tooltip("Лимит изменения PWM за один физический тик (плавность)")]
    [SerializeField] private float maxPwmStep = 15f;
    
    [Tooltip("Масштабный коэффициент перевода скорости (м/с) в PWM")]
    [SerializeField] private float speedToPwmMultiplier = 200f;

    // Входные команды управления (диапазон [-1.0, 1.0])
    public float GasInput { get; set; }
    public float SteerInput { get; set; }

    // Выходные сглаженные значения PWM для моторов (диапазон [-100.0, 100.0])
    public float CurrentLeftPwm { get; private set; }
    public float CurrentRightPwm { get; private set; }

    // Расчетные физические скорости бортов (м/с), доступны для чтения
    public float LeftSpeed { get; private set; }
    public float RightSpeed { get; private set; }

    private void FixedUpdate()
    {
        CalculateTrackOutputs();
    }

    private void CalculateTrackOutputs()
    {
        // 1. Расчет и ограничение линейной скорости
        float targetLinearSpeed = GasInput * moveSpeed;
        targetLinearSpeed = Mathf.Clamp(targetLinearSpeed, -maxLinearCmd, maxLinearCmd);

        // 2. Расчет угловой составляющей скорости
        // Нюанс: если turnSpeed равен 120, прямое умножение (steer * 120 * 0.3) даст 36 м/с.
        // Для физической корректности переводим угловую скорость в радианы (или трактуем как масштаб).
        float angularSpeedRad = turnSpeed * Mathf.Deg2Rad;
        float targetAngularSpeed = SteerInput * angularSpeedRad * turnK;

        // 3. Смешивание скоростей (дифференциальный привод)
        LeftSpeed = targetLinearSpeed + targetAngularSpeed;
        RightSpeed = targetLinearSpeed - targetAngularSpeed;

        // 4. Перевод физической скорости (м/с) в сырой PWM
        float targetLeftPwmRaw = LeftSpeed * speedToPwmMultiplier;
        float targetRightPwmRaw = RightSpeed * speedToPwmMultiplier;

        // 5. Применение логики мертвой зоны и минимального PWM
        float targetLeftPwm = ProcessPwmDeadzoneAndLimits(targetLeftPwmRaw);
        float targetRightPwm = ProcessPwmDeadzoneAndLimits(targetRightPwmRaw);

        // 6. Ограничение скорости нарастания сигнала (темп разгона) за тик
        CurrentLeftPwm = Mathf.MoveTowards(CurrentLeftPwm, targetLeftPwm, maxPwmStep);
        CurrentRightPwm = Mathf.MoveTowards(CurrentRightPwm, targetRightPwm, maxPwmStep);
    }

    private float ProcessPwmDeadzoneAndLimits(float rawPwm)
    {
        float absPwm = Mathf.Abs(rawPwm);

        // Если сигнал меньше мертвой зоны — мотор стоит
        if (absPwm < motorDeadzone)
        {
            return 0f;
        }

        // Если выше — подтягиваем к minMotorPwm и ограничиваем максимумом в 100%
        float clampedPwm = Mathf.Clamp(absPwm, minMotorPwm, 100f);

        // Возвращаем исходное направление вращения (знак)
        return clampedPwm * Mathf.Sign(rawPwm);
    }
}