using UnityEngine;

/// <summary>
/// Автономное прохождение полосы препятствий по регламенту финальной эстафеты.
/// Конечный автомат вместо нейросети — детерминированное, отлаживаемое поведение.
///
/// ФАЗЫ (по регламенту, Этап 2):
///   DRIVE_TO_BALL  — едем вдоль полигона, обходим 5 коробок алгоритмом BUG-0   (+20 за старт, +15 за доезд)
///   ALIGN_TO_BALL  — мяч виден: доворачиваем на него по YOLO
///   APPROACH_BALL  — медленный подъезд вплотную
///   GRAB           — закрываем клешню, ждём подтверждения удержания           (+20)
///   TURN_AROUND    — разворот на 180 градусов
///   RETURN         — едем обратно к старту, обходим коробки                    (+15)
///   FINISHED       — стоп в стартовой зоне                                     (+15 если мяч удержан)
///
/// ВАЖНО: этот скрипт и RobotBrain — взаимоисключающие источники команд.
/// Включён этот — RobotBrain (Agent) должен быть выключен, и наоборот.
///
/// ИК-датчики стоят под 45 градусов к переду робота — они смотрят вперёд-вбок
/// и используются для выбора стороны обхода препятствия.
/// </summary>
public class MissionController : MonoBehaviour
{
    public enum Phase
    {
        IDLE,
        DRIVE_TO_BALL,
        ALIGN_TO_BALL,
        APPROACH_BALL,
        GRAB,
        TURN_AROUND,
        RETURN,
        FINISHED
    }

    [Header("References")]
    [SerializeField] private TrackController track;
    [SerializeField] private VirtualSensors sensors;
    [SerializeField] private GripperController gripper;
    [SerializeField] private SimulatedYoloCamera yoloCamera;

    [Header("Запуск")]
    [Tooltip("Стартовать сразу при запуске сцены. Для соревнования — включить.")]
    public bool autoStart = false;
    [Tooltip("Клавиша ручного старта миссии (для тестов).")]
    public KeyCode startKey = KeyCode.G;

    [Header("Скорости (0..1, идут в TrackController)")]
    [Tooltip("Крейсерская скорость по прямой.")]
    [Range(0f, 1f)] public float cruiseSpeed = 0.6f;
    [Tooltip("Скорость при обходе препятствия.")]
    [Range(0f, 1f)] public float avoidSpeed = 0.4f;
    [Tooltip("Скорость подъезда к мячу (медленно, чтобы не сбить).")]
    [Range(0f, 1f)] public float approachSpeed = 0.25f;
    [Tooltip("Сила руления при обходе.")]
    [Range(0f, 1f)] public float avoidSteer = 0.8f;
    [Tooltip("Сила руления при доворотe на мяч.")]
    [Range(0f, 1f)] public float alignSteer = 0.5f;

    [Header("Пороги датчиков")]
    [Tooltip("Нормализованный УЗ, ниже которого считаем препятствие спереди. " +
             "При дальности УЗ 5 м: 0.08 = 40 см, 0.06 = 30 см, 0.04 = 20 см.")]
    [Range(0.01f, 0.5f)] public float frontObstacleThreshold = 0.08f;
    [Tooltip("Критическая близость — сдаём назад. 0.03 = 15 см.")]
    [Range(0.005f, 0.2f)] public float criticalObstacleThreshold = 0.03f;

    [Header("Мяч")]
    [Tooltip("|RelativeAngle| ниже которого считаем, что мяч по центру.")]
    [Range(0.02f, 0.5f)] public float ballCenteredAngle = 0.12f;
    [Tooltip("NormalizedDistance, ниже которой переходим к медленному подъезду.")]
    [Range(0.05f, 1f)] public float ballCloseDistance = 0.35f;

    [Header("Тайминги (секунды)")]
    [Tooltip("Сколько держать манёвр обхода после пропадания препятствия (проход мимо коробки).")]
    public float avoidCommitTime = 0.8f;
    [Tooltip("Длительность разворота на 180 градусов. ПОДОБРАТЬ ЗАМЕРОМ!")]
    public float turnAroundDuration = 2.5f;
    [Tooltip("Сколько держать клешню закрытой до подтверждения захвата.")]
    public float grabHoldTime = 1.5f;
    [Tooltip("Максимум времени на дорогу к мячу (страховка).")]
    public float driveTimeout = 90f;
    [Tooltip("Максимум времени на возврат (страховка).")]
    public float returnTimeout = 90f;
    [Tooltip("Ехать вперёд вслепую после потери мяча (мяч ушёл в слепую зону под бампером).")]
    public float blindApproachTime = 1.2f;

    [Header("Возврат")]
    [Tooltip("Сколько секунд ехать назад, прежде чем считать, что вернулись " +
             "(если нет ориентира). ПОДОБРАТЬ ЗАМЕРОМ!")]
    public float returnDriveTime = 25f;
    [Tooltip("Останавливаться, если УЗ показал стену прямо перед стартом.")]
    public bool stopOnWallAtReturn = true;

    [Header("Статус (только чтение)")]
    [SerializeField] private Phase phase = Phase.IDLE;
    [SerializeField] private string statusText = "Ожидание старта";

    public Phase CurrentPhase => phase;

    // --- Внутреннее состояние ---
    private float phaseTimer;          // время в текущей фазе
    private float avoidTimer;          // сколько ещё держать манёвр обхода
    private int avoidDirection;        // -1 = уходим влево, +1 = вправо, 0 = не обходим
    private float blindTimer;          // слепой подъезд к мячу
    private bool ballWasSeen;          // видели ли мяч хоть раз в этой фазе

    void Start()
    {
        if (track == null) track = GetComponent<TrackController>();
        if (sensors == null) sensors = GetComponent<VirtualSensors>();

        if (track == null) Debug.LogError("[Mission] TrackController не назначен — робот не поедет!");
        if (sensors == null) Debug.LogError("[Mission] VirtualSensors не назначен — обход не работает!");
        if (gripper == null) Debug.LogWarning("[Mission] GripperController не назначен — захвата не будет!");
        if (yoloCamera == null) Debug.LogWarning("[Mission] yoloCamera не назначена — мяч искать нечем!");

        if (autoStart) StartMission();
    }

    void Update()
    {
        if (Input.GetKeyDown(startKey))
        {
            if (phase == Phase.IDLE || phase == Phase.FINISHED) StartMission();
            else AbortMission();
        }
    }

    public void StartMission()
    {
        phase = Phase.DRIVE_TO_BALL;
        phaseTimer = 0f;
        avoidTimer = 0f;
        avoidDirection = 0;
        blindTimer = 0f;
        ballWasSeen = false;
        Debug.Log("[Mission] СТАРТ — фаза DRIVE_TO_BALL");
    }

    public void AbortMission()
    {
        phase = Phase.IDLE;
        Drive(0f, 0f);
        Debug.Log("[Mission] Прервано оператором");
    }

    void FixedUpdate()
    {
        if (track == null) return;

        phaseTimer += Time.fixedDeltaTime;

        switch (phase)
        {
            case Phase.IDLE:          Drive(0f, 0f); statusText = "Ожидание старта"; break;
            case Phase.DRIVE_TO_BALL: DoDriveToBall(); break;
            case Phase.ALIGN_TO_BALL: DoAlignToBall(); break;
            case Phase.APPROACH_BALL: DoApproachBall(); break;
            case Phase.GRAB:          DoGrab(); break;
            case Phase.TURN_AROUND:   DoTurnAround(); break;
            case Phase.RETURN:        DoReturn(); break;
            case Phase.FINISHED:      Drive(0f, 0f); statusText = "ФИНИШ"; break;
        }
    }

    // ================= ФАЗА 1: ДОРОГА К МЯЧУ =================
    private void DoDriveToBall()
    {
        // Мяч в кадре — переходим к доворотy (основной триггер прибытия в зону)
        if (BallVisible())
        {
            statusText = "Мяч замечен";
            SetPhase(Phase.ALIGN_TO_BALL);
            return;
        }

        // Страховка по времени
        if (phaseTimer > driveTimeout)
        {
            Debug.LogWarning("[Mission] Таймаут дороги к мячу — пробуем искать мяч на месте");
            SetPhase(Phase.ALIGN_TO_BALL);
            return;
        }

        NavigateBug("Едем к мячу");
    }

    // ================= BUG-0: движение вперёд с обходом =================
    /// <summary>
    /// BUG-0: едем прямо, при препятствии уходим в свободную сторону по ИК (45 град),
    /// держим манёвр ещё avoidCommitTime, чтобы объехать коробку целиком, затем
    /// возвращаемся на прямой курс.
    /// </summary>
    private void NavigateBug(string label)
    {
        float uz = sensors != null ? sensors.USNormalizedDistance : 1f;
        bool leftIR = sensors != null && sensors.LeftIRObstacle > 0.5f;
        bool rightIR = sensors != null && sensors.RightIRObstacle > 0.5f;

        // 1. Критическая близость — сдаём назад с доворотом
        if (uz < criticalObstacleThreshold)
        {
            int back = avoidDirection != 0 ? avoidDirection : (leftIR ? 1 : -1);
            Drive(-avoidSpeed, back * avoidSteer);
            statusText = $"{label}: слишком близко, назад";
            avoidTimer = avoidCommitTime;
            avoidDirection = back;
            return;
        }

        // 2. Препятствие спереди — выбираем сторону обхода
        if (uz < frontObstacleThreshold)
        {
            if (avoidDirection == 0)
            {
                // ИК под 45 град: если левый чист — уходим влево, иначе вправо.
                if (!leftIR) avoidDirection = -1;
                else if (!rightIR) avoidDirection = +1;
                else avoidDirection = -1; // оба заняты — произвольно влево
            }
            avoidTimer = avoidCommitTime;
            Drive(avoidSpeed, avoidDirection * avoidSteer);
            statusText = $"{label}: обход {(avoidDirection < 0 ? "влево" : "вправо")}";
            return;
        }

        // 3. Препятствие сбоку (задели ИК) — чуть отруливаем от него
        if (leftIR && !rightIR)
        {
            Drive(avoidSpeed, +avoidSteer * 0.6f);
            statusText = $"{label}: борт слева";
            return;
        }
        if (rightIR && !leftIR)
        {
            Drive(avoidSpeed, -avoidSteer * 0.6f);
            statusText = $"{label}: борт справа";
            return;
        }

        // 4. Докатываем манёвр обхода (коробка ещё сбоку, хоть УЗ её и не видит)
        if (avoidTimer > 0f)
        {
            avoidTimer -= Time.fixedDeltaTime;
            Drive(cruiseSpeed, avoidDirection * avoidSteer * 0.5f);
            statusText = $"{label}: докатываем обход";
            return;
        }

        // 5. Свободно — прямо
        avoidDirection = 0;
        Drive(cruiseSpeed, 0f);
        statusText = $"{label}: прямо";
    }

    // ================= ФАЗА 2: ДОВОРОТ НА МЯЧ =================
    private void DoAlignToBall()
    {
        if (!BallVisible())
        {
            // Мяч потерян: крутимся на месте, пытаясь его найти
            if (phaseTimer > 8f)
            {
                Debug.LogWarning("[Mission] Мяч не найден — возвращаемся к движению вперёд");
                SetPhase(Phase.DRIVE_TO_BALL);
                return;
            }
            Drive(0f, alignSteer); // медленный поиск вращением
            statusText = "Ищем мяч вращением";
            return;
        }

        ballWasSeen = true;
        float angle = yoloCamera.RelativeAngle;

        // Мяч по центру — едем к нему
        if (Mathf.Abs(angle) < ballCenteredAngle)
        {
            SetPhase(Phase.APPROACH_BALL);
            return;
        }

        // Доворачиваем на месте (не едем, чтобы не промахнуться)
        Drive(0f, Mathf.Sign(angle) * alignSteer);
        statusText = $"Доворот на мяч ({angle:F2})";
    }

    // ================= ФАЗА 3: ПОДЪЕЗД К МЯЧУ =================
    private void DoApproachBall()
    {
        // Клешня уже видит мяч — хватаем
        if (GripperSeesBall())
        {
            SetPhase(Phase.GRAB);
            return;
        }

        if (BallVisible())
        {
            blindTimer = 0f;
            float angle = yoloCamera.RelativeAngle;

            // Сильно ушёл вбок — возвращаемся к довороту
            if (Mathf.Abs(angle) > ballCenteredAngle * 2.5f)
            {
                SetPhase(Phase.ALIGN_TO_BALL);
                return;
            }

            // Едем медленно, подруливая на мяч
            Drive(approachSpeed, angle * alignSteer);
            statusText = $"Подъезд к мячу (dist {yoloCamera.NormalizedDistance:F2})";
            return;
        }

        // Мяч пропал из кадра — скорее всего ушёл в слепую зону под бампером.
        // Продолжаем медленно ползти вперёд ещё blindApproachTime.
        blindTimer += Time.fixedDeltaTime;
        if (blindTimer < blindApproachTime)
        {
            Drive(approachSpeed, 0f);
            statusText = "Слепой подъезд (мяч под бампером)";
            return;
        }

        // Не нашли — сдаём назад и пробуем заново
        if (blindTimer < blindApproachTime + 1.0f)
        {
            Drive(-approachSpeed, 0f);
            statusText = "Мяч потерян, сдаём назад";
            return;
        }

        blindTimer = 0f;
        SetPhase(Phase.ALIGN_TO_BALL);
    }

    // ================= ФАЗА 4: ЗАХВАТ =================
    private void DoGrab()
    {
        Drive(0f, 0f); // стоим намертво, пока хватаем

        if (gripper != null) gripper.GripperCloseCommand = true;

        statusText = "Захват мяча...";

        // Даём клешне время сомкнуться и проверяем удержание
        if (phaseTimer > grabHoldTime)
        {
            if (gripper != null && gripper.IsHolding)
            {
                Debug.Log("[Mission] Мяч захвачен! Разворот.");
                SetPhase(Phase.TURN_AROUND);
            }
            else
            {
                Debug.LogWarning("[Mission] Захват не удался — новая попытка");
                if (gripper != null) gripper.GripperCloseCommand = false;
                SetPhase(Phase.ALIGN_TO_BALL);
            }
        }
    }

    // ================= ФАЗА 5: РАЗВОРОТ =================
    private void DoTurnAround()
    {
        // Клешню держим закрытой всю обратную дорогу!
        if (gripper != null) gripper.GripperCloseCommand = true;

        Drive(0f, 1f); // разворот на месте
        statusText = $"Разворот ({phaseTimer:F1}/{turnAroundDuration:F1} с)";

        if (phaseTimer > turnAroundDuration) SetPhase(Phase.RETURN);
    }

    // ================= ФАЗА 6: ВОЗВРАТ =================
    private void DoReturn()
    {
        // Клешня закрыта — иначе потеряем +15 за возврат с мячом
        if (gripper != null) gripper.GripperCloseCommand = true;

        float uz = sensors != null ? sensors.USNormalizedDistance : 1f;

        // Доехали по времени
        if (phaseTimer > returnDriveTime)
        {
            Debug.Log("[Mission] Возврат по таймеру — финиш");
            SetPhase(Phase.FINISHED);
            return;
        }

        // Упёрлись в стену старта на приличной скорости пути — считаем, что дома
        if (stopOnWallAtReturn && phaseTimer > returnDriveTime * 0.5f && uz < criticalObstacleThreshold)
        {
            Debug.Log("[Mission] Стена старта — финиш");
            SetPhase(Phase.FINISHED);
            return;
        }

        // Страховка
        if (phaseTimer > returnTimeout)
        {
            SetPhase(Phase.FINISHED);
            return;
        }

        NavigateBug("Возврат");
    }

    // ================= Утилиты =================
    private void SetPhase(Phase next)
    {
        phase = next;
        phaseTimer = 0f;
        avoidTimer = 0f;
        avoidDirection = 0;
        Debug.Log($"[Mission] -> {next}");
    }

    private bool BallVisible()
    {
        return yoloCamera != null && yoloCamera.IsBallVisible;
    }

    private bool GripperSeesBall()
    {
        return sensors != null && sensors.GripperIRBallDetected > 0.5f;
    }

    /// <summary>Единственная точка записи команд — как в RobotBrain.</summary>
    private void Drive(float gas, float steer)
    {
        track.GasInput = Mathf.Clamp(gas, -1f, 1f);
        track.SteerInput = Mathf.Clamp(steer, -1f, 1f);
    }

    // Экранная подсказка при тестах
    void OnGUI()
    {
        GUI.Label(new Rect(10, 10, 600, 22), $"[{phase}] {statusText}");
        if (sensors != null)
        {
            GUI.Label(new Rect(10, 32, 600, 22),
                $"УЗ {sensors.USNormalizedDistance:F3}  ИК Л{sensors.LeftIRObstacle:F0} П{sensors.RightIRObstacle:F0}  " +
                $"клешня {sensors.GripperIRBallDetected:F0}  держит {(gripper != null && gripper.IsHolding ? "ДА" : "нет")}");
        }
        if (phase == Phase.IDLE) GUI.Label(new Rect(10, 54, 600, 22), $"Нажмите {startKey} для старта");
    }
}