using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

[RequireComponent(typeof(Rigidbody))]
public class RobotBrain : Agent
{
    [Header("References")]
    [SerializeField] private TrackController trackController;
    [SerializeField] private GripperController gripperController;
    [SerializeField] private VirtualSensors virtualSensors;
    [SerializeField] private SimulatedYoloCamera yoloCamera;
    [SerializeField] private Transform cameraPivot;

    [Header("Target")]
    [SerializeField] private Transform targetBall;

    [Header("Settings")]
    [SerializeField] private float fallHeightThreshold = -1f;
    [SerializeField] private float cameraPivotMaxAngle = 45f;
    [SerializeField] private float cameraPivotSpeed = 60f;

    // === MVP-набор наград: сумма НЕЗАВИСИМЫХ слагаемых (не потенциальная функция). ===
    // Каждое слагаемое можно включить/выключить и прологировать отдельно — это осознанный
    // выбор в пользу простоты отладки, а не потенциал-based shaping (см. вариант в самом
    // низу файла, помеченный "ADVANCED / PHASE 2", если позже понадобится его теоретическая
    // гарантия сохранения оптимальной политики).
    [Header("1. Сближение с мячом (Distance Delta)")]
    [SerializeField] private float distanceRewardFar = 0.05f;   // награда за 1 метр сближения, если мяч далеко
    [SerializeField] private float distanceRewardNear = 0.15f;  // награда за 1 метр сближения, если мяч близко
    [SerializeField] private float nearDistanceThreshold = 0.5f; // порог "близко" в метрах

    [Header("2. Центрирование мяча перед захватом")]
    [SerializeField] private float centeringRewardScale = 0.02f;

    [Header("3. Плавность управления")]
    [SerializeField] private float actionRatePenalty = 0.01f;

    [Header("4. Дистанция до стен (УЗ, континуальный штраф)")]
    [SerializeField] private float obstaclePenaltyScale = 0.05f;
    [SerializeField] private float obstacleSafeDistance = 0.3f;

    [Header("5. Реальный контакт со стеной (ИК, ближний диапазон)")]
    [SerializeField] private float irCollisionPenalty = 0.02f;

    [Header("6. Задний ход")]
    [SerializeField] private float backwardPenalty = 0.01f;
    [SerializeField] private float backwardGasThreshold = -0.1f;

    [Header("7-8. Терминальные условия")]
    [SerializeField] private float successReward = 5.0f;
    [SerializeField] private float fallPenalty = 1.0f;

    private Rigidbody rb;
    private Vector3 startPosition;
    private Quaternion startRotation;

    private float prevDistanceToBall;
    private float prevGas;
    private float prevSteer;
    private float lastKnownBallAngle;
    private float timeSinceLastDetection;
    private float cameraPivotAngle;

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        startPosition = transform.position;
        startRotation = transform.rotation;

        // Частая причина "тренируется, но ничему не учится": забыли перетащить ссылку
        // в инспекторе, и часть наград тихо всегда равна нулю. Проверяем один раз при старте.
        if (targetBall == null) Debug.LogWarning("[RobotBrain] targetBall не назначен — награда за сближение всегда будет 0!");
        if (virtualSensors == null) Debug.LogWarning("[RobotBrain] virtualSensors не назначен — штрафы за препятствия работать не будут!");
        if (yoloCamera == null) Debug.LogWarning("[RobotBrain] yoloCamera не назначен — награда за центрирование всегда будет 0!");
        if (gripperController == null) Debug.LogWarning("[RobotBrain] gripperController не назначен — терминальная награда за захват не сработает!");
    }

    public override void OnEpisodeBegin()
    {
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        transform.SetPositionAndRotation(startPosition, startRotation);

        if (gripperController != null && gripperController.IsHolding)
        {
            gripperController.GripperCloseCommand = false;
            gripperController.ReleaseBall();
        }

        prevGas = 0f;
        prevSteer = 0f;
        lastKnownBallAngle = 0f;
        timeSinceLastDetection = 0f;
        cameraPivotAngle = 0f;

        if (targetBall != null)
            prevDistanceToBall = Vector3.Distance(transform.position, targetBall.position);
    }

    private void  CalculateRewards(float gas, float steer, float cameraSignal)
    {
        // --- Терминальные условия проверяем первыми и выходим сразу (return),
        //     чтобы на последнем шаге эпизода не намешивались другие награды
        //     поверх успеха/падения — так проще читать логи по эпизодам. ---

        // 7. Падение с арены
        if (transform.position.y < fallHeightThreshold)
        {
            AddReward(-fallPenalty);
            EndEpisode();
            return;
        }

        // 8. Успешный захват мяча
        if (gripperController != null && gripperController.IsHolding)
        {
            AddReward(successReward);
            EndEpisode();
            return;
        }

        // 1. Награда за сближение с мячом (Distance Delta).
        //    Порог nearDistanceThreshold переключает на более сильную награду —
        //    "дожать" последние сантиметры важнее, чем плыть издалека.
        if (targetBall != null)
        {
            float currentDistance = Vector3.Distance(transform.position, targetBall.position);
            float delta = prevDistanceToBall - currentDistance; // > 0, если стали ближе

            float rewardScale = currentDistance < nearDistanceThreshold ? distanceRewardNear : distanceRewardFar;
            AddReward(delta * rewardScale);

            prevDistanceToBall = currentDistance;
        }

        // 2. Бонус за центрирование мяча в кадре камеры (готовность к захвату)
        bool isballVisible = yoloCamera != null && yoloCamera.IsBallVisible;

        if (isballVisible)
        {
            AddReward(centeringRewardScale * (1f - Mathf.Abs(yoloCamera.RelativeAngle)));
        }
        else{
            if (lastKnownBallAngle > 0f && cameraSignal > 0.1f) 
            {
                AddReward(0.002f);
            }
            else if (lastKnownBallAngle < 0f && cameraSignal < -0.1f) 
            {
                AddReward(0.002f);
            }
        }

        // 3. Штраф за резкость управления (плавность газ/руль между шагами)
        float actionMagnitude = Mathf.Abs(gas - prevGas) + Mathf.Abs(steer - prevSteer);
        AddReward(-actionRatePenalty * actionMagnitude);

        // 4. Штраф за приближение к стене по УЗ-дальномеру.
        //    Континуальный (не "да/нет"): чем ближе к стене, тем больнее.
        //    Отдельной "награды за объезд" не нужно — как только робот отъехал,
        //    danger падает и штраф сам уменьшается до нуля на следующем шаге.
        if (virtualSensors != null)
        {
            float us = virtualSensors.USNormalizedDistance; // 0 = вплотную, 1 = чисто
            if (us < obstacleSafeDistance)
            {
                float danger = (obstacleSafeDistance - us) / obstacleSafeDistance; // 0..1
                AddReward(-obstaclePenaltyScale * danger);
            }
        }

        // 5. Штраф за реальный/почти реальный контакт со стеной по ближним ИК-датчикам
        if (virtualSensors != null && (virtualSensors.LeftIRObstacle > 0.5f || virtualSensors.RightIRObstacle > 0.5f))
        {
            AddReward(-irCollisionPenalty);
        }

        // 6. Небольшой штраф за движение задним ходом.
        //    Не запрещаем полностью (иногда нужно сдать назад от стены),
        //    просто не даём агенту выбрать "ехать назад" как основную стратегию.
        if (gas < backwardGasThreshold)
        {
            AddReward(-backwardPenalty);
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(virtualSensors != null ? virtualSensors.USNormalizedDistance : 1f);
        sensor.AddObservation(virtualSensors != null ? virtualSensors.LeftIRObstacle : 0f);
        sensor.AddObservation(virtualSensors != null ? virtualSensors.RightIRObstacle : 0f);
        sensor.AddObservation(virtualSensors != null ? virtualSensors.GripperIRBallDetected : 0f);

        bool ballVisible = yoloCamera != null && yoloCamera.IsBallVisible;
        if (ballVisible) lastKnownBallAngle = yoloCamera.RelativeAngle;

        sensor.AddObservation(ballVisible ? yoloCamera.RelativeAngle : 0f);
        sensor.AddObservation(yoloCamera != null ? yoloCamera.NormalizedDistance : 1f);
        sensor.AddObservation(lastKnownBallAngle);
        sensor.AddObservation(ballVisible ? 1f : 0f);
        sensor.AddObservation(cameraPivotMaxAngle > 0f ? cameraPivotAngle / cameraPivotMaxAngle : 0f);
        sensor.AddObservation(gripperController != null && gripperController.IsHolding ? 1f : 0f);

        Vector3 offset = transform.position - startPosition;
        sensor.AddObservation(offset.x);
        sensor.AddObservation(offset.z);
        sensor.AddObservation(Mathf.DeltaAngle(0f, transform.eulerAngles.y) / 180f);
        sensor.AddObservation(rb.linearVelocity.magnitude);

        if (ballVisible) timeSinceLastDetection = 0f;
        else timeSinceLastDetection += Time.fixedDeltaTime;
        sensor.AddObservation(timeSinceLastDetection);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float gas = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float steer = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        float cameraSignal = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);

        if (trackController != null)
        {
            trackController.GasInput = gas;
            trackController.SteerInput = steer;
        }

        cameraPivotAngle = Mathf.Clamp(cameraPivotAngle + cameraSignal * cameraPivotSpeed * Time.fixedDeltaTime,
            -cameraPivotMaxAngle, cameraPivotMaxAngle);
        if (cameraPivot != null)
            cameraPivot.localRotation = Quaternion.Euler(0f, cameraPivotAngle, 0f);

        int gripperCommand = actions.DiscreteActions[0];
        if (gripperController != null)
        {
            if (gripperCommand == 1) gripperController.GripperCloseCommand = true;
            else if (gripperCommand == 2) gripperController.GripperCloseCommand = false;
        }

        CalculateRewards(gas, steer, cameraSignal);
        prevGas = gas;
        prevSteer = steer;
    }
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var ca = actionsOut.ContinuousActions;
        ca[0] = Input.GetAxis("Vertical");
        ca[1] = Input.GetAxis("Horizontal");
        ca[2] = 0f;

        var da = actionsOut.DiscreteActions;
        da[0] = Input.GetKey(KeyCode.Space) ? 1 : (Input.GetKey(KeyCode.LeftShift) ? 2 : 0);
    }
}