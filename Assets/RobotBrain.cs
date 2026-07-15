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
    [SerializeField] private Rigidbody targetBall;

    [Header("Settings")]
    [SerializeField] private float fallHeightThreshold = -1f;
    [SerializeField] private float cameraPivotMaxAngle = 45f;
    [SerializeField] private float cameraPivotSpeed = 60f;

    [Header("Rewards")]
    [SerializeField] private float goalPotentialScale = 0.3f;
    [SerializeField] private float goalPotentialEps = 0.3f;
    [SerializeField] private float alignPotentialScale = 0.1f;
    [SerializeField] private float obstaclePotentialScale = 0.5f;
    [SerializeField] private float obstacleSafeDistance = 0.3f;
    [SerializeField] private float actionRatePenalty = 0.01f;
    [SerializeField] private float irCollisionPenalty = 0.02f;
    [SerializeField] private float successReward = 5.0f;
    [SerializeField] private float fallPenalty = 1.0f;

    private int burstDropoutRemaining = 0;
    private float lastDetectionTime = 0;

    private const float gamma = 0.99f; // sync gamma with config.yaml
    private float prevPotential;

    private Rigidbody rb;
    private Vector3 startPosition;
    private Quaternion startRotation;

    private float prevDistanceToBall;
    private float prevGas;
    private float prevSteer;
    private float lastKnownBallAngle;
    private float cameraPivotAngle;

    private Vector3 startBallScale;
    private float startBallMass;
    private Vector3 startBallLocalPosition; 

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();

        startPosition = transform.position;
        startRotation = transform.rotation;

        if (targetBall != null)
        {
            startBallScale = targetBall.transform.localScale;
            startBallMass = targetBall.mass;
            startBallLocalPosition = targetBall.transform.localPosition;
        }
    }

    public void ResetBall()
    {
        if (targetBall == null) return;
        
        targetBall.transform.localPosition = startBallLocalPosition;
        targetBall.mass = startBallMass * (1.0f + UnityEngine.Random.Range(0.0f, 1.0f));
        targetBall.transform.localScale = startBallScale * (1.0f + UnityEngine.Random.Range(-0.2f, 0.2f));
        targetBall.linearVelocity = Vector3.zero;
        targetBall.angularVelocity = Vector3.zero;
    }


    public override void OnEpisodeBegin()
    {
        ResetBall();
        if (Academy.Instance.IsCommunicatorOn)
        {
            if (rb != null)
            {
                // Рандомизируем массу робота от 1.0кг до 4.0кг (базовый вес 2.5кг)
                rb.mass = UnityEngine.Random.Range(1.0f, 4.0f);
            }
            if (trackController != null)
            {
                // Меняем динамические характеристики двигателей
                trackController.moveSpeed = UnityEngine.Random.Range(0.3f, 0.7f);   // базовый м/с +-40%
                trackController.turnSpeed = UnityEngine.Random.Range(80f, 160f);    // скорость вращения
                trackController.smoothing = UnityEngine.Random.Range(0.01f, 0.25f); // инерция привода
            }
        }

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
        lastDetectionTime = Time.time;
        cameraPivotAngle = 0f;

        if (targetBall != null)
            // prevDistanceToBall = Vector3.Distance(transform.position, targetBall.position);
            prevPotential = ComputeStatePotential();
    }

    private float ComputeStatePotential()
    {
        float phiGoal = 0f;
        if (targetBall != null)
        {
            float d = Vector3.Distance(transform.position, targetBall.position);
            phiGoal = goalPotentialScale / (d + goalPotentialEps);
        }

        float phiAlign = 0f;
        if (yoloCamera != null && yoloCamera.IsBallVisible)
            phiAlign = alignPotentialScale * (1f - Mathf.Abs(yoloCamera.RelativeAngle));

        return phiGoal + phiAlign;
    }

    private void CalculateRewards(float gas, float steer)
    {
        float currentPotential = ComputeStatePotential();
        AddReward(gamma * currentPotential - prevPotential);
        prevPotential = currentPotential;

        float actionMagnitude = Mathf.Abs(gas - prevGas) + Mathf.Abs(steer - prevSteer);
        AddReward(-actionRatePenalty * actionMagnitude);

        if (virtualSensors != null && (virtualSensors.LeftIRObstacle > 0.5f || virtualSensors.RightIRObstacle > 0.5f))
            AddReward(-irCollisionPenalty);

        if (transform.position.y < fallHeightThreshold)
        {
            AddReward(-fallPenalty);
            EndEpisode();
            return;
        }

        if (gripperController != null && gripperController.IsHolding)
        {
            AddReward(successReward);
            EndEpisode();
        }
        if (virtualSensors != null && virtualSensors.USNormalizedDistance < obstacleSafeDistance)
        {
            float danger = (obstacleSafeDistance - virtualSensors.USNormalizedDistance) / obstacleSafeDistance;
            AddReward(-obstaclePotentialScale * danger * danger * Time.fixedDeltaTime);
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if (virtualSensors == null || yoloCamera == null || gripperController == null) return;

        // 1. УЛЬТРАЗВУК С ШУМОМ
        // Подмешиваем случайную погрешность в пределах +-5% к нормализованной дистанции
        float noiseUS = Academy.Instance.IsCommunicatorOn ? UnityEngine.Random.Range(-0.05f, 0.05f) : 0f;
        float noisyDistance = Mathf.Clamp01(virtualSensors.USNormalizedDistance + noiseUS);
        sensor.AddObservation(noisyDistance); // 0 (УЗ дальномер с шумом)
        
        // Боковые ИК датчики оставляем бинарными (1 или 0)
        sensor.AddObservation(virtualSensors.LeftIRObstacle);  // 1
        sensor.AddObservation(virtualSensors.RightIRObstacle); // 2
        sensor.AddObservation(virtualSensors.GripperIRBallDetected); // 3

        // 2. СИМУЛЯЦИЯ ПОТЕРЬ КАДРОВ YOLO (Burst Dropout)
        // Если счетчик активен — уменьшаем его на 1 за каждый физический тик
        if (burstDropoutRemaining > 0) burstDropoutRemaining--;
        // Если робот крутится на месте быстрее 0.5 рад/с — с шансом 15% активируем слепую зону на 5-15 шагов
        else if (Academy.Instance.IsCommunicatorOn && rb != null && rb.angularVelocity.magnitude > 0.5f)
        {
            if (UnityEngine.Random.value < 0.15f) burstDropoutRemaining = UnityEngine.Random.Range(5, 16);
        }

        // Переопределяем видимость мяча с учетом симулированного лага камеры
        bool ballVisible = yoloCamera.IsBallVisible && !(burstDropoutRemaining > 0);
        if (ballVisible) 
        {
            lastDetectionTime = Time.time;
            lastKnownBallAngle = yoloCamera.RelativeAngle;
        }
        
        // Отправляем данные камеры в нейросеть
        sensor.AddObservation(ballVisible ? yoloCamera.RelativeAngle : 0f);  // 4 (угол до мяча)
        sensor.AddObservation(ballVisible ? yoloCamera.NormalizedDistance : 1f); // 5 (дистанция до мяча)
        sensor.AddObservation(lastKnownBallAngle);                       // 6
        sensor.AddObservation(ballVisible ? 1.0f : 0.0f);                       // 7
        sensor.AddObservation(cameraPivotMaxAngle > 0f ? cameraPivotAngle / cameraPivotMaxAngle : 0f); // 8
        
        // Оставшиеся наблюдения (состояние клешни, смещение, одометрия) отправляем без изменений
        sensor.AddObservation(gripperController.IsHolding ? 1f : 0f); // 9
        
        Vector3 offset = transform.position - startPosition;
        // TODO верно что дальше 5и метров не уедем?
        const float maxOffset = 5f;
        sensor.AddObservation(Mathf.Clamp(offset.x / maxOffset, -1f, 1f));               // 10
        sensor.AddObservation(Mathf.Clamp(offset.z / maxOffset, -1f, 1f));               // 11
        sensor.AddObservation(transform.eulerAngles.y / 360f);                          // 12
        sensor.AddObservation(Mathf.Clamp01(rb.linearVelocity.magnitude / 0.5f));        // 13
        
        float timeSinceLastDetection = Time.time - lastDetectionTime;;
        sensor.AddObservation(timeSinceLastDetection);                                           // 14
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

        CalculateRewards(gas, steer);
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
