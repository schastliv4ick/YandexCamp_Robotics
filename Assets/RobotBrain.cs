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

    private const float gamma = 0.99f; // sync gamma with config.yaml
    private float prevPotential;

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
            // prevDistanceToBall = Vector3.Distance(transform.position, targetBall.position);
            prevPotential = ComputeStatePotential();
    }

    private float ComputeStatePotential()
    {
        float phiGoal = 0f;
        if (targetBall != null)
        {
            float d = Vector3.Distance(transform.position, targetBall.position);
            phiGoal = -goalPotentialScale / (d + goalPotentialEps);
        }

        float phiAlign = 0f;
        if (yoloCamera != null && yoloCamera.IsBallVisible)
            phiAlign = alignPotentialScale * (1f - Mathf.Abs(yoloCamera.RelativeAngle));

        float phiObstacle = 0f;
        if (virtualSensors != null)
        {
            float us = virtualSensors.USNormalizedDistance;
            if (us < obstacleSafeDistance)
            {
                float danger = (obstacleSafeDistance - us) / obstacleSafeDistance;
                phiObstacle = -obstaclePotentialScale * danger * danger;
            }
        }

        return phiGoal + phiAlign + phiObstacle;
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

        CalculateRewards(gas, steer);
        prevGas = gas;
        prevSteer = steer;
    }

    // private void CalculateRewards(float gas, float steer)
    // {
    //     if (targetBall == null) return;

    //     float currentDistance = Vector3.Distance(transform.position, targetBall.position);

    //     if (currentDistance < prevDistanceToBall)
    //     {
    //         float rewardScale = currentDistance < nearDistanceThreshold ? distanceRewardNear : distanceRewardFar;
    //         AddReward(rewardScale);
    //     }

    //     float actionMagnitude = Mathf.Abs(gas - prevGas) + Mathf.Abs(steer - prevSteer);
    //     AddReward(-actionRatePenalty * actionMagnitude);

    //     bool ballVisible = yoloCamera != null && yoloCamera.IsBallVisible;
    //     if (ballVisible)
    //     {
    //         float relativeAngle = Mathf.Abs(yoloCamera.RelativeAngle);
    //         AddReward(centeringRewardScale * (1f - relativeAngle));
    //     }

    //     if (virtualSensors != null)
    //     {
    //         if (virtualSensors.LeftIRObstacle > 0.5f || virtualSensors.RightIRObstacle > 0.5f)
    //             AddReward(-obstaclePenalty);
    //     }

    //     if (transform.position.y < fallHeightThreshold)
    //     {
    //         AddReward(fallPenalty);
    //         EndEpisode();
    //     }

    //     if (gripperController != null && gripperController.IsHolding)
    //     {
    //         AddReward(successReward);
    //         EndEpisode();
    //     }

    //     prevDistanceToBall = currentDistance;
    // }

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
