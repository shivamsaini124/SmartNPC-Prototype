using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class NPCAgent : Agent
{
    public Transform target;
    public float moveSpeed = 5f;
    public float rotationSpeed = 200f;
    public float reachReward = 80.0f;  // INCREASED - make success very clear
    public float stepPenalty = -0.01f;
    public float outOfBoundsPenalty = -5.0f;  // INCREASED - make failure clear
    public float maxDistance = 10f;
    public float targetReachDistance = 1.5f;

    private Rigidbody rb;
    private float previousDistanceToTarget;

    [SerializeField] private Material winMaterial;
    [SerializeField] private Material loseMaterial;
    [SerializeField] private Renderer floorRenderer;

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
    }

    public override void OnEpisodeBegin()
    {
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        transform.rotation = Quaternion.identity;  // Reset rotation too!

        // Randomize positions
        float randomX = Random.Range(-4f, 4f);
        float randomZ = Random.Range(-4f, 4f);
        transform.localPosition = new Vector3(randomX, 1f, randomZ);

        float targetX = Random.Range(-4f, 4f);
        float targetZ = Random.Range(-4f, 4f);
        target.localPosition = new Vector3(targetX, 1f, targetZ);

        previousDistanceToTarget = Vector3.Distance(
            new Vector3(transform.localPosition.x, 0, transform.localPosition.z),
            new Vector3(target.localPosition.x, 0, target.localPosition.z)
        );
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // SIMPLIFIED: Only use relative direction and distance (5 values total)
        
        // 1. Direction to target in LOCAL SPACE (2 values: x, z only)
        Vector3 directionToTarget = target.localPosition - transform.localPosition;
        directionToTarget.y = 0; // Ignore height
        Vector3 localDirection = transform.InverseTransformDirection(directionToTarget.normalized);
        sensor.AddObservation(localDirection.x);  // Left/Right
        sensor.AddObservation(localDirection.z);  // Forward/Back
        
        // 2. Distance to target (1 value, normalized)
        float distance = Vector3.Distance(
            new Vector3(transform.localPosition.x, 0, transform.localPosition.z),
            new Vector3(target.localPosition.x, 0, target.localPosition.z)
        );
        sensor.AddObservation(distance / 12f);  // Normalize by max expected distance
        
        // 3. Agent's rotation velocity (1 value)
        sensor.AddObservation(rb.angularVelocity.y / 100f);  // Normalized rotation speed
        
        // 4. Agent's forward velocity (1 value)
        float forwardVelocity = Vector3.Dot(rb.linearVelocity, transform.forward);
        sensor.AddObservation(forwardVelocity / moveSpeed);  // Normalized
        
        // Total: 5 observations (much cleaner!)
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Get actions
        float moveAction = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float rotateAction = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);

        // SIMPLIFIED MOVEMENT - apply forces directly
        Vector3 moveForce = transform.forward * moveAction * moveSpeed;
        rb.linearVelocity = new Vector3(moveForce.x, rb.linearVelocity.y, moveForce.z);
        
        // Apply rotation
        float rotation = rotateAction * rotationSpeed * Time.fixedDeltaTime;
        transform.Rotate(0f, rotation, 0f);

        // Calculate distance (ignore Y-axis)
        float distanceToTarget = Vector3.Distance(
            new Vector3(transform.localPosition.x, 0, transform.localPosition.z),
            new Vector3(target.localPosition.x, 0, target.localPosition.z)
        );

        // MAIN REWARD: Strong reward for getting closer
        float distanceDelta = previousDistanceToTarget - distanceToTarget;
        AddReward(distanceDelta * 1.0f);  // INCREASED multiplier
        previousDistanceToTarget = distanceToTarget;

        // BONUS: Reward for facing target (helps with early training)
        Vector3 toTarget = (target.localPosition - transform.localPosition);
        toTarget.y = 0;
        toTarget.Normalize();
        float alignment = Vector3.Dot(transform.forward, toTarget);
        AddReward(alignment * 0.02f);  // Small bonus

        // SUCCESS
        if (distanceToTarget < targetReachDistance)
        {
            AddReward(reachReward);
            floorRenderer.material = winMaterial;
            EndEpisode();
        }

        // FAILURE
        if (Mathf.Abs(transform.localPosition.x) > maxDistance ||
            Mathf.Abs(transform.localPosition.z) > maxDistance)
        {
            AddReward(outOfBoundsPenalty);
            floorRenderer.material = loseMaterial;
            EndEpisode();
        }

        // Small step penalty
        AddReward(stepPenalty);
    }

    // Optional: For manual testing
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Vertical");
        continuousActionsOut[1] = Input.GetAxis("Horizontal");
    }
}