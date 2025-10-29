using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class NPCAgent : Agent
{
    [Header("References")]
    public Transform target;
    
    [Header("Movement")]
    public float moveSpeed = 5f;
    public float rotationSpeed = 200f;

    private Rigidbody rb;

    private float previousDistanceToTarget;
    
    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
    }
    
    public override void OnEpisodeBegin()
    {
        // Reset agent
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        
        // Random start position
        float randomX = Random.Range(-4f, 4f);
        float randomZ = Random.Range(-4f, 4f);
        transform.localPosition = new Vector3(randomX, 1f, randomZ);
        
        // Random target position
        float targetX = Random.Range(-4f, 4f);
        float targetZ = Random.Range(-4f, 4f);
        target.localPosition = new Vector3(targetX, 0.5f, targetZ);

        previousDistanceToTarget = Vector3.Distance(transform.position, target.position);
        
    }
    
    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(target.position - transform.position);
        sensor.AddObservation(transform.forward);

        sensor.AddObservation(rb.linearVelocity);
        sensor.AddObservation(rb.angularVelocity);
    }
    
    public override void OnActionReceived(ActionBuffers actions)
    {
        // Get AI's decisions
        float moveForward = actions.ContinuousActions[0];
        float rotate = actions.ContinuousActions[1];
        
        // Apply rotation
        transform.Rotate(0f, rotate * rotationSpeed * Time.fixedDeltaTime, 0f);
        
        // Apply movement
        Vector3 move = transform.forward * moveForward * moveSpeed * Time.fixedDeltaTime;
        rb.MovePosition(transform.position + move);

        // Calculate distance to target
        float distanceToTarget = Vector3.Distance(transform.position, target.position);

        
        // SUCCESS: Reached target
        if (distanceToTarget < 1.5f)
        {
            AddReward(10.0f);
            Debug.Log("Reached Target: +10.0");
            EndEpisode();
        }

        // FAILURE: Went too far
        if (Mathf.Abs(transform.localPosition.x) > 10f ||
            Mathf.Abs(transform.localPosition.z) > 10f)
        {
            AddReward(-10.0f);
            Debug.Log("Lost: -1.0");
            EndEpisode();
        }

        // Reward for reducing distance
        float distanceDelta = previousDistanceToTarget - distanceToTarget;
        AddReward(distanceDelta * 1.0f);

        // Small time penalty
        AddReward(-0.001f);

        previousDistanceToTarget = distanceToTarget;
    }
    
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Manual control for testing
        ActionSegment<float> continuousActions = actionsOut.ContinuousActions;
        
        continuousActions[0] = Input.GetAxisRaw("Vertical");   // W/S
        continuousActions[1] = Input.GetAxisRaw("Horizontal"); // A/D
    }
}
