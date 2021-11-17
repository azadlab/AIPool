using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using System.Collections;
using System.Collections.Generic;

public class BilliardAgent : Agent
{
    public GameObject table;
    Rigidbody agentRb;
    BehaviorParameters behaviorParameters;
    public Team teamId;

    // To get ball's location for observations
    public GameObject wball;
    Rigidbody wballRb;

    //BilliardSettings billiardSettings;
    BilliardEnvController envController;

    float agentRot;

    EnvironmentParameters resetParams;
    BilliardSettings billiardSettings;
    public LineRenderer lr;
    public float laserWidth = 0.1f;
    public float laserMaxLength = 5f;

    public List<GameObject> Balls = new List<GameObject>();

    void Start()
    {
        envController = table.GetComponent<BilliardEnvController>();
    }

    public override void Initialize()
    {
        billiardSettings = FindObjectOfType<BilliardSettings>();
        behaviorParameters = gameObject.GetComponent<BehaviorParameters>();

        foreach (var ball in Balls)
        {
            Rigidbody ballRb = ball.GetComponent<Rigidbody>();
            if (ball.CompareTag("whiteBall"))
            {
                wball = ball;
                wballRb = ballRb;
            }

        }
        agentRb = GetComponent<Rigidbody>();

        Vector3[] initLaserPositions = new Vector3[2] { Vector3.zero, Vector3.zero };
        lr.SetPositions(initLaserPositions);
        lr.startWidth = laserWidth;
        lr.endWidth = laserWidth;

        meter_pos = meter_arrow.transform.localPosition.x;
        meter_cur_pos = meter_pos;
    }

    /// <summary>
    /// Moves  a rigidbody towards a position smoothly.
    /// </summary>
    /// <param name="targetPos">Target position.</param>
    /// <param name="rb">The rigidbody to be moved.</param>
    /// <param name="targetVel">The velocity to target during the
    ///  motion.</param>
    /// <param name="maxVel">The maximum velocity posible.</param>
    void MoveTowards(
        Vector3 targetPos, Rigidbody rb, float targetVel, float maxVel)
    {
        var moveToPos = targetPos - rb.worldCenterOfMass;
        var velocityTarget = Time.fixedDeltaTime * targetVel * moveToPos;
        if (float.IsNaN(velocityTarget.x) == false)
        {
            rb.velocity = Vector3.MoveTowards(rb.velocity, velocityTarget, maxVel);
        }
    }


    /// <summary>
    /// Called when agent collides with the ball
    /// </summary>
    void OnCollisionEnter(Collision c)
    {
        if (c.gameObject.CompareTag("Pocket"))
        {
            if (this.gameObject.CompareTag("whiteBall"))
            {
                Debug.Log("White Ball Pocketed");
                envController.ResetScene();
            }
        }


    }


    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        var rot_angle = 180 * Mathf.Clamp(actionBuffers.ContinuousActions[0], -1f, 1f);
        var agentVelocity = 50f * Mathf.Clamp(actionBuffers.ContinuousActions[1], -1f, 1f);

        if (!envController.isMoving)
        {
            //envController.ResetStick();
            //transform.Rotate(transform.up * rot_angle);

            //MoveTowards(wball.transform.position, agentRb, agentVelocity, 5);

            envController.isMoving = true;
            //apply the y rotation to the global 'forward' vector3 to get the
            //forward vector direction that our sphere is moving in
            Vector3 newForward = Quaternion.AngleAxis(rot_angle, Vector3.up) * Vector3.forward.normalized;
            agentRb.AddForce(agentVelocity * newForward, ForceMode.VelocityChange);

        }



    }

    public override void CollectObservations(VectorSensor sensor)
    {

        // Agent velocity (2 floats)
        //sensor.AddObservation(agentRb.velocity.x);
        //sensor.AddObservation(agentRb.velocity.z);


        foreach (var ball in Balls)
        {
            
            if(ball.CompareTag("whiteBall"))
            {
                sensor.AddObservation(transform.position.x);
                sensor.AddObservation(transform.position.z);   
            }
            else
            {
                sensor.AddObservation(ball.transform.position.x);
                sensor.AddObservation(ball.transform.position.z);
            }
            
        }
    }


    void FireRay(float angle)
    {
        Vector3 shoot_dir = Quaternion.AngleAxis(angle, Vector3.up) * Vector3.forward.normalized;
        Vector3 pos = transform.position;
        Ray ray = new Ray(pos, shoot_dir);

        RaycastHit hitData;

        
        
        if(Physics.Raycast(ray, out hitData))
        {
            lr.SetPosition(0, ray.origin);
            lr.SetPosition(1, hitData.point);
            //Debug.Log("Collided with:"+hitData.collider.tag);
        }
        if(envController.isMoving)
        {
            lr.SetPosition(1,ray.origin);
        }
       
        //Debug.DrawRay(ray.origin, ray.direction*100);
                
    }

    public float ray_angle  = 0.5f;
    public float shot_velocity  = 0.0f;
    public GameObject meter_arrow;

    public float meter_pos = 0.0f;
    public float meter_cur_pos = 0.0f;

    public float angle_step = 0.01f;
    public float force_step = 0.2f;
    public float meter_display_step = 3.2f;
    // For human controller
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;

        continuousActionsOut[0] = 0.0f;
        continuousActionsOut[1] = 0.0f;



        if (Input.GetKey(KeyCode.LeftArrow) || Input.GetKey(KeyCode.A))
        {
            // move left
            ray_angle -= angle_step;

        }

        if (Input.GetKey(KeyCode.RightArrow) || Input.GetKey(KeyCode.D))
        {
            // move right
            ray_angle += angle_step;
        }
        if (Input.GetKey(KeyCode.UpArrow) || Input.GetKey(KeyCode.W))
        {
            // Increase Velocity
            shot_velocity += force_step;
            meter_cur_pos += meter_display_step;
        }
        if (Input.GetKey(KeyCode.DownArrow) || Input.GetKey(KeyCode.S))
        {
            // Decrease Velocity
            shot_velocity -= force_step;
            meter_cur_pos -= meter_display_step;
        }
        if(shot_velocity>1) shot_velocity=1;
        if(shot_velocity<0) shot_velocity=0;

        if(ray_angle > 1)
            ray_angle = ray_angle - 2;
        if(ray_angle < -1)
            ray_angle = 2 + ray_angle;
        FireRay(ray_angle * 180);

        continuousActionsOut[0] = ray_angle;

        if (continuousActionsOut[0] != 0 && Input.GetKey(KeyCode.Space))
            continuousActionsOut[1] = shot_velocity;

        
        if (meter_cur_pos < meter_pos) meter_cur_pos = meter_pos;
        if (meter_cur_pos > -1*meter_pos) meter_cur_pos = -1*meter_pos;

        Vector3 t_pos;
        if (meter_arrow != null)
        {
            t_pos = meter_arrow.transform.localPosition;
            t_pos.x = meter_cur_pos;
            meter_arrow.transform.localPosition = t_pos;
        }

    }
}
