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

    public List<GameObject> Balls;
    public List<GameObject> Pockets;

    public AudioSource audioPlayer;
    void Start()
    {

    }

    public override void Initialize()
    {
        billiardSettings = FindObjectOfType<BilliardSettings>();
        behaviorParameters = gameObject.GetComponent<BehaviorParameters>();
        envController = table.GetComponent<BilliardEnvController>();
        Balls = envController.Balls;
        Pockets = envController.Pockets;
        audioPlayer = GetComponent<AudioSource>();

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
                //envController.ResetScene();
                envController.ResolveEvent(Event.WhiteBallPocketed);
            }
        }
        if (c.gameObject.CompareTag("ball") && !envController.hasPlayedSound)
        {
            //audioPlayer.Play();
            envController.hasPlayedSound = true;
            envController.ResolveEvent(Event.HitBall);
        }

    }


    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        var rot_angle = 180 * Mathf.Clamp(actionBuffers.ContinuousActions[0], -1f, 1f);
        var agentVelocity = 50f * Mathf.Clamp(actionBuffers.ContinuousActions[1], -1f, 1f);

        if (!envController.isMoving)
        {
            envController.isMoving = true;
            Vector3 newForward = Quaternion.AngleAxis(rot_angle, Vector3.up) * Vector3.right.normalized;
            agentRb.AddForce(agentVelocity * newForward, ForceMode.VelocityChange);

        }



    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Adding Position of Balls
        foreach (var ball in Balls)
        {

            if (ball.CompareTag("whiteBall"))
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
        // Adding Position of Pockets
        foreach (var pocket in Pockets)
        {

            sensor.AddObservation(pocket.transform.position.x);
            sensor.AddObservation(pocket.transform.position.z);

        }
    }

    public Vector3 GetNearestBall()
    {
        Vector3 nearest = Vector3.zero;
        float dist = 1000000.0f;
        float aux_dist = 0.0f;
        foreach (var ball in Balls)
        {
            if (ball.gameObject.CompareTag("whiteBall"))
                continue;
            var npos = ball.transform.localPosition;

            aux_dist = (npos - transform.localPosition).magnitude;
            if (aux_dist < dist)
            {
                dist = aux_dist;
                nearest = npos;
            }
        }
        return nearest;
    }

    void FireRay(float angle)
    {
        Vector3 shoot_dir = Quaternion.AngleAxis(angle, Vector3.up) * Vector3.right.normalized;
        Vector3 pos = transform.position;
        Ray ray = new Ray(pos, shoot_dir);

        RaycastHit hitData;


        if (Physics.Raycast(ray, out hitData))
        {
            lr.SetPosition(0, ray.origin);
            lr.SetPosition(1, hitData.point);
            //Debug.Log("Collided with:"+hitData.collider.tag);
        }
        if (envController.isMoving)
        {
            lr.SetPosition(1, ray.origin);
        }

        //Debug.DrawRay(ray.origin, ray.direction*100);

    }

    public float ray_angle = 0.5f;
    public float shot_velocity = 0.0f;
    public GameObject meter_arrow;

    public float meter_pos = 0.0f;
    public float meter_cur_pos = 0.0f;

    public float angle_step = 0.01f;
    public float force_step = 0.2f;
    public float meter_display_step = 20f;

    public bool init_shot = true;
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
        if (shot_velocity > 1) shot_velocity = 1;
        if (shot_velocity < 0) shot_velocity = 0;

        if (init_shot && !envController.isMoving)
        {
            init_shot = false;
            Vector3 nball_pos = GetNearestBall();
            ray_angle = Mathf.Atan2((nball_pos.z - transform.localPosition.z), (nball_pos.x - transform.localPosition.x));
            ray_angle = -1 * ray_angle / Mathf.PI;
        }

        if (ray_angle > 1)
            ray_angle = ray_angle - 2;
        if (ray_angle < -1)
            ray_angle = 2 + ray_angle;
        FireRay(ray_angle * 180);

        continuousActionsOut[0] = ray_angle;

        if (Input.GetKey(KeyCode.Space))
        {
            continuousActionsOut[1] = shot_velocity;
            init_shot = true;
        }


        if (meter_cur_pos < meter_pos) meter_cur_pos = meter_pos;
        if (meter_cur_pos > -1 * meter_pos) meter_cur_pos = -1 * meter_pos;

        Vector3 t_pos;
        if (meter_arrow != null)
        {
            t_pos = meter_arrow.transform.localPosition;
            t_pos.x = meter_cur_pos;
            meter_arrow.transform.localPosition = t_pos;
        }

    }
}
