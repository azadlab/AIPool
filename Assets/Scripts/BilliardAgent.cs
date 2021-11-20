#define USE_AUTOSHOTS

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
        var randomPosX = Random.Range(-100f, 100f);
        var randomPosZ = Random.Range(-30f, 30f);
        var PosY = 82.0f;

        if (c.gameObject.CompareTag("Pocket"))
        {
            Debug.Log(envController.name + ":White Ball in the pocket, Spitting out...");
            envController.ResolveEvent(Event.WhiteBallPocketed);
            transform.localPosition = new Vector3(randomPosX, PosY, randomPosZ);

        }
        if (c.gameObject.CompareTag("ball") && !envController.hasPlayedSound)
        {
            //audioPlayer.Play();
            envController.hasPlayedSound = true;
            envController.ResolveEvent(Event.HitBall);
        }
        if (c.gameObject.CompareTag("wall"))
        {
            Debug.Log(envController.name + ":White Ball out of boundary");
            transform.localPosition = new Vector3(randomPosX, PosY, randomPosZ);
            envController.ResolveEvent(Event.HitOutOfBounds);
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
                sensor.AddObservation(transform.localPosition.x);
                sensor.AddObservation(transform.localPosition.z);
            }
            else
            {
                sensor.AddObservation(ball.transform.localPosition.x);
                sensor.AddObservation(ball.transform.localPosition.z);
            }

        }
        // Adding Position of Pockets
        foreach (var pocket in Pockets)
        {

            sensor.AddObservation(pocket.transform.localPosition.x);
            sensor.AddObservation(pocket.transform.localPosition.z);

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

        /*if (init_shot && !envController.isMoving)
        {
            init_shot = false;
            Vector3 nball_pos = GetNearestBall();
            ray_angle = Mathf.Atan2((nball_pos.z - transform.localPosition.z), (nball_pos.x - transform.localPosition.x));
            ray_angle = -1 * ray_angle / Mathf.PI;
        }*/

#if USE_AUTOSHOTS
        if (!envController.isMoving)
        {
            Vector3 shot = CalculateShot();
            shot_velocity = shot.magnitude;
            shot = shot / shot.magnitude;
            ray_angle = Mathf.Atan2(shot.z, shot.x);
            ray_angle = -1 * ray_angle / Mathf.PI;
            //Debug.Log("Shot.x="+shot.x+",Shot.z="+shot.z);
            //Debug.Log("ray_angle="+ray_angle+",velocity="+shot_velocity);
        }
#endif

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


    public Vector3 CalculateShot()
    {
        List<Vector3> shots = new List<Vector3>();
        List<Vector3> pocket_lines = new List<Vector3>();
        List<float> angles = new List<float>();


        foreach (var pocket in Pockets)
        {
            Vector3 pocket_pos = pocket.transform.localPosition;
            Vector3 line = pocket_pos - transform.localPosition;
            pocket_lines.Add(line);  // Draw lines from cue ball to all the pockets

            float aux_angle = 0.0f, angle = Mathf.PI;
            Vector3 nearest_shot = Vector3.zero;
            foreach (var ball in Balls)
            {
                if (ball.gameObject.CompareTag("whiteBall"))
                    continue;
                if (envController.balls_in_pocket.Contains(ball.name))
                    continue;

                // Find which ball is the nearest to each pocket line
                Vector3 ball_pos = ball.transform.localPosition;
                Vector3 shot = ball_pos - transform.localPosition;
                float dir = Vector3.Dot(shot / shot.magnitude, line / line.magnitude);
                if (dir < 0)   //If Ball and Pocket are not in the same direction
                    continue;
                aux_angle = Mathf.Acos(dir);
                if (Mathf.Abs(aux_angle) < angle)
                {
                    angle = aux_angle;
                    nearest_shot = shot;
                }

            }
            shots.Add(nearest_shot);
            angles.Add(angle);
        }

        // Best shot is the one with the minimum angle from the pocket lines
        float min_angle = Mathf.PI;
        int idx = 0;
        for (int i = 0; i < shots.Count; i++)
        {
            if (Mathf.Abs(angles[i]) < min_angle)
            {
                min_angle = angles[i];
                idx = i;
            }
        }

        // Find the correction for spin
        float dir1 = Mathf.Atan2(shots[idx].z, shots[idx].x);
        float dir2 = Mathf.Atan2(pocket_lines[idx].z, pocket_lines[idx].x);

        if (dir1 > dir2 && (dir1 > 0))     //If Ball is Left of pocket line , it means it has to be hit little bit to the right side
            min_angle = -1f * min_angle;  //Unity has positive in Clockwise direction!
        if (dir1 > dir2 && (dir1 < 0))     //If Ball is left of pocket line
            min_angle = -1f * min_angle;
        Debug.Log("Ball Angle:"+(dir1*180)/Mathf.PI);
        Debug.Log("Line Angle:"+(dir2*180)/Mathf.PI);
        envController.status.text = "Taking Shot for Pocket "+(idx+1);
        Vector3 best_shot = shots[idx];
        best_shot = best_shot / best_shot.magnitude;
        float spin_angle = (min_angle * 180) / Mathf.PI;
        spin_angle = spin_angle / 10.0f;
        spin_angle = Mathf.Clamp(spin_angle, -3, 3);
        Debug.Log("Spin Angle=" + spin_angle);
        best_shot = Quaternion.AngleAxis(spin_angle, Vector3.up) * best_shot;
        Vector3 table_diag = Pockets[0].transform.localPosition - Pockets[3].transform.localPosition;
        float max_dist = table_diag.magnitude;
        
        
        float velocity = pocket_lines[idx].magnitude / max_dist;
        return velocity * (best_shot);

    }
}
