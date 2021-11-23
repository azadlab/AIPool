//#define USE_AUTOSHOTS
#define USE_MANUAL

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
            audioPlayer.Play();
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
            #if !USE_MANUAL
                agentRb.AddForce(agentVelocity * newForward, ForceMode.VelocityChange);
            #endif

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

    public float ray_angle = 0.5f;
    public float shot_velocity = 0.0f;
    public GameObject meter_arrow;

    public float meter_pos = 0.0f;
    public float meter_cur_pos = 0.0f;

    public float angle_step = 0.01f;
    public float force_step = 0.2f;
    public float meter_display_step = 20f;

    public bool init_shot = true;
    public bool isShot = false;
    // For human controller

    public void HandleManualControl()
    {
        #if USE_MANUAL

         if (Input.GetKey(KeyCode.LeftArrow) || Input.GetKey(KeyCode.A))
        {
            // rotating left
            ray_angle -= angle_step;

        }

        if (Input.GetKey(KeyCode.RightArrow) || Input.GetKey(KeyCode.D))
        {
            // rotating right
            ray_angle += angle_step;

        }
        if (Input.GetKey(KeyCode.UpArrow) || Input.GetAxis("Mouse ScrollWheel") > 0f)
        {
            // Increase Velocity
            shot_velocity += force_step;
            meter_cur_pos += meter_display_step;
        }
        if (Input.GetKey(KeyCode.DownArrow) || Input.GetAxis("Mouse ScrollWheel") < 0f)
        {
            // Decrease Velocity
            shot_velocity -= force_step;
            meter_cur_pos -= meter_display_step;
        }
        if (shot_velocity > 1) shot_velocity = 1;
        if (shot_velocity < 0) shot_velocity = 0;

        Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
        RaycastHit hit;
        Physics.Raycast(ray, out hit);
        ray_angle = -1* Mathf.Atan2(hit.point.z - transform.position.z, hit.point.x - transform.position.x) / Mathf.PI;
        
        Vector3 shot_dir = Quaternion.AngleAxis(ray_angle * 180, transform.up) * transform.right.normalized;
        if(Input.GetAxis("Mouse X") != 0 || Input.GetAxis("Mouse Y")!=0)
        {
            shot_dist = 0.0f;
            shot_rays.Clear();
            ray_hits.Clear();
            
            FireShotRay(transform.position, shot_dir);
            DrawShot(true);    

        }
        
        if (ray_angle > 1)
            ray_angle = ray_angle - 2;
        if (ray_angle < -1)
            ray_angle = 2 + ray_angle;

        
        if (Input.GetKey(KeyCode.Space) || Input.GetMouseButtonDown(0))
        {
            DrawShot(false);    
            init_shot = true;
            isShot = true;
            agentRb.AddForce(100f * shot_velocity * shot_dir, ForceMode.VelocityChange);
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
#if USE_AUTOSHOTS
        if (init_shot && !envController.isMoving)
        {
            init_shot = false;
            /*Vector3 nball_pos = GetNearestBall();
            ray_angle = Mathf.Atan2((nball_pos.z - transform.localPosition.z), (nball_pos.x - transform.localPosition.x));
            ray_angle = -1 * ray_angle / Mathf.PI;*/
            
            Vector3 shot = FindBestShot();
            ray_angle = Mathf.Atan2(shot.z, shot.x);
            ray_angle = -1 * ray_angle / Mathf.PI;
        }
            /*Vector3 shot = CalculateShot();
            shot_velocity = shot.magnitude;
            shot = shot / shot.magnitude;
            ray_angle = Mathf.Atan2(shot.z, shot.x);
            ray_angle = -1 * ray_angle / Mathf.PI;*/
            //Debug.Log("Shot.x="+shot.x+",Shot.z="+shot.z);
            //Debug.Log("ray_angle="+ray_angle+",velocity="+shot_velocity);        
       
#endif
#endif

}

    
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = 0;
        continuousActionsOut[1] = 0;
        if(isShot)
        {
            isShot = false;
            continuousActionsOut[0] = ray_angle;
            continuousActionsOut[1] = shot_velocity;
        }
    }
    

    Graph graph;
    public Vector3 FindBestShot()
    {
        graph = new Graph();
        Vector3 shot_dir;
        float ray_angle = 0;
        Vector3 best_shot = Vector3.zero;
        for (ray_angle = 0; ray_angle < 360; ray_angle += 0.1f)  //Scan for shots
        {
            shot_dist = 0.0f;
            shot_rays.Clear();
            ray_hits.Clear();
            shot_dir = Quaternion.AngleAxis(ray_angle, transform.up) * transform.right.normalized;
            FireShotRay(transform.position, shot_dir);
        }
        Debug.Log("Graph has " + graph.Size + " Nodes");
        Node start = graph.GetNode("whiteball");
        List<Node> end_nodes = graph.GetAllNodes("pocket");

        if (start == null || end_nodes.Count < 1)
            return best_shot;

        SPath shot_path = new SPath();
        SPath best_path = new SPath();
        float dist = float.MaxValue;
        foreach (Node end in end_nodes)
        {
            shot_path = graph.GetShortestPath(start, end);
            if (shot_path.length < dist)
            {
                best_path = shot_path;
                dist = shot_path.length;
            }
        }
        Debug.Log("Shortest Path has " + best_path.nodes.Count + " Nodes and is of Length: " + best_path.length);
        Debug.Log("# of Pocket Nodes:" + end_nodes.Count);



        start = best_path.nodes[0];
        Vector3 target = best_path.nodes[1].Position;
        ray_angle = 180f * Mathf.Atan2(target.z - start.Position.z, target.x - start.Position.x) / Mathf.PI;
        shot_dir = Quaternion.AngleAxis(ray_angle, transform.up) * transform.right.normalized;
        shot_dist = 0.0f;
        shot_rays.Clear();
        ray_hits.Clear();
        FireShotRay(start.Position, shot_dir);
        return shot_dir;
    }


    private List<Ray> shot_rays = new List<Ray>();
    private List<RaycastHit> ray_hits = new List<RaycastHit>();
    private List<GameObject> shot_segments = new List<GameObject>();
    private float shot_dist = 0.0f;

    void FireShotRay(Vector3 pos, Vector3 shot_dir)
    {
        Ray ray = new Ray(pos, shot_dir);

        RaycastHit hit;

        if (shot_dist >= 80 || shot_rays.Count >= 5)
            return;

        if (Physics.Raycast(ray, out hit))
        {

            shot_dist += hit.distance;

            pos = hit.point;


            shot_rays.Add(ray);
            ray_hits.Add(hit);

            GenerateLinerenderer();


            if (hit.transform.name == "Boundary")
            {
                shot_dir = Vector3.Reflect(shot_dir, hit.normal);
                shot_dir.y = 0;
                FireShotRay(pos, shot_dir);
            }
            if (hit.transform.tag == "ball")
            {
                //FireShotRay(pos, ray.direction);
#if USE_AUTOSHOTS
                RegisterShot();
#endif
            }

        }


        //Debug.DrawRay(ray.origin, ray.direction*100);

    }

    public Vector3 FindNearestPocket(Vector3 pos, Vector3 shot_dir)
    {
        Ray ray = new Ray(pos, shot_dir);

        RaycastHit hit;

        if (Physics.Raycast(ray, out hit))
        {
            if (hit.transform.tag == "pocket_loft")   //Pocket is in the line of sight of shot
            {
                return hit.point;
            }
            else if (hit.transform.tag == "ball")
                pos = FindNearestPocket(pos + 0.5f * shot_dir, shot_dir);
            else   //Pocket is not in the line of sight, find the nearest Pocket
            {
                /*float dist = 1000000.0f, aux_dist = 0.0f;
                
                foreach (GameObject pocket in Pockets)
                {
                    aux_dist = (pocket.transform.position - hit.point).magnitude;
                    if(aux_dist < dist)
                    {
                        dist = aux_dist;
                        pos = pocket.transform.position;
                    }
                }*/
                return Vector3.zero;
            }
        }
        return pos;
    }

    private void RegisterShot()
    {
        Node n1, n2;
        int aux_count = 0;
        Ray ray = shot_rays[0];

        n1 = new Node("whiteball", ray.origin); //First element of shot is white ball

        for (int i = 0; i < ray_hits.Count; i++)
        {
            ray = shot_rays[i];
            if (i == ray_hits.Count - 1)   //Last element of hit is Red Ball
                n2 = new Node("ball", ray_hits[i].point);
            else  //The intermediate nodes are boundary nodes
                n2 = new Node(ray_hits[i].collider.tag + "-" + aux_count++, ray_hits[i].point);

            graph.AddEdge(n1, n2);
            n1 = n2;
        }
        Vector3 pocket_pos = FindNearestPocket(n1.Position + 0.5f * ray.direction, ray.direction);
        if (pocket_pos.magnitude > 0)
        {
            n2 = new Node("pocket", pocket_pos);
            graph.AddEdge(n1, n2);           //Add an edge from the ball to the nearest pocket
        }
    }
    private void DrawShot(bool isEnabled)
    {
        GameObject obj;
        for (int i = 0; i < shot_rays.Count; i++)
        {
            Ray ray = shot_rays[i];
            obj = shot_segments[i];
            lr = obj.GetComponent<LineRenderer>();
            lr.enabled = isEnabled;
            if(!envController.isMoving)
            {
                lr.enabled = isEnabled;
                lr.SetPosition(0, ray.origin);
                lr.SetPosition(1, ray_hits[i].point);
            }
            
            
        }
        for(int i=shot_rays.Count;i<shot_segments.Count;i++)
        {
            obj = shot_segments[i];
            lr = obj.GetComponent<LineRenderer>();
            lr.enabled = false;
        }
    }
    private void GenerateLinerenderer()
    {

        GameObject obj;
        if (shot_segments.Count < shot_rays.Count)
        {
            obj = new GameObject();
            obj.name = "ShotLine";
            obj.AddComponent<LineRenderer>();
            lr = obj.GetComponent<LineRenderer>();
            lr.startWidth = 0.5f;
            lr.endWidth = 0.5f;
            
            lr.material = new Material(Resources.Load<Material>("DottedLine"));
            lr.material.mainTextureScale = new Vector2(1f / lr.startWidth, 1.0f);
            lr.textureMode = LineTextureMode.Tile;
            lr.startColor = Color.green;
            lr.endColor = Color.green;
            lr.useWorldSpace = false;          
            lr.numCapVertices = 5;
            shot_segments.Add(obj);
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

            float aux_angle = 0.0f, angle = 0.0f, aux_dist = 0.0f, dist = 1000000.0f;
            Vector3 nearest_shot = Vector3.zero;
            bool found_shot = false;
            foreach (var ball in Balls)
            {
                if (ball.gameObject.CompareTag("whiteBall"))
                    continue;
                if (envController.balls_in_pocket.Contains(ball.name))
                    continue;

                // Find which ball is the nearest to each pocket
                Vector3 ball_pos = ball.transform.localPosition;
                Vector3 shot = ball_pos - transform.localPosition;
                float dir = Vector3.Dot(shot / shot.magnitude, line / line.magnitude);
                if (dir < 0)   //If Ball and Pocket are not in the same direction
                    continue;
                aux_angle = Mathf.Acos(dir);
                aux_dist = (pocket_pos - ball_pos).magnitude;

                if (aux_dist < dist)
                {
                    dist = aux_dist;
                    angle = aux_angle;
                    nearest_shot = shot;
                    found_shot = true;
                }

            }
            if (found_shot)
            {
                shots.Add(nearest_shot);
                angles.Add(angle);
            }
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
        //Debug.Log("Ball Angle:"+(dir1*180)/Mathf.PI);
        //Debug.Log("Line Angle:"+(dir2*180)/Mathf.PI);
        envController.status.text = "Taking Shot for Pocket " + (idx + 1);
        Vector3 best_shot = shots[idx];
        best_shot = best_shot / best_shot.magnitude;
        float spin_angle = (min_angle * 180) / Mathf.PI;
        spin_angle = spin_angle / 10.0f;
        spin_angle = Mathf.Clamp(spin_angle, -3, 3);
        Debug.Log("Spin Angle=" + spin_angle);
        //best_shot = Quaternion.AngleAxis(spin_angle, Vector3.up) * best_shot;
        Vector3 table_diag = Pockets[0].transform.localPosition - Pockets[3].transform.localPosition;
        float max_dist = table_diag.magnitude;


        float velocity = pocket_lines[idx].magnitude / max_dist;
        return velocity * (best_shot);

    }

}



