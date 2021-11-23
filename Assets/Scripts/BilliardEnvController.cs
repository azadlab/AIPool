using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using UnityEngine;
 using UnityEngine.UI;


public enum ballType
{
    White = 0,
    Red = 1
}
public enum Event
{
    HitPocket = 0,

    HitBall = 1,
    HitOutOfBounds = 2,
    WhiteBallPocketed = 3,
    LeftPocket = 4,
    EndGame = 5
}



public class BilliardEnvController : MonoBehaviour
{
    public int BallsInPockets=0;
    public BilliardAgent bAgent;
    Rigidbody bAgentRb;
    public GameObject wball;
    public Rigidbody wballRb;

    /// <summary>
    /// The ground. The bounds are used to spawn the elements.
    /// </summary>
    public GameObject play_surface;

    
    public Bounds areaBounds;

    Material m_GroundMaterial; //cached on Awake()

    /// <summary>
    /// We will be changing the ground material based on success/failue
    /// </summary>
    Renderer m_GroundRenderer;

    List<Renderer> RenderersList = new List<Renderer>();
    public List<GameObject> Balls = new List<GameObject>();
    public List<GameObject> Pockets = new List<GameObject>();

    public bool UseRandomAgentRotation = true;
    public bool UseRandomAgentPosition = true;
    public bool UseRandomBlockRotation = true;
    public bool UseRandomBlockPosition = true;

    private int m_NumberOfRemainingBlocks;

    private int m_ResetTimer;
    public int MaxEnvironmentSteps;
    BilliardSettings billiardSettings;
    public List<string> balls_in_pocket;

    public Text status;
    public static bool IsTraining = false;
    private float score = 0.0f;

    void Start()
    {

        // Get the ground's bounds
        areaBounds = play_surface.GetComponent<Collider>().bounds;
        // Get the ground renderer so we can change the material when a goal is scored
        m_GroundRenderer = play_surface.GetComponent<Renderer>();
        // Starting material
        m_GroundMaterial = m_GroundRenderer.material;
        billiardSettings = FindObjectOfType<BilliardSettings>();

        wballRb = wball.GetComponent<Rigidbody>();
        balls_in_pocket = new List<string>();
        bAgentRb = bAgent.GetComponent<Rigidbody>();
        if(IsTraining)
            status = GameObject.Find("TrainingStatus/"+this.name).GetComponent<Text>();
        else
            status = GameObject.Find("StatusBoard/status").GetComponent<Text>();
    
        
    }

    public bool isMoving = false;
    public bool hasPlayedSound = false;
    void FixedUpdate()
    {
        m_ResetTimer += 1;
        if (m_ResetTimer >= MaxEnvironmentSteps && MaxEnvironmentSteps > 0)
        {
            bAgent.EpisodeInterrupted();
            //ResetScene();
        }
        int ballCount = 0;
        foreach (var ball in Balls)
        {
            Rigidbody rb;
            
            if(ball.CompareTag("whiteBall"))
                rb = bAgent.GetComponent<Rigidbody>();
            else
                rb = ball.GetComponent<Rigidbody>();
            
            var rbVelocity = rb.velocity;
            if (isMoving && rbVelocity.magnitude < 0.3f)
            {
                //If All Balls have stopped Moving
                if(++ballCount == Balls.Count)
                    {
                        isMoving = false;
                        
                    }
            }
            else if (!isMoving && rbVelocity.magnitude > 0.3f)
            {
                //Balls are moving
                isMoving = true;
                break;
            }
        }
        if(!isMoving && ballCount>=Balls.Count)  //Resetting the velocity and orientation when stopped
        {
            bAgentRb.velocity = Vector3.zero;
            bAgentRb.angularVelocity = Vector3.zero;
            bAgentRb.transform.rotation = Quaternion.identity;
        }

        if(!isMoving && hasPlayedSound)
            hasPlayedSound = false;

        if(BallsInPockets >= Balls.Count-1)
            ResolveEvent(Event.EndGame);

        //status.text = this.name+":Balls="+(Balls.Count-BallsInPockets-1)+",Score="+10*BallsInPockets;
        status.text = "Balls="+(Balls.Count-BallsInPockets-1)+",Score="+10*BallsInPockets;
        //bAgent.transform.rotation = Quaternion.Euler(0,0,0);
    }

    void Update()
    {
        if(!isMoving)
            bAgent.HandleManualControl();
        if(Input.GetKeyDown(KeyCode.Escape))
            Application.Quit();
    }

    /// <summary>
    /// Changes the color of the ground for a moment.
    /// </summary>
    /// <returns>The Enumerator to be used in a Coroutine.</returns>
    /// <param name="mat">The material to be swapped.</param>
    /// <param name="time">The time the material will remain.</param>
    IEnumerator GoalScoredSwapGroundMaterial(Material mat, List<Renderer> rendererList, float time)
    {
        foreach (var renderer in rendererList)
        {
            renderer.material = mat;
        }

        yield return new WaitForSeconds(time); // wait for 2 sec

        foreach (var renderer in rendererList)
        {
            renderer.material = billiardSettings.defaultMaterial;
        }

    }


    /*public void ResetStick()
    {
        int rot_angle = Random.Range(0, 360);
        bAgent.transform.Rotate(bAgent.transform.up * rot_angle);
        bAgent.transform.position = wball.transform.position + bAgent.transform.forward * -1f;
    }*/

    private int resetTimer;
    /// <summary>
    /// Reset agent and ball spawn conditions.
    /// </summary>
    public void ResetScene()
    {
        resetTimer = 0;

        foreach (var ball in Balls)
        {
            var randomPosX = Random.Range(-100f, 100f);
            var randomPosZ = Random.Range(-30f, 30f);
            var PosY = 82.0f;
            if(ball.CompareTag("whiteBall"))
           {
               bAgent.transform.localPosition = new Vector3(randomPosX, PosY, randomPosZ);
           }
           else
               ball.transform.localPosition = new Vector3(randomPosX, PosY, randomPosZ);
           
        }
        BallsInPockets = 0;
        balls_in_pocket = new List<string>();
        score = 0;
        //ResetStick();

    }

    /// <summary>
    /// Resolves scenarios when ball enters a trigger and assigns rewards
    /// </summary>
    public void ResolveEvent(Event triggerEvent)
    {
        switch (triggerEvent)
        {
            case Event.HitOutOfBounds:

                bAgent.AddReward(-1f);
                score-=1f;
                bAgent.EndEpisode();
            break;

            case Event.HitPocket:
                // scored
                bAgent.AddReward(1f);
                score+=1f;
                bAgent.EndEpisode();
            break;
            case Event.HitBall:
                // Encourage to hit the ball
                bAgent.AddReward(0.01f);
                score+=0.01f;
                bAgent.EndEpisode();
            break;
            case Event.EndGame:
                Debug.Log("Game Ended for "+this.name);
                bAgent.EndEpisode();
                ResetScene();
            break;
            case Event.WhiteBallPocketed:
                bAgent.AddReward(-0.5f);
                score-=0.5f;
                bAgent.EndEpisode();
            break;
            case Event.LeftPocket:
                bAgent.AddReward(-1f);
                score-=1f;
                bAgent.EndEpisode();
            break;
        }
    }

}
