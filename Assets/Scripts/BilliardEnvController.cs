using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using UnityEngine;


public enum ballType
{
    White = 0,
    Red = 1
}
public enum Event
{
    HitGoal = 0,
    HitOutOfBounds = 1
}

public class BilliardEnvController : MonoBehaviour
{
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

    public bool UseRandomAgentRotation = true;
    public bool UseRandomAgentPosition = true;
    public bool UseRandomBlockRotation = true;
    public bool UseRandomBlockPosition = true;

    private int m_NumberOfRemainingBlocks;

    private int m_ResetTimer;
    public int MaxEnvironmentSteps;
    BilliardSettings billiardSettings;

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

        //ResetScene();
    }

    public bool isMoving = false;
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
            if (isMoving && rbVelocity.magnitude < 0.1f)
            {
                //If All Balls have stopped Moving
                if(++ballCount == Balls.Count)
                    isMoving = false;
            }
            else if (!isMoving && rbVelocity.magnitude > 0.1f)
            {
                //Balls are moving
                isMoving = true;
                //bAgent.transform.position = new Vector3(0.7f,0.05f,0.5f);
                //bAgent.transform.eulerAngles = new Vector3(0,0,0);
                break;
            }
        }


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


    public void ResetStick()
    {
        int rot_angle = Random.Range(0, 360);
        bAgent.transform.Rotate(bAgent.transform.up * rot_angle);
        bAgent.transform.position = wball.transform.position + bAgent.transform.forward * -1f;
    }

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


                // end episode
                bAgent.EndEpisode();

                ResetScene();
                break;

            case Event.HitGoal:
                // blue wins
                bAgent.AddReward(1f);

                // turn floor blue
                StartCoroutine(GoalScoredSwapGroundMaterial(m_GroundMaterial, RenderersList, .5f));

                // end episode
                
                bAgent.EndEpisode();
                //ResetScene();
                break;
        }
    }

}
