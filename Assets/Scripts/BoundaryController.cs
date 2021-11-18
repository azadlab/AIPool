using UnityEngine;

public class BoundaryController : MonoBehaviour
{
    public GameObject bAgent;
    public BilliardEnvController envController;
    void Start()
    {

        envController = GetComponentInParent<BilliardEnvController>();
    }


    /// <summary>
    /// Detects whether the ball lands out of bounds
    /// </summary>
    void OnTriggerEnter(Collider other)
    {
        var randomPosX = Random.Range(-100f, 100f);
        var randomPosZ = Random.Range(-30f, 30f);
        var PosY = 82.0f;

        if (other.CompareTag("ball"))
        {
            Debug.Log("Ball Out of Bounds");
            var ball = other.gameObject;
            ball.transform.localPosition = new Vector3(randomPosX, PosY, randomPosZ);
            
        }
        else if (other.CompareTag("whiteBall") || other.CompareTag("agent"))
        {
            

            bAgent.transform.localPosition = new Vector3(randomPosX, PosY, randomPosZ);
        }
        
        envController.ResolveEvent(Event.HitOutOfBounds);


    }


}