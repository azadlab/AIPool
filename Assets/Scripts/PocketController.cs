using UnityEngine;

public class PocketController : MonoBehaviour
{
    
    public int BallsInPockets=0;
    public GameObject bAgent;
    void Start()
    {
        
        

    }


    /// <summary>
    /// Detects whether the ball lands in the goal pockets or out of bounds
    /// </summary>
    void OnCollisionEnter(Collision collision)
    {

        if(collision.gameObject.CompareTag("ball"))
        {
            BallsInPockets++;
            Debug.Log("Red Ball in the pocket");
            Debug.Log("Total Balls in Pocket"+BallsInPockets);
        }
        else if(collision.gameObject.CompareTag("whiteBall") || collision.gameObject.CompareTag("agent"))
        {
            Debug.Log("White Ball in the pocket, Spitting out...");
            var randomPosX = Random.Range(-100f, 100f);
            var randomPosZ = Random.Range(-30f, 30f);
            var PosY = 82.0f;
            bAgent.transform.localPosition = new Vector3(randomPosX, PosY, randomPosZ);
        }
        
        
    }


}