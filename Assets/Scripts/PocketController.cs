using UnityEngine;

public class PocketController : MonoBehaviour
{
    
    public int numBalls;


    
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
            Debug.Log("Red Ball in the pocket");
        }
        
        
    }


}