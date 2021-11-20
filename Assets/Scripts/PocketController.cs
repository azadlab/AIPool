using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class PocketController : MonoBehaviour
{
    
    public BilliardEnvController envController;
    public GameObject bAgent;
    
    void Start()
    {
        
        envController = GetComponentInParent<BilliardEnvController>();

    }


    /// <summary>
    /// Detects whether the ball lands in the goal pockets or out of bounds
    /// </summary>
    void OnCollisionStay(Collision collision)
    {
        if(collision.gameObject.CompareTag("ball"))
        {
            
            bool alreadyCollided = false;
            foreach(var ballName in envController.balls_in_pocket)
                if(ballName == collision.gameObject.name)
                {
                    alreadyCollided = true;
                    return;
                }
            if(!alreadyCollided)   
            { 
                envController.balls_in_pocket.Add(collision.gameObject.name);
                envController.BallsInPockets++;
                envController.ResolveEvent(Event.HitPocket);
                Debug.Log(envController.name+": Ball in the Pocket");
            }
        }
        
        
        
    }

    void OnCollisionExit(Collision collision)
    {
        /*if(collision.gameObject.CompareTag("ball"))
        {
            
            if(envController.balls_in_pocket.Contains(collision.gameObject.name))
            {   
                    envController.BallsInPockets--;
                    envController.balls_in_pocket.Remove(collision.gameObject.name);
                    envController.ResolveEvent(Event.LeftPocket);
                    Debug.Log(envController.name+":Ball No Longer in the Pocket");
            }
                
        }*/
    }


}