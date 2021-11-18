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
                Debug.Log("Total Balls in Pocket="+envController.BallsInPockets);
                envController.ResolveEvent(Event.HitPocket);
                
            }
        }
        else
        if(collision.gameObject.CompareTag("whiteBall") || collision.gameObject.CompareTag("agent"))
        {
            Debug.Log("White Ball in the pocket, Spitting out...");
            var randomPosX = Random.Range(-100f, 100f);
            var randomPosZ = Random.Range(-30f, 30f);
            var PosY = 82.0f;
            bAgent.transform.localPosition = new Vector3(randomPosX, PosY, randomPosZ);
        }
        
        
    }

    void OnCollisionExit(Collision collision)
    {
        if(collision.gameObject.CompareTag("ball"))
        {
            
            if(envController.balls_in_pocket.Contains(collision.gameObject.name))
            {   
                    envController.BallsInPockets--;
                    envController.balls_in_pocket.Remove(collision.gameObject.name);
                    envController.ResolveEvent(Event.LeftPocket);
                    Debug.Log("Ball No Longer in the Pocket");
            }
                
        }
    }


}