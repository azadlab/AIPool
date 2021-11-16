using UnityEngine;

public class BilliardballController : MonoBehaviour
{
    public BilliardEnvController envController;
    
    public ballType ballColor;

    
    void Start()
    {
        //GoalColliders = redGoal.GetComponent<Collider>();
        
        
        envController = GetComponentInParent<BilliardEnvController>();

    }

    /// <summary>
    /// Detects whether the ball lands in the goal pockets or out of bounds
    /// </summary>
    void OnTriggerEnter(Collider other)
    {

        if(other.gameObject.CompareTag("boundary"))
        {
            envController.ResolveEvent(Event.HitOutOfBounds);
            
        }
        
        else if(other.gameObject.CompareTag("Pocket"))
        {
            envController.ResolveEvent(Event.HitGoal);
        }
        
        
    }


}