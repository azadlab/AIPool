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
    void OnCollisionEnter(Collision col)
    {

        if(col.gameObject.CompareTag("wall"))
        {
            envController.ResolveEvent(Event.HitOutOfBounds);
            
        }
        
        
        
        
    }


}