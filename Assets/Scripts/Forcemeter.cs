using System.Collections;
using System.Collections.Generic;

using UnityEngine;
using UnityEngine.UI;

public class Forcemeter : MonoBehaviour
{


    public GameObject arrow; 
    public float pos = 0.0f;
    public float cur_pos = 0.0f;
    void Start()
    {
        pos = arrow.transform.localPosition.x;
        cur_pos = pos;
    }

    private void Update()
    {

        if (Input.GetKey(KeyCode.UpArrow))
            cur_pos += 3.2f;
        else if (Input.GetKey(KeyCode.DownArrow))
            cur_pos -= 3.2f;

        if (cur_pos < pos) cur_pos = pos;
        if (cur_pos > -1*pos) cur_pos = -1*pos;

        Vector3 t_pos;
        if (arrow != null)
        {
            t_pos = arrow.transform.localPosition;
            t_pos.x = cur_pos;
            arrow.transform.localPosition = t_pos;
        }
            
    }
}
