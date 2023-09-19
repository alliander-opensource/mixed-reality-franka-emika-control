using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CollisionTriggerEvent : MonoBehaviour
{

    public int counter;
    // Start is called before the first frame update
    void Start()
    {
        counter = 0;
    }

    public void OnTriggerEnter(Collider other)
    {
        // Debug.Log("In the OnTriggerEnter");
        if (other.tag == "EnvironmentObstacle")
        {
            // Debug.Log("Entered obstacle");
            counter += 1;
        }
    }

    public void ResetCounter()
    {
        counter = 0;
    }
}
