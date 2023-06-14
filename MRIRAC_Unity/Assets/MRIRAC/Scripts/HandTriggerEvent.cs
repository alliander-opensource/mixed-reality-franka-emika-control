using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HandTriggerEvent : MonoBehaviour
{
    private bool HandInEnv;

    public void OnTriggerEnter(Collider other)
    {
        
        if (other.tag == "DirectControlEnvironment")
        {
            Debug.Log("Entered object");
            HandInEnv = true;
        }
    }

    public void OnTriggerExit(Collider other)
    {

        if (other.tag == "DirectControlEnvironment")
        {
            Debug.Log("Exited object");
            HandInEnv = false;
        }
    }

    public bool GetHandInEnv()
    {
        //Debug.Log(HandInEnv);
        return HandInEnv;
    }
}
