using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HandTriggerEvent : MonoBehaviour
{
    [SerializeField]
    public GameObject PublisherObject;

    private HandPosePublisher HandPosePublisherScript;

    void Start()
    {
        HandPosePublisherScript = PublisherObject.GetComponent<HandPosePublisher>();
    }

    public void OnTriggerEnter(Collider other)
    {
        
        if (other.tag == "DirectControlEnvironment")
        {
            // Debug.Log("Entered object");
            HandPosePublisherScript.toggle_direct_control();
        }
    }

    public void OnTriggerExit(Collider other)
    {

        if (other.tag == "DirectControlEnvironment")
        {
            // Debug.Log("Exited object");
            HandPosePublisherScript.toggle_direct_control();
        }
    }
}
