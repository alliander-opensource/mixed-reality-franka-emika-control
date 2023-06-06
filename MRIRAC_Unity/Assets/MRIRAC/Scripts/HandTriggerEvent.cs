using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HandTriggerEvent : MonoBehaviour
{
    private void OnTriggerStay(Collider other)
    {
        if (other.tag == "DirectControlEnvironment")
        {
            Debug.Log("Hand in environment");
            other.transform.Translate(Vector3.forward * Time.deltaTime * 1.0f);
        }
    }
}
