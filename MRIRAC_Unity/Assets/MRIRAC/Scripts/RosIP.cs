using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class RosIP : MonoBehaviour
{
    public GameObject RosConnectorObject;
    private TouchScreenKeyboard keyboard;
    public static string ipAdress = "";

    public void ChangeRosIP() 
    {
        RosConnectorObject = GameObject.Find("ROSConnectionPrefab(Clone)");
        ROSConnection RosConnector = RosConnectorObject.GetComponent<ROSConnection>();
        keyboard = TouchScreenKeyboard.Open("text to edit");
        ipAdress = keyboard.text;
        Debug.Log(ipAdress);
        Debug.Log(RosConnector);
        RosConnector.Disconnect();
        RosConnector.Connect(ipAdress, 10000);
    }
}
