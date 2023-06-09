using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Microsoft.MixedReality.Toolkit.Experimental.UI;

public class RosIP : MonoBehaviour
{
    public GameObject ipAddressObject;
    public GameObject RosConnectorObject;
    private TouchScreenKeyboard keyboard;

    public void ChangeRosIP() 
    {
        RosConnectorObject = GameObject.Find("ROSConnectionPrefab(Clone)");
        ROSConnection RosConnector = RosConnectorObject.GetComponent<ROSConnection>();
        MRTKTMPInputField ipAddress = ipAddressObject.GetComponent<MRTKTMPInputField>();


        RosConnector.Disconnect();
        RosConnector.RosIPAddress = ipAddress.text;
        RosConnector.Connect();
    }
}
