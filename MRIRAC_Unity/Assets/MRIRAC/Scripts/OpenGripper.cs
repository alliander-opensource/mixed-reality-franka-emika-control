using RosMessageTypes.Std;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class OpenGripper : MonoBehaviour
{
    ROSConnection ros;

    [SerializeField]
    private string openGripperServiceName;

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<EmptyRequest, EmptyResponse>(openGripperServiceName);

    }

    public void CallOpenGripper()
    {
        ros.SendServiceMessage<EmptyResponse>(openGripperServiceName, new EmptyRequest());
    }
}