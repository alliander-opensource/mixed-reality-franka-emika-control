using RosMessageTypes.Std;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class CloseGripper : MonoBehaviour
{
    ROSConnection ros;

    [SerializeField]
    private string closeGripperServiceName;

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<EmptyRequest, EmptyResponse>(closeGripperServiceName);

    }

    public void CallCloseGripper()
    {
        ros.SendServiceMessage<EmptyResponse>(closeGripperServiceName, new EmptyRequest());
    }
}