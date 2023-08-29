using RosMessageTypes.Std;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class SetRRTConnectPlanner : MonoBehaviour
{
    ROSConnection ros;

    [SerializeField]
    private string setRRTConnectPlannerServiceName;

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<EmptyRequest, EmptyResponse>(setRRTConnectPlannerServiceName);

    }

    public void CallSetRRTConnectPlanner()
    {
        Debug.Log("Set RRTConnect");
        ros.SendServiceMessage<EmptyResponse>(setRRTConnectPlannerServiceName, new EmptyRequest());
    }
}
