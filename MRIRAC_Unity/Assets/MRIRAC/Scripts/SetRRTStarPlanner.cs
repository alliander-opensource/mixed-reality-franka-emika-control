using RosMessageTypes.Std;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class SetRRTStarPlanner : MonoBehaviour
{
    ROSConnection ros;

    [SerializeField]
    private string setRRTStarPlannerServiceName;

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<EmptyRequest, EmptyResponse>(setRRTStarPlannerServiceName);

    }

    public void CallSetRRTStarPlanner()
    {
        ros.SendServiceMessage<EmptyResponse>(setRRTStarPlannerServiceName, new EmptyRequest());
    }
}
