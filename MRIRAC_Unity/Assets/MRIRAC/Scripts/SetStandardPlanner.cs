using RosMessageTypes.Std;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class SetStandardPlanner : MonoBehaviour
{
    ROSConnection ros;

    [SerializeField]
    private string setStandardPlannerServiceName;

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<EmptyRequest, EmptyResponse>(setStandardPlannerServiceName);

    }

    public void CallSetStandardPlanner()
    {
        ros.SendServiceMessage<EmptyResponse>(setStandardPlannerServiceName, new EmptyRequest());
    }
}
