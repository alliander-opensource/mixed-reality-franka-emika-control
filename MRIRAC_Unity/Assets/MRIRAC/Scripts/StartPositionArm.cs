using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class StartPositionArm : MonoBehaviour
{
    ROSConnection ros;

    [SerializeField]
    private string startPositionServiceName;

    [SerializeField]
    private GameObject serviceObject;

    private SetRRTConnectPlanner serviceSetRRTConnect;
    private SetRRTStarPlanner serviceSetRRTStar;

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<EmptyRequest, EmptyResponse>(startPositionServiceName);

        serviceSetRRTConnect = serviceObject.GetComponent<SetRRTConnectPlanner>();
        serviceSetRRTStar = serviceObject.GetComponent<SetRRTStarPlanner>();

    }

    public void CallStartPositionArm()
    {
        serviceSetRRTConnect.CallSetRRTConnectPlanner();
        ros.SendServiceMessage<EmptyResponse>(startPositionServiceName, new EmptyRequest());
        serviceSetRRTStar.CallSetRRTStarPlanner();
    }
}
