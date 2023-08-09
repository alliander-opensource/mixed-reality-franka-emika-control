using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class StartPositionArm : MonoBehaviour
{
    ROSConnection ros;

    [SerializeField]
    private string startPositionServiceName;

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<EmptyRequest, EmptyResponse>(startPositionServiceName);

    }

    public void CallStartPositionArm()
    {
        ros.SendServiceMessage<EmptyResponse>(startPositionServiceName, new EmptyRequest());
    }
}
