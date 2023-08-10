using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class SetWorkspaceConstraint : MonoBehaviour
{
    ROSConnection ros;

    [SerializeField]
    private string setWorkspaceConstraintServiceName;

    [SerializeField]
    private string clearWorkspaceConstraintServiceName;

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<EmptyRequest, EmptyResponse>(setWorkspaceConstraintServiceName);
        ros.RegisterRosService<EmptyRequest, EmptyResponse>(clearWorkspaceConstraintServiceName);
    }

    public void CallSetWorkspaceConstraint()
    {
        ros.SendServiceMessage<EmptyResponse>(setWorkspaceConstraintServiceName, new EmptyRequest());
    }

    public void CallClearWorkspaceConstraint()
    {
        ros.SendServiceMessage<EmptyResponse>(clearWorkspaceConstraintServiceName, new EmptyRequest());
    }
}
