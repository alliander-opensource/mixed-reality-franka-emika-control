using UnityEngine;

using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class GripperJointSubscriber : MonoBehaviour
{
    public float[] gripperJointStatesDist;

    [SerializeField]
    private int numberOfGripperJoints;

    [SerializeField]
    private string gripperJointTopic;


    void Awake()
    {
        gripperJointStatesDist = new float[numberOfGripperJoints];
    }

    // Start is called before the first frame update
    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<JointStateMsg>(gripperJointTopic, Updatejoints);

    }

    void Updatejoints(JointStateMsg gripperJointState)
    {
        for (int i = 0; i < numberOfGripperJoints; i++)
        {
            gripperJointStatesDist[i] = (float)(gripperJointState.position[i]);
        }

    }
}
