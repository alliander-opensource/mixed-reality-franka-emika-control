using UnityEngine;

public class RobotManager : MonoBehaviour
{
    // Original code
    [SerializeField]
    private string[] linkTags;
    RobotJoint[] robotJoints;
    GameObject[] joints;

    float[] prevAnglesDeg;
    public float[] PreviousJointAnglesDeg { get => prevAnglesDeg; private set { prevAnglesDeg = value; } }

    // Added gripper code
    [SerializeField]
    private string[] linkTags_Gripper;
    RobotJoint[] robotGripperJoints;
    GameObject[] gripperJoints;

    float[] gripperPrevDist;
    public float[] GripperPreviousJointDist { get => gripperPrevDist; private set { gripperPrevDist = value; } }


    void Awake()
    {
        // Orignial code
        robotJoints = new RobotJoint[linkTags.Length];
        joints = new GameObject[linkTags.Length];
        prevAnglesDeg = new float[linkTags.Length];
        for (int jointIdx = 0; jointIdx < linkTags.Length; jointIdx++)
        {
            joints[jointIdx] = GameObject.FindGameObjectWithTag(linkTags[jointIdx]);
            robotJoints[jointIdx] = joints[jointIdx].GetComponent<RobotJoint>();
        }

        // Added gripper code
        robotGripperJoints = new RobotJoint[linkTags_Gripper.Length];
        gripperJoints = new GameObject[linkTags_Gripper.Length];
        gripperPrevDist = new float[linkTags_Gripper.Length];
        for (int gripperJointIdx = 0; gripperJointIdx < linkTags_Gripper.Length; gripperJointIdx++)
        {
            gripperJoints[gripperJointIdx] = GameObject.FindGameObjectWithTag(linkTags_Gripper[gripperJointIdx]);
            robotGripperJoints[gripperJointIdx] = gripperJoints[gripperJointIdx].GetComponent<RobotJoint>();
        }
    }

    public void SetJointAngles(float[] angles)
    {
        // TrhowIfnotEqual
        for (int jointAngleIdx = 0; jointAngleIdx < angles.Length; jointAngleIdx++)
        {
            joints[jointAngleIdx].transform.localRotation = joints[jointAngleIdx].transform.localRotation * Quaternion.AngleAxis(angles[jointAngleIdx] - prevAnglesDeg[jointAngleIdx], robotJoints[jointAngleIdx].axis);
            prevAnglesDeg[jointAngleIdx] = angles[jointAngleIdx];
        }
    }

    public void SetGripperJointDist(float[] gripperDist)
    {
        // TrhowIfnotEqual
        for (int gripperJointDistIdx = 0; gripperJointDistIdx < gripperDist.Length; gripperJointDistIdx++)
        {
            if (gripperJointDistIdx == 0)
            {
                gripperJoints[gripperJointDistIdx].transform.localPosition = gripperJoints[gripperJointDistIdx].transform.localPosition + new Vector3((gripperDist[gripperJointDistIdx] - gripperPrevDist[gripperJointDistIdx]), 0, 0); //new Vector3(0.1, 0, 0);
                gripperPrevDist[gripperJointDistIdx] = gripperDist[gripperJointDistIdx];
            }
            else
            {
                gripperJoints[gripperJointDistIdx].transform.localPosition = gripperJoints[gripperJointDistIdx].transform.localPosition - new Vector3((gripperDist[gripperJointDistIdx] - gripperPrevDist[gripperJointDistIdx]), 0, 0); //new Vector3(0.1, 0, 0);
                gripperPrevDist[gripperJointDistIdx] = gripperDist[gripperJointDistIdx];
            }
        }
    }
}
