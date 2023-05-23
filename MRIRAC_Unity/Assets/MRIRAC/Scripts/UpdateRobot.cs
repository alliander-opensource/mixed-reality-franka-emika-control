using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UpdateRobot : MonoBehaviour
{
    [SerializeField]
    private GameObject subscriber;
    RobotManager robotManager;
    JointSubscriber jointSubscriber;
    GripperJointSubscriber gripperJointSubscriber;


    void Awake()
    {
        robotManager = GetComponent<RobotManager>();
        jointSubscriber = subscriber.GetComponent<JointSubscriber>();
        gripperJointSubscriber = subscriber.GetComponent<GripperJointSubscriber>();
    }

    // Update is called once per frame
    void Update()
    {
        robotManager.SetJointAngles(jointSubscriber.jointStatesDeg);

        robotManager.SetGripperJointDist(gripperJointSubscriber.gripperJointStatesDist);
    }
}
