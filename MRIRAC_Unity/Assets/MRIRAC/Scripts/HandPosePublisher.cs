using RosMessageTypes.Geometry;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Microsoft.MixedReality.Toolkit


public class HandPosePublisher : MonoBehaviour
{

    ROSConnection ros;
    [SerializeField]
    private string handPosePublisherTopic;
    [SerializeField]
    private GameObject targetEEF;
    EndEffectorPos targetEndEffectorPos;


    void Awake()
    {
        targetEndEffectorPos = targetEEF.GetComponent<EndEffectorPos>();
    }
    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseMsg>(handPosePublisherTopic);
    }

    // Update is called once per frame
    void Update()
    {
        PoseMsg targetPose = new PoseMsg()
        {
            position = new PointMsg(targetEndEffectorPos.Position.x, targetEndEffectorPos.Position.y, targetEndEffectorPos.Position.z),
            orientation = new QuaternionMsg(targetEndEffectorPos.Orientation.x, targetEndEffectorPos.Orientation.y, targetEndEffectorPos.Orientation.z, targetEndEffectorPos.Orientation.w)
        };
        ros.Publish(handPosePublisherTopic, targetPose);
    }
}
