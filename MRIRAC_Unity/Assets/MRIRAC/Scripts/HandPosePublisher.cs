using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class HandPosePublisher : MonoBehaviour
{
    private IMixedRealityHand handRight;
    private MixedRealityPose pose;

    ROSConnection ros;
    [SerializeField]
    private string HandPosePublisherTopic;

    

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseMsg>(HandPosePublisherTopic);
    }

    // Update is called once per frame
    void Update()
    {
        if (handRight == null)
        {
            handRight = HandJointUtils.FindHand(Handedness.Left);
            Debug.Log("Finding hand...");
        }
        else 
        {
            handRight.TryGetJoint(TrackedHandJoint.Palm, out MixedRealityPose pose);

            PoseMsg handPose = new PoseMsg()
            {
                position = new PointMsg(pose.Position[0], pose.Position[1], pose.Position[2]),
                orientation = new QuaternionMsg(pose.Rotation[0], pose.Rotation[1], pose.Rotation[2], pose.Rotation[3])
            };
            ros.Publish(HandPosePublisherTopic, handPose);
        }

        
    }
}