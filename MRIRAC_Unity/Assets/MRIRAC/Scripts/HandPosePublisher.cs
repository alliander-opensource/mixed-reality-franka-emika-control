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
    private MixedRealityPose Palmpose;
    private MixedRealityPose IndexTipPose;
    private MixedRealityPose ThumbTipPose;
    public bool gripper_state_changed;
    public string gripper_state;
    public string prev_gripper_state;

    ROSConnection ros;
    [SerializeField]
    private string HandPosePublisherTopic;

    [SerializeField]
    private GameObject HandObject;

    [SerializeField]
    private GameObject ServiceObject;

    [SerializeField]
    private GameObject DirectControlEnvObject;


    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseMsg>(HandPosePublisherTopic);

        gripper_state_changed = false;
        gripper_state = null;
        prev_gripper_state = null;
    }

    public bool check_gripper_change(string gripper_state, string prev_gripper_state)
    {
        if (gripper_state != prev_gripper_state)
        {
            return true;
        }

        else
        {
            return false;
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (handRight == null)
        {
            handRight = HandJointUtils.FindHand(Handedness.Right);
        }
        else 
        {
            handRight = HandJointUtils.FindHand(Handedness.Right);

            handRight.TryGetJoint(TrackedHandJoint.Palm, out MixedRealityPose PalmPose);
            HandObject.transform.position = PalmPose.Position;

            PoseMsg handPalmPose = new PoseMsg()
            {
                position = new PointMsg(PalmPose.Position[0], PalmPose.Position[1], PalmPose.Position[2]),
                orientation = new QuaternionMsg(PalmPose.Rotation[0], PalmPose.Rotation[1], PalmPose.Rotation[2], PalmPose.Rotation[3])
            };

            //Debug.Log(DirectControlEnvObject.GetComponent<HandTriggerEvent>().GetHandInEnv());

            if (DirectControlEnvObject.GetComponent<HandTriggerEvent>().GetHandInEnv())
            {
                
                handRight.TryGetJoint(TrackedHandJoint.IndexTip, out MixedRealityPose IndexTipPose);
                handRight.TryGetJoint(TrackedHandJoint.ThumbTip, out MixedRealityPose ThumbTipPose);             

                // Calculate euclidean distance between index tip and thumb tip
                float dist = Vector3.Distance(IndexTipPose.Position, ThumbTipPose.Position);

                //Debug.Log("Palm pose is:");
                //Debug.Log(PalmPose);
                //Debug.Log("Index tip pose is:");
                //Debug.Log(IndexTipPose);
                //Debug.Log("Thumb tip pose is:");
                //Debug.Log(ThumbTipPose);
                //Debug.Log("Distance is:");
                //Debug.Log(dist);

                if (dist < 0.02f)
                {
                    gripper_state = "closed";

                    gripper_state_changed = check_gripper_change(gripper_state, prev_gripper_state);

                    if (gripper_state_changed)
                    {
                        CloseGripper CloseGripperScript = ServiceObject.GetComponent<CloseGripper>();
                        CloseGripperScript.CallCloseGripper();
                    }

                    Debug.Log("Hand Closed");

                }

                else
                {
                    gripper_state = "open";

                    gripper_state_changed = check_gripper_change(gripper_state, prev_gripper_state);

                    if (gripper_state_changed)
                    {
                        OpenGripper OpenGripperScript = ServiceObject.GetComponent<OpenGripper>();
                        OpenGripperScript.CallOpenGripper();
                    }
                    Debug.Log("Hand Open");
                }

                prev_gripper_state = gripper_state;

                ros.Publish(HandPosePublisherTopic, handPalmPose);
            }
        }     
    }
}