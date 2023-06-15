using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public class HandPosePublisher : MonoBehaviour
{
    private IMixedRealityHand handRight;
    private MixedRealityPose Palmpose;
    private MixedRealityPose IndexTipPose;
    private MixedRealityPose ThumbTipPose;
    private bool gripper_state_changed;
    private string gripper_state;
    private string prev_gripper_state;
    private bool direct_control_active;


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
        //mrirac_trajectory_planner_Fr3/unity_hand_pose
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>(HandPosePublisherTopic);

        gripper_state_changed = false;
        gripper_state = null;
        prev_gripper_state = null;
        direct_control_active = false;
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

    public void toggle_direct_control()
    {
        Debug.Log("Toggling direct control");

        if (direct_control_active)
        {
            direct_control_active = false;
        }

        else
        {
            direct_control_active = true;
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
                position = new PointMsg((PalmPose.Position[0] - DirectControlEnvObject.transform.position.x) * 10, (PalmPose.Position[1] - DirectControlEnvObject.transform.position.y) * 10, (PalmPose.Position[2] - DirectControlEnvObject.transform.position.z) * 10),
                orientation = new QuaternionMsg(PalmPose.Rotation[0], PalmPose.Rotation[1], PalmPose.Rotation[2], PalmPose.Rotation[3])
            };

            HeaderMsg handPalmHeader = new HeaderMsg()
            {
                frame_id = "fr3_link0"
            };

            PoseStampedMsg handPalmMessage = new PoseStampedMsg()
            {
                header = handPalmHeader,
                pose = handPalmPose
            };


            if (direct_control_active)
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

                ros.Publish(HandPosePublisherTopic, handPalmMessage);
            }
        }     
    }
}