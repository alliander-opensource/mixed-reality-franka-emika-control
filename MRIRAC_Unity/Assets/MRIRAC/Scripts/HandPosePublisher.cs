using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using System;

public class HandPosePublisher : MonoBehaviour
{
    private IMixedRealityHand handTarget;
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

    [SerializeField]
    private GameObject RobotHandle;

    public HandEnum handedness;


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

        HandObject.transform.SetParent(DirectControlEnvObject.transform);

        // Set handedness on startup
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
        //Debug.Log("Toggling direct control");

        if (direct_control_active)
        {
            direct_control_active = false;
        }

        else
        {
            direct_control_active = true;
        }
    }

    public void activate_left_hand()
    {
        handedness = HandEnum.L;
    }

    public void activate_right_hand()
    {
        handedness = HandEnum.R;
    }

    // Update is called once per frame
    void Update()
    {   
        PoseMsg cmdPose = new PoseMsg();        
        handTarget = (handedness == HandEnum.R) ? HandJointUtils.FindHand(Handedness.Right) : HandJointUtils.FindHand(Handedness.Left);
        
        if (handTarget != null) // no object errrors
        {
            handTarget.TryGetJoint(TrackedHandJoint.Palm, out MixedRealityPose PalmPose);

            HandObject.transform.position = PalmPose.Position;
            HandObject.transform.rotation = PalmPose.Rotation;
            
            Vector3 T_Hand_DirectEnv = HandObject.transform.localPosition * 1.5f; // The factor 1.5 is to increase the sensitivty of the movement
            Quaternion R_Hand_DirectEnv_unity = HandObject.transform.localRotation;         
            
            //Rotation Matrices (ZXY)
            Quaternion ROS2Panda_R = Quaternion.Euler(new Vector3(180, 0, 0));

            //Rotation steps taken (Unity_lh --> ROS_rh --> End_Effector)
            Quaternion R_Hand_DirectEnv_ROS = new Quaternion(-R_Hand_DirectEnv_unity.z, -R_Hand_DirectEnv_unity.x, R_Hand_DirectEnv_unity.y, R_Hand_DirectEnv_unity.w);  // HOW???
            Quaternion R_EndEffector = ROS2Panda_R * R_Hand_DirectEnv_ROS;            

            cmdPose = new PoseMsg()
            {
                position = new PointMsg((T_Hand_DirectEnv[2] + 0.6f), -(T_Hand_DirectEnv[0]), (T_Hand_DirectEnv[1] + 0.5f)), //[2] + 0.4f
                orientation = new QuaternionMsg(R_EndEffector[0], R_EndEffector[1], R_EndEffector[2], R_EndEffector[3])
                // orientation = new QuaternionMsg(0.7f, 0f, 0.7f, 0f)
            };

            if (direct_control_active)
            {
                handTarget.TryGetJoint(TrackedHandJoint.IndexTip, out MixedRealityPose IndexTipPose);
                handTarget.TryGetJoint(TrackedHandJoint.ThumbTip, out MixedRealityPose ThumbTipPose);             

                // Calculate euclidean distance between index tip and thumb tip
                float dist = Vector3.Distance(IndexTipPose.Position, ThumbTipPose.Position);

                if (dist < 0.02f)
                {
                    gripper_state = "closed";
                    gripper_state_changed = check_gripper_change(gripper_state, prev_gripper_state);

                    if (gripper_state_changed)
                    {
                        CloseGripper CloseGripperScript = ServiceObject.GetComponent<CloseGripper>();
                        CloseGripperScript.CallCloseGripper();
                    }
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
                }

                prev_gripper_state = gripper_state;
            }

            else
            {
                cmdPose = new PoseMsg()
                {
                    position = new PointMsg(0.4f, 0f, 0.5f),
                    orientation = new QuaternionMsg(1f, 0f, 0f, 0f)
                };
            }

            HeaderMsg cmdHeader = new HeaderMsg()
            {
                frame_id = "fr3_link0"
            };

            PoseStampedMsg cmdMessage = new PoseStampedMsg()
            {
                header = cmdHeader,
                pose = cmdPose
            };

            ros.Publish(HandPosePublisherTopic, cmdMessage);
        }
    }
}     


public enum HandEnum
{
    R,
    L
}