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

    [SerializeField]
    private GameObject RobotHandle;

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
            HandObject.transform.rotation = PalmPose.Rotation;

            Vector3 T_Hand_DirectEnv = HandObject.transform.localPosition;
            Quaternion R_Hand_DirectEnv_lh = HandObject.transform.localRotation;
            Quaternion R_Hand_DirectEnv_rh = new Quaternion(-R_Hand_DirectEnv_lh.y, R_Hand_DirectEnv_lh.z, R_Hand_DirectEnv_lh.x, R_Hand_DirectEnv_lh.w);
            // Quaternion R_DirectEnv_Unity = DirectControlEnvObject.transform.rotation;
            // Quaternion R_RobotBase_Unity = RobotHandle.transform.rotation;
            // Quaternion R_Ros_RobotBase = Quaternion.Euler(new Vector3(-90, -90, 0));
            // Quaternion R_Hand_unity = R_Hand_DirectEnv * R_DirectEnv_Unity;

            // Quaternion R_Unity_EndEffector_z = Quaternion.Euler(new Vector3(0, 0, -90));
            // Quaternion R_Unity_EndEffector_y = Quaternion.Euler(new Vector3(0, 90, 0));
            // Quaternion R_Unity_EndEffector_x = Quaternion.Euler(new Vector3(90, 0, 0));

            Quaternion R_EndEffector = R_Hand_DirectEnv_rh; //Quaternion.Euler(new Vector3(-180, 0, 0)) * 

            //Debug.Log("Palm pose rotation in environment frame left handed:");
            //Debug.Log(HandObject.transform.position);
            //Debug.Log(R_Hand_DirectEnv_lh.eulerAngles);

            //Debug.Log("Palm pose rotation in environment frame right handed:");
            //Debug.Log(T_Hand_DirectEnv);
            //Debug.Log(R_Hand_DirectEnv_rh.eulerAngles);

            //Debug.Log("Hand rotation in robotbase frame:");
            //Debug.Log(R_EndEffector.eulerAngles);

            // Debug.Log("Env rotation in unity frame:");
            // Debug.Log(R_DirectEnv_Unity.eulerAngles);

            // Debug.Log("Robotbase rotation in unity frame:");
            // Debug.Log(R_RobotBase_Unity.eulerAngles);

            // Debug.Log("Ros rotation in Robotbase frame:");
            // Debug.Log(R_Ros_RobotBase.eulerAngles);

            

            PoseMsg handPalmPose = new PoseMsg()
            {
                //position = new PointMsg((PalmPose.Position[2] - DirectControlEnvObject.transform.position.z), -(PalmPose.Position[0] - DirectControlEnvObject.transform.position.x), (PalmPose.Position[1] - DirectControlEnvObject.transform.position.y)),
                position = new PointMsg((T_Hand_DirectEnv[2] + 0.6f), -(T_Hand_DirectEnv[0]), (T_Hand_DirectEnv[1] + 0.5f)), //[2] + 0.4f
                //position = new PointMsg((0.4f), -(0.0f), (0.5f)),
                orientation = new QuaternionMsg(R_EndEffector[0], R_EndEffector[1], R_EndEffector[2], R_EndEffector[3])
                //orientation = new QuaternionMsg(1f, 0f, 0f, 0f)
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
                //Debug.Log(handPalmPose.position.x);
                //Debug.Log(handPalmPose.position.y);
                //Debug.Log(handPalmPose.position.z);

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

                    //Debug.Log("Hand Closed");

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
                    //Debug.Log("Hand Open");
                }

                prev_gripper_state = gripper_state;

                ros.Publish(HandPosePublisherTopic, handPalmMessage);
            }

            else 
            {
                PoseMsg readyPose = new PoseMsg()
                {
                    position = new PointMsg(0.4f, 0f, 0.5f),
                    orientation = new QuaternionMsg(1f, 0f, 0f, 0f)
                };

                HeaderMsg readyHeader = new HeaderMsg()
                {
                    frame_id = "fr3_link0"
                };

                PoseStampedMsg readyMessage = new PoseStampedMsg()
                {
                    header = readyHeader,
                    pose = readyPose
                };

                ros.Publish(HandPosePublisherTopic, readyMessage);
            }

        }
    }     
}
