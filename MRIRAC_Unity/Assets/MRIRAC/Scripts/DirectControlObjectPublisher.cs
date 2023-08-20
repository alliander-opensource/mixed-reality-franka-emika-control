using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using System;

public class DirectControlObjectPublisher : MonoBehaviour
{
    ROSConnection ros;
    [SerializeField]
    private string HandPosePublisherTopic;

    [SerializeField]
    private GameObject ControlObject;

    [SerializeField]
    private GameObject RobotHandle;

    // Start is called before the first frame update
    void Start()
    {
        //mrirac_trajectory_planner_Fr3/unity_hand_pose
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>(HandPosePublisherTopic);
    }

    // Update is called once per frame
    void Update()
    {   
        PoseMsg cmdPose = new PoseMsg();        
                    
        Vector3 T_Hand_DirectEnv = ControlObject.transform.localPosition * 2f;
        Quaternion R_Hand_DirectEnv_unity = ControlObject.transform.localRotation;         
            
        //Rotation Matrices (ZXY)
        Quaternion ROS2Panda_R = Quaternion.Euler(new Vector3(180, 0, 0));

        //Rotation steps taken (Unity_lh --> ROS_rh --> End_Effector)
        Quaternion R_Hand_DirectEnv_ROS = new Quaternion(-R_Hand_DirectEnv_unity.z, -R_Hand_DirectEnv_unity.x, R_Hand_DirectEnv_unity.y, R_Hand_DirectEnv_unity.w);  // HOW???
        Quaternion R_EndEffector = ROS2Panda_R * R_Hand_DirectEnv_ROS;            

        cmdPose = new PoseMsg()
        {
            // position = new PointMsg((T_Hand_DirectEnv[2] + 0.6f), -(T_Hand_DirectEnv[0]), (T_Hand_DirectEnv[1] + 0.5f)), //[2] + 0.4f
            position = new PointMsg((0.6f), -(T_Hand_DirectEnv[0]-0.6), (T_Hand_DirectEnv[1] + 0.4f)), //T_Hand_DirectEnv[2] + 
            // orientation = new QuaternionMsg(R_EndEffector[0], R_EndEffector[1], R_EndEffector[2], R_EndEffector[3])
            orientation = new QuaternionMsg(0.7f, 0f, 0.7f, 0f)
        };

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

    public void ResetControlObject()
    {
        ControlObject.transform.localPosition = new Vector3(0f, 0f, 0f);
    }
}     