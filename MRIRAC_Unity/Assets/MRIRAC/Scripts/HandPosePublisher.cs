using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;

public class HandPosePublisher : MonoBehaviour
{

    public IMixedRealityHand handRight;
    public MixedRealityPose pose;

    // Start is called before the first frame update
    //void Start()
    //{
    //    
    //}

    // Update is called once per frame
    void Update()
    {
        if (handRight == null)
        {
            handRight = HandJointUtils.FindHand(Handedness.Right);
            Debug.Log("Finding hand...");
        }
        else 
        {
            handRight.TryGetJoint(TrackedHandJoint.Palm, out MixedRealityPose pose);
            Debug.Log(pose);
            //Debug.Log(pose.rotation);
        }
    }
}