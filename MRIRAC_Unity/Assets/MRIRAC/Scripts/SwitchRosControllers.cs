using RosMessageTypes.Std;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using RosMessageTypes.Mrirac;

public class SwitchRosControllers : MonoBehaviour
{
    ROSConnection ros;

    [SerializeField]
    private string SwitchRosControllerServiceName;

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<SwitchControllerRequest, SwitchControllerResponse>(SwitchRosControllerServiceName);
    }

    void CallSwitchController(string startcontroller, string stopcontroller)
    {
        SwitchControllerRequest request = new SwitchControllerRequest()
        {
            start_controllers = new string[] {startcontroller},
            stop_controllers = new string[] {stopcontroller},
            strictness = 2,
            start_asap = true,
            timeout = 0.0 
        };

        ros.SendServiceMessage<SwitchControllerResponse>(SwitchRosControllerServiceName, request, SwitchControllerCallback);
    }

    void SwitchControllerCallback(SwitchControllerResponse response)
    {
        if (response.ok)
        {
            Debug.Log("Controller switched!");
        }
        else
        {
            Debug.Log("Controller not fully switched!");
        }
    }

    public void AcitaveTrajectoryControl()
    {
        CallSwitchController("effort_joint_trajectory_controller", "cartesian_impedance_example_controller");
    }

    public void AcitaveCartesianImpedanceControl()
    {
        CallSwitchController("cartesian_impedance_example_controller", "effort_joint_trajectory_controller");
    }

}
