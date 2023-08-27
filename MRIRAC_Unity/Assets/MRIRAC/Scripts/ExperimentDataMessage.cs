using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using RosMessageTypes.Trajectory;
using RosMessageTypes.Std;
using RosMessageTypes.Mrirac;
using Unity.Robotics.ROSTCPConnector;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;
using Microsoft.MixedReality.Toolkit;

public class ExperimentDataMessage : MonoBehaviour
{
    ROSConnection ros;
    [SerializeField]
    private string ExperimentDataTopic;

    [SerializeField]
    private GameObject RobotBaseObject;

    [SerializeField]
    private GameObject EnvironmentsObject;

    [SerializeField]
    private CollisionTriggerEvent CollisionCounter;

    [SerializeField]
    private GameObject TimerUI;

    [SerializeField]
    private GameObject ExperimentWaypointControlUI;

    [SerializeField]
    private GameObject ExperimentDirectControlUI;

    [SerializeField]
    private GameObject ExperimentCommandControlUI;

    [SerializeField]
    private GameObject TargetEndEffector;

    [SerializeField]
    private GameObject EndEffectorObject;

    [SerializeField]
    private Environments EnvironmentsScript;

    [SerializeField]
    private WaypointPlanner waypointPlanner;

    [SerializeField]
    private TrajectoryPlanner trajectoryPlanner;

    private string ControlMethod;
    private string Condition;

    public EndEffectorPos EndEffectorCoordinates;
    public bool StoringEndEffectorPosition;
    public List<PoseStampedMsg> EndEffectorPositionList;
    public JointTrajectoryMsg Trajectory;
    public bool Success;
    private int timer;
    private int interval;
    private float elapsedtime;
    public List<double> time_list;

    private IMixedRealityHand handTarget;
    private MixedRealityPose Palmpose;
    public List<PoseMsg> handtracking_list;
    private PoseMsg handpose;

    [SerializeField]
    private HandPosePublisher HandPosePublisher;

    public List<PointMsg> gazedirection_list;
    public List<PointMsg> gazeorigin_list;
    public List<string> gazetargetname_list;
    public List<PointMsg> gazetargetposition_list;
    public List<PointMsg> headvelocity_list;
    public List<PointMsg> headmovementdirection_list;

    public List<long> waypointcount_list;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ExperimentDataMsg>(ExperimentDataTopic);

        EndEffectorCoordinates = EndEffectorObject.GetComponent<EndEffectorPos>();

        StoringEndEffectorPosition = false;

        timer = 0;
        interval = 6;
        elapsedtime = 0f;
       
    }

    void Update()
    {
        elapsedtime += Time.deltaTime;

        if (timer%interval == 0)
        {
            if (StoringEndEffectorPosition)
            {

                // Timeseries end effector position
                PoseStampedMsg msg = new PoseStampedMsg()
                {
                    pose = new PoseMsg()
                    {
                        position = new PointMsg(EndEffectorCoordinates.Position.x, EndEffectorCoordinates.Position.y, EndEffectorCoordinates.Position.z),
                        orientation = new QuaternionMsg(EndEffectorCoordinates.Orientation.x, EndEffectorCoordinates.Orientation.y, EndEffectorCoordinates.Orientation.z, EndEffectorCoordinates.Orientation.w)
                    }
                };

                msg.header.stamp.sec = (uint)elapsedtime;

                EndEffectorPositionList.Add(msg);
                // Debug.Log("added end-effector");
                // Debug.Log(msg);

                // Time timeseries
                time_list.Add(elapsedtime);
                // Debug.Log("added elapsed time");
                // Debug.Log(elapsedtime);

                // Hand tracking timeseries
                handTarget = (HandPosePublisher.handedness == HandEnum.R) ? HandJointUtils.FindHand(Handedness.Right) : HandJointUtils.FindHand(Handedness.Left);
                if (handTarget != null)
                {
                    handTarget.TryGetJoint(TrackedHandJoint.Palm, out MixedRealityPose PalmPose);

                    handpose = new PoseMsg()
                    {
                        position = new PointMsg(PalmPose.Position.x, PalmPose.Position.y, PalmPose.Position.z),
                        orientation = new QuaternionMsg(PalmPose.Rotation.x, PalmPose.Rotation.y, PalmPose.Rotation.z, PalmPose.Rotation.w)
                    };
                }
                else
                {
                    handpose = new PoseMsg();
                }

                handtracking_list.Add(handpose);
                // Debug.Log("added handpose");
                // Debug.Log(handpose);

                // Eye tracking timeseries
                PointMsg gazedirection_msg = new PointMsg()
                {
                    x = CoreServices.InputSystem.GazeProvider.GazeDirection.x,
                    y = CoreServices.InputSystem.GazeProvider.GazeDirection.y,
                    z = CoreServices.InputSystem.GazeProvider.GazeDirection.z
                };
                gazedirection_list.Add(gazedirection_msg);
                // Debug.Log(CoreServices.InputSystem.GazeProvider.GazeDirection);
                PointMsg gazeoriging_msg = new PointMsg()
                {
                    x = CoreServices.InputSystem.GazeProvider.GazeOrigin.x,
                    y = CoreServices.InputSystem.GazeProvider.GazeOrigin.y,
                    z = CoreServices.InputSystem.GazeProvider.GazeOrigin.z
                };
                gazeorigin_list.Add(gazeoriging_msg);
                // Debug.Log(CoreServices.InputSystem.GazeProvider.GazeOrigin);
                if (CoreServices.InputSystem.GazeProvider.GazeTarget)
                {
                    gazetargetname_list.Add(CoreServices.InputSystem.GazeProvider.GazeTarget.name.ToString());

                    PointMsg gazetargetposition_msg = new PointMsg()
                    {
                        x = CoreServices.InputSystem.GazeProvider.GazeTarget.transform.position.x,
                        y = CoreServices.InputSystem.GazeProvider.GazeTarget.transform.position.y,
                        z = CoreServices.InputSystem.GazeProvider.GazeTarget.transform.position.z
                    };
                    gazetargetposition_list.Add(gazetargetposition_msg);
                }
                else
                {
                    gazetargetname_list.Add(null);

                    PointMsg gazetargetposition_msg = new PointMsg()
                    {
                        x = 999999f,
                        y = 999999f,
                        z = 999999f
                    };
                    gazetargetposition_list.Add(gazetargetposition_msg);
                }
                // Debug.Log(CoreServices.InputSystem.GazeProvider.GazeTarget);

                // Head tracking timeseries
                PointMsg headvelocity_msg = new PointMsg()
                    {
                        x = CoreServices.InputSystem.GazeProvider.HeadVelocity.x,
                        y = CoreServices.InputSystem.GazeProvider.HeadVelocity.y,
                        z = CoreServices.InputSystem.GazeProvider.HeadVelocity.z
                    };
                headvelocity_list.Add(headvelocity_msg);
                PointMsg headmovementdirection_msg = new PointMsg()
                    {
                        x = CoreServices.InputSystem.GazeProvider.HeadMovementDirection.x,
                        y = CoreServices.InputSystem.GazeProvider.HeadMovementDirection.y,
                        z = CoreServices.InputSystem.GazeProvider.HeadMovementDirection.z
                    };
                headmovementdirection_list.Add(headmovementdirection_msg);

                // Waypoint count timeseries
                waypointcount_list.Add(waypointPlanner.listOfWaypoints.Count);
            }
        }

        timer++;
    }

    public void ExperimentData()
    {
        // Ros Frame
        // positive x is moving away from arm towards participant
        // Positive y is moving toward the rigth from the perspective of the particpant
        // Positive z is moving upwards from the robot base
        // ROS FRAME (x, y, z) = UNITY FRAME (z, -x , y)

        // Environment number    CONSTANT
        string EnvironmentName = EnvironmentsObject.transform.GetChild(0).gameObject.name[12].ToString();
        Debug.Log(EnvironmentName);

        // Environment condition    CONSTANT
        FindCondition();
        Debug.Log(Condition);

        // Environment control method    CONSTANT
        FindControlMethod();
        Debug.Log(ControlMethod);

        // Number of collisions    CONSTANT
        int AmountCollisions = CollisionCounter.counter;
        Debug.Log(AmountCollisions);

        // Time of human operation left over   CONSTANT
        float time =  TimerUI.transform.GetChild(0).GetChild(1).gameObject.GetComponent<Timer>().currentTime;
        Debug.Log(time);

        // Target End Effector transform in Franka Emika frame    CONSTANT
        EndEffectorPos PlanningTarget = TargetEndEffector.GetComponent<EndEffectorPos>();
        PoseMsg TargetEndEffectorPlacement = new PoseMsg()
        {
            position = new PointMsg(PlanningTarget.Position.x, PlanningTarget.Position.y, PlanningTarget.Position.z),
            orientation = new QuaternionMsg(PlanningTarget.Orientation.x, PlanningTarget.Orientation.y, PlanningTarget.Orientation.z, PlanningTarget.Orientation.w)
        };
        Debug.Log(TargetEndEffectorPlacement.position);
        Debug.Log(TargetEndEffectorPlacement.orientation);

        // End Effector transform at the end in Franka Emika frame    CONSTANT
        PoseMsg EndCoordinates = new PoseMsg()
        {
            position = new PointMsg(EndEffectorCoordinates.Position.x, EndEffectorCoordinates.Position.y, EndEffectorCoordinates.Position.z),
            orientation = new QuaternionMsg(EndEffectorCoordinates.Orientation.x, EndEffectorCoordinates.Orientation.y, EndEffectorCoordinates.Orientation.z, EndEffectorCoordinates.Orientation.w)
        };
        Debug.Log(EndCoordinates.position);
        Debug.Log(EndCoordinates.orientation);

        // Start coordinates In unity frame     CONSTANT
        Vector3 StartCoordinates = EnvironmentsScript.StartPositionCoordinates;
        Debug.Log(StartCoordinates);

        // Goal position in Unity frame     CONSTANT
        Vector3 GoalCoordinates = EnvironmentsScript.GoalPositionCoordinates;
        Debug.Log(GoalCoordinates);

        // Robot base in Unity frame        CONSTANT
        PoseMsg RobotBaseCoordinates = new PoseMsg()
        {
            position = new PointMsg(RobotBaseObject.transform.position.x, RobotBaseObject.transform.position.y, RobotBaseObject.transform.position.z),
            orientation = new QuaternionMsg(RobotBaseObject.transform.rotation.x, RobotBaseObject.transform.rotation.y, RobotBaseObject.transform.rotation.z, RobotBaseObject.transform.rotation.w)
        };
        Debug.Log(RobotBaseCoordinates);
        
        // Euclidean distance between target end effector and goal    CONSTANT
        float dist = Vector3.Distance(new Vector3(GoalCoordinates.z, -GoalCoordinates.x, GoalCoordinates.y), new Vector3((float)EndCoordinates.position.x, (float)EndCoordinates.position.y, (float)EndCoordinates.position.z));
        Debug.Log(dist);

        // Waypoints used for waypoint control in Franka Emika Frame    CONSTANT
        List<PoseMsg> WaypointsList = waypointPlanner.listOfWaypoints;
        Debug.Log(WaypointsList.Count);

        PoseArrayMsg waypoints = new PoseArrayMsg()
        {
            poses = WaypointsList.ToArray()
        };

        // Waypoint counter timeseries
        Debug.Log(waypointcount_list.Count);
        long[] WaypointCountArray = waypointcount_list.ToArray();

        // Time series end effector
        // Create function that tracks it from the moment start is entered
        Debug.Log(EndEffectorPositionList.Count);

        PoseStampedMsg[] EndEffectorPositionArray = EndEffectorPositionList.ToArray();

        // Time timeseries
        Debug.Log(time_list.Count);
        double[] TimeArray = time_list.ToArray();

        // Handtracking timeseries
        Debug.Log(handtracking_list.Count);
        PoseMsg[] HandTrackingArray = handtracking_list.ToArray();

        //Gazetracking timeseries
        Debug.Log(gazedirection_list.Count);
        PointMsg[] GazeDirectionArray = gazedirection_list.ToArray();
        Debug.Log(gazeorigin_list.Count);
        PointMsg[] GazeOriginArray = gazeorigin_list.ToArray();
        Debug.Log(gazetargetname_list.Count);
        string[] GazeTargetNameArray = gazetargetname_list.ToArray();
        Debug.Log(gazetargetposition_list.Count);
        PointMsg[] GazeTargetPositionArray = gazetargetposition_list.ToArray();

        //Headtracking timeseiers
        Debug.Log(headmovementdirection_list.Count);
        PointMsg[] HeadMovementDirectionArray = headmovementdirection_list.ToArray();
        Debug.Log(headvelocity_list.Count);
        PointMsg[] HeadVelocityArray = headvelocity_list.ToArray();

        // Trajectory in joint values and succes
        // Find it in waypoint trajectory or normal trajectory script
        if (ControlMethod == "Waypoint Control")
        {
            Debug.Log("In WC");
            Trajectory = waypointPlanner.TrajectoryData;
            Success = waypointPlanner.SuccessData;
            Debug.Log(Trajectory.points.Length);
            Debug.Log(Success);
        }

        else if (ControlMethod == "Command Control")
        {
            Debug.Log("In CC");
            Trajectory = trajectoryPlanner.TrajectoryData;
            Success = trajectoryPlanner.SuccessData;
            Debug.Log(Trajectory.points.Length);
            Debug.Log(Success);
        }
        
        else
        {
            Debug.Log("In else");
            Trajectory = new JointTrajectoryMsg();
            Success = false;
            Debug.Log(Trajectory.points.Length);
            Debug.Log(Success);
        }
        
        ExperimentDataMsg msg = new ExperimentDataMsg()
        {
            environment_name = EnvironmentName,
            condition_number = Condition,
            control_method = ControlMethod,
            collisions_amount = AmountCollisions,
            operation_time = time,
            target_end_effector_placement = TargetEndEffectorPlacement,
            end_effector_finishing_placement = EndCoordinates,
            start_coordinates = new PointMsg(StartCoordinates.z, -StartCoordinates.x, StartCoordinates.y), // Set into franka emika frame
            goal_coordinates = new PointMsg(GoalCoordinates.z, -GoalCoordinates.x, GoalCoordinates.y), // Set into franka emika frame
            robot_base_coordinates = RobotBaseCoordinates,
            euclidean_distance = dist,
            waypoints_placements = waypoints,
            waypointcount_list = WaypointCountArray,
            end_effector_position_list = EndEffectorPositionArray,
            time_list = TimeArray,
            handtracking_list = HandTrackingArray,
            gazedirection_list = GazeDirectionArray,
            gazeorigin_list = GazeOriginArray,
            gazetargetname_list = GazeTargetNameArray,
            gazetargetposition_list = GazeTargetPositionArray,
            headvelocity_list = HeadVelocityArray,
            headmovementdirection_list = HeadMovementDirectionArray,
            trajectory = Trajectory,
            success = Success
        };

        ros.Publish(ExperimentDataTopic, msg);
    }

    void FindControlMethod()
    {
        ControlMethod = "Error";

        if (ExperimentWaypointControlUI.activeSelf)
        {
            ControlMethod = "Waypoint Control";
        }

        if (ExperimentDirectControlUI.activeSelf)
        {
            ControlMethod = "Direct Control";
        }

        if (ExperimentCommandControlUI.activeSelf)
        {
            ControlMethod = "Command Control";
        }
    }

    void FindCondition()
    {
        Condition = "Error";

        if (EnvironmentsObject.transform.GetChild(0).GetChild(0).gameObject.activeSelf)
        {
            Condition = "Condition 1";
        }

        if (EnvironmentsObject.transform.GetChild(0).GetChild(1).gameObject.activeSelf)
        {
            Condition = "Condition 2";
        }

        if (EnvironmentsObject.transform.GetChild(0).GetChild(2).gameObject.activeSelf)
        {
            Condition = "Condition 3";
        }
    }

    public void StartEndEffectorStoring()
    {
        EndEffectorPositionList.Clear();
        time_list.Clear();
        handtracking_list.Clear();
        gazedirection_list.Clear();
        gazeorigin_list.Clear();
        gazetargetname_list.Clear();
        gazetargetposition_list.Clear();
        headmovementdirection_list.Clear();
        headvelocity_list.Clear();
        StoringEndEffectorPosition = true;
    }

    public void StopEndEffectorStoring()
    {
        StoringEndEffectorPosition = false;
    }
}
