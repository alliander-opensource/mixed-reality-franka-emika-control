using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using RosMessageTypes.Trajectory;
using RosMessageTypes.Std;
using RosMessageTypes.Mrirac;
using Unity.Robotics.ROSTCPConnector;

public class ExperimentDataMessage : MonoBehaviour
{
    ROSConnection ros;
    [SerializeField]
    private string ExperimentDataTopic;


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

    public EndEffectorPos EndEffectorCoordinates;
    public bool StoringEndEffectorPosition;
    public List<PoseStampedMsg> EndEffectorPositionList;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ExperimentDataMsg>(ExperimentDataTopic);

        EndEffectorCoordinates = EndEffectorObject.GetComponent<EndEffectorPos>();

        StoringEndEffectorPosition = false;
    }

    void Update()
    {
        if (StoringEndEffectorPosition)
        {
            PoseStampedMsg msg = new PoseStampedMsg()
            {
                pose = new PoseMsg()
                {
                    position = new PointMsg(EndEffectorCoordinates.Position.x, EndEffectorCoordinates.Position.y, EndEffectorCoordinates.Position.z),
                    orientation = new QuaternionMsg(EndEffectorCoordinates.Orientation.x, EndEffectorCoordinates.Orientation.y, EndEffectorCoordinates.Orientation.z, EndEffectorCoordinates.Orientation.w)
                }
            };
            EndEffectorPositionList.Add(msg);
        }
    }

    public void ExperimentData()
    {
        // Ros Frame
        // positive x is moving away from arm towards participant
        // Positive y is moving toward the rigth from the perspective of the particpant
        // Positive z is moving upwards from the robot base
        // ROS FRAME (x, y, z) = UNITY FRAME (z, -x , y)

        // Environment number
        string EnvironmentName = EnvironmentsObject.transform.GetChild(0).gameObject.name[12].ToString();
        Debug.Log(EnvironmentName);

        // Environment condition

        // Environment control method
        FindControlMethod();
        Debug.Log(ControlMethod);

        // Number of collisions
        int AmountCollisions = CollisionCounter.counter;
        Debug.Log(AmountCollisions);

        // Time of human operation
        float time =  TimerUI.transform.GetChild(0).GetChild(1).gameObject.GetComponent<Timer>().currentTime;
        Debug.Log(time);

        // Target End Effector transform in Franka Emika frame
        EndEffectorPos PlanningTarget = TargetEndEffector.GetComponent<EndEffectorPos>();
        PoseMsg TargetEndEffectorPlacement = new PoseMsg()
        {
            position = new PointMsg(PlanningTarget.Position.x, PlanningTarget.Position.y, PlanningTarget.Position.z),
            orientation = new QuaternionMsg(PlanningTarget.Orientation.x, PlanningTarget.Orientation.y, PlanningTarget.Orientation.z, PlanningTarget.Orientation.w)
        };
        Debug.Log(TargetEndEffectorPlacement.position);
        Debug.Log(TargetEndEffectorPlacement.orientation);

        // End Effector transform at the end in Franka Emika frame
        PoseMsg EndCoordinates = new PoseMsg()
        {
            position = new PointMsg(EndEffectorCoordinates.Position.x, EndEffectorCoordinates.Position.y, EndEffectorCoordinates.Position.z),
            orientation = new QuaternionMsg(EndEffectorCoordinates.Orientation.x, EndEffectorCoordinates.Orientation.y, EndEffectorCoordinates.Orientation.z, EndEffectorCoordinates.Orientation.w)
        };
        Debug.Log(EndCoordinates.position);
        Debug.Log(EndCoordinates.orientation);

        // Start coordinates In unity frame 
        Vector3 StartCoordinates = EnvironmentsScript.StartPositionCoordinates;
        Debug.Log(StartCoordinates);

        // Goal position in Unity frame 
        Vector3 GoalCoordinates = EnvironmentsScript.GoalPositionCoordinates;
        Debug.Log(GoalCoordinates);
        
        // Euclidean distance between target end effector and goal
        float dist = Vector3.Distance(new Vector3(GoalCoordinates.z, -GoalCoordinates.x, GoalCoordinates.y), new Vector3((float)EndCoordinates.position.x, (float)EndCoordinates.position.y, (float)EndCoordinates.position.z));
        Debug.Log(dist);

        // Waypoints used for waypoint control in Franka Emika Frame
        List<PoseMsg> WaypointsList = waypointPlanner.listOfWaypoints;
        Debug.Log(WaypointsList.Count);

        PoseArrayMsg waypoints = new PoseArrayMsg()
        {
            poses = WaypointsList.ToArray()
        };

        // Time series end effector
        // Create function that tracks it from the moment start is entered
        Debug.Log(EndEffectorPositionList.Count);

        // Trajectory in joint values and succes
        // Find it in waypoint trajectory or normal trajectory script
        if (ControlMethod == "Waypoint Control")
        {
            Debug.Log("In WC");
            JointTrajectoryMsg Trajectory = waypointPlanner.TrajectoryData;
            bool Success = waypointPlanner.SuccessData;
            Debug.Log(Trajectory.points.Length);
            Debug.Log(Success);
        }

        else if (ControlMethod == "Command Control")
        {
            Debug.Log("In CC");
            JointTrajectoryMsg Trajectory = trajectoryPlanner.TrajectoryData;
            bool Success = trajectoryPlanner.SuccessData;
            Debug.Log(Trajectory.points.Length);
            Debug.Log(Success);
        }
        
        else
        {
            Debug.Log("In else");
            JointTrajectoryMsg Trajectory = new JointTrajectoryMsg();
            bool Success = false;
            Debug.Log(Trajectory.points.Length);
            Debug.Log(Success);
        }
        
        ExperimentDataMsg msg = new ExperimentDataMsg()
        {
            environment_name = EnvironmentName,
            control_method = ControlMethod,
            operation_time = time,
            target_end_effector_placement = TargetEndEffectorPlacement,
            end_effector_finishing_placement = EndCoordinates,
            start_coordinates = new PointMsg(StartCoordinates.z, -StartCoordinates.x, StartCoordinates.y), // Set into franka emika frame
            goal_coordinates = new PointMsg(GoalCoordinates.z, -GoalCoordinates.x, GoalCoordinates.y), // Set into franka emika frame
            euclidean_distance = dist,
            waypoints_placements = waypoints
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

    public void StartEndEffectorStoring()
    {
        EndEffectorPositionList.Clear();
        StoringEndEffectorPosition = true;
    }
}
