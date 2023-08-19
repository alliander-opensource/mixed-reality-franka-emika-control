using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Mrirac;
using RosMessageTypes.Geometry;
using RosMessageTypes.Trajectory;
using RosMessageTypes.Std;
using System;
using System.Linq;

public class WaypointPlanner : MonoBehaviour
{
    private ROSConnection ros;

    [SerializeField]
    private string plannerServiceName;
    [SerializeField]
    private string executorServiceName;

    [SerializeField]
    private GameObject robot;
    [SerializeField]
    private GameObject root;


    [SerializeField]
    private GameObject targetObject;


    [SerializeField]
    private GameObject targetEndEffector1;
    // private GameObject targetEndEffector2;
    // private GameObject targetEndEffector3;

    // private GameObject waypoint1;
    // private GameObject waypoint2;
    // private GameObject waypoint3;
    SetTargetShaders setTargetShaders1;
    // SetTargetShaders setTargetShaders2;
    // SetTargetShaders setTargetShaders3;

    EndEffectorPos planningTarget1;
    // EndEffectorPos planningTarget2;
    // EndEffectorPos planningTarget3;

    public UpdateRobot robotUpdater;
    RobotManager robotManager;


    // Trajectory Lines
    bool showLines;
    [Header("Visualization Options")]
    [SerializeField]
    private GameObject linePrefab;
    [SerializeField]
    [Range(10.0f, 100.0f)]
    private float planVizualizationSpeed;
    static string[] linkLineTags = { "link_1", "link_2", "link_3", "link_4", "link_5", "link_6", "link_7" };
    GameObject[] joints = new GameObject[linkLineTags.Length];
    GameObject[] lines = new GameObject[linkLineTags.Length];
    LineRenderer[] lineRenderers = new LineRenderer[linkLineTags.Length];

    float[] lastTrajAngles;

    // Waypoint trajectory
    [SerializeField]
    private GameObject WaypointsGameObject;
    [SerializeField]
    private GameObject WaypointPrefab;
    public PoseArrayMsg waypoints;
    public List<PoseMsg> listOfWaypoints;

    public JointTrajectoryMsg TrajectoryData;
    public bool SuccessData;


    void Awake()
    {
        planningTarget1 = targetEndEffector1.GetComponent<EndEffectorPos>();
        // planningTarget2 = targetEndEffector2.GetComponent<EndEffectorPos>();
        // planningTarget3 = targetEndEffector3.GetComponent<EndEffectorPos>();

        robotUpdater = robot.GetComponent<UpdateRobot>();
        robotManager = robot.GetComponent<RobotManager>();

        setTargetShaders1 = targetObject.GetComponent<SetTargetShaders>();
        // setTargetShaders2 = waypoint2.GetComponent<SetTargetShaders>();
        // setTargetShaders3 = waypoint3.GetComponent<SetTargetShaders>();

    }

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<WaypointTrajectoryPlanRequest, WaypointTrajectoryPlanResponse>(plannerServiceName);
        ros.RegisterRosService<EmptyRequest, EmptyResponse>(executorServiceName);


        for (int i = 0; i < linkLineTags.Length; i++)
        {
            lines[i] = Instantiate(linePrefab);
            lineRenderers[i] = lines[i].GetComponent<LineRenderer>();
            joints[i] = GameObject.FindGameObjectWithTag(linkLineTags[i]);
        }

        TrajectoryData = new JointTrajectoryMsg();
        SuccessData = false;

    }

    public void CallTrajectoryPlanner()
    {
        waypoints.poses = listOfWaypoints.ToArray();

        WaypointTrajectoryPlanRequest request = new WaypointTrajectoryPlanRequest(waypoints);

        ros.SendServiceMessage<WaypointTrajectoryPlanResponse>(plannerServiceName, request, TrajectoryPlannerCallback);
    }


    void TrajectoryPlannerCallback(WaypointTrajectoryPlanResponse response)
    {
        if (response.success)
        {
            TrajectoryData = response.trajectory;
            SuccessData = response.success;

            robotUpdater.enabled = false;
            JointTrajectoryMsg trajectory = response.trajectory;

            JointTrajectoryPointMsg[] points = trajectory.points;

            StartCoroutine(ShowTrajectory(points, planVizualizationSpeed, endWait: 3.0f));
        }
        else
        {
            TrajectoryData = response.trajectory;
            SuccessData = response.success;

            Debug.Log("No motion plan found!");
            StartCoroutine(setTargetShaders1.ShowError());
        }
    }

    public IEnumerator ShowTrajectory(JointTrajectoryPointMsg[] points, float vizSpeed, float endWait)
    {
        //float[] prevJointAnglesDeg = robotManager.prevAnglesDeg;
        float[] prevJointAnglesDeg = robotManager.PreviousJointAnglesDeg;

        for (int pointIdx = 0; pointIdx < points.Length; pointIdx++)
        {

            JointTrajectoryPointMsg point = points[pointIdx];

            float[] jointAnglesDeg = point.positions.Select(angleRad => (float)angleRad * Mathf.Rad2Deg).ToArray();

            if (pointIdx == points.Length - 1)
            {
                lastTrajAngles = jointAnglesDeg;
            }

            // bool to check whether interpolation causes the angle to change more than 180deg, which results in strange visuals
            bool spin = false;

            int numOfInterpSteps = 5;
            for (int iter = 0; iter < numOfInterpSteps; iter++)
            {
                float[] interpolatedJointAngles = new float[point.positions.Length];
                for (int angleIdx = 0; angleIdx < interpolatedJointAngles.Length; angleIdx++)
                {
                    if (Mathf.Abs(prevJointAnglesDeg[angleIdx] - jointAnglesDeg[angleIdx]) > 180f)
                    {
                        spin = true;
                    }
                    interpolatedJointAngles[angleIdx] = Mathf.Lerp(prevJointAnglesDeg[angleIdx], jointAnglesDeg[angleIdx], iter * 0.2f);
                }
                if (spin)
                {
                    break;
                }
                robotManager.SetJointAngles(interpolatedJointAngles);
                yield return new WaitForSeconds(1 / vizSpeed);
            }

            if (showLines)
            {
                for (int lineRendererIdx = 0; lineRendererIdx < lineRenderers.Length; lineRendererIdx++)
                {
                    lineRenderers[lineRendererIdx].positionCount = lineRenderers[lineRendererIdx].positionCount + 1;
                    lineRenderers[lineRendererIdx].SetPosition(pointIdx, joints[lineRendererIdx].transform.position);
                }
            }
            robotManager.SetJointAngles(jointAnglesDeg);
            yield return new WaitForSeconds(1 / vizSpeed);

            prevJointAnglesDeg = jointAnglesDeg;

        }

        yield return new WaitForSeconds(endWait);

        for (int j = 0; j < lineRenderers.Length; j++)
        {
            lineRenderers[j].positionCount = 0;
        }
        robotUpdater.enabled = true;

    }

    public void CallTrajectoryExecutor()
    {
        ros.SendServiceMessage<EmptyResponse>(executorServiceName, new EmptyRequest(), (o) => Debug.Log("executed planned trajectory"));
    }

    public void ToggleLines() { showLines = !showLines; }

    public void addPoseToWaypointList()
    {

        PoseMsg new_waypoint = new PoseMsg()
        {
            position = new PointMsg(planningTarget1.Position.x, planningTarget1.Position.y, planningTarget1.Position.z),
            orientation = new QuaternionMsg(planningTarget1.Orientation.x, planningTarget1.Orientation.y, planningTarget1.Orientation.z, planningTarget1.Orientation.w)
        };

        listOfWaypoints.Add(new_waypoint);

        GameObject WaypointObject = Instantiate(WaypointPrefab);
        WaypointObject.name = System.Guid.NewGuid().ToString();
        WaypointObject.transform.SetParent(WaypointsGameObject.transform, false);
        WaypointObject.transform.position = new Vector3(targetEndEffector1.transform.position.x, targetEndEffector1.transform.position.y, targetEndEffector1.transform.position.z);

        

        
    }

    public void resetWaypointList()
    {
        // Debug.Log(listOfWaypoints.Count);
        listOfWaypoints.Clear();
        // Debug.Log(listOfWaypoints.Count);

        foreach (Transform WaypointTransform in WaypointsGameObject.transform)
        {
            Destroy(WaypointTransform.gameObject);
        }
    }
}