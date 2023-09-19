#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <std_msgs/String.h>
#include <shape_msgs/Mesh.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/PositionConstraint.h>
#include <geometry_msgs/Vector3.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>

#include "mrirac_msgs/TrajectoryPlan.h"
#include "mrirac_msgs/WaypointTrajectoryPlan.h"
#include "mrirac_msgs/MeshObstacle.h"
#include "mrirac_msgs/MeshObstacles.h"
#include "mrirac_msgs/PoseCorrectionAction.h"
#include "mrirac_lib/mrirac_lib.h"

class TrajectoryPlannerNode
{
private:
  ros::NodeHandle node_handle_;

  ros::ServiceServer trajectory_server_;
  ros::ServiceServer execute_server_;
  ros::ServiceServer waypoint_server_;
  ros::ServiceServer execute_waypoint_server_;
  ros::ServiceServer clear_obstacles_server_;
  ros::ServiceServer home_service_;
  ros::ServiceServer startPosition_service_;
  ros::ServiceServer set_standard_planner_server_;
  ros::ServiceServer set_RRTConnect_planner_server_;
  ros::ServiceServer set_RRTStar_planner_server_;
  ros::ServiceServer set_workspace_constraint_service_;
  ros::ServiceServer clear_workspace_constraint_service_;

  ros::Subscriber target_pose_subscriber_;
  ros::Subscriber waypoint_subscriber_;
  ros::Subscriber hologram_obstacle_sub_;
  ros::Subscriber spatial_obstacle_sub_;

  ros::Publisher n_obstacles_pub_;

  const std::string kPlanningGroup_ = "fr3_manipulator";
  moveit::planning_interface::MoveGroupInterface move_group_interface_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  bool trajectory_planned_ = false;
  moveit::planning_interface::MoveGroupInterface::Plan current_plan_;

  std::vector<std::string> hologram_mesh_obstacles_;

  const std::string remove_string_ = "remove";

  geometry_msgs::Pose target_pose_;
  geometry_msgs::Pose pre_grasp_pose_;
  geometry_msgs::Pose grasp_pose_;
  geometry_msgs::PoseArray waypoints_;

  bool simulation;

  actionlib::SimpleActionClient<mrirac_msgs::PoseCorrectionAction> pose_correction_action_client_;

  bool PlanTrajectory(mrirac_msgs::TrajectoryPlan::Request &req, mrirac_msgs::TrajectoryPlan::Response &res);
  bool ExecuteTrajectory(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool PlanWaypoints(mrirac_msgs::WaypointTrajectoryPlan::Request &req, mrirac_msgs::WaypointTrajectoryPlan::Response &res);
  bool ExecuteWaypoints(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool ClearObstacles(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool SetStandardPlanner(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool SetRRTConnectPlanner(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool SetRRTStarPlanner(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  void TargetPoseCallback(geometry_msgs::Pose target_pose);
  void WaypointCallback(geometry_msgs::PoseArray waypoints);
  void UpdateHologramObstacles(const mrirac_msgs::MeshObstacles msg);
  void UpdateSpatialObstacles(const mrirac_msgs::MeshObstacle msg);
  bool HomeArm(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool StartPositionArm(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool SetWorkspaceConstraint(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool ClearWorkspaceConstraint(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  bool SetJointspaceConstraint();

public:
  TrajectoryPlannerNode(const ros::NodeHandle &node_handle);
  ~TrajectoryPlannerNode();
};

TrajectoryPlannerNode::TrajectoryPlannerNode(const ros::NodeHandle &node_handle) : node_handle_(node_handle), pose_correction_action_client_("/mrirac_pose_correction/pose_correction", true), move_group_interface_(kPlanningGroup_)

{
  ros::param::param<bool>("~simulation", simulation, false);

  if (!simulation)
  {
    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    pose_correction_action_client_.waitForServer(); // will wait for infinite time
    ROS_INFO("Action server started");
  }

  trajectory_server_ = node_handle_.advertiseService("plan_trajectory", &TrajectoryPlannerNode::PlanTrajectory, this);
  execute_server_ = node_handle_.advertiseService("execute_trajectory", &TrajectoryPlannerNode::ExecuteTrajectory, this);
  waypoint_server_ = node_handle_.advertiseService("plan_waypoints", &TrajectoryPlannerNode::PlanWaypoints, this);
  execute_waypoint_server_ = node_handle_.advertiseService("execute_waypoints", &TrajectoryPlannerNode::ExecuteWaypoints, this);

  clear_obstacles_server_ = node_handle_.advertiseService("clear_obstacles", &TrajectoryPlannerNode::ClearObstacles, this);
  home_service_ = node_handle_.advertiseService("home_arm", &TrajectoryPlannerNode::HomeArm, this);
  startPosition_service_ = node_handle_.advertiseService("start_position_arm", &TrajectoryPlannerNode::StartPositionArm, this);

  set_standard_planner_server_ = node_handle_.advertiseService("set_standard_planner", &TrajectoryPlannerNode::SetStandardPlanner, this);
  set_RRTConnect_planner_server_ = node_handle_.advertiseService("set_RRTConnect_planner", &TrajectoryPlannerNode::SetRRTConnectPlanner, this);
  set_RRTStar_planner_server_ = node_handle_.advertiseService("set_RRTStar_planner", &TrajectoryPlannerNode::SetRRTStarPlanner, this);
  set_workspace_constraint_service_ = node_handle_.advertiseService("set_workspace_constraint", &TrajectoryPlannerNode::SetWorkspaceConstraint, this);
  clear_workspace_constraint_service_ = node_handle_.advertiseService("clear_workspace_constraint", &TrajectoryPlannerNode::ClearWorkspaceConstraint, this);

  target_pose_subscriber_ = node_handle_.subscribe("unity_target_pose", 100, &TrajectoryPlannerNode::TargetPoseCallback, this);
  waypoint_subscriber_ = node_handle_.subscribe("unity_waypoints", 100, &TrajectoryPlannerNode::WaypointCallback, this);
  hologram_obstacle_sub_ = node_handle_.subscribe("unity_hologram_obstacles", 100, &TrajectoryPlannerNode::UpdateHologramObstacles, this);
  spatial_obstacle_sub_ = node_handle_.subscribe("unity_spatial_obstacles", 100, &TrajectoryPlannerNode::UpdateSpatialObstacles, this);

  n_obstacles_pub_ = node_handle_.advertise<std_msgs::String>("n_obstacles", 100);

  move_group_interface_.setPlanningTime(10.0f);
  move_group_interface_.setMaxAccelerationScalingFactor(0.3f);
  move_group_interface_.setMaxVelocityScalingFactor(0.3f);
}

TrajectoryPlannerNode::~TrajectoryPlannerNode()
{
}

bool TrajectoryPlannerNode::PlanTrajectory(mrirac_msgs::TrajectoryPlan::Request &req, mrirac_msgs::TrajectoryPlan::Response &res)
{
  ROS_INFO("received pose");
  target_pose_ = req.target_pose;
  moveit::planning_interface::MoveGroupInterface::Plan motion_plan;

  bool success = RobotMovements::PlanMovementToPose(req.target_pose, move_group_interface_, motion_plan);

  if (success)
  {
    res.trajectory = motion_plan.trajectory_.joint_trajectory;
    res.success = true;

    current_plan_ = motion_plan;
    trajectory_planned_ = true;
    return true;
  }
  else
  {
    res.trajectory = trajectory_msgs::JointTrajectory();
    res.success = false;
    return true;
  }
}

bool TrajectoryPlannerNode::ExecuteTrajectory(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  if (trajectory_planned_)
  {
    RobotMovements::ExecutePlannedTrajectory(move_group_interface_, current_plan_, target_pose_, !simulation, pose_correction_action_client_);
    trajectory_planned_ = false;
  }
  else
  {
    moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
    RobotMovements::PlanMovementToPose(target_pose_, move_group_interface_, motion_plan);
    RobotMovements::ExecutePlannedTrajectory(move_group_interface_, motion_plan, target_pose_, !simulation, pose_correction_action_client_);
  }
  return true;
}

bool TrajectoryPlannerNode::PlanWaypoints(mrirac_msgs::WaypointTrajectoryPlan::Request &req, mrirac_msgs::WaypointTrajectoryPlan::Response &res)
{
  // Store the received waypoints
  waypoints_ = req.waypoints;
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.insert(waypoints.end(), waypoints_.poses.begin(), waypoints_.poses.end());

  // Plan the trajectory passing through the waypoints
  moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
  moveit_msgs::RobotTrajectory robot_trajectory;
  std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_vector;
  bool overall_succes = false;

  unsigned int vecSize = waypoints.size();

  move_group_interface_.setPlannerId("RRTConnect");

  for(unsigned int i = 0; i < vecSize; i++)
  {
    moveit::planning_interface::MoveGroupInterface::Plan temp_motion_plan;
    bool success = RobotMovements::PlanMovementToPose(waypoints[i], move_group_interface_, temp_motion_plan);

    if (success)
    {
      std::vector<trajectory_msgs::JointTrajectoryPoint> temp_vector;
      temp_vector.insert(temp_vector.begin(), std::begin(temp_motion_plan.trajectory_.joint_trajectory.points), std::end(temp_motion_plan.trajectory_.joint_trajectory.points));
      trajectory_vector.insert(trajectory_vector.end(), temp_vector.begin(), temp_vector.end());

      RobotMovements::ExecutePlannedTrajectory(move_group_interface_, temp_motion_plan, waypoints[i], !simulation, pose_correction_action_client_);

      overall_succes = true;
    }

    else
    {
      overall_succes = false;
      break;
    }

  };

  if (overall_succes)
  {
    unsigned int n = trajectory_vector.size();

    for(unsigned int j = 0; j < n; j++)
    {
      robot_trajectory.joint_trajectory.points.push_back(trajectory_vector[j]);
    }
    
    motion_plan.trajectory_ = robot_trajectory;


    res.trajectory = motion_plan.trajectory_.joint_trajectory;
    res.success = true;

    current_plan_ = motion_plan;
    trajectory_planned_ = true;
  }
  else
  {
    res.trajectory = trajectory_msgs::JointTrajectory();
    res.success = false;
  }

  return true;
}

bool TrajectoryPlannerNode::ExecuteWaypoints(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  if (trajectory_planned_)
  {
    RobotMovements::ExecutePlannedTrajectory(move_group_interface_, current_plan_, target_pose_, !simulation, pose_correction_action_client_);
    trajectory_planned_ = false;

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.insert(waypoints.end(), waypoints_.poses.begin(), waypoints_.poses.end());

    // Optionally, apply pose correction action after each waypoint execution
    for (const auto& waypoint : waypoints)
    {
      mrirac_msgs::PoseCorrectionGoal goal;
      goal.target_pose = waypoint;
      pose_correction_action_client_.sendGoal(goal);
      pose_correction_action_client_.waitForResult();
    }
  }
  else
  {
    ROS_WARN("No trajectory is planned.");
  }

  return true;
}

bool TrajectoryPlannerNode::ClearObstacles(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  ROS_INFO("clearing planning scene");
  auto obstacles = planning_scene_interface_.getObjects();

  std::vector<std::string> to_remove;

  for (auto const &obstacle : obstacles)
  {
    to_remove.push_back(obstacle.first);
  }

  planning_scene_interface_.removeCollisionObjects(to_remove);

  return true;
}

bool TrajectoryPlannerNode::SetStandardPlanner(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  move_group_interface_.setPlannerId("RRT");

  return true;
}

bool TrajectoryPlannerNode::SetRRTConnectPlanner(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  move_group_interface_.setPlannerId("RRTConnect");

  return true;
}

bool TrajectoryPlannerNode::SetRRTStarPlanner(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  move_group_interface_.setPlannerId("RRTstar");

  return true;
}

bool TrajectoryPlannerNode::SetWorkspaceConstraint(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  moveit_msgs::Constraints constraints;
  moveit_msgs::PositionConstraint position_constraint;
  geometry_msgs::Vector3 target_point_offset;
  shape_msgs::SolidPrimitive box;
  geometry_msgs::Pose pose;

  constraints.name = "Workspace constraint for the experiment";

  position_constraint.header.frame_id = "fr3_link0";
  position_constraint.link_name = "fr3_link8";

  target_point_offset.x = 0.01;
  target_point_offset.y = 0.01;
  target_point_offset.z = 0.01;
  position_constraint.target_point_offset = target_point_offset;

  box.type = 1;
  box.dimensions.push_back(0.8);  // x
  box.dimensions.push_back(2);    // y
  box.dimensions.push_back(0.8);  // z
  position_constraint.constraint_region.primitives.push_back(box);

  pose.position.x = 0.6;
  pose.position.y = 0;
  pose.position.z = 0.4;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  position_constraint.constraint_region.primitive_poses.push_back(pose);

  constraints.position_constraints.push_back(position_constraint);

  move_group_interface_.setPathConstraints(constraints);

  ROS_INFO("set the constraint");

  return true;
}

bool TrajectoryPlannerNode::ClearWorkspaceConstraint(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  move_group_interface_.clearPathConstraints();

  return true;
}

void TrajectoryPlannerNode::TargetPoseCallback(geometry_msgs::Pose target_pose)
{
  target_pose_ = target_pose;
}

void TrajectoryPlannerNode::WaypointCallback(geometry_msgs::PoseArray waypoints)
{
  waypoints_ = waypoints;
}

void TrajectoryPlannerNode::UpdateHologramObstacles(const mrirac_msgs::MeshObstacles msg)
{

  planning_scene_interface_.removeCollisionObjects(hologram_mesh_obstacles_);

  std::vector<std::string> new_obstacles;
  hologram_mesh_obstacles_ = new_obstacles;

  std::vector<moveit_msgs::CollisionObject> collision_objects;

  for (auto mesh_obstacle : msg.mesh_obstacles)
  {
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface_.getPlanningFrame();
    collision_object.id = mesh_obstacle.name;
    hologram_mesh_obstacles_.push_back(mesh_obstacle.name);
    collision_object.meshes.push_back(mesh_obstacle.mesh);
    collision_object.mesh_poses.push_back(mesh_obstacle.mesh_pose);
    collision_object.operation = collision_object.ADD;

    collision_objects.push_back(collision_object);
  }

  planning_scene_interface_.addCollisionObjects(collision_objects);

  std_msgs::String n_obstacles_msg;
  n_obstacles_msg.data = std::to_string(planning_scene_interface_.getObjects().size());
  n_obstacles_pub_.publish(n_obstacles_msg);
}

void TrajectoryPlannerNode::UpdateSpatialObstacles(const mrirac_msgs::MeshObstacle msg)
{

  if (msg.action == remove_string_)
  {
    std::vector<std::string> to_remove = {msg.name};
    planning_scene_interface_.removeCollisionObjects(to_remove);
    return;
  }

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_interface_.getPlanningFrame();
  collision_object.id = msg.name;
  collision_object.meshes.push_back(msg.mesh);
  collision_object.mesh_poses.push_back(msg.mesh_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  ROS_INFO("adding spatial obstacle %s", msg.name.c_str());
  planning_scene_interface_.addCollisionObjects(collision_objects);
}

bool TrajectoryPlannerNode::HomeArm(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  ROS_INFO("sending to home");
  move_group_interface_.setNamedTarget("ready");
  moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
  bool success = (move_group_interface_.plan(motion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("planning complete");
  if (success)
  {
    move_group_interface_.execute(motion_plan);
    return true;
  }
  else
  {
    return false;
  }
}

bool TrajectoryPlannerNode::StartPositionArm(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  ROS_INFO("sending to starting position");
  move_group_interface_.setNamedTarget("experiment_start");
  moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
  bool success = (move_group_interface_.plan(motion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("planning complete");
  if (success)
  {
    move_group_interface_.execute(motion_plan);
    return true;
  }
  else
  {
    return false;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mrirac_trajectory_planner");
  ros::NodeHandle node_handle("~");

  ros::AsyncSpinner spinner(0);
  spinner.start();

  TrajectoryPlannerNode node(node_handle);

  ros::waitForShutdown();

  return 0;
}