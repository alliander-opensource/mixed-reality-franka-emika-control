#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <std_srvs/Empty.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "mrirac_msgs/PickAndPlaceAction.h"
#include "mrirac_msgs/PoseCorrectionAction.h"
#include "mrirac_msgs/StartPickAndPlace.h"
#include "mrirac_msgs/PlanPickAndPlace.h"
#include "mrirac_lib/mrirac_lib.h"

class PickAndPlaceNode
{
private:
    ros::NodeHandle node_handle_;
    actionlib::SimpleActionServer<mrirac_msgs::PickAndPlaceAction> pick_and_place_server_;

    actionlib::SimpleActionClient<mrirac_msgs::PoseCorrectionAction> pose_correction_client_;
    actionlib::SimpleActionClient<mrirac_msgs::PickAndPlaceAction> pick_and_place_client_;

    ros::ServiceServer plan_pick_and_place_server_;
    ros::ServiceServer start_pick_and_place_server_;
    ros::ServiceServer abort_pick_and_place_server_;
    ros::ServiceServer open_gripper_server_;
    ros::ServiceServer close_gripper_server_;
    mrirac_msgs::PickAndPlaceResult pick_and_place_result_;

    const std::string kPlanningGroup_ = "fr3_arm";
    moveit::planning_interface::MoveGroupInterface move_group_interface_;
    moveit::planning_interface::MoveGroupInterface planner_interface_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    actionlib::SimpleActionClient<franka_gripper::MoveAction> *fr3_move_finger_client_;
    actionlib::SimpleActionClient<franka_gripper::GraspAction> *fr3_grasp_finger_client_; 
    moveit::planning_interface::MoveGroupInterface::Plan pre_grasp_motion_plan_;
    moveit::planning_interface::MoveGroupInterface::Plan grasp_motion_plan_;
    moveit::planning_interface::MoveGroupInterface::Plan pre_grasp_2_motion_plan_;
    moveit::planning_interface::MoveGroupInterface::Plan pre_place_motion_plan_;
    moveit::planning_interface::MoveGroupInterface::Plan place_motion_plan_;
    bool trajectory_planned_ = false;

    const double kFingerMax_ = 6400;

    bool simulation;

    void ExecuteCallback(const mrirac_msgs::PickAndPlaceGoalConstPtr &goal);
    bool Start(mrirac_msgs::StartPickAndPlace::Request &req, mrirac_msgs::StartPickAndPlace::Response &res);
    bool Abort(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool OpenGripper(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool CloseGripper(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    bool PlanPickAndPlace(mrirac_msgs::PlanPickAndPlace::Request &req, mrirac_msgs::PlanPickAndPlace::Response &res);

public:
    PickAndPlaceNode(const std::string &pick_and_place_server_name, const ros::NodeHandle &node_handle);
    ~PickAndPlaceNode();
};

PickAndPlaceNode::PickAndPlaceNode(const std::string &pick_and_place_server_name, const ros::NodeHandle &node_handle) : node_handle_(node_handle),
                                                                                                                        pick_and_place_server_(node_handle_, pick_and_place_server_name, boost::bind(&PickAndPlaceNode::ExecuteCallback, this, _1), false),
                                                                                                                        pose_correction_client_("/mrirac_pose_correction/pose_correction", true),
                                                                                                                        pick_and_place_client_("/mrirac_pick_and_place_Fr3/pick_and_place", true),
                                                                                                                        move_group_interface_(kPlanningGroup_),
                                                                                                                        planner_interface_(kPlanningGroup_)
{
    ros::param::param<bool>("~simulation", simulation, false);
    fr3_move_finger_client_ = new actionlib::SimpleActionClient<franka_gripper::MoveAction>("/franka_gripper/move", false);
    fr3_grasp_finger_client_ = new actionlib::SimpleActionClient<franka_gripper::GraspAction>("/franka_gripper/grasp", false);
    ROS_INFO("starting action server");
    pick_and_place_server_.start();
    ROS_INFO("started action server");
    plan_pick_and_place_server_ = node_handle_.advertiseService("plan", &PickAndPlaceNode::PlanPickAndPlace, this);
    start_pick_and_place_server_ = node_handle_.advertiseService("start", &PickAndPlaceNode::Start, this);
    abort_pick_and_place_server_ = node_handle_.advertiseService("abort", &PickAndPlaceNode::Abort, this);
    open_gripper_server_ = node_handle_.advertiseService("open_gripper", &PickAndPlaceNode::OpenGripper, this);
    close_gripper_server_ = node_handle_.advertiseService("close_gripper", &PickAndPlaceNode::CloseGripper, this);
    move_group_interface_.setPlannerId("RRTstar");
    move_group_interface_.setPlanningTime(10.0f);
    planner_interface_.setPlannerId("RRTstar");
    planner_interface_.setPlanningTime(10.0f);

}

PickAndPlaceNode::~PickAndPlaceNode()
{
}

bool PickAndPlaceNode::PlanPickAndPlace(mrirac_msgs::PlanPickAndPlace::Request &req, mrirac_msgs::PlanPickAndPlace::Response &res)
{
    bool success;
    moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
    const moveit::core::JointModelGroup *joint_model_group =
        move_group_interface_.getRobotModel()->getJointModelGroup(kPlanningGroup_);

    std::vector<std::string> joint_names = joint_model_group->getJointModelNames();
    // removing unactuated joints
    //joint_names.erase(joint_names.begin());
    //joint_names.pop_back();

    moveit::core::RobotState robot_state(move_group_interface_.getRobotModel());
    robot_state.setToDefaultValues();


    success = RobotMovements::PlanMovementToPose(req.pre_grasp_pose, move_group_interface_, motion_plan);
    if (success)
    {
        pre_grasp_motion_plan_ = motion_plan;
        ROS_INFO("pre_grasp_motion_plan_");
        res.pre_grasp_trajectory = motion_plan.trajectory_.joint_trajectory;
    }
    else
    {
        res.success = false;
        return true;
    }
    RobotMovements::SetPlannerStartState(motion_plan.trajectory_.joint_trajectory.points.back().positions, joint_names, planner_interface_, robot_state);

    success = RobotMovements::PlanMovementToPose(req.grasp_pose, planner_interface_, motion_plan);
    if (success)
    {
        grasp_motion_plan_ = motion_plan;
        ROS_INFO("grasp_motion_plan_");
        res.grasp_trajectory = motion_plan.trajectory_.joint_trajectory;
    }
    else
    {
        res.success = false;
        return true;
    }
    RobotMovements::SetPlannerStartState(motion_plan.trajectory_.joint_trajectory.points.back().positions, joint_names, planner_interface_, robot_state);

    success = RobotMovements::PlanMovementToPose(req.pre_grasp_pose, planner_interface_, motion_plan);
    if (success)
    {
        pre_grasp_2_motion_plan_ = motion_plan;
        ROS_INFO("pre_grasp_2_plan_");
        res.pre_grasp_trajectory = motion_plan.trajectory_.joint_trajectory;
    }
    else
    {
        res.success = false;
        return true;
    }
    // RobotMovements::SetPlannerStartState(motion_plan.trajectory_.joint_trajectory.points.back().positions, joint_names, planner_interface_, robot_state);

    // success = RobotMovements::PlanMovementToPose(req.pre_place_pose, planner_interface_, motion_plan);
    // if (success)
    // {
    //     pre_place_motion_plan_ = motion_plan;
    //     ROS_INFO("pre_place_pose_plan_"); 
    //     res.pre_place_trajectory = motion_plan.trajectory_.joint_trajectory;
    // }
    // else
    // {
    //     res.success = false;
    //     return true;
    // }
    RobotMovements::SetPlannerStartState(motion_plan.trajectory_.joint_trajectory.points.back().positions, joint_names, planner_interface_, robot_state);

    success = RobotMovements::PlanMovementToPose(req.place_pose, planner_interface_, motion_plan);
    if (success)
    {
        place_motion_plan_ = motion_plan;
        ROS_INFO("place_pose_plan_");
        res.place_trajectory = motion_plan.trajectory_.joint_trajectory;
    }
    else
    {
        res.success = false;
        return true;
    }
    RobotMovements::SetPlannerStartState(motion_plan.trajectory_.joint_trajectory.points.back().positions, joint_names, planner_interface_, robot_state);

    res.success = true;
    trajectory_planned_ = true;

    return true;
}

bool PickAndPlaceNode::Start(mrirac_msgs::StartPickAndPlace::Request &req, mrirac_msgs::StartPickAndPlace::Response &res)
{
    mrirac_msgs::PickAndPlaceGoal goal;
    goal.pre_grasp_pose = req.pre_grasp_pose;
    goal.grasp_pose = req.grasp_pose;
    goal.pre_place_pose = req.pre_place_pose;
    goal.place_pose = req.place_pose;

    pick_and_place_client_.sendGoal(goal);
    pick_and_place_client_.waitForResult();
    res.success = true;
    return true;
}

bool PickAndPlaceNode::Abort(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Aborting Pick and Place");
    move_group_interface_.stop();
    pick_and_place_client_.cancelAllGoals();
    return true;
}

bool PickAndPlaceNode::OpenGripper(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Opening gripper");
    RobotMovements::Fr3_GripperAction(0.08, fr3_move_finger_client_);
    return true;
}

bool PickAndPlaceNode::CloseGripper(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Closing gripper");
    RobotMovements::Fr3_GripperGraspAction(5, fr3_grasp_finger_client_);
    return true;
}

void PickAndPlaceNode::ExecuteCallback(const mrirac_msgs::PickAndPlaceGoalConstPtr &goal)
{
    if (!simulation)
    {
        RobotMovements::Fr3_GripperAction(0.08, fr3_move_finger_client_);
    }

    bool action_success = true;
    if (trajectory_planned_)
    {
        RobotMovements::ExecutePlannedTrajectory(move_group_interface_, pre_grasp_motion_plan_, goal->pre_grasp_pose, !simulation, pose_correction_client_);
    }

    else
    {
        action_success = RobotMovements::PickAndPlaceMovement(goal->pre_grasp_pose, move_group_interface_, pick_and_place_server_, !simulation, pose_correction_client_);
    }

    if (!action_success)
    {
        trajectory_planned_ = false;
        pick_and_place_result_.success = false;
        ROS_INFO("Pick and Place Action: Failed");
        // set the action state to failed
        pick_and_place_server_.setAborted(pick_and_place_result_);
        return;
    }
    std::vector<std::string> object_ids;
    object_ids.push_back("pickObject");
    planning_scene_interface_.removeCollisionObjects(object_ids);


    if (trajectory_planned_)
    {
        RobotMovements::ExecutePlannedTrajectory(move_group_interface_, grasp_motion_plan_, goal->grasp_pose, !simulation, pose_correction_client_);
    }

    else
    {
        action_success = RobotMovements::PickAndPlaceMovement(goal->grasp_pose, move_group_interface_, pick_and_place_server_, !simulation, pose_correction_client_);
    }
    
    if (!action_success)
    {
        trajectory_planned_ = false;
        pick_and_place_result_.success = false;
        ROS_INFO("Pick and Place Action: Failed");
        // set the action state to failed
        pick_and_place_server_.setAborted(pick_and_place_result_);
        return;
    }

    if (!simulation)
    {
        RobotMovements::Fr3_GripperGraspAction(20, fr3_grasp_finger_client_);
    }

    if (trajectory_planned_)
    {
        RobotMovements::ExecutePlannedTrajectory(move_group_interface_, pre_grasp_2_motion_plan_, goal->pre_grasp_pose, !simulation, pose_correction_client_);
    }

    else
    {
        action_success = RobotMovements::PickAndPlaceMovement(goal->pre_grasp_pose, move_group_interface_, pick_and_place_server_, !simulation, pose_correction_client_);
    }

    if (!action_success)
    {
        trajectory_planned_ = false;
        pick_and_place_result_.success = false;
        ROS_INFO("Pick and Place Action: Failed");
        // set the action state to failed
        pick_and_place_server_.setAborted(pick_and_place_result_);
        return;
    }
    
    // if (trajectory_planned_)
    // {
    //     RobotMovements::ExecutePlannedTrajectory(move_group_interface_, pre_place_motion_plan_, goal->pre_place_pose, !simulation, pose_correction_client_);
    // }

    // else
    // {
    //     action_success = RobotMovements::PickAndPlaceMovement(goal->pre_place_pose, move_group_interface_, pick_and_place_server_, !simulation, pose_correction_client_);
    // }

    // if (!action_success)
    // {
    //     trajectory_planned_ = false;
    //     pick_and_place_result_.success = false;
    //     ROS_INFO("Pick and Place Action: Failed");
    //     // set the action state to failed
    //     pick_and_place_server_.setAborted(pick_and_place_result_);
    //     return;
    // }

    if (trajectory_planned_)
    {
        RobotMovements::ExecutePlannedTrajectory(move_group_interface_, place_motion_plan_, goal->place_pose, !simulation, pose_correction_client_);
    }

    else
    {
        action_success = RobotMovements::PickAndPlaceMovement(goal->place_pose, move_group_interface_, pick_and_place_server_, !simulation, pose_correction_client_);
    }
    
    if (!action_success)
    {
        trajectory_planned_ = false;
        pick_and_place_result_.success = false;
        ROS_INFO("Pick and Place Action: Failed");
        // set the action state to failed
        pick_and_place_server_.setAborted(pick_and_place_result_);
        return;
    }

    if (!simulation)
    {
        RobotMovements::Fr3_GripperAction(0.08, fr3_move_finger_client_);
    }

    if (action_success)
    {
        pick_and_place_result_.success = true;
        ROS_INFO("Pick and Place Action: Succeeded");
        // set the action state to failed
        pick_and_place_server_.setSucceeded(pick_and_place_result_);
    }
    else
    {
        pick_and_place_result_.success = false;
        ROS_INFO("Pick and Place Action: Failed");
        // set the action state to failed
        pick_and_place_server_.setAborted(pick_and_place_result_);
    }

    trajectory_planned_ = false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mrirac_pick_and_place_Fr3");
    ros::NodeHandle node_handle("~");

    ros::AsyncSpinner spinner(0);
    spinner.start();
    std::string pick_and_place_server_name = "pick_and_place";
    PickAndPlaceNode node(pick_and_place_server_name, node_handle);

    ros::waitForShutdown();

    return 0;
}