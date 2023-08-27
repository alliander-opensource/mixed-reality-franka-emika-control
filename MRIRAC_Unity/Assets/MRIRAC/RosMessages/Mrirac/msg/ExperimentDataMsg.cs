//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Mrirac
{
    [Serializable]
    public class ExperimentDataMsg : Message
    {
        public const string k_RosMessageName = "mrirac_msgs/ExperimentData";
        public override string RosMessageName => k_RosMessageName;

        public string environment_name;
        public string condition_number;
        public string control_method;
        public long collisions_amount;
        public double operation_time;
        public Geometry.PoseMsg target_end_effector_placement;
        public Geometry.PoseMsg end_effector_finishing_placement;
        public Geometry.PointMsg start_coordinates;
        public Geometry.PointMsg goal_coordinates;
        public double euclidean_distance;
        public Geometry.PoseArrayMsg waypoints_placements;
        public Geometry.PoseStampedMsg[] end_effector_position_list;
        public Trajectory.JointTrajectoryMsg trajectory;
        public bool success;

        public ExperimentDataMsg()
        {
            this.environment_name = "";
            this.condition_number = "";
            this.control_method = "";
            this.collisions_amount = 0;
            this.operation_time = 0.0;
            this.target_end_effector_placement = new Geometry.PoseMsg();
            this.end_effector_finishing_placement = new Geometry.PoseMsg();
            this.start_coordinates = new Geometry.PointMsg();
            this.goal_coordinates = new Geometry.PointMsg();
            this.euclidean_distance = 0.0;
            this.waypoints_placements = new Geometry.PoseArrayMsg();
            this.end_effector_position_list = new Geometry.PoseStampedMsg[0];
            this.trajectory = new Trajectory.JointTrajectoryMsg();
            this.success = false;
        }

        public ExperimentDataMsg(string environment_name, string condition_number, string control_method, long collisions_amount, double operation_time, Geometry.PoseMsg target_end_effector_placement, Geometry.PoseMsg end_effector_finishing_placement, Geometry.PointMsg start_coordinates, Geometry.PointMsg goal_coordinates, double euclidean_distance, Geometry.PoseArrayMsg waypoints_placements, Geometry.PoseStampedMsg[] end_effector_position_list, Trajectory.JointTrajectoryMsg trajectory, bool success)
        {
            this.environment_name = environment_name;
            this.condition_number = condition_number;
            this.control_method = control_method;
            this.collisions_amount = collisions_amount;
            this.operation_time = operation_time;
            this.target_end_effector_placement = target_end_effector_placement;
            this.end_effector_finishing_placement = end_effector_finishing_placement;
            this.start_coordinates = start_coordinates;
            this.goal_coordinates = goal_coordinates;
            this.euclidean_distance = euclidean_distance;
            this.waypoints_placements = waypoints_placements;
            this.end_effector_position_list = end_effector_position_list;
            this.trajectory = trajectory;
            this.success = success;
        }

        public static ExperimentDataMsg Deserialize(MessageDeserializer deserializer) => new ExperimentDataMsg(deserializer);

        private ExperimentDataMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.environment_name);
            deserializer.Read(out this.condition_number);
            deserializer.Read(out this.control_method);
            deserializer.Read(out this.collisions_amount);
            deserializer.Read(out this.operation_time);
            this.target_end_effector_placement = Geometry.PoseMsg.Deserialize(deserializer);
            this.end_effector_finishing_placement = Geometry.PoseMsg.Deserialize(deserializer);
            this.start_coordinates = Geometry.PointMsg.Deserialize(deserializer);
            this.goal_coordinates = Geometry.PointMsg.Deserialize(deserializer);
            deserializer.Read(out this.euclidean_distance);
            this.waypoints_placements = Geometry.PoseArrayMsg.Deserialize(deserializer);
            deserializer.Read(out this.end_effector_position_list, Geometry.PoseStampedMsg.Deserialize, deserializer.ReadLength());
            this.trajectory = Trajectory.JointTrajectoryMsg.Deserialize(deserializer);
            deserializer.Read(out this.success);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.environment_name);
            serializer.Write(this.condition_number);
            serializer.Write(this.control_method);
            serializer.Write(this.collisions_amount);
            serializer.Write(this.operation_time);
            serializer.Write(this.target_end_effector_placement);
            serializer.Write(this.end_effector_finishing_placement);
            serializer.Write(this.start_coordinates);
            serializer.Write(this.goal_coordinates);
            serializer.Write(this.euclidean_distance);
            serializer.Write(this.waypoints_placements);
            serializer.WriteLength(this.end_effector_position_list);
            serializer.Write(this.end_effector_position_list);
            serializer.Write(this.trajectory);
            serializer.Write(this.success);
        }

        public override string ToString()
        {
            return "ExperimentDataMsg: " +
            "\nenvironment_name: " + environment_name.ToString() +
            "\ncondition_number: " + condition_number.ToString() +
            "\ncontrol_method: " + control_method.ToString() +
            "\ncollisions_amount: " + collisions_amount.ToString() +
            "\noperation_time: " + operation_time.ToString() +
            "\ntarget_end_effector_placement: " + target_end_effector_placement.ToString() +
            "\nend_effector_finishing_placement: " + end_effector_finishing_placement.ToString() +
            "\nstart_coordinates: " + start_coordinates.ToString() +
            "\ngoal_coordinates: " + goal_coordinates.ToString() +
            "\neuclidean_distance: " + euclidean_distance.ToString() +
            "\nwaypoints_placements: " + waypoints_placements.ToString() +
            "\nend_effector_position_list: " + System.String.Join(", ", end_effector_position_list.ToList()) +
            "\ntrajectory: " + trajectory.ToString() +
            "\nsuccess: " + success.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}