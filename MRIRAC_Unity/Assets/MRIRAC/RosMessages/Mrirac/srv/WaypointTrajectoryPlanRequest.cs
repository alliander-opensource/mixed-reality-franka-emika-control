//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Mrirac
{
    [Serializable]
    public class WaypointTrajectoryPlanRequest : Message
    {
        public const string k_RosMessageName = "mrirac_msgs/WaypointTrajectoryPlan";
        public override string RosMessageName => k_RosMessageName;

        public Geometry.PoseMsg[] waypoints;

        public WaypointTrajectoryPlanRequest()
        {
            this.waypoints = new Geometry.PoseMsg[0];
        }

        public WaypointTrajectoryPlanRequest(Geometry.PoseMsg[] waypoints)
        {
            this.waypoints = waypoints;
        }

        public static WaypointTrajectoryPlanRequest Deserialize(MessageDeserializer deserializer) => new WaypointTrajectoryPlanRequest(deserializer);

        private WaypointTrajectoryPlanRequest(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.waypoints, Geometry.PoseMsg.Deserialize, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.WriteLength(this.waypoints);
            serializer.Write(this.waypoints);
        }

        public override string ToString()
        {
            return "WaypointTrajectoryPlanRequest: " +
            "\nwaypoints: " + System.String.Join(", ", waypoints.ToList());
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