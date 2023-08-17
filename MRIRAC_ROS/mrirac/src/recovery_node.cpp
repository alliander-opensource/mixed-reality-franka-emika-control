#include <ros/ros.h>
#include <std_msgs/String.h>
#include <franka_msgs/ErrorRecoveryActionGoal.h>
#include <ros/console.h>
#include <stdlib.h>

int main(int argc, char** argv)
{
    //rostopic pub -1 /franka_control/error_recovery/goal franka_msgs/ErrorRecoveryActionGoal "{}"

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Error))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    // Initialize the ROS node
    ros::init(argc, argv, "recovery_node");

    // Create a ROS node handle
    ros::NodeHandle node_handle("~");

    // Create a publisher that publishes messages of type std_msgs::String on the topic "my_topic"
    ros::Publisher pub = node_handle.advertise<franka_msgs::ErrorRecoveryActionGoal>("/franka_control/error_recovery/goal", 10);

    // Set the loop rate (10 Hz in this example)
    ros::Rate loop_rate(30);

    while (ros::ok())
    {
        // Create a message object
        franka_msgs::ErrorRecoveryActionGoal msg;
 
        // Publish the message
        pub.publish(msg);

        // Print the message to the console
        // ROS_INFO("Published error recovery");

        // Spin once to process callbacks
        ros::spinOnce();

        // Sleep to maintain the loop rate
        loop_rate.sleep();

    }

    return 0;
}
