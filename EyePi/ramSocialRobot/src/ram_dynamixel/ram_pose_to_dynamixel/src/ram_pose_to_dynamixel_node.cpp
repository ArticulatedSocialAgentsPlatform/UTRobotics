#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

std_msgs::Float64 poseJointAngle1;
std_msgs::Float64 poseJointAngle2;
std_msgs::Float64 poseJointAngle3;

// A callback function. Executed each time a new pose message arrives
void poseMessageReceived(const sensor_msgs::JointState &msg) {
    poseJointAngle1.data = msg.position[0];
    poseJointAngle2.data = msg.position[1] - 1.1;
    poseJointAngle3.data = (-1 * msg.position[2]) + 1.15;
}

int main(int argc, char **argv) {
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "ram_pose_to_dynamixel_node");
    ros::NodeHandle nh;

    // Create a subscriber object.
    ros::Subscriber sub = nh.subscribe("/ram/animation/generated/joint", 10, &poseMessageReceived);

    // Create the publisher objects.
    ros::Publisher poseJointAngle1Pub = nh.advertise<std_msgs::Float64>("/ram/dynamixel/pitch_joint/command", 100);
    ros::Publisher poseJointAngle2Pub = nh.advertise<std_msgs::Float64>("/ram/dynamixel/zoom_joint/command", 100);
    ros::Publisher poseJointAngle3Pub = nh.advertise<std_msgs::Float64>("/ram/dynamixel/nod_joint/command", 100);

    ros::Rate loop_rate(100);

    while (ros::ok()) {
        // Publish the message.
        poseJointAngle1Pub.publish(poseJointAngle1);
        poseJointAngle2Pub.publish(poseJointAngle2);
        poseJointAngle3Pub.publish(poseJointAngle3);
        ros::spinOnce();
        loop_rate.sleep();
    }
}


