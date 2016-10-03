#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

float poseJointAngle1;
float poseJointAngle2;
float poseJointAngle3;

void poseMessageReceived(const sensor_msgs::JointState& msg) {
	poseJointAngle1 = msg.position[0];
	poseJointAngle2 = msg.position[1];
	poseJointAngle3 = msg.position[2];
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle nh;
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1000);

    ros::Subscriber sub = nh.subscribe("/jointState", 1000, &poseMessageReceived);
    ros::Rate loop_rate(1000);

    while (ros::ok()) {
	sensor_msgs::JointState joint_state;
        
        joint_state.name.resize(3);
        joint_state.position.resize(3);
        joint_state.name[0] ="joint1";
        joint_state.position[0] = poseJointAngle1;
        joint_state.name[1] ="joint2";
        joint_state.position[1] = poseJointAngle2;
        joint_state.name[2] ="joint3";
        joint_state.position[2] = poseJointAngle3;

        joint_state.header.stamp = ros::Time::now();

        //send the joint state and transform	
	joint_pub.publish(joint_state);	
		
	ros::spinOnce();
        // This will adjust as needed per iteration
        loop_rate.sleep();
    }

    return 0;
}
