#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <ram_animation_msgs/Face.h>
#include <ram_output_msgs/MidiOutput.h>
#include <dynamixel_msgs/JointState.h>

// Animator constants
#include <ram_animation_msgs/animatorConstants.hpp>

volatile double pitchError;
volatile double zoomError;
volatile double nodError;

std_msgs::Float64 motorMsg;
ram_animation_msgs::Face faceMsg;

// For controller button behavior
ros::Publisher midiOutPublisher;
ram_output_msgs::MidiOutput midiOutput;
volatile double secs;

void pitchStateReceived(const dynamixel_msgs::JointState msg) {
    pitchError = msg.error;
}

void zoomStateReceived(const dynamixel_msgs::JointState msg) {
    zoomError = msg.error;
}

void nodStateReceived(const dynamixel_msgs::JointState msg) {
    nodError = msg.error;
}

void toggleButton(){
    double current = ros::Time::now().toSec();
    if((secs + 0.3) < current){
        midiOutput.on = !midiOutput.on;
        midiOutPublisher.publish(midiOutput);
        secs = current;
    }
}

int main(int argc, char **argv) {
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "ram_safe_shutdown_node");
    ros::NodeHandle nh;

    // Update once in a second
    ros::Rate rosRate(1);

    // Kill controlling/publishing nodes
    system("rosnode kill ram_animator_node");

    // Set face to sad
    ros::Publisher faceCommand = nh.advertise<ram_animation_msgs::Face>("/ram/animation/generated/face", 100);
    faceMsg.eyePosX = 64;
    faceMsg.eyePosY = 64;
    faceMsg.emotionState = RamAnimatorConstants::EmotionSad;
    faceMsg.emotionLevel = 127;
    faceMsg.screenBrightness = 127;
    while (faceCommand.getNumSubscribers() < 1) {
        ros::spinOnce();
    }
    faceCommand.publish(faceMsg);
    ros::spinOnce();

    // Kill dynamixel node
    system("rosnode kill ram_pose_to_dynamixel_node");

    // Restart dynamixel controllers, do not wait for the first one
    system("rosnode kill /ram/dynamixel/dynamixel_manager");
    system("roslaunch ram_dynamixel_config shutdown_manager.launch &");
    system("roslaunch ram_dynamixel_config shutdown_controller.launch");

    // Create the publisher objects.
    ros::Publisher pitchCommand = nh.advertise<std_msgs::Float64>("/ram/dynamixel/pitch_joint/command", 100);
    ros::Publisher zoomCommand = nh.advertise<std_msgs::Float64>("/ram/dynamixel/zoom_joint/command", 100);
    ros::Publisher nodCommand = nh.advertise<std_msgs::Float64>("/ram/dynamixel/nod_joint/command", 100);

    // Create midioutput publisher
    midiOutPublisher = nh.advertise<ram_output_msgs::MidiOutput>("/ram/output/midiout", 100);
    midiOutput.button = 42;
    midiOutput.on = false;
    midiOutPublisher.publish(midiOutput);
    secs = ros::Time::now().toSec();
    toggleButton();

    // Create subscriber object
    ros::Subscriber pitchState = nh.subscribe("/ram/dynamixel/pitch_joint/state", 100, &pitchStateReceived);
    ros::Subscriber zoomState = nh.subscribe("/ram/dynamixel/zoom_joint/state", 100, &zoomStateReceived);
    ros::Subscriber nodState = nh.subscribe("/ram/dynamixel/nod_joint/state", 100, &nodStateReceived);

    while (pitchCommand.getNumSubscribers() < 1 ||
           zoomCommand.getNumSubscribers() < 1 ||
           nodCommand.getNumSubscribers() < 1) {
        toggleButton();
        rosRate.sleep();
        ros::spinOnce();
    }

    motorMsg.data = -6;
    pitchCommand.publish(motorMsg);
    // Wait at least 5 second for this to complete
    ros::Duration(5).sleep();

    toggleButton();
    rosRate.sleep();
    while (fabs(pitchError) > 0.05) {
        toggleButton();
        ros::spinOnce();
        rosRate.sleep();
    }

    motorMsg.data = -1;
    nodCommand.publish(motorMsg);

    toggleButton();
    rosRate.sleep();
    while (fabs(nodError) > 0.05) {
        toggleButton();
        ros::spinOnce();
        rosRate.sleep();
    }

    motorMsg.data = -2;
    zoomCommand.publish(motorMsg);

    toggleButton();
    rosRate.sleep();
    while (fabs(zoomError) > 0.05) {
        toggleButton();
        ros::spinOnce();
        rosRate.sleep();
    }

    system("sudo shutdown -h now");

    while(ros::ok()){
        toggleButton();
        ros::spinOnce();
        rosRate.sleep();
    }
}


