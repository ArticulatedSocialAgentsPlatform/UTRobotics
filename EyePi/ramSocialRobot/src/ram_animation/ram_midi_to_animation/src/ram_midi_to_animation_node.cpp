// Default ROS include
#include <ros/ros.h>

// For atan2 and sqrt functions
#include <cmath>

// Used messages
#include <ram_input_msgs/Midi.h>
#include <ram_input_msgs/Emotion.h>
#include <ram_output_msgs/MidiOutput.h>
#include <ram_animation_msgs/Face.h>
#include <ram_animation_msgs/Animation.h>
#include <ram_animation_msgs/EnabledModule.h>
#include <sensor_msgs/JointState.h>

// Animator constants
#include <ram_animation_msgs/animatorConstants.hpp>

// Enabled flag
bool nodeEnabled = false;

// For controller button behavior
ros::Publisher midiOutPublisher;
ram_output_msgs::MidiOutput midiOutput;
int savedButton;

// Global variables
float frequency;
float smoothing;
float compression;
bool demo;
double emotionAngle;
int arousal, valence, previousArousal = 0, previousValence = 0;

// Initialized global variables
char input = RamAnimatorConstants::InputDefault;
char sequence = RamAnimatorConstants::SequenceEmpty;
float poseJointAngle1 = 0;
float poseJointAngle2 = 0.25 * 3.14;
float poseJointAngle3 = 0.50;
// An extra slider to manipulate two joints simultaneously
float poseJointAngle4 = 0;
char xPosition = 64;
char yPosition = 64;
char emotionState = RamAnimatorConstants::EmotionNeutral;
char faceEmotionLevel = 64;
char screenBrightness = 64;
// Switches between using valence and arousal or just pressing buttons and setting intensity for emotions
bool emotionSliders = false;

/**
 * Enabled flag callback
 */
void enabledModuleCallback(const ram_animation_msgs::EnabledModuleConstPtr &enabledModuleMessage) {
    nodeEnabled = enabledModuleMessage->midi;
}

/**
 * Callback that handles new midi messages
 */
void midiMessageReceived(const ram_input_msgs::Midi &msg) {
    ROS_DEBUG_STREAM("Chan: "<<msg.chan<<", val: "<<msg.val);

    midiOutput.button = savedButton;
    midiOutput.on = false;

    switch (msg.chan) {
        case 0:
            // Mapping midi 0-127 input to a range for this robot from 0 to .5*pi*rad
            poseJointAngle2 = float(msg.val) / 127 * 0.5 * 3.1416;
            break;
        case 1:
            // Mapping midi 0-127 input to a range for this robot from 0 to .5*pi*rad
            poseJointAngle3 = (127.0 - float(msg.val)) / 127 * 0.5 * 3.1416;
            break;
        case 2:
            poseJointAngle4 = (float(msg.val) - 63.5) / 63.5 * 0.5 * 3.1416;
            break;
        case 4:
            arousal = msg.val - 64;
            emotionSliders = true;
            break;
        case 5:
            valence = msg.val - 64;
            emotionSliders = true;
            break;
        case 6:
            faceEmotionLevel = msg.val;
            emotionSliders = false;
            break;
        case 7:
            yPosition = msg.val;
            break;
        case 16:
            // Mapping midi 0-127 input to a range for this robot from -.5*pi*rad to .5*pi*rad
            poseJointAngle1 = -(float(msg.val) - 63.5) / 63.5 * 0.5 * 3.1416;
            break;
        case 17:
            frequency = float(msg.val) / 127;
            break;
        case 18:
            // Map to 0.2 for better response
            smoothing = 0.8 + (float(msg.val) / 127 / 5);
            break;
        case 19:
            compression = float(msg.val) / 127;
            break;
        case 22:
            screenBrightness = msg.val;
            break;
        case 23:
            xPosition = msg.val;
            break;
        case 34:
            sequence = RamAnimatorConstants::SequenceNodding;
            savedButton = msg.chan;
            break;
        case 35:
            if (sequence == RamAnimatorConstants::SequenceEmpty && msg.val == 127) {
                input = RamAnimatorConstants::InputDefault;
            }
            sequence = RamAnimatorConstants::SequenceEmpty;
            demo = false;
            savedButton = 0;
            break;
        case 36:
            input = RamAnimatorConstants::InputDefault;
            demo = true;
            savedButton = msg.chan;
            break;
        case 37:
            emotionState = RamAnimatorConstants::EmotionFriendly;
            emotionSliders = false;
            savedButton = 37;
            break;
        case 38:
            emotionState = RamAnimatorConstants::EmotionNeutral;
            emotionSliders = false;
            savedButton = 38;
            break;
        case 39:
            emotionState = RamAnimatorConstants::EmotionExcited;
            emotionSliders = false;
            savedButton = 39;
            break;
        case 41: //Start button
            if (msg.val == 127) {
                //system("rosnode kill /ram_midi_driver");
                //system("rosbag play /home/pi/ros/test.bag");
            }
            break;
        case 42: //Stop button
            if (msg.val == 127) {
                //system("rosrun ram_midi_driver mididriver.py 3");
            }
            break;
        case 50:
            sequence = RamAnimatorConstants::SequenceShaking;
            savedButton = msg.chan;
            break;
        case 51:
            sequence = RamAnimatorConstants::SequencePointingLeft;
            savedButton = msg.chan;
            break;
        case 52:
            sequence = RamAnimatorConstants::SequencePointingRight;
            savedButton = msg.chan;
            break;
        case 54:
            emotionState = RamAnimatorConstants::EmotionSleepy;
            emotionSliders = false;
            savedButton = 54;
            break;
        case 55:
            emotionState = RamAnimatorConstants::EmotionAmazed;
            emotionSliders = false;
            savedButton = 55;
            break;
        case 66:
            sequence = RamAnimatorConstants::SequenceDancing;
            savedButton = msg.chan;
            break;
        case 67:
            sequence = RamAnimatorConstants::SequencePointingMiddle;
            savedButton = msg.chan;
            break;
        case 68:
            sequence = RamAnimatorConstants::SequenceEmpty;
            demo = false;
            input = RamAnimatorConstants::InputSaccade;
            savedButton = msg.chan;
            break;
        case 69:
            emotionState = RamAnimatorConstants::EmotionCrash; //Go into the system crashing emotion state
            savedButton = 69;
            break;
        case 70:
            emotionState = RamAnimatorConstants::EmotionSad;
            emotionSliders = false;
            savedButton = 70;
            break;
        case 71:
            emotionState = RamAnimatorConstants::EmotionAngry;
            emotionSliders = false;
            savedButton = 71;
            break;
        default:
            break;
    }

    if (savedButton != midiOutput.button) {
        midiOutPublisher.publish(midiOutput);

        midiOutput.button = savedButton;
        midiOutput.on = true;
        midiOutPublisher.publish(midiOutput);
    }

    // Update emotionState if the sliders are used
    if (emotionSliders) {
        emotionAngle = atan2(arousal, valence);
        if (emotionAngle > 0 && emotionAngle < 3.14 / 3) { emotionState = RamAnimatorConstants::EmotionExcited; }
        if (emotionAngle > 3.14 / 3 && emotionAngle < 3.14 / 3 * 2) { emotionState = RamAnimatorConstants::EmotionAmazed; }
        if (emotionAngle > 3.14 / 3 * 2 && emotionAngle < 3.14) { emotionState = RamAnimatorConstants::EmotionAngry; }
        if (emotionAngle > -3.14 && emotionAngle < -3.14 / 3 * 2) { emotionState = RamAnimatorConstants::EmotionSad; }
        if (emotionAngle > -3.14 / 3 * 2 && emotionAngle < -3.14 / 3) { emotionState = RamAnimatorConstants::EmotionSleepy; }
        if (emotionAngle > -3.14 / 3 && emotionAngle < 0) { emotionState = RamAnimatorConstants::EmotionNeutral; }
        faceEmotionLevel = int(2 * sqrt(valence * valence + arousal * arousal));

        //Limiting
        if (faceEmotionLevel < 0) faceEmotionLevel = 0;
        if (faceEmotionLevel > 127) faceEmotionLevel = 127;
    }
}

int main(int argc, char **argv) {
    // Initialize the ROS system.
    ros::init(argc, argv, "ram_midi_to_animation_node");

    // Establish this program as a ROS node.
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    // Register enabled subscriber
    ros::Subscriber enabledSubscriber = nh.subscribe("/ram/animation/enabled/nodes", 1, &enabledModuleCallback);

    // Register midi subscriber & midi output publisher
    ros::Subscriber sub = nh.subscribe("/ram/input/midi", 100, &midiMessageReceived);
    midiOutPublisher = nh.advertise<ram_output_msgs::MidiOutput>("/ram/output/midiout", 100);

    // Register normal publishers
    ros::Publisher animationPublisher = nh.advertise<ram_animation_msgs::Animation>("/ram/animation/desired/animation", 100);
    ros::Publisher facePublisher = nh.advertise<ram_animation_msgs::Face>("/ram/animation/desired/face", 100);
    ros::Publisher jointPublisher = nh.advertise<sensor_msgs::JointState>("/ram/animation/desired/joint", 100);

    // Create HMMM publisher
    ros::Publisher emotionHmmmPublisher = nh.advertise<ram_input_msgs::Emotion>("/ram/animation/hmmm/emotion", 100);

    // Start main program
    while (ros::ok()) {

        // Check if the node is enabled
        if (!nodeEnabled) {

            // Forward arousal/valence to HMMM node, but only on change
            if (previousArousal != arousal || previousValence != valence) {
                ram_input_msgs::Emotion emotionMsg;
                emotionMsg.arousal = ((float) arousal) / 64.0;
                emotionMsg.arousalWeight = 40;
                emotionMsg.valence = ((float) valence) / 64.0;
                emotionMsg.valenceWeight = 40;
                emotionHmmmPublisher.publish(emotionMsg);

                previousArousal = arousal;
                previousValence = valence;
            }

            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }

        // Fill the messages
        // Animation
        ram_animation_msgs::Animation animationMsg;
        animationMsg.id = "midi";
        animationMsg.sequence = sequence;
        animationMsg.frequency = frequency;
        animationMsg.smoothing = smoothing;
        animationMsg.compression = compression;
        animationMsg.demo = demo;
        animationMsg.input = input;

        // Face
        ram_animation_msgs::Face faceMsg;
        faceMsg.emotionState = emotionState;
        faceMsg.emotionLevel = faceEmotionLevel;
        faceMsg.eyePosX = xPosition;
        faceMsg.eyePosY = yPosition;
        faceMsg.screenBrightness = screenBrightness;

        // Joint
        sensor_msgs::JointState jointMsg;
        jointMsg.name.resize(3);
        jointMsg.position.resize(3);
        jointMsg.name[0] = "joint1";
        jointMsg.name[1] = "joint2";
        jointMsg.name[2] = "joint3";
        jointMsg.position[0] = poseJointAngle1;
        jointMsg.position[1] = poseJointAngle2 + poseJointAngle4;
        jointMsg.position[2] = poseJointAngle3 + poseJointAngle4;

        // Publish the messages
        animationPublisher.publish(animationMsg);
        jointPublisher.publish(jointMsg);
        facePublisher.publish(faceMsg);

        // Wait until it's time for another iteration.
        ros::spinOnce();
        loop_rate.sleep();
    }
}
