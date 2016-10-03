//
// Created by Bob on 008 8 3 2016.
//

#ifndef RAMSOCIALROBOT_RAM_ANIMATOR_NODE_HPP
#define RAMSOCIALROBOT_RAM_ANIMATOR_NODE_HPP

#include <ros/ros.h>
#include <cstdlib> //for random amount of eye blinks

// Messages
#include "std_msgs/Float64.h"
#include <ram_animation_msgs/Animation.h>
#include <ram_animation_msgs/Face.h>
#include <ram_output_msgs/GeneralFeedback.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>

// Animator constants
#include <ram_animation_msgs/animatorConstants.hpp>
// Feedback constants
#include <ram_output_msgs/feedbackConstants.hpp>
// Sequence timing
#include <ram_animator/_sequence_timing.hpp>

// The 'viewing angle' in rads of the eyes, used for the saccades
const float eyeAngleX = 15.0 / 180.0 * 3.14;
const float eyeAngleY = 20.0 / 180.0 * 3.14;

// The home position of the robot which would be a natural pose for it
float joint1HomeAngle = 0;
float joint2HomeAngle = 0.4 * 3.14;
float joint3HomeAngle = 0.35 * 3.14;
float homingTime = 1.0;

float saccadeCurrentAngle1;
float saccadeCurrentAngle2;
float saccadeCurrentAngle3;
float saccadeRequiredAngle1;
float saccadeRequiredAngle2;
float saccadeRequiredAngle3;

float reqPoseJointAngle1 = joint1HomeAngle;
float reqPoseJointAngle2 = joint2HomeAngle;
float reqPoseJointAngle3 = joint3HomeAngle;
float midiInputAngle1 = joint1HomeAngle;
float midiInputAngle2 = joint2HomeAngle;
float midiInputAngle3 = joint3HomeAngle;
float demoInputAngle1 = joint1HomeAngle;
float demoInputAngle2 = joint2HomeAngle;
float demoInputAngle3 = joint3HomeAngle;
int reqFaceEyePosX = 64;
int reqFaceEyePosY = 64;
int reqFaceEmotionState;
int reqFaceEmotionLevel;

double personLostMoment = 0;
double personFoundMoment = 0;

float outPoseJointAngle1 = 0;
float outPoseJointAngle2 = 0.25 * 3.14;
float outPoseJointAngle3 = 0;
float prevAngle1;
float prevAngle2;
float prevAngle3;
int outFaceEyePosX = 64;
int outFaceEyePosY = 64;
int outFaceEmotionState = RamAnimatorConstants::EmotionNeutral;
int outFaceEmotionLevel = 120;
int outFaceScreenBrightness = 64;
ram_animation_msgs::Face oldFaceMsg;

char status;
double secs;

bool gazeInProgress = false;
double gazeStartTime;
float gazeAngle1;
float gazeAngle2;
float gazeAngle3;
double gazeMoveTime;
float oldGazeAngle1;
float oldGazeAngle2;
float oldGazeAngle3;

////////////////////////////////////////////////////////////
// Natural animation parameters for nodding and eye blinking
double anim_alfa = 3.14 / 70;  // Base amplitude of breathing in rads
double breathDiv = 4;  // Divide the current animation freq by this number for breathing freq
double blinkInterval = 5.0; // Time in seconds between blinking of the robot
double blinkDuration = 0.5;  // Time in seconds to blink
///////////////////////////////////////////////////
// Natural animation inits
bool demo = false;
double demoStartTime = 0;
double demoPlayTime = 0;
float preDemoAngle1;
float preDemoAngle2;
float preDemoAngle3;
float summedAngle1;
float summedAngle2;
float summedAngle3;

bool naturalMovement = true;
bool gazeEyes = true;
bool followEyes = false;
float breath_add_joint2 = 0;
float breath_add_joint3 = 0;
double blinkStartTime = 0;
float breathPhase = 0;
float prevBreathPhase = 0;    // To enable change of the breath freq without changing the phase of the breathing animation
int blinkTimes = 1;
double timeSinceLastBlink = 0;    // Timer helps with the timing of the blinking of the eyes of the robot
double lastBlinkTime = 0;
bool blinking = false;        // Determines if the blink animation is now playing
double blink_f = 1.00 / (double(blinkDuration)) * 2 * 3.14;
float compression = 0;

// Inputs can be: default / saccade / blobTrack / external
char input = RamAnimatorConstants::InputDefault;
////////////////////////////////////////////////////////////
// NOTE: nodeUpdateRate, animFreqMin and animFreqMax have been moved to sequences/_sequence_timing.cpp
////////////////////////////////////////////////////////////
// ANIMATION CONFIGURABLE PARAMETERS
// Determines how much of the old position is used for determining the new value (higher is more smoothing)
double smoothingMin = 0.0;
double smoothingMax = 0.98;
// Amplitude of sequence animations in rads
double animAngle = 3.14 / 15;
// ANIMATION INITS
double animationPhase = 0;
double animFreq = 2.133333 * 2 * 3.14;
double smoothing = 0.04;
float add_angle1 = 0;
float add_angle2 = 0;
float add_angle3 = 0;
float animation_joint1 = 0;
float animation_joint2 = 0;
float animation_joint3 = 0;
float saved_animation_joint = 0;
double animationStartTime = 0;
bool animationInProgress = false;
char sequence;
std::string animationId;
unsigned char emotion;
bool newAnimation = true;
int nodAmount = 0;
ram_output_msgs::GeneralFeedback generalFeedbackMsg;
bool progressFeedbackSend = false;

// ROS publishers/subscribers
ros::Publisher facePublisher;
ros::Publisher jointPublisher;
ros::Publisher generalFeedbackPublisher;
ros::Subscriber animationSubscriber;
ros::Subscriber faceSubscriber;
ros::Subscriber jointSubscriber;

/**
 * Callback used to process incoming animation messages
 */
void _animationMessageReceived(const ram_animation_msgs::Animation &msg) ;

/**
 * Callback used to process incoming face messages
 */
void _faceMessageReceived(const ram_animation_msgs::Face &msg);

/**
 * Callback to process incoming joint messages
 */
void _jointMessageReceived(const sensor_msgs::JointState &msg) ;

/**
 * Progress the animation
 */
void animate();

/**
 * Demo sequence, which is preprogrammed
 */
void animateDemo() ;

/**
 * A full range soft knee compressor for the movement
 */
float compressMovement(float position, float minThres, float maxThres, float minAngle, float maxAngle, float maxRatio) ;

/**
 * Estimates the sequence time
 */
float estimateAnimationTime(char sequence) ;

/**
 * Look at a certain spot, by updating the gaze
 */
void gaze();

/**
 * Process the movement compression
 */
void processCompression();

/**
 * Limit the output angles
 */
void processLimiting();

/**
 * Process the movement smoothing
 * For every cycle, the new control value is a certain ratio of the old one, this value is set here.
 */
void processSmoothing();

/**
 * Processes the saccade eye movements
 */
void processSaccade() ;

/**
 * Publish a general feedback message
 * Use the constants from feedbackConstant.h as parameters
 * @param type
 * @param id
 * @param ack
 * @param reason
 */
void publishGeneralFeedback(int type, std::string id, int ack, int reason);

/**
 * Reset the internal movement state
 */
void resetMovements(bool resetBreathing);

/**
 * Start the gaze
 */
void startGaze(float angle1, float angle2, float angle3, double moveTime);

/**
 * Start moving to the robot's default pose
 */
void startHoming() ;

/**
 * Stops the blink action
 */
void stopBlink() ;

/**
 * Function to sweep a param during a given time
 */
double sweepParam(double currentTime, double startTime, double endTime, double min, double max) ;

/**
 * Start moving to the midi input pose
 */
void transitionToMidiInput();

/**
 * Updates the natural movement of the robot (breathing)
 */
void updateBreath();

/**
 * Updates the blinking of the eyes
 */
void updateBlink();

/**
 * Main program
 */
int main(int argc, char **argv);


#endif //RAMSOCIALROBOT_RAM_ANIMATOR_NODE_HPP
