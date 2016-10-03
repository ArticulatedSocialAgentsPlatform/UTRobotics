//
// Created by Bob on 008 8 3 2016.
//

#ifndef RAMSOCIALROBOT_RAM_HMMM_NODE_HPP
#define RAMSOCIALROBOT_RAM_HMMM_NODE_HPP

// C++ includes
#include <limits>
#include <cmath>
#include <signal.h>

// Default ROS includes
#include <ros/ros.h>

// OpenCV includes
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// Dynamic reconfiguration includes
#include <dynamic_reconfigure/server.h>
#include <ram_hmmm/RamHmmmConfig.h>

// Used messages
#include <ram_animation_msgs/Animation.h>
#include <ram_animation_msgs/EnabledModule.h>
#include <ram_input_msgs/Emotion.h>
#include <ram_input_msgs/Saliency.h>
#include <ram_input_msgs/Sequence.h>
#include <ram_output_msgs/GeneralFeedback.h>
#include <ram_output_msgs/GazeFeedback.h>
#include <ram_output_msgs/PredictFeedback.h>
#include <ram_animation_msgs/Face.h>
#include <sensor_msgs/JointState.h>

// Animator constants
#include <ram_animation_msgs/animatorConstants.hpp>

// Feedback constants
#include <ram_output_msgs/feedbackConstants.hpp>

// Input constants
#include <ram_input_msgs/inputConstants.hpp>

// Sequence timing
#include <ram_animator/_sequence_timing.hpp>

/**
 * Method to handle incoming emotion messages
 * @param emotionMessage
 */
void _emotionMessageCallback(const ram_input_msgs::EmotionConstPtr &emotionMessage);

/**
 * Enabled flag callback
 * @param enabledModuleMessage
 */
void _enabledModuleCallback(const ram_animation_msgs::EnabledModuleConstPtr &enabledModuleMessage) ;

/**
 * The callback function which is executed when a Saliency message arrives
 * @param saliencyMessage
 */
void _saliencyMessageCallback(const ram_input_msgs::SaliencyConstPtr &saliencyMessage) ;

/**
 * Calculates the eye-X-position based on the salient X
 * @param xScale
 * @return int
 */
int calculateEyeXPosition(const double &xScale);

/**
 * Calculates the eye-Y-position based on the salient Y
 * @param yScale
 * @return int
 */
int calculateEyeYPosition(const double &yScale);

/**
 * Calculates the X-position based on the salient X
 * @param xScale
 * @return double
 */
double calculateXPosition(const double &xScale) ;

/**
 * Calculates the Y2-position based on the salient Y
 * @param yScale
 * @return double
 */
double calculateY1Position(const double &yScale) ;

/**
 * Calculates the Y1-position based on the salient Y
 * @param yScale
 * @return double
 */
double calculateY2Position(const double &yScale);

/**
 * Computes the saliency and thus the gaze direction
 */
void computeSaliency() ;

/**
 * Computes the required emotion
 */
void computeEmotion();

/**
 * Computes the new joint states according to the saliency
 */
void computeJointStates();

/**
 * Creates the default messages
 */
void createDefaultMessages();

/**
 * Progress the animation
 */
void emotionProgress();

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
 * Publish the gaze feedback message
 */
void publishGazeFeedback();

/**
 * Publish a nack message, including reason
 * @param id
 * @param reason
 */
void publishNackFeedback(std::string id, int reason);

/**
 * Send progress end updates
 */
void publishProgressEnd();

/**
 * Sends progress start update
 */
void publishProgressStart();

/**
 * Convert saliency weight to circle
 * @param val
 * @return int
 */
int saliencyWeightToSize(double val);

/**
 * Convert the Salient value back to the -1, 1 scale
 * @param size
 * @param scaleFactor
 * @return float
 */
float scaleSize(int size, float scaleFactor);

/**
 * Set the current HMMM emotion in the message
 */
void setHmmmEmotion();

/**
 * Sweep a param for in a certain time
 * @return int
 */
int sweepEmotionLevel();

/**
 * Main program
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv) ;

#endif //RAMSOCIALROBOT_RAM_HMMM_NODE_HPP
