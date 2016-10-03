/**
 * This program behaves as an mixing agent for different external inputs.
 *`It takes the input and roughly translates it into direct controls which are used by the original animator.
 *
 * Saliency <ram_input_msgs::Saliency.h>:
 *  - LocationX: [-1,1]
 *  - LocationY: [-1,1]
 *  - Weight: 0 - 10000
 *
 * Copyright B. van de Vijver, Robotics and Mechatronics chair, University of Twente
 */

#include "ram_hmmm_node.hpp"

// Configurable ROS parameters
#define ROS_RATE 50

// Deployed lens
//#define NO_LENS
#define WIDE_LENS

// Window names
const char *incoming_saliency_map_window = "Incoming saliency";
const char *calculated_saliency_map_window = "Calculated saliency";

////////////////////////////////////////////////
// Configuration
////////////////////////////////////////////////

// Configurable show parameters
bool SHOW_INCOMING_SALIENCY_MAP = false;    // When set to try, the incoming location of the saliency will be shown
bool SHOW_CALCULATED_SALIENCY_MAP = false;  // When set to try, the calculated location of the saliency will be shown

// Configurable Saliency parameters
int SALIENCY_X_RES = 640;                     // X-resolution of the input
int SALIENCY_X_RES_HALF = SALIENCY_X_RES / 2; // Half of X-resolution of the input
int SALIENCY_Y_RES = 640;                     // Y-resolution of the input
int SALIENCY_Y_RES_HALF = SALIENCY_Y_RES / 2; // Half of Y-resolution of the input
#ifdef NO_LENS
double SALIENCY_X_TO_JOINT = 1.10;          // X-resolution to the motor resolution (no lens)
#endif
#ifdef WIDE_LENS
double SALIENCY_X_TO_JOINT = 1.30;          // X-resolution to the motor resolution (wide angle lens
#endif
bool USE_SACCADE = true;                    // Use
double SALIENCY_X_TO_FACE = 80;             // X to saccade action
double SALIENCY_X_TO_FACE_HALF = SALIENCY_X_TO_FACE / 2;
double SALIENCY_Y_TO_FACE = 128;            // Y movement mapping
double SALIENCY_Y_TO_FACE_HALF = SALIENCY_Y_TO_FACE / 2;

// Y-to-joint are defined in the methods calculate Y1/Y2 position
int SALIENCY_BUFFER = 2;                    // Amount of messages that will be buffered
int SALIENCY_PIXEL_MARGIN_X = 40;           // Amount of pixels in X to group with (both directions)
int SALIENCY_PIXEL_MARGIN_Y = 40;           // Amount of pixels in Y to group with (both directions)
int SALIENCY_REMOVE_SPEED = 5;              // Amount of messages not seen
double SALIENCY_WEIGHT_FACTOR = 0.8;        // Weight of every new message
double SALIENCY_USED_WEIGHT = 0.995;        // When used, multiply it with this number to lower the chance of being salient
double SALIENCY_RECOVER_WEIGHT = 1.03;      // When penalized, recover the saliency of the point over time
double SALIENCY_NOT_REPORTED_WEIGHT = 1.5;  // When not reported, this factor indicated the weight of the loss
double SALIENCY_CHOSEN_BONUS = 3.0;         // When a point is chosen, assign a bonus
double SALIENCY_TIMEOUT = 5.0;              // Used to determine if an input is lost

// Emotion change settings
int EMOTION_BUFFER = 2;                     // Amount of messages that will be buffered
double EMOTION_AMAZED_SPEED = 0.2;          // Speed of the amazed emotion on a new object
double EMOTION_RESET_SPEED = 0.5;           // Speed to reset the activated emotion
int EMOTION_REPLACE_DIFFERENCE = 20;        // How fast a emotion should be instantly replaced
double EMOTION_WEIGHT_FACTOR = 0.8;         // Weight of every new message

// Action settings
int ACTION_SURPRISED_THRESHOLD = 1000;      // Threshold for surprised action
double ACTION_SURPRISED_TIMEOUT = 5.0;      // Timeout for the emotion after execution

// Sequence Settings
int SEQUENCE_BUFFER = 2;                    // Amount of messages that will be buffered

////////////////////////////////////////////////
// Initialisation
////////////////////////////////////////////////

// Enabled flag
bool nodeEnabled = false;

// Display variables
cv::Mat incomingSaliencyMap;
cv::Mat calculatedSaliencyMap;
cv::Scalar cvWhite = cv::Scalar(255, 255, 255);
cv::Scalar cvRed = cv::Scalar(0, 0, 255);

// Global saliency variables
std::vector<unsigned short int> saliencyLocationsX; // The locations (X)
std::vector<unsigned short int> saliencyLocationsY; // The locations (Y)
std::vector<double> saliencyWeights;                // >= 0, <= 10000, The weight of the locations
std::vector<double> saliencyNotReported;            // >= 1, <= SALIENCY_REMOVE_SPEED, counts the amount of times not reported
std::vector<double> saliencyUsed;                   // > 0, <= 1, Adjusted weights for when used
std::vector <std::string> saliencyIds;              // Used to identify the saliency message from the suppliers
std::vector<short int> saliencyDeliberates;         // Used to identify the saliency type from the suppliers
std::vector<int> saliencySeqIds;                    // Used to identify the saliency messages in the system
std::map<std::string, double> saliencyTimeout;      // Used to determine if an input has gone missing
unsigned int saliencyNothingFound = 0;              // Counts the amount of cycles nothing is found
int salientSeqIdCounter = 0;                        // Used to keep track of the salient number
int salientSeqId = 0;                               // Used to keep track of the salient number
int salientSeqIdPrevious = 0;                       // Used to keep track of the salient number
unsigned short int salientX = SALIENCY_X_RES_HALF;  // Used to store the salient X
unsigned short int salientY = SALIENCY_Y_RES_HALF;  // Used to store the salient Y
double salientWeightPrevious = 0;                   // Used to store the previous saliency weight
double salientWeight = 0;                           // Used to store the saliency weight
std::string salientIdPrevious = "";                 // Used to store the previous saliency id
std::string salientId = "";                         // Used to store the saliency id
short int salientDeliberate = 0;                    // Used to store the saliency mapping type

// Global emotion variables
double emotionStartTime = 0;     // Start time of the current emotion
double emotionEndTime = 0;
double emotionStart = 0;
double emotionEnd = 127;
bool emotionConstant = false;
bool emotionResetInProgress = false;
int inputArousal = 64;
int inputValence = 64;

// Global action variables
double actionSurprisedTimeout = 0;

// Global sequence variables
bool sequenceInProgress = false;
bool sequenceReset = false;
// Sequence queue
std::vector<unsigned short int> seqQueueSequence;
std::vector <std::string> seqQueueIdentifier;
std::vector <ros::Time> seqQueueStart;
std::vector <ros::Time> seqQueueEnd;

// Global publishers and messages
ros::Publisher facePublisher;
ros::Publisher jointPublisher;
ros::Publisher animationPublisher;
ros::Publisher predictFeedbackPublisher;
ros::Publisher generalFeedbackPublisher;
ros::Publisher gazeFeedbackPublisher;

// Create the messages
ram_animation_msgs::Face faceMsg;
sensor_msgs::JointState jointMsg;
ram_animation_msgs::Animation animationMsg;
ram_output_msgs::PredictFeedback predictFeedbackMsg;
ram_output_msgs::GeneralFeedback generalFeedbackMsg;
ram_output_msgs::GazeFeedback gazeFeedbackMsg;

#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCDFAInspection"

/**
 * Sigint handler to close all current windows
 */
void mySigintHandler(int sig) {
    // All the default sigint handler does is call shutdown()
    ros::shutdown();

    // Close all windows
    cv::destroyAllWindows();
    cv::waitKey(1);
}

#pragma clang diagnostic pop

/**
 * Method to handle incoming emotion messages
 */
void _emotionMessageCallback(const ram_input_msgs::EmotionConstPtr &emotionMessage) {
    // [-1, 1] -> [0, 2] -> [0, 128]
    int arousalPlus = (emotionMessage->arousal + 1) * 64;
    int valencePlus = (emotionMessage->valence + 1) * 64;
    unsigned int emotionDifference = abs(arousalPlus - inputValence) +
                                     abs(valencePlus - inputValence);

    // Return an feedback ack
    publishGeneralFeedback(
            FeedbackConstants::FeedbackType,
            emotionMessage->id,
            FeedbackConstants::FeedbackMessageAck,
            FeedbackConstants::FeedbackReasonEmpty
    );

    // Check if the difference, if to large, simply replace the current emotion
    if (emotionDifference > EMOTION_REPLACE_DIFFERENCE) {
        inputArousal = arousalPlus;
        inputValence = valencePlus;
        return;
    }

    // The variables are simply averaged over time
    inputArousal = (int) ((1.0 - EMOTION_WEIGHT_FACTOR) * (double) inputArousal
                          + EMOTION_WEIGHT_FACTOR * (double) arousalPlus);
    inputValence = (int) ((1.0 - EMOTION_WEIGHT_FACTOR) * (double) inputValence
                          + EMOTION_WEIGHT_FACTOR * (double) valencePlus);
}

/**
 * Enabled flag callback
 */
void _enabledModuleCallback(const ram_animation_msgs::EnabledModuleConstPtr &enabledModuleMessage) {
    nodeEnabled = enabledModuleMessage->hmmm;
    if (nodeEnabled) {
        // Publish emotion/face updates, but a wait few moments to ensure it is the new message
        ros::Duration(0.1).sleep();
        jointPublisher.publish(jointMsg);
        facePublisher.publish(faceMsg);
        animationPublisher.publish(animationMsg);

        // Also, clear sequence queue
        seqQueueIdentifier.clear();
        seqQueueSequence.clear();
    }
}

/**
 * Reconfiguration callback
 */
void _reconfigureCallback(ram_hmmm::RamHmmmConfig &config, uint32_t level) {
    // Display group
    if (!SHOW_INCOMING_SALIENCY_MAP && config.display_incoming_saliency_map) {
        // Open image preview
        cv::namedWindow(incoming_saliency_map_window, cv::WINDOW_AUTOSIZE);
    }
    if (!SHOW_CALCULATED_SALIENCY_MAP && config.display_calculated_saliency_map) {
        // Open image preview
        cv::namedWindow(calculated_saliency_map_window, cv::WINDOW_AUTOSIZE);
    }
    SHOW_INCOMING_SALIENCY_MAP = config.display_incoming_saliency_map;
    SHOW_CALCULATED_SALIENCY_MAP = config.display_calculated_saliency_map;

    // Camera group
    SALIENCY_X_RES = config.saliency_x_res;
    SALIENCY_X_RES_HALF = SALIENCY_X_RES / 2;
    SALIENCY_Y_RES = config.saliency_y_res;
    SALIENCY_Y_RES_HALF = SALIENCY_Y_RES / 2;
    SALIENCY_X_TO_JOINT = config.saliency_x_to_joint;
    USE_SACCADE = config.use_saccade;
    if (USE_SACCADE) {
        animationMsg.input = RamAnimatorConstants::InputSaccade;
    } else {
        animationMsg.input = RamAnimatorConstants::InputDefault;
    }
    SALIENCY_X_TO_FACE = config.saliency_x_to_face;
    SALIENCY_X_TO_FACE_HALF = SALIENCY_X_TO_FACE / 2;
    SALIENCY_Y_TO_FACE = config.saliency_y_to_face;
    SALIENCY_Y_TO_FACE_HALF = SALIENCY_Y_TO_FACE / 2;

    // Saliency group
    SALIENCY_BUFFER = config.saliency_buffer;
    SALIENCY_PIXEL_MARGIN_X = config.saliency_pixel_margin_x;
    SALIENCY_PIXEL_MARGIN_Y = config.saliency_pixel_margin_y;
    SALIENCY_REMOVE_SPEED = config.saliency_remove_speed;
    SALIENCY_WEIGHT_FACTOR = config.saliency_weight_factor;
    SALIENCY_USED_WEIGHT = config.saliency_used_weight;
    SALIENCY_RECOVER_WEIGHT = config.saliency_recover_weight;
    SALIENCY_NOT_REPORTED_WEIGHT = config.saliency_not_reported_weight;
    SALIENCY_CHOSEN_BONUS = config.saliency_chosen_bonus;
    SALIENCY_TIMEOUT = config.saliency_timeout;

    // Emotion group
    EMOTION_BUFFER = config.emotion_buffer;
    EMOTION_AMAZED_SPEED = config.emotion_amazed_speed;
    EMOTION_RESET_SPEED = config.emotion_reset_speed;
    EMOTION_REPLACE_DIFFERENCE = config.emotion_replace_difference;
    EMOTION_WEIGHT_FACTOR = config.emotion_weight_factor;

    // Action group
    ACTION_SURPRISED_THRESHOLD = config.action_surprised_threshold;
    ACTION_SURPRISED_TIMEOUT = config.action_surprised_timeout;

    // Sequence group
    SEQUENCE_BUFFER = config.sequence_buffer;
}

/**
 * The callback function which is executed when a Saliency message arrives
 */
void _saliencyMessageCallback(const ram_input_msgs::SaliencyConstPtr &saliencyMessage) {

    // Local variables
    std::vector<int> notFound;
    bool found;
    unsigned short int incomingX, incomingY;

    // Loop the message updates
    for (std::vector<unsigned short int>::size_type i = 0; i != saliencyMessage->locationsX.size(); i++) {

        found = false;

        // Loop the current recorded salient places
        for (std::vector<unsigned short int>::size_type j = 0; j != saliencyLocationsX.size(); j++) {

            // Convert input
            incomingX = (unsigned short int) ((saliencyMessage->locationsX[i] * (SALIENCY_X_RES_HALF)) + SALIENCY_X_RES_HALF);
            incomingY = (unsigned short int) ((saliencyMessage->locationsY[i] * (SALIENCY_Y_RES_HALF)) + SALIENCY_Y_RES_HALF);

            // Check if the pixel is already in the array
            // Keep a small margin to group small changes in a single location
            if (saliencyMessage->inputId.compare(saliencyIds[j]) == 0 &&
                incomingX >= (saliencyLocationsX[j] - SALIENCY_PIXEL_MARGIN_X) &&
                incomingX <= (saliencyLocationsX[j] + SALIENCY_PIXEL_MARGIN_X) &&
                incomingY >= (saliencyLocationsY[j] - SALIENCY_PIXEL_MARGIN_Y) &&
                incomingY <= (saliencyLocationsY[j] + SALIENCY_PIXEL_MARGIN_Y)
                    ) {

                // Update the location, weights & usage
                saliencyLocationsX[j] = incomingX;
                saliencyLocationsY[j] = incomingY;
                saliencyWeights[j] = (SALIENCY_WEIGHT_FACTOR * (double) saliencyMessage->weights[i]) +
                                     (1 - SALIENCY_WEIGHT_FACTOR) * saliencyWeights[j];
                saliencyNotReported[j] = 1.0;
                found = true;
                break;
            }

        }

        if (!found) {
            notFound.push_back(i);
        }
    }

    // Add new locations to the array
    for (auto const &index: notFound) {

        // Convert input
        incomingX = (unsigned short int) ((saliencyMessage->locationsX[index] * (SALIENCY_X_RES_HALF)) + SALIENCY_X_RES_HALF);
        incomingY = (unsigned short int) ((saliencyMessage->locationsY[index] * (SALIENCY_Y_RES_HALF)) + SALIENCY_Y_RES_HALF);

        // Set the values
        saliencyLocationsX.push_back(incomingX);
        saliencyLocationsY.push_back(incomingY);
        saliencyWeights.push_back((double) saliencyMessage->weights[index]);
        saliencyNotReported.push_back(1.0);
        saliencyUsed.push_back(1);
        saliencyIds.push_back(saliencyMessage->inputId);
        saliencyDeliberates.push_back(saliencyMessage->deliberate);
        salientSeqIdCounter++;
        saliencySeqIds.push_back(salientSeqIdCounter);
    }

    // Update the timeout time
    double curTime = ros::Time::now().toSec();
    saliencyTimeout[saliencyMessage->inputId] = curTime;

    // Loop all data to remove old locations from the map
    int size = saliencyLocationsX.size();
    double timeout = curTime - SALIENCY_TIMEOUT;
    for (std::vector<unsigned short int>::size_type i = 0; i < size; i++) {

        // Remove old point when the are no longer there
        // Skip the not-reported check if the message is not from the same id
        int sameId = saliencyIds[i].compare(saliencyMessage->inputId);
        if ((sameId == 0 && saliencyNotReported[i] >= SALIENCY_REMOVE_SPEED) ||
            saliencyTimeout[saliencyIds[i]] < timeout) {
            saliencyLocationsX.erase(saliencyLocationsX.begin() + i);
            saliencyLocationsY.erase(saliencyLocationsY.begin() + i);
            saliencyWeights.erase(saliencyWeights.begin() + i);
            saliencyNotReported.erase(saliencyNotReported.begin() + i);
            saliencyUsed.erase(saliencyUsed.begin() + i);
            saliencyIds.erase(saliencyIds.begin() + i);
            saliencyDeliberates.erase(saliencyDeliberates.begin() + i);
            saliencySeqIds.erase(saliencySeqIds.begin() + i);
            size--;
            i--;
            continue;
        }

        // Let points become more salient in time
        if (saliencyUsed[i] < 0.9 && saliencySeqIds[i] != salientSeqId) {
            saliencyUsed[i] = SALIENCY_RECOVER_WEIGHT * saliencyUsed[i];
            // Limit value to 1.0
            saliencyUsed[i] = saliencyUsed[i] > 1.0 ? 1.0 : saliencyUsed[i];
        }

        // Always increment the not used counter for the current id
        if (sameId == 0) {
            saliencyNotReported[i] = saliencyNotReported[i] * SALIENCY_NOT_REPORTED_WEIGHT;
        }
    }
}

/**
 * The callback function which is executed when a Sequence message arrives
 */
void _sequenceMessageCallback(const ram_input_msgs::SequenceConstPtr &sequenceMessage) {

    // We do not accept updates here when the node is disabled
    if (!nodeEnabled) return;

    ROS_DEBUG("");
    ROS_DEBUG("Sequence message received");

    // Actuator check
    ROS_DEBUG("Actuator check not yet implemented");

    ////////////////////////////////////////////////////////////////
    /// Check the timing constraints                             ///
    /// We have different types, which are processed differently ///
    ////////////////////////////////////////////////////////////////

    // First, calculate the expected timing
    double sequenceDuration = _getSequenceDuration(sequenceMessage->sequence, animationMsg.frequency);
    ROS_DEBUG("Sequence duration: %f", sequenceDuration);

    if(sequenceMessage->requestType == InputConstants::SequencePredictType){
        predictFeedbackMsg.id = sequenceMessage->id;
        predictFeedbackMsg.duration = sequenceDuration;
        predictFeedbackMsg.stroke = _getSequenceStrokeDuration(sequenceMessage->sequence, sequenceDuration);
        predictFeedbackPublisher.publish(predictFeedbackMsg);
        return;
    }

    ros::Duration allDuration = ros::Duration(
            sequenceMessage->requestCount * sequenceDuration
    );
    ros::Duration requestTiming = ros::Duration(
            ros::Duration(sequenceMessage->request)
    );
    ros::Time now = ros::Time::now();
    ros::Time start;
    ros::Time end;
    switch (sequenceMessage->requestType) {
        case InputConstants::SequenceStartAtType:
            ROS_DEBUG("Start at request");
            start = now + requestTiming;
            end = start + allDuration;
            break;
        case InputConstants::SequenceStrokeAtType: {
            ROS_DEBUG("Stroke at request");
            ros::Duration strokeDuration = ros::Duration(
                    _getSequenceStrokeDuration(sequenceMessage->sequence, sequenceDuration)
            );
            start = now + requestTiming - strokeDuration;
            end = start + allDuration;
        }
            break;
        case InputConstants::SequenceEndAtType:
            ROS_DEBUG("End at request");
            end = now + requestTiming;
            start = end - allDuration;
            break;
        default:
            // Error, return nack
            ROS_DEBUG("Invalid message type");
            publishNackFeedback(sequenceMessage->id, FeedbackConstants::FeedbackReasonInvalid);
            return;
    }

    // Check if the timing is possible
    // End can not be before now
    if (end < now) {
        publishNackFeedback(sequenceMessage->id, FeedbackConstants::FeedbackReasonInvalid);
        return;
    }
    // Start should not be to soon
    if (start < (now + ros::Duration(0.1))){
        publishNackFeedback(sequenceMessage->id, FeedbackConstants::FeedbackReasonTooSoon);
        return;
    }

    // Find if there is a sequence conflicting with the requested timing
    std::vector<int>::size_type queueSize = seqQueueSequence.size();
    std::vector<int>::size_type i;
    for (i = 0; i < queueSize; i++) {
        // Check for timing possible errors
        if (
                // Start positioned in timing of the current item
                (start >= seqQueueStart[i] && start < seqQueueEnd[i]) ||
                // End positioned in timing of the current item
                (end > seqQueueStart[i] && end <= seqQueueEnd[i])
                ){
            publishNackFeedback(sequenceMessage->id, FeedbackConstants::FeedbackReasonFluency);
            return;
        }

        // Check if it can be positioned here
        if (end <= seqQueueStart[i]){
            break;
        }
    }

    ROS_DEBUG("Start: %f, end: %f", start.toSec(), end.toSec());

    // Simply queue item
    seqQueueIdentifier.insert(seqQueueIdentifier.begin() + i, sequenceMessage->id);
    seqQueueSequence.insert(seqQueueSequence.begin() + i, sequenceMessage->sequence);
    seqQueueStart.insert(seqQueueStart.begin() + i, start);
    seqQueueEnd.insert(seqQueueEnd.begin() + i, end);

    // Return an feedback ack
    publishGeneralFeedback(
            FeedbackConstants::FeedbackType,
            sequenceMessage->id,
            FeedbackConstants::FeedbackMessageAck,
            FeedbackConstants::FeedbackReasonEmpty
    );
}

/**
 * Convert arousal & valence values to an emotion
 */
const char arousalValenceToEmotion() {
    double emotionAngle = atan2(inputArousal - 64, inputValence - 64);
    char emotionState = RamAnimatorConstants::EmotionNeutral;
    if (emotionAngle > 0 && emotionAngle < 3.14 / 3) emotionState = RamAnimatorConstants::EmotionExcited;
    if (emotionAngle > 3.14 / 3 && emotionAngle < 3.14 / 3 * 2) emotionState = RamAnimatorConstants::EmotionAmazed;
    if (emotionAngle > 3.14 / 3 * 2 && emotionAngle < 3.14) emotionState = RamAnimatorConstants::EmotionAngry;
    if (emotionAngle > -3.14 && emotionAngle < -3.14 / 3 * 2) emotionState = RamAnimatorConstants::EmotionSad;
    if (emotionAngle > -3.14 / 3 * 2 && emotionAngle < -3.14 / 3) emotionState = RamAnimatorConstants::EmotionSleepy;
    if (emotionAngle > -3.14 / 3 && emotionAngle < 0) emotionState = RamAnimatorConstants::EmotionNeutral;

    return emotionState;
}

/**
 * Convert arousal & valence to an emotion intensity
 */
int arousalValenceToEmotionIntensity() {
    int inputValenceMinus = inputValence - 64;
    int inputArousalMinus = inputArousal - 64;
    int faceEmotionLevel = int(2 * sqrt(inputValenceMinus * inputValenceMinus + inputArousalMinus * inputArousalMinus));

    if (faceEmotionLevel < 0) {
        return 0;
    } else if (faceEmotionLevel > 127) {
        return 127;
    }

    return faceEmotionLevel;
}

/**
 * Calculates the eye-X-position based on the salient X
 */
int calculateEyeXPosition(const double &xScale, int previous) {

    int newVal = (int) (((xScale - 0.5) * -SALIENCY_X_TO_FACE) + 64);

    // Will do some default scaling
    int difference = previous - newVal;
    if (difference > 2) {
        return previous - 2;
    } else if (difference <= -2) {
        return previous + 2;
    }

    return newVal;
}

/**
 * Calculates the eye-Y-position based on the salient Y
 */
int calculateEyeYPosition(const double &yScale) {
    // As the Y is not used to directly move the eyes, we do not need scaling
    // However, we add 25 to compensate for the camera angle
    return std::min((int) (((yScale - 0.5) * SALIENCY_Y_TO_FACE) + 64), 127);
}

/**
 * Calculates the X-position based on the salient X
 */
double calculateXPosition(const double &xScale) {
    return ((xScale * -SALIENCY_X_TO_JOINT) + (SALIENCY_X_TO_JOINT / 2));
}

/**
 * Calculates the Y2-position based on the salient Y
 */
double calculateY1Position(const double &yScale) {
#ifdef NO_LENS
    // Y1-1: y = -2,64x2 + 4,3x - 0,95
    // Y1-2: y = -3,92x2 + 2,18x + 0,43
    if (yScale > 0.5) {
        return (-2.64 * yScale * yScale) + (4.3 * yScale) - 0.95;
    }
    return (-3.92 * yScale * yScale) + (2.18 * yScale) + 0.43;
#endif
#ifdef WIDE_LENS
    // Y1-1: y = 0,88x2 - 0,9x + 1,12
    // Y1-2: y = 1,2x2 + 0,1x + 0,54
    if (yScale > 0.5) {
        return (0.88 * yScale * yScale) - (0.9 * yScale) + 1.12;
    }
    return (1.2 * yScale * yScale) + (0.1 * yScale) + 0.54;
#endif
}

/**
 * Calculates the Y1-position based on the salient Y
 */
double calculateY2Position(const double &yScale) {
#ifdef NO_LENS
    // Y2-1: y = -2,96x2 + 4,14x - 1,18
    // Y2-2: y = -4,4x2 + 1,74x + 0,38
    if (yScale > 0.5) {
        return (-2.96 * yScale * yScale) + (4.14 * yScale) - 1.18;
    }

    return (-4.4 * yScale * yScale) + (1.74 * yScale) + 0.38;
#endif
#ifdef WIDE_LENS
    // Y2-1: y = 0,24x2 - 0,86x + 0,94
    // Y2-2: y = -3,44x2 + 2,26x + 0,3
    if (yScale > 0.5) {
        return (0.24 * yScale * yScale) - (0.86 * yScale) + 0.94;
    }

    return (-3.44 * yScale * yScale) + (2.26 * yScale) + 0.3;
#endif
}

/**
 * Computes the saliency and thus the gaze direction
 */
void computeSaliency(bool calculateImage) {

    // Check if we want to display the image, if so, create a new map
    if (calculateImage && SHOW_INCOMING_SALIENCY_MAP) {
        incomingSaliencyMap = cv::Mat::zeros(SALIENCY_Y_RES_HALF, SALIENCY_X_RES_HALF, CV_8UC3);
    }
    if (calculateImage && SHOW_CALCULATED_SALIENCY_MAP) {
        calculatedSaliencyMap = cv::Mat::zeros(SALIENCY_Y_RES_HALF, SALIENCY_X_RES_HALF, CV_8UC3);
    }

    // Search the most important region
    double largest = 0, calculated = 0;
    unsigned short int index = 0;
    for (std::vector<unsigned short int>::size_type i = 0; i < saliencyLocationsX.size(); i++) {
        calculated = saliencyWeights[i]
                     // When used, they use saliency
                     * (double) saliencyUsed[i]
                     // Not reported point loose interest fast
                     * (1 / saliencyNotReported[i]);
        if (calculated > largest) {
            largest = calculated;
            index = i;
        }

        // Add point to the incoming map
        if (calculateImage && SHOW_INCOMING_SALIENCY_MAP) {
            cv::circle(incomingSaliencyMap,
                       cv::Point((int) (saliencyLocationsX[i] / 2), (int) (saliencyLocationsY[i] / 2)),
                       saliencyWeightToSize(saliencyWeights[i]),
                       cvWhite, // White
                       -1,
                       8
            );
        }

        // Add point to processed
        if (calculateImage && SHOW_CALCULATED_SALIENCY_MAP) {
            cv::circle(calculatedSaliencyMap,
                       cv::Point((int) (saliencyLocationsX[i] / 2), (int) (saliencyLocationsY[i] / 2)),
                       saliencyWeightToSize(calculated),
                       cvWhite, // White
                       -1,
                       8
            );
        }
    }

    // Set the previous state
    salientSeqIdPrevious = salientSeqId;
    salientWeightPrevious = salientWeight;
    salientIdPrevious = salientId;

    // Check if something can be and is found
    if (saliencyLocationsX.size() > 0 && largest != 0) {

        // Update saliency used value
        saliencyUsed[index] = SALIENCY_USED_WEIGHT * saliencyUsed[index];

        salientSeqId = saliencySeqIds[index];
        salientX = saliencyLocationsX[index];
        salientY = saliencyLocationsY[index];
        salientWeight = saliencyWeights[index];
        salientId = saliencyIds[index];
        salientDeliberate = saliencyDeliberates[index];
        saliencyNothingFound = 0;

        // Reset smoothing on sequence id change
        if (salientSeqId != salientSeqIdPrevious) {
            animationMsg.smoothing = 1.0;
        }

        // Add red point to the map
        if (calculateImage && SHOW_CALCULATED_SALIENCY_MAP) {
            cv::circle(calculatedSaliencyMap,
                       cv::Point((int) (saliencyLocationsX[index] / 2), (int) (saliencyLocationsY[index] / 2)),
                       saliencyWeightToSize(largest),
                       cvRed, // White
                       -1,
                       8
            );
        }
    } else {
        // Nothing found, return to initial position
        salientX = SALIENCY_X_RES_HALF;
        salientY = SALIENCY_Y_RES_HALF;
        salientWeight = 0;
        if (saliencyNothingFound < std::numeric_limits<int>::max()) {
            // Add 1, but guard for overflow
            saliencyNothingFound++;
        }
    }

    // Check if we need to give an penalty (given if a item is not longer most salient)
    if (salientSeqId != salientSeqIdPrevious) {
        // Find the previous sequence, and penalize it
        for (std::vector<unsigned short int>::size_type i = 0; i < saliencyLocationsX.size(); i++) {
            if (saliencySeqIds[i] == salientSeqIdPrevious) {
                saliencyUsed[i] = 0.1;
                break;
            }
        }

        // Give the current point a bonus
        saliencyUsed[index] = saliencyUsed[index] * SALIENCY_CHOSEN_BONUS;
    }

    // Show calculated images, if needed
    if (SHOW_INCOMING_SALIENCY_MAP) {
        if (calculateImage) {
            cv::imshow(incoming_saliency_map_window, incomingSaliencyMap);
            cv::waitKey(1);
        }
    } else {
        cv::destroyWindow(incoming_saliency_map_window);
        cv::waitKey(1);
    }
    if (SHOW_CALCULATED_SALIENCY_MAP) {
        if (calculateImage) {
            cv::imshow(calculated_saliency_map_window, calculatedSaliencyMap);
            cv::waitKey(1);
        }
    } else {
        cv::destroyWindow(calculated_saliency_map_window);
        cv::waitKey(1);
    }

}

/**
 * Computes the required emotion
 */
void computeEmotion() {
    // Check if new salient weight is significant larger than the previous one
    // Only is emotion is neutral
    // Only if saliency mapping is not deliberate
    double curTime = ros::Time::now().toSec();
    if (salientDeliberate == 0 &&
        faceMsg.emotionState == RamAnimatorConstants::EmotionNeutral &&
        (
                (salientId.compare(salientIdPrevious) != 0 && salientWeight >= ACTION_SURPRISED_THRESHOLD) ||
                (salientId.compare(salientIdPrevious) == 0 && (salientWeight - salientWeightPrevious) >= ACTION_SURPRISED_THRESHOLD)
        ) &&
        curTime > actionSurprisedTimeout) {

        // Init surprised emotion
        faceMsg.emotionState = RamAnimatorConstants::EmotionAmazed;
        faceMsg.emotionLevel = 120;
        animationMsg.smoothing = 0.95;
        emotionStartTime = ros::Time::now().toSec();
        emotionEndTime = emotionStartTime + EMOTION_AMAZED_SPEED;
        emotionStart = 0;
        emotionEnd = 127;
        actionSurprisedTimeout = curTime + ACTION_SURPRISED_TIMEOUT;

        // Disable constant emotion
        emotionConstant = false;
    }
}

/**
 * Computes the new joint states according to the saliency
 */
void computeJointStates() {
    double xScale = fabs((double) salientX / (double) SALIENCY_X_RES);
    double yScale = fabs(((double) salientY / (double) SALIENCY_Y_RES) - 1.0);
    if (USE_SACCADE) {
        faceMsg.eyePosX = calculateEyeXPosition(xScale, faceMsg.eyePosX);
        faceMsg.eyePosY = calculateEyeYPosition(yScale);
    } else {
        jointMsg.position[0] = calculateXPosition(xScale);
        jointMsg.position[1] = calculateY1Position(yScale);
        jointMsg.position[2] = calculateY2Position(yScale);
    }
}

/**
 * Computes the progress of the sequence
 */
void computeSequence() {

    // Check if we need to reset the current sequence
    if (sequenceReset) {

        ROS_DEBUG("Sequence reset");

        // Send progress feedback on message end
        publishProgressEnd();

        // Update message
        animationMsg.id = seqQueueIdentifier.back();
        animationMsg.sequence = seqQueueSequence.back();
        animationMsg.smoothing = 1.0;

        // Reset queue
        seqQueueIdentifier.clear();
        seqQueueSequence.clear();
        seqQueueStart.clear();
        seqQueueEnd.clear();
        sequenceReset = false;

        // Send progress feedback on sequence start
        publishProgressStart();

        return;
    }

    // Check if we are ready for the next item in the
    ros::Time now = ros::Time::now();
    if (sequenceInProgress && seqQueueEnd.front() <= now) {

        ROS_DEBUG("Remove sequence from queue");

        // Send progress feedback on message end
        publishProgressEnd();

        // Removed finished item from queue
        seqQueueIdentifier.erase(seqQueueIdentifier.begin());
        seqQueueSequence.erase(seqQueueSequence.begin());
        seqQueueStart.erase(seqQueueStart.begin());
        seqQueueEnd.erase(seqQueueEnd.begin());
        sequenceInProgress = false;

        // Reset message
        animationMsg.id = "";
        animationMsg.sequence = RamAnimatorConstants::SequenceEmpty;
    }

    if (!sequenceInProgress && seqQueueSequence.size() > 0 && seqQueueStart.front() <= now) {

        ROS_DEBUG("Activate next sequence");

        // Set new item in current, if any
        sequenceInProgress = true;
        animationMsg.id = seqQueueIdentifier.front();
        animationMsg.sequence = seqQueueSequence.front();

        // Change smoothing for pointing sequence
        if (animationMsg.sequence == RamAnimatorConstants::SequencePointingLeft ||
            animationMsg.sequence == RamAnimatorConstants::SequencePointingMiddle ||
            animationMsg.sequence == RamAnimatorConstants::SequencePointingRight) {
            animationMsg.smoothing = 0.82;
        }

        // Send progress feedback on sequence start
        publishProgressStart();
    }
}

/**
 * Creates the default messages
 */
void createDefaultMessages() {

    // Fill face message
    faceMsg.eyePosX = 64;
    faceMsg.eyePosY = 64;
    faceMsg.emotionState = 0;
    faceMsg.emotionLevel = 64;
    faceMsg.screenBrightness = 64;

    // Fill joint message
    jointMsg.name.resize(3);
    jointMsg.position.resize(3);
    jointMsg.name[0] = "joint1";
    jointMsg.name[1] = "joint2";
    jointMsg.name[2] = "joint3";
    jointMsg.position[0] = 0.0;
    jointMsg.position[1] = 0.6;
    jointMsg.position[2] = 0.45;

    // Fill animation message
    animationMsg.id = "";
    animationMsg.sequence = RamAnimatorConstants::SequenceEmpty;
    animationMsg.frequency = 0.5;
    animationMsg.smoothing = 1.0;
    animationMsg.compression = 0.0;
    if (USE_SACCADE) {
        animationMsg.input = RamAnimatorConstants::InputSaccade;
    } else {
        animationMsg.input = RamAnimatorConstants::InputDefault;
    }
    animationMsg.demo = false;
}

/**
 * Progress the animation
 */
void emotionProgress() {
    // Reset the emotion to neutral, or sweep with the given params
    double curTime = ros::Time::now().toSec();
    if (emotionConstant) {
        setHmmmEmotion();
    } else if (emotionResetInProgress &&
               curTime > emotionEndTime) {
        // Reset emotion
        faceMsg.emotionState = RamAnimatorConstants::EmotionNeutral;
        faceMsg.emotionLevel = 64;
        animationMsg.smoothing = 1.0;
        emotionResetInProgress = false;
    } else if (curTime < emotionEndTime) {
        faceMsg.emotionLevel = sweepEmotionLevel();
    } else if (faceMsg.emotionState != RamAnimatorConstants::EmotionNeutral &&
               !emotionResetInProgress &&
               curTime > emotionEndTime) {
        emotionStartTime = curTime;
        emotionEndTime = emotionStartTime + EMOTION_RESET_SPEED;
        emotionStart = faceMsg.emotionLevel;
        emotionEnd = 0;
        emotionResetInProgress = true;
    } else {
        // Process default emotion from hmmm mixing
        setHmmmEmotion();
        emotionConstant = true;
    }
}

/**
 * Publish a general feedback message
 * Use the constants from feedbackConstant.h as parameters
 * @param type
 * @param id
 * @param ack
 * @param reason
 */
void publishGeneralFeedback(int type, std::string id, int ack, int reason) {
    generalFeedbackMsg.type = type;
    generalFeedbackMsg.id = id;
    generalFeedbackMsg.message = ack;
    generalFeedbackMsg.reason = reason;
    generalFeedbackPublisher.publish(generalFeedbackMsg);
}

/**
 * Publish the gaze feedback message
 */
void publishGazeFeedback() {
    // X&Y feedback
    gazeFeedbackMsg.x = scaleSize(salientX, (float) SALIENCY_X_RES_HALF);
    gazeFeedbackMsg.y = scaleSize(salientY, (float) SALIENCY_Y_RES_HALF);
    gazeFeedbackPublisher.publish(gazeFeedbackMsg);
}

/**
 * Publish a nack message, including reason
 */
void publishNackFeedback(std::string id, int reason) {
    publishGeneralFeedback(
            FeedbackConstants::FeedbackType,
            id,
            FeedbackConstants::FeedbackMessageNack,
            reason
    );
}

/**
 * Send progress end updates
 */
void publishProgressEnd() {
    publishGeneralFeedback(
            FeedbackConstants::ProgressType,
            animationMsg.id,
            FeedbackConstants::ProgressMessageEnd,
            FeedbackConstants::ProgressReasonEmpty
    );
}

/**
 * Sends progress start update
 */
void publishProgressStart() {
    publishGeneralFeedback(
            FeedbackConstants::ProgressType,
            animationMsg.id,
            FeedbackConstants::ProgressMessageStart,
            FeedbackConstants::ProgressReasonEmpty
    );
}

/**
 * Convert saliency weight to circle size
 */
int saliencyWeightToSize(double val) {
    return (int) ((val / 10000) * (SALIENCY_X_RES_HALF));
}

/**
 * Convert the Salient value back to the [-1, 1]-range
 * @param size
 * @param scaleFactor
 * @return
 */
float scaleSize(int size, float scaleFactor) {
    return (((float) size - scaleFactor) / scaleFactor);
}

/**
 * Set the current HMMM emotion in the message
 */
void setHmmmEmotion() {
    faceMsg.emotionLevel = arousalValenceToEmotionIntensity();
    if (faceMsg.emotionLevel < 40) {
        faceMsg.emotionState = RamAnimatorConstants::EmotionNeutral;
    } else {
        faceMsg.emotionState = arousalValenceToEmotion();
    }
}

/**
 * Sweep a param for in a certain time
 */
int sweepEmotionLevel() {
    if (emotionEnd > emotionStart) {
        return (int) (
                (ros::Time::now().toSec() - emotionStartTime) /
                (emotionEndTime - emotionStartTime) * (emotionEnd - emotionStart)
        ) + emotionStart + 0.5;
    } else {
        return (int) emotionStart - (
                (ros::Time::now().toSec() - emotionStartTime) /
                (emotionEndTime - emotionStartTime) *
                (emotionStart - emotionEnd)
        ) - 0.5;
    }
}

/**
 * Main functionality
 */
int main(int argc, char **argv) {
    // Initialize the ROS system.
    ros::init(argc, argv, "ram_hmmm_node");

    // Establish this program as a ROS node.
    ros::NodeHandle nh;

    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, mySigintHandler);

    // Create a loop rate
    ros::Rate loop_rate(ROS_RATE);

    // Configure reconfigure server
    dynamic_reconfigure::Server <ram_hmmm::RamHmmmConfig> reconfigureServer;
    dynamic_reconfigure::Server<ram_hmmm::RamHmmmConfig>::CallbackType reconfigureCallback;
    reconfigureCallback = boost::bind(&_reconfigureCallback, _1, _2);
    reconfigureServer.setCallback(reconfigureCallback);

    // Register enabled subscriber
    ros::Subscriber enabledSubscriber = nh.subscribe("/ram/animation/enabled/nodes", 1, &_enabledModuleCallback);

    // Subscribe to the correct channels
    ros::Subscriber saliencySubscriber = nh.subscribe("/ram/animation/hmmm/saliency", SALIENCY_BUFFER, &_saliencyMessageCallback);
    ros::Subscriber emotionSubscriber = nh.subscribe("/ram/animation/hmmm/emotion", EMOTION_BUFFER, &_emotionMessageCallback);
    ros::Subscriber sequenceSubscriber = nh.subscribe("/ram/animation/hmmm/sequence", SEQUENCE_BUFFER, &_sequenceMessageCallback);

    // Create the publisher objects and messages
    facePublisher = nh.advertise<ram_animation_msgs::Face>("/ram/animation/desired/face", ROS_RATE);
    jointPublisher = nh.advertise<sensor_msgs::JointState>("/ram/animation/desired/joint", ROS_RATE);
    animationPublisher = nh.advertise<ram_animation_msgs::Animation>("/ram/animation/desired/animation", ROS_RATE);

    // Create the publisher for the feedback channel
    predictFeedbackPublisher = nh.advertise<ram_output_msgs::PredictFeedback>("/ram/asap/feedback/prediction", ROS_RATE);
    generalFeedbackPublisher = nh.advertise<ram_output_msgs::GeneralFeedback>("/ram/asap/feedback/general", ROS_RATE);
    gazeFeedbackPublisher = nh.advertise<ram_output_msgs::GazeFeedback>("/ram/asap/feedback/gaze", ROS_RATE);

    // Fill default messages
    createDefaultMessages();

    // Publish default messages
    facePublisher.publish(faceMsg);
    jointPublisher.publish(jointMsg);
    animationPublisher.publish(animationMsg);

    unsigned short int frameCount = 0;

    // Main program loop
    while (ros::ok()) {

        // Check if the node is enabled
        if (!nodeEnabled) {
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }

        // Calculate saliency and thus gaze direction
        computeSaliency(frameCount == 0);

        // Check the frames
        if (frameCount >= 10) {
            frameCount = 0;
        } else {
            frameCount++;
        }

        // Compute emotion and sequences
        computeEmotion();
        computeSequence();
        emotionProgress();

        // Compute the new joint states
        computeJointStates();

        // Publish the messages
        animationPublisher.publish(animationMsg);
        facePublisher.publish(faceMsg);
        jointPublisher.publish(jointMsg);
        publishGazeFeedback();

        // Do not loop too fast
        ros::spinOnce();
        loop_rate.sleep();
    }

}
