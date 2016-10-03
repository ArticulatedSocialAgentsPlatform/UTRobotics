// Header file with global variables
#include "ram_animator_node.hpp"

// Sequences
#include "sequences/defaults.cpp"
#include "sequences/demo.cpp"

/**
 * Callback used to process incoming animation messages
 */
void _animationMessageReceived(const ram_animation_msgs::AnimationConstPtr &incomingAnimationMsg) {
    // If the received sequence name is not empty and you're not busy with gazing somewhere, start the animation
    if (incomingAnimationMsg->sequence != RamAnimatorConstants::SequenceEmpty && !gazeInProgress) {
        // Store the start time for the animation
        if (!animationInProgress) {
            animationStartTime = ros::Time::now().toSec();
        }

        // Set animation to in be in progress
        animationInProgress = true;
    }

    // If the received sequence name was empty, stop the animation.
    if (incomingAnimationMsg->sequence == RamAnimatorConstants::SequenceEmpty && !demo) {
        animationInProgress = false;
    }

    // If the system changes from playing a sequence to not playing a sequence start homing.
    if (sequence != RamAnimatorConstants::SequenceEmpty &&
        incomingAnimationMsg->sequence == RamAnimatorConstants::SequenceEmpty &&
        !demo) {
        resetMovements(false);
        if (input != RamAnimatorConstants::InputSaccade) {
            transitionToMidiInput();
        }
    }

    // Store the desired sequence, but check for changes
    if (sequence != incomingAnimationMsg->sequence) {
        animationId = incomingAnimationMsg->id;
        sequence = incomingAnimationMsg->sequence;
        newAnimation = true;
        progressFeedbackSend = false;
    }

    // When the saccade mode is activated
    if (incomingAnimationMsg->input == RamAnimatorConstants::InputSaccade && input != RamAnimatorConstants::InputSaccade) {
        // Calculate where the starting position is
        saccadeRequiredAngle1 = (float(reqFaceEyePosX) - 64.0) / 64.0 * 0.5 * 3.14;
        saccadeRequiredAngle2 = 0.5 * (float(reqFaceEyePosY) - 64.0) / 64.0 * 0.25 * 3.14 + 0.25 * 3.14;
        saccadeRequiredAngle3 = -0.5 * (float(reqFaceEyePosY) - 64.0) / 64.0 * 0.25 * 3.14 + 0.25 * 3.14;
        // Store the starting location
        saccadeCurrentAngle1 = saccadeRequiredAngle1;
        saccadeCurrentAngle2 = saccadeRequiredAngle2;
        saccadeCurrentAngle3 = saccadeRequiredAngle3;
        // Go to the starting location
        startGaze(saccadeRequiredAngle1, saccadeRequiredAngle2, saccadeRequiredAngle3, 2);
    }

    // When the saccade mode is deactivated
    if (incomingAnimationMsg->input != RamAnimatorConstants::InputSaccade && input == RamAnimatorConstants::InputSaccade) {
        transitionToMidiInput();
    }

    // When the demo is activated
    if (!demo && incomingAnimationMsg->demo) {
        demoStartTime = ros::Time::now().toSec();
        demo = true;
        animationPhase = 0;
        // Store the current location
        preDemoAngle1 = reqPoseJointAngle1;
        preDemoAngle2 = reqPoseJointAngle2;
        preDemoAngle3 = reqPoseJointAngle3;
    }

    // When the demo is deactivated
    if (demo && !incomingAnimationMsg->demo) {
        animationInProgress = false;
        reqPoseJointAngle1 = preDemoAngle1;
        reqPoseJointAngle2 = preDemoAngle2;
        reqPoseJointAngle3 = preDemoAngle3;
        transitionToMidiInput();
    }

    // Only change smoothing, compression and animation frequency if the demo is not playing
    if (!demo) {
        smoothing = smoothingMin + incomingAnimationMsg->smoothing * (smoothingMax - smoothingMin);

        // Calculate the desired animation frequency from the configured limits
        animFreq = animFreqMin + incomingAnimationMsg->frequency * (animFreqMax - animFreqMin);
        compression = incomingAnimationMsg->compression;
    }

    input = incomingAnimationMsg->input;
    demo = incomingAnimationMsg->demo;
}

/**
 * Callback used to process incoming face messages
 */
void _faceMessageReceived(const ram_animation_msgs::FaceConstPtr &incomingFaceMsg) {
    reqFaceEyePosX = incomingFaceMsg->eyePosX;
    reqFaceEyePosY = incomingFaceMsg->eyePosY;
    reqFaceEmotionState = incomingFaceMsg->emotionState;
    reqFaceEmotionLevel = incomingFaceMsg->emotionLevel;
    outFaceScreenBrightness = incomingFaceMsg->screenBrightness;
}

/**
 * Callback to process incoming joint messages
 */
void _jointMessageReceived(const sensor_msgs::JointStateConstPtr &incomingJointMsg) {
    midiInputAngle1 = incomingJointMsg->position[0];
    midiInputAngle2 = incomingJointMsg->position[1];
    midiInputAngle3 = incomingJointMsg->position[2];
}

/**
 * Progress the animation
 */
void animate() {
    if (demo) {
        animateDemo();
    } else {
        animAngle = 3.14 / 15;
    }

    // These are the basic 'animation' building blocks
    if (animationInProgress || demo) {
        ROS_DEBUG("Animation in progress! \n");
        animationPhase = animationPhase + animFreq / nodeUpdateRate;

        animateSequences();
    }
}

/**
 * A full range soft knee compressor for the movement
 */
float compressMovement(float position, float minThres, float maxThres, float minAngle, float maxAngle, float maxRatio) {
    if (position > maxThres) {
        float exceed = position - maxThres;
        float softRatio = 1 + maxRatio * (exceed / (maxAngle - maxThres));
        return (maxThres + (position - maxThres) / softRatio);
    }

    if (position < minThres) {
        float exceed = position - minThres;
        float softRatio = 1 + maxRatio * (exceed / (minAngle - minThres));
        return (minThres + (position - minThres) / softRatio);
    }

    return position;
}

/**
 * Look at a certain spot, by updating the gaze
 */
void gaze() {
    summedAngle1 = oldGazeAngle1 + (gazeAngle1 - oldGazeAngle1) * (secs - gazeStartTime) / gazeMoveTime;
    summedAngle2 = oldGazeAngle2 + (gazeAngle2 - oldGazeAngle2) * (secs - gazeStartTime) / gazeMoveTime;
    summedAngle3 = oldGazeAngle3 + (gazeAngle3 - oldGazeAngle3) * (secs - gazeStartTime) / gazeMoveTime;

    // If you've reached your destination pose, stop moving
    if (secs - gazeStartTime > gazeMoveTime) {
        gazeInProgress = false;

        resetMovements(true);
    }
}

/**
 * Process the movement compression
 */
void processCompression() {
    outPoseJointAngle1 = compressMovement(outPoseJointAngle1,
                                          -0.25 * 3.14, 0.25 * 3.14, -0.5 * 3.14, 0.5 * 3.14,
                                          compression);
    outPoseJointAngle2 = compressMovement(outPoseJointAngle2,
                                          3.14 / 8, 0.5 * 3.14 - 3.14 / 8, 0, 0.5 * 3.14,
                                          compression);
    outPoseJointAngle3 = compressMovement(outPoseJointAngle3,
                                          -0.25 * 3.14, 0.25 * 3.14, -0.5 * 3.14, 0.5 * 3.14,
                                          compression);
}

/**
 * Limit the output angles
 */
void processLimiting() {
    if (outPoseJointAngle1 > 0.5 * 3.14) { outPoseJointAngle1 = 0.5 * 3.14; }
    if (outPoseJointAngle1 < -0.5 * 3.14) { outPoseJointAngle1 = -0.5 * 3.14; }
    if (outPoseJointAngle2 > 0.5 * 3.14) { outPoseJointAngle2 = 0.5 * 3.14; }
    if (outPoseJointAngle2 < 0) { outPoseJointAngle2 = 0; }
    if (outPoseJointAngle3 > 0.5 * 3.14) { outPoseJointAngle3 = 0.5 * 3.14; }
    if (outPoseJointAngle3 < 0) { outPoseJointAngle3 = 0; }
}

/**
 * Process the movement smoothing
 * For every cycle, the new control value is a certain ratio of the old one, this value is set here.
 */
void processSmoothing() {
    float newVal = 1 - smoothing;
    outPoseJointAngle1 = newVal * (summedAngle1) + smoothing * prevAngle1;
    outPoseJointAngle2 = newVal * (summedAngle2) + smoothing * prevAngle2;
    outPoseJointAngle3 = newVal * (summedAngle3) + smoothing * prevAngle3;
    prevAngle1 = outPoseJointAngle1;
    prevAngle2 = outPoseJointAngle2;
    prevAngle3 = outPoseJointAngle3;
}

/**
 * Processes the saccade eye movements
 */
void processSaccade() {
    short int previousOutX = outFaceEyePosX;
    short int previousOutY = outFaceEyePosY;

    saccadeRequiredAngle1 = (float(reqFaceEyePosX) - 64.0) / 64.0 * 0.5 * 3.14;
    saccadeRequiredAngle2 = 0.5 * (float(reqFaceEyePosY) - 64.0) / 64.0 * 0.25 * 3.14 + 0.35 * 3.14;
    saccadeRequiredAngle3 = -0.5 * (float(reqFaceEyePosY) - 64.0) / 64.0 * 0.25 * 3.14 + 0.25 * 3.14;
    saccadeCurrentAngle1 = 0.96 * saccadeCurrentAngle1 + 0.04 * saccadeRequiredAngle1;
    saccadeCurrentAngle2 = 0.90 * saccadeCurrentAngle2 + 0.10 * saccadeRequiredAngle2;
    saccadeCurrentAngle3 = 0.90 * saccadeCurrentAngle3 + 0.10 * saccadeRequiredAngle3;

    reqPoseJointAngle1 = saccadeCurrentAngle1;
    reqPoseJointAngle2 = saccadeCurrentAngle2;
    reqPoseJointAngle3 = saccadeCurrentAngle3;

    // Do not use this during dance or shake
    if (sequence != RamAnimatorConstants::SequenceDancing &&
        sequence != RamAnimatorConstants::SequenceShaking &&
        sequence != RamAnimatorConstants::SequenceEmpty) {
        outFaceEyePosX = int((saccadeRequiredAngle1 - (saccadeCurrentAngle1 + add_angle1)) / eyeAngleX * 63 + 64);
    } else {
        outFaceEyePosX = int((saccadeRequiredAngle1 - (saccadeCurrentAngle1)) / eyeAngleX * 63 + 64);
    }


    // Do not use this during nod
    if (sequence != RamAnimatorConstants::SequenceDancing &&
        sequence != RamAnimatorConstants::SequenceNodding &&
        sequence != RamAnimatorConstants::SequenceEmpty) {
        outFaceEyePosY = int((saccadeRequiredAngle2 - (saccadeCurrentAngle2 + add_angle2) - saccadeRequiredAngle3 +
                              (saccadeCurrentAngle3 + add_angle3)) / eyeAngleY * 63 + 64);
    } else {
        outFaceEyePosY = int((saccadeRequiredAngle2 - saccadeCurrentAngle2 - saccadeRequiredAngle3 +
                              saccadeCurrentAngle3) / eyeAngleY * 63 + 64);
    }

    outFaceEyePosX = std::max(0, std::min(127, outFaceEyePosX));
    outFaceEyePosY = std::max(0, std::min(127, outFaceEyePosY));

    // Smoothing of the output
    outFaceEyePosX = 0.9 * previousOutX + 0.1 * outFaceEyePosX;
    outFaceEyePosY = 0.9 * previousOutY + 0.1 * outFaceEyePosY;
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
 * Reset the internal movement state
 */
void resetMovements(bool resetBreathing) {
    // Reset all internal movement variables
    animation_joint1 = 0;
    animation_joint2 = 0;
    animation_joint3 = 0;
    saved_animation_joint = 0;
    demoInputAngle1 = gazeAngle1;
    demoInputAngle2 = gazeAngle2;
    demoInputAngle3 = gazeAngle3;
    animationPhase = 0;

    // Reset breathing
    if(resetBreathing) {
        breathPhase = 0;
        breath_add_joint2 = 0;
        breath_add_joint3 = 0;
    }
}

/**
 * Start the gaze
 */
void startGaze(float angle1, float angle2, float angle3, double moveTime) {
    gazeInProgress = true;
    gazeStartTime = ros::Time::now().toSec();
    gazeAngle1 = angle1;
    gazeAngle2 = angle2;
    gazeAngle3 = angle3;
    oldGazeAngle1 = outPoseJointAngle1;
    oldGazeAngle2 = outPoseJointAngle2;
    oldGazeAngle3 = outPoseJointAngle3;
    gazeMoveTime = moveTime;
}

/**
 * Start moving to the robot's default pose
 */
void startHoming() {
    startGaze(joint1HomeAngle, joint2HomeAngle, joint3HomeAngle, homingTime);
}

/**
 * Stops the blink action
 */
void stopBlink() {
    lastBlinkTime = secs;
    blinking = false;
    outFaceEmotionState = RamAnimatorConstants::EmotionNeutral;
    blinkTimes = 1 + round(float((rand() % 100)) / 100);
}

/**
 * Function to sweep a param during a given time
 */
double sweepParam(double currentTime, double startTime, double endTime, double min, double max) {
    return ((currentTime - startTime) / (endTime - startTime) * (max - min)) + min;
}

/**
 * Start moving to the midi input pose
 */
void transitionToMidiInput() {
    startGaze(midiInputAngle1, midiInputAngle2, midiInputAngle3, homingTime);
}

/**
 * Updates the natural movement of the robot (breathing)
 */
void updateBreath() {
    // Breathing
    breathPhase = breathPhase + (animFreq / breathDiv) / nodeUpdateRate;
    breath_add_joint2 = -anim_alfa * sin(breathPhase);
    breath_add_joint3 = anim_alfa * sin(breathPhase);
}

/**
 * Updates the blinking of the eyes
 */
void updateBlink() {
    // Blinking
    timeSinceLastBlink = (secs - lastBlinkTime);
    if ((timeSinceLastBlink > blinkInterval) && (outFaceEmotionState == RamAnimatorConstants::EmotionNeutral)) {
        blinking = true;
        blinkStartTime = secs;
    }

    if (blinking) {
        // Set to the sleepy/blinking emotion state
        outFaceEmotionState = RamAnimatorConstants::EmotionSleepy;
        // Move the eyes from open to closed and back with a cos shape
        outFaceEmotionLevel = int(127 - 63.5 * (1 + cos(blink_f * (secs - blinkStartTime))));

        // If you're already blinking for the blinkInterval time, stop blinking and set the lastblinktime to now
        if ((secs - blinkStartTime) > (blinkDuration * blinkTimes)) {
            stopBlink();
        }
    }
}

/**
 * Main program
 */
int main(int argc, char **argv) {
    // Initialize the ROS system.
    ros::init(argc, argv, "ram_animator_node");

    // Establish this program as a ROS node.
    ros::NodeHandle nh;
    ros::Rate loop_rate(nodeUpdateRate);

    // Create publisher objects for the joints and face state
    facePublisher = nh.advertise<ram_animation_msgs::Face>("/ram/animation/generated/face", 100, true);
    jointPublisher = nh.advertise<sensor_msgs::JointState>("/ram/animation/generated/joint", 100);

    //Status feedback publisher for asap
    generalFeedbackPublisher = nh.advertise<ram_output_msgs::GeneralFeedback>("/ram/asap/feedback/general", 100);

    // Subscribe to all incoming animations, desired face and joint state messages.
    animationSubscriber = nh.subscribe("/ram/animation/desired/animation", 100, &_animationMessageReceived);
    faceSubscriber = nh.subscribe("/ram/animation/desired/face", 100, &_faceMessageReceived);
    jointSubscriber = nh.subscribe("/ram/animation/desired/joint", 100, &_jointMessageReceived);

    while (ros::ok()) {
        secs = ros::Time::now().toSec();
        // Only if you are not running a sequence animation you should add the breathing
        if (!animationInProgress && !demo) {
            updateBreath();
        }

        // Do not blink in demo mode
        if (!demo) {
            updateBlink();
        }

        // Check for animation or gaze
        if (!gazeInProgress) {
            animate();
        } else {
            gaze();
        }

        // If you are not gazing
        if (!gazeInProgress) {
            // Add the breathing
            add_angle1 = animation_joint1;
            add_angle2 = breath_add_joint2 + animation_joint2;
            add_angle3 = breath_add_joint3 + animation_joint3;
            if (demo) {
                reqPoseJointAngle1 = demoInputAngle1;
                reqPoseJointAngle2 = demoInputAngle2;
                reqPoseJointAngle3 = demoInputAngle3;
            } else {
                reqPoseJointAngle1 = midiInputAngle1;
                reqPoseJointAngle2 = midiInputAngle2;
                reqPoseJointAngle3 = midiInputAngle3;
            }
        }

        // Manual control of the eyes only available if you are in default (midi) input mode and if the demo is not playing
        if (input == RamAnimatorConstants::InputDefault && !demo) {
            outFaceEyePosX = reqFaceEyePosX;
            outFaceEyePosY = reqFaceEyePosY;
        } else if (demo) {
            outFaceEyePosX = 64;
            outFaceEyePosY = 64;
        }

        // First wait for a gaze to go to the initial pose, then the saccade starts
        if (input == RamAnimatorConstants::InputSaccade && !gazeInProgress) {
            processSaccade();
        }

        // Add the breathing and such if you're not gazing
        if (!gazeInProgress) {
            summedAngle1 = reqPoseJointAngle1 + add_angle1;
            summedAngle2 = reqPoseJointAngle2 + add_angle2;
            summedAngle3 = reqPoseJointAngle3 + add_angle3;
        }

        // Smooth movement
        processSmoothing();

        // Compress movement
        processCompression();

        // Mapping for the face from desired to wishes of the ram_animator.
        // Only if you're not in demo
        if (!demo) {
            if (blinking && reqFaceEmotionState != RamAnimatorConstants::EmotionNeutral) {
                stopBlink();
            }
            if (!blinking) {
                outFaceEmotionState = reqFaceEmotionState;
                outFaceEmotionLevel = reqFaceEmotionLevel;
            }
        }

        // Limit outputs
        processLimiting();

        // Face message
        ram_animation_msgs::Face faceMsg;
        faceMsg.eyePosX = outFaceEyePosX;
        faceMsg.eyePosY = outFaceEyePosY;
        faceMsg.emotionState = outFaceEmotionState;
        faceMsg.emotionLevel = outFaceEmotionLevel;
        faceMsg.screenBrightness = outFaceScreenBrightness;

        // Joint message
        sensor_msgs::JointState jointMsg;
        jointMsg.name.resize(3);
        jointMsg.position.resize(3);
        jointMsg.name[0] = "joint1";
        jointMsg.name[1] = "joint2";
        jointMsg.name[2] = "joint3";
        jointMsg.position[0] = outPoseJointAngle1;
        jointMsg.position[1] = outPoseJointAngle2;
        jointMsg.position[2] = outPoseJointAngle3;

        // Publish the face message, but check for change
        // This is done to give the Arduino time some rest
        if (oldFaceMsg.emotionState != faceMsg.emotionState ||
            oldFaceMsg.emotionLevel != faceMsg.emotionLevel ||
            oldFaceMsg.eyePosX != faceMsg.eyePosX ||
            oldFaceMsg.eyePosY != faceMsg.eyePosY ||
            oldFaceMsg.screenBrightness != faceMsg.screenBrightness) {
            facePublisher.publish(faceMsg);
            oldFaceMsg = faceMsg;
        }

        // Publish joint update
        jointPublisher.publish(jointMsg);

        // Wait until it's time for another iteration.
        ros::spinOnce();
        loop_rate.sleep();
    }
}
