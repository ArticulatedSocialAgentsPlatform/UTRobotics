/**
 * Default small sequences, which are relatively easy
 */
void animateSequences() {

    double sinResult = sin(animationPhase);
    double halfSinResult = sin(animationPhase / 2);

    if (sequence == RamAnimatorConstants::SequenceNodding) {
        // Nodding sequence
        animation_joint3 = animAngle * sinResult;

        if (sinResult >= 0.99 && !progressFeedbackSend) {
            // Send feedback message
            publishGeneralFeedback(
                    FeedbackConstants::ProgressType,
                    animationId,
                    FeedbackConstants::ProgressMessageStroke,
                    FeedbackConstants::ProgressReasonEmpty
            );
            progressFeedbackSend = true;
        }

        if(sinResult <= -0.99 && progressFeedbackSend){
            progressFeedbackSend = false;
        }
    }

    else if (sequence == RamAnimatorConstants::SequenceDancing) {
        // Dancing sequence
        animation_joint1 = 3.0 * animAngle * halfSinResult;
        animation_joint2 = 0.8 * animAngle * sinResult;
        animation_joint3 = 1.8 * animAngle * cos(animationPhase);
    }

    else if (sequence == RamAnimatorConstants::SequenceShaking) {
        // Shaking sequence
        animation_joint1 = 1.0 * animAngle * sinResult;

        if (sinResult >= 0.99 && !progressFeedbackSend) {
            // Send feedback message
            publishGeneralFeedback(
                    FeedbackConstants::ProgressType,
                    animationId,
                    FeedbackConstants::ProgressMessageStroke,
                    FeedbackConstants::ProgressReasonEmpty
            );
            progressFeedbackSend = true;
        }

        if(sinResult <= -0.99 && progressFeedbackSend){
            // Send feedback message
            publishGeneralFeedback(
                    FeedbackConstants::ProgressType,
                    animationId,
                    FeedbackConstants::ProgressMessageStroke,
                    FeedbackConstants::ProgressReasonEmpty
            );
            progressFeedbackSend = false;
        }
    }

    else if (
            sequence == RamAnimatorConstants::SequencePointingLeft ||
            sequence == RamAnimatorConstants::SequencePointingMiddle ||
            sequence == RamAnimatorConstants::SequencePointingRight) {
        // Pointing sequence (point is the same for all directions)

        if (newAnimation) {
            if (sequence == RamAnimatorConstants::SequencePointingLeft ||
                sequence == RamAnimatorConstants::SequencePointingRight) {
                animation_joint1 = 4.5 * animAngle * halfSinResult;
                if (sequence == RamAnimatorConstants::SequencePointingRight) {
                    // Change direction
                    animation_joint1 = animation_joint1 * -1.0;
                }
            }

            animation_joint2 = -3.0 * animAngle * halfSinResult;
            animation_joint3 = -1.0 * animAngle * halfSinResult;

            // Check if we reached the end point
            if (halfSinResult >= 0.99) {
                newAnimation = false;
                nodAmount = 0;

                // Save the current state
                if (sequence == RamAnimatorConstants::SequencePointingMiddle) {
                    saved_animation_joint = animation_joint3;
                } else {
                    saved_animation_joint = animation_joint1;
                }

                // Send feedback message
                publishGeneralFeedback(
                        FeedbackConstants::ProgressType,
                        animationId,
                        FeedbackConstants::ProgressMessageStroke,
                        FeedbackConstants::ProgressReasonEmpty
                );
            }
        } else if (nodAmount < 3) {

            // Update the nod
            double fourSinResult = sin(animationPhase * 4);
            if (sequence == RamAnimatorConstants::SequencePointingLeft) {
                animation_joint1 = saved_animation_joint + 0.3 * animAngle * fourSinResult;
            } else if (sequence == RamAnimatorConstants::SequencePointingRight) {
                animation_joint1 = saved_animation_joint - 0.3 * animAngle * fourSinResult;
            } else {
                animation_joint3 = saved_animation_joint + 0.3 * animAngle * fourSinResult;
            }

            // Count the amount of nods
            if (nodAmount == 0 && fourSinResult <= -0.99) {
                nodAmount++;
            } else if (nodAmount == 1 && fourSinResult >= 0.98) {
                nodAmount++;
            } else if (nodAmount == 2 && fourSinResult <= -0.99) {
                nodAmount++;
            }

            // Update the blink
            updateBlink();
        } else {
            // Update the breath and blink after move
            updateBreath();
            updateBlink();
        }

    }
}