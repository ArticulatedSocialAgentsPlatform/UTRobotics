//
// Created by Bob on 019 19 8 2016.
//

// ANIMATION CONFIGURABLE PARAMETERS
// Update rate of messages for this node [= 100 Hz. DO NOT CHANGE, SMOOTHING AND SACCADE SMOOTHING IS LINKED TO THIS AND IT'S NOT DONE PROPERLY YET!!!]
int nodeUpdateRate = 100;
// Base Frequency in rad/s of the sequences. (f*2*Pi) (for music, BPM/60 is the frequency you want, 128BPM is 2.133333Hz so 2.13*2*Pi)
double animFreqMin = 1.0666666 * 2 * 3.14;
double animFreqMax = 3.2 * 2 * 3.14;

/**
 * This function returns the estimated sequence duration based on the current motion parameters
 * The duration is of one complete sequence motion
 * @param sequence
 * @param timing
 * @return
 */
double _getSequenceDuration(char sequence, double frequency) {

    // Frequency of the sequence in Hz
    ROS_DEBUG("animFreq: %f", (animFreqMin + frequency * (animFreqMax - animFreqMin)));
    double val = 6.28 / (animFreqMin + frequency * (animFreqMax - animFreqMin)) * 1.2;
    double baseDuration = ceilf(val * 100) / 100;
    ROS_DEBUG("Original base duration: %f", val, baseDuration);

    switch (sequence) {
        case RamAnimatorConstants::SequenceDancing:
            return baseDuration * 2;
        case RamAnimatorConstants::SequencePointingLeft:
        case RamAnimatorConstants::SequencePointingMiddle:
        case RamAnimatorConstants::SequencePointingRight:
            return baseDuration * 1.5;
        case RamAnimatorConstants::SequenceNodding:
        case RamAnimatorConstants::SequenceShaking:
        default:
            return baseDuration;
    }

}

/**
 * This function returns the estimated duration until the first stroke based on the current motion parameters
 * @param sequence
 * @param frequency
 * @return
 */
double _getSequenceStrokeDuration(char sequence, double timing) {
    switch (sequence) {
        case RamAnimatorConstants::SequencePointingLeft:
        case RamAnimatorConstants::SequencePointingMiddle:
        case RamAnimatorConstants::SequencePointingRight:
            return (timing / 3) * 2;
        case RamAnimatorConstants::SequenceNodding:
        case RamAnimatorConstants::SequenceShaking:
        case RamAnimatorConstants::SequenceDancing:
        default:
            // Set a default for a sequence that might be close
            return timing / 2;
    }
}