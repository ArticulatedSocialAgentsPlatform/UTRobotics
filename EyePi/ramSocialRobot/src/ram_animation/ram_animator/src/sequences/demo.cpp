/**
 * Demo sequence, which is preprogrammed
 */
void animateDemo() {

    // Start by gaze to starting prosition
    demoPlayTime = secs - demoStartTime - 1;
    if (demoPlayTime < 0) {
        if (!gazeInProgress) { startGaze(0, 0.4 * 3.14, 0.4 * 3.14, 1); }
        return;
    }

    // Sleepy: Fall asleep
    if (demoPlayTime < 5 && demoPlayTime > 0) {
        sequence = RamAnimatorConstants::SequenceNodding;
        outFaceEmotionState = RamAnimatorConstants::EmotionSleepy;
        outFaceEmotionLevel = int(sweepParam(demoPlayTime, 0, 5, 64, 115));
        animFreq = 0.5 * 2 * 3.14;
        animAngle = 1.0 / 15;
        demoInputAngle1 = 0;
        demoInputAngle2 = sweepParam(demoPlayTime, 0, 5, 0.4 * 3.14, 0.7);
        demoInputAngle3 = sweepParam(demoPlayTime, 0, 5, 0.4 * 3.14, 1.2);
        return;
    }

    // Sleepy: Sleeping
    if (demoPlayTime > 5 && demoPlayTime < 10) {
        sequence = RamAnimatorConstants::SequenceNodding;
        outFaceEmotionState = RamAnimatorConstants::EmotionSleepy;
        outFaceEmotionLevel = 115;
        animFreq = 0.3 * 2 * 3.14;
        animAngle = 1.0 / 15;
        demoInputAngle1 = 0;
        demoInputAngle2 = 0.7;
        demoInputAngle3 = 1.2;
        return;
    }

    // Sleepy: Wake up
    if (demoPlayTime > 10 && demoPlayTime < 13) {
        sequence = RamAnimatorConstants::SequenceShaking;
        outFaceEmotionState = RamAnimatorConstants::EmotionSleepy;
        outFaceEmotionLevel = int(sweepParam(demoPlayTime, 10, 13, 115, 0));
        animFreq = sweepParam(demoPlayTime, 10, 13, 0.2 * 2 * 3.14, 3 * 2 * 3.14);
        animAngle = sweepParam(demoPlayTime, 10, 13, 0, 2.0 / 15);
        demoInputAngle1 = 0;
        demoInputAngle2 = sweepParam(demoPlayTime, 10, 13, 0.7, 0.4 * 3.14);
        demoInputAngle3 = sweepParam(demoPlayTime, 10, 13, 1.2, 0.4 * 3.14);
        return;
    }

    // Neutral: Look up right
    if (demoPlayTime > 13 && demoPlayTime < 13.1) {
        outFaceEmotionState = RamAnimatorConstants::EmotionNeutral;
        if (!gazeInProgress) { startGaze(-1, 1.4, 0.4, 2); }
        return;
    }

    // Neutral: Look upwards, cutesy up from left
    if (demoPlayTime > 15 && demoPlayTime < 15.1) {
        if (!gazeInProgress) { startGaze(1, 0.2, -0.6, 2); }
        return;
    }

    // Neutral: Back to center
    if (demoPlayTime > 17 && demoPlayTime < 17.1) {
        if (!gazeInProgress) { startGaze(0, 0.4 * 3.14, 0.4 * 3.14, 2); }
        return;
    }

    // Happy: Dance
    if (demoPlayTime > 19 && demoPlayTime < 25) {
        sequence = RamAnimatorConstants::SequenceDancing;
        outFaceEmotionState = RamAnimatorConstants::EmotionExcited;
        outFaceEmotionLevel = int(sweepParam(demoPlayTime, 19, 25, 50, 120));
        animFreq = sweepParam(demoPlayTime, 19, 25, 1, 1.6 * 2 * 3.14);
        animAngle = sweepParam(demoPlayTime, 19, 25, 0.9 / 15, 1.5 / 15);
        demoInputAngle1 = 0;
        demoInputAngle2 = 0.4 * 3.14;
        demoInputAngle3 = 0.4 * 3.14;
        return;
    }

    // Back to center
    if (demoPlayTime > 25 && demoPlayTime < 25.1) {
        if (!gazeInProgress) { startGaze(0, 0.4 * 3.14, 0.4 * 3.14, 2); }
        return;
    }

    // Sad: Shake and drop head
    if (demoPlayTime > 27 && demoPlayTime < 33) {
        smoothing = smoothingMax;
        sequence = RamAnimatorConstants::SequenceShaking;
        outFaceEmotionState = RamAnimatorConstants::EmotionSad;
        outFaceEmotionLevel = int(sweepParam(demoPlayTime, 27, 33, 50, 127));
        animFreq = 0.7 * 2 * 3.14;
        animAngle = sweepParam(demoPlayTime, 27, 33, 1.2 / 15, 2.0 / 15);
        demoInputAngle1 = 0;
        demoInputAngle2 = sweepParam(demoPlayTime, 27, 33, 0.4 * 3.14, 0.4);
        demoInputAngle3 = sweepParam(demoPlayTime, 27, 33, 0.4 * 3.14, 0.3 * 3.14);
        return;
    }

    // Sad: look up
    if (demoPlayTime > 34 && demoPlayTime < 36) {
        sequence = RamAnimatorConstants::SequenceEmpty;
        demoInputAngle2 = sweepParam(demoPlayTime, 34, 36, 0.4, 0.5);
        demoInputAngle3 = sweepParam(demoPlayTime, 34, 36, 0.3 * 3.14, 0.1);
        return;
    }

    // Sad: Look up left
    if (demoPlayTime > 38 && demoPlayTime < 38.1) {
        if (!gazeInProgress) { startGaze(1, 1.4, 0.7, 2); }
        return;
    }

    // Sad: shake and head moving down to right
    if (demoPlayTime > 42 && demoPlayTime < 47) {
        sequence = RamAnimatorConstants::SequenceShaking;
        outFaceEmotionState = RamAnimatorConstants::EmotionSad;
        outFaceEmotionLevel = 127;
        animFreq = sweepParam(demoPlayTime, 42, 47, 2.0 / 15, 0.5 / 15);
        animAngle = sweepParam(demoPlayTime, 42, 47, 1.1 / 15, 1.1 / 15);
        demoInputAngle1 = sweepParam(demoPlayTime, 42, 47, 1, -1.2);
        demoInputAngle2 = sweepParam(demoPlayTime, 42, 47, 1.4, 0.4);
        demoInputAngle3 = sweepParam(demoPlayTime, 42, 47, 0.4, 0.3 * 3.14);
        return;
    }

    // Back to center
    if (demoPlayTime > 48 && demoPlayTime < 48.1) {
        if (!gazeInProgress) { startGaze(0, 0.4 * 3.14, 0.4 * 3.14, 3); }
        return;
    }

    // Angry: look forward
    if (demoPlayTime > 51 && demoPlayTime < 54) {
        smoothing = smoothingMin;
        sequence = RamAnimatorConstants::SequenceShaking;
        outFaceEmotionState = RamAnimatorConstants::EmotionAngry;
        outFaceEmotionLevel = int(sweepParam(demoPlayTime, 51, 54, 40, 115));
        animFreq = 0.5 * 2 * 3.14;
        animAngle = 2.0 / 15;
        demoInputAngle2 = sweepParam(demoPlayTime, 51, 54, 0.4 * 3.14, 0);
        demoInputAngle3 = sweepParam(demoPlayTime, 51, 54, 0.4 * 3.14, 0);
        return;
    }

    // Angry: Shake head moving backwards
    if (demoPlayTime > 54 && demoPlayTime < 56) {
        sequence = RamAnimatorConstants::SequenceShaking;
        outFaceEmotionState = RamAnimatorConstants::EmotionAngry;
        outFaceEmotionLevel = 120;
        animFreq = 2 * 2 * 3.14;
        animAngle = sweepParam(demoPlayTime, 54, 56, 1.1 / 15, 3.0 / 15);
        demoInputAngle1 = 0;
        demoInputAngle2 = sweepParam(demoPlayTime, 54, 56, 0, 0.4 * 3.14);
        demoInputAngle3 = sweepParam(demoPlayTime, 54, 56, 0, 0.4 * 3.14);
        return;
    }

    // Angry: Shake
    if (demoPlayTime > 56 && demoPlayTime < 58) {
        sequence = RamAnimatorConstants::SequenceShaking;
        outFaceEmotionState = RamAnimatorConstants::EmotionAngry;
        outFaceEmotionLevel = 120;
        animFreq = 1 * 2 * 3.14;
        animAngle = 1.0 / 15;
        return;
    }

    // Look up right
    if (demoPlayTime > 58 && demoPlayTime < 58.1) {
        if (!gazeInProgress) { startGaze(-1, 1.4, 0.4, 1.5); }
        return;
    }

    // Look upwards from left
    if (demoPlayTime > 60 && demoPlayTime < 60.1) {
        if (!gazeInProgress) { startGaze(1, 0.2, -0.6, 2.5); }
        return;
    }

    // Back to center
    if (demoPlayTime > 63 && demoPlayTime < 63.1) {
        if (!gazeInProgress) { startGaze(0, 0.4 * 3.14, 0.4 * 3.14, 1); }
        return;
    }

    // Amazed: move head back
    if (demoPlayTime > 64 && demoPlayTime < 64.5) {
        sequence = RamAnimatorConstants::SequenceEmpty;
        outFaceEmotionState = RamAnimatorConstants::EmotionAmazed;
        outFaceEmotionLevel = int(sweepParam(demoPlayTime, 64, 64.5, 0, 35));
        demoInputAngle1 = 0;
        demoInputAngle2 = sweepParam(demoPlayTime, 64, 64.5, 0.4 * 3.14, 0.5 * 3.14);
        demoInputAngle3 = sweepParam(demoPlayTime, 64, 64.5, 0.4 * 3.14, 0.5 * 3.14);
        return;
    }

    // Amazed: move head forward amazed
    if (demoPlayTime > 64.5 && demoPlayTime < 65) {
        sequence = RamAnimatorConstants::SequenceEmpty;
        outFaceEmotionState = RamAnimatorConstants::EmotionAmazed;
        outFaceEmotionLevel = int(sweepParam(demoPlayTime, 64.5, 65, 35, 127));
        demoInputAngle1 = 0;
        demoInputAngle2 = sweepParam(demoPlayTime, 64.5, 65, 0.5 * 3.14, 0);
        demoInputAngle3 = sweepParam(demoPlayTime, 64.5, 65, 0.5 * 3.14, -0.3);
        return;
    }

    // Amazed: restore
    if (demoPlayTime > 65 && demoPlayTime < 66) {
        sequence = RamAnimatorConstants::SequenceEmpty;
        outFaceEmotionState = RamAnimatorConstants::EmotionAmazed;
        outFaceEmotionLevel = int(sweepParam(demoPlayTime, 65, 66, 127, 70));
        demoInputAngle1 = 0;
        demoInputAngle2 = sweepParam(demoPlayTime, 65, 66, 0, 0.4 * 3.14);
        demoInputAngle3 = sweepParam(demoPlayTime, 65, 66, -0.3, 0.4 * 3.14);
        return;
    }

    // Restart the demo sequence
    if (demoPlayTime > 66) { demoStartTime = secs; }
}
