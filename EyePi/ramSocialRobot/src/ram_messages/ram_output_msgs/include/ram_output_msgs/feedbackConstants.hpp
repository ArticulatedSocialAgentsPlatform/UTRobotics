//
// Created by Bob on 022 22 7 2016.
//

#ifndef RAMSOCIALROBOT_FEEDBACKCONSTANTS_HPP
#define RAMSOCIALROBOT_FEEDBACKCONSTANTS_HPP

namespace FeedbackConstants {

    // Type definitions
    const char FeedbackType = 1;
    const char ProgressType = 2;

    // Feedback type
    const char FeedbackMessageNack = 0;
    const char FeedbackMessageAck = 1;
    const char FeedbackReasonEmpty = 0;
    const char FeedbackReasonFluency = 1;
    const char FeedbackReasonTooSoon = 2;
    const char FeedbackReasonInvalid = 3;

    // Progress type
    const char ProgressMessageStart = 1;
    const char ProgressMessageStroke = 2;
    const char ProgressMessageEnd = 3;
    const char ProgressReasonEmpty = 0;
}

#endif //RAMSOCIALROBOT_FEEDBACKCONSTANTS_HPP
