//
// Created by Bob on 022 22 7 2016.
//

#ifndef RAMSOCIALROBOT_INPUTCONSTANTS_HPP
#define RAMSOCIALROBOT_INPUTCONSTANTS_HPP

namespace InputConstants {

    // Type definitions
    const char SequenceStartAtType = 1;         // Start sequence at requested time
    const char SequenceStrokeAtType = 2;        // Have the (first) sequence stroke at requested time
    const char SequenceEndAtType = 3;           // Have the end of the sequence at requested time

    // Predict timing
    const char SequencePredictType = 4;         // THis request return the predicted timing

}

#endif //RAMSOCIALROBOT_INPUTCONSTANTS_HPP
