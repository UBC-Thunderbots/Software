//
// Created by grayson on 2024-10-19.
//

#ifndef BAZEL_BUILD_THUNDERBOTS_HALT_PLAY_FSM_H
#define BAZEL_BUILD_THUNDERBOTS_HALT_PLAY_FSM_H

#endif //BAZEL_BUILD_THUNDERBOTS_HALT_PLAY_FSM_H


#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/defense/defense_play_base.h"
#include "software/ai/hl/stp/play/play_fsm.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/pass_defender/pass_defender_tactic.h"
#include "software/logger/logger.h"

class HaltPlayFSM{
    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(StopState)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(updateStop)

        return make_transition_table(
                // src_state + event [guard] / action = dest_state
                *StopState_S + Update_E / updateStop_A = StopState_S,
                StopState_S + Update_E / updateStop_A  = X,
                X + Update_E / updateStop_A            = StopState_S,
                X + Update_E /updateStop_A             = X);
    }
};