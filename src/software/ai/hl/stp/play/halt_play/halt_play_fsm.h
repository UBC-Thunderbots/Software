//
// Created by grayson on 2024-10-19.
//

#ifndef BAZEL_BUILD_THUNDERBOTS_HALT_PLAY_FSM_H
#define BAZEL_BUILD_THUNDERBOTS_HALT_PLAY_FSM_H

#endif //BAZEL_BUILD_THUNDERBOTS_HALT_PLAY_FSM_H


#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/play_fsm.h"
#include "software/logger/logger.h"
#include "software/ai/hl/stp/tactic/stop/stop_tactic.h"
#include "software/ai/hl/stp/play/play_fsm.h"

class HaltPlayFSM{
    struct ControlParams
    {
    };
    class HaltState;

    DEFINE_PLAY_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    void updateStop(const Update& event);

    private:
        PriorityTacticVector halt_tactics;

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(HaltState)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(updateStop)

        return make_transition_table(
                // src_state + event [guard] / action = dest_state
                *HaltState_S + Update_E / updateStop_A = HaltState_S,

                X + Update_E /updateStop_A             = X);
    }
};