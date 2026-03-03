#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/play_fsm.hpp"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"

struct MoveTestPlayFSM : PlayFSM<MoveTestPlayFSM>
{
    struct ControlParams
    {
    };

    class MoveTestState;

    explicit MoveTestPlayFSM(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr);
    void updateMove(const Update& event);
    bool moveDone(const Update& event);


    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(MoveTestState)
        DEFINE_SML_EVENT(Update)
        DEFINE_SML_ACTION(updateMove)
        DEFINE_SML_GUARD(moveDone)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *MoveTestState_S + Update_E[!moveDone_G] / updateMove_A = MoveTestState_S,
            MoveTestState_S + Update_E[moveDone_G] / updateMove_A   = X,
            X + Update_E / updateMove_A                             = X);
    }

   private:
    std::shared_ptr<MoveTactic> move_test_tactic_friendly_goal;
    std::shared_ptr<MoveTactic> move_test_tactic_enemy_goal;
    std::shared_ptr<MoveTactic> move_test_tactic_center_field;
};
