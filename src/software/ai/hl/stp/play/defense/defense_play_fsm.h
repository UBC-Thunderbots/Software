#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/defense/defense_play_base.h"
#include "software/ai/hl/stp/play/play_fsm.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/pass_defender/pass_defender_tactic.h"
#include "software/logger/logger.h"

struct DefensePlayFSM : public DefensePlayFSMBase
{
    class DefenseState;

    /**
     * Creates a defense play FSM
     *
     * @param strategy the Strategy shared by all of AI
     */
    explicit DefensePlayFSM(std::shared_ptr<Strategy> strategy);

    /**
     * Action to identify all immediate enemy threats and assign
     * defenders to block enemy shots and passes
     *
     * @param event the FSM event
     */
    void defendAgainstThreats(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(DefenseState)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(defendAgainstThreats)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *DefenseState_S + Update_E / defendAgainstThreats_A = DefenseState_S,
            X + Update_E                                        = X);
    }
};
