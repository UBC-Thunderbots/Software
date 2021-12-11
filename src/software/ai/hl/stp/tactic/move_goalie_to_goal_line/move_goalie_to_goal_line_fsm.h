#pragma once
#include "software/ai/hl/stp/tactic/move/move_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/hl/stp/tactic/transition_conditions.h"
#include "software/ai/intent/move_intent.h"
#include "software/geom/algorithms/acute_angle.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/intersection.h"
#include "software/geom/ray.h"
#include "software/logger/logger.h"

// Which way the crease defender should be aligned
struct MoveGoalieToGoalLineFSM
{
    struct ControlParams
    {
    };

    DEFINE_TACTIC_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * This is an Action that blocks the threat
     *
     * @param event MoveGoalieToGoalLineFSM::Update event
     */
    void moveToGoalLine(const Update &event,
                        boost::sml::back::process<MoveFSM::Update> processEvent);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(MoveFSM)
        DEFINE_SML_EVENT(Update)

        DEFINE_SML_SUB_FSM_UPDATE_ACTION(moveToGoalLine, MoveFSM)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *MoveFSM_S + Update_E / moveToGoalLine_A, MoveFSM_S = X,
            X + Update_E / moveToGoalLine_A = MoveFSM_S);
    }
};
