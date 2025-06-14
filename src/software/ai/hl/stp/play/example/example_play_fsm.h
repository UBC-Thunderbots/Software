#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/logger/logger.h"

/**
 * An example play that moves the robots in a circle around the ball
 */
struct ExamplePlayFSM
{
    class MoveState;

    struct ControlParams
    {
    };

    DEFINE_PLAY_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Creates an example play FSM
     */
    explicit ExamplePlayFSM();

    /**
     * Action that moves the robots to certain positions around the ball
     *
     * @param event the ExamplePlayFSM Update event
     */
    void moveToPosition(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(MoveState)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(moveToPosition)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *MoveState_S + Update_E / moveToPosition_A = MoveState_S, X + Update_E = X);
    }

   private:
    std::vector<std::shared_ptr<MoveTactic>> move_tactics;
};
