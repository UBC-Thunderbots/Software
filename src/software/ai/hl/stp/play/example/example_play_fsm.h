#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/logger/logger.h"

/**
 * Control parameters for Example Play
 */
struct ExamplePlayControlParams
{
};

/**
 * An example play that moves the robots in a circle around the ball
 */
struct ExamplePlayFSM : PlayFSM<ExamplePlayControlParams>
{
    class MoveState;

    /**
     * Creates an example play FSM
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit ExamplePlayFSM(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr);

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
