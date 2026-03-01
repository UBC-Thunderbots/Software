#pragma once

#include <memory>

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/play_fsm.hpp"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"

struct StopPlayFSM
{
    struct ControlParams
    {
    };
    class StopState;

    struct Update
    {
        Update(const ControlParams& control_params, const PlayUpdate& common)
            : control_params(control_params), common(common)
        {
        }
        ControlParams control_params;
        PlayUpdate common;
    };

    /**
     * Creates a Stop Play FSM
     *
     * @param ai_config_ptr shared pointer to the play config for this FSM
     */
    explicit StopPlayFSM(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr);

    /**
     * Action to position robots during Stop (goalie handled by Play; this sets
     * crease defenders and robots near the ball).
     *
     * @param event the StopPlayFSM Update event
     */
    void updateStopPosition(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(StopState)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(updateStopPosition)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *StopState_S + Update_E / updateStopPosition_A = StopState_S,

            X + Update_E / updateStopPosition_A = X);
    }

   private:
    std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr;
    std::vector<std::shared_ptr<MoveTactic>> move_tactics;
    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics;
};
