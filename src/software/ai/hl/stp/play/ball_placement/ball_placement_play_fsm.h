#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/play_fsm.h"
#include "software/ai/hl/stp/tactic/dribble/dribble_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"


using Zones = std::unordered_set<EighteenZoneId>;

struct BallPlacementPlayFSM
{
    class PlaceBallState;

    struct ControlParams
    {
    };

    DEFINE_PLAY_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Creates a ball placement play FSM
     *
     * @param ai_config the play config for this play FSM
     */
    explicit BallPlacementPlayFSM(TbotsProto::AiConfig ai_config);

    /**
     * Action that has the placing robot dribble the ball to the destination, while other
     * robots line up outside of the defense area
     *
     * @param event the BallPlacementPlayFSM Update event
     */
    void placeBall(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(PlaceBallState)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(placeBall)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *PlaceBallState_S + Update_E / placeBall_A = PlaceBallState_S);
    }

   private:
    TbotsProto::AiConfig ai_config;
    std::shared_ptr<DribbleTactic> place_ball_tactic;
    std::vector<std::shared_ptr<MoveTactic>> move_tactics;
};
