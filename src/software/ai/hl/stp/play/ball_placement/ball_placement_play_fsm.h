#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/play_fsm.h"
#include "software/ai/hl/stp/tactic/dribble/dribble_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/pivot_kick/pivot_kick_tactic.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"


using Zones = std::unordered_set<EighteenZoneId>;

struct BallPlacementPlayFSM
{
    class StartState;
    class KickOffWallState;
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
     * Action that has the placing kick the ball off the wall to give more space to
     * dribble
     *
     * @param event the BallPlacementPlayFSM Update event
     */
    void kickOffWall(const Update& event);

    /**
     * Action that has the placing robot dribble the ball to the destination, while other
     * robots line up outside of the defense area
     *
     * @param event the BallPlacementPlayFSM Update event
     */
    void placeBall(const Update& event);

    /**
     * Guard on whether the ball is in a "dead zone"
     *
     * @param event the BallPlacementPlayFSM Update event
     *
     * @return whether the ball is in a "dead zone"
     */
    bool shouldKickOffWall(const Update& event);

    /**
     * Guard on whether the placing robot has finished moving the ball into a better
     * position with a kick
     *
     * @param event the BallPlacementPlayFSM Update event
     *
     * @return whether the kick has been performed
     */
    bool kickDone(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(StartState)
        DEFINE_SML_STATE(KickOffWallState)
        DEFINE_SML_STATE(PlaceBallState)
        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(placeBall)
        DEFINE_SML_ACTION(kickOffWall)

        DEFINE_SML_GUARD(shouldKickOffWall)
        DEFINE_SML_GUARD(kickDone)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *StartState_S + Update_E[shouldKickOffWall_G]  = KickOffWallState_S,
            *StartState_S + Update_E[!shouldKickOffWall_G] = PlaceBallState_S,
            KickOffWallState_S + Update_E[!kickDone_G] / kickOffWall_A =
                KickOffWallState_S,
            KickOffWallState_S + Update_E[kickDone_G] = StartState_S,
            PlaceBallState_S + Update_E / placeBall_A = PlaceBallState_S);
    }

   private:
    TbotsProto::AiConfig ai_config;
    std::shared_ptr<WallKickoffTactic> pivot_kick_tactic;
    std::shared_ptr<PlaceBallTactic> place_ball_tactic;
    std::vector<std::shared_ptr<PlaceBallMoveTactic>> move_tactics;
};
