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
    class RetreatState;

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
     * Action that has the placing robot retreat after placing the ball
     *
     * @param event the BallPlacementPlayFSM Update event
     */
    void retreat(const Update& event);

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

    /**
     * Guard on whether the placing robot has finished placing the ball into the desired
     * position
     *
     * @param event the BallPlacementPlayFSM Update event
     *
     * @return whether the kick has been performed
     */
    bool ballPlaced(const Update& event);

    /**
     * Helper function for calculating the angle to kick the ball off of a wall
     *
     * @param ball_pos
     * @param field_lines
     *
     * @return the kick angle
     */
    Angle calculateWallKickoffAngle(const Point& ball_pos, const Rectangle& field_lines);

    /**
     * Helper function for setting up the MoveTactics of the robots away from the ball
     * placing robot
     *
     * @param event the BallPlacementPlayFSM Update event
     */
    void setupMoveTactics(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(StartState)
        DEFINE_SML_STATE(KickOffWallState)
        DEFINE_SML_STATE(PlaceBallState)
        DEFINE_SML_STATE(RetreatState)
        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(placeBall)
        DEFINE_SML_ACTION(kickOffWall)
        DEFINE_SML_ACTION(retreat)

        DEFINE_SML_GUARD(shouldKickOffWall)
        DEFINE_SML_GUARD(kickDone)
        DEFINE_SML_GUARD(ballPlaced)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *StartState_S + Update_E[shouldKickOffWall_G]       = KickOffWallState_S,
            *StartState_S + Update_E[!shouldKickOffWall_G]      = PlaceBallState_S,
            KickOffWallState_S + Update_E[!shouldKickOffWall_G] = PlaceBallState_S,
            KickOffWallState_S + Update_E[!kickDone_G] / kickOffWall_A =
                KickOffWallState_S,
            KickOffWallState_S + Update_E[kickDone_G]                = StartState_S,
            PlaceBallState_S + Update_E[shouldKickOffWall_G]         = StartState_S,
            PlaceBallState_S + Update_E[!ballPlaced_G] / placeBall_A = PlaceBallState_S,
            PlaceBallState_S + Update_E[ballPlaced_G] / retreat_A    = RetreatState_S,
            RetreatState_S + Update_E[!ballPlaced_G]                 = StartState_S,
            RetreatState_S + Update_E[ballPlaced_G] / retreat_A      = X,
            X + Update_E[!ballPlaced_G]                              = StartState_S);
    }

   private:
    TbotsProto::AiConfig ai_config;
    std::shared_ptr<WallKickoffTactic> pivot_kick_tactic;
    std::shared_ptr<PlaceBallTactic> place_ball_tactic;
    std::shared_ptr<MoveTactic> retreat_tactic;
    std::vector<std::shared_ptr<PlaceBallMoveTactic>> move_tactics;
    constexpr static double const SHOT_VELOCITY_THRESHOLD_M_PER_S = 1.0;
    constexpr static double const WALL_KICKOFF_VELOCITY_M_PER_S = 3.0;
};
