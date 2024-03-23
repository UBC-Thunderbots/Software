#pragma once

#include <chrono>

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/play_fsm.h"
#include "software/ai/hl/stp/tactic/assigned_skill/named_skill_tactics.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"


using Zones = std::unordered_set<EighteenZoneId>;

struct BallPlacementPlayFSM
{
    class StartState;
    class KickOffWallState;
    class AlignPlacementState;
    class PlaceBallState;
    class WaitState;
    class RetreatState;

    struct ControlParams
    {
    };

    DEFINE_PLAY_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Creates a ball placement play FSM
     *
     * @param strategy the Strategy
     */
    explicit BallPlacementPlayFSM(std::shared_ptr<Strategy> strategy);

    /**
     * Action that has the placing robot kick the ball off the wall to give more space to
     * dribble
     *
     * @param event the BallPlacementPlayFSM Update event
     */
    void kickOffWall(const Update& event);

    /**
     * Action that finds a point where the ball's current position aligns with the ball
     * placement position, and moves the robot so that it is aligned to dribble straight
     * directly to the placement point
     *
     * @param event the BallPlacementPlayFSM Update event
     */
    void alignPlacement(const Update& event);

    /**
     * Action that has the placing robot dribble the ball to the destination, while other
     * robots line up outside of the defense area
     *
     * @param event the BallPlacementPlayFSM Update event
     */
    void placeBall(const Update& event);

    /**
     * Action that waits stationary 5 seconds for the dribbler to stop spinning
     *
     * @param event the BallPlacementPlayFSM Update event
     */
    void startWait(const Update& event);

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
     * Guard on whether the robot is aligned with the ball and placement point
     *
     * @param event the BallPlacementPlayFSM Update event
     * @return whether the robot is in position to begin placing the ball
     */
    bool alignDone(const Update& event);

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
     * @return whether the ball has been placed into the desired position
     */
    bool ballPlaced(const Update& event);

    /**
     * Guard on whether the robot has waited sufficiently
     * @param event the BallPlacementPlayFSM Update event
     * @return whether the robot has waited for 5 seconds
     */
    bool waitDone(const Update& event);

    /**
     * Guard on whether the robot has retreated outside of the required range
     * @param event the BallPlacementPlayFSM Update event
     * @return whether the robot has retreated outside of the required range
     */
    bool retreatDone(const Update& event);

    /**
     * Helper function for calculating the angle to kick the ball off of a wall
     *
     * @param ball_pos the ball position to use when calculating the kick angle
     * @param field_lines the field lines of the playing area
     *
     * @return the kick angle
     */
    Angle calculateWallKickoffAngle(const Point& ball_pos, const Rectangle& field_lines);

    /**
     * Helper function that populates the move_tactics field with MoveTactics that
     * organize the robots away from the ball placing robot
     *
     * @param event the BallPlacementPlayFSM Update event
     */
    void setupMoveTactics(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(StartState)
        DEFINE_SML_STATE(KickOffWallState)
        DEFINE_SML_STATE(AlignPlacementState)
        DEFINE_SML_STATE(PlaceBallState)
        DEFINE_SML_STATE(WaitState)
        DEFINE_SML_STATE(RetreatState)
        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(alignPlacement)
        DEFINE_SML_ACTION(placeBall)
        DEFINE_SML_ACTION(kickOffWall)
        DEFINE_SML_ACTION(startWait)
        DEFINE_SML_ACTION(retreat)

        DEFINE_SML_GUARD(shouldKickOffWall)
        DEFINE_SML_GUARD(alignDone)
        DEFINE_SML_GUARD(kickDone)
        DEFINE_SML_GUARD(ballPlaced)
        DEFINE_SML_GUARD(waitDone)
        DEFINE_SML_GUARD(retreatDone)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *StartState_S + Update_E[!shouldKickOffWall_G] / alignPlacement_A =
                AlignPlacementState_S,
            StartState_S + Update_E[shouldKickOffWall_G] = KickOffWallState_S,
            KickOffWallState_S + Update_E[!kickDone_G && shouldKickOffWall_G] /
                                     kickOffWall_A                = KickOffWallState_S,
            KickOffWallState_S + Update_E[kickDone_G]             = KickOffWallState_S,
            KickOffWallState_S + Update_E[!kickDone_G]            = AlignPlacementState_S,
            AlignPlacementState_S + Update_E[shouldKickOffWall_G] = KickOffWallState_S,
            AlignPlacementState_S + Update_E[!alignDone_G] / alignPlacement_A =
                AlignPlacementState_S,
            AlignPlacementState_S + Update_E[alignDone_G]            = PlaceBallState_S,
            PlaceBallState_S + Update_E[!ballPlaced_G] / placeBall_A = PlaceBallState_S,
            PlaceBallState_S + Update_E[ballPlaced_G] / startWait_A  = WaitState_S,
            WaitState_S + Update_E[!waitDone_G]                      = WaitState_S,
            WaitState_S + Update_E[waitDone_G]                       = RetreatState_S,
            RetreatState_S + Update_E[retreatDone_G && ballPlaced_G] = X,
            RetreatState_S + Update_E[ballPlaced_G] / retreat_A      = RetreatState_S);
    }

   private:
    TbotsProto::AiConfig ai_config;
    std::shared_ptr<WallKickoffTactic> pivot_kick_tactic;
    std::shared_ptr<PlaceBallTactic> place_ball_tactic;
    std::shared_ptr<MoveTactic> align_placement_tactic;
    std::shared_ptr<MoveTactic> retreat_tactic;
    std::vector<std::shared_ptr<PlaceBallMoveTactic>> move_tactics;
    Point setup_point;
    std::chrono::time_point<std::chrono::steady_clock> start_time;
    constexpr static double const WALL_KICKOFF_VELOCITY_M_PER_S   = 3.0;
    constexpr static double const RETREAT_DISTANCE_METERS         = 0.6;
    constexpr static double const PLACEMENT_DIST_THRESHOLD_METERS = 0.15;
    constexpr static double const BALL_IS_PLACED_WAIT_S           = 3.0;
};
