#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/play_fsm.h"
#include "software/ai/hl/stp/tactic/dribble/dribble_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/pivot_kick/pivot_kick_tactic.h"
#include "software/ai/hl/stp/tactic/stop/stop_tactic.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/geom/algorithms/closest_point.h"

using Zones = std::unordered_set<EighteenZoneId>;

struct BallPlacementPlayFSM
{
    double BACK_AWAY_FROM_CORNER_EXTRA_M = 0.9;
    double BACK_AWAY_FROM_WALL_M = ROBOT_MAX_RADIUS_METERS * 5.5;
    double MINIMUM_DISTANCE_FROM_WALL_FOR_ALIGN_METERS = ROBOT_MAX_RADIUS_METERS * 4.0;

    class StartState;
    class PickOffWallState;
    class AlignWallState;
    class AlignPlacementState;
    class PlaceBallState;
    class ReleaseBallState;
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
     * Action that has the robot align with the wall in order to pick the ball off of it.
     *
     * @param event the BallPlacementPlayFSM Update event
     */
    void alignWall(const Update& event);

    /**
     * The transition action into picking the ball off the wall, set the target
     * destination.
     *
     * @param event the BallPlacementPlayFSM Update event
     */
    void setPickOffDest(const Update& event);

    /**
     * Action that has the robot slowly pick up the ball and dribble it away the wall
     *
     * @param event the BallPlacementPlayFSM Update event
     */
    void pickOffWall(const Update& event);

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
     * Action that stops the robot and spins the dribbler out while waiting
     *
     * @param event the BallPlacementPlayFSM Update event
     */
    void releaseBall(const Update& event);

    /**
     * Action that has the placing robot retreat after placing the ball
     *
     * @param event the BallPlacementPlayFSM Update event
     */
    void retreat(const Update& event);

    /**
     * Guard on whether the ball is close enough to the wall that the robot cannot safely
     * fit behind it
     *
     * @param event the BallPlacementPlayFSM Update event
     *
     * @return whether the ball is close to the wall
     */
    bool shouldPickOffWall(const Update& event);

    /**
     * Guard on whether the robot is aligned with the ball and placement point
     *
     * @param event the BallPlacementPlayFSM Update event
     * @return whether the robot is in position to begin placing the ball
     */
    bool alignDone(const Update& event);

    /**
     * Guard on wether the robot is in position to pick the ball off the wall
     *
     * @param event the BallPlacementPlayFSM Update event
     * @return whether the robot is in position to begin picking the ball off the wall
     */
    bool wallAlignDone(const Update& event);

    /**
     * Guard on whether the placing robot has finished moving the ball into a better
     * position
     *
     * @param event the BallPlacementPlayFSM Update event
     *
     * @return whether the pick off has been performed
     */
    bool wallPickOffDone(const Update& event);

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
     * @return whether the robot has waited for 3 seconds
     */
    bool waitDone(const Update& event);

    /**
     * Guard on whether the robot has retreated outside of the required range
     * @param event the BallPlacementPlayFSM Update event
     * @return whether the robot has retreated outside of the required range
     */
    bool retreatDone(const Update& event);



    /**
     * Helper function for calculating the angle at which the robot must face towards to
     * pick up ball
     *
     * @param ball_pos the ball position to use when calculating the kick angle
     * @param field_lines the field lines of the playing area
     *
     * @return the kick angle
     */
    std::pair<Angle, Point> calculateWallPickOffLocation(const Point& ball_pos,
                                                         const Rectangle& field_lines,
                                                         double max_dist);

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
        DEFINE_SML_STATE(AlignWallState)
        DEFINE_SML_STATE(PickOffWallState)
        DEFINE_SML_STATE(AlignPlacementState)
        DEFINE_SML_STATE(PlaceBallState)
        DEFINE_SML_STATE(ReleaseBallState)
        DEFINE_SML_STATE(RetreatState)
        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(alignPlacement)
        DEFINE_SML_ACTION(placeBall)
        DEFINE_SML_ACTION(setPickOffDest)
        DEFINE_SML_ACTION(alignWall)
        DEFINE_SML_ACTION(pickOffWall)
        DEFINE_SML_ACTION(startWait)
        DEFINE_SML_ACTION(retreat)
        DEFINE_SML_ACTION(releaseBall)

        DEFINE_SML_GUARD(shouldPickOffWall)
        DEFINE_SML_GUARD(alignDone)
        DEFINE_SML_GUARD(wallAlignDone)
        DEFINE_SML_GUARD(wallPickOffDone)
        DEFINE_SML_GUARD(ballPlaced)
        DEFINE_SML_GUARD(waitDone)
        DEFINE_SML_GUARD(retreatDone)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *StartState_S + Update_E[!shouldPickOffWall_G] / alignPlacement_A =
                AlignPlacementState_S,
            StartState_S + Update_E[shouldPickOffWall_G] = AlignWallState_S,

            AlignWallState_S + Update_E[!wallAlignDone_G && shouldPickOffWall_G] /
                                   alignWall_A = AlignWallState_S,
            AlignWallState_S + Update_E[wallAlignDone_G] / setPickOffDest_A =
                PickOffWallState_S,
            AlignWallState_S + Update_E[!shouldPickOffWall_G] = AlignPlacementState_S,

            PickOffWallState_S + Update_E[!wallPickOffDone_G] / pickOffWall_A =
                PickOffWallState_S,
            PickOffWallState_S + Update_E[wallPickOffDone_G] / startWait_A =
                ReleaseBallState_S,

            AlignPlacementState_S + Update_E[shouldPickOffWall_G] = AlignWallState_S,
            AlignPlacementState_S + Update_E[!alignDone_G] / alignPlacement_A =
                AlignPlacementState_S,
            AlignPlacementState_S + Update_E[alignDone_G]            = PlaceBallState_S,

            PlaceBallState_S + Update_E[!ballPlaced_G] / placeBall_A = PlaceBallState_S,
            PlaceBallState_S + Update_E[ballPlaced_G] / startWait_A  = ReleaseBallState_S,

            ReleaseBallState_S + Update_E[!waitDone_G && ballPlaced_G] / releaseBall_A = ReleaseBallState_S,
            ReleaseBallState_S + Update_E[!ballPlaced_G]                        = StartState_S,
            ReleaseBallState_S + Update_E[waitDone_G]                           = RetreatState_S,

            RetreatState_S + Update_E[retreatDone_G && ballPlaced_G]     = X,
            RetreatState_S + Update_E[ballPlaced_G] / retreat_A          = RetreatState_S,
            RetreatState_S + Update_E[!ballPlaced_G]                     = StartState_S);
    }

   private:
    TbotsProto::AiConfig ai_config;
    std::shared_ptr<BallPlacementMoveTactic> align_wall_tactic;
    std::shared_ptr<BallPlacementDribbleTactic> pickoff_wall_tactic;
    std::shared_ptr<BallPlacementDribbleTactic> place_ball_tactic;
    std::shared_ptr<BallPlacementMoveTactic> align_placement_tactic;
    std::shared_ptr<MoveTactic> retreat_tactic;
    std::shared_ptr<MoveTactic> wait_tactic;
    std::vector<std::shared_ptr<BallPlacementMoveTactic>> move_tactics;
    Point setup_point;
    Point pickoff_point;
    Point pickoff_destination;
    Angle pickoff_final_orientation;
    Timestamp start_time;
    constexpr static double const WALL_KICKOFF_VELOCITY_M_PER_S   = 3.0;
    constexpr static double const RETREAT_DISTANCE_METERS         = 0.6;
    constexpr static double const PLACEMENT_DIST_THRESHOLD_METERS = 0.15;
    constexpr static double const BALL_IS_PLACED_WAIT_S           = 2.0;
};
