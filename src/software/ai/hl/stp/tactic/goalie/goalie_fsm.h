#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/hl/stp/tactic/chip/chip_fsm.h"
#include "software/ai/hl/stp/tactic/pivot_kick/pivot_kick_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/geom/algorithms/calculate_block_cone.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/convex_angle.h"
#include "software/geom/algorithms/intersection.h"
#include "software/geom/line.h"


struct GoalieFSM
{
   public:
    class Panic;
    class PositionToBlock;
    class MoveToGoalLine;

    struct ControlParams
    {
        bool should_move_to_goal_line;
    };

    DEFINE_TACTIC_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    // Distance to chip the ball when trying to yeet it
    // TODO (#1878): Replace this with a more intelligent chip distance system
    static constexpr double YEET_CHIP_DISTANCE_METERS = 2.0;

    /**
     * Constructor for GoalieFSM struct
     *
     * @param goalie_tactic_config The config to fetch parameters from
     * @param max_allowed_speed_mode The maximum allowed speed mode
     */
    explicit GoalieFSM(
        TbotsProto::GoalieTacticConfig goalie_tactic_config,
        TbotsProto::RobotNavigationObstacleConfig robot_navigation_obstacle_config,
        TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode)
        : goalie_tactic_config(goalie_tactic_config),
          robot_navigation_obstacle_config(robot_navigation_obstacle_config),
          max_allowed_speed_mode(max_allowed_speed_mode)
    {
    }

    /**
     * Gets the position for the goalie to move to, to best position itself between the
     * ball and the friendly goal
     * @param ball the ball to position the goalie relative to
     * @param field the field to position the goalie on
     * @param goalie_tactic_config the goalie tactic config
     *
     * @return the position that the goalie should move to
     */
    static Point getGoaliePositionToBlock(
        const Ball &ball, const Field &field,
        TbotsProto::GoalieTacticConfig goalie_tactic_config);

    /**
     * Gets intersections between the ball velocity ray and the full goal segment
     *
     * @param ball the ball to find interceptions with the full goal segment
     * @param field the field to find interceptions on
     *
     * @return the intersections between the ball velocity ray and the full goal segment
     */
    static std::vector<Point> getIntersectionsBetweenBallVelocityAndFullGoalSegment(
        const Ball &ball, const Field &field);

    /**
     * Gets the area within the friendly goalie's no-chip rectangle
     *
     * @return the area within the friendly goalie's no-chip rectangle
     */
    static Rectangle getNoChipRectangle(const Field &field);

    /**
     * Finds a good point to chip the ball to from its current position
     *
     * @param world the world
     * @param goalie_tactic_config the goalie tactic config
     *
     * @return a point on the field that is a good place to chip to
     */
    static Point findGoodChipTarget(
        const World &world, const TbotsProto::GoalieTacticConfig &goalie_tactic_config);

    /**
     * Guard that checks if the goalie should leave the crease the intercept the ball
     * when it is stuck in the dead zone right that exists right outside of the crease
     *
     * @param event
     * @return if the goalie should leave the crease
     */
    bool shouldEvacuateCrease(const Update &event);

    /**
     * Guard that checks if the ball is moving faster than the time_to_panic threshold
     * and has a clear path to the goal, if both are true then the goalie should panic
     * and move to block the ball
     *
     * @param event GoalieFSM::Update
     *
     * @return if the goalie should panic
     */
    bool shouldPanic(const Update &event);

    bool shouldMoveToGoalLine(const Update &event);

    /**
     * Guard that checks if the ball is moving slower than the panic threshold and is
     * inside the defense area, if true then the goalie should dribble and chip the
     * ball
     *
     * @param event GoalieFSM::Update
     *
     * @return if the goalie should pivot chip the ball
     */
    bool shouldPivotChip(const Update &event);

    /**
     * Guard that checks if the ball is moving slower than the panic threshold
     * or has no intersections with the friendly goal, if true then the goalie
     * should stop panicking
     *
     * @param event GoalieFSM::Update
     *
     * @return if the goalie should stop panicking
     */
    bool panicDone(const Update &event);

    /**
     * Action that prompts the goalie to leave the crease momentarily to chip the ball away
     *
     * @param event
     */
    void evacuateCrease(const Update &event);

    /**
     * Action that updates the MovePrimitive to time_to_panic and stop the ball
     *
     * @param event GoalieFSM::Update event
     */
    void panic(const Update &event);

    /**
     * Move the robot to the goal line
     *
     * @param event GoalieFSM::Update event
     */
    void moveToGoalLine(const Update &event);

    /**
     * Action that updates the PivotKickFSM
     *
     * @param event GoalieFSM::Update event
     * @param processEvent processes the PivotKickFSM::Update
     */
    void updatePivotKick(const Update &event,
                         boost::sml::back::process<PivotKickFSM::Update> processEvent);

    /**
     * Action that updates the MovePrimitive to position the goalie in the best spot to
     * block shots.
     *
     * @param event GoalieFSM::Update event
     */
    void positionToBlock(const Update &event);

    /**
     * Checks if ball is in the friendly defense area
     *
     * @param event GoalieFSM::Update event
     */
    bool ballInInflatedDefenseArea(const Update &event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(Panic)
        DEFINE_SML_STATE(PivotKickFSM)
        DEFINE_SML_STATE(PositionToBlock)
        DEFINE_SML_STATE(MoveToGoalLine)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_GUARD(ballInInflatedDefenseArea)
        DEFINE_SML_GUARD(panicDone)
        DEFINE_SML_GUARD(shouldEvacuateCrease)
        DEFINE_SML_GUARD(shouldPivotChip)
        DEFINE_SML_GUARD(shouldPanic)
        DEFINE_SML_GUARD(shouldMoveToGoalLine)

        DEFINE_SML_ACTION(panic)
        DEFINE_SML_ACTION(positionToBlock)
        DEFINE_SML_ACTION(moveToGoalLine)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(updatePivotKick, PivotKickFSM)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *PositionToBlock_S + Update_E[shouldMoveToGoalLine_G] / moveToGoalLine_A =
                MoveToGoalLine_S,
            PositionToBlock_S + Update_E[shouldEvacuateCrease_G] / updatePivotKick_A = PivotKickFSM_S,
            PositionToBlock_S + Update_E[shouldPanic_G] / panic_A = Panic_S,
            PositionToBlock_S + Update_E[shouldPivotChip_G] / updatePivotKick_A =
                PivotKickFSM_S,
            PositionToBlock_S + Update_E / positionToBlock_A,
            Panic_S + Update_E[shouldMoveToGoalLine_G] / moveToGoalLine_A =
                MoveToGoalLine_S,
            Panic_S + Update_E[shouldPivotChip_G] / updatePivotKick_A = PivotKickFSM_S,
            Panic_S + Update_E[panicDone_G] / positionToBlock_A       = PositionToBlock_S,
            Panic_S + Update_E / panic_A,
            PivotKickFSM_S + Update_E[shouldMoveToGoalLine_G] / moveToGoalLine_A =
                MoveToGoalLine_S,
            PivotKickFSM_S + Update_E[ballInInflatedDefenseArea_G] / updatePivotKick_A,
            PivotKickFSM_S + Update_E[!ballInInflatedDefenseArea_G] / positionToBlock_A =
                PositionToBlock_S,
            MoveToGoalLine_S + Update_E[shouldMoveToGoalLine_G] / moveToGoalLine_A =
                MoveToGoalLine_S,
            MoveToGoalLine_S + Update_E[!shouldMoveToGoalLine_G] / positionToBlock_A =
                PositionToBlock_S,
            X + Update_E = X);
    }

   private:
    // the goalie tactic config
    TbotsProto::GoalieTacticConfig goalie_tactic_config;
    TbotsProto::RobotNavigationObstacleConfig robot_navigation_obstacle_config;
    // The maximum allowed speed mode
    TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode;
};
