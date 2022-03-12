#pragma once

#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/hl/stp/tactic/chip/chip_fsm.h"
#include "software/ai/hl/stp/tactic/pivot_kick/pivot_kick_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/move_intent.h"
#include "software/geom/algorithms/acute_angle.h"
#include "software/geom/algorithms/calculate_block_cone.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/intersection.h"
#include "software/geom/line.h"


struct GoalieFSM
{
   public:
    class PanicState;
    class PositionToBlockState;

    struct ControlParams
    {
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
    explicit GoalieFSM(std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config,
                       TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode)
        : goalie_tactic_config(goalie_tactic_config),
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
        std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config);

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
     * Guard that checks if the ball is moving faster than the time_to_panic threshold
     * and has a clear path to the goal, if both are true then the goalie should panic
     * and move to block the ball
     *
     * @param event GoalieFSM::Update
     *
     * @return if the goalie should panic
     */
    bool shouldPanic(const Update &event);

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
     * Action that updates the MoveIntent to time_to_panic and stop the ball
     *
     * @param event GoalieFSM::Update event
     */
    void updatePanic(const Update &event);

    /**
     * Action that updates the PivotKickFSM
     *
     * @param event GoalieFSM::Update event
     * @param processEvent processes the PivotKickFSM::Update
     */
    void updatePivotKick(const Update &event,
                         boost::sml::back::process<PivotKickFSM::Update> processEvent);

    /**
     * Action that updates the MoveIntent to position the goalie in the best spot to
     * block shots.
     *
     * @param event GoalieFSM::Update event
     */
    void updatePositionToBlock(const Update &event);

    /**
     * Checks if ball is in the friendly defense area
     *
     * @param event GoalieFSM::Update event
     */
    bool ballInDefenseArea(const Update &event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(PanicState)
        DEFINE_SML_STATE(PivotKickFSM)
        DEFINE_SML_STATE(PositionToBlockState)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_GUARD(ballInDefenseArea)
        DEFINE_SML_GUARD(panicDone)
        DEFINE_SML_GUARD(shouldPivotChip)
        DEFINE_SML_GUARD(shouldPanic)

        DEFINE_SML_ACTION(updatePanic)
        DEFINE_SML_ACTION(updatePositionToBlock)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(updatePivotKick, PivotKickFSM)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *PositionToBlockState_S + Update_E[shouldPanic_G] / updatePanic_A =
                PanicState_S,
            PositionToBlockState_S + Update_E[shouldPivotChip_G] / updatePivotKick_A =
                PivotKickFSM_S,
            PositionToBlockState_S + Update_E / updatePositionToBlock_A,
            PanicState_S + Update_E[shouldPivotChip_G] / updatePivotKick_A =
                PivotKickFSM_S,
            PanicState_S + Update_E[panicDone_G] = X,
            PanicState_S + Update_E / updatePanic_A,
            PivotKickFSM_S + Update_E[ballInDefenseArea_G] / updatePivotKick_A,
            PivotKickFSM_S + Update_E[!ballInDefenseArea_G] / updatePositionToBlock_A = X,
            X + Update_E / updatePositionToBlock_A = PositionToBlockState_S);
    }

   private:
    /*
     * Restrains the goalie to a rectangle, with the preferred point being the one
     * that intersects the point the goalie wants to move to and the center of the
     * goal
     *
     * @param field the field to restrain the goalie on
     * @param goalie_desired_position The point the goalie would like to go to
     * @param goalie_restricted_area The rectangle that the goalie is to stay in
     * @return goalie_suggested_position That the goalie should go to
     */
    // TODO: Refactor this function (#2045)
    static std::optional<Point> restrainGoalieInRectangle(
        const Field &field, Point goalie_desired_position,
        Rectangle goalie_restricted_area);

   private:
    // the goalie tactic config
    std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config;
    // The maximum allowed speed mode
    TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode;
};
