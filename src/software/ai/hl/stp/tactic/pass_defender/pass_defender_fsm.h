#pragma once

#include "shared/constants.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/logger/logger.h"

struct PassDefenderFSM
{
    class BlockPassState;
    class InterceptBallState;

    // This struct defines the unique control parameters that the PassDefenderFSM requires
    // in its update
    struct ControlParams
    {
        // The location on the field to block enemy passes from
        Point position_to_block_from;
    };

    DEFINE_TACTIC_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    // The minimum speed of the ball for it to be considered a pass
    static constexpr double MIN_PASS_SPEED = 0.5;

    // The maximum angle difference to determine if ball has been kicked in
    // the approximate direction of the defender
    static constexpr Angle MAX_PASS_ANGLE_DIFFERENCE = Angle::fromDegrees(30);

    // The minimum angle difference between a ball's trajectory and
    // pass_orientation for which we can consider a pass to be deflected
    static constexpr Angle MIN_DEFLECTION_ANGLE = Angle::fromDegrees(30);

    // Distance to chip the ball when trying to yeet it
    // TODO (#1878): Replace this with a more intelligent chip distance system
    static constexpr double YEET_CHIP_DISTANCE_METERS = 2.0;

    /**
     * Guard that checks if the ball has been kicked
     *
     * @param event PassDefenderFSM::Update event
     *
     * @return if the ball has been kicked
     */
    bool passStarted(const Update& event);

    /**
     * Guard that checks if the ball is deviating more than MIN_DEFLECTION_ANGLE
     * when the ball has been kicked and a pass is in progress.
     * Ball could be deflected due to another robot intercepting the pass,
     * or the pass defender could have chipped the ball away themselves.
     *
     * @param event PassDefenderFSM::Update event
     *
     * @return true if stray pass
     */
    bool ballDeflected(const Update& event);

    /**
     * Action to move to the specified location to block a potential pass
     *
     * @param event PassDefenderFSM::Update event
     */
    void blockPass(const Update& event);

    /**
     * Action to intercept an active pass and chip the ball away, adjusting
     * the defender's position to be directly in front of the ball's trajectory
     * for better interception.
     *
     * @param event PassDefenderFSM::Update event
     */
    void interceptBall(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(BlockPassState)
        DEFINE_SML_STATE(InterceptBallState)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_GUARD(passStarted)
        DEFINE_SML_GUARD(ballDeflected)

        DEFINE_SML_ACTION(blockPass)
        DEFINE_SML_ACTION(interceptBall)

        // otherwise
        //  ┌─────┐
        //  │  ┌──┴─────────────┐  passStarted?  ┌────────────────────┐
        //  └─►│ BlockPassState ├────────────────┤ InterceptBallState │
        //     └────────────────┘                └─────────┬──────────┘
        //              ▲                                  │
        //              └──────────────────────────────────┘
        //                        ballDeflected?
        //

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *BlockPassState_S + Update_E[passStarted_G] / interceptBall_A =
                InterceptBallState_S,
            BlockPassState_S + Update_E / blockPass_A,
            InterceptBallState_S + Update_E[ballDeflected_G] / blockPass_A =
                BlockPassState_S,
            InterceptBallState_S + Update_E / interceptBall_A,
            X + Update_E / SET_STOP_PRIMITIVE_ACTION = X);
    }

   private:
    Angle pass_orientation;
};
