#pragma once

#include "shared/constants.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/logger/logger.h"

struct PassDefenderFSM
{
   public:
    class BlockPassState;
    class InterceptBallState;

    // This struct defines the unique control parameters that the PassDefenderFSM requires
    // in its update
    struct ControlParams
    {
        Point position_to_block;
    };

    DEFINE_TACTIC_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    // Distance to chip the ball when trying to yeet it
    // TODO (#1878): Replace this with a more intelligent chip distance system
    static constexpr double YEET_CHIP_DISTANCE_METERS = 2.0;

    static constexpr double MIN_PASS_START_SPEED = 0.02;

    // The minimum angle between a ball's trajectory and the ball-receiver_point vector
    // for which we can consider a pass to be stray (i.e it won't make it to the receiver)
    static constexpr Angle MIN_STRAY_PASS_ANGLE = Angle::fromDegrees(60);

    // the minimum speed required for a pass to be considered stray
    static constexpr double MIN_STRAY_PASS_SPEED = 0.3;

    /**
     * Guard that checks if the ball has been kicked
     *
     * @param event PivotKickFSM::Update event
     *
     * @return if the ball has been kicked
     */
    bool passStarted(const Update& event);

    /**
     * If the pass is in progress and is deviating more than MIN_STRAY_PASS_ANGLE,
     * return true so that the receiver can react and intercept the ball.
     *
     * @param event ReceiverFSM::Update event
     * @return true if stray pass
     */
    bool strayPass(const Update& event);

    /**
     * Action to block the shot from our shadowee
     *
     *
     * @param event ShadowEnemyFSM::Update
     */
    void blockPass(const Update &event);

    /**
     * Action to intercept the ball
     *
     * Intercept a pass and chip the ball away
     *
     * @param event ShadowEnemyFSM::Update
     */
    void interceptBall(const Update &event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(BlockPassState)
        DEFINE_SML_STATE(InterceptBallState)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_GUARD(passStarted)
        DEFINE_SML_GUARD(strayPass)

        DEFINE_SML_ACTION(blockPass)
        DEFINE_SML_ACTION(interceptBall)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *BlockPassState_S + Update_E[passStarted_G] / interceptBall_A = InterceptBallState_S,
            BlockPassState_S + Update_E / blockPass_A,
            InterceptBallState_S + Update_E[strayPass_G] / blockPass_A = BlockPassState_S,
            InterceptBallState_S + Update_E / interceptBall_A,
            X + Update_E / SET_STOP_PRIMITIVE_ACTION = X);
    }
};
