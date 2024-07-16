#pragma once

#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/hl/stp/skill/dribble/dribble_skill_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/passing/pass.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/geom/ray.h"
#include "software/logger/logger.h"

struct ReceiverFSM
{
    /**
     * Constructor for ReceiverFSM
     *
     * @param strategy the Strategy shared by all of AI
     */
    explicit ReceiverFSM(std::shared_ptr<Strategy> strategy);

    class OneTouchShotState;
    class ReceiveAndDribbleState;
    class WaitingForPassState;

    struct ControlParams
    {
        // The point at which to receive the pass
        std::optional<Point> receiving_position = std::nullopt;

        // If set to false, we will only receive and dribble
        bool enable_one_touch_shot = true;
    };

    DEFINE_TACTIC_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    // The minimum speed required for ball to be considered moving
    static constexpr double BALL_MIN_MOVEMENT_SPEED_M_PER_SEC = 0.04;

    // The minimum angle between a ball's trajectory and the ball-receiver_point vector
    // for which we can consider a pass to be stray (i.e it won't make it to the receiver)
    static constexpr Angle MIN_STRAY_PASS_ANGLE = Angle::fromDegrees(60);

    // The minimum speed required for a pass to be considered stray
    static constexpr double MIN_STRAY_PASS_SPEED = 0.3;

    /**
     * Given a shot and the ball, figures out the angle the robot should be facing
     * to perform a one-touch shot.
     *
     * @param shot The shot to take
     * @param ball The ball on the field
     */
    static Angle getOneTouchShotDirection(const Ray& shot, const Ball& ball);

    /**
     * Figures out the location of the one-touch shot and orientation the robot should
     * face
     *
     * @param robot The robot performing the one-touch
     * @param ball The ball on the field
     * @param best_shot_target The point to shoot at
     */
    static Shot getOneTouchShotPositionAndOrientation(const Robot& robot,
                                                      const Ball& ball,
                                                      const Point& best_shot_target);

    /*
     * Finds a shot that is greater than min_open_angle_for_one_touch_deg and
     * respects max_deflection_for_one_touch_deg for the highest chance
     * of scoring with a one-touch shot. If neither of those are true, return a nullopt
     *
     * @param world The world to find a feasible shot on
     * @param assigned_robot The robot that will be performing the one-touch
     */
    std::optional<Shot> findFeasibleShot(const World& world, const Robot& assigned_robot);

    /**
     * Checks if a one touch shot is possible
     *
     * @param event ReceiverFSM::Update event
     * @return true if one-touch possible
     */
    bool onetouchPossible(const Update& event);

    /**
     * If we have a shot on net, then update the receiver fsm
     * to setup for a one-touch shot.
     *
     * NOTE: This must be used with the onetouch_possible guard,
     * which checks for one-touch feasibility.
     *
     * @param event ReceiverFSM::Update event
     */
    void updateOnetouch(const Update& event);

    /**
     * One-touch shot is not possible, just receive ball as cleanly as possible.
     *
     * @param event ReceiverFSM::Update event
     */
    void updateReceive(const Update& event);

    /**
     * Constantly adjust the receives position to be directly
     * infront of the ball for better reception. This is especially
     * useful for long passes where the ball might not end up
     * exactly at the pass.receiverPoint()
     *
     * @param event ReceiverFSM::Update event
     */
    void adjustReceive(const Update& event);

    /**
     * Retrieves the ball using DribbleSkillFSM.
     * The interception algorithm used by DribbleSkillFSM works well for balls
     * that are not moving or went astray from the intended pass.
     *
     * @param event ReceiverFSM::Update event
     * @param processEvent processes the DribbleSkillFSM::Update
     */
    void retrieveBall(const Update& event,
                      boost::sml::back::process<DribbleSkillFSM::Update> processEvent);

    /**
     * Guard that checks if the ball has been kicked
     *
     * @param event ReceiverFSM::Update event
     *
     * @return if the ball has been kicked
     */
    bool passStarted(const Update& event);

    /**
     * Check if the pass has been received by the robot executing this tactic
     *
     * @param event ReceiverFSM::Update event
     *
     * @return true if the ball is near the robot's mouth
     */
    bool passReceived(const Update& event);

    /**
     * Check if the pass has been received by any friendly robot other than
     * the robot executing this tactic
     *
     * @param event ReceiverFSM::Update event
     *
     * @return true if the ball is near the mouth of a friendly teammate
     */
    bool passReceivedByTeammate(const Update& event);

    /**
     * If the pass is in progress and is deviating more than MIN_STRAY_PASS_ANGLE,
     * return true so that the receiver can react and intercept the ball.
     *
     * @param event ReceiverFSM::Update event
     * @return true if stray pass
     */
    bool strayPass(const Update& event);

    /**
     * Guard that if the ball is moving slowly.
     *
     * @param event ReceiverFSM::Update event
     *
     * @return true if the ball is moving slowly
     */
    bool slowPass(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(WaitingForPassState)
        DEFINE_SML_STATE(ReceiveAndDribbleState)
        DEFINE_SML_STATE(OneTouchShotState)
        DEFINE_SML_STATE(DribbleSkillFSM)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_GUARD(onetouchPossible)
        DEFINE_SML_GUARD(passStarted)
        DEFINE_SML_GUARD(passReceived)
        DEFINE_SML_GUARD(passReceivedByTeammate)
        DEFINE_SML_GUARD(strayPass)
        DEFINE_SML_GUARD(slowPass)

        DEFINE_SML_ACTION(updateOnetouch)
        DEFINE_SML_ACTION(updateReceive)
        DEFINE_SML_ACTION(adjustReceive)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(retrieveBall, DribbleSkillFSM)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *WaitingForPassState_S + Update_E[!passStarted_G] / updateReceive_A,
            WaitingForPassState_S + Update_E[passStarted_G && onetouchPossible_G] /
                                        updateOnetouch_A = OneTouchShotState_S,
            WaitingForPassState_S + Update_E[passStarted_G && !onetouchPossible_G] /
                                        updateReceive_A = ReceiveAndDribbleState_S,

            ReceiveAndDribbleState_S + Update_E[passReceivedByTeammate_G] /
                                           updateReceive_A = WaitingForPassState_S,
            ReceiveAndDribbleState_S +
                Update_E[strayPass_G || slowPass_G] / retrieveBall_A = DribbleSkillFSM_S,
            ReceiveAndDribbleState_S + Update_E / adjustReceive_A,

            DribbleSkillFSM_S + Update_E[passReceivedByTeammate_G] / updateReceive_A =
                WaitingForPassState_S,
            DribbleSkillFSM_S + Update_E / retrieveBall_A,

            OneTouchShotState_S +
                Update_E[!passReceived_G && !strayPass_G] / updateOnetouch_A,
            OneTouchShotState_S + Update_E[!passReceived_G && strayPass_G] /
                                      adjustReceive_A = ReceiveAndDribbleState_S,
            OneTouchShotState_S + Update_E[passReceived_G] / updateOnetouch_A =
                WaitingForPassState_S,

            X + Update_E / SET_STOP_PRIMITIVE_ACTION = X);
    }

   private:
    std::shared_ptr<Strategy> strategy_;
};
