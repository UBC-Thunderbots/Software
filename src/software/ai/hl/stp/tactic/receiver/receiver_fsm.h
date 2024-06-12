#pragma once

#include "shared/constants.h"
#include "software/ai/hl/stp/skill/dribble/dribble_skill_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/passing/pass.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/logger/logger.h"

struct ReceiverFSM
{
    /**
     * Constructor for AttackerFSM
     *
     * @param attacker_tactic_config The config to fetch parameters from
     */
    explicit ReceiverFSM(std::shared_ptr<Strategy> strategy) : strategy_(strategy) {}

    class ReceiveAndDribbleState;
    class WaitingForPassState;

    struct ControlParams
    {
        // The point at which to receive the pass
        std::optional<Point> receiver_point = std::nullopt;
    };

    DEFINE_TACTIC_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    static constexpr double MIN_PASS_START_SPEED    = 0.02;

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

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(ReceiveAndDribbleState)
        DEFINE_SML_STATE(WaitingForPassState)
        DEFINE_SML_EVENT(Update)

        DEFINE_SML_GUARD(passStarted)
        DEFINE_SML_GUARD(passReceived)
        DEFINE_SML_GUARD(passReceivedByTeammate)

        DEFINE_SML_ACTION(updateReceive)
        DEFINE_SML_ACTION(adjustReceive)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *WaitingForPassState_S + Update_E[!passStarted_G] / updateReceive_A,
            WaitingForPassState_S + Update_E / updateReceive_A = ReceiveAndDribbleState_S,
            ReceiveAndDribbleState_S + Update_E[passReceived_G] / adjustReceive_A = X,
            ReceiveAndDribbleState_S + Update_E[passReceivedByTeammate_G] /
                                           updateReceive_A = WaitingForPassState_S,
            ReceiveAndDribbleState_S + Update_E / adjustReceive_A,
            X + Update_E / SET_STOP_PRIMITIVE_ACTION = X);
    }

   private:
    std::shared_ptr<Strategy> strategy_;
};
