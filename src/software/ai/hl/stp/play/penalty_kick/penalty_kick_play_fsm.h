#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/play_fsm.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/penalty_kick/penalty_kick_tactic.h"
#include "software/ai/hl/stp/tactic/stop/halt_tactic.h"
#include "software/logger/logger.h"


struct PenaltyKickPlayFSM
{
    class SetupPositionState;
    class PerformKickState;

    struct ControlParams
    {
    };

    DEFINE_PLAY_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Creates a penalty kick play FSM
     *
     * @param ai_config the play config for this play FSM
     */
    explicit PenaltyKickPlayFSM(TbotsProto::AiConfig ai_config);

    /**
     * Action to set up the robots in position to start the penalty kick
     *
     * @param event the PenaltyKickPlayFSM Update event
     */
    void setupPosition(const Update& event);

    /**
     * Action to make the penalty kick taking robot perform the PenaltyKickTactic
     *
     * @param event the PenaltyKickPlayFSM Update event
     */
    void performKick(const Update& event);

    /**
     * Guard to check if robots are in position to start the penalty kick
     *
     * @param event the PenaltyKickPlayFSM Update event
     *
     * @return whether the robots have gotten into position to start the penalty kick
     */
    bool setupPositionDone(const Update& event);

    /**
     * Guard to check if the robot has performed the kick
     *
     * @param event the PenaltyKickPlayFSM Update event
     *
     * @return whether the robot has finished performing a kick
     */
    bool kickDone(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(SetupPositionState)
        DEFINE_SML_STATE(PerformKickState)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(setupPosition)
        DEFINE_SML_ACTION(performKick)

        DEFINE_SML_GUARD(setupPositionDone)
        DEFINE_SML_GUARD(kickDone)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *SetupPositionState_S + Update_E[!setupPositionDone_G] / setupPosition_A =
                SetupPositionState_S,
            SetupPositionState_S + Update_E[setupPositionDone_G] = PerformKickState_S,
            PerformKickState_S + Update_E[!kickDone_G] / performKick_A,
            PerformKickState_S + Update_E[kickDone_G] = X, X + Update_E = X);
    }

   private:
    TbotsProto::AiConfig ai_config;
    std::shared_ptr<PenaltyKickTactic> penalty_kick_tactic;
    std::vector<std::shared_ptr<PenaltySetupTactic>> penalty_setup_tactics;
};
