#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/play_fsm.hpp"
#include "software/ai/hl/stp/tactic/halt/halt_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/penalty_kick/penalty_kick_tactic.h"
#include "software/logger/logger.h"

struct PenaltyKickPlayFSM : PlayFSM<PenaltyKickPlayFSM>
{
    /**
     * Control Parameters for PenaltyKickPlay
     */
    struct ControlParams
    {
    };

    class SetupPositionState;
    class PerformKickState;

    /**
     * Creates a penalty kick play FSM
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit PenaltyKickPlayFSM(
        std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr);

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

        // clang-format off
        constexpr auto SetupPositionState_S = boost::sml::state<SetupPositionState>;
        constexpr auto PerformKickState_S   = boost::sml::state<PerformKickState>;

        constexpr auto Update_E             = boost::sml::event<Update>;

        const auto setupPositionDone_G      = SMLGuard<&PenaltyKickPlayFSM::setupPositionDone>{this};
        const auto kickDone_G               = SMLGuard<&PenaltyKickPlayFSM::kickDone>{this};

        const auto setupPosition_A          = SMLAction<&PenaltyKickPlayFSM::setupPosition>{this};
        const auto performKick_A            = SMLAction<&PenaltyKickPlayFSM::performKick>{this};

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *SetupPositionState_S + Update_E[!setupPositionDone_G] / setupPosition_A = SetupPositionState_S,
            SetupPositionState_S  + Update_E[setupPositionDone_G]                    = PerformKickState_S,
            PerformKickState_S    + Update_E[!kickDone_G]          / performKick_A,
            PerformKickState_S    + Update_E[kickDone_G]                             = X,
            X                     + Update_E                                         = X);
        // clang-format on
    }

   private:
    std::shared_ptr<PenaltyKickTactic> penalty_kick_tactic;
    std::vector<std::shared_ptr<PenaltySetupTactic>> penalty_setup_tactics;
};
