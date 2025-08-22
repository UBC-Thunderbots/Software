#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/play_fsm.h"
#include "software/ai/hl/stp/tactic/goalie/goalie_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/logger/logger.h"

/**
 * Control Parameters for Penalty Kick Enemy Play
 */
struct PenaltyKickEnemyPlayControlParams
{
    // The goalie tactic common to all plays
    std::shared_ptr<GoalieTactic> goalie_tactic;
};

struct PenaltyKickEnemyPlayFSM : PlayFSM<PenaltyKickEnemyPlayControlParams>
{
    class SetupPositionState;
    class DefendKickState;

    /**
     * Creates a penalty kick enemy play FSM
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit PenaltyKickEnemyPlayFSM(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr);

    /**
     * Action to set up the robots in position to start the enemy penalty kick
     *
     * @param event the FSM event
     */
    void setupPosition(const Update& event);

    /**
     * Action to make the defending goalkeeper defend against the enemy penalty kick
     *
     * @param event the FSM event
     */
    void defendKick(const Update& event);

    /**
     * Guard to check if robots are in position to start the enemy penalty kick
     *
     * @param event the FSM event
     *
     * @return whether the robots have gotten into position to start the enemy penalty
     * kick
     */
    bool setupPositionDone(const Update& event);

    DEFINE_SML_GUARD_CLASS(setupPositionDone, PenaltyKickEnemyPlayFSM)

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(SetupPositionState)
        DEFINE_SML_STATE(DefendKickState)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_GUARD(setupPositionDone)

        DEFINE_SML_ACTION(setupPosition)
        DEFINE_SML_ACTION(defendKick)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *SetupPositionState_S + Update_E[!setupPositionDone_G] / setupPosition_A =
                SetupPositionState_S,
            SetupPositionState_S + Update_E[setupPositionDone_G] / defendKick_A =
                DefendKickState_S,
            DefendKickState_S + Update_E / defendKick_A, X + Update_E = X);
    }

   private:
    std::vector<std::shared_ptr<MoveTactic>> move_tactics;
};
