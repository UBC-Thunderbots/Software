#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/play_fsm.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/pass_defender/pass_defender_tactic.h"
#include "software/logger/logger.h"

struct DefensePlayFSM
{
    class DefenseState;

    struct ControlParams
    {
        // The maximum allowed speed mode
        TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode;
    };

    DEFINE_PLAY_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    static constexpr unsigned int SHOOTING_LANE_MULTIPLIER = 3;

    /**
     * Creates a defense play FSM
     *
     * @param ai_config the play config for this play FSM
     */
    explicit DefensePlayFSM(TbotsProto::AiConfig ai_config);

    /**
     * Action to defend the defense area
     *
     * @param event the FSM event
     */
    void defend(const Update& event);

    void setUpCreaseDefenders(unsigned int num_crease_defenders);

    void setUpPassDefenders(unsigned int num_pass_defenders);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(DefenseState)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(defend)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *DefenseState_S + Update_E / defend_A = DefenseState_S,
            X + Update_E                          = X);
    }

   private:
    TbotsProto::AiConfig ai_config;
    std::vector<std::shared_ptr<CreaseDefenderTactic>> crease_defenders;
    std::vector<std::shared_ptr<PassDefenderTactic>> pass_defenders;
};
