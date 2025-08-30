#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/play_fsm.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/logger/logger.h"

/**
 * Control parameters for crease defense play
 */
struct CreaseDefensePlayControlParams
{
    // The origin point of the enemy threat
    Point enemy_threat_origin;
    // The maximum allowed speed mode
    TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode;
};

struct CreaseDefensePlayFSM : PlayFSM<CreaseDefensePlayControlParams>
{
    class DefenseState;

    /**
     * Creates a crease defense play FSM
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit CreaseDefensePlayFSM(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr);

    /**
     * Action to defend the defense area
     *
     * @param event the FSM event
     */
    void defendDefenseArea(const Update& event);

    /**
     * Action to set up the defenders taking into account the new number of defenders
     *
     * @param num_defenders The number of defenders
     */
    void setUpDefenders(unsigned int num_defenders);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(DefenseState)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(defendDefenseArea)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *DefenseState_S + Update_E / defendDefenseArea_A = DefenseState_S,
            X + Update_E                                     = X);
    }

   private:
    std::vector<std::shared_ptr<CreaseDefenderTactic>> crease_defenders;
};
