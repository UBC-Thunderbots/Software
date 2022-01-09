#pragma once

#include <include/boost/sml.hpp>

#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/play_fsm.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/logger/logger.h"

struct CreaseDefensePlayFSM
{
    struct ControlParams
    {
        // The origin point of the enemy threat
        Point enemy_threat_origin;
        // The maximum allowed speed mode
        MaxAllowedSpeedMode max_allowed_speed_mode;
    };

    DEFINE_PLAY_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Creates a crease defense play fsm
     *
     * @param play_config the play config for this play fsm
     */
    explicit CreaseDefensePlayFSM(std::shared_ptr<const PlayConfig> play_config)
        : play_config(play_config), crease_defenders({})
    {
    }

    /**
     * Action to defend the defense defense area
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

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(defendDefenseArea)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *X + Update_E / defendDefenseArea_A = X);
    }

   private:
    std::shared_ptr<const PlayConfig> play_config;
    std::vector<std::shared_ptr<CreaseDefenderTactic>> crease_defenders;
};
