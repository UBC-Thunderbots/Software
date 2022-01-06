#pragma once

#include <include/boost/sml.hpp>

#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/crease_defense/crease_defense_play_fsm.h"
#include "software/ai/hl/stp/play/play_fsm.h"
#include "software/ai/hl/stp/play/shoot_or_pass/shoot_or_pass_play_fsm.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"

struct OffensePlayFSM
{
    class OffenseState;

    struct ControlParams
    {
    };

    DEFINE_PLAY_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS


    /**
     * Creates a offense play fsm
     *
     * @param play_config the play config for this play fsm
     */
    explicit OffensePlayFSM(std::shared_ptr<const PlayConfig> play_config);

    /**
     * Updates the offense play
     *
     * @param event the OffensePlayFSM event
     */
    void updateOffense(const Update& event);

    /**
     * Guard for whehter offense is done
     *
     * @param event the OffensePlayFSM event
     *
     * @return whether offense is done
     */
    bool doneOffense(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(OffenseState)
        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(updateOffense)
        DEFINE_SML_GUARD(doneOffense)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *OffenseState_S + Update_E[doneOffense_G] / updateOffense_A = X,
            OffenseState_S + Update_E[!doneOffense_G] / updateOffense_A = OffenseState_S);
    }

   private:
    std::shared_ptr<const PlayConfig> play_config;
    std::shared_ptr<FSM<ShootOrPassPlayFSM>> shoot_or_pass_play_fsm;
    std::shared_ptr<FSM<CreaseDefensePlayFSM>> crease_defense_play_fsm;
};
