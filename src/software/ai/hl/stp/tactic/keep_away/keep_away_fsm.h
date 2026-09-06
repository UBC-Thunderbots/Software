#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/evaluation/keep_away.h"
#include "software/ai/hl/stp/tactic/dribble/dribble_fsm.h"

/**
 * Finite State Machine class for Keep Away
 */
struct KeepAwayFSM : TacticFSM<KeepAwayFSM>
{
    using Update = TacticFSM<KeepAwayFSM>::Update;
    struct ControlParams
    {
        std::optional<Pass> best_pass_so_far;
    };
    /**
     * Constructor for KeepAwayFSM
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit KeepAwayFSM(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr);

    /**
     * Action that updates the DribbleFSM to keep the ball away
     *
     * @param event KeepAwayFSM::Update event
     * @param processEvent processes the DribbleFSM::Update
     */
    void keepAway(const Update& event,
                  boost::sml::back::process<DribbleFSM::Update> processEvent);

    auto operator()()
    {
        using namespace boost::sml;
        constexpr auto Update_E     = boost::sml::event<Update>;
        constexpr auto DribbleFSM_S = boost::sml::state<DribbleFSM>;
        const auto keepAway_A       = SMLSubFSMUpdateAction<&KeepAwayFSM::keepAway>{this};

        return make_transition_table(*DribbleFSM_S + Update_E / keepAway_A,
                                     DribbleFSM_S                             = X,
                                     X + Update_E / SET_STOP_PRIMITIVE_ACTION = X);
    }
};
