#pragma once

#include "software/ai/hl/stp/tactic/tactic_base.hpp"

/**
 * Finite State Machine class for Halting
 */
struct HaltFSM : TacticFSM<HaltFSM>
{
    using Update = TacticFSM<HaltFSM>::Update;
    class StopState;

    struct ControlParams
    {
    };
    /**
     * Constructor for HaltFSM struct
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit HaltFSM(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr);

    /**
     * Action to set the StopPrimitive
     *
     * @param event HaltFSM::Update
     */
    void updateStop(const Update& event);

    /**
     * Guard if the halt is done
     *
     * @param event HaltFSM::Update
     *
     * @return if the robot has halted
     */
    bool stopDone(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        constexpr auto StopState_S = boost::sml::state<StopState>;
        constexpr auto Update_E    = boost::sml::event<Update>;
        const auto stopDone_G      = SMLGuard<&HaltFSM::stopDone>{this};
        const auto updateStop_A    = SMLAction<&HaltFSM::updateStop>{this};

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *StopState_S + Update_E[!stopDone_G] / updateStop_A = StopState_S,
            StopState_S + Update_E[stopDone_G] / updateStop_A   = X,
            X + Update_E[!stopDone_G] / updateStop_A            = StopState_S,
            X + Update_E[stopDone_G] / updateStop_A             = X);
    }
};
