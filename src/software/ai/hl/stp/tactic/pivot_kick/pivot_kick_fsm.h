#pragma once

#include "software/ai/hl/stp/tactic/dribble/dribble_fsm.h"
#include "software/ai/hl/stp/tactic/move/move_fsm.h"
#include "software/ai/hl/stp/tactic/tactic_base.hpp"
#include "software/geom/point.h"
#include "software/logger/logger.h"

/**
 * Finite State Machine class for Pivot Kick
 */
struct PivotKickFSM : TacticFSM<PivotKickFSM>
{
    using Update = TacticFSM<PivotKickFSM>::Update;
    class KickState;
    class StartState;

    struct ControlParams
    {
        // The location where the kick will be taken from
        Point kick_origin;
        // The direction the Robot will kick in
        Angle kick_direction;
        // How the robot will chip or kick the ball
        AutoChipOrKick auto_chip_or_kick;
    };

    /**
     * Constructor for PivotKickFSM
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit PivotKickFSM(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr);

    /**
     * Action that updates the DribbleFSM to get possession of the ball and pivot
     *
     * @param event PivotKickFSM::Update event
     * @param processEvent processes the GetBehindBallFSM::Update
     */
    void getPossessionAndPivot(
        const Update& event, boost::sml::back::process<DribbleFSM::Update> processEvent);

    /**
     * Action that kicks the ball
     *
     * @param event PivotKickFSM::Update event
     */
    void kickBall(const Update& event);

    /**
     * Guard that checks if the ball has been kicked
     *
     * @param event PivotKickFSM::Update event
     *
     * @return if the ball has been kicked
     */
    bool ballKicked(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        // clang-format off
        constexpr auto StartState_S        = boost::sml::state<StartState>;
        constexpr auto KickState_S         = boost::sml::state<KickState>;

        constexpr auto Update_E            = boost::sml::event<Update>;

        const auto ballKicked_G            = SMLGuard<&PivotKickFSM::ballKicked>{this};

        const auto kickBall_A              = SMLAction<&PivotKickFSM::kickBall>{this};

        constexpr auto DribbleFSM_S        = boost::sml::state<DribbleFSM>;
        const auto getPossessionAndPivot_A = SMLSubFSMUpdateAction<&PivotKickFSM::getPossessionAndPivot>{this};

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *StartState_S + Update_E                / getPossessionAndPivot_A   = DribbleFSM_S,
            DribbleFSM_S  + Update_E                / getPossessionAndPivot_A,
            DribbleFSM_S                                                        = KickState_S,
            KickState_S   + Update_E[!ballKicked_G] / kickBall_A,
            KickState_S   + Update_E[ballKicked_G]  / SET_STOP_PRIMITIVE_ACTION = X,
            X             + Update_E                / SET_STOP_PRIMITIVE_ACTION = X);
        // clang-format on
    }
};
