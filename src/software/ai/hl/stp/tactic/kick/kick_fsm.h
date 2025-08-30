#pragma once

#include "software/ai/hl/stp/tactic/get_behind_ball/get_behind_ball_fsm.h"
#include "software/ai/hl/stp/tactic/tactic_base.hpp"
#include "software/geom/point.h"

/**
 * The control parameters for updating KickFSM
 */
struct KickFSMControlParams
{
    // The location where the kick will be taken from
    Point kick_origin;
    // The direction the Robot will kick in
    Angle kick_direction;
    // How fast the Robot will kick the ball in meters per second
    double kick_speed_meters_per_second;
};


struct KickFSM : TacticFSM<KickFSMControlParams>
{
   public:
    using Update = TacticFSM<KickFSMControlParams>::Update;
    class KickState;

    /**
     * Constructor for KickFSM
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit KickFSM(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
        : TacticFSM<KickFSMControlParams>(ai_config_ptr)
    {
    }

    /**
     * Action that updates the MovePrimitive
     *
     * @param event KickFSM::Update event
     */
    void updateKick(const Update &event);

    /**
     * Action that updates the GetBehindBallFSM
     *
     * @param event KickFSM::Update event
     * @param processEvent processes the GetBehindBallFSM::Update
     */
    void updateGetBehindBall(
        const Update &event,
        boost::sml::back::process<GetBehindBallFSM::Update> processEvent);

    /**
     * Guard that checks if the ball has been chicked
     *
     * @param event KickFSM::Update event
     *
     * @return if the ball has been chicked
     */
    bool ballChicked(const Update &event);

    /**
     * Guard that checks if the robot is aligned for the kick
     *
     * @param event KickFSM::Update event
     *
     * @return if the robot is aligned for the kick
     */
    bool shouldRealignWithBall(const Update &event);

    DEFINE_SML_GUARD_CLASS(ballChicked, KickFSM)
    DEFINE_SML_GUARD_CLASS(shouldRealignWithBall, KickFSM)

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(GetBehindBallFSM)
        DEFINE_SML_STATE(KickState)
        DEFINE_SML_EVENT(Update)

        DEFINE_SML_GUARD(ballChicked)
        DEFINE_SML_GUARD(shouldRealignWithBall)

        DEFINE_SML_ACTION(updateKick)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(updateGetBehindBall, GetBehindBallFSM)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *GetBehindBallFSM_S + Update_E / updateGetBehindBall_A,
            GetBehindBallFSM_S = KickState_S,

            KickState_S + Update_E[shouldRealignWithBall_G] / updateGetBehindBall_A =
                GetBehindBallFSM_S,
            KickState_S + Update_E[!ballChicked_G] / updateKick_A = KickState_S,
            KickState_S + Update_E[ballChicked_G] / SET_STOP_PRIMITIVE_ACTION = X,
            X + Update_E / SET_STOP_PRIMITIVE_ACTION                          = X);
    }
};
