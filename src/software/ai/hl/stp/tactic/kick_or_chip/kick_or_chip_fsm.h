#pragma once

#include "software/ai/hl/stp/tactic/get_behind_ball/get_behind_ball_fsm.h"
#include "software/ai/hl/stp/tactic/tactic_base.hpp"
#include "software/geom/point.h"

/**
 * Finite State Machine class for Kicks
 */
struct KickOrChipFSM : TacticFSM<KickOrChipFSM>
{
    class KickOrChipState;

    struct ControlParams
    {
	// Control params for kick
        // The location where the kick will be taken from
        Point kick_or_chip_origin;
        // The direction the Robot will kick in
        Angle kick_or_chip_direction;

	//True if chipping, false is kicking.
	bool isChipping;
	
	// Unique for kick
        // How fast the Robot will kick the ball in meters per second
        double kick_speed_meters_per_second;

	// Unique for chip
        // The distance the robot will chip to
        double chip_distance_meters;
    };

    using Update = TacticFSM<KickOrChipFSM>::Update;

    /**
     * Constructor for KickFSM
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit KickOrChipFSM(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr);

    /**
     * Action that updates the MovePrimitive for kicking
     *
     * @param event KickOrChipFSM::Update event
     */
    void updateKick(const Update &event);


    /**
     * Action that updates the MovePrimitive for chippping
     *
     * @param event KickOrChipFSM::Update event
     */
    void updateChip(const Update &event);

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

    /**
     * Guard that checks if ball should be chipped
     *
     * @param event KickFSM::Update event
     *
     * @return if the ball should be chipped
     */
    bool isChipping(const Update &event);




    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(GetBehindBallFSM)
        DEFINE_SML_STATE(KickOrChipState)
        DEFINE_SML_EVENT(Update)

        DEFINE_SML_GUARD(ballChicked)
        DEFINE_SML_GUARD(shouldRealignWithBall)
	DEFINE_SML_GUARD(isChipping)

        DEFINE_SML_ACTION(updateKickOrChip)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(updateGetBehindBall, GetBehindBallFSM)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *GetBehindBallFSM_S + Update_E / updateGetBehindBall_A,
            GetBehindBallFSM_S = KickOrChipState_S,

            KickOrChipState_S + Update_E[shouldRealignWithBall_G] / updateGetBehindBall_A =
                GetBehindBallFSM_S,
            KickOrChipState_S + Update_E[!ballChicked_G && !isChipping] / updateKick_A = KickOrchipState_S,
            KickOrChipState_S + Update_E[!ballChicked_G && isChipping] / updateChip_A = KickOrChipState_S,
            KickOrChipState_S + Update_E[ballChicked_G] / SET_STOP_PRIMITIVE_ACTION = X,
            X + Update_E / SET_STOP_PRIMITIVE_ACTION                          = X);
    }
};
