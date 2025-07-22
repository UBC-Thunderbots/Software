#pragma once

#include "software/ai/hl/stp/tactic/get_behind_ball/get_behind_ball_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"

struct ChipFSMControlParams
{
    // The location where the chip will be taken from
    Point chip_origin;
    // The direction the Robot will chip in
    Angle chip_direction;
    // The distance the robot will chip to
    double chip_distance_meters;
};

struct ChipFSM : TacticFSM<ChipFSMControlParams>
{
   public:
    using Update = TacticFSM<ChipFSMControlParams>::Update;

    class ChipState;

    /**
     * Constructor for ChipFSM
     *
     * @param ai_config_ptr Shared pointer to ai_config
     */
    explicit ChipFSM(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr): TacticFSM<ChipFSMControlParams>(ai_config_ptr) {}

    /**
     * Action that updates the MovePrimitive
     *
     * @param event ChipFSM::Update event
     */
    void updateChip(const Update &event);

    /**
     * Action that updates the GetBehindBallFSM
     *
     * @param event ChipFSM::Update event
     * @param processEvent processes the GetBehindBallFSM::Update
     */
    void updateGetBehindBall(
        const Update &event,
        boost::sml::back::process<GetBehindBallFSM::Update> processEvent);

    /**
     * Guard that checks if the ball has been chicked
     *
     * @param event ChipFSM::Update event
     *
     * @return if the ball has been chicked
     */
    bool ballChicked(const Update &event);

    /**
     * Guard that checks if the robot is aligned for the chip
     *
     * @param event KickFSM::Update event
     *
     * @return if the robot is aligned for the chip
     */
    bool shouldRealignWithBall(const Update &event);

    /**
     * Updates the params for this tactic that cannot be derived from the world
     *
     * @param chip_origin The location where the chip will be taken
     * @param chip_direction The direction the Robot will chip in
     * @param chip_distance_meters The distance between the starting location
     * of the chip and the location of the first bounce
     */
    void updateControlParams(const Point& chip_origin, const Angle& chip_direction,
                             double chip_distance_meters);

    /**
     * Updates the control parameters for this ChipTactic.
     *
     * @param chip_origin The location where the chip will be taken
     * @param chip_direction The direction the Robot will chip in
     */
    void updateControlParams(const Point& chip_origin, const Point& chip_target);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(GetBehindBallFSM)
        DEFINE_SML_STATE(ChipState)
        DEFINE_SML_EVENT(Update)

        DEFINE_SML_GUARD(ballChicked)
        DEFINE_SML_GUARD(shouldRealignWithBall)
        DEFINE_SML_ACTION(updateChip)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(updateGetBehindBall, GetBehindBallFSM)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *GetBehindBallFSM_S + Update_E / updateGetBehindBall_A,
            GetBehindBallFSM_S = ChipState_S,

            ChipState_S + Update_E[shouldRealignWithBall_G] / updateGetBehindBall_A =
                GetBehindBallFSM_S,
            ChipState_S + Update_E[!ballChicked_G] / updateChip_A = ChipState_S,
            ChipState_S + Update_E[ballChicked_G] / SET_STOP_PRIMITIVE_ACTION = X,
            X + Update_E / SET_STOP_PRIMITIVE_ACTION                          = X);
    }
    protected:
    // Tactic parameters
    ChipFSMControlParams control_params;
};
