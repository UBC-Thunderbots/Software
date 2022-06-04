#pragma once

#include "software/ai/hl/stp/tactic/get_behind_ball/get_behind_ball_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"

struct ChipFSM
{
   public:
    class ChipState;

    struct ControlParams
    {
        // The location where the chip will be taken from
        Point chip_origin;
        // The direction the Robot will chip in
        Angle chip_direction;
        // The distance the robot will chip to
        double chip_distance_meters;
    };

    DEFINE_TACTIC_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

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


    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(GetBehindBallFSM)
        DEFINE_SML_STATE(ChipState)
        DEFINE_SML_EVENT(Update)

        DEFINE_SML_GUARD(ballChicked)
        DEFINE_SML_ACTION(updateChip)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(updateGetBehindBall, GetBehindBallFSM)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *GetBehindBallFSM_S + Update_E / updateGetBehindBall_A,
            GetBehindBallFSM_S                                    = ChipState_S,
            ChipState_S + Update_E[!ballChicked_G] / updateChip_A = ChipState_S,
            ChipState_S + Update_E[ballChicked_G] / SET_STOP_PRIMITIVE_ACTION = X,
            X + Update_E / SET_STOP_PRIMITIVE_ACTION                          = X);
    }
};
