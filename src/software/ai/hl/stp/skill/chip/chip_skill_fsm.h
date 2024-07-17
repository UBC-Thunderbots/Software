#pragma once

#include "software/ai/hl/stp/skill/get_behind_ball/get_behind_ball_skill_fsm.h"
#include "software/ai/hl/stp/skill/skill_fsm.h"

struct ChipSkillFSM
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

    DEFINE_SKILL_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Action that updates the MovePrimitive
     *
     * @param event ChipSkillFSM::Update event
     */
    void updateChip(const Update &event);

    /**
     * Action that updates the GetBehindBallSkillFSM
     *
     * @param event ChipSkillFSM::Update event
     * @param processEvent processes the GetBehindBallSkillFSM::Update
     */
    void updateGetBehindBall(
        const Update &event,
        boost::sml::back::process<GetBehindBallSkillFSM::Update> processEvent);

    /**
     * Guard that checks if the ball has been chicked
     *
     * @param event ChipSkillFSM::Update event
     *
     * @return if the ball has been chicked
     */
    bool ballChicked(const Update &event);

    /**
     * Guard that checks if the robot is aligned for the chip
     *
     * @param event ChipSkillFSM::Update event
     *
     * @return if the robot is aligned for the chip
     */
    bool shouldRealignWithBall(const Update &event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(GetBehindBallSkillFSM)
        DEFINE_SML_STATE(ChipState)
        DEFINE_SML_EVENT(Update)

        DEFINE_SML_GUARD(ballChicked)
        DEFINE_SML_GUARD(shouldRealignWithBall)
        DEFINE_SML_ACTION(updateChip)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(updateGetBehindBall, GetBehindBallSkillFSM)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *GetBehindBallSkillFSM_S + Update_E / updateGetBehindBall_A,
            GetBehindBallSkillFSM_S = ChipState_S,

            ChipState_S + Update_E[shouldRealignWithBall_G] / updateGetBehindBall_A =
                GetBehindBallSkillFSM_S,
            ChipState_S + Update_E[!ballChicked_G] / updateChip_A = ChipState_S,
            ChipState_S + Update_E[ballChicked_G] / SET_STOP_PRIMITIVE_ACTION = X,
            X + Update_E / SET_STOP_PRIMITIVE_ACTION                          = X);
    }
};
