#pragma once

#include "software/ai/hl/stp/skill/dribble/dribble_skill_fsm.h"
#include "software/ai/hl/stp/skill/pivot_kick/pivot_kick_skill_fsm.h"
#include "software/ai/hl/stp/skill/skill_fsm.h"

struct PassSkillFSM
{
    struct ControlParams
    {
        // Whether the robot should chip (true) or kick (false) the ball to make the pass
        bool should_chip;
    };

    DEFINE_SKILL_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS
    DEFINE_SUSPENDED_STATE_AND_UPDATE_STRUCT

    /**
     * Guard that checks if the best pass so far is viable
     *
     * @param event the Update event
     *
     * @return true if the best pass so far is viable, false otherwise
     */
    bool passFound(const Update& event);

    /**
     * Guard that checks if the ball has been received by another friendly
     * robot, completing the pass
     *
     * @param event the SuspendedUpdate event
     *
     * @return true if the pass was received, false otherwise
     */
    bool passReceived(const SuspendedUpdate& event);

    /**
     * Guard that checks if the current pass should be aborted
     * (because the ball went astray from the target receiver point,
     * stopped moving, got intercepted, etc.)
     *
     * @param event the SuspendedUpdate event
     *
     * @return true if the pass should be aborted, false otherwise
     */
    bool shouldAbortPass(const SuspendedUpdate& event);

    /**
     * Action that updates the DribbleSkillFSM to get control of the ball,
     * while simultaneously updating the best pass so far
     *
     * @param event the Update event
     * @param processEvent processes the DribbleSkillFSM::Update event
     */
    void findPass(const Update& event,
                  boost::sml::back::process<DribbleSkillFSM::Update> processEvent);

    /**
     * Action that updates the PivotKickSkillFSM to take the committed pass
     *
     * @param event the Update event
     * @param processEvent processes the PivotKickSkillFSM::Update event
     */
    void takePass(const Update& event,
                  boost::sml::back::process<PivotKickSkillFSM::Update> processEvent);

    /**
     * Action that resets the SkillState
     *
     * @param event the SuspendedUpdate event
     */
    void resetSkillState(const SuspendedUpdate& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(DribbleSkillFSM)
        DEFINE_SML_STATE(PivotKickSkillFSM)
        DEFINE_SML_STATE(Suspended)
        DEFINE_SML_EVENT(Update)
        DEFINE_SML_EVENT(SuspendedUpdate)
        DEFINE_SML_GUARD(passFound)
        DEFINE_SML_GUARD(passReceived)
        DEFINE_SML_GUARD(shouldAbortPass)
        DEFINE_SML_ACTION(resetSkillState)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(findPass, DribbleSkillFSM)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(takePass, PivotKickSkillFSM)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *DribbleSkillFSM_S + Update_E[passFound_G] / takePass_A = PivotKickSkillFSM_S,
            DribbleSkillFSM_S + Update_E / findPass_A,

            PivotKickSkillFSM_S + Update_E / takePass_A,
            PivotKickSkillFSM_S = Suspended_S,

            Suspended_S + SuspendedUpdate_E[passReceived_G || shouldAbortPass_G] /
                              resetSkillState_A = X,
            Suspended_S + SuspendedUpdate_E     = Suspended_S,

            X + Update_E / SET_STOP_PRIMITIVE_ACTION = X);
    }

   private:
    std::optional<PassWithRating> best_pass_so_far_;
    std::optional<Timestamp> pass_optimization_start_time;
    Duration time_since_commit_stage_start;
    double min_pass_score_threshold_;
};
