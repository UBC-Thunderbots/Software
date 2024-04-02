#pragma once

#include "software/ai/hl/stp/skill/keep_away/keep_away_skill_fsm.h"
#include "software/ai/hl/stp/skill/pivot_kick/pivot_kick_skill_fsm.h"
#include "software/ai/hl/stp/skill/skill_fsm.h"

struct PassSkillFSM
{
    struct ControlParams
    {
    };

    DEFINE_SKILL_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    bool foundPass(const Update& event);

    void findPass(const Update& event,
                  boost::sml::back::process<KeepAwaySkillFSM::Update> processEvent);

    void takePass(const Update& event,
                  boost::sml::back::process<PivotKickSkillFSM::Update> processEvent);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(KeepAwaySkillFSM)
        DEFINE_SML_STATE(PivotKickSkillFSM)
        DEFINE_SML_EVENT(Update)
        DEFINE_SML_GUARD(foundPass)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(findPass, KeepAwaySkillFSM)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(takePass, PivotKickSkillFSM)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *KeepAwaySkillFSM_S + Update_E[foundPass_G] / takePass_A = PivotKickSkillFSM_S, 
            KeepAwaySkillFSM_S + Update_E / findPass_A, 
            KeepAwaySkillFSM_S = X,
            PivotKickSkillFSM_S + Update_E / takePass_A, PivotKickSkillFSM_S = X,
            X + Update_E / SET_STOP_PRIMITIVE_ACTION = X);
    }

   private:
    std::optional<PassWithRating> best_pass_so_far_;
    Timestamp pass_optimization_start_time;
    Duration time_since_commit_stage_start;
    double min_pass_score_threshold_;
};
