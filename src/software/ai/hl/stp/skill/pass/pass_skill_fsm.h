#pragma once

#include "software/ai/hl/stp/skill/dribble/dribble_skill_fsm.h"
#include "software/ai/hl/stp/skill/kick/kick_skill_fsm.h"
#include "software/ai/hl/stp/skill/skill_fsm.h"

struct PassSkillFSM
{
    struct ControlParams
    {
    };

    DEFINE_SKILL_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    bool foundPass(const Update& event);

    void findPass(const Update& event,
                  boost::sml::back::process<DribbleSkillFSM::Update> processEvent);

    void takePass(const Update& event,
                  boost::sml::back::process<KickSkillFSM::Update> processEvent);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(DribbleSkillFSM)
        DEFINE_SML_STATE(KickSkillFSM)
        DEFINE_SML_EVENT(Update)
        DEFINE_SML_GUARD(foundPass)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(findPass, DribbleSkillFSM)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(takePass, KickSkillFSM)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *DribbleSkillFSM_S + Update_E[foundPass_G] / takePass_A = KickSkillFSM_S, 
            DribbleSkillFSM_S + Update_E / findPass_A, 
            DribbleSkillFSM_S = X,
            KickSkillFSM_S + Update_E / takePass_A, KickSkillFSM_S = X,
            X + Update_E / SET_STOP_PRIMITIVE_ACTION = X);
    }

   private:
    std::optional<PassWithRating> best_pass_so_far_;
    Timestamp pass_optimization_start_time;
    Duration time_since_commit_stage_start;
    double min_pass_score_threshold_;
    std::optional<Point> passer_point_; 
};
