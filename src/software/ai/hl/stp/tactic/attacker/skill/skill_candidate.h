#include "software/ai/evaluation/scoring/candidate.h"

template <typename TSkill>
class SkillCandidate
{
    using SkillCreatorFun = std::function<std::shared_ptr<TSkill>>();

    virtual void accept(SkillCandidateVisitor& skill_candidate_visitor,
                        const Robot& robot, const Strategy& strategy, const World& world);

    virtual std::shared_ptr<TSkill> createSkill(SkillCreatorFun creator);
}

template <typename TSkill>
void SkillCandidate::accept<TSkill>(SkillCandidateVisitor& skill_candidate_visitor)
{
    skill_candidate_visitor.visit<TSkill>(this);
}

template <typename TSkill>
std::shared_ptr<TSkill> SkillCandidate::createSkill()
{
    return creator();
}
