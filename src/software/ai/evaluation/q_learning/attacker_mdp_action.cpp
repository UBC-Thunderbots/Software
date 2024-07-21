#include "software/ai/evaluation/q_learning/attacker_mdp_action.h"

std::unique_ptr<Skill> createSkillFromAttackerMdpAction(
    AttackerMdpAction action, std::shared_ptr<Strategy> strategy)
{
    switch (action)
    {
        case AttackerMdpAction::KEEP_AWAY:
            return std::make_unique<KeepAwaySkill>(strategy);
        case AttackerMdpAction::KICK_PASS:
            return std::make_unique<KickPassSkill>(strategy);
        case AttackerMdpAction::SHOOT:
            return std::make_unique<ShootSkill>(strategy);
        default:
            LOG(FATAL) << "AttackerMdpAction value " << action << " not handled";
    }
    __builtin_unreachable();
}
