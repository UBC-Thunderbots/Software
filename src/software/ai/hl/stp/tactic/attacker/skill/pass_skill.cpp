#include "software/ai/hl/stp/tactic/attacker/skill/pass_skill.h"

PassSkill::PassSkill(const TbotsProto::AiConfig& ai_config,
                     std::shared_ptr<Strategy> strategy, double initial_score)
    : Skill(ai_config, strategy, initial_score),
      fsm(AttackerFSM(ai_config.attacker_tactic_config())),
      control_params({.best_pass_so_far = std::nullopt,
                      .pass_committed   = false,
                      .shot             = std::nullopt,
                      .chip_target      = std::nullopt})
{
}

double PassSkill::calculateViability(const Robot& robot, const World& world)
{
    std::vector<OffenseSupportType> committed_offense_support =
        strategy_->getCommittedOffenseSupport();
    auto it =
        std::find(committed_offense_support.begin(), committed_offense_support.end(),
                  OffenseSupportType::PASS_RECEIVER);
    if (it == committed_offense_support.end())
    {
        return 0;
    }

    return score;
}

bool PassSkill::done() const
{
    return fsm.is(boost::sml::X);
}

void PassSkill::updatePrimitive(const TacticUpdate& tactic_update)
{
    Pass best_pass                  = strategy->getBestPass(robot);
    control_params.best_pass_so_far = best_pass;
    control_params.pass_committed   = true;

    fsm.process_event(AttackerFSM::Update(control_params, tactic_update));
}

static TGenericFactory<std::string, Skill, PassSkill, TbotsProto::AiConfig,
                       std::shared_ptr<Strategy>, double>
    factory;
