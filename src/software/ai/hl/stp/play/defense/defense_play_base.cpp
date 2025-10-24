#include "defense_play_base.h"

#include "software/ai/evaluation/defender_assignment.h"
#include "software/ai/evaluation/enemy_threat.h"


DefensePlayFSMBase::DefensePlayFSMBase(
    std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : PlayFSM<DefensePlayFSMBase>(ai_config_ptr),
      crease_defenders({}),
      pass_defenders({}),
      shadowers({})
{
}

void DefensePlayFSMBase::setUpCreaseDefenders(int num_crease_defenders)
{
    if (num_crease_defenders == int(crease_defenders.size()))
    {
        return;
    }

    crease_defenders =
        std::vector<std::shared_ptr<CreaseDefenderTactic>>(num_crease_defenders);
    std::generate(crease_defenders.begin(), crease_defenders.end(),
                  [this]()
                  { return std::make_shared<CreaseDefenderTactic>(ai_config_ptr); });
}

void DefensePlayFSMBase::setUpPassDefenders(int num_pass_defenders)
{
    if (num_pass_defenders == int(pass_defenders.size()))
    {
        return;
    }

    pass_defenders = std::vector<std::shared_ptr<PassDefenderTactic>>(num_pass_defenders);
    std::generate(pass_defenders.begin(), pass_defenders.end(),
                  [this]()
                  { return std::make_shared<PassDefenderTactic>(ai_config_ptr); });
}

void DefensePlayFSMBase::updatePassDefenderControlParams(
    std::vector<DefenderAssignment>& pass_defender_assignments,
    TbotsProto::BallStealMode ball_steal_mode)
{
    for (unsigned int i = 0; i < pass_defenders.size(); i++)
    {
        pass_defenders.at(i)->updateControlParams(pass_defender_assignments.at(i).target,
                                                  ball_steal_mode);
    }
}

void DefensePlayFSMBase::setAlignment(
    const Update& event,
    const std::vector<DefenderAssignment>& crease_defender_assignments,
    TbotsProto::BallStealMode ball_steal_mode)
{
    for (unsigned int i = 0; i < crease_defenders.size(); i++)
    {
        auto target = crease_defender_assignments.at(i).target;

        // Determine the number of crease defenders already assigned to the target
        auto defenders_with_target_count = std::count_if(
            crease_defender_assignments.begin(), crease_defender_assignments.begin() + i,
            [&target](const auto& assignment) { return assignment.target == target; });

        // Pick alignment based on how many crease defenders are already assigned to the
        // target
        auto alignment = TbotsProto::CreaseDefenderAlignment::CENTRE;
        if (defenders_with_target_count == 1)
        {
            if (event.common.world_ptr->ball().position().y() > 0)
            {
                alignment = TbotsProto::CreaseDefenderAlignment::LEFT;
            }
            else
            {
                alignment = TbotsProto::CreaseDefenderAlignment::RIGHT;
            }
        }
        else if (defenders_with_target_count == 2)
        {
            if (event.common.world_ptr->ball().position().y() > 0)
            {
                alignment = TbotsProto::CreaseDefenderAlignment::RIGHT;
            }
            else
            {
                alignment = TbotsProto::CreaseDefenderAlignment::LEFT;
            }
        }

        crease_defenders.at(i)->updateControlParams(
            target, alignment, event.control_params.max_allowed_speed_mode,
            ball_steal_mode);
    }
}
