#include "software/ai/hl/stp/play/defense/defense_play_fsm.h"

DefensePlayFSM::DefensePlayFSM(TbotsProto::AiConfig ai_config)
    : ai_config(ai_config), enemy_threats({}), crease_defenders({}), pass_defenders({}), shadowers({}), 
      defender_assignments_queue(), highest_cov_rating_assignment(std::nullopt)
{
}

bool DefensePlayFSM::shouldDefendAggressively(const Update& event)
{
    TeamPossession possession = event.common.world.getTeamWithPossession();
    return (possession == TeamPossession::STAGNANT_ENEMY_TEAM);
}

void DefensePlayFSM::blockShots(const Update& event)
{
    resetAssignmentsAndEnemyThreats(event);

    int num_tactics_left_to_assign = static_cast<int>(event.common.num_tactics);

    if (event.control_params.defender_assignments.size() > 0)
    {
        // Assign two robots to defend the most dangerous enemy threat
        num_tactics_left_to_assign -= assignPrimaryCreaseDefender(event, num_tactics_left_to_assign);
        num_tactics_left_to_assign -= assignPrimaryCreaseDefender(event, num_tactics_left_to_assign);
    }

    if (num_tactics_left_to_assign != static_cast<int>(event.common.num_tactics))
    {
        // Defenders were assigned to the assignment with the highest coverage rating,
        // so we no longer need to assign defenders to that assignment
        defender_assignments_queue.pop();
    }

    num_tactics_left_to_assign -= updateCreaseAndPassDefenders(event, num_tactics_left_to_assign);

    // Assign shadowers to left over tactics if we need to regain possession of the ball
    TeamPossession possession = event.common.world.getTeamWithPossession();
    if (possession != TeamPossession::FRIENDLY_TEAM)
    {
        num_tactics_left_to_assign -= updateShadowers(event, num_tactics_left_to_assign);
    }

    if (num_tactics_left_to_assign > 0)
    {
        LOG(WARNING) << "[DefensePlayFSM] Unable to assign " << num_tactics_left_to_assign << 
            " remaining tactics. Consider assigning fewer defensive robots"; 
    }

    setTactics(event);
}

void DefensePlayFSM::shadowAndBlockShots(const Update& event)
{
    LOG(DEBUG) << "[DefensePlayFSM] Aggressive defensive on";

    resetAssignmentsAndEnemyThreats(event);

    int num_tactics_left_to_assign = static_cast<int>(event.common.num_tactics);

    // Assign a robot to defend the most dangerous enemy threat
    num_tactics_left_to_assign -= assignPrimaryCreaseDefender(event, num_tactics_left_to_assign);


    if (num_tactics_left_to_assign != static_cast<int>(event.common.num_tactics) && num_tactics_to_assign > 0 && enemy_threats.size() > 0)
    {
        // Try to add a shadower targeting the most threatening enemy threat
        addShadower(enemy_threats.front());
        num_tactics_left_to_assign--;

        // Remove the most threatening enemy threat from the list so
        // that we don't assign more shadowers to defend against it
        enemy_threats.erase(enemy_threats.begin());
    }

    // Assign another robot to defend the most dangerous enemy threat
    num_tactics_left_to_assign -= assignPrimaryCreaseDefender(event, num_tactics_left_to_assign); 

    if (num_tactics_left_to_assign != static_cast<int>(event.common.num_tactics))
    {
        // Defenders were assigned to the assignment with the highest coverage rating,
        // so we no longer need to assign defenders to that assignment
        defender_assignments_queue.pop();
    }

    num_tactics_left_to_assign -= updateCreaseAndPassDefenders(event, num_tactics_left_to_assign);

    // Assign shadowers to left over tactics
    num_tactics_left_to_assign -= updateShadowers(event, num_tactics_left_to_assign);

    if (num_tactics_left_to_assign > 0)
    {
        LOG(WARNING) << "[DefensePlayFSM] Unable to assign " << num_tactics_left_to_assign << 
            " remaining tactics. Consider assigning fewer defensive robots";
    }

    setTactics(event);
}

void DefensePlayFSM::resetAssignmentsAndEnemyThreats(const Update& event)
{
    crease_defender_assignments = std::vector<DefenderAssignment>();
    pass_defender_assignments = std::vector<DefenderAssignment>();
    shadowers = std::vector<std::shared_ptr<ShadowEnemyTactic>>();

    highest_cov_rating_assignment = std::nullopt;
    if (event.control_params.defender_assignments.size() > 0)
    {
        highest_cov_rating_assignment = std::make_optional<DefenderAssignment>(
            event.control_params.defender_assignments.front());
    }

    defender_assignments_queue = std::queue(event.control_params.defender_assignments);

    LOG(DEBUG) << "[DefensePlayFSM] Found " << defender_assignments_queue.size() << " defender assignments";

    enemy_threats = getAllEnemyThreats(
        event.common.world.field(), event.common.world.friendlyTeam(),
        event.common.world.enemyTeam(), event.common.world.ball(), false);

    LOG(DEBUG) << "[DefensePlayFSM] Found " << enemy_threats.size() << " enemy threats";
}

int DefensePlayFSM::assignPrimaryCreaseDefender(const Update &event, int num_tactics_available)
{
    if (num_tactics_available > 0 && 
        highest_cov_rating_assignment.has_value() &&
        highest_cov_rating_assignment->type == CREASE_DEFENDER)
    {
        crease_defender_assignments.emplace_back(highest_cov_rating_assignment.value());
        return 1;
    }

    return 0;
}

int DefensePlayFSM::updateCreaseAndPassDefenders(
    const Update& event, const int num_tactics_to_assign)
{
    int num_tactics_assigned = num_tactics_to_assign;

    // Choose which defender assignments to assign defenders to based on number
    // of tactics available to set
    for (int i = 0; i < num_tactics_to_assign; i++)
    {
        std::unique_ptr<DefenderAssignment> defender_assignment;
        if (!defender_assignments_queue.empty())
        {
            defender_assignment = std::make_unique<DefenderAssignment>(defender_assignments_queue.front());
            defender_assignments_queue.pop();
        }
        else if (highest_cov_rating_assignment.has_value())
        {
            // If we have more tactics to set than determined defender assignments,
            // assign remaining defenders to the defender assignment with the
            // highest coverage rating
            defender_assignment = std::make_unique<DefenderAssignment>(highest_cov_rating_assignment.value());
        }
        else
        {
            LOG(WARNING) << "[DefensePlayFSM] Trying to assign too many robots to defend, and too few enemy threats to work with";
            num_tactics_assigned = i;
            break;
        }

        if (defender_assignment->type == CREASE_DEFENDER)
        {
            crease_defender_assignments.emplace_back(*defender_assignment);
        }
        else if (defender_assignment->type == PASS_DEFENDER)
        {
            pass_defender_assignments.emplace_back(*defender_assignment);
        }
        else if (defender_assignments_queue.empty())
        {
            num_tactics_assigned = i;
            break;
        }
    }

    // Reset tactics if the number of crease defenders or pass defenders
    // we intend to assign has changed
    setUpCreaseDefenders(static_cast<unsigned int>(crease_defender_assignments.size()));
    setUpPassDefenders(static_cast<unsigned int>(pass_defender_assignments.size()));

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
            if (event.common.world.ball().position().y() > 0)
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
            if (event.common.world.ball().position().y() > 0)
            {
                alignment = TbotsProto::CreaseDefenderAlignment::RIGHT;
            }
            else
            {
                alignment = TbotsProto::CreaseDefenderAlignment::LEFT;
            }
        }
        else if (defenders_with_target_count == 3)
        {
            alignment = TbotsProto::CreaseDefenderAlignment::FAR_LEFT;
        }
        else if (defenders_with_target_count == 4)
        {
            alignment = TbotsProto::CreaseDefenderAlignment::FAR_RIGHT;
        }

        crease_defenders.at(i)->updateControlParams(
            target, alignment, event.control_params.max_allowed_speed_mode);
    }

    for (unsigned int i = 0; i < pass_defenders.size(); i++)
    {
        pass_defenders.at(i)->updateControlParams(pass_defender_assignments.at(i).target);
    }

    return num_tactics_assigned;
}

int DefensePlayFSM::updateShadowers(const Update& event, const int num_shadowers_to_assign)
{
    int num_tactics_left_to_assign = num_shadowers_to_assign;
    int num_tactics_assigned = 0;

    for (int i = 0; (enemy_threats.size() > 0 && num_tactics_left_to_assign > 0); i++)
    {
        EnemyThreat enemy_threat = enemy_threats.at(i % static_cast<int>(enemy_threats.size()));
        addShadower(enemy_threat);
        
        num_tactics_left_to_assign--;
        num_tactics_assigned++;
    }

    return num_tactics_assigned;
}

void DefensePlayFSM::addShadower(EnemyThreat &enemy_threat)
{
    std::shared_ptr<ShadowEnemyTactic> shadower = std::make_shared<ShadowEnemyTactic>();
    shadower->updateControlParams(enemy_threat, ROBOT_SHADOWING_DISTANCE_METERS);
    shadowers.push_back(shadower);
}

void DefensePlayFSM::setUpCreaseDefenders(unsigned int num_crease_defenders)
{
    if (num_crease_defenders == crease_defenders.size())
    {
        return;
    }

    crease_defenders =
        std::vector<std::shared_ptr<CreaseDefenderTactic>>(num_crease_defenders);
    std::generate(crease_defenders.begin(), crease_defenders.end(), [this]() {
        return std::make_shared<CreaseDefenderTactic>(
            ai_config.robot_navigation_obstacle_config());
    });
}

void DefensePlayFSM::setUpPassDefenders(unsigned int num_pass_defenders)
{
    if (num_pass_defenders == pass_defenders.size())
    {
        return;
    }

    pass_defenders = std::vector<std::shared_ptr<PassDefenderTactic>>(num_pass_defenders);
    std::generate(pass_defenders.begin(), pass_defenders.end(),
                  [this]() { return std::make_shared<PassDefenderTactic>(); });
}

void DefensePlayFSM::setTactics(const Update& event)
{
    PriorityTacticVector tactics_to_return = {{}, {}, {}};

    tactics_to_return[0].insert(tactics_to_return[0].end(), crease_defenders.begin(),
                                crease_defenders.end());
    tactics_to_return[1].insert(tactics_to_return[1].end(), pass_defenders.begin(),
                                pass_defenders.end());
    tactics_to_return[2].insert(tactics_to_return[2].end(), shadowers.begin(),
                                shadowers.end());

    event.common.set_tactics(tactics_to_return);
}
