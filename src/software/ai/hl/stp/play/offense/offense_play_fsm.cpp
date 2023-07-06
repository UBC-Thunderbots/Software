#include "software/ai/hl/stp/play/offense/offense_play_fsm.h"

OffensePlayFSM::OffensePlayFSM(TbotsProto::AiConfig ai_config)
    : ai_config(ai_config),
      shoot_or_pass_play(std::make_shared<ShootOrPassPlay>(ai_config)),
      defense_play(std::make_shared<DefensePlay>(ai_config))
{
}

bool OffensePlayFSM::enemyHasPossession(const Update& event)
{
    TeamPossession possession = event.common.world.getTeamWithPossession();
    return (possession == TeamPossession::ENEMY_TEAM) ||
           (possession == TeamPossession::STAGNANT_ENEMY_TEAM);
}

void OffensePlayFSM::setupOffensiveStrategy(const Update& event)
{
    unsigned int num_shoot_or_pass, num_defenders;
    auto num_enemy_robots = event.common.world.enemyTeam().numRobots();

    if (event.common.num_tactics > num_enemy_robots)
    {
        // Always reduce number of defenders if we have more bots on the field than
        // the enemy team, so that we take advantage of the scoring opportunity
        // while the enemy team is down a robot
        num_defenders = 1;
    }
    else if (event.common.num_tactics <= 2)
    {
        // Set number of defenders to 0 so we don't get stuck defending and never
        // attempt to score
        num_defenders = 0;
    }
    else
    {
        num_defenders = 2;
    }

    if (overbalancedFriendlyRobotsInFriendlyHalf(event.common.world) && num_defenders > 0)
    {
        num_defenders -= 1;
    }

    if (fewEnemyThreatsInFriendlyHalf(event.common.world) && num_defenders > 1)
    {
        num_defenders = 1;
    }

    num_shoot_or_pass = event.common.num_tactics - num_defenders;

    setTactics(event, num_shoot_or_pass, num_defenders, true);
}

bool OffensePlayFSM::overbalancedFriendlyRobotsInFriendlyHalf(const World& world)
{
    const std::vector<Robot> friendly_robots = world.friendlyTeam().getAllRobots();

    return (world.getTeamWithPossession() == TeamPossession::FRIENDLY_TEAM &&
            std::count_if(world.friendlyTeam().getAllRobots().begin(),
                          world.friendlyTeam().getAllRobots().end(),
                          [&](const Robot& robot) {
                              return robot.position().x() < ROBOT_MIN_X_THRESHOLD;
                          }) >= TOO_MANY_FRIENDLY_HALF_ROBOTS_THRESHOLD);
}

bool OffensePlayFSM::fewEnemyThreatsInFriendlyHalf(const World& world)
{
    return std::none_of(world.enemyTeam().getAllRobots().begin(),
                        world.enemyTeam().getAllRobots().end(),
                        [](const Robot& robot) { return robot.position().x() < 0; });
}

void OffensePlayFSM::setupDefensiveStrategy(const Update& event)
{
    setTactics(event, 0, event.common.num_tactics);
}

void OffensePlayFSM::setTactics(const Update& event, int num_shoot_or_pass,
                                int num_defenders, bool is_currently_in_possession)
{
    PriorityTacticVector tactics_to_return;

    if (num_shoot_or_pass > 0)
    {
        shoot_or_pass_play->updateTactics(PlayUpdate(
            event.common.world, num_shoot_or_pass,
            [&tactics_to_return](PriorityTacticVector new_tactics) {
                for (const auto& tactic_vector : new_tactics)
                {
                    tactics_to_return.push_back(tactic_vector);
                }
            },
            event.common.inter_play_communication,
            event.common.set_inter_play_communication_fun));
    }

    defense_play->updateControlParams(TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    if (num_defenders > 0)
    {
        defense_play->updateTactics(PlayUpdate(
            event.common.world, num_defenders,
            [&tactics_to_return](PriorityTacticVector new_tactics) {
                for (const auto& tactic_vector : new_tactics)
                {
                    tactics_to_return.push_back(tactic_vector);
                }
            },
            event.common.inter_play_communication,
            event.common.set_inter_play_communication_fun));
    }

    event.common.set_tactics(tactics_to_return);
}
