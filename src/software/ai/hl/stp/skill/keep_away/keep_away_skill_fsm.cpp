#include "software/ai/hl/stp/skill/keep_away/keep_away_skill_fsm.h"

#include "software/ai/evaluation/keep_away.h"

bool KeepAwaySkillFSM::isPossessionThreatened(const Update& event)
{
    TbotsProto::AttackerTacticConfig attacker_tactic_config =
        event.common.strategy->getAiConfig().attacker_tactic_config();

    Circle about_to_steal_danger_zone(
        event.common.robot.position(),
        attacker_tactic_config.enemy_about_to_steal_ball_radius());

    auto enemy_robots = event.common.world.enemyTeam().getAllRobots();
    return std::any_of(enemy_robots.begin(), enemy_robots.end(), [&](const auto& enemy) {
        return contains(about_to_steal_danger_zone, enemy.position());
    });
}

void KeepAwaySkillFSM::keepAway(
    const Update& event, boost::sml::back::process<DribbleSkillFSM::Update> processEvent)
{
    Pass best_pass             = (*event.common.strategy)->getBestPass().pass;
    auto keepaway_dribble_dest = findKeepAwayTargetPoint(event.common.world, best_pass);

    const Team& enemy_team = event.common.world.enemyTeam();
    const Ball& ball       = event.common.world.ball();

    auto final_dribble_orientation = best_pass.passerOrientation();

    if (enemy_team.numRobots() > 0)
    {
        // there is a robot on the enemy team, face away from the nearest one
        auto nearest_enemy_robot =
            *enemy_team.getNearestRobot(event.common.robot.position());
        auto dribble_orientation_vec = ball.position() - nearest_enemy_robot.position();
        final_dribble_orientation    = dribble_orientation_vec.orientation();
    }

    DribbleSkillFSM::ControlParams control_params = {
        .dribble_destination       = keepaway_dribble_dest,
        .final_dribble_orientation = final_dribble_orientation,
        .allow_excessive_dribbling = false};

    processEvent(DribbleSkillFSM::Update(control_params, event.common));
}