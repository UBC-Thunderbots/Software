#include "software/ai/hl/stp/skill/keep_away/keep_away_skill_fsm.h"

#include "software/ai/evaluation/keep_away.h"
#include "software/geom/algorithms/contains.h"

bool KeepAwaySkillFSM::shouldKeepAway(const Update& event)
{
    const std::vector<Robot> enemy_robots =
        event.common.world_ptr->enemyTeam().getAllRobotsExceptGoalie();

    const Circle about_to_steal_danger_zone(event.common.robot.position(),
                                            event.common.strategy->getAiConfig()
                                                .attacker_tactic_config()
                                                .enemy_about_to_steal_ball_radius());
    const bool possession_threatened =
        std::any_of(enemy_robots.begin(), enemy_robots.end(), [&](const Robot& enemy) {
            return contains(about_to_steal_danger_zone, enemy.position());
        });

    return !event.control_params.terminate_if_unthreatened || possession_threatened;
}

void KeepAwaySkillFSM::keepAway(
    const Update& event, boost::sml::back::process<DribbleSkillFSM::Update> processEvent)
{
    Pass best_pass = event.common.strategy->getBestPass().pass;
    auto keepaway_dribble_dest =
        findKeepAwayTargetPoint(*event.common.world_ptr, best_pass,
                                event.common.strategy->getAiConfig().passing_config());

    const Team& enemy_team = event.common.world_ptr->enemyTeam();
    const Ball& ball       = event.common.world_ptr->ball();

    auto final_dribble_orientation = best_pass.passerOrientation();

    // If there is a robot on the enemy team close to us, face away from the nearest one
    auto nearest_enemy_robot = enemy_team.getNearestRobot(event.common.robot.position());
    if (nearest_enemy_robot.has_value() &&
        distance(ball.position(), nearest_enemy_robot->position()) <
            event.common.strategy->getAiConfig()
                .attacker_tactic_config()
                .enemy_about_to_steal_ball_radius())
    {
        auto dribble_orientation_vec = ball.position() - nearest_enemy_robot->position();
        final_dribble_orientation    = dribble_orientation_vec.orientation();
    }

    DribbleSkillFSM::ControlParams control_params = {
        .dribble_destination       = keepaway_dribble_dest,
        .final_dribble_orientation = final_dribble_orientation,
        .excessive_dribbling_mode  = TbotsProto::ExcessiveDribblingMode::NOT_ALLOWED};

    processEvent(DribbleSkillFSM::Update(control_params, event.common));
}
