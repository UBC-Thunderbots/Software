#include "ai/hl/stp/play/kickoff_enemy_play.h"

#include "ai/hl/stp/evaluation/enemy_threat.h"
#include "ai/hl/stp/evaluation/possession.h"
#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/tactic/move_tactic.h"
#include "ai/hl/stp/tactic/shadow_kickoff_tactic.h"
#include "shared/constants.h"

const std::string KickoffEnemyPlay::name = "KickoffEnemy Play";

std::string KickoffEnemyPlay::getName() const
{
    return KickoffEnemyPlay::name;
}

bool KickoffEnemyPlay::isApplicable(const World &world) const
{
    return world.gameState().isTheirKickoff();
}

bool KickoffEnemyPlay::invariantHolds(const World &world) const
{
    return world.gameState().isTheirKickoff();
}

void KickoffEnemyPlay::getNextTactics(TacticCoroutine::push_type &yield)
{
    // TODO: This needs to be a goalie tactic
    auto lone_goalie_tactic_0 = std::make_shared<MoveTactic>(true);

    // these tactics will be assigned
    auto shadow_kickoff_tactic_1 =
        std::make_shared<ShadowKickoffTactic>(world.field(), true);
    auto shadow_kickoff_tactic_2 =
        std::make_shared<ShadowKickoffTactic>(world.field(), true);
    auto shadow_kickoff_tactic_3 =
        std::make_shared<ShadowKickoffTactic>(world.field(), true);

    // move tactic for robots to defend net
    auto move_tactic_4 = std::make_shared<MoveTactic>(true);
    auto move_tactic_5 = std::make_shared<MoveTactic>(true);

    do
    {
        // TODO: Replace placeholder tactic with goalie tactic
        lone_goalie_tactic_0->updateParams(Point(4, 0), Angle::half(), 0);

        // Assign a robot to deal with the enemy robot doing the immediate kicking
        auto robot = Evaluation::getRobotWithEffectiveBallPossession(
            world.enemyTeam(), world.ball(), world.field());

        if (robot)
        {
            shadow_kickoff_tactic_1->updateParams((*robot).position());
        }

        // the next two most threatening robots need to be blocked
        auto enemy_threats = Evaluation::getAllEnemyThreats(
            world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball());
        Evaluation::sortThreatsInDecreasingOrder(enemy_threats);

        // threat 0 is always the ball with position, threat 1 is always the goalie, so
        // threats 2 and 3 need to be shadowed during kickoff
        shadow_kickoff_tactic_2->updateParams(enemy_threats[2].robot.position());
        shadow_kickoff_tactic_3->updateParams(enemy_threats[3].robot.position());

        auto goal_post_pos = world.field().enemyGoalpostPos();
        auto goal_post_neg = world.field().enemyGoalpostNeg();
        auto goal_width    = world.field().goalWidth();

        // remaining robots can stay near the goalie, defending each post
        move_tactic_4->updateParams(
            Point(goal_post_pos.x() - goal_width, goal_post_pos.y()), Angle::half(), 0);

        move_tactic_5->updateParams(
            Point(goal_post_neg.x() - goal_width, goal_post_neg.y()), Angle::half(), 0);

        // yield the Tactics this Play wants to run, in order of priority
        yield({lone_goalie_tactic_0, shadow_kickoff_tactic_1, shadow_kickoff_tactic_2,
               shadow_kickoff_tactic_3, move_tactic_4, move_tactic_5});
    } while (true);
}

// Register this play in the PlayFactory
static TPlayFactory<KickoffEnemyPlay> factory;
