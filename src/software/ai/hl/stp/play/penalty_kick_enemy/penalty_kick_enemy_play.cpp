#include "software/ai/hl/stp/play/penalty_kick_enemy/penalty_kick_enemy_play.h"

#include "shared/constants.h"
#include "software/ai/hl/stp/tactic/goalie/goalie_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/util/generic_factory/generic_factory.h"

PenaltyKickEnemyPlay::PenaltyKickEnemyPlay(TbotsProto::AiConfig config)
    : Play(config, true)
{
}

void PenaltyKickEnemyPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                          const World &world)
{
    auto move_tactic_2 = std::make_shared<MoveTactic>();
    auto move_tactic_3 = std::make_shared<MoveTactic>();
    auto move_tactic_4 = std::make_shared<MoveTactic>();
    auto move_tactic_5 = std::make_shared<MoveTactic>();
    auto move_tactic_6 = std::make_shared<MoveTactic>();

    PriorityTacticVector tactics_to_run = {
        {move_tactic_2, move_tactic_3, move_tactic_4, move_tactic_5, move_tactic_6}};

    do
    {
        // Move all non-shooter robots behind the penalty mark
        move_tactic_2->updateControlParams(
            Point(world.field().enemyPenaltyMark().x() + 1.5, 0),
            world.field().enemyGoalCenter().toVector().orientation(), 0);
        move_tactic_3->updateControlParams(
            Point(world.field().enemyPenaltyMark().x() + 1.5,
                  4 * ROBOT_MAX_RADIUS_METERS),
            world.field().enemyGoalCenter().toVector().orientation(), 0);
        move_tactic_4->updateControlParams(
            Point(world.field().enemyPenaltyMark().x() + 1.5,
                  -4 * ROBOT_MAX_RADIUS_METERS),
            world.field().enemyGoalCenter().toVector().orientation(), 0);
        move_tactic_5->updateControlParams(
            Point(world.field().enemyPenaltyMark().x() + 1.5,
                  8 * ROBOT_MAX_RADIUS_METERS),
            world.field().enemyGoalCenter().toVector().orientation(), 0);
        move_tactic_6->updateControlParams(
            Point(world.field().enemyPenaltyMark().x() + 1.5,
                  -8 * ROBOT_MAX_RADIUS_METERS),
            world.field().enemyGoalCenter().toVector().orientation(), 0);

        if (world.gameState().isSetupState())
        {
            goalie_tactic->updateControlParams(true);
        }
        else
        {
            goalie_tactic->updateControlParams(false);
        }

        // yield the Tactics this Play wants to run, in order of priority
        yield(tactics_to_run);
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, PenaltyKickEnemyPlay, TbotsProto::AiConfig>
    factory;
