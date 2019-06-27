#include "ai/hl/stp/play/defense_play.h"

#include <ai/hl/stp/tactic/crease_defender_tactic.h>

#include "ai/hl/stp/evaluation/enemy_threat.h"
#include "ai/hl/stp/evaluation/possession.h"
#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/tactic/goalie_tactic.h"
#include "ai/hl/stp/tactic/move_tactic.h"
#include "ai/hl/stp/tactic/shadow_enemy_tactic.h"
#include "ai/hl/stp/tactic/stop_tactic.h"
#include "ai/world/game_state.h"
#include "shared/constants.h"
#include "util/parameter/dynamic_parameters.h"

const std::string DefensePlay::name = "Defense Play";

std::string DefensePlay::getName() const
{
    return DefensePlay::name;
}

bool DefensePlay::isApplicable(const World &world) const
{
    return world.gameState().isPlaying() &&
           Evaluation::teamHasPossession(world.enemyTeam(), world.ball());
}

bool DefensePlay::invariantHolds(const World &world) const
{
    return world.gameState().isPlaying() &&
           Evaluation::teamHasPossession(world.enemyTeam(), world.ball());
}

void DefensePlay::getNextTactics(TacticCoroutine::push_type &yield)
{
    auto goalie_tactic = std::make_shared<GoalieTactic>(
        world.ball(), world.field(), world.friendlyTeam(), world.enemyTeam());
    // TODO: Robot to try steal the ball from most threatening enemy
    std::vector<std::shared_ptr<ShadowEnemyTactic>> shadow_enemy_tactics = {
        std::make_shared<ShadowEnemyTactic>(world.field(), world.friendlyTeam(),
                                            world.enemyTeam(), true, true),
        std::make_shared<ShadowEnemyTactic>(world.field(), world.friendlyTeam(),
                                            world.enemyTeam(), true, true),
        std::make_shared<ShadowEnemyTactic>(world.field(), world.friendlyTeam(),
                                            world.enemyTeam(), true, true)};

    std::vector<std::shared_ptr<CreaseDefenderTactic>> crease_defender_tactics = {
        std::make_shared<CreaseDefenderTactic>(world.field(), world.ball(),
                                               world.friendlyTeam(), world.enemyTeam(),
                                               CreaseDefenderTactic::LeftOrRight::LEFT),
        std::make_shared<CreaseDefenderTactic>(world.field(), world.ball(),
                                               world.friendlyTeam(), world.enemyTeam(),
                                               CreaseDefenderTactic::LeftOrRight::RIGHT),
    };

    // TODO: replace with reasonable fallback tactics
    std::vector<std::shared_ptr<StopTactic>> stop_tactics = {
        std::make_shared<StopTactic>(false, true),
        std::make_shared<StopTactic>(false, true),
        std::make_shared<StopTactic>(false, true)};

    do
    {
        auto enemy_threats = Evaluation::getAllEnemyThreats(
            world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(), false);
        bool enemy_team_can_pass =
            Util::DynamicParameters::EnemyCapability::enemy_team_can_pass.value();

        // If we have any crease defenders, we don't want the goalie tactic to consider
        // them when deciding where to block
        Team friendly_team_for_goalie = world.friendlyTeam();
        for (auto crease_defender_tactic : crease_defender_tactics)
        {
            if (crease_defender_tactic->getAssignedRobot())
            {
                friendly_team_for_goalie.removeRobotWithId(
                    crease_defender_tactic->getAssignedRobot()->id());
            }
        }
        goalie_tactic->updateParams(world.ball(), world.field(), friendly_team_for_goalie,
                                    world.enemyTeam());

        std::vector<std::shared_ptr<Tactic>> result = {goalie_tactic};

        // Update crease defenders
        for (auto crease_defender_tactic : crease_defender_tactics)
        {
            crease_defender_tactic->updateParams(world.ball(), world.field(),
                                                 world.friendlyTeam(), world.enemyTeam());
            result.emplace_back(crease_defender_tactic);
        }

        // Assign ShadowEnemy tactics until we have every enemy covered. If there any
        // extra friendly robots, have them perform a reasonable default defensive tactic
        for (int i = 0; i < std::min(stop_tactics.size(), shadow_enemy_tactics.size());
             i++)
        {
            if (i < enemy_threats.size())
            {
                shadow_enemy_tactics.at(i)->updateParams(
                    enemy_threats.at(i), world.field(), world.friendlyTeam(),
                    world.enemyTeam(), ROBOT_MAX_RADIUS_METERS * 3, enemy_team_can_pass);
                result.emplace_back(shadow_enemy_tactics.at(i));
            }
            else
            {
                stop_tactics.at(i)->updateParams();
                result.emplace_back(stop_tactics.at(i));
            }
        }

        // yield the Tactics this Play wants to run, in order of priority
        yield(result);
    } while (true);
}

// Register this play in the PlayFactory
static TPlayFactory<DefensePlay> factory;
