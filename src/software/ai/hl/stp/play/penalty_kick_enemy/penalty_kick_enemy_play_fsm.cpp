#include "software/ai/hl/stp/play/penalty_kick_enemy/penalty_kick_enemy_play_fsm.h"

PenaltyKickEnemyPlayFSM::PenaltyKickEnemyPlayFSM(std::shared_ptr<Strategy> strategy)
    : strategy(strategy),
      move_skill_tactics(std::vector<std::shared_ptr<AssignedSkillTactic<MoveSkill>>>())
{
}

void PenaltyKickEnemyPlayFSM::setupPosition(const Update &event)
{
    PriorityTacticVector tactics_to_run = {{}};

    // Adjust number of tactics based on the number of robots available
    unsigned int num_tactics = event.common.num_tactics;
    if (num_tactics != move_skill_tactics.size())
    {
        move_skill_tactics =
            std::vector<std::shared_ptr<AssignedSkillTactic<MoveSkill>>>(num_tactics);
        std::generate(move_skill_tactics.begin(), move_skill_tactics.end(), [&]() {
            return std::make_shared<AssignedSkillTactic<MoveSkill>>(strategy);
        });
    }

    // Move all robots behind the penalty mark
    for (unsigned int i = 0; i < move_skill_tactics.size(); i++)
    {
        double y_offset = 8 *
                          ((double)i - ((double)move_skill_tactics.size() - 1) / 2.0) *
                          ROBOT_MAX_RADIUS_METERS;
        move_skill_tactics.at(i)->updateControlParams(
            {.destination = Point(
                 event.common.world_ptr->field().enemyPenaltyMark().x() + 1.75, y_offset),
             .final_orientation = event.common.world_ptr->field()
                                      .enemyGoalCenter()
                                      .toVector()
                                      .orientation(),
             .final_speed = 0});
    }

    // Move goalie to the goal line
    event.control_params.goalie_tactic->updateControlParams(true);

    tactics_to_run[0].insert(tactics_to_run[0].end(), move_skill_tactics.begin(),
                             move_skill_tactics.end());
    event.common.set_tactics(tactics_to_run);
}

void PenaltyKickEnemyPlayFSM::defendKick(const Update &event)
{
    // Allow goalie to move freely and defend against penalty kick
    event.control_params.goalie_tactic->updateControlParams(false);
}

bool PenaltyKickEnemyPlayFSM::setupPositionDone(const Update &event)
{
    return !event.common.world_ptr->gameState().isSetupState();
}
