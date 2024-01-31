#include "software/ai/hl/stp/play/penalty_kick/penalty_kick_play_fsm.h"

PenaltyKickPlayFSM::PenaltyKickPlayFSM(TbotsProto::AiConfig ai_config)
    : ai_config(ai_config),
      penalty_kick_tactic(std::make_shared<PenaltyKickTactic>(ai_config)),
      penalty_setup_tactics(std::vector<std::shared_ptr<PenaltySetupTactic>>())
{
}

void PenaltyKickPlayFSM::performKick(const Update &event)
{
    PriorityTacticVector tactics_to_run = {{}};
    tactics_to_run[0].emplace_back(penalty_kick_tactic);
    event.common.set_tactics(tactics_to_run);
}

void PenaltyKickPlayFSM::setupPosition(const Update &event)
{
    PriorityTacticVector tactics_to_run = {{}};

    Vector behind_ball_direction = (event.common.world_ptr->ball().position() -
                                    event.common.world_ptr->field().enemyGoalpostPos())
                                       .normalize();
    Angle shoot_angle = (event.common.world_ptr->field().enemyGoalpostPos() -
                         event.common.world_ptr->ball().position())
                            .orientation();

    Point behind_ball = event.common.world_ptr->ball().position() +
                        behind_ball_direction.normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                                        BALL_MAX_RADIUS_METERS + 0.1);
    double ball_position_x = event.common.world_ptr->field().friendlyPenaltyMark().x();

    // Adjust number of tactics based on the number of robots available
    unsigned int num_tactics = event.common.num_tactics;
    if (num_tactics != penalty_setup_tactics.size())
    {
        penalty_setup_tactics =
            std::vector<std::shared_ptr<PenaltySetupTactic>>(num_tactics);
        std::generate(penalty_setup_tactics.begin(), penalty_setup_tactics.end(),
                      []() { return std::make_shared<PenaltySetupTactic>(); });
    }

    // Move all non-shooter robots behind the penalty
    for (unsigned int i = 0; i < penalty_setup_tactics.size() - 1; i++)
    {
        double y_offset = 8 *
                          ((double)i - ((double)penalty_setup_tactics.size() - 1) / 2.0) *
                          ROBOT_MAX_RADIUS_METERS;
        penalty_setup_tactics.at(i)->updateControlParams(
            Point(ball_position_x - 1.25, y_offset),
            event.common.world_ptr->field().enemyGoalCenter().toVector().orientation(),
            0);
    }

    // Move shooting robot behind the ball
    penalty_setup_tactics.back()->updateControlParams(behind_ball, shoot_angle, 0.0);

    tactics_to_run[0].insert(tactics_to_run[0].end(), penalty_setup_tactics.begin(),
                             penalty_setup_tactics.end());
    event.common.set_tactics(tactics_to_run);
}

bool PenaltyKickPlayFSM::setupPositionDone(const Update &event)
{
    return !event.common.world_ptr->gameState().isSetupState();
}

bool PenaltyKickPlayFSM::kickDone(const Update &event)
{
    return event.common.world_ptr->gameState().isStopped();
}
