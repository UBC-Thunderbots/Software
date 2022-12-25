#include "software/ai/hl/stp/play/penalty_kick_play_fsm.h"

PenaltyKickPlayFSM::PenaltyKickPlayFSM(TbotsProto::AiConfig ai_config):
        ai_config(ai_config),
        penalty_kick_tactic(std::make_shared<PenaltyKickTactic>(ai_config)),
        penalty_setup_tactics({}),
        away_from_kick_tactics({})
{
    setPositioningTactics();
}

void PenaltyKickPlayFSM::performKick(const Update &event) {
    PriorityTacticVector tactics_to_run = {{}};

    tactics_to_run[0].insert(tactics_to_run[0].end(), away_from_kick_tactics.begin(),
                             away_from_kick_tactics.end());
    tactics_to_run[0].emplace_back(penalty_kick_tactic);
    event.common.set_tactics(tactics_to_run);
}

void PenaltyKickPlayFSM::setupPosition(const Update &event) {
    PriorityTacticVector tactics_to_run = {{}};

    Vector behind_ball_direction =
            (event.common.world.ball().position() - event.common.world.field().enemyGoalpostPos()).normalize();
    Angle shoot_angle =
            (event.common.world.field().enemyGoalpostPos() - event.common.world.ball().position()).orientation();

    Point behind_ball = event.common.world.ball().position() + behind_ball_direction.normalize(
            DIST_TO_FRONT_OF_ROBOT_METERS +
            BALL_MAX_RADIUS_METERS + 0.1);
    double ball_position_x = event.common.world.field().friendlyPenaltyMark().x();

    // Move all non-shooter robots behind the penalty mark
    penalty_setup_tactics.at(0)->updateControlParams(
            Point(ball_position_x - 1.25, 0),
            event.common.world.field().enemyGoalCenter().toVector().orientation(), 0);
    penalty_setup_tactics.at(1)->updateControlParams(
            Point(ball_position_x - 1.25, 4 * ROBOT_MAX_RADIUS_METERS),
            event.common.world.field().enemyGoalCenter().toVector().orientation(), 0);
    penalty_setup_tactics.at(2)->updateControlParams(
            Point(ball_position_x - 1.25, -4 * ROBOT_MAX_RADIUS_METERS),
            event.common.world.field().enemyGoalCenter().toVector().orientation(), 0);
    penalty_setup_tactics.at(3)->updateControlParams(
            Point(ball_position_x - 1.25, 8 * ROBOT_MAX_RADIUS_METERS),
            event.common.world.field().enemyGoalCenter().toVector().orientation(), 0);

    penalty_setup_tactics.at(4)->updateControlParams(behind_ball, shoot_angle, 0.0);

    tactics_to_run[0].insert(tactics_to_run[0].end(), penalty_setup_tactics.begin(),
                             penalty_setup_tactics.end());
    event.common.set_tactics(tactics_to_run);
}

bool PenaltyKickPlayFSM::setupPositionDone(const Update &event) {
    return !event.common.world.gameState().isSetupState();
}

bool PenaltyKickPlayFSM::kickDone(const Update &event) {
    return event.common.world.gameState().isStopped();
}

void PenaltyKickPlayFSM::setPositioningTactics() {
    penalty_setup_tactics = std::vector<std::shared_ptr<PenaltySetupTactic>>(5);
    std::generate(penalty_setup_tactics.begin(), penalty_setup_tactics.end(), [this]() {
        return std::make_shared<PenaltySetupTactic>();
    });

    away_from_kick_tactics = std::vector<std::shared_ptr<StopTactic>>(4);
    std::generate(away_from_kick_tactics.begin(), away_from_kick_tactics.end(), [this]() {
        return std::make_shared<StopTactic>();
    });
}
