#include "software/ai/hl/stp/play/enemy_ball_placement/enemy_ball_placement_play_fsm.h"

EnemyBallPlacementPlayFSM::EnemyBallPlacementPlayFSM(TbotsProto::AiConfig ai_config)
    : ai_config(ai_config),
      crease_defenders({std::make_shared<CreaseDefenderTactic>(
                            ai_config.robot_navigation_obstacle_config()),
                        std::make_shared<CreaseDefenderTactic>(
                            ai_config.robot_navigation_obstacle_config())}),
      move_tactics({
          std::make_shared<MoveTactic>(),
          std::make_shared<MoveTactic>(),
          std::make_shared<MoveTactic>(),
      }),
      distance_to_keep(ENEMY_BALL_PLACEMENT_DISTANCE_METERS + 0.2)
{
}

bool EnemyBallPlacementPlayFSM::hasPlacementPoint(const Update& event)
{
    return event.common.world.gameState().getBallPlacementPoint().has_value();
}

void EnemyBallPlacementPlayFSM::setPlacementPoint(const Update& event)
{
    placement_point = event.common.world.gameState().getBallPlacementPoint().value();
}

void EnemyBallPlacementPlayFSM::avoid(const Update& event)
{
    PriorityTacticVector tactics_to_run = {{}};

    World world = event.common.world;

    // Create crease defenders
    crease_defenders[0]->updateControlParams(placement_point,
                                             TbotsProto::CreaseDefenderAlignment::LEFT);
    crease_defenders[1]->updateControlParams(placement_point,
                                             TbotsProto::CreaseDefenderAlignment::RIGHT);

    tactics_to_run[0].emplace_back(crease_defenders[0]);
    tactics_to_run[0].emplace_back(crease_defenders[1]);

    // Create move tactics
    Vector positioning_vector = (world.ball().position() - placement_point);

    // If the ball is nearly placed, then adjust move tactics to position between
    // friendly goal and the ball
    if (positioning_vector.length() < 0.5)
    {
        positioning_vector = world.field().friendlyGoalCenter() - world.ball().position();
    }
    positioning_vector = positioning_vector.normalize() * distance_to_keep;

    Vector left_vector  = positioning_vector.rotate(Angle::fromDegrees(-30));
    Vector right_vector = positioning_vector.rotate(Angle::fromDegrees(30));

    Point center = world.ball().position() + positioning_vector;
    Point left   = world.ball().position() + left_vector;
    Point right  = world.ball().position() + right_vector;

    move_tactics[0]->updateControlParams(
        center, positioning_vector.orientation() + Angle::half(), 0);
    move_tactics[1]->updateControlParams(left, left_vector.orientation() + Angle::half(),
                                         0);
    move_tactics[2]->updateControlParams(right,
                                         right_vector.orientation() + Angle::half(), 0);
    tactics_to_run[0].emplace_back(move_tactics[0]);
    tactics_to_run[0].emplace_back(move_tactics[1]);
    tactics_to_run[0].emplace_back(move_tactics[2]);

    // Set tactics to run
    event.common.set_tactics(tactics_to_run);
}
