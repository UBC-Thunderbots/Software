#include "software/simulated_tests/simulated_play_test_fixture.h"

#include "software/gui/drawing/navigator.h"
#include "software/proto/message_translation/primitive_google_to_nanopb_converter.h"
#include "software/test_util/test_util.h"

SimulatedPlayTestFixture::SimulatedPlayTestFixture()
    : ai_config(mutable_thunderbots_config->getMutableAiConfig()),
      ai_control_config(mutable_thunderbots_config->getMutableAiControlConfig()),
      sensor_fusion_config(mutable_thunderbots_config->getMutableSensorFusionConfig()),
      game_state(),
      ai(thunderbots_config->getAiConfig(), thunderbots_config->getAiControlConfig(),
         thunderbots_config->getPlayConfig())
{
}

void SimulatedPlayTestFixture::SetUp()
{
    SimulatedTestFixture::SetUp();

    ai_config            = mutable_thunderbots_config->getMutableAiConfig();
    ai_control_config    = mutable_thunderbots_config->getMutableAiControlConfig();
    sensor_fusion_config = mutable_thunderbots_config->getMutableSensorFusionConfig();

    ai = AI(thunderbots_config->getAiConfig(), thunderbots_config->getAiControlConfig(),
            thunderbots_config->getPlayConfig());
}

void SimulatedPlayTestFixture::setFriendlyGoalie(RobotId goalie_id)
{
    sensor_fusion_config->getMutableFriendlyGoalieId()->setValue(
        static_cast<int>(goalie_id));
}

void SimulatedPlayTestFixture::setEnemyGoalie(RobotId goalie_id)
{
    sensor_fusion_config->getMutableEnemyGoalieId()->setValue(
        static_cast<int>(goalie_id));
}

void SimulatedPlayTestFixture::setAIPlay(const std::string& ai_play)
{
    ai_control_config->getMutableOverrideAiPlay()->setValue(true);
    ai_control_config->getMutableCurrentAiPlay()->setValue(ai_play);
}

void SimulatedPlayTestFixture::setRefereeCommand(
    const RefereeCommand& current_referee_command,
    const RefereeCommand& previous_referee_command)
{
    game_state.updateRefereeCommand(previous_referee_command);
    game_state.updateRefereeCommand(current_referee_command);
}

void SimulatedPlayTestFixture::setGameState(const GameState& game_state_)
{
    game_state = game_state_;
}

void SimulatedPlayTestFixture::updatePrimitives(
    const World& world, std::shared_ptr<Simulator> simulator_to_update)
{
    auto world_with_updated_game_state = world;
    world_with_updated_game_state.updateGameState(game_state);

    auto start_tick_time = std::chrono::system_clock::now();

    auto primitive_set_msg = ai.getPrimitives(world_with_updated_game_state);
    double duration_ms     = ::TestUtil::millisecondsSince(start_tick_time);
    registerTickTime(duration_ms);
    simulator_to_update->setYellowRobotPrimitiveSet(
        createNanoPbPrimitiveSet(*primitive_set_msg));
}

std::optional<PlayInfo> SimulatedPlayTestFixture::getPlayInfo()
{
    return ai.getPlayInfo();
}

AIDrawFunction SimulatedPlayTestFixture::getDrawFunctions()
{       // TODO HACK (#2167) if we don't combine these draw functions
        // the visualizer flickers waaay too much. We need better and
        // more generic layering support
        auto planned_paths      = ai.getNavigator()->getPlannedPathPoints();
        auto obstacles          = ai.getNavigator()->getObstacles();
        auto circles_with_color = ai.getCirclesWithColorToDraw();
        auto draw_function      = [circles_with_color, planned_paths,
                              obstacles](QGraphicsScene* scene) {
            QPen path_pen(navigator_path_color);
            // The cap style must be NOT be set to SquareCap. It can be set to anything
            // else. Drawing a line of length 0 with the SquareCap style causes a large
            // line to be drawn
            path_pen.setCapStyle(Qt::PenCapStyle::RoundCap);
            path_pen.setWidth(2);
            path_pen.setCosmetic(true);
            path_pen.setStyle(Qt::DashLine);
            // Create a set a custom dash pattern. We do this because the default
            // patterns don't have enough space between the dashes so aren't easily
            // distinguishable as dashed lines.
            QVector<qreal> dashes;
            qreal space = 7;
            dashes << 2 << space << 2 << space;
            path_pen.setDashPattern(dashes);

            for (const auto& path : planned_paths)
            {
                for (size_t i = 1; i < path.size(); i++)
                {
                    Segment path_segment(path[i - 1], path[i]);
                    drawSegment(scene, path_segment, path_pen);
                }
            }

            QPen obstacle_pen(navigator_obstacle_color);
            // The cap style must be NOT be set to SquareCap. It can be set to anything
            // else. Drawing a line of length 0 with the SquareCap style causes a large
            // line to be drawn
            obstacle_pen.setCapStyle(Qt::PenCapStyle::RoundCap);
            obstacle_pen.setWidth(2);
            obstacle_pen.setCosmetic(true);

            ObstacleArtist obstacle_artist(scene, obstacle_pen);
            for (const auto& obstacle : obstacles)
            {
                obstacle->accept(obstacle_artist);
            }
            for (auto circle_with_color : circles_with_color)
            {
                // TODO (#2167) put this somwehere else
                if (circle_with_color.second == "pink")
                {
                    QPen pen(pink);
                    pen.setWidth(2);
                    pen.setCosmetic(true);
                    drawCircle(scene, circle_with_color.first, pen);
                }
                if (circle_with_color.second == "blue")
                {
                    QPen pen(blue);
                    pen.setWidth(2);
                    pen.setCosmetic(true);
                    drawCircle(scene, circle_with_color.first, pen);
                }
            }
        };
    return AIDrawFunction(draw_function);
}
