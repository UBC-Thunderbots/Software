#include "software/simulated_tests/simulated_play_test_fixture.h"

#include "software/gui/drawing/navigator.h"
#include "software/proto/message_translation/primitive_google_to_nanopb_converter.h"

SimulatedPlayTestFixture::SimulatedPlayTestFixture()
    : ai_config(mutable_thunderbots_config->getMutableAiConfig()),
      ai_control_config(mutable_thunderbots_config->getMutableAiControlConfig()),
      sensor_fusion_config(mutable_thunderbots_config->getMutableSensorFusionConfig()),
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

void SimulatedPlayTestFixture::updatePrimitives(
    const World& world, std::shared_ptr<Simulator> simulator_to_update)
{
    auto primitive_set_msg = ai.getPrimitives(world);
    simulator_to_update->setYellowRobotPrimitiveSet(
        createNanoPbPrimitiveSet(*primitive_set_msg));
}

std::optional<PlayInfo> SimulatedPlayTestFixture::getPlayInfo()
{
    return ai.getPlayInfo();
}

AIDrawFunction SimulatedPlayTestFixture::getDrawFunctions()
{
    return drawNavigator(ai.getNavigator());
}
