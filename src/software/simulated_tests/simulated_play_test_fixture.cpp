#include "software/simulated_tests/simulated_play_test_fixture.h"

#include "software/gui/drawing/navigator.h"
#include "software/proto/message_translation/primitive_google_to_nanopb_converter.h"

SimulatedPlayTestFixture::SimulatedPlayTestFixture()
    : ai(DynamicParameters->getAiConfig(), DynamicParameters->getAiControlConfig())
{
}

void SimulatedPlayTestFixture::SetUp()
{
    SimulatedTestFixture::SetUp();
    ai = AI(DynamicParameters->getAiConfig(), DynamicParameters->getAiControlConfig());
}

void SimulatedPlayTestFixture::setFriendlyGoalie(RobotId goalie_id)
{
    MutableDynamicParameters->getMutableSensorFusionConfig()
        ->getMutableFriendlyGoalieId()
        ->setValue(static_cast<int>(goalie_id));
}

void SimulatedPlayTestFixture::setEnemyGoalie(RobotId goalie_id)
{
    MutableDynamicParameters->getMutableSensorFusionConfig()
        ->getMutableEnemyGoalieId()
        ->setValue(static_cast<int>(goalie_id));
}

void SimulatedPlayTestFixture::setAIPlay(const std::string& ai_play)
{
    MutableDynamicParameters->getMutableAiControlConfig()
        ->getMutableOverrideAiPlay()
        ->setValue(true);
    MutableDynamicParameters->getMutableAiControlConfig()
        ->getMutableCurrentAiPlay()
        ->setValue(ai_play);
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
