#include "software/simulated_tests/simulated_test_fixture.h"

#include "software/gui/drawing/navigator.h"
#include "software/logger/logger.h"
#include "software/proto/message_translation/primitive_google_to_nanopb_converter.h"
#include "software/proto/message_translation/tbots_protobuf.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"

SimulatedTestFixture::SimulatedTestFixture()
    : simulator(std::make_unique<Simulator>(Field::createSSLDivisionBField())),
      sensor_fusion(DynamicParameters->getSensorFusionConfig()),
      ai(DynamicParameters->getAIConfig(), DynamicParameters->getAIControlConfig()),
      run_simulation_in_realtime(false)
{
}

void SimulatedTestFixture::SetUp()
{
    LoggerSingleton::initializeLogger(
        DynamicParameters->getStandaloneSimulatorMainCommandLineArgs()
            ->LoggingDir()
            ->value());

    // init() resets all DynamicParameters for each test. Since DynamicParameters are
    // still partially global, we need to reinitialize simulator, sensor_fusion, and ai,
    // so that they can grab the new dynamic parameter pointers. Note that this is a bit
    // of hack because we're changing a global variable, but it can't be easily fixed
    // through dependency injection until
    // https://github.com/UBC-Thunderbots/Software/issues/1299
    // MutableDynamicParameters->init();
    simulator     = std::make_unique<Simulator>(Field::createSSLDivisionBField());
    sensor_fusion = SensorFusion(DynamicParameters->getSensorFusionConfig());
    ai = AI(DynamicParameters->getAIConfig(), DynamicParameters->getAIControlConfig());

    MutableDynamicParameters->getMutableAIControlConfig()->mutableRunAI()->setValue(true);

    // The simulated test abstracts and maintains the invariant that the friendly team
    // is always the yellow team
    MutableDynamicParameters->getMutableSensorFusionConfig()
        ->mutableOverrideGameControllerDefendingSide()
        ->setValue(true);
    MutableDynamicParameters->getMutableSensorFusionConfig()
        ->mutableDefendingPositiveSide()
        ->setValue(false);

    // The simulated test abstracts and maintains the invariant that the friendly team
    // is always defending the "negative" side of the field. This is so that the
    // coordinates given when setting up tests is from the perspective of the friendly
    // team
    MutableDynamicParameters->getMutableSensorFusionConfig()
        ->mutableFriendlyColorYellow()
        ->setValue(true);
    if (SimulatedTestFixture::enable_visualizer)
    {
        enableVisualizer();
    }
}

void SimulatedTestFixture::setBallState(const BallState &ball)
{
    simulator->setBallState(ball);
}

void SimulatedTestFixture::addFriendlyRobots(const std::vector<RobotStateWithId> &robots)
{
    simulator->addYellowRobots(robots);
}

void SimulatedTestFixture::addEnemyRobots(const std::vector<RobotStateWithId> &robots)
{
    simulator->addBlueRobots(robots);
}

Field SimulatedTestFixture::field() const
{
    return simulator->getField();
}

void SimulatedTestFixture::setFriendlyGoalie(RobotId goalie_id)
{
    MutableDynamicParameters->getMutableSensorFusionConfig()
        ->mutableFriendlyGoalieId()
        ->setValue(static_cast<int>(goalie_id));
}

void SimulatedTestFixture::setEnemyGoalie(RobotId goalie_id)
{
    MutableDynamicParameters->getMutableSensorFusionConfig()
        ->mutableEnemyGoalieId()
        ->setValue(static_cast<int>(goalie_id));
}

void SimulatedTestFixture::setAIPlay(const std::string &ai_play)
{
    MutableDynamicParameters->getMutableAIControlConfig()
        ->mutableOverrideAIPlay()
        ->setValue(true);
    MutableDynamicParameters->getMutableAIControlConfig()
        ->mutableCurrentAIPlay()
        ->setValue(ai_play);
}

void SimulatedTestFixture::setRefereeCommand(
    const RefereeCommand &current_referee_command,
    const RefereeCommand &previous_referee_command)
{
    MutableDynamicParameters->getMutableAIControlConfig()
        ->mutableOverrideRefereeCommand()
        ->setValue(true);
    MutableDynamicParameters->getMutableAIControlConfig()
        ->mutableCurrentRefereeCommand()
        ->setValue(toString(current_referee_command));
    MutableDynamicParameters->getMutableAIControlConfig()
        ->mutablePreviousRefereeCommand()
        ->setValue(toString(previous_referee_command));
}

void SimulatedTestFixture::enableVisualizer()
{
    full_system_gui            = std::make_shared<ThreadedFullSystemGUI>();
    run_simulation_in_realtime = true;
}

bool SimulatedTestFixture::validateAndCheckCompletion(
    std::vector<TerminatingFunctionValidator> &terminating_function_validators,
    std::vector<NonTerminatingFunctionValidator> &non_terminating_function_validators)
{
    for (auto &function_validator : non_terminating_function_validators)
    {
        function_validator.executeAndCheckForFailures();
    }

    bool validation_successful = std::all_of(
        terminating_function_validators.begin(), terminating_function_validators.end(),
        [](TerminatingFunctionValidator &fv) { return fv.executeAndCheckForSuccess(); });

    return terminating_function_validators.empty() ? false : validation_successful;
}

void SimulatedTestFixture::updateSensorFusion()
{
    auto ssl_wrapper_packet = simulator->getSSLWrapperPacket();
    assert(ssl_wrapper_packet);

    auto sensor_msg                        = SensorProto();
    *(sensor_msg.mutable_ssl_vision_msg()) = *ssl_wrapper_packet;

    sensor_fusion.processSensorProto(sensor_msg);
}

void SimulatedTestFixture::sleep(
    const std::chrono::steady_clock::time_point &wall_start_time,
    const Duration &desired_wall_tick_time)
{
    auto wall_time_now = std::chrono::steady_clock::now();
    auto current_tick_wall_time_duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(wall_time_now -
                                                              wall_start_time);
    auto ms_to_sleep = std::chrono::milliseconds(
                           static_cast<int>(desired_wall_tick_time.toMilliseconds())) -
                       current_tick_wall_time_duration;
    if (ms_to_sleep > std::chrono::milliseconds(0))
    {
        std::this_thread::sleep_for(ms_to_sleep);
    }
}

void SimulatedTestFixture::runTest(
    const std::vector<ValidationFunction> &terminating_validation_functions,
    const std::vector<ValidationFunction> &non_terminating_validation_functions,
    const Duration &timeout)
{
    updateSensorFusion();
    std::shared_ptr<World> world;
    if (auto world_opt = sensor_fusion.getWorld())
    {
        world = std::make_shared<World>(world_opt.value());
    }
    else
    {
        FAIL() << "Invalid initial world state";
    }

    for (const auto &validation_function : terminating_validation_functions)
    {
        terminating_function_validators.emplace_back(
            TerminatingFunctionValidator(validation_function, world));
    }

    for (const auto &validation_function : non_terminating_validation_functions)
    {
        non_terminating_function_validators.emplace_back(
            NonTerminatingFunctionValidator(validation_function, world));
    }

    const Timestamp timeout_time = simulator->getTimestamp() + timeout;
    const Duration simulation_time_step =
        Duration::fromSeconds(1.0 / SIMULATED_CAMERA_FPS);
    const Duration ai_time_step = Duration::fromSeconds(simulation_time_step.toSeconds() *
                                                        CAMERA_FRAMES_PER_AI_TICK);
    bool validation_functions_done = false;
    while (simulator->getTimestamp() < timeout_time)
    {
        if (!DynamicParameters->getAIControlConfig()->RunAI()->value())
        {
            continue;
        }
        auto wall_start_time = std::chrono::steady_clock::now();
        for (size_t i = 0; i < CAMERA_FRAMES_PER_AI_TICK; i++)
        {
            simulator->stepSimulation(simulation_time_step);
            updateSensorFusion();
        }

        if (auto world_opt = sensor_fusion.getWorld())
        {
            *world = world_opt.value();

            validation_functions_done = validateAndCheckCompletion(
                terminating_function_validators, non_terminating_function_validators);
            if (validation_functions_done)
            {
                break;
            }

            auto primitive_set_msg = ai.getPrimitives(*world);
            simulator->setYellowRobotPrimitiveSet(
                createNanoPbPrimitiveSet(*primitive_set_msg));

            if (run_simulation_in_realtime)
            {
                sleep(wall_start_time, ai_time_step);
            }

            if (full_system_gui)
            {
                full_system_gui->onValueReceived(*world);
                full_system_gui->onValueReceived(ai.getPlayInfo());
                full_system_gui->onValueReceived(drawNavigator(ai.getNavigator()));
            }
        }
        else
        {
            LOG(WARNING) << "SensorFusion did not output a valid World";
        }
    }

    if (!validation_functions_done && !terminating_validation_functions.empty())
    {
        ADD_FAILURE()
            << "Not all validation functions passed within the timeout duration";
    }
}
