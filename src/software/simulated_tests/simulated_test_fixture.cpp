#include "software/simulated_tests/simulated_test_fixture.h"

#include "software/logger/logger.h"
#include "software/test_util/test_util.h"

SimulatedTestFixture::SimulatedTestFixture()
    : mutable_thunderbots_config(std::make_shared<ThunderbotsConfig>()),
      thunderbots_config(
          std::const_pointer_cast<const ThunderbotsConfig>(mutable_thunderbots_config)),
      simulator(std::make_unique<Simulator>(Field::createSSLDivisionBField(), thunderbots_config->getSimulatorConfig())),
      sensor_fusion(thunderbots_config->getSensorFusionConfig()),
      run_simulation_in_realtime(false)
{
}

void SimulatedTestFixture::SetUp()
{
    LoggerSingleton::initializeLogger(
        thunderbots_config->getStandaloneSimulatorMainCommandLineArgs()
            ->getLoggingDir()
            ->value());

    mutable_thunderbots_config = std::make_shared<ThunderbotsConfig>();
    thunderbots_config = std::const_pointer_cast<const ThunderbotsConfig>(mutable_thunderbots_config);

    simulator     = std::make_unique<Simulator>(Field::createSSLDivisionBField(), thunderbots_config->getSimulatorConfig());
    sensor_fusion = SensorFusion(thunderbots_config->getSensorFusionConfig());

    mutable_thunderbots_config->getMutableAiControlConfig()->getMutableRunAi()->setValue(
        !SimulatedTestFixture::stop_ai_on_start);

    // The simulated test abstracts and maintains the invariant that the friendly team
    // is always the yellow team
    mutable_thunderbots_config->getMutableSensorFusionConfig()
        ->getMutableOverrideGameControllerDefendingSide()
        ->setValue(true);
    mutable_thunderbots_config->getMutableSensorFusionConfig()
        ->getMutableDefendingPositiveSide()
        ->setValue(false);

    // The simulated test abstracts and maintains the invariant that the friendly team
    // is always defending the "negative" side of the field. This is so that the
    // coordinates given when setting up tests is from the perspective of the friendly
    // team
    mutable_thunderbots_config->getMutableSensorFusionConfig()
        ->getMutableFriendlyColorYellow()
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

void SimulatedTestFixture::setRefereeCommand(
    const RefereeCommand &current_referee_command,
    const RefereeCommand &previous_referee_command)
{
    mutable_thunderbots_config->getMutableAiControlConfig()
        ->getMutableOverrideRefereeCommand()
        ->setValue(true);
    mutable_thunderbots_config->getMutableAiControlConfig()
        ->getMutableCurrentRefereeCommand()
        ->setValue(toString(current_referee_command));
    mutable_thunderbots_config->getMutableAiControlConfig()
        ->getMutablePreviousRefereeCommand()
        ->setValue(toString(previous_referee_command));
}

void SimulatedTestFixture::enableVisualizer()
{
    full_system_gui            = std::make_shared<ThreadedFullSystemGUI>(mutable_thunderbots_config);
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

    // Tick one frame to aid with visualization
    bool validation_functions_done =
        tickTest(terminating_validation_functions, non_terminating_validation_functions,
                 simulation_time_step, ai_time_step, world);
    while (simulator->getTimestamp() < timeout_time)
    {
        if (!thunderbots_config->getAiControlConfig()->getRunAi()->value())
        {
            continue;
        }
        validation_functions_done = tickTest(terminating_validation_functions,
                                             non_terminating_validation_functions,
                                             simulation_time_step, ai_time_step, world);
    }

    if (!validation_functions_done && !terminating_validation_functions.empty())
    {
        ADD_FAILURE()
            << "Not all validation functions passed within the timeout duration";
    }
}

bool SimulatedTestFixture::tickTest(
    const std::vector<ValidationFunction> &terminating_validation_functions,
    const std::vector<ValidationFunction> &non_terminating_validation_functions,
    Duration simulation_time_step, Duration ai_time_step, std::shared_ptr<World> world)
{
    auto wall_start_time           = std::chrono::steady_clock::now();
    bool validation_functions_done = false;
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
            return validation_functions_done;
        }

        updatePrimitives(*world_opt, simulator);

        if (run_simulation_in_realtime)
        {
            sleep(wall_start_time, ai_time_step);
        }

        if (full_system_gui)
        {
            full_system_gui->onValueReceived(*world);
            if (auto play_info = getPlayInfo())
            {
                full_system_gui->onValueReceived(*play_info);
            }
            full_system_gui->onValueReceived(getDrawFunctions());
        }
    }
    else
    {
        LOG(WARNING) << "SensorFusion did not output a valid World";
    }
    return validation_functions_done;
}
