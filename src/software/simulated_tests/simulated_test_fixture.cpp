#include "software/simulated_tests/simulated_test_fixture.h"

#include "software/logger/logger.h"
#include "software/test_util/test_util.h"

SimulatedTestFixture::SimulatedTestFixture()
    : mutable_thunderbots_config(std::make_shared<ThunderbotsConfig>()),
      thunderbots_config(
          std::const_pointer_cast<const ThunderbotsConfig>(mutable_thunderbots_config)),
      simulator(std::make_unique<Simulator>(Field::createSSLDivisionBField(),
                                            thunderbots_config->getSimulatorConfig())),
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

    // Experimentally determined restitution value
    mutable_thunderbots_config->getMutableSimulatorConfig()
        ->getMutableBallRestitution()
        ->setValue(0.8);
    // Measured these values from fig. 9 on page 8 of
    // https://ssl.robocup.org/wp-content/uploads/2020/03/2020_ETDP_ZJUNlict.pdf
    mutable_thunderbots_config->getMutableSimulatorConfig()
        ->getMutableSlidingFrictionAcceleration()
        ->setValue(6.9);
    mutable_thunderbots_config->getMutableSimulatorConfig()
        ->getMutableRollingFrictionAcceleration()
        ->setValue(0.5);

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

void SimulatedTestFixture::enableVisualizer()
{
    full_system_gui = std::make_shared<ThreadedFullSystemGUI>(mutable_thunderbots_config);
    run_simulation_in_realtime = true;
}

bool SimulatedTestFixture::validateAndCheckCompletion(
    std::vector<TerminatingFunctionValidator> &terminating_function_validators,
    std::vector<NonTerminatingFunctionValidator> &non_terminating_function_validators)
{
    for (auto &function_validator : non_terminating_function_validators)
    {
        auto error_message = function_validator.executeAndCheckForFailures();
        if (error_message)
        {
            ADD_FAILURE() << error_message.value();
        }
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

    auto start_tick_time = std::chrono::system_clock::now();

    // Tick one frame to aid with visualization
    bool validation_functions_done = tickTest(simulation_time_step, ai_time_step, world);

    // Logging duration of each tick
    unsigned int tick_count    = 1;
    double duration_ms         = ::TestUtil::millisecondsSince(start_tick_time);
    double total_tick_duration = duration_ms;
    double max_tick_duration   = duration_ms;
    double min_tick_duration   = duration_ms;

    while (simulator->getTimestamp() < timeout_time && !validation_functions_done)
    {
        if (!thunderbots_config->getAiControlConfig()->getRunAi()->value())
        {
            auto ms_to_sleep = std::chrono::milliseconds(
                static_cast<int>(ai_time_step.toMilliseconds()));
            std::this_thread::sleep_for(ms_to_sleep);
            continue;
        }

        // Record starting time
        start_tick_time = std::chrono::system_clock::now();

        validation_functions_done = tickTest(simulation_time_step, ai_time_step, world);

        // Calculate tick durations
        duration_ms = ::TestUtil::millisecondsSince(start_tick_time);
        total_tick_duration += duration_ms;
        max_tick_duration = std::max(max_tick_duration, duration_ms);
        min_tick_duration = std::min(min_tick_duration, duration_ms);
        tick_count++;
    }
    // Output the tick duration results
    double avg_tick_duration = total_tick_duration / tick_count;
    LOG(INFO) << "max tick duration: " << max_tick_duration << "ms" << std::endl;
    LOG(INFO) << "min tick duration: " << min_tick_duration << "ms" << std::endl;
    LOG(INFO) << "avg tick duration: " << avg_tick_duration << "ms" << std::endl;

    if (!validation_functions_done && !terminating_validation_functions.empty())
    {
        std::string failure_message =
            "Not all validation functions passed within the timeout duration:\n";
        for (const auto &fun : terminating_function_validators)
        {
            if (fun.currentErrorMessage() != "")
            {
                failure_message += fun.currentErrorMessage() + std::string("\n");
            }
        }
        ADD_FAILURE() << failure_message;
    }
}

bool SimulatedTestFixture::tickTest(Duration simulation_time_step, Duration ai_time_step,
                                    std::shared_ptr<World> world)
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
