#include "software/simulated_tests/simulated_test_fixture.h"

#include "software/gui/visualizer/drawing/navigator.h"
#include "software/logger/logger.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"

SimulatedTestFixture::SimulatedTestFixture()
    : simulator(std::make_unique<Simulator>(::TestUtil::createSSLDivBField())),
      ai(Util::DynamicParameters->getAIConfig(),
         Util::DynamicParameters->getAIControlConfig()),
      run_simulation_in_realtime(false)
{
}

void SimulatedTestFixture::SetUp()
{
    LoggerSingleton::initializeLogger();

    // TODO: Ideally we should reset all DynamicParameters for each test. However
    // because DynamicParameters are still partially global, this can't be done
    // until https://github.com/UBC-Thunderbots/Software/issues/1483 is complete

    // Re-create all objects for each test so we start from a clean setup
    // every time. Because the simulator is created initially in the constructor's
    // initialization list, and before every test in this SetUp function, we can
    // guarantee the pointer will never be null / empty
    simulator     = std::make_unique<Simulator>(::TestUtil::createSSLDivBField());
    ai            = AI(Util::DynamicParameters->getAIConfig(),
            Util::DynamicParameters->getAIControlConfig());
    sensor_fusion = SensorFusion();

    Util::MutableDynamicParameters->getMutableAIControlConfig()->mutableRunAI()->setValue(
        true);

    // The simulated test abstracts and maintains the invariant that the friendly team
    // is always the yellow team
    Util::MutableDynamicParameters->getMutableAIControlConfig()
        ->getMutableRefboxConfig()
        ->mutableOverrideRefboxDefendingSide()
        ->setValue(true);
    Util::MutableDynamicParameters->getMutableAIControlConfig()
        ->getMutableRefboxConfig()
        ->mutableDefendingPositiveSide()
        ->setValue(false);

    // The simulated test abstracts and maintains the invariant that the friendly team
    // is always defending the "negative" side of the field. This is so that the
    // coordinates given when setting up tests is from the perspective of the friendly
    // team
    Util::MutableDynamicParameters->getMutableAIControlConfig()
        ->getMutableRefboxConfig()
        ->mutableOverrideRefboxFriendlyTeamColor()
        ->setValue(true);
    Util::MutableDynamicParameters->getMutableAIControlConfig()
        ->getMutableRefboxConfig()
        ->mutableFriendlyColorYellow()
        ->setValue(true);
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
    Util::MutableDynamicParameters->getMutableAIControlConfig()
        ->getMutableRefboxConfig()
        ->mutableFriendlyGoalieId()
        ->setValue(static_cast<int>(goalie_id));
}

void SimulatedTestFixture::setEnemyGoalie(RobotId goalie_id)
{
    Util::MutableDynamicParameters->getMutableAIControlConfig()
        ->getMutableRefboxConfig()
        ->mutableEnemyGoalieId()
        ->setValue(static_cast<int>(goalie_id));
}

void SimulatedTestFixture::setPlay(const std::string &play_name)
{
    Util::MutableDynamicParameters->getMutableAIControlConfig()
        ->mutableOverrideAIPlay()
        ->setValue(true);
    Util::MutableDynamicParameters->getMutableAIControlConfig()
        ->mutableCurrentAIPlay()
        ->setValue(play_name);
}

void SimulatedTestFixture::enableVisualizer()
{
    // We mock empty argc and argv since we don't have access to them when running
    // tests These arguments do not matter for simply running the Visualizer
    char *argv[]               = {NULL};
    int argc                   = sizeof(argv) / sizeof(char *) - 1;
    visualizer                 = std::make_shared<VisualizerWrapper>(argc, argv);
    run_simulation_in_realtime = true;
}

bool SimulatedTestFixture::validateAndCheckCompletion(
    std::vector<FunctionValidator> &function_validators,
    std::vector<ContinuousFunctionValidator> &continuous_function_validators)
{
    for (auto &continuous_function_validator : continuous_function_validators)
    {
        continuous_function_validator.executeAndCheckForFailures();
    }

    bool validation_successful =
        std::all_of(function_validators.begin(), function_validators.end(),
                    [](FunctionValidator &fv) { return fv.executeAndCheckForSuccess(); });

    return function_validators.empty() ? false : validation_successful;
}

void SimulatedTestFixture::updateSensorFusion()
{
    auto ssl_wrapper_packet = simulator->getSSLWrapperPacket();
    assert(ssl_wrapper_packet);

    auto sensor_msg = SensorMsg();
    sensor_msg.set_allocated_ssl_vision_msg(ssl_wrapper_packet.release());

    sensor_fusion.updateWorld(sensor_msg);
}

void SimulatedTestFixture::sleep(const std::chrono::steady_clock::time_point &wall_start_time,
                                 const Timestamp &current_time)
{
    // How long to wait for the wall-clock time to match the
    // current simulation time
    auto wall_time_now = std::chrono::steady_clock::now();
    auto wall_time_since_sim_start =
        std::chrono::duration_cast<std::chrono::milliseconds>(wall_time_now -
                                                              wall_start_time);
    auto ms_to_sleep =
        std::chrono::milliseconds(static_cast<int>(current_time.getMilliseconds())) -
        wall_time_since_sim_start;
    if (ms_to_sleep > std::chrono::milliseconds(0))
    {
        std::this_thread::sleep_for(ms_to_sleep);
    }
}

void SimulatedTestFixture::runTest(
    const std::vector<ValidationFunction> &validation_functions,
    const std::vector<ValidationFunction> &continuous_validation_functions,
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

    for (const auto &validation_function : validation_functions)
    {
        function_validators.emplace_back(FunctionValidator(validation_function, world));
    }

    for (const auto &continuous_validation_function : continuous_validation_functions)
    {
        continuous_function_validators.emplace_back(
            ContinuousFunctionValidator(continuous_validation_function, world));
    }

    Timestamp timeout_time         = simulator->getTimestamp() + timeout;
    Duration time_step             = Duration::fromSeconds(1.0 / SIMULATED_CAMERA_FPS);
    auto wall_start_time           = std::chrono::steady_clock::now();
    bool validation_functions_done = false;
    while (simulator->getTimestamp() < timeout_time)
    {
        for (size_t i = 0; i < CAMERA_FRAMES_PER_AI_TICK; i++)
        {
            simulator->stepSimulation(time_step);
            updateSensorFusion();
        }

        if (auto world_opt = sensor_fusion.getWorld())
        {
            *world = world_opt.value();

            validation_functions_done =
                validateAndCheckCompletion(function_validators, continuous_function_validators);
            if (validation_functions_done)
            {
                break;
            }

            auto primitives = ai.getPrimitives(*world);
            auto primitives_ptr =
                std::make_shared<const std::vector<std::unique_ptr<Primitive>>>(
                    std::move(primitives));
            simulator->setYellowRobotPrimitives(primitives_ptr);

            if (visualizer)
            {
                visualizer->onValueReceived(*world);
                visualizer->onValueReceived(ai.getPlayInfo());
                visualizer->onValueReceived(drawNavigator(ai.getNavigator()));
            }
        }
        else
        {
            LOG(WARNING) << "SensorFusion did not output a valid World";
        }

        if (run_simulation_in_realtime)
        {
            sleep(wall_start_time, simulator->getTimestamp());
        }
    }

    if (!validation_functions_done && !validation_functions.empty())
    {
        ADD_FAILURE()
            << "Not all validation functions passed within the timeout duration";
    }
}
