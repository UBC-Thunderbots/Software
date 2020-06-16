#include "software/simulated_tests/simulated_test_fixture.h"

#include "software/logger/logger.h"
#include "software/time/duration.h"
#include "software/test_util/test_util.h"

SimulatedTest::SimulatedTest() : simulator(std::make_unique<Simulator>(::TestUtil::createSSLDivBField())),
                                 ai(Util::DynamicParameters->getAIConfig(), Util::DynamicParameters->getAIControlConfig()), run_simulation_in_realtime(false)
{
}

void SimulatedTest::SetUp()
{
    LoggerSingleton::initializeLogger();

    // Re-create all objects for each test so we start from a clean setup
    // every time. Because the simulator is created initially in the constructor's
    // initialization list, and before every test in this SetUp function, we can
    // guarantee the pointer will never be null / empty
    simulator = std::make_unique<Simulator>(::TestUtil::createSSLDivBField());
    ai = AI(Util::DynamicParameters->getAIConfig(), Util::DynamicParameters->getAIControlConfig());
    sensor_fusion = SensorFusion();

    // The simulated test abstracts and maintains the invariant that the friendly team
    // is always the yellow team
    Util::MutableDynamicParameters->getMutableAIControlConfig()->getMutableRefboxConfig()->mutableOverrideRefboxDefendingSide()->setValue(true);
    Util::MutableDynamicParameters->getMutableAIControlConfig()->getMutableRefboxConfig()->mutableDefendingPositiveSide()->setValue(false);

    // The simulated test abstracts and maintains the invariant that the friendly team
    // is always defending the "negative" side of the field. This is so that the
    // coordinates given when setting up tests is from the perspective of the friendly team
    Util::MutableDynamicParameters->getMutableAIControlConfig()->getMutableRefboxConfig()->mutableOverrideRefboxFriendlyTeamColor()->setValue(true);
    Util::MutableDynamicParameters->getMutableAIControlConfig()->getMutableRefboxConfig()->mutableFriendlyColorYellow()->setValue(true);
}

void SimulatedTest::setBallState(const BallState &ball) {
    simulator->setBallState(ball);
}

void SimulatedTest::addFriendlyRobots(const std::vector<RobotStateWithId> &robots) {
    simulator->addYellowRobots(robots);
}

void SimulatedTest::addEnemyRobots(const std::vector<RobotStateWithId> &robots) {
    simulator->addBlueRobots(robots);
}

void SimulatedTest::setFriendlyGoalie(RobotId goalie_id) {
    Util::MutableDynamicParameters->getMutableAIControlConfig()->getMutableRefboxConfig()->mutableFriendlyGoalieId()->setValue(static_cast<int>(goalie_id));
}

void SimulatedTest::setEnemyGoalie(RobotId goalie_id) {
    Util::MutableDynamicParameters->getMutableAIControlConfig()->getMutableRefboxConfig()->mutableEnemyGoalieId()->setValue(static_cast<int>(goalie_id));
}

void SimulatedTest::setPlay(const std::string& play_name) {
    Util::MutableDynamicParameters->getMutableAIControlConfig()
            ->mutableOverrideAIPlay()
            ->setValue(true);
    Util::MutableDynamicParameters->getMutableAIControlConfig()
            ->mutableCurrentAIPlay()
            ->setValue(play_name);
}

void SimulatedTest::enableVisualizer()
{
    // We mock empty argc and argv since we don't have access to them when running
    // tests These arguments do not matter for simply running the Visualizer
    char *argv[] = {NULL};
    int argc     = sizeof(argv) / sizeof(char *) - 1;
//    visualizer   = std::make_shared<VisualizerWrapper>(argc, argv);
    run_simulation_in_realtime = true;
}

bool SimulatedTest::validateWorld(std::shared_ptr<World> world_ptr,
                                  std::vector<FunctionValidator> &function_validators,
                                  std::vector<ContinuousFunctionValidator> &continuous_function_validators) {
    for (auto &continuous_function_validator : continuous_function_validators)
    {
        continuous_function_validator.executeAndCheckForFailures();
    }

    bool validation_successful = std::all_of(
            function_validators.begin(), function_validators.end(),
            [](FunctionValidator &fv) { return fv.executeAndCheckForSuccess(); });

    return function_validators.empty() ? false : validation_successful;
}

void SimulatedTest::runTest(const std::vector<ValidationFunction> &validation_functions,
                            const std::vector<ValidationFunction> &continuous_validation_functions,
                            const Duration &timeout) {
    // Set up initial world state
    auto ssl_wrapper_packet = simulator->getSSLWrapperPacket();
    assert(ssl_wrapper_packet);
    auto sensor_msg = SensorMsg();
    sensor_msg.set_allocated_ssl_vision_msg(ssl_wrapper_packet.release());
    sensor_fusion.updateWorld(sensor_msg);
    std::optional<World> world_opt = sensor_fusion.getWorld();
    std::shared_ptr<World> world;
    if(world_opt) {
        world = std::make_shared<World>(world_opt.value());
    }else {
        ADD_FAILURE() << "initial world was invalid";
    }

    // Set up function validators
    for (const auto& validation_function : validation_functions)
    {
        function_validators.emplace_back(
                FunctionValidator(validation_function, world));
    }

    for (const auto &continuous_validation_function : continuous_validation_functions)
    {
        continuous_function_validators.emplace_back(
                ContinuousFunctionValidator(continuous_validation_function, world));
    }

    Timestamp current_time = Timestamp::fromSeconds(0.0);
    Timestamp timeout_time = current_time + timeout;
    Duration dt = Duration::fromSeconds(1.0 / 60.0);
    auto wall_clock_start_time = std::chrono::steady_clock::now();
    while(current_time < timeout_time) {
        simulator->stepSimulation(dt);
        auto ssl_wrapper_packet = simulator->getSSLWrapperPacket();
        assert(ssl_wrapper_packet);

        auto sensor_msg = SensorMsg();
        sensor_msg.set_allocated_ssl_vision_msg(ssl_wrapper_packet.release());

        sensor_fusion.updateWorld(sensor_msg);
        std::optional<World> world_opt = sensor_fusion.getWorld();

        if(!world_opt) {
            // TODO: log
            continue;
        }

        *world = world_opt.value();

        bool stop_test = SimulatedTest::validateWorld(world, function_validators, continuous_function_validators);
        if(stop_test) {
            // TODO: log
            break;
        }

        auto p = ai.getPrimitives(*world);
        auto new_primitives_ptr =
                std::make_shared<const std::vector<std::unique_ptr<Primitive>>>(
                        std::move(p));

        simulator->setYellowRobotPrimitives(new_primitives_ptr);

        current_time = current_time + dt;

        if (run_simulation_in_realtime)
        {
            // How long to wait for sim time to match wall clock time
            auto timestamp_now = std::chrono::steady_clock::now();
            auto milliseconds_since_test_start =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                            timestamp_now - wall_clock_start_time);
            auto ms_to_sleep = std::chrono::milliseconds(static_cast<int>(
                    current_time.getMilliseconds())) - milliseconds_since_test_start;
            if (ms_to_sleep > std::chrono::milliseconds(0))
            {
                std::this_thread::sleep_for(ms_to_sleep);
            }
//
//
//            // Calculate how much wall-clock time has passed since we last published a
//            // world, and sleep for as much time as necessary for it to have been
//            // world_time_increment seconds in wall-clock time since the last world was
//            // published.
//            auto timestamp_now = std::chrono::steady_clock::now();
//            auto milliseconds_since_world_publish =
//                    std::chrono::duration_cast<std::chrono::milliseconds>(
//                            timestamp_now - world_publish_timestamp);
//            auto remaining_milliseconds = std::chrono::milliseconds(std::lrint(
//                    world_time_increment.getMilliseconds())) -
//                                          milliseconds_since_world_publish;
//
//            if (remaining_milliseconds > std::chrono::milliseconds(0))
//            {
//                std::this_thread::sleep_for(remaining_milliseconds);
//            }
        }
    }

//    if (!validation_successful && !function_validators.empty())
//    {
//        LOG(WARNING)
//            << "Validation failed. Not all validation functions passed within the timeout duration";
//        return false;
//    }
//    else
//    {
//        return true;
//    }
}
