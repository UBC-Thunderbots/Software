#include "software/simulated_tests/simulated_test_fixture.h"

#include <cstdlib>
#include <filesystem>

#include "proto/message_translation/ssl_wrapper.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "shared/2015_robot_constants.h"
#include "software/logger/logger.h"
#include "software/test_util/test_util.h"


SimulatedTestFixture::SimulatedTestFixture()
    : friendly_thunderbots_config(TbotsProto::ThunderbotsConfig()),
      enemy_thunderbots_config(TbotsProto::ThunderbotsConfig()),
      friendly_sensor_fusion(friendly_thunderbots_config.sensor_fusion_config()),
      enemy_sensor_fusion(enemy_thunderbots_config.sensor_fusion_config()),
      should_log_replay(false),
      run_simulation_in_realtime(false)
{
}

void SimulatedTestFixture::SetUp()
{
    LoggerSingleton::initializeLogger(TbotsGtestMain::logging_dir);

    friendly_thunderbots_config = TbotsProto::ThunderbotsConfig();
    enemy_thunderbots_config    = TbotsProto::ThunderbotsConfig();

    // The simulated test abstracts and maintains the invariant that the friendly team
    // is always the yellow team
    friendly_thunderbots_config.mutable_ai_control_config()->set_run_ai(
        !TbotsGtestMain::stop_ai_on_start);
    friendly_thunderbots_config.mutable_sensor_fusion_config()
        ->set_override_game_controller_defending_side(true);
    friendly_thunderbots_config.mutable_sensor_fusion_config()
        ->set_defending_positive_side(false);

    // Experimentally determined restitution value
    friendly_thunderbots_config.mutable_simulator_config()->set_ball_restitution(0.8);

    // Measured these values from fig. 9 on page 8 of
    // https://ssl.robocup.org/wp-content/uploads/2020/03/2020_ETDP_ZJUNlict.pdf
    friendly_thunderbots_config.mutable_simulator_config()
        ->set_sliding_friction_acceleration(6.9);
    friendly_thunderbots_config.mutable_simulator_config()
        ->set_rolling_friction_acceleration(0.5);

    // The simulated test abstracts and maintains the invariant that the friendly team
    // is always defending the "negative" side of the field. This is so that the
    // coordinates given when setting up tests is from the perspective of the friendly
    // team
    enemy_thunderbots_config.mutable_sensor_fusion_config()->set_friendly_color_yellow(
        true);
    enemy_thunderbots_config.mutable_ai_control_config()->set_run_ai(
        !TbotsGtestMain::stop_ai_on_start);

    // enemy

    // The simulated test abstracts and maintains the invariant that the friendly team
    // is always the yellow team
    enemy_thunderbots_config.mutable_sensor_fusion_config()
        ->set_override_game_controller_defending_side(true);
    enemy_thunderbots_config.mutable_sensor_fusion_config()->set_defending_positive_side(
        false);

    // Experimentally determined restitution value
    enemy_thunderbots_config.mutable_simulator_config()->set_ball_restitution(0.8);

    // Measured these values from fig. 9 on page 8 of
    // https://ssl.robocup.org/wp-content/uploads/2020/03/2020_ETDP_ZJUNlict.pdf
    enemy_thunderbots_config.mutable_simulator_config()
        ->set_sliding_friction_acceleration(6.9);
    enemy_thunderbots_config.mutable_simulator_config()
        ->set_rolling_friction_acceleration(0.5);

    // The simulated test abstracts and maintains the invariant that the friendly team
    // is always defending the "negative" side of the field. This is so that the
    // coordinates given when setting up tests is from the perspective of the friendly
    // team
    enemy_thunderbots_config.mutable_sensor_fusion_config()->set_friendly_color_yellow(
        false);

    friendly_sensor_fusion =
        SensorFusion(friendly_thunderbots_config.sensor_fusion_config());
    enemy_sensor_fusion = SensorFusion(enemy_thunderbots_config.sensor_fusion_config());

    if (TbotsGtestMain::run_sim_in_realtime)
    {
        run_simulation_in_realtime = true;
    }
    setupReplayLogging();

    // Reset tick duration trackers
    total_friendly_tick_duration = 0.0;
    total_enemy_tick_duration    = 0.0;
    // all tick times should be greater than 0
    max_friendly_tick_duration = 0.0;
    max_enemy_tick_duration    = 0.0;
    // all tick times should be less than the max value of a double
    min_friendly_tick_duration = std::numeric_limits<double>::max();
    min_enemy_tick_duration    = std::numeric_limits<double>::max();
    friendly_tick_count        = 0;
    enemy_tick_count           = 0;
}


void SimulatedTestFixture::setupReplayLogging()
{
    // get the name of the current test to name the replay output directory
    auto test_name = ::testing::UnitTest::GetInstance()->current_test_info()->name();

    namespace fs                                           = std::filesystem;
    static constexpr auto SIMULATED_TEST_OUTPUT_DIR_SUFFIX = "simulated_test_outputs";

    const char *test_outputs_dir_or_null = std::getenv("TEST_UNDECLARED_OUTPUTS_DIR");
    if (!test_outputs_dir_or_null)
    {
        // we're not running with the Bazel test env vars set, don't set up replay logging
        return;
    }

    fs::path bazel_test_outputs_dir(test_outputs_dir_or_null);
    fs::path out_dir =
        bazel_test_outputs_dir / SIMULATED_TEST_OUTPUT_DIR_SUFFIX / test_name;
    fs::create_directories(out_dir);

    LOG(INFO) << "Logging " << test_name << " replay to " << out_dir;

    fs::path sensorproto_out_dir = out_dir / "Simulator_SensorProto";
    fs::path ssl_wrapper_out_dir = out_dir / "SensorFusion_SSL_WrapperPacket";

    simulator_sensorproto_logger =
        std::make_shared<ProtoLogger<SensorProto>>(sensorproto_out_dir);
    sensorfusion_wrapper_logger =
        std::make_shared<ProtoLogger<SSLProto::SSL_WrapperPacket>>(ssl_wrapper_out_dir);
    should_log_replay = true;
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

void SimulatedTestFixture::updateSensorFusion(std::shared_ptr<Simulator> simulator)
{
    auto ssl_wrapper_packet = simulator->getSSLWrapperPacket();
    assert(ssl_wrapper_packet);

    auto sensor_msg                        = SensorProto();
    *(sensor_msg.mutable_ssl_vision_msg()) = *ssl_wrapper_packet;

    friendly_sensor_fusion.processSensorProto(sensor_msg);
    enemy_sensor_fusion.processSensorProto(sensor_msg);

    if (should_log_replay)
    {
        simulator_sensorproto_logger->onValueReceived(sensor_msg);
        auto friendly_world_or_null = friendly_sensor_fusion.getWorld();

        if (friendly_world_or_null)
        {
            auto filtered_ssl_wrapper = *createSSLWrapperPacket(
                *friendly_sensor_fusion.getWorld(), TeamColour::YELLOW);
            sensorfusion_wrapper_logger->onValueReceived(filtered_ssl_wrapper);
        }
    }
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
    const Field &field, const BallState &ball,
    const std::vector<RobotStateWithId> &friendly_robots,
    const std::vector<RobotStateWithId> &enemy_robots,
    const std::vector<ValidationFunction> &terminating_validation_functions,
    const std::vector<ValidationFunction> &non_terminating_validation_functions,
    const Duration &timeout)
{
    std::shared_ptr<Simulator> simulator(std::make_shared<Simulator>(
        field, create2015RobotConstants(), create2015WheelConstants(),
        friendly_thunderbots_config.simulator_config()));
    simulator->setBallState(ball);
    simulator->addYellowRobots(friendly_robots);
    simulator->addBlueRobots(enemy_robots);

    updateSensorFusion(simulator);
    std::shared_ptr<World> friendly_world;
    std::shared_ptr<World> enemy_world;
    if (friendly_sensor_fusion.getWorld().has_value() &&
        enemy_sensor_fusion.getWorld().has_value())
    {
        friendly_world =
            std::make_shared<World>(friendly_sensor_fusion.getWorld().value());
        enemy_world = std::make_shared<World>(enemy_sensor_fusion.getWorld().value());
    }
    else
    {
        FAIL() << "Invalid initial world state";
    }

    for (const auto &validation_function : terminating_validation_functions)
    {
        terminating_function_validators.emplace_back(
            TerminatingFunctionValidator(validation_function, friendly_world));
    }

    for (const auto &validation_function : non_terminating_validation_functions)
    {
        non_terminating_function_validators.emplace_back(
            NonTerminatingFunctionValidator(validation_function, friendly_world));
    }

    const Timestamp timeout_time = simulator->getTimestamp() + timeout;
    const Duration simulation_time_step =
        Duration::fromSeconds(1.0 / SIMULATED_CAMERA_FPS);


    double speed_factor         = 1 / (TbotsGtestMain::test_speed);
    const Duration ai_time_step = Duration::fromSeconds(
        simulation_time_step.toSeconds() * CAMERA_FRAMES_PER_AI_TICK * speed_factor);

    // Tick one frame to aid with visualization
    bool validation_functions_done = tickTest(simulation_time_step, ai_time_step,
                                              friendly_world, enemy_world, simulator);

    while (simulator->getTimestamp() < timeout_time && !validation_functions_done)
    {
        if (!friendly_thunderbots_config.ai_control_config().run_ai())
        {
            auto ms_to_sleep = std::chrono::milliseconds(
                static_cast<int>(ai_time_step.toMilliseconds()));
            std::this_thread::sleep_for(ms_to_sleep);
            continue;
        }

        validation_functions_done = tickTest(simulation_time_step, ai_time_step,
                                             friendly_world, enemy_world, simulator);
    }
    // Output the tick duration results
    double avg_friendly_tick_duration =
        total_friendly_tick_duration / friendly_tick_count;
    LOG(INFO) << "max friendly tick duration: " << max_friendly_tick_duration << "ms"
              << std::endl;
    LOG(INFO) << "min friendly tick duration: " << min_friendly_tick_duration << "ms"
              << std::endl;
    LOG(INFO) << "avg friendly tick duration: " << avg_friendly_tick_duration << "ms"
              << std::endl;

    if (enemy_tick_count > 0)
    {
        double avg_enemy_tick_duration = total_enemy_tick_duration / enemy_tick_count;
        LOG(INFO) << "max enemy tick duration: " << max_enemy_tick_duration << "ms"
                  << std::endl;
        LOG(INFO) << "min enemy tick duration: " << min_enemy_tick_duration << "ms"
                  << std::endl;
        LOG(INFO) << "avg enemy tick duration: " << avg_enemy_tick_duration << "ms"
                  << std::endl;
    }


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

void SimulatedTestFixture::registerFriendlyTickTime(double tick_time_ms)
{
    total_friendly_tick_duration += tick_time_ms;
    max_friendly_tick_duration = std::max(max_friendly_tick_duration, tick_time_ms);
    min_friendly_tick_duration = std::min(min_friendly_tick_duration, tick_time_ms);
    friendly_tick_count++;
}

void SimulatedTestFixture::registerEnemyTickTime(double tick_time_ms)
{
    total_enemy_tick_duration += tick_time_ms;
    max_enemy_tick_duration = std::max(max_enemy_tick_duration, tick_time_ms);
    min_enemy_tick_duration = std::min(min_enemy_tick_duration, tick_time_ms);
    enemy_tick_count++;
}

bool SimulatedTestFixture::tickTest(Duration simulation_time_step, Duration ai_time_step,
                                    std::shared_ptr<World> friendly_world,
                                    std::shared_ptr<World> enemy_world,
                                    std::shared_ptr<Simulator> simulator)
{
    auto wall_start_time           = std::chrono::steady_clock::now();
    bool validation_functions_done = false;

    for (size_t i = 0; i < CAMERA_FRAMES_PER_AI_TICK; i++)
    {
        simulator->stepSimulation(simulation_time_step);
        updateSensorFusion(simulator);
    }

    if (friendly_sensor_fusion.getWorld().has_value() &&
        enemy_sensor_fusion.getWorld().has_value())
    {
        *friendly_world = friendly_sensor_fusion.getWorld().value();
        *enemy_world    = enemy_sensor_fusion.getWorld().value();

        LOG(VISUALIZE) << *createWorld(*friendly_world);

        validation_functions_done = validateAndCheckCompletion(
            terminating_function_validators, non_terminating_function_validators);
        if (validation_functions_done)
        {
            return validation_functions_done;
        }

        updatePrimitives(*friendly_world, *enemy_world,
                         simulator);  // pass friendly and enemy world

        if (run_simulation_in_realtime)
        {
            sleep(wall_start_time, ai_time_step);
        }
    }
    else
    {
        LOG(WARNING) << "SensorFusion did not output a valid World";
    }
    return validation_functions_done;
}
