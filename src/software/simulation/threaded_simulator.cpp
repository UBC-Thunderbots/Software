#include "software/simulation/threaded_simulator.h"

ThreadedSimulator::ThreadedSimulator(
    const Field &field, std::shared_ptr<const SimulatorConfig> simulator_config)
    : simulator(field, simulator_config),
      simulation_thread_started(false),
      stopping_simulation(false),
      slow_motion_multiplier(1.0)
{
}

ThreadedSimulator::~ThreadedSimulator()
{
    stopSimulation();
}

void ThreadedSimulator::registerOnSSLWrapperPacketReadyCallback(
    const std::function<void(SSLProto::SSL_WrapperPacket)> &callback)
{
    std::scoped_lock lock(callback_mutex);
    ssl_wrapper_packet_callbacks.emplace_back(callback);
}

void ThreadedSimulator::startSimulation()
{
    std::scoped_lock lock(simulation_thread_started_mutex);
    // Start the thread to do the simulation in the background
    if (!simulation_thread_started)
    {
        simulation_thread_started = true;
        simulation_thread = std::thread(&ThreadedSimulator::runSimulationLoop, this);
    }
}

void ThreadedSimulator::stopSimulation()
{
    std::scoped_lock lock(simulation_thread_started_mutex);
    if (simulation_thread_started)
    {
        simulation_thread_started = false;

        // Set this flag so the simulation_thread knows to end
        stopping_simulation = true;

        simulation_thread.join();

        // Reset the flag in case we want to re-start the simulation
        stopping_simulation = false;
    }
}

void ThreadedSimulator::setSlowMotionMultiplier(double multiplier)
{
    if (multiplier < 1.0)
    {
        throw std::invalid_argument("Slow motion multiplier must by >= 1.0");
    }

    slow_motion_multiplier = multiplier;
}

void ThreadedSimulator::resetSlowMotionMultiplier()
{
    setSlowMotionMultiplier(1.0);
}

void ThreadedSimulator::setBallState(const BallState &ball_state)
{
    simulator_mutex.lock();
    simulator.setBallState(ball_state);
    simulator_mutex.unlock();
    updateCallbacks();
}

void ThreadedSimulator::removeBall()
{
    simulator_mutex.lock();
    simulator.removeBall();
    simulator_mutex.unlock();
    updateCallbacks();
}

void ThreadedSimulator::addYellowRobots(const std::vector<RobotStateWithId> &robots)
{
    simulator_mutex.lock();
    simulator.addYellowRobots(robots);
    simulator_mutex.unlock();
    updateCallbacks();
}

void ThreadedSimulator::addBlueRobots(const std::vector<RobotStateWithId> &robots)
{
    simulator_mutex.lock();
    simulator.addBlueRobots(robots);
    simulator_mutex.unlock();
    updateCallbacks();
}

void ThreadedSimulator::setYellowRobotPrimitive(RobotId id,
                                                const TbotsProto_Primitive &primitive_msg)
{
    std::scoped_lock lock(simulator_mutex);
    simulator.setYellowRobotPrimitive(id, primitive_msg);
}

void ThreadedSimulator::setBlueRobotPrimitive(RobotId id,
                                              const TbotsProto_Primitive &primitive_msg)
{
    std::scoped_lock lock(simulator_mutex);
    simulator.setBlueRobotPrimitive(id, primitive_msg);
}

void ThreadedSimulator::setYellowRobotPrimitiveSet(
    const TbotsProto_PrimitiveSet &primitive_set_msg)
{
    std::scoped_lock lock(simulator_mutex);
    simulator.setYellowRobotPrimitiveSet(primitive_set_msg);
}

void ThreadedSimulator::setBlueRobotPrimitiveSet(
    const TbotsProto_PrimitiveSet &primitive_set_msg)
{
    std::scoped_lock lock(simulator_mutex);
    simulator.setBlueRobotPrimitiveSet(primitive_set_msg);
}

void ThreadedSimulator::setYellowTeamDefendingSide(
    const DefendingSideProto &defending_side_proto)
{
    std::scoped_lock lock(simulator_mutex);
    simulator.setYellowTeamDefendingSide(defending_side_proto);
}

void ThreadedSimulator::setBlueTeamDefendingSide(
    const DefendingSideProto &defending_side_proto)
{
    std::scoped_lock lock(simulator_mutex);
    simulator.setBlueTeamDefendingSide(defending_side_proto);
}

void ThreadedSimulator::runSimulationLoop()
{
    Duration time_step = Duration::fromSeconds(TIME_STEP_SECONDS);

    while (!stopping_simulation.load())
    {
        auto simulation_step_start_time = std::chrono::steady_clock::now();

        simulator_mutex.lock();
        simulator.stepSimulation(time_step);
        simulator_mutex.unlock();

        updateCallbacks();

        auto simulation_step_end_time =
            simulation_step_start_time +
            std::chrono::microseconds(
                static_cast<unsigned int>(slow_motion_multiplier.load() *
                                          TIME_STEP_SECONDS * MICROSECONDS_PER_SECOND));
        // TODO: Warn or indicate if we are running slower than real-time
        // https://github.com/UBC-Thunderbots/Software/issues/1491
        std::this_thread::sleep_until(simulation_step_end_time);
    }
}

void ThreadedSimulator::updateCallbacks()
{
    simulator_mutex.lock();
    auto ssl_wrapper_packet_ptr = simulator.getSSLWrapperPacket();
    simulator_mutex.unlock();
    assert(ssl_wrapper_packet_ptr);

    {
        std::scoped_lock lock(callback_mutex);
        for (const auto &callback : ssl_wrapper_packet_callbacks)
        {
            callback(*ssl_wrapper_packet_ptr);
        }
    }
}

std::weak_ptr<PhysicsRobot> ThreadedSimulator::getRobotAtPosition(const Point &position)
{
    std::scoped_lock lock(simulator_mutex);
    return simulator.getRobotAtPosition(position);
}

void ThreadedSimulator::addYellowRobot(const Point &position)
{
    std::scoped_lock lock(simulator_mutex);
    simulator.addYellowRobot(position);
}

void ThreadedSimulator::addBlueRobot(const Point &position)
{
    std::scoped_lock lock(simulator_mutex);
    simulator.addBlueRobot(position);
}

void ThreadedSimulator::removeRobot(std::weak_ptr<PhysicsRobot> robot)
{
    std::scoped_lock lock(simulator_mutex);
    simulator.removeRobot(robot);
}
