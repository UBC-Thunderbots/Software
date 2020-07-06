#include "software/gui/standalone_simulator/threaded_standalone_simulator_gui.h"
#include "software/logger/logger.h"
#include "software/simulation/standalone_simulator.h"
#include "software/world/field.h"

int main(int argc, char *argv[])
{
    LoggerSingleton::initializeLogger();

    StandaloneSimulator standalone_simulator(
        MutableDynamicParameters->getMutableStandaloneSimulatorConfig());
    standalone_simulator.setupInitialSimulationState();

    auto ball_placement_callback = [&standalone_simulator](Point ball_placement_point) {
        BallState state(ball_placement_point, Vector(0, 0));
        standalone_simulator.setBallState(state);
    };

    auto simulation_mode_callback =
        [&standalone_simulator](StandaloneSimulator::SimulationMode new_simulation_mode) {
            switch (new_simulation_mode)
            {
                case StandaloneSimulator::SimulationMode::PLAY:
                    standalone_simulator.resetSlowMotionMultiplier();
                    standalone_simulator.startSimulation();
                    break;
                case StandaloneSimulator::SimulationMode::PAUSE:
                    standalone_simulator.stopSimulation();
                    break;
                case StandaloneSimulator::SimulationMode::SLOW_MOTION:
                    standalone_simulator.setSlowMotionMultiplier(
                        StandaloneSimulator::DEFAULT_SLOW_MOTION_MULTIPLIER);
                    standalone_simulator.startSimulation();
                    break;
            }
        };

    ThreadedStandaloneSimulatorGUI threaded_standalone_simulator_gui(
        ball_placement_callback, simulation_mode_callback);

    standalone_simulator.registerOnSSLWrapperPacketReadyCallback(
        [&threaded_standalone_simulator_gui](SSL_WrapperPacket wrapper_packet) {
            threaded_standalone_simulator_gui.onValueReceived(wrapper_packet);
        });

    // This blocks forever without using the CPU.
    // Wait for the Simulator GUI to shut down before shutting
    // down the rest of the system
    threaded_standalone_simulator_gui.getTerminationPromise()->get_future().wait();

    return 0;
}
