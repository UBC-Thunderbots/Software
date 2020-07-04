#include "software/gui/standalone_simulator/threaded_standalone_simulator_gui.h"
#include "software/logger/logger.h"
#include "software/simulation/threaded_simulator.h"
#include "software/world/field.h"

int main(int argc, char *argv[])
{
    LoggerSingleton::initializeLogger();

    auto simulation_config = MutableDynamicParameters->getMutableStandaloneSimulatorConfig()->getMutableSimulatorCommandsConfig();

    ThreadedSimulator simulator = ThreadedSimulator(Field::createSSLDivisionBField());
    ThreadedStandaloneSimulatorGUI standalone_simulator_gui_wrapper([&simulator](Point ball_placement_point) {
        BallState state(ball_placement_point, Vector(0, 0));
        simulator.setBallState(state);
    }, simulation_config);
    simulator.registerOnSSLWrapperPacketReadyCallback(
        [&standalone_simulator_gui_wrapper](SSL_WrapperPacket packet) {
            standalone_simulator_gui_wrapper.onValueReceived(packet);
        });
    simulation_config->mutableSimulationSpeedMode()->registerCallbackFunction([&simulator](int mode) {
        std::cout << "callback 2 electrib boolasfd" << std::endl;
        switch(mode) {
            case 0:
                simulator.setSlowMotionMultiplier(1.0);
                simulator.startSimulation();
                break;
            case 1:
                simulator.stopSimulation();
                break;
            case 2:
                simulator.setSlowMotionMultiplier(5.0);
                simulator.startSimulation();
                break;
            default:
                throw std::runtime_error("Unknown simulator mode");
        }

    });

    simulator.setBallState(BallState(Point(1, 1), Vector(2, 1.2)));

    simulator.startSimulation();

    // This blocks forever without using the CPU.
    // Wait for the Simulator GUI to shut down before shutting
    // down the rest of the system
    standalone_simulator_gui_wrapper.getTerminationPromise()->get_future().wait();

    simulator.stopSimulation();

    return 0;
}
