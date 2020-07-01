#include "software/gui/standalone_simulator/threaded_standalone_simulator_gui.h"
#include "software/logger/logger.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/simulation/standalone_simulator.h"

int main(int argc, char* argv[])
{
    LoggerSingleton::initializeLogger();

    ThreadedStandaloneSimulatorGUI standalone_simulator_gui(argc, argv);

    StandaloneSimulator standalone_simulator(
        MutableDynamicParameters->getMutableStandaloneSimulatorConfig());
    standalone_simulator.setupInitialSimulationState();
    standalone_simulator.registerOnSSLWrapperPacketReadyCallback(
        [&standalone_simulator_gui](SSL_WrapperPacket wrapper_packet) {
            standalone_simulator_gui.onValueReceived(wrapper_packet);
        });

    // This blocks forever without using the CPU.
    // Wait for the Simulator GUI to shut down before shutting
    // down the rest of the system
    standalone_simulator_gui.getTerminationPromise()->get_future().wait();

    return 0;
}
