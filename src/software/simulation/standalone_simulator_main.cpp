#include "software/gui/standalone_simulator/threaded_standalone_simulator_gui.h"
#include "software/logger/logger.h"
#include "software/simulation/standalone_simulator.h"
#include "software/simulation/threaded_simulator.h"
#include "software/world/field.h"

int main()
{
    LoggerSingleton::initializeLogger();

    StandaloneSimulator standalone_simulator(
        MutableDynamicParameters->getMutableStandaloneSimulatorConfig());

    standalone_simulator.setupInitialSimulationState();

    ThreadedStandaloneSimulatorGUI threaded_standalone_simulator_gui;

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
