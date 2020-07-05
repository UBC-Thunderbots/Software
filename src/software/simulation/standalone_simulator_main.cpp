#include "software/gui/standalone_simulator/threaded_standalone_simulator_gui.h"
#include "software/logger/logger.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/simulation/standalone_simulator.h"

int main()
{
    LoggerSingleton::initializeLogger();

    ThreadedStandaloneSimulatorGUI standalone_simulator_gui_wrapper;
    StandaloneSimulator standalone_simulator(
        MutableDynamicParameters->getMutableStandaloneSimulatorConfig());
    standalone_simulator.registerOnSSLWrapperPacketReadyCallback(
        [&standalone_simulator_gui_wrapper](SSL_WrapperPacket packet) {
            standalone_simulator_gui_wrapper.onValueReceived(packet);
        });

    // This blocks forever without using the CPU.
    // Wait for the Simulator GUI to shut down before shutting
    // down the rest of the system
    standalone_simulator_gui_wrapper.getTerminationPromise()->get_future().wait();

    return 0;
}
