#include "software/gui/standalone_simulator/standalone_simulator_gui_wrapper.h"
#include "software/logger/logger.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/simulation/standalone_simulator.h"

int main(int argc, char* argv[])
{
    LoggerSingleton::initializeLogger();

    StandaloneSimulatorGUIWrapper standalone_simulator_gui_wrapper(argc, argv);

    StandaloneSimulator standalone_simulator(MutableDynamicParameters->getMutableStandaloneSimulatorConfig());
    standalone_simulator.setupInitialSimulationState();
    standalone_simulator.registerOnSSLWrapperPacketReadyCallback([&standalone_simulator_gui_wrapper](SSL_WrapperPacket wrapper_packet) {
        standalone_simulator_gui_wrapper.onValueReceived(wrapper_packet);
    });

    // This blocks forever without using the CPU.
    // Wait for the Simulator GUI to shut down before shutting
    // down the rest of the system
    standalone_simulator_gui_wrapper.getTerminationPromise()->get_future().wait();

    return 0;
}
