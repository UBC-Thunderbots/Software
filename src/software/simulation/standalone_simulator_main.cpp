#include "software/gui/standalone_simulator/standalone_simulator_gui_wrapper.h"
#include "software/logger/logger.h"
#include "software/simulation/threaded_simulator.h"

int main(int argc, char *argv[])
{
    LoggerSingleton::initializeLogger();

    StandaloneSimulatorGUIWrapper standalone_simulator_gui_wrapper(argc, argv);
    // TODO: temporary hack creating a Division B field until static field constructors
    // are moved to the Field class. Will be fixed once
    // https://github.com/UBC-Thunderbots/Software/pull/1497 is merged
    Field field                 = Field(9.0, 6.0, 1.0, 2.0, 0.18, 1.0, 0.3, 0.5);
    ThreadedSimulator simulator = ThreadedSimulator(field);
    simulator.registerOnSSLWrapperPacketReadyCallback(
        [&standalone_simulator_gui_wrapper](SSL_WrapperPacket packet) {
            standalone_simulator_gui_wrapper.onValueReceived(packet);
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
