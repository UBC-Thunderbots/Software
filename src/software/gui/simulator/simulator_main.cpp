#include "software/gui/simulator/simulator_gui_wrapper.h"
#include "software/simulation/threaded_simulator.h"
#include "software/logger/logger.h"

int main(int argc, char *argv[])
{
    LoggerSingleton::initializeLogger();

    SimulatorGUIWrapper simulator_gui_wrapper(argc, argv);
    // TODO: temporary hack creating a Division B field until static field constructors
    // are moved to the Field class
    Field field = Field(9.0, 6.0, 1.0, 2.0, 0.18, 1.0, 0.3, 0.5);
    ThreadedSimulator simulator = ThreadedSimulator(field);
    simulator.registerOnSSLWrapperPacketReadyCallback([&simulator_gui_wrapper](SSL_WrapperPacket packet) {
        simulator_gui_wrapper.onValueReceived(packet);
    });

    simulator.setBallState(BallState(Point(1, 1), Vector(2, 1.2)));

    simulator.startSimulation();

    // This blocks forever without using the CPU.
    // Wait for the Simulator GUI to shut down before shutting
    // down the rest of the system
    simulator_gui_wrapper.getTerminationPromise()->get_future().wait();

    simulator.stopSimulation();

    return 0;
}
