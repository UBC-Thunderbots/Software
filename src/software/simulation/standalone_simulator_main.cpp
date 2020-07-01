#include "software/gui/standalone_simulator/threaded_standalone_simulator_gui.h"
#include "software/logger/logger.h"
#include "software/simulation/threaded_simulator.h"
#include "software/world/field.h"

int main(int argc, char *argv[])
{
    LoggerSingleton::initializeLogger();

    ThreadedStandaloneSimulatorGUI standalone_simulator_gui_wrapper;
    ThreadedSimulator simulator = ThreadedSimulator(Field::createSSLDivisionBField());
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
