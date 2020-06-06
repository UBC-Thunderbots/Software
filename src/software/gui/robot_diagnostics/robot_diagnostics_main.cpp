#include "software/gui/robot_diagnostics/robot_diagnostics_wrapper.h"

int main(int argc, char *argv[])
{
    std::shared_ptr<RobotDiagnosticsWrapper> robot_diagnostics_wrapper;
    robot_diagnostics_wrapper = std::make_shared<RobotDiagnosticsWrapper>(argc, argv);

    // This blocks forever without using the CPU
    // Wait for the robot_diagnostics to shut down before shutting
    // down the rest of the system
    robot_diagnostics_wrapper->getTerminationPromise()->get_future().wait();

    return 0;
}
