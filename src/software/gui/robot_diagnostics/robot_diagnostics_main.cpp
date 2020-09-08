#include "software/gui/robot_diagnostics/threaded_robot_diagnostics_gui.h"

int main(int argc, char *argv[])
{
    std::shared_ptr<ThreadedRobotDiagnosticsGUI> threaded_robot_diagnostics_gui;
    threaded_robot_diagnostics_gui =
        std::make_shared<ThreadedRobotDiagnosticsGUI>(argc, argv);

    // This blocks forever without using the CPU
    // Wait for the GUI to shut down before shutting
    // down the rest of the system
    threaded_robot_diagnostics_gui->getTerminationPromise()->get_future().wait();
    return 0;
}
