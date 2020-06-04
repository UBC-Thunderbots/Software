#include "software/gui/robot_diagnostics/widgets/robot_diagnostics.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    RobotDiagnostics w;
    w.show();

    return a.exec();
}
