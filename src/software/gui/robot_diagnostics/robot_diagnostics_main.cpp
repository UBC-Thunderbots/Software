#include "software/gui/robot_diagnostics/widgets/main_widget.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWidget w;
    w.show();

    return a.exec();
}
