//#include "visualizer.h"
#include <QApplication>
//#include <QtGui>
//#include <QWidget>
//#include <QMainWindow>
#include "software/gui/main_window.h"
//#include <ros/ros.h>


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ThunderbotsVisualizer visualizer;
    visualizer.show();

    return a.exec();
}
