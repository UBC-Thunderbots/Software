#pragma once

#include <QtWidgets/QWidget>

#include "software/robot_diagnostics/widgets/slider.h"

// This include is autogenerated by the .ui file in the same folder
// The generated version will be names 'ui_<filename>.h'
#include "software/robot_diagnostics/ui/ui_main_widget.h"

void setupDrive(Ui::AutoGeneratedMainWidget *widget, double motor_power_fl,
                double motor_power_fr, double motor_power_bl, double motor_power_br,
                double matrix_x, double matrix_y, double matrix_theta);
