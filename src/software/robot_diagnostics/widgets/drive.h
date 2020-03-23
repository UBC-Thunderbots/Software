#pragma once

#include <QtWidgets/QWidget>

#include "software/robot_diagnostics/widgets/slider.h"

// This include is autogenerated by the .ui file in the same folder
// The generated version will be names 'ui_<filename>.h'
#include "software/robot_diagnostics/ui/ui_main_widget.h"

void setupDrive(Ui::AutoGeneratedMainWidget *widget, int motor_power_fl,
                int motor_power_fr, int motor_power_bl, int motor_power_br, int matrix_x, int matrix_y, int matrix_theta);
