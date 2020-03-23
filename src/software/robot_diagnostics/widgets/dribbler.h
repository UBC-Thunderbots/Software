#pragma once

#include <QtWidgets/QWidget>

#include "software/robot_diagnostics/widgets/slider.h"

// This include is autogenerated by the .ui file in the same folder
// The generated version will be names 'ui_<filename>.h'
#include "software/robot_diagnostics/ui/ui_main_widget.h"

void setupDribbler(Ui::AutoGeneratedMainWidget *widget, int &dribbler_power);
