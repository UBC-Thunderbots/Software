#include "software/robot_diagnostics/widgets/drive.h"

void setupDrive(Ui::AutoGeneratedMainWidget *widget, int motor_power_fl,
                int motor_power_fr, int motor_power_bl, int motor_power_br, int matrix_x, int matrix_y, int matrix_theta)
{
    setupSliderLineEdit(widget->lineEdit_motor_fl, widget->slider_motor_fl,
                        motor_power_fl, -100, 100);
    setupSliderLineEdit(widget->lineEdit_motor_fr, widget->slider_motor_fr,
                        motor_power_fr, -100, 100);
    setupSliderLineEdit(widget->lineEdit_motor_bl, widget->slider_motor_bl,
                        motor_power_bl, -100, 100);
    setupSliderLineEdit(widget->lineEdit_motor_br, widget->slider_motor_br,
                        motor_power_br, -100, 100);
    setupSliderLineEdit(widget->lineEdit_matrix_x, widget->slider_matrix_x,
                        matrix_x, -1000, 1000);
    setupSliderLineEdit(widget->lineEdit_matrix_y, widget->slider_matrix_y,
                        matrix_y, -1000, 1000);
    setupSliderLineEdit(widget->lineEdit_matrix_theta, widget->slider_matrix_theta,
                        matrix_theta, -1000, 1000);
}
