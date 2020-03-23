#include "software/robot_diagnostics/widgets/drive.h"

void setupDrive(Ui::AutoGeneratedMainWidget *widget, double motor_power_fl,
                double motor_power_fr, double motor_power_bl, double motor_power_br,
                double matrix_x_vel, double matrix_y_vel, double matrix_angular_vel)
{
    setupSliderLineEdit(widget->lineEdit_motor_fl, widget->slider_motor_fl,
                        motor_power_fl, -100.0, 100.0, 1.0);
    setupSliderLineEdit(widget->lineEdit_motor_fr, widget->slider_motor_fr,
                        motor_power_fr, -100.0, 100.0, 1.0);
    setupSliderLineEdit(widget->lineEdit_motor_bl, widget->slider_motor_bl,
                        motor_power_bl, -100.0, 100.0, 1.0);
    setupSliderLineEdit(widget->lineEdit_motor_br, widget->slider_motor_br,
                        motor_power_br, -100.0, 100.0, 1.0);
    setupSliderLineEdit(widget->lineEdit_matrix_x, widget->slider_matrix_x, matrix_x_vel,
                        -10.0, 10.0, 100.0);
    setupSliderLineEdit(widget->lineEdit_matrix_y, widget->slider_matrix_y, matrix_y_vel,
                        -10.0, 10.0, 100.0);
    setupSliderLineEdit(widget->lineEdit_matrix_theta, widget->slider_matrix_theta,
                        matrix_angular_vel, -1000.0, 1000.0, 1.0);
}
