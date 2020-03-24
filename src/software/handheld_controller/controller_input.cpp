#include "software/handheld_controller/controller_input.h"

#include <algorithm>

ControllerInput::ControllerInput()
    : linear_motion_x(0.0),
      linear_motion_y(0.0),
      angular_motion(0.0),
      kick_button_pressed(false),
      chip_button_pressed(false),
      dribbler_button_pressed(false)
{
}

double ControllerInput::getLinearMotionX() const
{
    return linear_motion_x;
}

double ControllerInput::getLinearMotionY() const
{
    return linear_motion_y;
}

double ControllerInput::getAngularMotion() const
{
    return angular_motion;
}

bool ControllerInput::isKickButtonPressed() const
{
    return kick_button_pressed;
}

bool ControllerInput::isChipButtonPressed() const
{
    return chip_button_pressed;
}

bool ControllerInput::isDribblerButtonPressed() const
{
    return dribbler_button_pressed;
}

void ControllerInput::setLinearMotionX(double value)
{
    linear_motion_x = std::clamp<double>(value, -1.0, 1.0);
}

void ControllerInput::setLinearMotionY(double value)
{
    linear_motion_y = std::clamp<double>(value, -1.0, 1.0);
}

void ControllerInput::setAngularMotion(double value)
{
    angular_motion = std::clamp<double>(value, -1.0, 1.0);
}

void ControllerInput::setKickButtonPressed(bool pressed)
{
    kick_button_pressed = pressed;
}

void ControllerInput::setChipButtonPressed(bool pressed)
{
    chip_button_pressed = pressed;
}

void ControllerInput::setDribblerButtonPressed(bool pressed)
{
    dribbler_button_pressed = pressed;
}
