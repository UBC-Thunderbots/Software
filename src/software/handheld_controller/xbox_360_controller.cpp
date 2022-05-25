#include "software/handheld_controller/xbox_360_controller.h"

Xbox360Controller::Xbox360Controller(
    std::shared_ptr<const HandheldControllerConfig> controller_input_config)
    : Controller(controller_input_config)
{
}

ControllerInput Xbox360Controller::handleButtonEvent(ControllerInput controller_input,
                                                     const unsigned int button_id,
                                                     const bool is_button_pressed) const
{
    switch (button_id)
    {
        case 0:  // The A button
            controller_input.setKickButtonPressed(is_button_pressed);
            break;
        case 1:  // The B button
            controller_input.setChipButtonPressed(is_button_pressed);
            break;
        case 2:  // The X button
            break;
        case 3:  // The Y button
            controller_input.setDribblerButtonPressed(is_button_pressed);
            break;
        case 4:  // The left bumper
            break;
        case 5:  // The right bumper
            break;
        case 6:  // The back button
            break;
        case 7:  // The start button
            break;
        case 8:  // The big middle Xbox button
            break;
        case 9:  // The left joystick button
            break;
        case 10:  // The right joystick button
            break;
        case 11:  // The left direction on the D-pad
            break;
        case 12:  // The right direction on the D-pad
            break;
        case 13:  // The up direction on the D-pad
            break;
        case 14:  // The down direction on the D-pad
            break;
        default:
            throw std::invalid_argument("Unhandled button id");
    }

    return controller_input;
}

ControllerInput Xbox360Controller::handleAxisEvent(ControllerInput controller_input,
                                                   const unsigned int axis_id,
                                                   const double axis_value) const
{
    switch (axis_id)
    {
        case 0:  // Left joystick, x-axis. Negative values = left, positive = right
            controller_input.setLinearMotionY(-axis_value);
            break;
        case 1:  // Left joystick, y-axis. Negative values = up, positive = down
            controller_input.setLinearMotionX(-axis_value);
            break;
        case 2:  // Left trigger. Negative values = unpressed, positive = pressed
            break;
        case 3:  // Right joystick, x-axis. Negative values = left, positive = right
            controller_input.setAngularMotion(-axis_value);
            break;
        case 4:  // Right joystick, y-axis. Negative values = up, positive = down
            break;
        case 5:  // Right trigger. Negative values = unpressed, positive = pressed
            break;
        case 6:  // D-pad x-axis. Negative values = left, positive = right
            break;
        case 7:  // D-pad y-axis. Negative values = up, positive = down
            break;
        default:
            throw std::invalid_argument("Unhandled axis id");
    }

    return controller_input;
}
