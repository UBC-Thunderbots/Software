#include "control_executor.h"

ControlExecutor::ControlExecutor(PowerController& power_controller)
    : power_controller_(power_controller)
{
}

void ControlExecutor::execute(const TbotsProto_PowerPulseControl& control)
{
    switch (control.chicker.which_chicker_command)
    {
        case TbotsProto_PowerPulseControl_ChickerControl_kick_pulse_width_tag:
            power_controller_.kick(control.chicker.chicker_command.kick_pulse_width);
            break;
        case TbotsProto_PowerPulseControl_ChickerControl_chip_pulse_width_tag:
            power_controller_.chip(control.chicker.chicker_command.chip_pulse_width);
            break;
        case TbotsProto_PowerPulseControl_ChickerControl_auto_chip_or_kick_tag:
            switch (
                control.chicker.chicker_command.auto_chip_or_kick.which_auto_chip_or_kick)
            {
                case TbotsProto_PowerPulseControl_AutoChipOrKick_autokick_pulse_width_tag:
                    power_controller_.autokick(
                        control.chicker.chicker_command.auto_chip_or_kick
                            .auto_chip_or_kick.autokick_pulse_width);
                    break;
                case TbotsProto_PowerPulseControl_AutoChipOrKick_autochip_pulse_width_tag:
                    power_controller_.autochip(
                        control.chicker.chicker_command.auto_chip_or_kick
                            .auto_chip_or_kick.autochip_pulse_width);
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}
