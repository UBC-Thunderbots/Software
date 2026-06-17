#include "control_executor.h"

#include <utility>

ControlExecutor::ControlExecutor(std::shared_ptr<Charger> charger,
                                 std::shared_ptr<Chicker> chicker,
                                 std::shared_ptr<Geneva> geneva)
    : charger(std::move(charger)), chicker(std::move(chicker)), geneva(std::move(geneva))
{
}

void ControlExecutor::execute(const TbotsProto_PowerPulseControl& control)
{
    switch (control.chicker.which_chicker_command)
    {
        case TbotsProto_PowerPulseControl_ChickerControl_kick_pulse_width_tag:
            chicker->kick(control.chicker.chicker_command.kick_pulse_width);
            break;
        case TbotsProto_PowerPulseControl_ChickerControl_chip_pulse_width_tag:
            chicker->chip(control.chicker.chicker_command.chip_pulse_width);
            break;
        case TbotsProto_PowerPulseControl_ChickerControl_auto_chip_or_kick_tag:
            switch (
                control.chicker.chicker_command.auto_chip_or_kick.which_auto_chip_or_kick)
            {
                case TbotsProto_PowerPulseControl_AutoChipOrKick_autokick_pulse_width_tag:
                    chicker->autokick(control.chicker.chicker_command.auto_chip_or_kick
                                          .auto_chip_or_kick.autokick_pulse_width);
                    break;
                case TbotsProto_PowerPulseControl_AutoChipOrKick_autochip_pulse_width_tag:
                    chicker->autochip(control.chicker.chicker_command.auto_chip_or_kick
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
