#include "control_executor.h"

#include <utility>

ControlExecutor::ControlExecutor(std::shared_ptr<Charger> charger,
                                 std::shared_ptr<Chicker> chicker,
                                 std::shared_ptr<Geneva> geneva)
    : charger(std::move(charger)), chicker(std::move(chicker)), geneva(std::move(geneva))
{
}

void ControlExecutor::execute(const TbotsProto_PowerControl& control)
{
    chicker->coolDownPoll();

    switch (control.chicker.which_chicker_command)
    {
        case TbotsProto_PowerControl_ChickerControl_kick_speed_m_per_s_tag:
            chicker->setKickSpeedMPerS(
                control.chicker.chicker_command.kick_speed_m_per_s);
            chicker->kick();
            break;
        case TbotsProto_PowerControl_ChickerControl_chip_distance_meters_tag:
            chicker->setChipDistanceMeters(
                control.chicker.chicker_command.chip_distance_meters);
            chicker->chip();
            break;
        case TbotsProto_PowerControl_ChickerControl_auto_chip_or_kick_tag:
            detachInterrupt(digitalPinToInterrupt(BREAK_BEAM_PIN));
            switch (
                control.chicker.chicker_command.auto_chip_or_kick.which_auto_chip_or_kick)
            {
                case TbotsProto_AutoChipOrKick_autokick_speed_m_per_s_tag:
                    chicker->setKickSpeedMPerS(
                        control.chicker.chicker_command.auto_chip_or_kick
                            .auto_chip_or_kick.autokick_speed_m_per_s);
                    chicker->autokick();
                    break;
                case TbotsProto_AutoChipOrKick_autochip_distance_meters_tag:
                    chicker->setChipDistanceMeters(
                        control.chicker.chicker_command.auto_chip_or_kick
                            .auto_chip_or_kick.autochip_distance_meters);
                    chicker->autochip();
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}
