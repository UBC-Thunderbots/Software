#include "power_board_runtime.h"

#ifdef PLATFORMIO_BUILD
#include "power_frame_msg_platformio.h"
#include "uart_framing_platformio.h"
#else
#include "proto/message_translation/power_frame_msg.hpp"
#include "shared/uart_framing/uart_framing.hpp"
#endif

PowerBoardRuntime::PowerBoardRuntime(PowerHAL& hal, PowerController& power_controller,
                                     DribblerController& dribbler_controller)
    : hal_(hal),
      power_controller_(power_controller),
      dribbler_controller_(dribbler_controller)
{
}

void PowerBoardRuntime::receiveByte(uint8_t byte)
{
    if (!receiving_)
    {
        if (byte == START_END_FLAG_BYTE)
        {
            receive_buffer_.clear();
            receive_buffer_.push_back(byte);
            receiving_ = true;
        }
        return;
    }

    if (receive_buffer_.size() >= MAX_PACKET_SIZE)
    {
        receive_buffer_.clear();
        receiving_ = false;
        return;
    }

    receive_buffer_.push_back(byte);
    if (byte != START_END_FLAG_BYTE)
    {
        return;
    }

    receiving_                  = false;
    TbotsProto_PowerFrame frame = TbotsProto_PowerFrame_init_default;
    const bool valid_frame      = unmarshalUartPacket(receive_buffer_, frame);
    receive_buffer_.clear();
    if (valid_frame)
    {
        dispatchCommand(frame);
    }
}

std::vector<uint8_t> PowerBoardRuntime::tick()
{
    power_controller_.update();
    dribbler_controller_.update();

    const TbotsProto_PowerStatus status = createNanoPbPowerStatus(
        hal_.getBatteryVoltage(), hal_.getCapacitorVoltage(), hal_.getCurrentDrawAmp(),
        sequence_num_++, hal_.getBreakBeamTripped());
    const TbotsProto_PowerFrame status_frame = createUartFrame(status);
    return marshallUartPacket(status_frame);
}

void PowerBoardRuntime::dispatchCommand(const TbotsProto_PowerFrame& frame)
{
    switch (frame.which_power_msg)
    {
        case TbotsProto_PowerFrame_power_control_tag:
            dispatchPowerControl(frame.power_msg.power_control);
            break;
        case TbotsProto_PowerFrame_dribbler_control_tag:
            dribbler_controller_.setTargetSpeed(
                frame.power_msg.dribbler_control.dribbler_speed);
            break;
        default:
            break;
    }
}

void PowerBoardRuntime::dispatchPowerControl(const TbotsProto_PowerPulseControl& control)
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
