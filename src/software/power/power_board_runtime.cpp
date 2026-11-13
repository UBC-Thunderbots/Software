#include "power_board_runtime.h"

#ifdef PLATFORMIO_BUILD
#include "power_frame_msg_platformio.h"
#include "uart_framing_platformio.h"
#else
#include "proto/message_translation/power_frame_msg.hpp"
#include "shared/uart_framing/uart_framing.hpp"
#endif

PowerBoardRuntime::PowerBoardRuntime(PowerHAL& hal, PowerCommandHandler& command_handler,
                                     DribblerController& dribbler_controller)
    : hal_(hal),
      command_handler_(command_handler),
      dribbler_controller_(dribbler_controller)
{
}

void PowerBoardRuntime::receiveByte(uint8_t byte)
{
    TbotsProto_PowerFrame frame = TbotsProto_PowerFrame_init_default;
    if (command_receiver_.receiveByte(byte, frame))
    {
        dispatchCommand(frame);
    }
}

std::vector<uint8_t> PowerBoardRuntime::tick()
{
    command_handler_.update();
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
            command_handler_.execute(frame.power_msg.power_control);
            break;
        case TbotsProto_PowerFrame_dribbler_control_tag:
            dribbler_controller_.setTargetSpeed(
                frame.power_msg.dribbler_control.dribbler_speed);
            break;
        default:
            break;
    }
}
