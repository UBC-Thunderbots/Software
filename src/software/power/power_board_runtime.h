#pragma once

#include <cstdint>
#include <vector>

#include "dribbler_controller.h"
#include "power_command_handler.h"
#include "power_command_receiver.h"
#include "power_hal.h"

/** Runs power-board command handling, policy updates, and status reporting. */
class PowerBoardRuntime
{
   public:
    PowerBoardRuntime(PowerHAL& hal, PowerCommandHandler& command_handler,
                      DribblerController& dribbler_controller);

    /** Passes one byte from the command transport to the receiver. */
    void receiveByte(uint8_t byte);
    /** Advances policy and returns the marshalled status packet for this tick. */
    std::vector<uint8_t> tick();

   private:
    void dispatchCommand(const TbotsProto_PowerFrame& frame);

    PowerHAL& hal_;
    PowerCommandHandler& command_handler_;
    DribblerController& dribbler_controller_;
    PowerCommandReceiver command_receiver_;
    uint32_t sequence_num_ = 0;
};
