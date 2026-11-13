#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "dribbler_controller.h"
#include "power_controller.h"
#include "power_hal.h"
#include "proto/power_frame_msg.nanopb.h"

/** Runs power-board command handling, policy updates, and status reporting. */
class PowerBoardRuntime
{
   public:
    PowerBoardRuntime(PowerHAL& hal, PowerController& power_controller,
                      DribblerController& dribbler_controller);

    /** Passes one byte from the command transport and dispatches complete commands. */
    void receiveByte(uint8_t byte);
    /** Advances policy and returns the marshalled status packet for this tick. */
    std::vector<uint8_t> tick();

   private:
    void dispatchCommand(const TbotsProto_PowerFrame& frame);
    void dispatchPowerControl(const TbotsProto_PowerPulseControl& control);

    // Leave room for COBS overhead and the leading/trailing delimiter bytes.
    static constexpr std::size_t MAX_PACKET_SIZE = TbotsProto_PowerFrame_size * 2 + 4;

    PowerHAL& hal_;
    PowerController& power_controller_;
    DribblerController& dribbler_controller_;
    std::vector<uint8_t> receive_buffer_;
    bool receiving_        = false;
    uint32_t sequence_num_ = 0;
};
