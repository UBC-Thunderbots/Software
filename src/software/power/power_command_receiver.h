#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "proto/power_frame_msg.nanopb.h"

/** Collects framed UART bytes and returns successfully decoded power frames. */
class PowerCommandReceiver
{
   public:
    /** Returns true when byte completes a valid frame written to frame. */
    bool receiveByte(uint8_t byte, TbotsProto_PowerFrame& frame);

   private:
    // Leave room for COBS overhead and the leading/trailing delimiter bytes.
    static constexpr size_t MAX_PACKET_SIZE = TbotsProto_PowerFrame_size * 2 + 4;

    std::vector<uint8_t> buffer_;
    bool receiving_ = false;
};
