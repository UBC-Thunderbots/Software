#pragma once

#include "proto/power_frame_msg.nanopb.h"

/** Policy boundary used by the power-board runtime for chicker commands. */
class PowerCommandHandler
{
   public:
    virtual ~PowerCommandHandler() = default;

    virtual void execute(const TbotsProto_PowerPulseControl& control) = 0;
    virtual void update()                                             = 0;
};
