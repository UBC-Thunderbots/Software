#pragma once

#include "power_controller.h"
#include "proto/power_frame_msg.nanopb.h"

/** Translates power commands into use-case calls. */
class ControlExecutor
{
   public:
    explicit ControlExecutor(PowerController& power_controller);

    void execute(const TbotsProto_PowerPulseControl& control);

   private:
    PowerController& power_controller_;
};
