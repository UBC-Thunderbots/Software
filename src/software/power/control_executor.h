#pragma once

#include "power_command_handler.h"
#include "power_controller.h"
#include "proto/power_frame_msg.nanopb.h"

/** Translates power commands into use-case calls. */
class ControlExecutor final : public PowerCommandHandler
{
   public:
    explicit ControlExecutor(PowerController& power_controller);

    void execute(const TbotsProto_PowerPulseControl& control) override;
    void update() override;

   private:
    PowerController& power_controller_;
};
