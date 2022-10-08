#pragma once

#include <memory>

#include "charger.h"
#include "chicker.h"
#include "geneva.h"
#include "proto/power_frame_msg.nanopb.h"

class ControlExecutor
{
   public:
    /**
     * Creates a control executor with the given classes
     * @param charger Charger to use
     * @param chicker Chicker to use
     * @param geneva Geneva to use
     */
    ControlExecutor(std::shared_ptr<Charger> charger, std::shared_ptr<Chicker> chicker,
                    std::shared_ptr<Geneva> geneva);
    /**
     * Executes the power control command using the relevant classes
     *
     * @param control control command to execute
     */
    void execute(const TbotsProto_PowerPulseControl& control);

   private:
    std::shared_ptr<Charger> charger;
    std::shared_ptr<Chicker> chicker;
    std::shared_ptr<Geneva> geneva;
};
