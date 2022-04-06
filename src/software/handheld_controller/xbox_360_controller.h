#pragma once

#include "software/handheld_controller/controller.h"

class Xbox360Controller : public Controller
{
   public:
    /**
     * Creates a new Xbox360Controller
     *
     * @param controller_input_config The config for this controller
     */
    explicit Xbox360Controller(
        std::shared_ptr<const HandheldControllerConfig> controller_input_config);

    ControllerInput handleButtonEvent(ControllerInput controller_input,
                                      const unsigned int button_id,
                                      const bool is_button_pressed) const override;
    ControllerInput handleAxisEvent(ControllerInput controller_input,
                                    const unsigned int axis_id,
                                    const double axis_value) const override;
};
