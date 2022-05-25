#pragma once

#include <fcntl.h>
#include <linux/joystick.h>
#include <unistd.h>

#include <atomic>
#include <thread>

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/handheld_controller/controller_input.h"
#include "software/multithreading/subject.hpp"

/**
 * Abstracts any handheld game controller, like an XBox or Playstation controller.
 *
 * Adapted from:
 * https://gist.github.com/jasonwhite/c5b2048c15993d285130#file-joystick-c-L112
 */
class Controller : public Subject<ControllerInput>
{
   public:
    /**
     * Creates a new Controller
     *
     * @param controller_input_config The config for this controller
     */
    explicit Controller(
        std::shared_ptr<const HandheldControllerConfig> controller_input_config);
    ~Controller() override;

   protected:
    /**
     * Updates the given ControllerInput based on the button event and returns a
     * new ControllerInput
     *
     * @param controller_input The initial ControllerInput
     * @param button_id The id of the button in the event
     * @param button_pressed Whether or not the button in the event is pressed or released
     *
     * @return A new ControllerInput updated by the button event
     */
    virtual ControllerInput handleButtonEvent(ControllerInput controller_input,
                                              const unsigned int button_id,
                                              const bool button_pressed) const = 0;

    /**
     * Updates the given ControllerInput based on the axis event and returns a
     * new ControllerInput
     *
     * @param controller_input The initial ControllerInput
     * @param axis_id The id of the axis in the event
     * @param axis_value The value of the axis, in the range [-1.0, 1.0]
     *
     * @return A new ControllerInput updated by the axis event
     */
    virtual ControllerInput handleAxisEvent(ControllerInput controller_input,
                                            const unsigned int axis_id,
                                            const double axis_value) const = 0;

   private:
    /**
     * The main function that reads controller events and handles input
     */
    void eventLoop();

    /**
     * Updates the given ControllerInput based on the controller event and
     * returns a new ControllerInput
     *
     * @param controller_input The initial ControllerInput
     * @param controller_event The controller event
     *
     * @return A new ControllerInput updated by the controller event
     */
    ControllerInput handleControllerEvent(ControllerInput controller_input,
                                          const struct js_event& controller_event) const;

    /**
     * Applies a deadzone to the given axis value.
     *
     * If the absolute value of the axis value is less than the deadzone, the value
     * becomes zero.
     *
     * @param axis_value The value to apply the deadzone to
     *
     * @return The new axis value accounting for the deadzone
     */
    double applyDeadzone(double axis_value) const;

    /**
     * Reads controller events from the file descriptor and stores them in the
     * given js_event struct. Returns 0 if the read was successful, and -1 if
     * there was an error
     *
     * @param fd The file descriptor for the controller whose input is being read
     * @param event The struct to store the event info in
     *
     * @return 0 if the read was successful, and -1 if an error occurred
     */
    int readControllerEvent(int fd, struct js_event* event) const;

    /**
     * Returns the number axes that exist on the controller. An axis is a single axis of
     * a joystick or trigger. For example, a traditional thumb joystick has 2 axes (x and
     * y), and a trigger has a single axis.
     *
     * @param fd The file descriptor for the controller
     *
     * @return the number of axes that exist on the controller
     */
    size_t getNumAxes(int fd) const;

    /**
     * Returns the number of buttons that exist on the controller
     *
     * @param fd The file descriptor for the controller
     *
     * @return the number of buttons that exist on the controller
     */
    size_t getNumButtons(int fd) const;

    std::thread event_loop_thread;
    std::shared_ptr<const HandheldControllerConfig> controller_input_config;
    std::atomic_bool in_destructor;
    static constexpr double axis_deadzone = 0.05;
};
