#include "software/handheld_controller/controller.h"

#include <exception>
#include <limits>

#include "software/logger/logger.h"
#include "software/math/math_functions.h"

Controller::Controller(
    std::shared_ptr<const HandheldControllerConfig> controller_input_config)
    : controller_input_config(controller_input_config), in_destructor(false)
{
    event_loop_thread = std::thread([this]() { this->eventLoop(); });
}

Controller::~Controller()
{
    in_destructor = true;

    event_loop_thread.join();
}

void Controller::eventLoop()
{
    std::string device_path        = controller_input_config->getDevicePath()->value();
    int controller_file_descriptor = open(device_path.c_str(), O_RDONLY);
    if (controller_file_descriptor == -1)
    {
        std::string exception_message =
            std::string("Unable to open the controller device path '") + device_path +
            std::string("' for reading");
        throw std::invalid_argument(exception_message);
    }

    size_t num_buttons = getNumButtons(controller_file_descriptor);
    size_t num_axes    = getNumAxes(controller_file_descriptor);
    LOG(INFO) << "Started listening to the handheld controller at '" << device_path
              << "' with " << num_buttons << " buttons and " << num_axes << " axes"
              << std::endl;

    struct js_event controller_event;
    ControllerInput controller_input;

    while (!in_destructor.load() &&
           readControllerEvent(controller_file_descriptor, &controller_event) == 0)
    {
        controller_input = handleControllerEvent(controller_input, controller_event);
        Subject<ControllerInput>::sendValueToObservers(controller_input);
    }
}

ControllerInput Controller::handleControllerEvent(
    ControllerInput controller_input, const struct js_event &controller_event) const
{
    switch (controller_event.type)
    {
        case JS_EVENT_BUTTON:
        {
            unsigned int button_id = static_cast<unsigned int>(controller_event.number);
            bool is_button_pressed = static_cast<bool>(controller_event.value);
            controller_input =
                handleButtonEvent(controller_input, button_id, is_button_pressed);
            break;
        }
        case JS_EVENT_AXIS:
        {
            unsigned int axis_id = static_cast<unsigned int>(controller_event.number);
            double max_axis_value =
                static_cast<double>(std::numeric_limits<short>::max());
            double axis_value =
                static_cast<double>(controller_event.value) / max_axis_value;
            axis_value       = applyDeadzone(axis_value);
            controller_input = handleAxisEvent(controller_input, axis_id, axis_value);
            break;
        }
        default:
            // Ignore init events
            break;
    }

    return controller_input;
}

double Controller::applyDeadzone(double axis_value) const
{
    if (axis_value > 0)
    {
        axis_value = std::max<double>(axis_value - axis_deadzone, 0);
        axis_value =
            normalizeValueToRange<double>(axis_value, 0, 1 - axis_deadzone, 0, 1);
    }
    else
    {
        axis_value = std::min<double>(axis_value + axis_deadzone, 0);
        axis_value =
            normalizeValueToRange<double>(axis_value, -1 + axis_deadzone, 0, -1, 0);
    }

    return axis_value;
}

int Controller::readControllerEvent(int fd, struct js_event *event) const
{
    ssize_t bytes;

    bytes = read(fd, event, sizeof(*event));

    if (bytes == sizeof(*event))
        return 0;

    // Error, could not read full event.
    return -1;
}

size_t Controller::getNumAxes(int fd) const
{
    __u8 axes;
    if (ioctl(fd, JSIOCGAXES, &axes) == -1)
        return 0;

    return axes;
}

size_t Controller::getNumButtons(int fd) const
{
    __u8 buttons;
    if (ioctl(fd, JSIOCGBUTTONS, &buttons) == -1)
        return 0;

    return buttons;
}
