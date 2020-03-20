#include "software/handheld_controller/controller.h"
#include <limits>
#include "software/logger/init.h"
#include <exception>

Controller::Controller(std::shared_ptr<const HandheldControllerInputConfig> controller_input_config) : controller_input_config(controller_input_config), in_destructor(false) {
    event_thread = std::thread([this]() { this->eventLoop(); });
}

Controller::~Controller() {
    in_destructor = true;

    event_thread.join();
}

void Controller::eventLoop() {
    std::string device_path = controller_input_config->DevicePath()->value();
    int controller_file_descriptor = open(device_path.c_str(), O_RDONLY);
    if (controller_file_descriptor == -1) {
        std::string exception_message = std::string("Unable to open the controller device path '") + device_path + std::string("' for reading");
        throw std::invalid_argument(exception_message);
    }

    size_t num_buttons = getNumButtons(controller_file_descriptor);
    size_t num_axes = getNumAxes(controller_file_descriptor);
    LOG(INFO) << "Started listening to the handheld controller at '" << device_path << "' with " << num_buttons << " buttons and " << num_axes << " axes" << std::endl;

    struct js_event controller_event;
    ControllerInput controller_input;

    while (!in_destructor.load() && read_event(controller_file_descriptor, &controller_event) == 0) {
        controller_input = handleControllerEvent(controller_input, controller_event);
        Subject<ControllerInput>::sendValueToObservers(controller_input);
    }
}

ControllerInput Controller::handleControllerEvent(ControllerInput controller_input, const struct js_event &controller_event) {
    switch (controller_event.type)
    {
        case JS_EVENT_BUTTON:
        {
            unsigned int button_id = static_cast<unsigned int>(controller_event.number);
            bool is_button_pressed = static_cast<bool>(controller_event.value);
            controller_input = handleButtonEvent(controller_input, button_id, is_button_pressed);
            break;
        }
        case JS_EVENT_AXIS:
        {
            unsigned int axis_id = static_cast<unsigned int>(controller_event.number);
            double max_axis_value = static_cast<double>(std::numeric_limits<short>::max());
            double axis_value = static_cast<double>(controller_event.value) / max_axis_value;
            controller_input = handleAxisEvent(controller_input, axis_id, axis_value);
            break;
        }
        default:
            // Ignore init events
            break;
    }

    return controller_input;
}

int Controller::read_event(int fd, struct js_event *event) {
    ssize_t bytes;

    bytes = read(fd, event, sizeof(*event));

    if (bytes == sizeof(*event))
        return 0;

    // Error, could not read full event.
    return -1;
}

size_t Controller::getNumAxes(int fd) {
    __u8 axes;
    if (ioctl(fd, JSIOCGAXES, &axes) == -1)
        return 0;

    return axes;
}

size_t Controller::getNumButtons(int fd) {
    __u8 buttons;
    if (ioctl(fd, JSIOCGBUTTONS, &buttons) == -1)
        return 0;

    return buttons;
}


//
//double Controller::foo(short val) {
//    double denom = static_cast<double>(std::numeric_limits<short>::max() - DEADZONE);
//    if(val >= 0) {
//        val = std::clamp<short>(val - DEADZONE, 0, val);
//    }else if(val < 0) {
//        val = std::clamp<short>(val + DEADZONE, val, 0);
//    }
//
//    double result = static_cast<double>(val / denom);
//    return result;
//}
