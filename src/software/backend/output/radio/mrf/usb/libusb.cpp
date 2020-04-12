#include "software/backend/output/radio/mrf/usb/libusb.h"

#include <poll.h>

#include <algorithm>
#include <cassert>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <iostream>
#include <limits>
#include <string>
#include <thread>

#include "software/logger/logger.h"

#define STALL_RETRIES 3

namespace
{
    std::thread libusb_event_thread;
    bool running;
}  // namespace

USB::Context::Context()
{
    // Init libusb
    check_fn("libusb_init", libusb_init(&context), 0);
    running = true;

    // Init event handling thread
    libusb_event_thread = std::thread([&]() {
        while (running)
            libusb_handle_events(context);

        LOG(DEBUG) << "Event thread exiting";
    });
}

USB::Context::~Context()
{
    // Terminate event thread
    running = false;

    // Close all devices
    for (auto &dev : open_devices)
    {
        dev->~DeviceHandle();
    }

    // This wakes up libusb_handle_events, now let the thread join
    libusb_event_thread.join();

    // Cleanup libusb
    libusb_exit(context);
    context = nullptr;

    LOG(DEBUG) << "Context destroyed";
}

USB::ConfigurationSetter::ConfigurationSetter(DeviceHandle &device, int configuration)
    : device(device)
{
    original_configuration = device.get_configuration();
    device.set_configuration(configuration);
}

USB::ConfigurationSetter::~ConfigurationSetter()
{
    try
    {
        device.set_configuration(original_configuration ? original_configuration : -1);
    }
    catch (const std::exception &exp)
    {
        try
        {
            LOG(WARNING) << exp.what() << std::endl;
        }
        catch (...)
        {
        }
    }
}

USB::InterfaceClaimer::InterfaceClaimer(DeviceHandle &device, int interface)
    : device(device), interface(interface)
{
    device.claim_interface(interface);
}

USB::InterfaceClaimer::~InterfaceClaimer()
{
    try
    {
        device.release_interface(interface);
    }
    catch (const std::exception &exp)
    {
        try
        {
            LOG(WARNING) << exp.what() << std::endl;
        }
        catch (...)
        {
        }
    }
}
