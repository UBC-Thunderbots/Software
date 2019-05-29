#include "libusb.h"

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

#include "util/logger/init.h"

#define STALL_RETRIES 3

USB::Context::Context()
{
    check_fn("libusb_init", libusb_init(&context), 0);
}

USB::Context::~Context()
{
    libusb_exit(context);
    context = nullptr;
}

void USB::Context::handle_usb_events()
{
    timeval tv = {0, 0};
    check_fn("libusb_handle_events_timeout", libusb_handle_events_timeout(context, &tv),
             0);
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
