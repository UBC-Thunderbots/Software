#include "libusb.h"

#include <glibmm/main.h>
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

#include "util/logger/init.h"

#define STALL_RETRIES 3

namespace
{
    Glib::RefPtr<Glib::MainLoop> glib_main_loop(Glib::MainLoop::create());
}

USB::Context::Context()
{
    // Spin up glib main loop.
    std::thread glib_thread([&]() { glib_main_loop->run(); });
    check_fn("libusb_init", libusb_init(&context), 0);
}

USB::Context::~Context()
{
    libusb_exit(context);
    context = nullptr;

    glib_main_loop->quit();
}

void USB::Context::add_pollfd(int fd, short events)
{
    auto old = fd_connections.find(fd);
    if (old != fd_connections.end())
    {
        old->second.disconnect();
    }
    Glib::IOCondition cond = static_cast<Glib::IOCondition>(0);
    if (events & POLLIN)
    {
        cond |= Glib::IO_IN;
    }
    if (events & POLLOUT)
    {
        cond |= Glib::IO_OUT;
    }
    fd_connections[fd] = Glib::signal_io().connect(
        sigc::bind_return(sigc::hide(sigc::mem_fun(this, &Context::handle_usb_fds)),
                          true),
        fd, cond);
}

void USB::Context::remove_pollfd(int fd)
{
    auto i = fd_connections.find(fd);
    if (i != fd_connections.end())
    {
        i->second.disconnect();
        fd_connections.erase(i);
    }
}

void USB::Context::handle_usb_fds()
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

void USB::usb_context_pollfd_add_trampoline(int fd, short events, void *user_data)
{
    static_cast<Context *>(user_data)->add_pollfd(fd, events);
}

void USB::usb_context_pollfd_remove_trampoline(int fd, void *user_data)
{
    static_cast<Context *>(user_data)->remove_pollfd(fd);
}
