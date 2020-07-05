#pragma once

#include <boost/asio.hpp>
#include <string>
#include <thread>

#include "software/networking/multicast_listener.h"

template <class ListenerT, typename CallBackFunctionArgT>
class ThreadedNetworkListener
{
   public:
    /**
     * Creates a ThreadedNetworkListener that asyncrhonously listens for data on the
     * network using a given listener that passes data of a specified type back
     * in a given callback function
     *
     * @param ip_address The ip address to listen to
     *  example IPv4: 192.168.0.2
     *  example IPv6: ff02::c3d0:42d2:bb8%wlp4s0 (the interface is specified after %)
     * @param port The port on which to listen
     * @param receive_callback The function to call with each new value received over
     *                         the network
     */
    ThreadedNetworkListener(const std::string& ip_address, unsigned short port,
                            std::function<void(CallBackFunctionArgT&)> receive_callback);

    ~ThreadedNetworkListener();

   private:
    // The io_service that will be used to service all network requests
    boost::asio::io_service io_service;
    // The thread running the io_service in the background. This thread will run for the
    // entire lifetime of the class
    std::thread io_service_thread;

    // The listener that will use the io_service to receive data over the network and
    // forward it to the given callback function
    ListenerT listener;
};

#include "software/networking/threaded_network_listener.tpp"
