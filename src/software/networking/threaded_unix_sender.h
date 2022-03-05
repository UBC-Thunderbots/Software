#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>

#include "software/networking/unix_sender.h"

class ThreadedUnixSender
{
   public:
    /**
     * Creates a ThreadedUnixSender that sends a string over the unix socket path.
     *
     * @param unix_socket_path The path to the unix socket
     */
    ThreadedUnixSender(const std::string& unix_socket_path);
    ~ThreadedUnixSender();

    /**
     * Sends the string over the unix socket
     *
     * @param message The string message to send
     */
    void sendString(const std::string& message);

   private:
    // The io_service that will be used to service all network requests
    boost::asio::io_service io_service;
    UnixSender unix_sender;

    // The thread running the io_service in the background. This thread will run
    // for the entire lifetime of the class
    std::thread io_service_thread;
};
