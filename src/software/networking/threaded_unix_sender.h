#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>

#include "software/networking/unix_sender.h"

class ThreadedUnixSender
{
   public:
    /**
     * TODO
     */
    ThreadedUnixSender(const std::string& unix_socket_path);

    ~ThreadedUnixSender();

    /**
     * TODO
     */
    void sendString(const std::string& message);

   private:
    // The io_service that will be used to service all network requests
    boost::asio::io_service io_service;
    UnixSender unix_sender;
    // The thread running the io_service in the background. This thread will run for the
    // entire lifetime of the class
    std::thread io_service_thread;
    std::string data_buffer_;
};
