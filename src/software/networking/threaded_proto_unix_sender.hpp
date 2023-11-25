#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>

#include "software/networking/unix_sender.h"

template <class SendProto>
class ThreadedProtoUnixSender
{
   public:
    /**
     * Create a ThreadedProtoUnixSender
     *
     * @param unix_path The unix socket path to send on
     */
    ThreadedProtoUnixSender(const std::string& unix_path);

    ~ThreadedProtoUnixSender();

    /**
     * Sends a protobuf message to the initialized ip address and port
     * This function returns after the message has been sent.
     *
     * @param message The protobuf message to send
     */
    void sendProto(const SendProto& message);

   private:
    // The io_service that will be used to service all network requests
    boost::asio::io_service io_service;

    std::string data_buffer;
    UnixSender unix_sender;

    // The thread running the io_service in the background. This thread will run for the
    // entire lifetime of the class
    std::thread io_service_thread;
};

template <class SendProtoT>
ThreadedProtoUnixSender<SendProtoT>::ThreadedProtoUnixSender(const std::string& unix_path)
    : io_service(),
      unix_sender(io_service, unix_path),
      io_service_thread([this]() { io_service.run(); })
{
}

template <class SendProtoT>
ThreadedProtoUnixSender<SendProtoT>::~ThreadedProtoUnixSender()
{
    // Stop the io_service. This is safe to call from another thread.
    // https://stackoverflow.com/questions/4808848/boost-asio-stopping-io-service
    // This MUST be done before attempting to join the thread because otherwise the
    // io_service will not stop and the thread will not join
    io_service.stop();

    // Join the io_service_thread so that we wait for it to exit before destructing the
    // thread object. If we do not wait for the thread to finish executing, it will call
    // `std::terminate` when we deallocate the thread object and kill our whole program
    io_service_thread.join();
}

template <class SendProtoT>
void ThreadedProtoUnixSender<SendProtoT>::sendProto(const SendProtoT& message)
{
    message.SerializeToString(&data_buffer);
    unix_sender.sendString(data_buffer);
}
