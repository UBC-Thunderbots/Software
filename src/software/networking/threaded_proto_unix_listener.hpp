#pragma once

#include "software/networking/proto_unix_listener.hpp"

/**
 * A threaded listener that receives serialized ReceiveProtoT protos over the network
 */
template <class ReceiveProtoT>
class ThreadedProtoUnixListener
{
   public:
    /*
     * Listens for packets over the provided unix socket and triggers the
     * receive_callback on a new message.
     *
     * @param unix_path The unix path to connect to
     * @param receive_callback The callback to trigger on a new packet
     */
    ThreadedProtoUnixListener(const std::string& unix_path,
                              std::function<void(ReceiveProtoT&)> receive_callback);

    ~ThreadedProtoUnixListener();

   private:
    // The io_service that will be used to service all network requests
    boost::asio::io_service io_service;
    // The thread running the io_service in the background. This thread will run for the
    // entire lifetime of the class
    std::thread io_service_thread;
    std::function<void(ReceiveProtoT&)> receive_callback_;
    ProtoUnixListener<ReceiveProtoT> unix_listener;
};

template <class ReceiveProtoT>
ThreadedProtoUnixListener<ReceiveProtoT>::ThreadedProtoUnixListener(
    const std::string& unix_path, std::function<void(ReceiveProtoT&)> receive_callback)
    : io_service(), unix_listener(io_service, unix_path, receive_callback)
{
    // start the thread to run the io_service in the background
    io_service_thread = std::thread([this]() { io_service.run(); });
}

template <class ReceiveProtoT>
ThreadedProtoUnixListener<ReceiveProtoT>::~ThreadedProtoUnixListener()
{
    // Stop the io_service. This is safe to call from another thread.
    // https://stackoverflow.com/questions/4808848/boost-asio-stopping-io-service
    // This MUST be done before attempting to join the thread because otherwise the
    // io_service will not stop and the thread will not join
    io_service.stop();

    // Join the io_service_thread so that we wait for it to exit before destructing the
    // thread object. If we do not wait for the thread to
    // finish executing, it will call
    // `std::terminate` when we deallocate the thread object and kill our whole program
    io_service_thread.join();
}
