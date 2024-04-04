#include "software/networking/unix/threaded_unix_sender.h"

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>

ThreadedUnixSender::ThreadedUnixSender(const std::string& unix_socket_path)
    : io_service(),
      unix_sender(io_service, unix_socket_path),
      io_service_thread([this]() { io_service.run(); })
{
}

ThreadedUnixSender::~ThreadedUnixSender()
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

void ThreadedUnixSender::sendString(const std::string& message)
{
    unix_sender.sendString(message);
}
