#include "software/networking/udp/threaded_io_context.h"

#include <boost/asio/executor_work_guard.hpp>

ThreadedIoContext::ThreadedIoContext()
    : work_guard_(boost::asio::make_work_guard(io_context_)),
      io_context_thread_([this]() { io_context_.run(); })
{
}

ThreadedIoContext::~ThreadedIoContext()
{
    io_context_.stop();
    io_context_thread_.join();
}

boost::asio::io_context& ThreadedIoContext::getIoContext()
{
    return io_context_;
}
