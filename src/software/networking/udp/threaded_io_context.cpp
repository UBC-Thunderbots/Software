#include "software/networking/udp/threaded_io_context.h"

#include <boost/asio/executor_work_guard.hpp>
#include <boost/asio/post.hpp>

ThreadedIoContext::ThreadedIoContext()
    : work_guard_(boost::asio::make_work_guard(io_context_)),
      io_context_thread_([this]() { io_context_.run(); })
{
}

ThreadedIoContext::~ThreadedIoContext()
{
    // Stop the io_context. This is safe to call from another thread.
    // This MUST be done before attempting to join the thread because otherwise the
    // io_context will not stop and the thread will not join.
    io_context_.stop();

    // Join the io_context thread so that we wait for it to exit before destructing
    // the thread object. If we do not wait for the thread to finish executing, it will
    // call std::terminate when the thread object is destroyed.
    io_context_thread_.join();
}

boost::asio::io_context& ThreadedIoContext::getIoContext()
{
    return io_context_;
}

void ThreadedIoContext::waitForHandlersToDrain()
{
    std::promise<void> completion;
    auto completion_future = completion.get_future();

    boost::asio::post(io_context_, [&completion] { completion.set_value(); });
    completion_future.wait();
}
