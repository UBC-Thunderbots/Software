#pragma once

#include <boost/asio/io_context.hpp>
#include <boost/asio/executor_work_guard.hpp>
#include <thread>

/**
 * Owns an io_context and the thread that services its asynchronous operations.
 *
 * Multiple UDP senders and listeners can share one instance of this class. The
 * service remains alive until all objects holding its shared ownership have
 * been destroyed.
 */
class ThreadedIoContext
{
   public:
    /**
     * Creates and starts the thread that runs the io_context.
     */
    ThreadedIoContext();

    /**
     * Stops the io_context and joins its service thread.
     */
    ~ThreadedIoContext();

    /**
     * Gets the io_context used by this service.
     *
     * @return The io_context used to service asynchronous operations
     */
    boost::asio::io_context& getIoContext();

   private:
    boost::asio::io_context io_context_;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
    std::thread io_context_thread_;
};
