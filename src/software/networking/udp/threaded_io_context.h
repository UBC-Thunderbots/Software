#pragma once

#include <boost/asio/executor_work_guard.hpp>
#include <boost/asio/io_context.hpp>
#include <future>
#include <thread>

/**
 * Owns an io_context and the thread that services its asynchronous operations.
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
     *
     */
    ~ThreadedIoContext();

    /**
     * Gets the io_context used by this service.
     *
     * @return The io_context used to service asynchronous operations
     */
    boost::asio::io_context& getIoContext();

    /**
     * Waits until handlers already queued on this context have completed.
     *
     * This must not be called from the io_context worker thread.
     */
    void waitForHandlersToDrain();

   private:
    boost::asio::io_context io_context_;
    // Keeps the io_context thread running while shared UDP objects are idle.
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
    std::thread io_context_thread_;
};
