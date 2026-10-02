#pragma once

#include <boost/asio/executor_work_guard.hpp>
#include <boost/asio/io_context.hpp>
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
     * All listeners and senders sharing this context must be closed before this object
     * is destroyed, otherwise their pending handlers will never be serviced.
     */
    ~ThreadedIoContext();

    /**
     * Gets the io_context used by this service.
     *
     * @return The io_context used to service asynchronous operations
     */
    boost::asio::io_context& getIoContext();

    /**
     * Returns whether the calling thread is the thread that runs this context's
     * io_context event loop. Blocking operations that wait on this thread must not be
     * called from it.
     *
     * @return true if the calling thread is the io_context worker thread
     */
    bool isInWorkerThread() const;

   private:
    boost::asio::io_context io_context_;
    // Keeps the io_context thread running while shared UDP objects are idle.
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
    std::thread io_context_thread_;
};
