#pragma once

#include <boost/asio/executor_work_guard.hpp>
#include <boost/asio/io_context.hpp>
#include <thread>

class ThreadedIoContext
{
   public:
    ThreadedIoContext();

    /**
     * The context must outlive all UDP objects that use it.
     */
    ~ThreadedIoContext();

    boost::asio::io_context& getIoContext();

    /**
     * True when called on the worker thread; blocking close operations must not wait
     * on that thread.
     */
    bool isInWorkerThread() const;

   private:
    boost::asio::io_context io_context_;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
    std::thread io_context_thread_;
};
