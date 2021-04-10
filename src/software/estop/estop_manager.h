#pragma once

#include "uart_communication.h"
#include <iostream>
#include <boost/asio.hpp>
#include <thread>
#include <mutex>
#include "threaded_estop_reader.h"


class EstopManager
{
   public:
    EstopManager();

    /**
    * returns whether estop is in PLAY state
    */
    bool isEstopStatePlay();

    /**
     * Starts estop continous polling if it is not currently started
     * @param startup_time_ms time in milliseconds to wait before polling for first estop value
     * @param polling_interval_ms time in milliseconds between consecutive estop polls
     * @param uart_reader The UART connection that will be used to read estop from
     */
    void startEstopContinousPolling(int startup_time_ms, int polling_interval_ms, std::unique_ptr<UartCommunication> uart_reader);

    /**
     *
     * @return whether Estop polling process has begun
     */
    bool isEstopPolling() const;

private:

    /*
     * The number of consecutive unknown messages we can receive before throwing an error
     */
    static int constexpr STATUS_ERROR_THRESHOLD = 10;

    enum EstopState estop_state;
    std::unique_ptr<ThreadedEstopReader> estop_reader;
    bool is_estop_polling = false;
    int status_error_counter = 0;

};



