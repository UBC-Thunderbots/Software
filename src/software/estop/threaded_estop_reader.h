#pragma once

#pragma once


#include <boost/asio.hpp>
#include <iostream>
#include <mutex>
#include <thread>

#include "shared/constants.h"
#include "software/uart/uart_communication.h"
#include "software/util/make_enum/make_enum.h"

// enum that represents the possible states of estop
MAKE_ENUM(EstopState, PLAY, STOP, STATUS_ERROR);

/*
 * class that reads status of estop at periodic intervals of time
 */
class ThreadedEstopReader
{
   public:
    /**
     * creates and starts a threadedEstopReader with the given parameters to read estop
     * value every 5 milliseconds
     * @param uart_reader the UART device acting as source of estop values
     */
    ThreadedEstopReader(std::unique_ptr<UartCommunication> uart_reader);
    ~ThreadedEstopReader();

    /**
     * Returns true if estop is in play state and false otherwise
     */
    bool isEstopPlay();


   private:
    /*
     * handler method that is called every time the timer expires and a new read is
     * requested
     */
    void tick(const boost::system::error_code&);

    /*
     * method that initiates timer
     */
    void continousRead();

    // In the case where we read an unknown message (not PLAY or STOP) we try again this
    // number of times
    static constexpr unsigned int MAXIMUM_CONSECUTIVE_STATUS_ERROR = 5;

    // time between consecutive reads
    static constexpr int INTERVAL_BETWEEN_READS_MS = 5;

    // thread that will periodically pull values from the buffer
    std::thread estop_thread;
    std::atomic_bool in_destructor = false;

    // current state of estop
    std::atomic<EstopState> estop_state;

    // tracks the number of unknown messages received in a row
    unsigned int num_consecutive_status_error = 0;

    // Time between reads
    unsigned int regular_read_interval_ms;


    // boost construct for managing io operations
    boost::asio::io_service io_service;

    // timer that expires after every specified interval of time
    boost::asio::deadline_timer timer;

    // the UART device acting as source of estop values
    std::unique_ptr<UartCommunication> uart_reader;
};
