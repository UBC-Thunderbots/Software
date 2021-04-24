#pragma once

#pragma once


#include <boost/asio.hpp>
#include <iostream>
#include <mutex>
#include <thread>

#include "software/util/make_enum/make_enum.h"
#include "uart_communication.h"

// enum that represents the possible states of estop
MAKE_ENUM(EstopState, PLAY, STOP, STATUS_ERROR);

/*
 * class that reads status of estop at periodic intervals of time
 */
class ThreadedEstopReader
{
   public:
    /**
     * creates and starts a threadedEstopReader with the given parameters
     * @param uart_reader the UART device acting as source of estop values
     * @param startup_time_ms time in milliseconds to wait before making first read
     * @param interval_ms periodic time in milliseconds between reads
     */
    ThreadedEstopReader(std::unique_ptr<UartCommunication> uart_reader,
                        unsigned int startup_time_ms,
                        unsigned int regular_interval_time_ms);

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

    /*
     * each estop message is one byte and is defined as follows
     * bit 0 (least significant bit): estop state, a value of 1 is play, 0 is stop
     * bit 1-7: set to 0
     * any other message received is considered a EstopState::STATUS_ERROR
     */
    static constexpr int ESTOP_MESSAGE_SIZE_BYTES = 1;
    static constexpr unsigned char ESTOP_PLAY     = 1;
    static constexpr unsigned char ESTOP_STOP     = 0;

    // In the case where we read an unknown message (not PLAY or STOP) we try again this
    // number of times
    static constexpr unsigned int MAXIMUM_CONSECUTIVE_STATUS_ERROR = 5;

    // In the case where we read an unknown message (not PLAY or STOP) we try
    // again with this time between reads
    static constexpr int INTERVAL_BETWEEN_ERROR_READS_MS = 5;

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
