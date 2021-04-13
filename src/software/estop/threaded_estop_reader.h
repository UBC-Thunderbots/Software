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
     * @param startup_ms time in milliseconds to wait before first read
     * @param interval_ms time in milliseconds between reads
     * @param uart_reader the UART device acting as source of estop values
     */
    ThreadedEstopReader(int startup_ms, int interval_ms,
                        std::unique_ptr<UartCommunication> uart_reader);

    ~ThreadedEstopReader();

    /*
     * returns the state of estop
     */
    EstopState getEstopState();


   private:
    /*
     * reads value of Estop and updates estop state
     */
    void readEstop();

    /*
     * handler method that is called every time the timer expires.
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
     * any other message received is considred a STATUS_ERROR
     */
    static constexpr int ESTOP_MESSAGE_SIZE = 1;

    boost::asio::io_service io_service;

    boost::posix_time::milliseconds startup_ms;
    boost::posix_time::milliseconds interval_ms;
    boost::asio::deadline_timer timer;

    // thread that will periodically pull values from the buffer
    std::thread estop_thread;

    std::mutex in_destructor_mutex;
    bool in_destructor = false;

    std::vector<unsigned char> estop_msg;
    std::unique_ptr<UartCommunication> uart_reader;
    enum EstopState estop_state;
};
