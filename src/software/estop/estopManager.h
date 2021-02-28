#pragma once

#include "software/util/make_enum/make_enum.h"
#include "arduinoSerialWrapper.h"
#include <iostream>
#include <boost/asio.hpp>
#include <thread>
#include <mutex>

MAKE_ENUM(EstopState, PLAY, STOP, STATUS_ERROR);

/*
 * class that interfaces with Arduino to read status of estop at specified intervals of time
 */
class estopManager
{
   public:
    estopManager();
    ~estopManager();
    bool isEstopStatePlay();
    int closeManager();
    int startManager();

   private:
    int readEstop();
    int setupManager();
    void tick(const boost::system::error_code&);


    /*
     * each estop message is one byte and is defined as follows
     * bit 0: estop state, a value of 1 is play, 0 is stop
     * bit 1-3: not used
     * bit 4-7: arduino status. a value of 0b1010 indicates status ok, anything else is error status
     */
    static constexpr unsigned char ESTOP_STATE_MASK = 1;
    static constexpr unsigned char ESTOP_STATUS_MASK = 0b10100000;
    static constexpr int ESTOP_MESSAGE_SIZE = 1;

    boost::asio::io_service io_service;

    boost::posix_time::seconds startup;  // 2 second
    boost::posix_time::milliseconds interval;  // 40ms
    boost::asio::deadline_timer timer;

    // This is the thread that will periodically pull values from the buffer
    std::thread estop_thread;
    std::mutex in_destructor_mutex;
    bool inDestructor = false;

    arduinoSerialWrapper estop;

    unsigned char estopMsg;
    enum EstopState estopState;

};



