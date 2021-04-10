
#include <cstdio>
#include <cstring>

#include <thread>
#include <boost/bind/bind.hpp>
#include <boost/function.hpp>
#include <utility>
#include "threaded_estop_reader.h"
#include "software/logger/logger.h"


ThreadedEstopReader::ThreadedEstopReader(int startup_ms, int interval_ms, std::unique_ptr<UartCommunication> uart_reader)
        :  startup_ms(startup_ms), interval_ms(interval_ms), timer(io_service, this->interval_ms), estopMsg(0),  uart_reader(std::move(uart_reader)), estopState(EstopState::STOP)
{
    estop_thread = std::thread(boost::bind(&ThreadedEstopReader::continousRead, this));
}

void ThreadedEstopReader::continousRead() {
    timer.async_wait(boost::bind( &ThreadedEstopReader::tick, this, boost::asio::placeholders::error));
    io_service.run();
}

EstopState ThreadedEstopReader::getEstopState() {
    return estopState;
}

void ThreadedEstopReader::readEstop() {
    estopMsg = uart_reader->serialRead(ESTOP_MESSAGE_SIZE);

    if(estopMsg.at(0) == 1){
        estopState = EstopState ::PLAY;
    } else if (estopMsg.at(0) == 0){
        estopState = EstopState::STOP;
    } else{
        estopState = EstopState::STATUS_ERROR;
        LOG(WARNING)<<"read unexpected estop message";
    }
}

void ThreadedEstopReader::tick(const boost::system::error_code& error)
{

    in_destructor_mutex.lock();
    if(inDestructor){
        in_destructor_mutex.unlock();
        timer.cancel();
    } else {
        in_destructor_mutex.unlock();

        readEstop();

        // Reschedule the timer for interval seconds in the future:
        timer.expires_at(timer.expires_at() + interval_ms);
        // Posts the timer event
        timer.async_wait(boost::bind(&ThreadedEstopReader::tick, this, boost::asio::placeholders::error));
    }
}


ThreadedEstopReader::~ThreadedEstopReader()
{
    in_destructor_mutex.lock();
    inDestructor = true;
    in_destructor_mutex.unlock();

    // We must wait for the thread to stop, as if we destroy it while it's still
    // running we will segfault
    estop_thread.join();
}

