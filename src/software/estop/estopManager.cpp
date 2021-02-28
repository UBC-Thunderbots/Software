
#include <stdio.h>
#include <string.h>

#include <thread>
#include <boost/bind/bind.hpp>
#include <boost/function.hpp>
#include <bitset>
#include "estopManager.h"


estopManager::estopManager()
    : startup(2), interval(40), timer(io_service, interval)
{
    estopState = EstopState::STOP;
}

int estopManager::setupManager() {
    //TODO: run platformio command to figure out device port

    std::cout<<"in setup manager"<<std::endl;

    estop.openPort(io_service, 9600, "/dev/ttyACM0");

    timer.async_wait(boost::bind( &estopManager::tick, this, boost::asio::placeholders::error));
    std::cout<<"starting timer io service"<<std::endl;
    io_service.run();

    return 1;
}

bool estopManager::isEstopStatePlay() {
    return estopState == EstopState::PLAY;
}

int estopManager::readEstop() {
    std::cout<<"handling estop"<<std::endl;
    boost::system::error_code ec;
    int status = estop.serialRead(&estopMsg, ESTOP_MESSAGE_SIZE, ec);

    if( status == -1 || ec.value() ==-1 || (estopMsg & ESTOP_STATUS_MASK) != ESTOP_STATUS_MASK){
        estopState = EstopState::STATUS_ERROR;
    } else if ((estopMsg & ESTOP_STATE_MASK)  == ESTOP_STATE_MASK){
        estopState = EstopState ::PLAY;
    } else{
        estopState = EstopState ::STOP;
    }

    return 0;
}

int estopManager::closeManager() {
    estop.closePort();
    return 1;
}

void estopManager::tick(const boost::system::error_code& error) {

    std::cout<<"tick"<<std::endl;

    in_destructor_mutex.lock();
    if(inDestructor){
        in_destructor_mutex.unlock();
        std::cout<<"breaking tick"<<std::endl;
        timer.cancel();
    } else {
        in_destructor_mutex.unlock();

        readEstop();
        std::cout<<"finished readEstop"<<std::endl;

        // Reschedule the timer for interval seconds in the future:
        timer.expires_at(timer.expires_at() + interval);
        // Posts the timer event
        timer.async_wait(boost::bind(&estopManager::tick, this, boost::asio::placeholders::error));
        std::cout<<"beginning readEstop"<<std::endl;
    }

    return;
}

int estopManager::startManager() {
    estop_thread = std::thread(boost::bind(&estopManager::setupManager, this));

    std::cout<<"setup thread started"<<std::endl;

    return 1;
}

estopManager::~estopManager()
{
    std::cout<<"destructor called"<<std::endl;

    in_destructor_mutex.lock();
    inDestructor = true;
    in_destructor_mutex.unlock();
    estop.closePort();

    // We must wait for the thread to stop, as if we destroy it while it's still
    // running we will segfault
    estop_thread.join();
}
