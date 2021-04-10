
#include <stdio.h>
#include <string.h>
#include "estop_manager.h"
#include "threaded_estop_reader.h"
#include "boost_uart_communication.h"
#include "uart_communication.h"
#include "software/logger/logger.h"


EstopManager::EstopManager() : estop_state(EstopState::PLAY), estop_reader(nullptr)
{}

bool EstopManager::isEstopStatePlay() {
    if(!is_estop_polling){
        return estop_state == EstopState::PLAY;
    }

    EstopState new_state = estop_reader->getEstopState();

    if(new_state != estop_state){
        LOG(INFO) << "estop state has changed from "<<estop_state<<" to  "<< new_state;
    }

    estop_state = new_state;

    switch(estop_state) {
        case EstopState::PLAY:
            status_error_counter = 0;
            return true;
        case EstopState::STOP:
            status_error_counter = 0;
            return false;
        case EstopState::STATUS_ERROR:
            if (status_error_counter >= STATUS_ERROR_THRESHOLD) {
                LOG(FATAL) << "Estop state could not be determined, exiting";
            } else {
                status_error_counter++;
                LOG(WARNING) << "Estop state could not be determined";
            }
            return false;
        default:
            return false;
    }

}

void EstopManager::startEstopContinousPolling(int startup_time_ms, int polling_interval_ms, std::unique_ptr<UartCommunication> uart_reader) {

    if(!is_estop_polling){
        estop_reader = std::make_unique<ThreadedEstopReader>(startup_time_ms, polling_interval_ms, std::move(uart_reader));
        is_estop_polling = true;
    }
}

bool EstopManager::isEstopPolling() const {
    return is_estop_polling;
}

