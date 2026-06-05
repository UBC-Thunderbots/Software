#pragma once

#include <atomic>
#include <thread>

#include "proto/power_frame_msg.pb.h"
#include "software/embedded/services/uart_communicator.h"

extern "C"
{
#include "proto/power_frame_msg.pb.h"
}

class PowerService
{
   public:
    /**
     * Service that interacts with the power board.
     * Connects to the UartCommunicator
     */
    PowerService(std::shared_ptr<UartCommunicator> uart);

    /**
     * When the power service is polled it sends the given power control msg and
     * returns the latest power status
     *
     * @param control The power control msg to send
     * @return the latest power status
     */
    TbotsProto::PowerStatus poll(const TbotsProto::PowerControl& control,
                                 double kick_coeff, int kick_constant, int chip_constant);

   private:
    std::shared_ptr<UartCommunicator> uart_;
};
