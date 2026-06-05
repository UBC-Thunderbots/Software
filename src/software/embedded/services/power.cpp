#include "software/embedded/services/power.h"

#include <boost/filesystem.hpp>
#include <cstdint>

#include "proto/power_frame_msg.pb.h"

PowerService::PowerService(std::shared_ptr<UartCommunicator> uart) : uart_(uart) {}

TbotsProto::PowerStatus PowerService::poll(const TbotsProto::PowerControl& command,
                                           double kick_coeff, int kick_constant,
                                           int chip_constant)
{
    return uart_->sendChipKickCommand(command, kick_coeff, kick_constant, chip_constant);
}
