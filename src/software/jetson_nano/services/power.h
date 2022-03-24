#pragma once

#include <boost/asio.hpp>

#include "software/jetson_nano/services/service.h"
#include "software/logger/logger.h"
#include "software/uart/boost_uart_communication.h"

// TODO(#2537): change to protobuf/move to different file with addition of thunderloop
struct GenevaMotorCommand
{
    float angle_deg;
    float rotation_speed;
};

struct ChickerCommand
{
    bool autochip;
    bool autokick;
    float chip_distance;
    float kick_distance;
    float pulse_width_sec;
};

struct PowerCommand
{
    ChickerCommand chicker;
    GenevaMotorCommand geneva;
};


struct PowerStatus
{
    bool breakbeam_tripped;
    bool flyback_fault;
    float battery_voltage;
    float current_draw;
    float geneva_angle;
    float high_voltage_measurement_volts;
};

class PowerService : public Service
{
   public:
    /**
     * Service that interacts with the power board.
     * Opens all the required ports and maintains them until destroyed.
     */
    PowerService();
    ~PowerService() = default;
    void start() override;
    void stop() override;
    /**
     * When the power service is polled it sends the given power command and
     * returns the latest power status
     *
     * @param command The power command to send
     * @return the latest power status
     */
    std::unique_ptr<PowerStatus> poll(const PowerCommand& command);

   private:
    boost::asio::io_service io_service;
    std::unique_ptr<BoostUartCommunication> uart;
    PowerStatus status;

    // Constants
    const size_t READ_BUFFER_SIZE;
    const unsigned int BAUD_RATE         = 115200;
    const std::string DEVICE_SERIAL_PORT = "/dev/ttyTHS1";
};
