#pragma once

#include <boost/asio.hpp>
#include <thread>
#include <atomic>

#include "software/jetson_nano/services/service.h"
#include "software/uart/boost_uart_communication.h"
#include "software/logger/logger.h"

// Move different file with powerloop
struct GenevaMotorCommand {
    float angle_deg;
    float rotation_speed;
};

struct ChickerCommand {
    bool  autochip;
    bool  autokick;
    float chip_distance;
    float kick_distance;
    float pulse_width_sec;
};

struct PowerCommand
{
    ChickerCommand     chicker;
    GenevaMotorCommand geneva;
};


struct PowerStatus
{
    bool  breakbeam_tripped;
    bool  flyback_fault;
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
    ~PowerService();
    void start() override;
    void stop() override;
    /**
     * When the power service is polled it sends the given power and
     * returns the latest power status
     *
     * @param command The
     * @return
     */
    std::unique_ptr<PowerStatus> poll(const PowerCommand & command);
   private:
    /**
     * Handler method called every time the timer expires a new read is requested
     */
    void tick(const boost::system::error_code&);
    /**
     * Initiates timer for serial reading
     */
    void continuousRead();

    boost::asio::io_service io_service;
    std::unique_ptr<BoostUartCommunication> uart;

    std::thread read_thread;
    boost::asio::deadline_timer timer;
    std::atomic_bool in_destructor = false;
    std::atomic<PowerStatus> status;

    // Constants
    const size_t READ_BUFFER_SIZE;
    static constexpr unsigned int BAUD_RATE                 = 115200;
    const std::string DEVICE_SERIAL_PORT                    = "/dev/ttyTHS1";
    static constexpr int MAX_WRITE_ATTEMPTS                 = 3;
    static constexpr unsigned int INTERVAL_BETWEEN_READS_MS = 5;
    static constexpr unsigned int STARTUP_TIME_MS           = 0;
};