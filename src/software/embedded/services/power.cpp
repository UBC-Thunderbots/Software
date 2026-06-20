#include "software/embedded/services/power.h"

#include <boost/filesystem.hpp>
#include <cstdint>

#include "proto/power_frame_msg.pb.h"

PowerService::PowerService(const double kick_coefficient, const int kick_constant,
                           const int chip_constant)
    : kick_coefficient_(kick_coefficient),
      kick_constant_(kick_constant),
      chip_constant_(chip_constant),
      power_pulse_command_(
          createNanoPbPowerPulseControl(TbotsProto::PowerControl(), 0.0, 0, 0)),
      dribbler_command_(createNanoPbDribblerControl(0))
{
    if (!boost::filesystem::exists(DEVICE_SERIAL_PORT))
    {
        throw std::runtime_error("No serial port exists");
    }

    this->uart_ = std::make_unique<BoostUartCommunication>(BAUD_RATE, DEVICE_SERIAL_PORT);
    this->read_thread_ = std::thread([&] { continuousRead(); });
}

PowerService::~PowerService()
{
    is_running_ = false;
    read_thread_.join();
}

void PowerService::continuousRead()
{
    while (is_running_)
    {
        tick();
    }
}

void PowerService::tick()
{
    std::optional<TbotsProto_PowerStatus> power_status = readPowerStatus();
    if (power_status.has_value())
    {
        power_status_ = power_status.value();
    }

    const TbotsProto_PowerPulseControl power_pulse_command =
        power_pulse_command_.load(std::memory_order_relaxed);

    const TbotsProto_DribblerControl dribbler_command =
        dribbler_command_.load(std::memory_order_relaxed);

    writePowerFrame(createUartFrame(power_pulse_command));
    writePowerFrame(createUartFrame(dribbler_command));
}

std::optional<TbotsProto_PowerStatus> PowerService::readPowerStatus() const
{
    std::vector<uint8_t> power_status;
    try
    {
        uart_->flushSerialPort(UartCommunication::FlushType::flush_receive);
        power_status = uart_->serialRead(READ_BUFFER_SIZE);
    }
    catch (std::exception& e)
    {
        LOG(FATAL) << "ESP32 has disconnected. Power service has crashed" << e.what();
    }

    TbotsProto_PowerFrame status_frame = TbotsProto_PowerFrame_init_default;
    if (!unmarshalUartPacket(power_status, status_frame))
    {
        LOG(WARNING) << "Unmarshalling power frame failed";
        return std::nullopt;
    }

    return status_frame.power_msg.power_status;
}

void PowerService::writePowerFrame(const TbotsProto_PowerFrame& frame) const
{
    try
    {
        const std::vector<uint8_t> power_frame_buffer = marshallUartPacket(frame);
        uart_->flushSerialPort(UartCommunication::FlushType::flush_send);
        if (!uart_->serialWrite(power_frame_buffer))
        {
            LOG(WARNING) << "Writing power frame into serial port failed";
        }
    }
    catch (std::exception& e)
    {
        LOG(FATAL) << "ESP32 has disconnected. Power service has crashed" << e.what();
    }
}

void PowerService::poll(const TbotsProto::DirectControlPrimitive& primitive,
                        TbotsProto::RobotStatus& robot_status)
{
    power_pulse_command_ = createNanoPbPowerPulseControl(
        primitive.power_control(), kick_coefficient_, kick_constant_, chip_constant_);

    *(robot_status.mutable_power_status()) = *createTbotsPowerStatus(power_status_);

    const uint32_t dribbler_rpm =
        std::abs(primitive.motor_control().dribbler_speed_rpm());
    dribbler_command_ = createNanoPbDribblerControl(dribbler_rpm);

    TbotsProto::DribblerStatus dribbler_status;
    dribbler_status.set_dribbler_rpm(static_cast<float>(dribbler_rpm));
    dribbler_status.set_enabled(true);
    *(robot_status.mutable_motor_status()->mutable_dribbler()) = dribbler_status;
}
