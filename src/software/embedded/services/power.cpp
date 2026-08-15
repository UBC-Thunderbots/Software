#include "software/embedded/services/power.h"

#include <Tracy.hpp>
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
      dribbler_command_(createNanoPbDribblerControl(0)),
      dmesg_file_(KERNEL_RING_BUFFER_LOG_PATH),
      cpu_temp_file_(CPU_TEMP_FILE_PATH)
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
    ZoneNamedN(_tracy_power_service_poll, "Thunderloop: Poll PowerService", true);

    const auto poll_start = std::chrono::steady_clock::now();

    updatePowerControlAndStatus(primitive, robot_status);
    updateErrorCodes(primitive, robot_status);
    updateChickerStatus(primitive, robot_status);
    updateDribblerStatus(primitive, robot_status);

    const auto poll_end    = std::chrono::steady_clock::now();
    using Millis           = std::chrono::duration<double, std::milli>;
    const Millis poll_time = std::chrono::duration_cast<Millis>(poll_end - poll_start);

    robot_status.mutable_thunderloop_status()->set_power_service_poll_time_ms(
        poll_time.count());
}

void PowerService::updatePowerControlAndStatus(
    const TbotsProto::DirectControlPrimitive& direct_control,
    TbotsProto::RobotStatus& robot_status)
{
    power_pulse_command_ =
        createNanoPbPowerPulseControl(direct_control.power_control(), kick_coefficient_,
                                      kick_constant_, chip_constant_);

    const TbotsProto::PowerStatus power_status = *createTbotsPowerStatus(power_status_);
    *(robot_status.mutable_power_status())     = power_status;
}

void PowerService::updateErrorCodes(
    const TbotsProto::DirectControlPrimitive& direct_control,
    TbotsProto::RobotStatus& robot_status)
{
    const TbotsProto::PowerStatus& power_status = robot_status.power_status();

    if (!isPowerSupplyStable())
    {
        robot_status.mutable_error_code()->Add(
            TbotsProto::ErrorCode::UNSTABLE_POWER_SUPPLY);
    }

    if (power_status.battery_voltage() <= BATTERY_WARNING_VOLTAGE)
    {
        robot_status.mutable_error_code()->Add(TbotsProto::ErrorCode::LOW_BATTERY);
    }

    if (power_status.capacitor_voltage() >= MAX_CAPACITOR_VOLTAGE)
    {
        robot_status.mutable_error_code()->Add(TbotsProto::ErrorCode::HIGH_CAP);
    }
}

void PowerService::updateChickerStatus(
    const TbotsProto::DirectControlPrimitive& direct_control,
    TbotsProto::RobotStatus& robot_status)
{
    const TbotsProto::PowerControl_ChickerControl& chicker_control =
        direct_control.power_control().chicker();

    if (chicker_control.has_kick_speed_m_per_s() ||
        chicker_control.auto_chip_or_kick().has_autokick_speed_m_per_s())
    {
        last_time_kicker_fired_ = std::chrono::steady_clock::now();
    }
    else if (chicker_control.has_chip_distance_meters() ||
             chicker_control.auto_chip_or_kick().has_autochip_distance_meters())
    {
        last_time_chipper_fired_ = std::chrono::steady_clock::now();
    }

    using Millis   = std::chrono::duration<double, std::milli>;
    const auto now = std::chrono::steady_clock::now();

    robot_status.mutable_chipper_kicker_status()->set_ms_since_kicker_fired(
        std::chrono::duration_cast<Millis>(now - last_time_kicker_fired_).count());

    robot_status.mutable_chipper_kicker_status()->set_ms_since_chipper_fired(
        std::chrono::duration_cast<Millis>(now - last_time_chipper_fired_).count());
}

void PowerService::updateDribblerStatus(
    const TbotsProto::DirectControlPrimitive& direct_control,
    TbotsProto::RobotStatus& robot_status)
{
    const uint32_t dribbler_rpm =
        std::abs(direct_control.motor_control().dribbler_speed_rpm());
    dribbler_command_ = createNanoPbDribblerControl(dribbler_rpm);

    TbotsProto::DribblerStatus& dribbler_status =
        *(robot_status.mutable_motor_status()->mutable_dribbler());
    dribbler_status.set_dribbler_rpm(static_cast<float>(dribbler_rpm));
    dribbler_status.set_enabled(true);
}

bool PowerService::isPowerSupplyStable()
{
    if (!dmesg_file_.is_open())
    {
        LOG(WARNING) << "Cannot read " << KERNEL_RING_BUFFER_LOG_PATH
                     << ". Do you have permission?";

        // Chances are that power is stable, we just can't read dmesg logs due to
        // incorrect permissions.
        return true;
    }

    std::string line;
    while (std::getline(dmesg_file_, line))
    {
        constexpr std::string_view OC_ALARM = "soctherm: OC ALARM 0x00000001";
        if (line.find(OC_ALARM) != std::string::npos)
        {
            return false;
        }
    }

    // getline() reached EOF; clear the EOF state so we can
    // read data appended to the file on the next call.
    dmesg_file_.clear();

    return true;
}

double PowerService::getCpuTemperature()
{
    if (!cpu_temp_file_.is_open())
    {
        LOG(WARNING) << "Could not open CPU temperature file";
        return 0.0;
    }

    std::string cpu_temp_str;
    std::getline(cpu_temp_file_, cpu_temp_str);
    cpu_temp_file_.close();

    // The temperature returned is in milli-Celsius, convert to Celsius
    return std::stod(cpu_temp_str) / 1000.0;
}
