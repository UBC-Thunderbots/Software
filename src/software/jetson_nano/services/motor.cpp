
#include "software/jetson_nano/services/motor.h"

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <limits.h>
#include <linux/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <malloc.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/mman.h>      // Needed for mlockall()
#include <sys/resource.h>  // needed for getrusage
#include <sys/time.h>      // needed for getrusage
#include <unistd.h>        // needed for sysconf(int name);

#include <bitset>

#include "proto/tbots_software_msgs.pb.h"
#include "shared/constants.h"
#include "software/logger/logger.h"
#include "software/util/scoped_timespec_timer/scoped_timespec_timer.h"

extern "C"
{
#include "external/trinamic/tmc/ic/TMC4671/TMC4671.h"
#include "external/trinamic/tmc/ic/TMC4671/TMC4671_Register.h"
#include "external/trinamic/tmc/ic/TMC4671/TMC4671_Variants.h"
#include "external/trinamic/tmc/ic/TMC6100/TMC6100.h"
}

// SPI Configs
static const uint32_t MAX_SPI_SPEED_HZ  = 2000000;  // 2 Mhz
static const uint32_t TMC6100_SPI_SPEED = 1000000;  // 1 Mhz
static const uint32_t TMC4671_SPI_SPEED = 1000000;  // 1 Mhz
static const uint8_t SPI_BITS           = 8;
static const uint32_t SPI_MODE          = 0x3u;
static const uint32_t NUM_RETRIES_SPI   = 3;


static const char* SPI_CS_DRIVER_TO_CONTROLLER_MUX_0_GPIO = "51";
static const char* SPI_CS_DRIVER_TO_CONTROLLER_MUX_1_GPIO = "76";
static const char* MOTOR_DRIVER_RESET_GPIO                = "168";
static const char* DRIVER_CONTROL_ENABLE_GPIO             = "194";


// All trinamic RPMS are electrical RPMS, they don't factor in the number of pole
// pairs of the drive motor.
//
// TODO (#2720): compute from robot constants (this was computed by hand and is accurate)
static double MECHANICAL_MPS_PER_ELECTRICAL_RPM = 0.000111;
static double ELECTRICAL_RPM_PER_MECHANICAL_MPS = 1 / MECHANICAL_MPS_PER_ELECTRICAL_RPM;

static double RUNAWAY_PROTECTION_THRESHOLD_MPS         = 2.00;
static int DRIBBLER_ACCELERATION_THRESHOLD_RPM_PER_S_2 = 10000;


extern "C"
{
    // We need a static pointer here, because trinamic externs the following two
    // SPI binding functions that we need to interface with their API.
    //
    // The motor service exclusively calls the trinamic API which triggers these
    // functions. The motor service will set this variable in the constructor.
    static MotorService* g_motor_service = NULL;

    uint8_t tmc4671_readwriteByte(uint8_t motor, uint8_t data, uint8_t last_transfer)
    {
        return g_motor_service->tmc4671ReadWriteByte(motor, data, last_transfer);
    }

    uint8_t tmc6100_readwriteByte(uint8_t motor, uint8_t data, uint8_t last_transfer)
    {
        return g_motor_service->tmc6100ReadWriteByte(motor, data, last_transfer);
    }
}

MotorService::MotorService(const RobotConstants_t& robot_constants,
                           int control_loop_frequency_hz)
    : spi_demux_select_0_(SPI_CS_DRIVER_TO_CONTROLLER_MUX_0_GPIO, GpioDirection::OUTPUT,
                          GpioState::LOW),
      spi_demux_select_1_(SPI_CS_DRIVER_TO_CONTROLLER_MUX_1_GPIO, GpioDirection::OUTPUT,
                          GpioState::LOW),
      driver_control_enable_gpio_(DRIVER_CONTROL_ENABLE_GPIO, GpioDirection::OUTPUT,
                                  GpioState::HIGH),
      reset_gpio_(MOTOR_DRIVER_RESET_GPIO, GpioDirection::OUTPUT, GpioState::HIGH),
      robot_constants_(robot_constants),
      euclidean_to_four_wheel_(robot_constants),
      motor_fault_detector_(0),
      dribbler_ramp_rpm_(0),
      tracked_motor_fault_start_time_(std::nullopt),
      num_tracked_motor_resets_(0)
{
    int ret = 0;

    /**
     * Opens SPI File Descriptor
     *
     * @param motor_name The name of the motor the spi path is connected to
     * @param chip_select Which chip select to use
     */
#define OPEN_SPI_FILE_DESCRIPTOR(motor_name, chip_select)                                \
                                                                                         \
    file_descriptors_[chip_select] = open(SPI_PATHS[chip_select], O_RDWR);               \
    CHECK(file_descriptors_[chip_select] >= 0)                                           \
        << "can't open device: " << #motor_name << "error: " << strerror(errno);         \
                                                                                         \
    ret = ioctl(file_descriptors_[chip_select], SPI_IOC_WR_MODE32, &SPI_MODE);           \
    CHECK(ret != -1) << "can't set spi mode for: " << #motor_name                        \
                     << "error: " << strerror(errno);                                    \
                                                                                         \
    ret = ioctl(file_descriptors_[chip_select], SPI_IOC_WR_BITS_PER_WORD, &SPI_BITS);    \
    CHECK(ret != -1) << "can't set bits_per_word for: " << #motor_name                   \
                     << "error: " << strerror(errno);                                    \
                                                                                         \
    ret = ioctl(file_descriptors_[chip_select], SPI_IOC_WR_MAX_SPEED_HZ,                 \
                &MAX_SPI_SPEED_HZ);                                                      \
    CHECK(ret != -1) << "can't set spi max speed hz for: " << #motor_name                \
                     << "error: " << strerror(errno);

    OPEN_SPI_FILE_DESCRIPTOR(front_left, FRONT_LEFT_MOTOR_CHIP_SELECT)
    OPEN_SPI_FILE_DESCRIPTOR(front_right, FRONT_RIGHT_MOTOR_CHIP_SELECT)
    OPEN_SPI_FILE_DESCRIPTOR(back_left, BACK_LEFT_MOTOR_CHIP_SELECT)
    OPEN_SPI_FILE_DESCRIPTOR(back_right, BACK_RIGHT_MOTOR_CHIP_SELECT)
    OPEN_SPI_FILE_DESCRIPTOR(dribbler, DRIBBLER_MOTOR_CHIP_SELECT)

    // Make this instance available to the static functions above
    g_motor_service = this;
}

MotorService::~MotorService() {}

void MotorService::setup()
{
    const auto now                             = std::chrono::system_clock::now();
    long int total_duration_since_last_fault_s = 0;
    if (tracked_motor_fault_start_time_.has_value())
    {
        total_duration_since_last_fault_s =
            std::chrono::duration_cast<std::chrono::seconds>(
                now - tracked_motor_fault_start_time_.value())
                .count();
    }

    if (tracked_motor_fault_start_time_.has_value() &&
        total_duration_since_last_fault_s < MOTOR_FAULT_TIME_THRESHOLD_S)
    {
        num_tracked_motor_resets_++;
    }
    else
    {
        tracked_motor_fault_start_time_ = std::make_optional(now);
        num_tracked_motor_resets_       = 1;
    }


    if (tracked_motor_fault_start_time_.has_value() &&
        num_tracked_motor_resets_ > MOTOR_FAULT_THRESHOLD_COUNT)
    {
        LOG(FATAL) << "In the last " << total_duration_since_last_fault_s
                   << "s, the motor board has reset " << num_tracked_motor_resets_
                   << " times. Thunderloop crashing for safety.";
    }

    prev_wheel_velocities_ = {0.0, 0.0, 0.0, 0.0};

    // Clear faults by resetting all the chips on the motor board
    reset_gpio_.setValue(GpioState::LOW);
    usleep(MICROSECONDS_PER_MILLISECOND * 100);

    reset_gpio_.setValue(GpioState::HIGH);
    usleep(MICROSECONDS_PER_MILLISECOND * 100);

    for (uint8_t motor = 0; motor < NUM_MOTORS; ++motor)
    {
        LOG(INFO) << "Clearing RESET for " << MOTOR_NAMES[motor];
        tmc6100_writeInt(motor, TMC6100_GSTAT, 0x00000001);
        cached_motor_faults_[motor] = MotorFaultIndicator();
        encoder_calibrated_[motor]  = false;
    }

    // Drive Motor Setup
    for (uint8_t motor = 0; motor < NUM_DRIVE_MOTORS; motor++)
    {
        startDriver(motor);
        checkDriverFault(motor);
        // Start all the controllers as drive motor controllers
        startController(motor, false);
        tmc4671_setTargetVelocity(motor, 0);
    }

    // Dribbler Motor Setup
    startDriver(DRIBBLER_MOTOR_CHIP_SELECT);
    checkDriverFault(DRIBBLER_MOTOR_CHIP_SELECT);
    startController(DRIBBLER_MOTOR_CHIP_SELECT, true);
    tmc4671_setTargetVelocity(DRIBBLER_MOTOR_CHIP_SELECT, 0);

    checkEncoderConnections();

    // calibrate the encoders
    for (uint8_t motor = 0; motor < NUM_DRIVE_MOTORS; motor++)
    {
        startEncoderCalibration(motor);
    }

    sleep(1);

    for (uint8_t motor = 0; motor < NUM_DRIVE_MOTORS; motor++)
    {
        endEncoderCalibration(motor);
    }

    is_initialized_ = true;
}

MotorService::MotorFaultIndicator MotorService::checkDriverFault(uint8_t motor)
{
    bool drive_enabled = true;
    std::unordered_set<TbotsProto::MotorFault> motor_faults;

    int gstat = tmc6100_readInt(motor, TMC6100_GSTAT);
    std::bitset<32> gstat_bitset(gstat);

    if (gstat_bitset.any())
    {
        LOG(WARNING) << "======= Faults For Motor " << std::to_string(motor) << "=======";
    }

    if (gstat_bitset[0])
    {
        LOG(WARNING)
            << "Indicates that the IC has been reset. All registers have been cleared to reset values."
            << "Attention: DRV_EN must be high to allow clearing reset";
        motor_faults.insert(TbotsProto::MotorFault::RESET);
    }

    if (gstat_bitset[1])
    {
        LOG(WARNING)
            << "drv_otpw : Indicates, that the driver temperature has exceeded overtemperature prewarning-level."
            << "No action is taken. This flag is latched.";
        motor_faults.insert(TbotsProto::MotorFault::DRIVER_OVERTEMPERATURE_PREWARNING);
    }

    if (gstat_bitset[2])
    {
        LOG(WARNING)
            << "drv_ot: Indicates, that the driver has been shut down due to overtemperature."
            << "This flag can only be cleared when the temperature is below the limit again."
            << "It is latched for information.";
        motor_faults.insert(TbotsProto::MotorFault::DRIVER_OVERTEMPERATURE);
    }

    if (gstat_bitset[3])
    {
        LOG(WARNING) << "uv_cp: Indicates an undervoltage on the charge pump."
                     << "The driver is disabled during undervoltage."
                     << "This flag is latched for information.";
        motor_faults.insert(TbotsProto::MotorFault::UNDERVOLTAGE_CHARGEPUMP);
        drive_enabled = false;
    }

    if (gstat_bitset[4])
    {
        LOG(WARNING) << "shortdet_u: Short to GND detected on phase U."
                     << "The driver becomes disabled until flag becomes cleared.";
        motor_faults.insert(TbotsProto::MotorFault::PHASE_U_SHORT_COUNTER_DETECTED);
        drive_enabled = false;
    }

    if (gstat_bitset[5])
    {
        LOG(WARNING) << "s2gu: Short to GND detected on phase U."
                     << "The driver becomes disabled until flag becomes cleared.";
        motor_faults.insert(TbotsProto::MotorFault::PHASE_U_SHORT_TO_GND_DETECTED);
        drive_enabled = false;
    }

    if (gstat_bitset[6])
    {
        LOG(WARNING) << "s2vsu: Short to VS detected on phase U."
                     << "The driver becomes disabled until flag becomes cleared.";
        motor_faults.insert(TbotsProto::MotorFault::PHASE_U_SHORT_TO_VS_DETECTED);
        drive_enabled = false;
    }

    if (gstat_bitset[8])
    {
        LOG(WARNING) << "shortdet_v: V short counter has triggered at least once.";
        motor_faults.insert(TbotsProto::MotorFault::PHASE_V_SHORT_COUNTER_DETECTED);
    }

    if (gstat_bitset[9])
    {
        LOG(WARNING) << "s2gv: Short to GND detected on phase V."
                     << "The driver becomes disabled until flag becomes cleared.";
        motor_faults.insert(TbotsProto::MotorFault::PHASE_V_SHORT_TO_GND_DETECTED);
        drive_enabled = false;
    }

    if (gstat_bitset[10])
    {
        LOG(WARNING) << "s2vsv: Short to VS detected on phase V."
                     << "The driver becomes disabled until flag becomes cleared.";
        motor_faults.insert(TbotsProto::MotorFault::PHASE_V_SHORT_TO_VS_DETECTED);
        drive_enabled = false;
    }

    if (gstat_bitset[12])
    {
        LOG(WARNING) << "shortdet_w: short counter has triggered at least once.";
        motor_faults.insert(TbotsProto::MotorFault::PHASE_W_SHORT_COUNTER_DETECTED);
    }

    if (gstat_bitset[13])
    {
        LOG(WARNING) << "s2gw: Short to GND detected on phase W."
                     << "The driver becomes disabled until flag becomes cleared.";
        motor_faults.insert(TbotsProto::MotorFault::PHASE_W_SHORT_TO_GND_DETECTED);
        drive_enabled = false;
    }

    if (gstat_bitset[14])
    {
        LOG(WARNING) << "s2vsw: Short to VS detected on phase W."
                     << "The driver becomes disabled until flag becomes cleared.";
        motor_faults.insert(TbotsProto::MotorFault::PHASE_W_SHORT_TO_VS_DETECTED);
        drive_enabled = false;
    }

    return MotorFaultIndicator(drive_enabled, motor_faults);
}

TbotsProto::MotorStatus MotorService::updateMotorStatus(double front_left_velocity_mps,
                                                        double front_right_velocity_mps,
                                                        double back_left_velocity_mps,
                                                        double back_right_velocity_mps,
                                                        double dribbler_rpm)
{
    TbotsProto::MotorStatus motor_status;

    cached_motor_faults_[motor_fault_detector_] = checkDriverFault(motor_fault_detector_);

    for (uint8_t motor = 0; motor < NUM_MOTORS; ++motor)
    {
        if (motor != DRIBBLER_MOTOR_CHIP_SELECT)
        {
            TbotsProto::DriveUnit drive_status;
            drive_status.set_enabled(cached_motor_faults_[motor].drive_enabled);

            for (const TbotsProto::MotorFault& fault :
                 cached_motor_faults_[motor].motor_faults)
            {
                drive_status.add_motor_faults(fault);
            }

            if (motor == FRONT_LEFT_MOTOR_CHIP_SELECT)
            {
                *(motor_status.mutable_front_left()) = drive_status;
            }
            if (motor == FRONT_RIGHT_MOTOR_CHIP_SELECT)
            {
                *(motor_status.mutable_front_right()) = drive_status;
            }
            if (motor == BACK_LEFT_MOTOR_CHIP_SELECT)
            {
                *(motor_status.mutable_back_left()) = drive_status;
            }
            if (motor == BACK_RIGHT_MOTOR_CHIP_SELECT)
            {
                *(motor_status.mutable_back_right()) = drive_status;
            }
        }
        else
        {
            TbotsProto::DribblerStatus dribbler_status;
            dribbler_status.set_dribbler_rpm(static_cast<float>(dribbler_rpm));
            dribbler_status.set_enabled(cached_motor_faults_[motor].drive_enabled);
            for (const TbotsProto::MotorFault& fault :
                 cached_motor_faults_[motor].motor_faults)
            {
                dribbler_status.add_motor_faults(fault);
            }

            *(motor_status.mutable_dribbler()) = dribbler_status;
        }
    }

    motor_status.mutable_front_left()->set_wheel_velocity(
        static_cast<float>(front_left_velocity_mps));
    motor_status.mutable_front_right()->set_wheel_velocity(
        static_cast<float>(front_right_velocity_mps));
    motor_status.mutable_back_left()->set_wheel_velocity(
        static_cast<float>(back_left_velocity_mps));
    motor_status.mutable_back_right()->set_wheel_velocity(
        static_cast<float>(back_right_velocity_mps));

    motor_fault_detector_ =
        static_cast<uint8_t>((motor_fault_detector_ + 1) % NUM_MOTORS);

    return motor_status;
}



TbotsProto::MotorStatus MotorService::poll(const TbotsProto::MotorControl& motor,
                                           double time_elapsed_since_last_poll_s)
{
    bool encoders_calibrated = (encoder_calibrated_[FRONT_LEFT_MOTOR_CHIP_SELECT] ||
                                encoder_calibrated_[FRONT_RIGHT_MOTOR_CHIP_SELECT] ||
                                encoder_calibrated_[BACK_LEFT_MOTOR_CHIP_SELECT] ||
                                encoder_calibrated_[BACK_RIGHT_MOTOR_CHIP_SELECT]);

    if (!encoders_calibrated)
    {
        is_initialized_ = false;
    }

    // checks if any motor has reset, sends a log message if so
    for (uint8_t motor = 0; motor < NUM_MOTORS; ++motor)
    {
        if (requiresMotorReinit(motor))
        {
            LOG(DEBUG) << "RESET DETECTED FOR MOTOR: " << MOTOR_NAMES[motor];
            is_initialized_ = false;
        }
    }

    if (!is_initialized_)
    {
        LOG(INFO) << "MotorService re-initializing";
        setup();
    }

    CHECK(encoder_calibrated_[FRONT_LEFT_MOTOR_CHIP_SELECT] &&
          encoder_calibrated_[FRONT_RIGHT_MOTOR_CHIP_SELECT] &&
          encoder_calibrated_[BACK_LEFT_MOTOR_CHIP_SELECT] &&
          encoder_calibrated_[BACK_RIGHT_MOTOR_CHIP_SELECT])
        << "Running without encoder calibration can cause serious harm, exiting";

    // Get current wheel electical RPMs (don't account for pole pairs). Values will be
    // written on next iteration
    double front_right_velocity =
        static_cast<double>(tmc4671ReadThenWriteValue(
            FRONT_RIGHT_MOTOR_CHIP_SELECT, TMC4671_PID_VELOCITY_ACTUAL,
            TMC4671_PID_VELOCITY_TARGET, front_right_target_velocity)) *
        MECHANICAL_MPS_PER_ELECTRICAL_RPM;
    double front_left_velocity =
        static_cast<double>(tmc4671ReadThenWriteValue(
            FRONT_LEFT_MOTOR_CHIP_SELECT, TMC4671_PID_VELOCITY_ACTUAL,
            TMC4671_PID_VELOCITY_TARGET, front_left_target_velocity)) *
        MECHANICAL_MPS_PER_ELECTRICAL_RPM;
    double back_right_velocity =
        static_cast<double>(tmc4671ReadThenWriteValue(
            BACK_RIGHT_MOTOR_CHIP_SELECT, TMC4671_PID_VELOCITY_ACTUAL,
            TMC4671_PID_VELOCITY_TARGET, back_right_target_velocity)) *
        MECHANICAL_MPS_PER_ELECTRICAL_RPM;
    double back_left_velocity =
        static_cast<double>(tmc4671ReadThenWriteValue(
            BACK_LEFT_MOTOR_CHIP_SELECT, TMC4671_PID_VELOCITY_ACTUAL,
            TMC4671_PID_VELOCITY_TARGET, back_left_target_velocity)) *
        MECHANICAL_MPS_PER_ELECTRICAL_RPM;
    double dribbler_rpm = static_cast<double>(
        tmc4671ReadThenWriteValue(DRIBBLER_MOTOR_CHIP_SELECT, TMC4671_PID_VELOCITY_ACTUAL,
                                  TMC4671_PID_VELOCITY_TARGET, dribbler_ramp_rpm_));

    // Construct a MotorStatus object with the current velocities and dribbler rpm
    TbotsProto::MotorStatus motor_status =
        updateMotorStatus(front_left_velocity, front_right_velocity, back_left_velocity,
                          back_right_velocity, dribbler_rpm);

    // This order needs to match euclidean_to_four_wheel converters order
    // We also want to work in the meters per second space rather than electrical RPMs
    WheelSpace_t current_wheel_velocities = {front_right_velocity, front_left_velocity,
                                             back_left_velocity, back_right_velocity};

    // Run-away protection
    if (std::abs(current_wheel_velocities[FRONT_RIGHT_WHEEL_SPACE_INDEX] -
                 prev_wheel_velocities_[FRONT_RIGHT_WHEEL_SPACE_INDEX]) >
        RUNAWAY_PROTECTION_THRESHOLD_MPS)
    {
        driver_control_enable_gpio_.setValue(GpioState::LOW);
        LOG(FATAL) << "Front right motor runaway";
    }
    else if (std::abs(current_wheel_velocities[FRONT_LEFT_WHEEL_SPACE_INDEX] -
                      prev_wheel_velocities_[FRONT_LEFT_WHEEL_SPACE_INDEX]) >
             RUNAWAY_PROTECTION_THRESHOLD_MPS)
    {
        driver_control_enable_gpio_.setValue(GpioState::LOW);
        LOG(FATAL) << "Front left motor runaway";
    }
    else if (std::abs(current_wheel_velocities[BACK_LEFT_WHEEL_SPACE_INDEX] -
                      prev_wheel_velocities_[BACK_LEFT_WHEEL_SPACE_INDEX]) >
             RUNAWAY_PROTECTION_THRESHOLD_MPS)
    {
        driver_control_enable_gpio_.setValue(GpioState::LOW);
        LOG(FATAL) << "Back left motor runaway";
    }
    else if (std::abs(current_wheel_velocities[BACK_RIGHT_WHEEL_SPACE_INDEX] -
                      prev_wheel_velocities_[BACK_RIGHT_WHEEL_SPACE_INDEX]) >
             RUNAWAY_PROTECTION_THRESHOLD_MPS)
    {
        driver_control_enable_gpio_.setValue(GpioState::LOW);
        LOG(FATAL) << "Back right motor runaway";
    }

    // Convert to Euclidean velocity_delta
    EuclideanSpace_t current_euclidean_velocity =
        euclidean_to_four_wheel_.getEuclideanVelocity(current_wheel_velocities);

    motor_status.mutable_local_velocity()->set_x_component_meters(
        current_euclidean_velocity[1]);
    motor_status.mutable_local_velocity()->set_y_component_meters(
        -current_euclidean_velocity[0]);
    motor_status.mutable_angular_velocity()->set_radians_per_second(
        current_euclidean_velocity[2]);

    WheelSpace_t target_wheel_velocities = WheelSpace_t::Zero();

    // Get target wheel velocities from the primitive
    if (motor.has_direct_per_wheel_control())
    {
        TbotsProto::MotorControl_DirectPerWheelControl direct_per_wheel =
            motor.direct_per_wheel_control();
        target_wheel_velocities = {
            direct_per_wheel.front_right_wheel_velocity(),
            direct_per_wheel.front_left_wheel_velocity(),
            direct_per_wheel.back_left_wheel_velocity(),
            direct_per_wheel.back_right_wheel_velocity(),
        };
    }
    else if (motor.has_direct_velocity_control())
    {
        TbotsProto::MotorControl_DirectVelocityControl direct_velocity =
            motor.direct_velocity_control();
        EuclideanSpace_t target_euclidean_velocity = {
            -direct_velocity.velocity().y_component_meters(),
            direct_velocity.velocity().x_component_meters(),
            direct_velocity.angular_velocity().radians_per_second()};

        target_wheel_velocities =
            euclidean_to_four_wheel_.getWheelVelocity(target_euclidean_velocity);
    }

    // ramp the target velocities to keep acceleration compared to current velocities
    // within safe bounds
    target_wheel_velocities = euclidean_to_four_wheel_.rampWheelVelocity(
        prev_wheel_velocities_, target_wheel_velocities, time_elapsed_since_last_poll_s);

    // TODO (#2719): interleave the angular accelerations in here at some point.
    prev_wheel_velocities_ = target_wheel_velocities;

    // Calculate speeds accounting for acceleration
    front_right_target_velocity =
        static_cast<int>(target_wheel_velocities[FRONT_RIGHT_WHEEL_SPACE_INDEX] *
                         ELECTRICAL_RPM_PER_MECHANICAL_MPS);
    front_left_target_velocity =
        static_cast<int>(target_wheel_velocities[FRONT_LEFT_WHEEL_SPACE_INDEX] *
                         ELECTRICAL_RPM_PER_MECHANICAL_MPS);
    back_left_target_velocity =
        static_cast<int>(target_wheel_velocities[BACK_LEFT_WHEEL_SPACE_INDEX] *
                         ELECTRICAL_RPM_PER_MECHANICAL_MPS);
    back_right_target_velocity =
        static_cast<int>(target_wheel_velocities[BACK_RIGHT_WHEEL_SPACE_INDEX] *
                         ELECTRICAL_RPM_PER_MECHANICAL_MPS);

    // Get target dribbler rpm from the primitive
    int target_dribbler_rpm;
    if (motor.drive_control_case() ==
        TbotsProto::MotorControl::DriveControlCase::DRIVE_CONTROL_NOT_SET)
    {
        target_dribbler_rpm = 0;
    }
    else
    {
        target_dribbler_rpm = motor.dribbler_speed_rpm();
    }

    // Ramp the dribbler velocity
    // Clamp the max acceleration
    int max_dribbler_delta_rpm = static_cast<int>(
        DRIBBLER_ACCELERATION_THRESHOLD_RPM_PER_S_2 * time_elapsed_since_last_poll_s);
    int delta_rpm = std::clamp(target_dribbler_rpm - dribbler_ramp_rpm_,
                               -max_dribbler_delta_rpm, max_dribbler_delta_rpm);
    dribbler_ramp_rpm_ += delta_rpm;

    // Clamp to the max rpm
    int max_dribbler_rpm =
        std::abs(static_cast<int>(robot_constants_.max_force_dribbler_speed_rpm));
    dribbler_ramp_rpm_ =
        std::clamp(dribbler_ramp_rpm_, -max_dribbler_rpm, max_dribbler_rpm);

    motor_status.mutable_dribbler()->set_dribbler_rpm(float(dribbler_ramp_rpm_));

    return motor_status;
}

bool MotorService::requiresMotorReinit(uint8_t motor)
{
    auto reset_search =
        cached_motor_faults_[motor].motor_faults.find(TbotsProto::MotorFault::RESET);

    return !cached_motor_faults_[motor].drive_enabled ||
           (reset_search != cached_motor_faults_[motor].motor_faults.end());
}

void MotorService::spiTransfer(int fd, uint8_t const* tx, uint8_t const* rx, unsigned len,
                               uint32_t spi_speed)
{
    int ret;

    struct spi_ioc_transfer tr[2];
    memset(tr, 0, sizeof(tr));

    tr[0].tx_buf        = (unsigned long)tx_;
    tr[0].rx_buf        = (unsigned long)rx_;
    tr[0].len           = len;
    tr[0].delay_usecs   = 0;
    tr[0].speed_hz      = spi_speed;
    tr[0].bits_per_word = 8;

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);

    CHECK(ret >= 1) << "SPI Transfer to motor failed, not safe to proceed: errno "
                    << strerror(errno);
}


void MotorService::readThenWriteSpiTransfer(int fd, const uint8_t* read_tx,
                                            const uint8_t* write_tx,
                                            const uint8_t* read_rx, uint32_t spi_speed)
{
    int ret1, ret2;

    uint8_t write_rx[5] = {0};

    struct spi_ioc_transfer tr[2];
    memset(tr, 0, sizeof(tr));

    tr[0].tx_buf        = (unsigned long)read_tx;
    tr[0].rx_buf        = (unsigned long)read_rx;
    tr[0].len           = 5;
    tr[0].delay_usecs   = 0;
    tr[0].speed_hz      = spi_speed;
    tr[0].bits_per_word = 8;
    tr[0].cs_change     = 0;
    tr[1].tx_buf        = (unsigned long)write_tx;
    tr[1].rx_buf        = (unsigned long)write_rx;
    tr[1].len           = 5;
    tr[1].delay_usecs   = 0;
    tr[1].speed_hz      = spi_speed;
    tr[1].bits_per_word = 8;
    tr[1].cs_change     = 0;

    ret1 = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    ret2 = ioctl(fd, SPI_IOC_MESSAGE(1), &tr[1]);

    CHECK(ret1 >= 1 && ret2 >= 1)
        << "SPI Transfer to motor failed, not safe to proceed: errno " << strerror(errno);
}


// Both the TMC4671 (the controller) and the TMC6100 (the driver) respect
// the same SPI interface. So when we bind the API, we can use the same
// readWriteByte function, provided that the chip select pin is turning on
// the right chip.
//
// Each TMC4671 controller, TMC6100 driver and encoder group have their chip
// selects coming in from a demux (see diagram below). The demux is controlled
// by two bits {spi_demux_select_0, spi_demux_select_1}. If the bits are
// 10 the TMC4671 is selected, when the select bits are 01 the TMC6100 is
// selected and when they are 11 the encoder is selected. 00 disconnects all
// 3 chips.
//
//
//                                      FRONT LEFT MOTOR
//                                 CONTROLLER + DRIVER + ENCODER
//
//                    ┌───────┐        ┌───────────────┐
//                    │       │        │               │
//                    │  2:4  │  10    │  ┌─────────┐  │
//                    │       ├────────┼──►TMC4671  │  │  B0
//     FRONT_LEFT_CS  │ DEMUX │        │  └─────────┘  │
//     ───────────────►       │        │               │
//                    │       │  01    │  ┌─────────┐  │
//                    │       ├────────┼──►TMC6100  │  │  B1
//                    │       │        │  └─────────┘  │
//                    │       │        │               │
//                    │       │  11    │  ┌─────────┐  │
//                    │       ├────────┼──►ENCODER  │  │  B2
//                    │       │        │  └─────────┘  │
//                    └───▲───┘        │               │
//                        │            └───────────────┘
//                        │
//                spi_demux_sel_0 & 1
//
uint8_t MotorService::tmc4671ReadWriteByte(uint8_t motor, uint8_t data,
                                           uint8_t last_transfer)
{
    spi_demux_select_0_.setValue(GpioState::HIGH);
    spi_demux_select_1_.setValue(GpioState::LOW);
    return readWriteByte(motor, data, last_transfer, TMC4671_SPI_SPEED);
}

int32_t MotorService::tmc4671ReadThenWriteValue(uint8_t motor, uint8_t read_addr,
                                                uint8_t write_addr, int32_t write_data)
{
    spi_demux_select_0_.setValue(GpioState::HIGH);
    spi_demux_select_1_.setValue(GpioState::LOW);
    // ensure tx_ and rx_ are cleared
    memset(read_tx_, 0, 5);
    memset(write_tx_, 0, 5);
    memset(read_rx_, 0, 5);

    //  Trinamic transactions looks like this:
    //  + - - - + - - - + - - - + - - - + - - - +
    //  |  ADDR |             DATA              |
    //  + - - - + - - - + - - - + - - - + - - - +
    //      0        1      2       3       4
    //  Also it is in BIG Endian, therefore MSB is leftmost bit of 0.
    //  For a write, MSB must be 1, for read, MSB must be 0
    //  https://github.com/trinamic/TMC-API/blob/master/tmc/ic/TMC4671/TMC4671.c
    read_tx_[0]  = read_addr & 0x7f;
    write_tx_[0] = write_addr | 0x80;

    // Convert from little endian to big endian
    for (int i = 3; i >= 0; i--)
    {
        uint8_t byte_to_copy = (uint8_t)(0xff & (write_data >> 8 * i));
        write_tx_[4 - i]     = byte_to_copy;
    }
    //    memcpy(write_tx_+1,&write_data,4);

    readThenWriteSpiTransfer(file_descriptors_[motor], read_tx_, write_tx_, read_rx_,
                             TMC4671_SPI_SPEED);

    int32_t value = read_rx_[0];
    for (int i = 1; i < 5; i++)
    {
        value <<= 8;
        value |= read_rx_[i];
    }
    return value;
}

uint8_t MotorService::tmc6100ReadWriteByte(uint8_t motor, uint8_t data,
                                           uint8_t last_transfer)
{
    spi_demux_select_0_.setValue(GpioState::LOW);
    spi_demux_select_1_.setValue(GpioState::HIGH);
    return readWriteByte(motor, data, last_transfer, TMC6100_SPI_SPEED);
}

uint8_t MotorService::readWriteByte(uint8_t motor, uint8_t data, uint8_t last_transfer,
                                    uint32_t spi_speed)
{
    uint8_t ret_byte = 0;

    if (!transfer_started_)
    {
        memset(tx_, 0, sizeof(tx_));
        memset(rx_, 0, sizeof(rx_));
        position_ = 0;

        if (data & TMC_WRITE_BIT)
        {
            // If the transfer started and its a write operation,
            // set the appropriate flags.
            currently_reading_ = false;
            currently_writing_ = true;
        }
        else
        {
            // The first byte should contain the address on a read operation.
            // Trigger a transfer (1 byte) and buffer the response (4 bytes)
            tx_[position_] = data;
            spiTransfer(file_descriptors_[motor], tx_, rx_, 5, spi_speed);

            currently_reading_ = true;
            currently_writing_ = false;
        }

        transfer_started_ = true;
    }

    if (currently_writing_)
    {
        // Buffer the data to send out when last_transfer is true.
        tx_[position_++] = data;
    }

    if (currently_reading_)
    {
        // If we are reading, we just need to return the buffered data
        // byte by byte.
        ret_byte = rx_[position_++];
    }

    if (currently_writing_ && last_transfer)
    {
        // we have all the bytes for this transfer, lets trigger the transfer and
        // reset state
        spiTransfer(file_descriptors_[motor], tx_, rx_, 5, spi_speed);
        transfer_started_ = false;
    }

    if (currently_reading_ && last_transfer)
    {
        // when reading, if last transfer is true, we just need to reset state
        transfer_started_ = false;
    }

    return ret_byte;
}

void MotorService::writeToDriverOrDieTrying(uint8_t motor, uint8_t address, int32_t value)
{
    int num_retires_left = NUM_RETRIES_SPI;
    int read_value       = 0;

    // The SPI lines have a lot of noise, and sometimes a transfer will fail
    // randomly. So we retry a few times before giving up.
    while (num_retires_left > 0)
    {
        tmc6100_writeInt(motor, address, value);
        read_value = tmc6100_readInt(motor, address);
        if (read_value == value)
        {
            return;
        }
        LOG(DEBUG) << "SPI Transfer to Driver Failed, retrying...";
        num_retires_left--;
    }

    // If we get here, we have failed to write to the driver. We reset
    // the chip to clear any bad values we just wrote and crash so everything stops.
    reset_gpio_.setValue(GpioState::LOW);
    CHECK(read_value == value) << "Couldn't write " << value
                               << " to the TMC6100 at address " << address
                               << " at address " << static_cast<uint32_t>(address)
                               << " on motor " << static_cast<uint32_t>(motor)
                               << " received: " << read_value;
}

void MotorService::writeToControllerOrDieTrying(uint8_t motor, uint8_t address,
                                                int32_t value)
{
    int num_retires_left = NUM_RETRIES_SPI;
    int read_value       = 0;

    // The SPI lines have a lot of noise, and sometimes a transfer will fail
    // randomly. So we retry a few times before giving up.
    while (num_retires_left > 0)
    {
        tmc4671_writeInt(motor, address, value);
        read_value = tmc4671_readInt(motor, address);
        if (read_value == value)
        {
            return;
        }
        LOG(DEBUG) << "SPI Transfer to Controller Failed, retrying...";
        num_retires_left--;
    }

    // If we get here, we have failed to write to the controller. We reset
    // the chip to clear any bad values we just wrote and crash so everything stops.
    reset_gpio_.setValue(GpioState::LOW);
    CHECK(read_value == value) << "Couldn't write " << value
                               << " to the TMC4671 at address " << address
                               << " at address " << static_cast<uint32_t>(address)
                               << " on motor " << static_cast<uint32_t>(motor)
                               << " received: " << read_value;
}

void MotorService::configurePWM(uint8_t motor)
{
    LOG(INFO) << "Configuring PWM for motor " << static_cast<uint32_t>(motor);
    // Please read the header file and the datasheet for more info
    writeToControllerOrDieTrying(motor, TMC4671_PWM_POLARITIES, 0x00000000);
    writeToControllerOrDieTrying(motor, TMC4671_PWM_MAXCNT, 0x00000F9F);
    writeToControllerOrDieTrying(motor, TMC4671_PWM_BBM_H_BBM_L, 0x00002828);
    writeToControllerOrDieTrying(motor, TMC4671_PWM_SV_CHOP, 0x00000107);
}

void MotorService::configureDrivePI(uint8_t motor)
{
    LOG(INFO) << "Configuring Drive PI for motor " << static_cast<uint32_t>(motor);
    // Please read the header file and the datasheet for more info
    // These values were calibrated using the TMC-IDE
    writeToControllerOrDieTrying(motor, TMC4671_PID_FLUX_P_FLUX_I, 67109376);
    writeToControllerOrDieTrying(motor, TMC4671_PID_TORQUE_P_TORQUE_I, 67109376);
    writeToControllerOrDieTrying(motor, TMC4671_PID_VELOCITY_P_VELOCITY_I, 52428800);

    // Explicitly disable the position controller
    writeToControllerOrDieTrying(motor, TMC4671_PID_POSITION_P_POSITION_I, 0);

    writeToControllerOrDieTrying(motor, TMC4671_PIDOUT_UQ_UD_LIMITS, 32767);
    writeToControllerOrDieTrying(motor, TMC4671_PID_TORQUE_FLUX_LIMITS, 2500);
    writeToControllerOrDieTrying(motor, TMC4671_PID_ACCELERATION_LIMIT, 1000);

    writeToControllerOrDieTrying(motor, TMC4671_PID_VELOCITY_LIMIT, 45000);

    tmc4671_switchToMotionMode(motor, TMC4671_MOTION_MODE_VELOCITY);
}

void MotorService::configureDribblerPI(uint8_t motor)
{
    LOG(INFO) << "Configuring Dribbler PI for motor " << static_cast<uint32_t>(motor);
    // Please read the header file and the datasheet for more info
    // These values were calibrated using the TMC-IDE
    writeToControllerOrDieTrying(motor, TMC4671_PID_FLUX_P_FLUX_I, 39337600);
    writeToControllerOrDieTrying(motor, TMC4671_PID_TORQUE_P_TORQUE_I, 39333600);
    writeToControllerOrDieTrying(motor, TMC4671_PID_VELOCITY_P_VELOCITY_I, 2621448);

    // Explicitly disable the position controller
    writeToControllerOrDieTrying(motor, TMC4671_PID_POSITION_P_POSITION_I, 0);

    writeToControllerOrDieTrying(motor, TMC4671_PIDOUT_UQ_UD_LIMITS, 32767);
    // TODO (#2677) support MAX_FORCE mode. This value can go up to 4.8 amps but we set it
    // to 2 for now (sufficient for INDEFINITE mode).
    writeToControllerOrDieTrying(motor, TMC4671_PID_TORQUE_FLUX_LIMITS, 4000);
    writeToControllerOrDieTrying(motor, TMC4671_PID_ACCELERATION_LIMIT, 40000);
    writeToControllerOrDieTrying(motor, TMC4671_PID_VELOCITY_LIMIT, 15000);
}

void MotorService::configureADC(uint8_t motor)
{
    LOG(INFO) << "Configuring ADC for motor " << static_cast<uint32_t>(motor);
    // ADC configuration
    writeToControllerOrDieTrying(motor, TMC4671_ADC_I_SELECT, 0x18000100);
    writeToControllerOrDieTrying(motor, TMC4671_dsADC_MDEC_B_MDEC_A, 0x014E014E);

    // These values have been calibrated for the TI INA240 current sense amplifier.
    // The scaling is also set to work with both the drive and dribbler motors.
    //
    // If you wish to use the TMC4671+TMC6100-BOB you can use the following values,
    // that work for the AD8418 current sense amplifier
    //
    // TMC4671_ADC_I0_SCALE_OFFSET = 0x010081DD
    // TMC4671_ADC_I1_SCALE_OFFSET = 0x0100818E
    //
    writeToControllerOrDieTrying(motor, TMC4671_ADC_I0_SCALE_OFFSET, 0x000981DD);
    writeToControllerOrDieTrying(motor, TMC4671_ADC_I1_SCALE_OFFSET, 0x0009818E);
}

void MotorService::configureEncoder(uint8_t motor)
{
    LOG(INFO) << "Configuring Encoder for motor " << static_cast<uint32_t>(motor);
    // ABN encoder settings
    writeToControllerOrDieTrying(motor, TMC4671_ABN_DECODER_MODE, 0x00000000);
    writeToControllerOrDieTrying(motor, TMC4671_ABN_DECODER_PPR, 0x00001000);
}

void MotorService::configureHall(uint8_t motor)
{
    LOG(INFO) << "Configuring Hall for motor " << static_cast<uint32_t>(motor);
    // Digital hall settings
    writeToControllerOrDieTrying(motor, TMC4671_HALL_MODE, 0x00000000);
    writeToControllerOrDieTrying(motor, TMC4671_HALL_PHI_E_PHI_M_OFFSET, 0x00000000);

    // Feedback selection
    writeToControllerOrDieTrying(motor, TMC4671_PHI_E_SELECTION, TMC4671_PHI_E_HALL);
    writeToControllerOrDieTrying(motor, TMC4671_VELOCITY_SELECTION,
                                 TMC4671_VELOCITY_PHI_E_HAL);
}

void MotorService::startEncoderCalibration(uint8_t motor)
{
    LOG(WARNING) << "Calibrating the encoder, ensure the robot is lifted off the ground";

    writeToControllerOrDieTrying(motor, TMC4671_PID_TORQUE_FLUX_LIMITS, 0x000003E8);
    writeToControllerOrDieTrying(motor, TMC4671_PID_TORQUE_P_TORQUE_I, 0x01000100);
    writeToControllerOrDieTrying(motor, TMC4671_PID_FLUX_P_FLUX_I, 0x01000100);

    writeToControllerOrDieTrying(motor, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008);
    writeToControllerOrDieTrying(motor, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET,
                                 0x00000000);
    writeToControllerOrDieTrying(motor, TMC4671_PHI_E_SELECTION, 0x00000001);
    writeToControllerOrDieTrying(motor, TMC4671_PHI_E_EXT, 0x00000000);
    writeToControllerOrDieTrying(motor, TMC4671_UQ_UD_EXT, 0x00000FFF);
}

void MotorService::endEncoderCalibration(uint8_t motor)
{
    LOG(WARNING) << "Calibrating the encoder, wheels may move";

    writeToControllerOrDieTrying(motor, TMC4671_ABN_DECODER_COUNT, 0x00000000);
    writeToControllerOrDieTrying(motor, TMC4671_UQ_UD_EXT, 0x00000000);
    writeToControllerOrDieTrying(motor, TMC4671_PHI_E_SELECTION, TMC4671_PHI_E_ABN);

    encoder_calibrated_[motor] = true;

    configureDrivePI(motor);
}

void MotorService::runOpenLoopCalibrationRoutine(uint8_t motor, size_t num_samples)
{
    // Some limits
    tmc4671_writeInt(motor, TMC4671_PID_TORQUE_FLUX_LIMITS, 0x000003E8);
    tmc4671_writeInt(motor, TMC4671_PID_TORQUE_P_TORQUE_I, 0x01000100);
    tmc4671_writeInt(motor, TMC4671_PID_FLUX_P_FLUX_I, 0x01000100);

    // Open loop settings
    tmc4671_writeInt(motor, TMC4671_OPENLOOP_MODE, 0x00000000);
    tmc4671_writeInt(motor, TMC4671_OPENLOOP_ACCELERATION, 0x0000003C);
    tmc4671_writeInt(motor, TMC4671_OPENLOOP_VELOCITY_TARGET, 0xFFFFFFFB);

    // Feedback selection
    tmc4671_writeInt(motor, TMC4671_PHI_E_SELECTION, TMC4671_PHI_E_OPEN_LOOP);
    tmc4671_writeInt(motor, TMC4671_UQ_UD_EXT, 0x00000799);

    // Switch to open loop velocity mode
    tmc4671_writeInt(motor, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008);

    // Rotate right
    tmc4671_writeInt(motor, TMC4671_OPENLOOP_VELOCITY_TARGET, 0x0000004A);

    // Setup CSVs
    LOG(CSV, "encoder_calibration_" + std::to_string(motor) + ".csv")
        << "actual_encoder,estimated_phi\n";
    LOG(CSV, "phase_currents_and_voltages_" + std::to_string(motor) + ".csv")
        << "adc_iv,adc_ux,adc_wy,pwm_iv,pwm_ux,pwm_wy\n";

    // Take samples of the useful registers
    for (size_t num_sample = 0; num_sample < num_samples; num_sample++)
    {
        int estimated_phi  = tmc4671_readInt(motor, TMC4671_OPENLOOP_PHI);
        int actual_encoder = tmc4671_readRegister16BitValue(
            motor, TMC4671_ABN_DECODER_PHI_E_PHI_M, BIT_16_TO_31);

        LOG(CSV, "encoder_calibration_" + std::to_string(motor) + ".csv")
            << actual_encoder << "," << estimated_phi << "\n";

        int16_t adc_v =
            tmc4671_readRegister16BitValue(motor, TMC4671_ADC_IV, BIT_0_TO_15);
        int16_t adc_u =
            tmc4671_readRegister16BitValue(motor, TMC4671_ADC_IWY_IUX, BIT_0_TO_15);
        int16_t adc_w =
            tmc4671_readRegister16BitValue(motor, TMC4671_ADC_IWY_IUX, BIT_16_TO_31);

        tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, INTERIM_ADDR_PWM_UV);
        int16_t pwm_v =
            tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15);

        tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, INTERIM_ADDR_PWM_WY_UX);
        int16_t pwm_u =
            tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15);
        int16_t pwm_w =
            tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_16_TO_31);

        LOG(CSV, "phase_currents_and_voltages_" + std::to_string(motor) + ".csv")
            << adc_v << "," << adc_u << "," << adc_w << "," << pwm_v << "," << pwm_u
            << "," << pwm_w << "\n";
    }

    // Stop open loop rotation
    tmc4671_writeInt(motor, TMC4671_OPENLOOP_VELOCITY_TARGET, 0x00000000);
}

void MotorService::startDriver(uint8_t motor)
{
    // Set the drive strength to 0, the weakest it can go as recommended
    // by the TMC4671-TMC6100-BOB datasheet.
    int32_t current_drive_conf = tmc6100_readInt(motor, TMC6100_DRV_CONF);
    writeToDriverOrDieTrying(motor, TMC6100_DRV_CONF,
                             current_drive_conf & (~TMC6100_DRVSTRENGTH_MASK));
    writeToDriverOrDieTrying(motor, TMC6100_GCONF, 0x40);

    // All default but updated SHORTFILTER to 2us to avoid false positive shorts
    // detection.
    writeToDriverOrDieTrying(motor, TMC6100_SHORT_CONF, 0x13020606);

    LOG(DEBUG) << "Driver " << std::to_string(motor) << " accepted conf";
}

void MotorService::startController(uint8_t motor, bool dribbler)
{
    // Read the chip ID to validate the SPI connection
    tmc4671_writeInt(motor, TMC4671_CHIPINFO_ADDR, 0x000000000);
    int chip_id = tmc4671_readInt(motor, TMC4671_CHIPINFO_DATA);

    //    int chip_id = tmc4671ReadThenWriteValue(motor, TMC4671_CHIPINFO_ADDR,
    //    TMC4671_CHIPINFO_ADDR,0x1); LOG(FATAL) <<"CHIP ID: " << chip_id;
    CHECK(0x34363731 == chip_id) << "The TMC4671 of motor "
                                 << static_cast<uint32_t>(motor) << " is not responding";

    LOG(INFO) << "Controller " << std::to_string(motor)
              << " online, responded with: " << chip_id;

    // Configure common controller params
    configurePWM(motor);
    configureADC(motor);

    if (dribbler)
    {
        // Configure to brushless DC motor with 1 pole pair
        writeToControllerOrDieTrying(motor, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, 0x00030001);
        configureHall(motor);

        configureDribblerPI(motor);
    }
    else
    {
        // Configure to brushless DC motor with 8 pole pairs
        writeToControllerOrDieTrying(motor, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, 0x00030008);
        configureEncoder(motor);
    }
}

void MotorService::checkEncoderConnections()
{
    LOG(INFO) << "Starting encoder connection check!";

    std::vector<bool> calibrated_motors(NUM_DRIVE_MOTORS, false);
    std::vector<int> initial_velocities(NUM_DRIVE_MOTORS, 0);

    for (uint8_t motor = 0; motor < NUM_DRIVE_MOTORS; ++motor)
    {
        // read back current velocity
        initial_velocities[motor] = tmc4671_readInt(motor, TMC4671_ABN_DECODER_COUNT);

        // open loop mode can be used without an encoder, set open loop phi positive
        // direction
        writeToControllerOrDieTrying(motor, TMC4671_OPENLOOP_MODE, 0x00000000);
        writeToControllerOrDieTrying(motor, TMC4671_PHI_E_SELECTION,
                                     TMC4671_PHI_E_OPEN_LOOP);
        writeToControllerOrDieTrying(motor, TMC4671_OPENLOOP_ACCELERATION, 0x0000003C);

        // represents effective voltage applied to the motors (% voltage)
        writeToControllerOrDieTrying(motor, TMC4671_UQ_UD_EXT, 0x00000799);

        // uq_ud_ext mode
        writeToControllerOrDieTrying(motor, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008);

        // 10 RPM
        writeToControllerOrDieTrying(motor, TMC4671_OPENLOOP_VELOCITY_TARGET, 0x0000000A);
    }

    for (int num_iterations = 0;
         num_iterations < 10 &&
         std::any_of(calibrated_motors.begin(), calibrated_motors.end(),
                     [](bool calibration_status) { return !calibration_status; });
         ++num_iterations)
    {
        for (uint8_t motor = 0; motor < NUM_DRIVE_MOTORS; ++motor)
        {
            if (calibrated_motors[motor])
            {
                continue;
            }
            // now read back the velocity
            int read_back_velocity = tmc4671_readInt(motor, TMC4671_ABN_DECODER_COUNT);
            LOG(INFO) << MOTOR_NAMES[motor] << " read back: " << read_back_velocity
                      << " and initially read: " << initial_velocities[motor];

            if (read_back_velocity != initial_velocities[motor])
            {
                calibrated_motors[motor] = true;
            }
        }

        // sleep for 100 milliseconds
        usleep(MICROSECONDS_PER_MILLISECOND * 100);
    }

    bool calibrated = true;
    for (uint8_t motor = 0; motor < NUM_DRIVE_MOTORS; ++motor)
    {
        if (!calibrated_motors[motor])
        {
            calibrated = false;
            LOG(WARNING) << "Encoder calibration check failure. " << MOTOR_NAMES[motor]
                         << " did not change as expected";
        }
    }
    if (!calibrated)
    {
        LOG(FATAL)
            << "Encoder calibration check failure. Not all encoders responded as expected";
    }

    // stop all motors, reset back to velocity control mode
    for (uint8_t motor = 0; motor < NUM_DRIVE_MOTORS; ++motor)
    {
        writeToControllerOrDieTrying(motor, TMC4671_OPENLOOP_VELOCITY_TARGET, 0x00000000);
        tmc4671_switchToMotionMode(motor, TMC4671_MOTION_MODE_VELOCITY);
    }

    LOG(INFO) << "All encoders appear to be connected!";
}

void MotorService::resetMotorBoard()
{
    reset_gpio_.setValue(GpioState::LOW);
}
