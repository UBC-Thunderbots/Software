#include "software/jetson_nano/services/motor.h"

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <limits.h>
#include <linux/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>

#include "proto/tbots_software_msgs.pb.h"
#include "software/logger/logger.h"

extern "C"
{
#include "external/trinamic/tmc/ic/TMC4671/TMC4671.h"
#include "external/trinamic/tmc/ic/TMC4671/TMC4671_Variants.h"
#include "external/trinamic/tmc/ic/TMC6100/TMC6100.h"
}

// SPI Configs
static uint32_t SPI_SPEED_HZ = 1000000;  // 1 Mhz
static uint8_t SPI_BITS      = 8;
static uint32_t SPI_MODE     = 0x3u;

// SPI Chip Selects
static const uint32_t FRONT_LEFT_MOTOR_CHIP_SELECT  = 0;
static const uint32_t FRONT_RIGHT_MOTOR_CHIP_SELECT = 3;
static const uint32_t BACK_LEFT_MOTOR_CHIP_SELECT   = 2;
static const uint32_t BACK_RIGHT_MOTOR_CHIP_SELECT  = 1;
static const uint32_t DRIBBLER_MOTOR_CHIP_SELECT    = 4;
static const uint32_t TOTAL_NUMBER_OF_MOTORS        = 5;

// SPI Trinamic Motor Driver Paths (indexed with chip select above)
static const char* SPI_PATHS[] = {"/dev/spidev0.0", "/dev/spidev0.1", "/dev/spidev0.2",
                                  "/dev/spidev0.3", "/dev/spidev0.4"};

static const char* SPI_CS_DRIVER_TO_CONTROLLER_MUX_GPIO = "77";
static const char* DRIVER_CONTROL_ENABLE_GPIO           = "216";

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
                           const WheelConstants_t& wheel_constants)
    : spi_cs_driver_to_controller_demux_gpio(SPI_CS_DRIVER_TO_CONTROLLER_MUX_GPIO,
                                             GpioDirection::OUTPUT, GpioState::LOW),
      driver_control_enable_gpio(DRIVER_CONTROL_ENABLE_GPIO, GpioDirection::OUTPUT,
                                 GpioState::LOW)
{
    robot_constants_ = robot_constants;
    wheel_constants_ = wheel_constants;

    int ret = 0;

    /**
     * Opens SPI File Descriptor
     *
     * @param motor_name The name of the motor the spi path is connected to
     * @param chip_select Which chip select to use
     */
#define OPEN_SPI_FILE_DESCRIPTOR(motor_name, chip_select)                                \
                                                                                         \
    file_descriptors[chip_select] = open(SPI_PATHS[chip_select], O_RDWR);                \
    CHECK(file_descriptors[chip_select] >= 0)                                            \
        << "can't open device: " << #motor_name << "error: " << strerror(errno);         \
                                                                                         \
    ret = ioctl(file_descriptors[chip_select], SPI_IOC_WR_MODE32, &SPI_MODE);            \
    CHECK(ret != -1) << "can't set spi mode for: " << #motor_name                        \
                     << "error: " << strerror(errno);                                    \
                                                                                         \
    ret = ioctl(file_descriptors[chip_select], SPI_IOC_WR_BITS_PER_WORD, &SPI_BITS);     \
    CHECK(ret != -1) << "can't set bits_per_word for: " << #motor_name                   \
                     << "error: " << strerror(errno);                                    \
                                                                                         \
    ret = ioctl(file_descriptors[chip_select], SPI_IOC_WR_MAX_SPEED_HZ, &SPI_SPEED_HZ);  \
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

std::unique_ptr<TbotsProto::DriveUnitStatus> MotorService::poll(
    const TbotsProto::DirectControlPrimitive& direct_control)
{
    // TODO (#2335) We can only spin 1 motor right now
    CHECK(encoder_calibrated_[FRONT_LEFT_MOTOR_CHIP_SELECT])
        << "Running without encoder calibration can cause serious harm";

    switch (direct_control.wheel_control_case())
    {
        case TbotsProto::DirectControlPrimitive::WheelControlCase::kDirectPerWheelControl:
        {
            // TODO (#2456) until we figure out the right factor considering
            // the gear ratio
            static float RANDOM_SCALING_FACTOR = 100.0;

            // TODO (#2456) We can only spin 1 motor right now, figure out how
            // to spin the rest and update the following code
            tmc4671_setTargetVelocity(
                FRONT_LEFT_MOTOR_CHIP_SELECT,
                static_cast<int>(
                    direct_control.direct_per_wheel_control().front_left_wheel_rpm() *
                    RANDOM_SCALING_FACTOR));

            break;
        }
        case TbotsProto::DirectControlPrimitive::WheelControlCase::kDirectVelocityControl:
        {
            // TODO (#2335) convert local velocity to per-wheel velocity
            // using http://robocup.mi.fu-berlin.de/buch/omnidrive.pdf and then
            // communicate velocities to trinamic.
            break;
        }
        case TbotsProto::DirectControlPrimitive::WheelControlCase::WHEEL_CONTROL_NOT_SET:
        {
            LOG(WARNING) << "Motor service polled with an empty DirectControlPrimitive ";
            break;
        }
    }

    return std::make_unique<TbotsProto::DriveUnitStatus>();
}

void MotorService::spiTransfer(int fd, uint8_t const* tx, uint8_t const* rx, unsigned len)
{
    int ret;

    struct spi_ioc_transfer tr[1];
    memset(tr, 0, sizeof(tr));

    tr[0].tx_buf        = (unsigned long)tx;
    tr[0].rx_buf        = (unsigned long)rx;
    tr[0].len           = len;
    tr[0].delay_usecs   = 0;
    tr[0].speed_hz      = SPI_SPEED_HZ;
    tr[0].bits_per_word = 8;

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);

    CHECK(ret >= 1) << "SPI Transfer to motor failed, not safe to proceed: errno "
                    << strerror(errno);
}

// Both the TMC4671 (the controller) and the TMC6100 (the driver) respect
// the same SPI interface. So when we bind the API, we can use the same
// readWriteByte function, provided that the chip select pin is turning on
// the right chip.
//
// Each TMC4671 and TMC6100 pair have their chip selects coming in from a
// demux (see diagram below). The demux is controlled by the
// spi_cs_driver_to_controller_demux_gpio. When low, the chip select is passed
// to the TMC4671, and to the TMC6100 when high.
//
//
//                                               FRONT LEFT MOTOR
//                                              CONTROLLER + DRIVER
//                                             ┌─────────────────┐
//                                             │                 │
//                            ┌───────┐        │   ┌─────────┐   │
//                            │       │ SEL(LOW)   │         │   │
//                            │  1:2  ├────────┬───►TMC4671  │   │
//                            │       │        │   └─────────┘   │
//             FRONT_LEFT_CS  │ DEMUX │        │                 │
//             ───────────────►       │        │   ┌─────────┐   │
//                            │       │SEL(HIGH)   │         │   │
//                            │       ├────────┬───►TMC6100  │   │
//                            │       │        │   └─────────┘   │
//                            │       │        │                 │
//                            └───▲───┘        └─────────────────┘
//                                │
//                                │ SEL
//                                │
//                  spi_cs_driver_to_controller_demux
//
uint8_t MotorService::tmc4671ReadWriteByte(uint8_t motor, uint8_t data,
                                           uint8_t last_transfer)
{
    spi_cs_driver_to_controller_demux_gpio.setValue(GpioState::LOW);
    return readWriteByte(motor, data, last_transfer);
}

uint8_t MotorService::tmc6100ReadWriteByte(uint8_t motor, uint8_t data,
                                           uint8_t last_transfer)
{
    spi_cs_driver_to_controller_demux_gpio.setValue(GpioState::HIGH);
    return readWriteByte(motor, data, last_transfer);
}

uint8_t MotorService::readWriteByte(uint8_t motor, uint8_t data, uint8_t last_transfer)
{
    uint8_t ret_byte = 0;

    if (!transfer_started)
    {
        memset(tx, 0, sizeof(tx));
        memset(rx, 0, sizeof(rx));
        position = 0;

        if (data & TMC_WRITE_BIT)
        {
            // If the transfer started and its a write operation,
            // set the appropriate flags.
            currently_reading = false;
            currently_writing = true;
        }
        else
        {
            // The first byte should contain the address on a read operation.
            // Trigger a transfer (1 byte) and buffer the response (4 bytes)
            tx[position] = data;
            spiTransfer(file_descriptors[motor], tx, rx, 5);

            currently_reading = true;
            currently_writing = false;
        }

        transfer_started = true;
    }

    if (currently_writing)
    {
        // Buffer the data to send out when last_transfer is true.
        tx[position++] = data;
    }

    if (currently_reading)
    {
        // If we are reading, we just need to return the buffered data
        // byte by byte.
        ret_byte = rx[position++];
    }

    if (currently_writing && last_transfer)
    {
        // we have all the bytes for this transfer, lets trigger the transfer and
        // reset state
        spiTransfer(file_descriptors[motor], tx, rx, 5);
        transfer_started = false;
    }

    if (currently_reading && last_transfer)
    {
        // when reading, if last transfer is true, we just need to reset state
        transfer_started = false;
    }

    return ret_byte;
}

void MotorService::writeToDriverOrDieTrying(uint8_t motor, uint8_t address, int32_t value)
{
    tmc6100_writeInt(motor, address, value);
    int read_value = tmc6100_readInt(motor, address);
    CHECK(read_value == value) << "Couldn't write " << value
                               << " to the TMC6100 at address "
                               << static_cast<uint32_t>(address) << " on motor "
                               << static_cast<uint32_t>(motor);
}

void MotorService::writeToControllerOrDieTrying(uint8_t motor, uint8_t address,
                                                int32_t value)
{
    tmc4671_writeInt(motor, address, value);
    int read_value = tmc4671_readInt(motor, address);
    CHECK(read_value == value) << "Couldn't write " << value
                               << " to the TMC4671 at address " << address << " on motor "
                               << static_cast<uint32_t>(address) << " on motor "
                               << static_cast<uint32_t>(motor);
}

void MotorService::configurePWM(uint8_t motor)
{
    // Please read the header file and the datasheet for more info
    writeToControllerOrDieTrying(motor, TMC4671_PWM_POLARITIES, 0x00000000);
    writeToControllerOrDieTrying(motor, TMC4671_PWM_MAXCNT, 0x00000F9F);
    writeToControllerOrDieTrying(motor, TMC4671_PWM_BBM_H_BBM_L, 0x00002828);
    writeToControllerOrDieTrying(motor, TMC4671_PWM_SV_CHOP, 0x00000107);
}

void MotorService::configurePI(uint8_t motor)
{
    // Please read the header file and the datasheet for more info
    writeToControllerOrDieTrying(motor, TMC4671_PID_FLUX_P_FLUX_I, 67109376);
    writeToControllerOrDieTrying(motor, TMC4671_PID_TORQUE_P_TORQUE_I, 67109376);
    writeToControllerOrDieTrying(motor, TMC4671_PID_VELOCITY_P_VELOCITY_I, 52428800);
    writeToControllerOrDieTrying(motor, TMC4671_PID_POSITION_P_POSITION_I, 0);

    writeToControllerOrDieTrying(motor, TMC4671_PID_TORQUE_FLUX_TARGET_DDT_LIMITS, 0);
    writeToControllerOrDieTrying(motor, TMC4671_PIDOUT_UQ_UD_LIMITS, 32767);
    writeToControllerOrDieTrying(motor, TMC4671_PID_TORQUE_FLUX_LIMITS, 5000);
    writeToControllerOrDieTrying(motor, TMC4671_PID_ACCELERATION_LIMIT, 1000);
    writeToControllerOrDieTrying(motor, TMC4671_PID_VELOCITY_LIMIT, 10000);
}

void MotorService::configureADC(uint8_t motor)
{
    // ADC configuration
    writeToControllerOrDieTrying(motor, TMC4671_ADC_I_SELECT, 0x18000100);
    writeToControllerOrDieTrying(motor, TMC4671_dsADC_MDEC_B_MDEC_A, 0x014E014E);
    writeToControllerOrDieTrying(motor, TMC4671_ADC_I0_SCALE_OFFSET, 0x010081DD);
    writeToControllerOrDieTrying(motor, TMC4671_ADC_I1_SCALE_OFFSET, 0x0100818E);
}

void MotorService::configureEncoder(uint8_t motor)
{
    // ABN encoder settings
    writeToControllerOrDieTrying(motor, TMC4671_ABN_DECODER_MODE, 0x00000000);
    writeToControllerOrDieTrying(motor, TMC4671_ABN_DECODER_PPR, 0x00001000);
}

void MotorService::calibrateEncoder(uint8_t motor)
{
    LOG(WARNING) << "Calibrating the encoder, ensure the robot is lifted off the ground";
    sleep(1);

    writeToControllerOrDieTrying(motor, TMC4671_PID_TORQUE_FLUX_LIMITS, 0x000003E8);
    writeToControllerOrDieTrying(motor, TMC4671_PID_TORQUE_P_TORQUE_I, 0x01000100);
    writeToControllerOrDieTrying(motor, TMC4671_PID_FLUX_P_FLUX_I, 0x01000100);

    writeToControllerOrDieTrying(motor, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008);
    writeToControllerOrDieTrying(motor, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET,
                                 0x00000000);
    writeToControllerOrDieTrying(motor, TMC4671_PHI_E_SELECTION, 0x00000001);
    writeToControllerOrDieTrying(motor, TMC4671_PHI_E_EXT, 0x00000000);
    writeToControllerOrDieTrying(motor, TMC4671_UQ_UD_EXT, 0x000007D0);
    sleep(1);

    writeToControllerOrDieTrying(motor, TMC4671_ABN_DECODER_COUNT, 0x00000000);
    writeToControllerOrDieTrying(motor, TMC4671_UQ_UD_EXT, 0x00000000);
    writeToControllerOrDieTrying(motor, TMC4671_PHI_E_SELECTION, TMC4671_PHI_E_ABN);

    encoder_calibrated_[motor] = true;
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
    tmc4671_writeInt(motor, TMC4671_PHI_E_SELECTION, 0x00000002);
    tmc4671_writeInt(motor, TMC4671_UQ_UD_EXT, 0x00000779);

    // Switch to open loop velocity mode
    tmc4671_writeInt(motor, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008);

    // Rotate right
    tmc4671_writeInt(motor, TMC4671_OPENLOOP_VELOCITY_TARGET, 0x0000004A);

    // Setup CSVs
    LOG(CSV, "encoder_calibration.csv") << "actual_encoder,estimated_phi\n";
    LOG(CSV, "phase_currents_and_voltages.csv")
        << "adc_iv,adc_ux,adc_wy,pwm_iv,pwm_ux,pwm_wy\n";

    // Take samples of the useful registers
    for (size_t num_sample = 0; num_sample < num_samples; num_sample++)
    {
        int estimated_phi  = tmc4671_readInt(motor, TMC4671_OPENLOOP_PHI);
        int actual_encoder = tmc4671_readRegister16BitValue(
            motor, TMC4671_ABN_DECODER_PHI_E_PHI_M, BIT_16_TO_31);

        LOG(CSV, "encoder_calibration.csv")
            << actual_encoder << "," << estimated_phi << "\n";

        int16_t adc_iv =
            tmc4671_readRegister16BitValue(motor, TMC4671_ADC_IV, BIT_0_TO_15);
        int16_t adc_ux =
            tmc4671_readRegister16BitValue(motor, TMC4671_ADC_IWY_IUX, BIT_0_TO_15);
        int16_t adc_wy =
            tmc4671_readRegister16BitValue(motor, TMC4671_ADC_IWY_IUX, BIT_16_TO_31);

        tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, INTERIM_ADDR_PWM_UV);
        int16_t pwm_iv =
            tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15);

        tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, INTERIM_ADDR_PWM_WY_UX);
        int16_t pwm_ux =
            tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15);
        int16_t pwm_wy =
            tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_16_TO_31);

        LOG(CSV, "phase_currents_and_voltages.csv")
            << adc_iv << "," << adc_ux << "," << adc_wy << "," << pwm_iv << "," << pwm_ux
            << "," << pwm_wy << "\n";
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
    writeToDriverOrDieTrying(motor, TMC6100_GCONF, 32);
}

void MotorService::startController(uint8_t motor)
{
    // Read the chip ID to validate the SPI connection
    tmc4671_writeInt(motor, TMC4671_CHIPINFO_ADDR, 0x000000000);
    CHECK(0x34363731 == tmc4671_readInt(motor, TMC4671_CHIPINFO_DATA))
        << "The TMC4671 of motor " << static_cast<uint32_t>(motor)
        << " is not responding";

    // Configure to brushless DC motor with 8 pole pairs
    writeToControllerOrDieTrying(motor, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, 0x00030008);

    // Configure other controller params
    configurePWM(motor);
    configureADC(motor);
    configureEncoder(motor);

    // Trigger encoder calibration
    // TODO (#2451) Don't call this here, its not safe because it moves the motors
    calibrateEncoder(motor);
    configurePI(motor);
}

void MotorService::start()
{
    // Enable the driver
    driver_control_enable_gpio.setValue(GpioState::HIGH);

    // TMC6100 Setup
    startDriver(FRONT_LEFT_MOTOR_CHIP_SELECT);

    // TMC4671 Setup
    startController(FRONT_LEFT_MOTOR_CHIP_SELECT);
}

void MotorService::stop()
{
    driver_control_enable_gpio.setValue(GpioState::LOW);
}
