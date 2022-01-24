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
#include <unistd.h>

#include "proto/tbots_software_msgs.pb.h"
#include "software/logger/logger.h"

#define POSITION_SCALE_MAX (int32_t)65536
#include <stdint.h>

extern "C"
{
#include "external/trinamic/tmc/ic/TMC4671/TMC4671.h"
#include "external/trinamic/tmc/ic/TMC6100/TMC6100.h"

    // We need a static pointer here, because trinamic externs the following two
    // SPI binding functions that we need to interface with their API.
    //
    // The motor service exclusively calls the trinamic API which triggers these
    // functions. The motor service will set this variable in the constructor.
    static MotorService* g_motor_service = NULL;

    uint8_t tmc4671_readwriteByte(uint8_t motor, uint8_t data, uint8_t lastTransfer)
    {
        return g_motor_service->tmc4671ReadWriteByte(motor, data, lastTransfer);
    }

    uint8_t tmc6100_readwriteByte(uint8_t motor, uint8_t data, uint8_t lastTransfer)
    {
        return g_motor_service->tmc6100ReadWriteByte(motor, data, lastTransfer);
    }
}

// SPI Configs
static uint32_t SPI_SPEED_HZ = 200000;
static uint8_t SPI_BITS      = 8;
static uint32_t SPI_MODE     = 0x3u;

// SPI Chip Selects
static const uint32_t FRONT_LEFT_MOTOR_CHIP_SELECT  = 0;
static const uint32_t FRONT_RIGHT_MOTOR_CHIP_SELECT = 1;
static const uint32_t BACK_LEFT_MOTOR_CHIP_SELECT   = 2;
static const uint32_t BACK_RIGHT_MOTOR_CHIP_SELECT  = 3;
static const uint32_t DRIBBLER_MOTOR_CHIP_SELECT    = 4;
static const uint32_t TOTAL_NUMBER_OF_MOTORS        = 5;

// SPI Trinamic Motor Driver Paths (indexed with chip select above)
static const char* SPI_PATHS[] = {"/dev/spidev0.0", "/dev/spidev0.1", "/dev/spidev0.2",
                                  "/dev/spidev0.3", "/dev/spidev0.4"};

static const char* SPI_CS_DRIVER_TO_CONTROLLER_MUX_GPIO = "77";
static const char* DRIVER_CONTROL_ENABLE_GPIO           = "216";


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
    if (file_descriptors[chip_select] < 0)                                               \
    {                                                                                    \
        LOG(FATAL) << "can't open device: " << #motor_name;                              \
    }                                                                                    \
                                                                                         \
    ret = ioctl(file_descriptors[chip_select], SPI_IOC_WR_MODE32, &SPI_MODE);            \
    if (ret == -1)                                                                       \
    {                                                                                    \
        LOG(FATAL) << "can't set spi mode for: " << #motor_name;                         \
    }                                                                                    \
                                                                                         \
    ret = ioctl(file_descriptors[chip_select], SPI_IOC_WR_BITS_PER_WORD, &SPI_BITS);     \
    if (ret == -1)                                                                       \
    {                                                                                    \
        LOG(FATAL) << "can't set bits for: " << #motor_name;                             \
    }                                                                                    \
                                                                                         \
    ret = ioctl(file_descriptors[chip_select], SPI_IOC_WR_MAX_SPEED_HZ, &SPI_SPEED_HZ);  \
    if (ret == -1)                                                                       \
    {                                                                                    \
        LOG(FATAL) << "can't set max speed hz for: " << #motor_name;                     \
    }

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
    const TbotsProto::DirectControlPrimitive_DirectVelocityControl& local_velocity,
    float dribbler_speed_rpm)
{
    LOG(DEBUG) << "WAITING FOR ENCODER INIT";
    sleep(10);
    LOG(DEBUG) << "ENCODER INIT";
    LOG(DEBUG) << "Haven't moved yet";

    // Simone Parameters
    tmc4671_writeInt(0, TMC4671_PID_FLUX_P_FLUX_I, 67109376);
    tmc4671_writeInt(0, TMC4671_PID_TORQUE_P_TORQUE_I, 67109376);
    tmc4671_writeInt(0, TMC4671_PID_VELOCITY_P_VELOCITY_I, 52428800);
    tmc4671_writeInt(0, TMC4671_PID_POSITION_P_POSITION_I, 0);

    tmc4671_writeInt(0, TMC4671_PID_TORQUE_FLUX_TARGET_DDT_LIMITS, 0);
    tmc4671_writeInt(0, TMC4671_PIDOUT_UQ_UD_LIMITS, 32767);
    tmc4671_writeInt(0, TMC4671_PID_TORQUE_FLUX_LIMITS, 5000);
    tmc4671_writeInt(0, TMC4671_PID_ACCELERATION_LIMIT, 15000);
    tmc4671_writeInt(0, TMC4671_PID_VELOCITY_LIMIT, 4000);
    tmc4671_writeInt(0, TMC4671_PID_POSITION_LIMIT_LOW, -2147483647);
    tmc4671_writeInt(0, TMC4671_PID_POSITION_LIMIT_HIGH, 2147483647);

    sleep(2);
    LOG(DEBUG) << "ENCODER INIT";
    LOG(DEBUG) << "Haven't moved yet";
    assert(tmc4671_readInt(0, TMC4671_PID_FLUX_P_FLUX_I) == 67109376);
    assert(tmc4671_readInt(0, TMC4671_PID_TORQUE_P_TORQUE_I) == 67109376);
    assert(tmc4671_readInt(0, TMC4671_PID_VELOCITY_P_VELOCITY_I) == 52428800);
    assert(tmc4671_readInt(0, TMC4671_PID_POSITION_P_POSITION_I) == 0);
    assert(tmc4671_readInt(0, TMC4671_PID_TORQUE_FLUX_TARGET_DDT_LIMITS) == 0);
    assert(tmc4671_readInt(0, TMC4671_PIDOUT_UQ_UD_LIMITS) == 32767);
    assert(tmc4671_readInt(0, TMC4671_PID_TORQUE_FLUX_LIMITS) == 5000);
    assert(tmc4671_readInt(0, TMC4671_PID_ACCELERATION_LIMIT) == 15000);
    assert(tmc4671_readInt(0, TMC4671_PID_VELOCITY_LIMIT) == 4000);
    assert(tmc4671_readInt(0, TMC4671_PID_POSITION_LIMIT_LOW) == -2147483647);
    assert(tmc4671_readInt(0, TMC4671_PID_POSITION_LIMIT_HIGH) == 2147483647);

    LOG(WARNING) << tmc4671_readInt(0, TMC6100_GSTAT);
    LOG(DEBUG) << tmc4671_readInt(0, TMC4671_PID_FLUX_P_FLUX_I);
    LOG(DEBUG) << tmc4671_readInt(0, TMC4671_PID_TORQUE_P_TORQUE_I);
    LOG(DEBUG) << tmc4671_readInt(0, TMC4671_PID_VELOCITY_P_VELOCITY_I);
    LOG(DEBUG) << tmc4671_readInt(0, TMC4671_PID_POSITION_P_POSITION_I);
    LOG(DEBUG) << tmc4671_readInt(0, TMC4671_PID_TORQUE_FLUX_TARGET_DDT_LIMITS);
    LOG(DEBUG) << tmc4671_readInt(0, TMC4671_PIDOUT_UQ_UD_LIMITS);
    LOG(DEBUG) << tmc4671_readInt(0, TMC4671_PID_TORQUE_FLUX_LIMITS);
    LOG(DEBUG) << tmc4671_readInt(0, TMC4671_PID_ACCELERATION_LIMIT);
    LOG(DEBUG) << tmc4671_readInt(0, TMC4671_PID_VELOCITY_LIMIT);
    LOG(DEBUG) << tmc4671_readInt(0, TMC4671_PID_POSITION_LIMIT_LOW);
    LOG(DEBUG) << tmc4671_readInt(0, TMC4671_PID_POSITION_LIMIT_HIGH);

    LOG(WARNING) << tmc4671_readInt(0, TMC6100_GSTAT);
    tmc4671_setTargetVelocity(0, 500);
    LOG(DEBUG) << "Moved clockwise";

    LOG(DEBUG) << tmc4671_readInt(0, TMC4671_PID_FLUX_P_FLUX_I);
    LOG(DEBUG) << tmc4671_readInt(0, TMC4671_PID_TORQUE_P_TORQUE_I);
    LOG(DEBUG) << tmc4671_readInt(0, TMC4671_PID_VELOCITY_P_VELOCITY_I);
    LOG(DEBUG) << tmc4671_readInt(0, TMC4671_PID_POSITION_P_POSITION_I);
    LOG(DEBUG) << tmc4671_readInt(0, TMC4671_PID_TORQUE_FLUX_TARGET_DDT_LIMITS);
    LOG(DEBUG) << tmc4671_readInt(0, TMC4671_PIDOUT_UQ_UD_LIMITS);
    LOG(DEBUG) << tmc4671_readInt(0, TMC4671_PID_TORQUE_FLUX_LIMITS);
    LOG(DEBUG) << tmc4671_readInt(0, TMC4671_PID_ACCELERATION_LIMIT);
    LOG(DEBUG) << tmc4671_readInt(0, TMC4671_PID_VELOCITY_LIMIT);
    LOG(DEBUG) << tmc4671_readInt(0, TMC4671_PID_POSITION_LIMIT_LOW);
    LOG(DEBUG) << tmc4671_readInt(0, TMC4671_PID_POSITION_LIMIT_HIGH);
    LOG(WARNING) << tmc4671_readInt(0, TMC6100_GSTAT);
    assert(tmc4671_readInt(0, TMC4671_PID_FLUX_P_FLUX_I) == 67109376);
    assert(tmc4671_readInt(0, TMC4671_PID_TORQUE_P_TORQUE_I) == 67109376);
    assert(tmc4671_readInt(0, TMC4671_PID_VELOCITY_P_VELOCITY_I) == 52428800);
    assert(tmc4671_readInt(0, TMC4671_PID_POSITION_P_POSITION_I) == 0);
    assert(tmc4671_readInt(0, TMC4671_PID_TORQUE_FLUX_TARGET_DDT_LIMITS) == 0);
    assert(tmc4671_readInt(0, TMC4671_PIDOUT_UQ_UD_LIMITS) == 32767);
    assert(tmc4671_readInt(0, TMC4671_PID_TORQUE_FLUX_LIMITS) == 5000);
    assert(tmc4671_readInt(0, TMC4671_PID_ACCELERATION_LIMIT) == 15000);
    assert(tmc4671_readInt(0, TMC4671_PID_VELOCITY_LIMIT) == 4000);
    assert(tmc4671_readInt(0, TMC4671_PID_POSITION_LIMIT_LOW) == -2147483647);
    assert(tmc4671_readInt(0, TMC4671_PID_POSITION_LIMIT_HIGH) == 2147483647);
    sleep(2);
    LOG(DEBUG) << "Moving counter clockwise";
    tmc4671_setTargetVelocity(0, -500);

    assert(tmc4671_readInt(0, TMC4671_PID_FLUX_P_FLUX_I) == 67109376);
    assert(tmc4671_readInt(0, TMC4671_PID_TORQUE_P_TORQUE_I) == 67109376);
    assert(tmc4671_readInt(0, TMC4671_PID_VELOCITY_P_VELOCITY_I) == 52428800);
    assert(tmc4671_readInt(0, TMC4671_PID_POSITION_P_POSITION_I) == 0);
    assert(tmc4671_readInt(0, TMC4671_PID_TORQUE_FLUX_TARGET_DDT_LIMITS) == 0);
    assert(tmc4671_readInt(0, TMC4671_PIDOUT_UQ_UD_LIMITS) == 32767);
    assert(tmc4671_readInt(0, TMC4671_PID_TORQUE_FLUX_LIMITS) == 5000);
    assert(tmc4671_readInt(0, TMC4671_PID_ACCELERATION_LIMIT) == 15000);
    assert(tmc4671_readInt(0, TMC4671_PID_VELOCITY_LIMIT) == 4000);
    assert(tmc4671_readInt(0, TMC4671_PID_POSITION_LIMIT_LOW) == -2147483647);
    assert(tmc4671_readInt(0, TMC4671_PID_POSITION_LIMIT_HIGH) == 2147483647);
    sleep(2);
    LOG(DEBUG) << "Stopping";
    tmc4671_setTargetVelocity(0, 0);
    sleep(2);

    // TODO (#2335) convert local velocity to per-wheel velocity
    // using http://robocup.mi.fu-berlin.de/buch/omnidrive.pdf and then
    // communicate velocities to trinamic. Also read back feedback and
    // return drive unit status.
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

    if (ret < 1)
    {
        LOG(FATAL) << "SPI Transfer to motor failed, not safe to proceed";
    }
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

void MotorService::start()
{
    // TMC6100 INIT
    tmc6100_writeInt(FRONT_LEFT_MOTOR_CHIP_SELECT, TMC6100_GCONF, 32);
    uint32_t result = tmc6100_readInt(FRONT_LEFT_MOTOR_CHIP_SELECT, TMC6100_GCONF);
    assert(32 == result);
    LOG(DEBUG) << "power stage online";

    // Motor type & PWM configuration
    tmc4671_writeInt(0, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, 0x00030008);
    tmc4671_writeInt(0, TMC4671_PWM_POLARITIES, 0x00000000);
    tmc4671_writeInt(0, TMC4671_PWM_MAXCNT, 0x00000F9F);
    tmc4671_writeInt(0, TMC4671_PWM_BBM_H_BBM_L, 0x00001919);
    tmc4671_writeInt(0, TMC4671_PWM_SV_CHOP, 0x00000107);

    // ADC configuration
    tmc4671_writeInt(0, TMC4671_ADC_I_SELECT, 0x09000100);
    tmc4671_writeInt(0, TMC4671_dsADC_MCFG_B_MCFG_A, 0x00100010);
    tmc4671_writeInt(0, TMC4671_dsADC_MCLK_A, 0x20000000);
    tmc4671_writeInt(0, TMC4671_dsADC_MCLK_B, 0x00000000);
    tmc4671_writeInt(0, TMC4671_dsADC_MDEC_B_MDEC_A, 0x014E014E);
    tmc4671_writeInt(0, TMC4671_ADC_I0_SCALE_OFFSET, 0x010081DD);
    tmc4671_writeInt(0, TMC4671_ADC_I1_SCALE_OFFSET, 0x0100818E);

    // Open loop settings
    tmc4671_writeInt(0, TMC4671_OPENLOOP_MODE, 0x00000000);
    tmc4671_writeInt(0, TMC4671_OPENLOOP_ACCELERATION, 0x0000003C);
    tmc4671_writeInt(0, TMC4671_OPENLOOP_VELOCITY_TARGET, 0xFFFFFFFB);

    // Feedback selection
    tmc4671_writeInt(0, TMC4671_PHI_E_SELECTION, 0x00000002);
    tmc4671_writeInt(0, TMC4671_UQ_UD_EXT, 0x00000779);

    // ===== Open loop test drive =====
    // Switch to open loop velocity mode
    tmc4671_writeInt(0, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008);

    // Rotate right
    tmc4671_writeInt(0, TMC4671_OPENLOOP_VELOCITY_TARGET, 0x0000003C);
    sleep(2);

    // Rotate left
    tmc4671_writeInt(0, TMC4671_OPENLOOP_VELOCITY_TARGET, 0xFFFFFFC4);
    sleep(4);

    // Stop
    tmc4671_writeInt(0, TMC4671_OPENLOOP_VELOCITY_TARGET, 0x00000000);
    sleep(2);
    tmc4671_writeInt(0, TMC4671_UQ_UD_EXT, 0x00000000);


    // ABN encoder settings
    tmc4671_writeInt(0, TMC4671_ABN_DECODER_MODE, 0x00001000);
    tmc4671_writeInt(0, TMC4671_ABN_DECODER_PPR, 0x00001000);
    tmc4671_writeInt(0, TMC4671_ABN_DECODER_COUNT, 0x0000099B);
    tmc4671_writeInt(0, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x00000000);


    // ABN encoder settings
    LOGF(DEBUG, "Motor type configured to : %x",
         tmc4671_getMotorType(FRONT_LEFT_MOTOR_CHIP_SELECT));
    LOGF(DEBUG, "Pole pairs configured to : %x",
         tmc4671_getPolePairs(FRONT_LEFT_MOTOR_CHIP_SELECT));

    for (uint32_t motor = 0; motor < 1; motor++)
    {
        motor_state_[motor].init_wait_time            = 400;
        motor_state_[motor].start_voltage             = 8000;  // UQ_UD_EXT
        motor_state_[motor].encoder_init_mode         = 0;
        motor_state_[motor].encoder_init_state        = 0;
        motor_state_[motor].hall_PHI_E_old            = 0;
        motor_state_[motor].hall_PHI_E_new            = 0;
        motor_state_[motor].hall_actual_coarse_offset = 0;
        motor_state_[motor].last_PHI_E_selection      = 0;
        motor_state_[motor].last_UQ_UD_EXT            = 0;
        motor_state_[motor].last_PHI_E_EXT            = 0;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
        tmc4671_writeInt(motor, TMC4671_PHI_E_SELECTION, TMC4671_PHI_E_ABN);
        assert(tmc4671_readInt(motor, TMC4671_PHI_E_SELECTION) == TMC4671_PHI_E_ABN);
#pragma GCC diagnostic pop
        tmc4671_startEncoderInitialization(0, &(motor_state_[motor].encoder_init_mode),
                                           &(motor_state_[motor].encoder_init_state));
    }
}


void MotorService::periodicJob(uint32_t ms_tick)
{
    for (uint32_t motor = 0; motor < TOTAL_NUMBER_OF_MOTORS; motor++)
    {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
        tmc4671_periodicJob(
            motor, ms_tick, motor_state_[motor].encoder_init_mode,
            &(motor_state_[motor].encoder_init_state), motor_state_[motor].init_wait_time,
            &(motor_state_[motor].actual_init_wait_time),
            motor_state_[motor].start_voltage, &(motor_state_[motor].hall_PHI_E_old),
            &(motor_state_[motor].hall_PHI_E_new),
            &(motor_state_[motor].hall_actual_coarse_offset),
            &(motor_state_[motor].last_PHI_E_selection),
            &(motor_state_[motor].last_UQ_UD_EXT), &(motor_state_[motor].last_PHI_E_EXT));
    }
#pragma GCC diagnostic pop
}

void MotorService::stop()
{
    // TODO (#2332)
}
