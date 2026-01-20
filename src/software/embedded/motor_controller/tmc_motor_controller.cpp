#include "software/embedded/motor_controller/tmc_motor_controller.h"

#include "shared/constants.h"
#include "software/embedded/spi_utils.h"
#include "software/logger/logger.h"

extern "C"
{
#include "tmc/ic/TMC4671/TMC4671.h"
#include "tmc/ic/TMC4671/TMC4671_Register.h"
#include "tmc/ic/TMC4671/TMC4671_Variants.h"
#include "tmc/ic/TMC6100/TMC6100.h"
}

#include <linux/spi/spidev.h>

#include <bitset>
#include <numeric>

extern "C"
{
    // We need a static pointer here, because trinamic externs the following two
    // SPI binding functions that we need to interface with their API.
    //
    // The motor service exclusively calls the trinamic API which triggers these
    // functions. The motor service will set this variable in the constructor.
    static TmcMotorController* g_motor_controller = NULL;

    uint8_t tmc4671_readwriteByte(uint8_t motor, uint8_t data, uint8_t last_transfer)
    {
        return g_motor_controller->tmc4671ReadWriteByte(motor, data, last_transfer);
    }

    uint8_t tmc6100_readwriteByte(uint8_t motor, uint8_t data, uint8_t last_transfer)
    {
        return g_motor_controller->tmc6100ReadWriteByte(motor, data, last_transfer);
    }
}

TmcMotorController::TmcMotorController()
    : spi_demux_select_0_(setupGpio(SPI_CS_DRIVER_TO_CONTROLLER_MUX_0_GPIO,
                                    GpioDirection::OUTPUT, GpioState::LOW)),
      spi_demux_select_1_(setupGpio(SPI_CS_DRIVER_TO_CONTROLLER_MUX_1_GPIO,
                                    GpioDirection::OUTPUT, GpioState::LOW)),
      driver_control_enable_gpio_(
          setupGpio(DRIVER_CONTROL_ENABLE_GPIO, GpioDirection::OUTPUT, GpioState::HIGH)),
      reset_gpio_(
          setupGpio(MOTOR_DRIVER_RESET_GPIO, GpioDirection::OUTPUT, GpioState::HIGH))
{
    openSpiFileDescriptor(MotorIndex::FRONT_LEFT);
    openSpiFileDescriptor(MotorIndex::FRONT_RIGHT);
    openSpiFileDescriptor(MotorIndex::BACK_LEFT);
    openSpiFileDescriptor(MotorIndex::BACK_RIGHT);
    openSpiFileDescriptor(MotorIndex::DRIBBLER);

    // Make this instance available to the static functions above
    g_motor_controller = this;
}

MotorControllerStatus TmcMotorController::earlyPoll()
{
    auto motors = driveMotors();
    bool encoders_calibrated =
        std::accumulate(motors.begin(), motors.end(), false,
                        [&](const bool& acc, const MotorIndex& motor)
                        { return acc || encoder_calibrated_[motor]; });

    if (!encoders_calibrated)
    {
        return MotorControllerStatus::CALIBRATION_FAILURE;
    }

    return MotorControllerStatus::OK;
}

int TmcMotorController::readThenWriteVelocity(const MotorIndex& motor,
                                              const int& target_velocity)
{
    return readThenWriteValue(motor, TMC4671_PID_VELOCITY_ACTUAL,
                              TMC4671_PID_VELOCITY_TARGET, target_velocity);
}

void TmcMotorController::writeToDriverOrDieTrying(uint8_t motor, uint8_t address,
                                                  int32_t value)
{
    int num_retries_left = NUM_RETRIES_SPI;
    int read_value       = 0;

    // The SPI lines have a lot of noise, and sometimes a transfer will fail
    // randomly. So we retry a few times before giving up.
    while (num_retries_left > 0)
    {
        tmc6100_writeInt(motor, address, value);
        read_value = tmc6100_readInt(motor, address);
        if (read_value == value)
        {
            return;
        }
        LOG(DEBUG) << "SPI Transfer to Driver Failed, retrying...";
        num_retries_left--;
    }

    // If we get here, we have failed to write to the driver. We reset
    // the chip to clear any bad values we just wrote and crash so everything stops.
    reset_gpio_->setValue(GpioState::LOW);
    CHECK(read_value == value) << "Couldn't write " << value
                               << " to the TMC6100 at address " << address
                               << " at address " << static_cast<uint32_t>(address)
                               << " on motor " << static_cast<uint32_t>(motor)
                               << " received: " << read_value;
}

void TmcMotorController::writeToControllerOrDieTrying(const MotorIndex& motor,
                                                      uint8_t address, int32_t value)
{
    int num_retries_left = NUM_RETRIES_SPI;
    int read_value       = 0;

    // The SPI lines have a lot of noise, and sometimes a transfer will fail
    // randomly. So we retry a few times before giving up.
    while (num_retries_left > 0)
    {
        tmc4671_writeInt(CHIP_SELECTS.at(motor), address, value);
        read_value = tmc4671_readInt(CHIP_SELECTS.at(motor), address);
        if (read_value == value)
        {
            return;
        }
        LOG(DEBUG) << "SPI Transfer to Controller Failed, retrying...";
        num_retries_left--;
    }
}

void TmcMotorController::setup()
{
    reset_gpio_->setValue(GpioState::LOW);
    usleep(MICROSECONDS_PER_MILLISECOND * 100);

    reset_gpio_->setValue(GpioState::HIGH);
    usleep(MICROSECONDS_PER_MILLISECOND * 100);

    for (const MotorIndex& motor : reflective_enum::values<MotorIndex>())
    {
        LOG(INFO) << "Clearing RESET for " << motor;
        tmc6100_writeInt(CHIP_SELECTS.at(motor), TMC6100_GSTAT, 0x00000001);
        encoder_calibrated_[motor] = false;
    }

    // Drive Motor Setup
    for (const MotorIndex& motor : driveMotors())
    {
        setupDriveMotor(motor);
    }

    // Dribbler Motor Setup
    startDriver(MotorIndex::DRIBBLER);
    checkDriverFault(MotorIndex::DRIBBLER);
    startController(MotorIndex::DRIBBLER, true);
    tmc4671_setTargetVelocity(DRIBBLER_MOTOR_CHIP_SELECT, 0);

    checkEncoderConnections();

    // calibrate the encoders
    for (const MotorIndex& motor : driveMotors())
    {
        startEncoderCalibration(motor);
    }

    sleep(1);

    for (const MotorIndex& motor : driveMotors())
    {
        endEncoderCalibration(motor);
    }

    auto motors = driveMotors();
    bool has_encoders_calibrated =
        std::accumulate(motors.begin(), motors.end(), false,
                        [&](const bool& acc, const MotorIndex& motor)
                        { return acc || encoder_calibrated_[motor]; });
    CHECK(has_encoders_calibrated)
        << "Running without encoder calibration can cause serious harm, exiting";
}

void TmcMotorController::configurePWM(const MotorIndex& motor)
{
    LOG(INFO) << "Configuring PWM for motor " << static_cast<uint32_t>(motor);
    // Please read the header file and the datasheet for more info
    writeToControllerOrDieTrying(motor, TMC4671_PWM_POLARITIES, 0x00000000);
    writeToControllerOrDieTrying(motor, TMC4671_PWM_MAXCNT, 0x00000F9F);
    writeToControllerOrDieTrying(motor, TMC4671_PWM_BBM_H_BBM_L, 0x00002828);
    writeToControllerOrDieTrying(motor, TMC4671_PWM_SV_CHOP, 0x00000107);
}

void TmcMotorController::configureDrivePI(const MotorIndex& motor)
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

    tmc4671_switchToMotionMode(CHIP_SELECTS.at(motor), TMC4671_MOTION_MODE_VELOCITY);
}

void TmcMotorController::configureDribblerPI(const MotorIndex& motor)
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

void TmcMotorController::configureADC(const MotorIndex& motor)
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

void TmcMotorController::configureEncoder(const MotorIndex& motor)
{
    LOG(INFO) << "Configuring Encoder for motor " << static_cast<uint32_t>(motor);
    // ABN encoder settings
    writeToControllerOrDieTrying(motor, TMC4671_ABN_DECODER_MODE, 0x00000000);
    writeToControllerOrDieTrying(motor, TMC4671_ABN_DECODER_PPR, 0x00001000);
}

void TmcMotorController::configureHall(const MotorIndex& motor)
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

MotorFaultIndicator TmcMotorController::checkDriverFault(const MotorIndex& motor)
{
    bool drive_enabled = true;
    std::unordered_set<TbotsProto::MotorFault> motor_faults;

    int gstat = tmc6100_readInt(CHIP_SELECTS.at(motor), TMC6100_GSTAT);
    std::bitset<32> gstat_bitset(gstat);

    if (gstat_bitset.any())
    {
        LOG(WARNING) << "======= Faults For Motor " << motor << "=======";
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

int TmcMotorController::readThenWriteValue(const MotorIndex& motor,
                                           const uint8_t& read_addr,
                                           const uint8_t& write_addr,
                                           const int& write_data)
{
    spi_demux_select_0_->setValue(GpioState::HIGH);
    spi_demux_select_1_->setValue(GpioState::LOW);

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

    readThenWriteSpiTransfer(file_descriptors_[CHIP_SELECTS.at(motor)], read_tx_,
                             write_tx_, read_rx_, TMC_CMD_MSG_SIZE, TMC_CMD_MSG_SIZE,
                             TMC4671_SPI_SPEED);

    int32_t value = read_rx_[0];
    for (int i = 1; i < 5; i++)
    {
        value <<= 8;
        value |= read_rx_[i];
    }
    return value;
}

void TmcMotorController::openSpiFileDescriptor(const MotorIndex& motor_index)
{
    file_descriptors_[CHIP_SELECTS.at(motor_index)] =
        open(SPI_PATHS.at(motor_index), O_RDWR);
    CHECK(file_descriptors_[CHIP_SELECTS.at(motor_index)] >= 0)
        << "can't open device: " << motor_index << "error: " << strerror(errno);

    int ret = ioctl(file_descriptors_[CHIP_SELECTS.at(motor_index)], SPI_IOC_WR_MODE32,
                    &SPI_MODE);
    CHECK(ret != -1) << "can't set spi mode for: " << motor_index
                     << "error: " << strerror(errno);

    ret = ioctl(file_descriptors_[CHIP_SELECTS.at(motor_index)], SPI_IOC_WR_BITS_PER_WORD,
                &SPI_BITS);
    CHECK(ret != -1) << "can't set bits_per_word for: " << motor_index
                     << "error: " << strerror(errno);

    ret = ioctl(file_descriptors_[CHIP_SELECTS.at(motor_index)], SPI_IOC_WR_MAX_SPEED_HZ,
                &MAX_SPI_SPEED_HZ);
    CHECK(ret != -1) << "can't set spi max speed hz for: " << motor_index
                     << "error: " << strerror(errno);
}

void TmcMotorController::setupDriveMotor(const MotorIndex& motor)
{
    startDriver(motor);
    checkDriverFault(motor);
    // Start all the controllers as drive motor controllers
    startController(motor, false);
    tmc4671_setTargetVelocity(CHIP_SELECTS.at(motor), 0);
}

void TmcMotorController::startDriver(MotorIndex motor)
{
    uint8_t motor_cs = CHIP_SELECTS.at(motor);

    // Set the drive strength to 0, the weakest it can go as recommended
    // by the TMC4671-TMC6100-BOB datasheet.
    int32_t current_drive_conf = tmc6100_readInt(motor_cs, TMC6100_DRV_CONF);
    writeToDriverOrDieTrying(motor_cs, TMC6100_DRV_CONF,
                             current_drive_conf & (~TMC6100_DRVSTRENGTH_MASK));
    writeToDriverOrDieTrying(motor_cs, TMC6100_GCONF, 0x40);

    // All default but updated SHORTFILTER to 2us to avoid false positive shorts
    // detection.
    writeToDriverOrDieTrying(motor_cs, TMC6100_SHORT_CONF, 0x13020606);

    LOG(DEBUG) << "Driver " << motor << " accepted conf";
}

void TmcMotorController::startController(MotorIndex motor, bool dribbler)
{
    // Read the chip ID to validate the SPI connection
    tmc4671_writeInt(CHIP_SELECTS.at(motor), TMC4671_CHIPINFO_ADDR, 0x000000000);
    int chip_id = tmc4671_readInt(CHIP_SELECTS.at(motor), TMC4671_CHIPINFO_DATA);

    CHECK(0x34363731 == chip_id)
        << "The TMC4671 of motor " << motor << " is not responding";

    LOG(DEBUG) << "Controller " << motor << " online, responded with: " << chip_id;

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

uint8_t TmcMotorController::tmc4671ReadWriteByte(uint8_t motor, uint8_t data,
                                                 uint8_t last_transfer)
{
    spi_demux_select_0_->setValue(GpioState::HIGH);
    spi_demux_select_1_->setValue(GpioState::LOW);
    return readWriteByte(motor, data, last_transfer, TMC4671_SPI_SPEED);
}

uint8_t TmcMotorController::tmc6100ReadWriteByte(uint8_t motor, uint8_t data,
                                                 uint8_t last_transfer)
{
    spi_demux_select_0_->setValue(GpioState::LOW);
    spi_demux_select_1_->setValue(GpioState::HIGH);
    return readWriteByte(motor, data, last_transfer, TMC6100_SPI_SPEED);
}

uint8_t TmcMotorController::readWriteByte(uint8_t motor, uint8_t data,
                                          uint8_t last_transfer, uint32_t spi_speed)
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

void TmcMotorController::checkEncoderConnections()
{
    LOG(INFO) << "Starting encoder connection check!";

    std::unordered_map<MotorIndex, bool> calibrated_motors;
    std::unordered_map<MotorIndex, int> initial_velocities;

    for (const MotorIndex& motor : driveMotors())
    {
        calibrated_motors[motor] = false;

        // read back current velocity
        initial_velocities[motor] =
            tmc4671_readInt(CHIP_SELECTS.at(motor), TMC4671_ABN_DECODER_COUNT);

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
                     [](std::pair<const MotorIndex, bool> calibration_status_pair)
                     { return !calibration_status_pair.second; });
         ++num_iterations)
    {
        for (const MotorIndex& motor : driveMotors())
        {
            if (calibrated_motors[motor])
            {
                continue;
            }
            // now read back the velocity
            int read_back_velocity =
                tmc4671_readInt(CHIP_SELECTS.at(motor), TMC4671_ABN_DECODER_COUNT);
            LOG(INFO) << motor << " read back: " << read_back_velocity
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
    for (const MotorIndex& motor : driveMotors())
    {
        if (!calibrated_motors[motor])
        {
            calibrated = false;
            LOG(WARNING) << "Encoder calibration check failure. " << motor
                         << " did not change as expected";
        }
    }
    if (!calibrated)
    {
        LOG(FATAL)
            << "Encoder calibration check failure. Not all encoders responded as expected";
    }

    // stop all motors, reset back to velocity control mode
    for (const MotorIndex& motor : driveMotors())
    {
        writeToControllerOrDieTrying(motor, TMC4671_OPENLOOP_VELOCITY_TARGET, 0x00000000);
        tmc4671_switchToMotionMode(CHIP_SELECTS.at(motor), TMC4671_MOTION_MODE_VELOCITY);
    }

    LOG(INFO) << "All encoders appear to be connected!";
}

void TmcMotorController::reset()
{
    reset_gpio_->setValue(GpioState::LOW);
}

void TmcMotorController::startEncoderCalibration(const MotorIndex& motor)
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

void TmcMotorController::endEncoderCalibration(const MotorIndex& motor)
{
    LOG(WARNING) << "Calibrating the encoder, wheels may move";

    writeToControllerOrDieTrying(motor, TMC4671_ABN_DECODER_COUNT, 0x00000000);
    writeToControllerOrDieTrying(motor, TMC4671_UQ_UD_EXT, 0x00000000);
    writeToControllerOrDieTrying(motor, TMC4671_PHI_E_SELECTION, TMC4671_PHI_E_ABN);

    encoder_calibrated_[motor] = true;

    configureDrivePI(motor);
}

void TmcMotorController::immediatelyDisable()
{
    driver_control_enable_gpio_->setValue(GpioState::LOW);
}
