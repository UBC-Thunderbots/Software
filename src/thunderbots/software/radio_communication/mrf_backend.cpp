#include "mrf_backend.h"

#include <chrono>

namespace
{
/**
 * \brief The number of attempts to request the build IDs before giving up.
 */
const unsigned int REQUEST_BUILD_IDS_COUNT = 7;

/**
 * \brief The number of seconds to wait between consecutive requests for
 * the build IDs.
 */
const double REQUEST_BUILD_IDS_INTERVAL = 0.5;

struct RSSITableEntry final
{
    int rssi;
    int db;
};
const struct RSSITableEntry RSSI_TABLE[] = {
    {255, -35}, {254, -36}, {253, -37}, {250, -38}, {245, -39}, {239, -40},
    {233, -41}, {228, -42}, {225, -43}, {221, -44}, {216, -45}, {212, -46},
    {207, -47}, {203, -48}, {198, -49}, {193, -50}, {188, -51}, {183, -52},
    {176, -53}, {170, -54}, {165, -55}, {159, -56}, {153, -57}, {148, -58},
    {143, -59}, {138, -60}, {133, -61}, {129, -62}, {125, -63}, {121, -64},
    {117, -65}, {111, -66}, {107, -67}, {100, -68}, {95, -69},  {89, -70},
    {83, -71},  {78, -72},  {73, -73},  {68, -74},  {63, -75},  {58, -76},
    {53, -77},  {48, -78},  {43, -79},  {37, -80},  {32, -81},  {27, -82},
    {23, -83},  {18, -84},  {13, -85},  {9, -86},   {5, -87},   {2, -88},
    {1, -89},   {0, -90},
};

const char *const SD_MESSAGES[] = {
    nullptr,
    u8"Bot %1 SD card uninitialized",
    nullptr,
    u8"Bot %1 SD card incompatible",
    u8"Bot %1 SD card sent illegal response",
    u8"Bot %1 SD layer logical error",
    u8"Bot %1 SD card CRC error",
    u8"Bot %1 SD card claimed illegal command",
    u8"Bot %1 SD card in unexpected state",
    u8"Bot %1 SD card internal error",
    u8"Bot %1 SD card command response timeout",
    u8"Bot %1 SD card parameter out of range",
    u8"Bot %1 SD card address misaligned",
    u8"Bot %1 SD card block length error",
    u8"Bot %1 SD card erase sequence error",
    u8"Bot %1 SD card erase parameter error",
    u8"Bot %1 SD card write protect violation",
    u8"Bot %1 SD card locked",
    u8"Bot %1 SD card lock or unlock failed",
    u8"Bot %1 SD card command CRC error",
    u8"Bot %1 SD card ECC error",
    u8"Bot %1 SD card CC error",
    u8"Bot %1 SD card generic error",
    u8"Bot %1 SD card CSD write error",
    u8"Bot %1 SD card partial erase due to write protection",
    u8"Bot %1 SD card ECC disabled",
    u8"Bot %1 SD card erase sequence cancelled",
    u8"Bot %1 SD card authentication sequence error",
    u8"Bot %1 SD card initialization timeout",
    u8"Bot %1 SD card data timeout",
    u8"Bot %1 SD card data CRC error",
    u8"Bot %1 SD card missing data start bit",
    u8"Bot %1 SD card FIFO overrun or underrun",
};

const char *const LOGGER_MESSAGES[] = {
    nullptr, u8"Bot %1 logger uninitialized", nullptr, u8"Bot %1 SD card full",
};

MRFBackend::MRFBackend()
    : dongle(MRFDongle()),
      ball(Ball(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(1)))
{
}

MRFBackend::~MRFBackend() {}

void MRFBackend::sendPrimitives(const std::vector<std::unique_ptr<Primitive>>& primitives)
{
    dongle.send_drive_packet(primitives);
}

void MRFBackend::update_detbots(std::vector<std::tuple<uint8_t, Point, Angle>> ft)
{
    detbots = ft;
}

void MRFBackend::update_ball(Ball b)
{
    ball = b;
}

void MRFBackend::send_vision_packet()
{
    /* TODO: Change handling of timestamp depending on age of team vs ball */
    uint64_t timestamp = static_cast<uint64_t>(ball.lastUpdateTimestamp().getSeconds());
    std::cout << "Calling dongle.send_camera_packet with: ";
    for (std::size_t i = 0; i < detbots.size(); ++i)
    {
        std::cout << "bot number = " << unsigned(std::get<0>(detbots[i])) << ", ";
        std::cout << "x = " << (std::get<1>(detbots[i])).x() << ", ";
        std::cout << "y = " << (std::get<1>(detbots[i])).y() << ", ";
        std::cout << "time capture = " << timestamp << ", ";
        std::cout << "theta = " << (std::get<2>(detbots[i])).toDegrees() << std::endl;
    }
    std::cout << "ball x = " << ball.position().x() << ", ";
    std::cout << "ball y = " << ball.position().y() << std::endl;

    dongle.send_camera_packet(detbots, ball.position() * 1000, timestamp);
}

void MRFBackend::update_dongle_events()
{
    dongle.handle_libusb_events();
}

void MRFBackend::handle_message(
    int index, const void *data, std::size_t len, uint8_t lqi, uint8_t rssi)
{
    int link_quality = lqi / 255.0;
    int received_signal_strength_db;
    bool alive;

    /**
     * \brief The rough maximum full-scale deflection of the laser
     * sensor.
     */
    const double break_beam_scale;

    /**
     * \brief The maximum possible kick speed, in m/s.
     */
    const double kick_speed_max;

    /**
     * \brief The kick resolution for HScale.
     */

    const double kick_speed_resolution;

    /**
     * \brief The maximum possible chip distance, in m.
     */
    const double chip_distance_max;

    /**
     * \brief The chip resolution for HScale.
     */

    const double chip_distance_resolution;

    /**
     * \brief The maximum power level understood by the \ref
     * direct_dribbler function.
     */
    const unsigned int direct_dribbler_max;

    /**
     * \brief Whether or not the robot is currently responding to radio
     * communication.
     */
    Property<bool> alive;

    /**
     * \brief Whether the robot is in direct mode.
     */
    Property<bool> direct_control;

    /**
     * \brief Whether or not the ball is interrupting the robot’s laser
     * beam.
     */
    Property<bool> ball_in_beam;

    /**
     * \brief Whether or not the robot’s capacitor is charged enough to
     * kick the ball.
     */
    Property<bool> capacitor_charged;

    /**
     * \brief The voltage on the robot’s battery, in volts.
     */
    Property<double> battery_voltage;

    /**
     * \brief The voltage on the robot’s kicking capacitor, in volts.
     */
    Property<double> capacitor_voltage;

    /**
     * \brief The reading of the robot’s laser sensor.
     */
    Property<double> break_beam_reading;

    /**
     * \brief The temperature of the robot’s dribbler motor, in degrees
     * Celsius.
     */
    Property<double> dribbler_temperature;

    /**
     * \brief The speed of the robot’s dribbler motor, in revolutions
     * per minute.
     */
    Property<int> dribbler_speed;

    /**
     * \brief The temperature of the robot’s mainboard, in degrees
     * Celsius.
     */
    Property<double> board_temperature;

    /**
     * \brief The link quality of the last received packet, from 0 to
     * 1.
     */
    double link_quality;

    /**
     * \brief The received signal strength of the last received packet,
     * in decibels.
     */
    Property<int> received_signal_strength;

    /**
     * \brief Whether or not the build ID information is valid.
     */
    Property<bool> build_ids_valid;

    /**
     * \brief The microcontroller firmware build ID.
     */
    Property<uint32_t> fw_build_id;

    /**
     * \brief The FPGA bitstream build ID.
     */
    Property<uint32_t> fpga_build_id;


    {
        bool found = false;
        for (std::size_t i = 0;
             !found && i < sizeof(RSSI_TABLE) / sizeof(*RSSI_TABLE); ++i)
        {
            if (RSSI_TABLE[i].rssi < rssi)
            {
                received_signal_strength_db = RSSI_TABLE[i].db;
                found                    = true;
            }
        }
        if (!found)
        {
            received_signal_strength_db = -90;
        }
    }

    const uint8_t *bptr = static_cast<const uint8_t *>(data);
    if (len)
    {
        switch (*bptr)
        {
            case 0x00:
                // General robot status update
                ++bptr;
                --len;
                if (len >= 13)
                {
                    alive = true;

                    battery_voltage =
                        (bptr[0] | static_cast<unsigned int>(bptr[1] << 8)) /
                        1000.0;
                    bptr += 2;
                    len -= 2;

                    capacitor_voltage =
                        (bptr[0] | static_cast<unsigned int>(bptr[1] << 8)) /
                        100.0;
                    // TODO: let visualizer know about low cap voltages?
                    // low_capacitor_message.active(capacitor_voltage < 5.0);
                    bptr += 2;
                    len -= 2;

                    break_beam_reading =
                        (bptr[0] | static_cast<unsigned int>(bptr[1] << 8)) /
                        1000.0;
                    bptr += 2;
                    len -= 2;

                    board_temperature =
                        (bptr[0] | static_cast<unsigned int>(bptr[1] << 8)) /
                        100.0;
                    bptr += 2;
                    len -= 2;

                    ball_in_beam               = !!(*bptr & 0x80);
                    capacitor_charged          = !!(*bptr & 0x40);
                    unsigned int logger_status = *bptr & 0x3F;
                    for (std::size_t i = 0; i < logger_messages.size(); ++i)
                    {
                        if (logger_messages[i])
                        {
                            logger_messages[i]->active(logger_status == i);
                        }
                    }
                    ++bptr;
                    --len;

                    for (std::size_t i = 0; i < sd_messages.size(); ++i)
                    {
                        if (sd_messages[i])
                        {
                            sd_messages[i]->active(*bptr == i);
                        }
                    }
                    ++bptr;
                    --len;

                    dribbler_speed = static_cast<int16_t>(static_cast<uint16_t>(
                                         bptr[0] | (bptr[1] << 8))) *
                                     25 * 60 / 6;
                    bptr += 2;
                    len -= 2;

                    dribbler_temperature = *bptr++;
                    --len;

                    bool has_error_extension = false;
                    while (len)
                    {
                        // Decode extensions.
                        switch (*bptr)
                        {
                            case 0x00:  // Error bits.
                                ++bptr;
                                --len;
                                if (len >= MRF::ERROR_BYTES)
                                {
                                    has_error_extension = true;
                                    for (unsigned int i = 0;
                                         i != MRF::ERROR_LT_COUNT; ++i)
                                    {
                                        unsigned int byte = i / CHAR_BIT;
                                        unsigned int bit  = i % CHAR_BIT;
                                        error_lt_messages[i]->active(
                                            bptr[byte] & (1 << bit));
                                    }
                                    for (unsigned int i = 0;
                                         i != MRF::ERROR_ET_COUNT; ++i)
                                    {
                                        unsigned int byte =
                                            (i + MRF::ERROR_LT_COUNT) /
                                            CHAR_BIT;
                                        unsigned int bit =
                                            (i + MRF::ERROR_LT_COUNT) %
                                            CHAR_BIT;
                                        if (bptr[byte] & (1 << bit))
                                        {
                                            error_et_messages[i]->fire();
                                        }
                                    }
                                    bptr += MRF::ERROR_BYTES;
                                    len -= MRF::ERROR_BYTES;
                                }
                                else
                                {
                                    LOG(WARNING) << "Received general robot status "
                                        "update with truncated error bits "
                                        "extension of length " << len << std::endl;
                                }
                                break;

                            case 0x01:  // Build IDs.
                                ++bptr;
                                --len;
                                if (len >= 8)
                                {
                                    build_ids_valid = true;
                                    fw_build_id     = decode_u32_le(bptr);
                                    fpga_build_id   = decode_u32_le(bptr + 4);
                                    check_build_id_mismatch();
                                    bptr += 8;
                                    len -= 8;
                                }
                                else
                                {
                                    LOG(WARNING) <<
                                        "Received general robot status "
                                        "update with truncated build IDs "
                                        "extension of length " << len << std::endl;
                                }
                                break;

                            case 0x02:  // LPS data.
                                ++bptr;
                                --len;
                                if (len >= 4)
                                {
                                    for (unsigned int i = 0; i < 4; ++i)
                                    {
                                        lps_values[i] =
                                            static_cast<int8_t>(*bptr++) / 10.0;
                                    }
                                    len -= 4;
                                }
                                else
                                {
                                    LOG(WARNING) << 
                                        "Received general robot status "
                                        "update with truncated LPS data "
                                        "extension of length " << len << std::endl;
                                }
                                break;

                            default:
                                LOG(WARNING) << 
                                    "Received general status packet from "
                                    "robot with unknown extension code " << 
                                    static_cast<unsigned int>(*bptr) << std::endl;
                                len = 0;
                                break;
                        }
                    }

                    if (!has_error_extension)
                    {
                        // Error reporting extension is absent → no errors are
                        // asserted.
                        for (auto &i : error_lt_messages)
                        {
                            i->active(false);
                        }
                    }
                }
                else
                {
                    LOG(WARNING) << 
                        "Received general robot status update with wrong "
                        "byte count " << len << std::endl;
                }

                if (!build_ids_valid &&
                    request_build_ids_timer.elapsed() >
                        REQUEST_BUILD_IDS_INTERVAL)
                {
                    request_build_ids_timer.stop();
                    request_build_ids_timer.reset();
                    request_build_ids_timer.start();
                    if (request_build_ids_counter)
                    {
                        --request_build_ids_counter;
                        static const uint8_t REQUEST = 0x0D;
                        dongle_.send_unreliable(index, 20, &REQUEST, 1);
                    }
                    else
                    {
                        build_id_fetch_error_message.active(true);
                    }
                }
                break;

            case 0x01:
                // Autokick fired
                // TODO handle this somehow
                // signal_autokick_fired.emit();
                break;

            case 0x04:
                // Robot has ball
                ball_in_beam = true;
                break;

            case 0x05:
                // Robot does not have ball
                ball_in_beam = false;
                break;

            default:
                LOG(WARNING) << "Received packet from robot with unknown message type " << static_cast<unsigned int>(*bptr) << std::endl;
                break;
        }
    }
}
