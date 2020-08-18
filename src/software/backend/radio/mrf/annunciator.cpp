#include "software/backend/radio/mrf/annunciator.h"

#include <time.h>

#include <mutex>

#include "shared/constants.h"
#include "software/backend/radio/mrf/dongle.h"
#include "software/backend/radio/mrf/messages.h"
#include "software/logger/logger.h"

namespace
{
    /**
     * Amount of time to keep an edge-triggered message sending, in seconds.
     */
    const int ET_MESSAGE_KEEPALIVE_TIME = 10;

    /**
     * Amount of time without radio communication (but detected on vision)
     * for a robot to be declared as dead, in seconds.
     */
    const int ROBOT_DEAD_TIME = 2;

    /**
     * Represents a mapping from RSSI to decibels.
     */
    struct RSSITableEntry final
    {
        /* Received Signal Strength Indicator */
        int rssi;

        /* Corresponding decibel value */
        int db;
    };

    // Table of conversions from RSSI to decibels
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

    /**
     * Extracts a 32-bit integer from a data buffer in little endian form.
     *
     * @param buffer the data to extract from.
     *
     * @return the integer.
     */
    inline uint32_t decode_u32_le(const void *buffer)
    {
        const uint8_t *buf = static_cast<const uint8_t *>(buffer);
        uint32_t val       = 0;
        for (std::size_t i = 0; i < 4; ++i)
        {
            val <<= 8;
            val |= buf[3 - i];
        }
        return val;
    }

    // Struct to keep track of previously published messages for a robot
    typedef struct
    {
        // Because vision updates and status updates occur from different
        // threads, a mutex is used to prevent race conditions.
        std::mutex bot_mutex = std::mutex();

        // Previously published status for this robot
        RadioRobotStatus previous_status;

        // Map of message to timestamp for edge-triggered messages
        std::map<std::string, time_t> et_messages;

        // Timestamp of when this bot last sent a status update
        time_t last_status_update;

    } RobotStatusState;

    // Map of robot number to their previously published messages
    std::map<uint8_t, RobotStatusState> robot_status_states;

}  // namespace

Annunciator::Annunciator(
    std::function<void(RadioRobotStatus)> received_robot_status_callback)
    : received_robot_status_callback(received_robot_status_callback)
{
    // Initialize messages with the correct robot ID
    for (uint8_t bot = 0; bot < MAX_ROBOTS_OVER_RADIO; ++bot)
    {
        robot_status_states[bot].bot_mutex.lock();
        robot_status_states[bot].previous_status.robot = bot;
        robot_status_states[bot].bot_mutex.unlock();
    }
}

void Annunciator::handle_robot_message(int index, const void *data, std::size_t len,
                                       uint8_t lqi, uint8_t rssi)
{
    std::vector<std::string> new_msgs;
    RadioRobotStatus robot_status;

    // Guard robot status state for this bot
    std::scoped_lock lock(
        robot_status_states[static_cast<unsigned char>(index)].bot_mutex);

    robot_status.link_quality = lqi / 255.0;

    {
        bool found = false;
        for (std::size_t i = 0; !found && i < sizeof(RSSI_TABLE) / sizeof(*RSSI_TABLE);
             ++i)
        {
            if (RSSI_TABLE[i].rssi < rssi)
            {
                robot_status.received_signal_strength_db = RSSI_TABLE[i].db;
                found                                    = true;
            }
        }
        if (!found)
        {
            robot_status.received_signal_strength_db = -90;
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
                    robot_status.robot = index;
                    robot_status.alive = true;

                    robot_status.battery_voltage =
                        (bptr[0] | static_cast<unsigned int>(bptr[1] << 8)) / 1000.0;

                    // Warn if battery voltage is too low
                    if (robot_status.battery_voltage < MRF::MIN_BATTERY_VOLTAGE)
                    {
                        new_msgs.insert(new_msgs.begin(), MRF::LOW_BATTERY_MESSAGE);
                    }

                    bptr += 2;
                    len -= 2;

                    robot_status.capacitor_voltage =
                        (bptr[0] | static_cast<unsigned int>(bptr[1] << 8)) / 100.0;

                    // Warn if capacitor voltage is too low
                    if (robot_status.capacitor_voltage < MRF::MIN_CAP_VOLTAGE)
                    {
                        new_msgs.insert(new_msgs.begin(), MRF::LOW_CAP_MESSAGE);
                    }

                    bptr += 2;
                    len -= 2;

                    robot_status.break_beam_reading =
                        (bptr[0] | static_cast<unsigned int>(bptr[1] << 8)) / 1000.0;
                    bptr += 2;
                    len -= 2;

                    robot_status.board_temperature =
                        (bptr[0] | static_cast<unsigned int>(bptr[1] << 8)) / 100.0;

                    // Warn if board temperature too high
                    if (robot_status.board_temperature > MRF::MAX_BOARD_TEMPERATURE)
                    {
                        new_msgs.insert(new_msgs.begin(), MRF::HIGH_BOARD_TEMP_MESSAGE);
                    }

                    bptr += 2;
                    len -= 2;

                    robot_status.ball_in_beam                          = !!(*bptr & 0x80);
                    robot_status.capacitor_charged_enough_to_kick_ball = !!(*bptr & 0x40);

                    // Robot logger status
                    unsigned int logger_status = *bptr & 0x3F;
                    for (std::size_t i = 0; i < MRF::LOGGER_MESSAGES.size(); ++i)
                    {
                        if (MRF::LOGGER_MESSAGES[i] && (logger_status == i))
                        {
                            new_msgs.push_back(MRF::LOGGER_MESSAGES[i]);
                        }
                    }
                    ++bptr;
                    --len;

                    // SD card messages
                    for (std::size_t i = 0; i < MRF::SD_MESSAGES.size(); ++i)
                    {
                        if (MRF::SD_MESSAGES[i] && (*bptr == i))
                        {
                            new_msgs.push_back(MRF::SD_MESSAGES[i]);
                        }
                    }
                    ++bptr;
                    --len;

                    robot_status.dribbler_speed =
                        static_cast<int16_t>(
                            static_cast<uint16_t>(bptr[0] | (bptr[1] << 8))) *
                        25 * 60 / 6;
                    bptr += 2;
                    len -= 2;

                    robot_status.dribbler_temperature = *bptr++;
                    --len;

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
                                    // Handling of level-triggered messages.
                                    for (unsigned int i = 0; i < MRF::ERROR_LT_COUNT; ++i)
                                    {
                                        unsigned int byte = i / CHAR_BIT;
                                        unsigned int bit  = i % CHAR_BIT;

                                        if (bptr[byte] & (1 << bit) &&
                                            MRF::ERROR_LT_MESSAGES[i])
                                        {
                                            new_msgs.push_back(MRF::ERROR_LT_MESSAGES[i]);
                                        }
                                    }

                                    /**
                                     * Handling of edge-triggered messages.
                                     * These messages are added to a separate map,
                                     * mapping to a timestamp of when it was most recently
                                     * transmitted
                                     */
                                    for (unsigned int i = 0; i < MRF::ERROR_ET_COUNT; ++i)
                                    {
                                        unsigned int byte =
                                            (i + MRF::ERROR_LT_COUNT) / CHAR_BIT;
                                        unsigned int bit =
                                            (i + MRF::ERROR_LT_COUNT) % CHAR_BIT;

                                        if (bptr[byte] & (1 << bit) &&
                                            MRF::ERROR_ET_MESSAGES[i])
                                        {
                                            robot_status_states[static_cast<
                                                                    unsigned char>(index)]
                                                .et_messages[MRF::ERROR_ET_MESSAGES[i]] =
                                                time(nullptr);
                                        }
                                    }

                                    bptr += MRF::ERROR_BYTES;
                                    len -= MRF::ERROR_BYTES;
                                }
                                else
                                {
                                    LOG(WARNING) << "Received general robot status "
                                                    "update with truncated error bits "
                                                    "extension of length "
                                                 << len << std::endl;
                                }
                                break;

                            case 0x01:  // Build IDs.
                                ++bptr;
                                --len;
                                if (len >= 8)
                                {
                                    robot_status.build_ids_valid = true;
                                    robot_status.fw_build_id     = decode_u32_le(bptr);
                                    robot_status.fpga_build_id = decode_u32_le(bptr + 4);
                                    bptr += 8;
                                    len -= 8;
                                }
                                else
                                {
                                    LOG(WARNING) << "Received general robot status "
                                                    "update with truncated build IDs "
                                                    "extension of length "
                                                 << len << std::endl;
                                }
                                break;

                            case 0x02:  // LPS data. WARNING: unused, do not delete until
                                        // it's removed from firmware
                                ++bptr;
                                --len;
                                if (len >= 4)
                                {
                                    len -= 4;
                                }
                                else
                                {
                                    LOG(WARNING) << "Received general robot status "
                                                    "update with truncated LPS data "
                                                    "extension of length "
                                                 << len << std::endl;
                                }
                                break;

                            default:
                                LOG(WARNING)
                                    << "Received general status packet from "
                                       "robot with unknown extension code "
                                    << static_cast<unsigned int>(*bptr) << std::endl;
                                len = 0;
                                break;
                        }
                    }
                }
                else
                {
                    LOG(WARNING) << "Received general robot status update with wrong "
                                    "byte count "
                                 << len << std::endl;
                }

                break;

            case 0x01:
                // Autokick fired
                robot_status.autokick_fired = true;
                break;

            case 0x04:
                // Robot has ball
                robot_status.ball_in_beam = true;
                break;

            case 0x05:
                // Robot does not have ball
                robot_status.ball_in_beam = false;
                break;

            default:
                LOG(WARNING) << "Received packet from robot with unknown message type "
                             << static_cast<unsigned int>(*bptr) << std::endl;
                break;
        }
    }

    // Edge-triggered messages: keep sending message for ET_MESSAGE_KEEPALIVE_TIME seconds
    for (auto const &et_msg :
         robot_status_states[static_cast<unsigned char>(index)].et_messages)
    {
        if (difftime(time(nullptr), et_msg.second) < ET_MESSAGE_KEEPALIVE_TIME)
        {
            new_msgs.push_back(et_msg.first);
        }
    }

    // Beep the dongle if there were new messages since the last update
    checkNewMessages(new_msgs, robot_status_states[static_cast<unsigned char>(index)]
                                   .previous_status.robot_messages);

    // Add the latest robot and dongle status messages
    robot_status.robot_messages  = new_msgs;
    robot_status.dongle_messages = dongle_messages;

    // Update previous message state
    robot_status_states[static_cast<unsigned char>(index)].previous_status = robot_status;

    // Update last communicated time
    robot_status_states[static_cast<unsigned char>(index)].last_status_update =
        time(nullptr);

    // Send out robot status
    received_robot_status_callback(robot_status);
}

void Annunciator::checkNewMessages(std::vector<std::string> new_msgs,
                                   std::vector<std::string> old_msgs)
{
    // If there is a new message that wasn't present in the previous status update, beep
    // the dongle
    for (std::string msg : new_msgs)
    {
        if (std::find(old_msgs.begin(), old_msgs.end(), msg) == old_msgs.end())
        {
            beep_dongle();
            break;
        }
    }
}

std::vector<std::string> Annunciator::handle_dongle_messages(uint8_t status)
{
    dongle_messages.clear();

    if (static_cast<MRFDongle::EStopState>(status & 3U) == MRFDongle::EStopState::BROKEN)
    {
        dongle_messages.push_back(MRF::ESTOP_BROKEN_MESSAGE);
    }

    if (status & 4U)
    {
        dongle_messages.push_back(MRF::RX_FCS_FAIL_MESSAGE);
    }
    if (status & 8U)
    {
        dongle_messages.push_back(MRF::SECOND_DONGLE_MESSAGE);
    }
    if (status & 16U)
    {
        dongle_messages.push_back(MRF::TRANSMIT_QUEUE_FULL_MESSAGE);
    }
    if (status & 32U)
    {
        dongle_messages.push_back(MRF::RECEIVE_QUEUE_FULL_MESSAGE);
    }
    if (status & 64U)
    {
        dongle_messages.push_back(MRF::PACKET_ABOVE_MAX_SIZE);
    }

    return dongle_messages;
}

void Annunciator::update_vision_detections(std::vector<uint8_t> robots)
{
    for (uint8_t bot : robots)
    {
        // Guard robot status state for this bot
        robot_status_states[bot].bot_mutex.lock();

        // Check if robot is dead, publish old status update with dead message if so
        if (difftime(time(nullptr), robot_status_states[bot].last_status_update) >
            ROBOT_DEAD_TIME)
        {
            RadioRobotStatus new_status = robot_status_states[bot].previous_status;
            new_status.robot_messages.clear();
            new_status.robot_messages.push_back(MRF::ROBOT_DEAD_MESSAGE);

            // Beep dongle if this is a recent event
            checkNewMessages(new_status.robot_messages,
                             robot_status_states[bot].previous_status.robot_messages);

            // Update and publish latest status
            robot_status_states[bot].previous_status = new_status;
            received_robot_status_callback(new_status);
        }

        robot_status_states[bot].bot_mutex.unlock();
    }
}
