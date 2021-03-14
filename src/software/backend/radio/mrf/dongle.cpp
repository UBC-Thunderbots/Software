#include "software/backend/radio/mrf/dongle.h"

#include <unistd.h>

#include <algorithm>
#include <bitset>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "shared/constants.h"
#include "software/backend/radio/mrf/messages.h"
#include "software/logger/logger.h"

namespace
{
    struct RadioConfig
    {
        uint8_t channel;
        int symbol_rate;
        uint16_t pan;
    };

    // Different configs for the dongle, to allow for communication over
    // different PANs and/or channels
    constexpr int NUM_DEFAULT_CONFIGS                      = 4;
    const RadioConfig DEFAULT_CONFIGS[NUM_DEFAULT_CONFIGS] = {
        {24U, 250, 0x1846U},
        {24U, 250, 0x1847U},
        {24U, 250, 0x1848U},
        {24U, 250, 0x1849U},
    };

    const unsigned int ANNUNCIATOR_BEEP_LENGTH_MILLISECONDS = 750;  // milliseconds

    // The dongle's MAC address
    static const uint64_t MAC = UINT64_C(0x20cb13bd834ab817);

    // Used for sorting by robot ID
    struct
    {
        bool operator()(std::tuple<uint8_t, Point, Angle> a,
                        std::tuple<uint8_t, Point, Angle> b) const
        {
            return std::get<0>(a) < std::get<0>(b);
        }
    } customLess;

}  // namespace

MRFDongle::MRFDongle(unsigned int config, Annunciator &annunciator)
    : estop_state(EStopState::STOP),
      context(),
      device(context, MRF::VENDOR_ID, MRF::PRODUCT_ID, std::getenv("MRF_SERIAL")),
      radio_interface(-1),
      configuration_altsetting(-1),
      normal_altsetting(-1),
      status_transfer(device, 3, 1, true, 0),
      pending_beep_length(0),
      annunciator(annunciator)
{
    // Sanity-check the dongle by looking for an interface with the appropriate
    // subclass and alternate settings with the appropriate protocols.
    // While doing so, discover which interface number is used for the radio and
    // which alternate settings are for configuration-setting and normal // operation.
    {
        const libusb_config_descriptor &desc =
            device.configuration_descriptor_by_value(1);
        for (int i = 0; i < desc.bNumInterfaces; ++i)
        {
            const libusb_interface &intf = desc.interface[i];
            if (intf.num_altsetting && intf.altsetting[0].bInterfaceClass == 0xFF &&
                intf.altsetting[1].bInterfaceSubClass == MRF::SUBCLASS)
            {
                radio_interface = i;
                for (int j = 0; j < intf.num_altsetting; ++j)
                {
                    const libusb_interface_descriptor &as = intf.altsetting[j];
                    if (as.bInterfaceClass == 0xFF &&
                        as.bInterfaceSubClass == MRF::SUBCLASS)
                    {
                        if (as.bInterfaceProtocol == MRF::PROTOCOL_OFF)
                        {
                            configuration_altsetting = j;
                        }
                        else if (as.bInterfaceProtocol == MRF::PROTOCOL_NORMAL)
                        {
                            normal_altsetting = j;
                        }
                    }
                }
                break;
            }
        }
        if (radio_interface < 0 || configuration_altsetting < 0 || normal_altsetting < 0)
        {
            throw std::runtime_error(
                "Wrong USB descriptors (is your dongle firmware or your "
                "software out of date or mismatched across branches?).");
        }
    }

    // Move the dongle into configuration 1 (it will nearly always already be
    // there).
    if (device.get_configuration() != 1)
    {
        device.set_configuration(1);
    }

    // Claim the radio interface.
    interface_claimer.reset(new USB::InterfaceClaimer(device, radio_interface));

    // Switch to configuration mode and configure the radio parameters.
    device.set_interface_alt_setting(radio_interface, configuration_altsetting);
    {
        if (static_cast<std::size_t>(config) >=
            sizeof(DEFAULT_CONFIGS) / sizeof(*DEFAULT_CONFIGS))
        {
            throw std::out_of_range("Config index must be between 0 and " +
                                    std::to_string(NUM_DEFAULT_CONFIGS - 1));
        }

        channel_ = DEFAULT_CONFIGS[config].channel;
        {
            const char *channel_string = std::getenv("MRF_CHANNEL");
            if (channel_string)
            {
                int i = std::stoi(channel_string, nullptr, 0);
                if (i < 0x0B || i > 0x1A)
                {
                    throw std::out_of_range(
                        "Channel number must be between 0x0B (11) and 0x1A "
                        "(26).");
                }
                channel_ = static_cast<uint8_t>(i);
            }
        }
        int symbol_rate = DEFAULT_CONFIGS[config].symbol_rate;
        {
            const char *symbol_rate_string = std::getenv("MRF_SYMBOL_RATE");
            if (symbol_rate_string)
            {
                int i = std::stoi(symbol_rate_string, nullptr, 0);
                if (i != 250 && i != 625)
                {
                    throw std::out_of_range("Symbol rate must be 250 or 625.");
                }
                symbol_rate = i;
            }
        }
        pan_ = DEFAULT_CONFIGS[config].pan;
        {
            const char *pan_string = std::getenv("MRF_PAN");
            if (pan_string)
            {
                int i = std::stoi(pan_string, nullptr, 0);
                if (i < 0 || i > 0xFFFE)
                {
                    throw std::out_of_range(
                        "PAN must be between 0x0000 (0) and 0xFFFE (65,534).");
                }
                pan_ = static_cast<uint16_t>(i);
            }
        }
        device.control_no_data(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
                               MRF::CONTROL_REQUEST_SET_CHANNEL, channel_,
                               static_cast<uint16_t>(radio_interface), 0);
        device.control_no_data(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
                               MRF::CONTROL_REQUEST_SET_SYMBOL_RATE,
                               symbol_rate == 625 ? 1 : 0,
                               static_cast<uint16_t>(radio_interface), 0);
        device.control_no_data(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
                               MRF::CONTROL_REQUEST_SET_PAN_ID, pan_,
                               static_cast<uint16_t>(radio_interface), 0);
        device.control_out(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
                           MRF::CONTROL_REQUEST_SET_MAC_ADDRESS, 0,
                           static_cast<uint16_t>(radio_interface), &MAC, sizeof(MAC), 0);

        {
            std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
            std::chrono::system_clock::time_point epoch =
                std::chrono::system_clock::from_time_t(0);
            std::chrono::system_clock::duration diff = now - epoch;
            std::chrono::microseconds micros =
                std::chrono::duration_cast<std::chrono::microseconds>(diff);
            uint64_t stamp = static_cast<uint64_t>(micros.count());
            device.control_out(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
                               MRF::CONTROL_REQUEST_SET_TIME, 0, 0, &stamp, sizeof(stamp),
                               0);
        }
    }

    // Switch to normal mode.
    device.set_interface_alt_setting(radio_interface, normal_altsetting);

    // Prepare the available message IDs for allocation.
    for (unsigned int i = 0; i < 256; ++i)
    {
        free_message_ids.push(static_cast<uint8_t>(i));
    }

    // Submit the message delivery report transfers.
    for (auto &i : mdr_transfers)
    {
        // Attempt to receive at most 8 bytes from endpoint 1
        i.reset(new USB::BulkInTransfer(device, 1, 8, false, 0));
        i->signal_done.connect(boost::bind(&MRFDongle::handle_mdrs, this, _1));
        i->submit();
    }

    // Submit the received message transfers.
    for (auto &i : message_transfers)
    {
        // Attempt to receive at most 105 bytes from endpoint 2
        i.reset(new USB::BulkInTransfer(device, 2, 105, false, 0));
        i->signal_done.connect(
            boost::bind(&MRFDongle::handle_message, this, _1, boost::ref(*i.get())));
        i->submit();
    }

    // Submit the estop transfer.
    status_transfer.signal_done.connect(boost::bind(&MRFDongle::handle_status, this, _1));
    status_transfer.submit();
}

MRFDongle::~MRFDongle()
{
    // Mark USB device as shutting down to squelch cancelled transfer warnings.
    device.mark_shutting_down();
}

void MRFDongle::beep(unsigned int length)
{
    pending_beep_length = std::max(length, pending_beep_length);
    if (!beep_transfer && pending_beep_length)
    {
        beep_transfer.reset(new USB::ControlNoDataTransfer(
            device, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
            MRF::CONTROL_REQUEST_BEEP, static_cast<uint16_t>(pending_beep_length),
            static_cast<uint16_t>(radio_interface), 0));
        beep_transfer->signal_done.connect(
            boost::bind(&MRFDongle::handle_beep_done, this, _1));
        beep_transfer->submit();
        pending_beep_length = 0;
    }
}

uint8_t MRFDongle::alloc_message_id()
{
    if (free_message_ids.empty())
    {
        throw std::runtime_error("Out of reliable message IDs");
    }
    uint8_t id = free_message_ids.front();
    free_message_ids.pop();
    return id;
}

void MRFDongle::free_message_id(uint8_t id)
{
    free_message_ids.push(id);
}

void MRFDongle::handle_mdrs(AsyncOperation<void> &op)
{
    USB::BulkInTransfer &mdr_transfer = dynamic_cast<USB::BulkInTransfer &>(op);
    mdr_transfer.result();
    if ((mdr_transfer.size() % 2) != 0)
    {
        throw std::runtime_error("MDR transfer has odd size");
    }
    for (unsigned int i = 0; i < mdr_transfer.size(); i += 2)
    {
        signal_message_delivery_report(mdr_transfer.data()[i],
                                       mdr_transfer.data()[i + 1]);
    }
    mdr_transfer.submit();
}

void MRFDongle::handle_message(AsyncOperation<void> &, USB::BulkInTransfer &transfer)
{
    if (!annunciator.beep_dongle.num_slots())
    {
        // Connect signal to beep dongle when annunciator requests it.
        this->annunciator.beep_dongle.connect(
            boost::bind(&MRFDongle::beep, this, ANNUNCIATOR_BEEP_LENGTH_MILLISECONDS));
    }
    transfer.result();

    // Only handle if there are more than 2 bytes in the transfer.
    if (transfer.size() > 2)
    {
        unsigned int robot = transfer.data()[0];
        annunciator.handle_robot_message(robot, transfer.data() + 1, transfer.size() - 3,
                                         transfer.data()[transfer.size() - 2],
                                         transfer.data()[transfer.size() - 1]);
    }
    transfer.submit();
}

void MRFDongle::handle_status(AsyncOperation<void> &)
{
    status_transfer.result();
    estop_state      = static_cast<EStopState>(status_transfer.data()[0] & 3U);
    auto dongle_msgs = annunciator.handle_dongle_messages(status_transfer.data()[0U]);
    status_transfer.submit();

    // These messages are critical enough that the dongle should continuously beep
    // while the conditions are true.
    if (!dongle_msgs.empty())
    {
        beep(ANNUNCIATOR_BEEP_LENGTH_MILLISECONDS);
    }
}


void MRFDongle::send_camera_packet(std::vector<std::tuple<uint8_t, Point, Angle>> detbots,
                                   Point ball, uint64_t timestamp)
{
    int8_t camera_packet[55] = {0};
    int8_t mask_vec = 0;  // Assume all robots don't have valid position at the start
    uint8_t numbots = static_cast<uint8_t>(detbots.size());
    std::vector<uint8_t> robot_ids;

    // Initialize pointer to start at location of storing ball data. First 2
    // bytes are for mask and flag vector
    int8_t *rptr = &camera_packet[1];

    int16_t ballX = static_cast<int16_t>(ball.x() * 1000.0);
    int16_t ballY = static_cast<int16_t>(ball.y() * 1000.0);

    *rptr++ = static_cast<int8_t>(ballX);  // Add Ball x position
    *rptr++ = static_cast<int8_t>(ballX >> 8);

    *rptr++ = static_cast<int8_t>(ballY);  // Add Ball Y position
    *rptr++ = static_cast<int8_t>(ballY >> 8);

    // Sort robots in ascending order by ID
    std::sort(detbots.begin(), detbots.end(), customLess);

    // For the number of robot for which data was passed in, assign robot ids to
    // mask vector and position/angle data to camera packet
    for (std::size_t i = 0; i < numbots; i++)
    {
        uint8_t robotID = std::get<0>(detbots[i]);
        robot_ids.push_back(robotID);

        int16_t robotX = static_cast<int16_t>((std::get<1>(detbots[i])).x() * 1000);
        int16_t robotY = static_cast<int16_t>((std::get<1>(detbots[i])).y() * 1000);
        int16_t robotT =
            static_cast<int16_t>((std::get<2>(detbots[i])).toRadians() * 1000);

        mask_vec |= int8_t(0x01 << (robotID));
        *rptr++ = static_cast<int8_t>(robotX);
        *rptr++ = static_cast<int8_t>(robotX >> 8);
        *rptr++ = static_cast<int8_t>(robotY);
        *rptr++ = static_cast<int8_t>(robotY >> 8);
        *rptr++ = static_cast<int8_t>(robotT);
        *rptr++ = static_cast<int8_t>(robotT >> 8);
    }

    // Write out the timestamp
    for (std::size_t i = 0; i < 8; i++)
    {
        *rptr++ = static_cast<int8_t>(timestamp >> 8 * i);
    }

    // Mask and Flag Vectors should be fully initialized by now. Assign them to
    // the packet
    camera_packet[0] = mask_vec;

    std::lock_guard<std::mutex> lock(cam_mtx);

    if (camera_transfers.size() >= 8)
    {
        LOG(WARNING) << "Camera transfer queue is full, ignoring camera packet"
                     << std::endl;
        return;
    }

    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    std::chrono::system_clock::time_point epoch =
        std::chrono::system_clock::from_time_t(0);
    std::chrono::system_clock::duration diff = now - epoch;
    std::chrono::microseconds micros =
        std::chrono::duration_cast<std::chrono::microseconds>(diff);
    uint64_t stamp = static_cast<uint64_t>(micros.count());

    // Create and submit USB transfer with camera packet
    std::unique_ptr<USB::BulkOutTransfer> elt(
        new USB::BulkOutTransfer(device, 2, camera_packet, 55, 55, 0));
    auto i = camera_transfers.insert(
        camera_transfers.end(),
        std::pair<std::unique_ptr<USB::BulkOutTransfer>, uint64_t>(std::move(elt),
                                                                   stamp));
    (*i).first->signal_done.connect(
        boost::bind(&MRFDongle::handle_camera_transfer_done, this, _1, i));
    (*i).first->submit();

    // Update annunciator with detected bots for dead bot detection
    if (!annunciator.beep_dongle.num_slots())
    {
        // Connect signal to beep dongle when annunciator requests it.
        this->annunciator.beep_dongle.connect(
            boost::bind(&MRFDongle::beep, this, ANNUNCIATOR_BEEP_LENGTH_MILLISECONDS));
    }
    annunciator.update_vision_detections(robot_ids);
};

void MRFDongle::send_drive_packet(const TbotsProto::PrimitiveSet &prims)
{
    for (auto &[robot_id, prim_proto] : prims.robot_primitives())
    {
        std::vector<uint8_t> encoded_primitive = encode_primitive(robot_id, prim_proto);
        if (encoded_primitive.size() > MAX_RADIO_PACKET_SIZE)
        {
            LOG(WARNING)
                << "Failed to send a primitive, because it had an encoded length of"
                << encoded_primitive.size() << ", max is  " << MAX_RADIO_PACKET_SIZE;
        }
        else
        {
            submit_drive_transfer(encoded_primitive);
        }
    }
}

void MRFDongle::submit_drive_transfer(std::vector<uint8_t> data)
{
    // Check the data is of valid size
    if (data.size() > MAX_RADIO_PACKET_SIZE)
    {
        std::stringstream ss;
        ss << "Tried to send packet with length " << data.size()
           << " over radio, max size is " << MAX_RADIO_PACKET_SIZE;
        throw std::runtime_error(ss.str());
    }

    // Submit drive_packet when possible.
    if (!drive_transfer)
    {
        std::cout << "sending data of size: " << data.size() << std::endl;
        drive_transfer.reset(new USB::BulkOutTransfer(device, 1, &data[0], data.size(),
                                                      MAX_RADIO_PACKET_SIZE, 0));
        drive_transfer->signal_done.connect(
            boost::bind(&MRFDongle::handle_drive_transfer_done, this, _1));
        drive_transfer->submit();
    }
}

std::vector<uint8_t> MRFDongle::encode_primitive(unsigned int robot_id,
                                                 TbotsProto::Primitive prim_proto)
{
    // Serialize the proto representation
    std::vector<uint8_t> serialized_proto(prim_proto.ByteSizeLong());
    prim_proto.SerializeToArray(serialized_proto.data(),
                                static_cast<int>(prim_proto.ByteSizeLong()));
    // Prepend the robot id
    // We place this outside the proto so that we can check on the robot if the message
    // was intended for that robot with de-serializing the message
    serialized_proto.insert(serialized_proto.begin(), static_cast<uint8_t>(robot_id));


    return serialized_proto;
}

void MRFDongle::handle_drive_transfer_done(AsyncOperation<void> &op)
{
    op.result();
    drive_transfer.reset();
}

void MRFDongle::handle_camera_transfer_done(
    AsyncOperation<void> &op,
    std::list<std::pair<std::unique_ptr<USB::BulkOutTransfer>, uint64_t>>::iterator iter)
{
    std::lock_guard<std::mutex> lock(cam_mtx);
    op.result();
    camera_transfers.erase(iter);
}

void MRFDongle::send_unreliable(unsigned int robot, unsigned int tries, const void *data,
                                std::size_t len)
{
    if (robot >= MAX_ROBOTS_OVER_RADIO)
    {
        throw std::out_of_range("Robot ID must be below 8");
    }

    if ((tries < 1) || (tries > 256))
    {
        throw std::out_of_range("Number of tries must be between 1 and 256 (inclusive)");
    }

    std::vector<uint8_t> buffer_vec(len + 2);
    buffer_vec[0]   = static_cast<uint8_t>(robot);
    buffer_vec[1]   = static_cast<uint8_t>(tries & 0xFF);
    uint8_t *buffer = buffer_vec.data();
    std::memcpy(buffer + 2, data, len);
    std::unique_ptr<USB::BulkOutTransfer> elt(
        new USB::BulkOutTransfer(device, 3, buffer, sizeof(buffer), 64, 0));
    auto i = unreliable_messages.insert(unreliable_messages.end(), std::move(elt));
    (*i)->signal_done.connect(
        boost::bind(&MRFDongle::check_unreliable_transfer, this, _1, i));
    (*i)->submit();
}

void MRFDongle::check_unreliable_transfer(
    AsyncOperation<void> &,
    std::list<std::unique_ptr<USB::BulkOutTransfer>>::iterator iter)
{
    (*iter)->result();
    unreliable_messages.erase(iter);
}

void MRFDongle::handle_beep_done(AsyncOperation<void> &)
{
    beep_transfer->result();
    beep_transfer.reset();
    beep(0);
}
