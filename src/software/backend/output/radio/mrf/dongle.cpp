#include "software/backend/output/radio/mrf/dongle.h"

#include <unistd.h>

#include <algorithm>
#include <bitset>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <g3log/g3log.hpp>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "shared/constants.h"
#include "shared/proto/camera.pb.h"
#include "shared/proto/primitive.pb.h"
#include "software/backend/output/radio/mrf/messages.h"
#include "software/backend/output/radio/mrf/mrf_primitive_visitor.h"

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
      annunciator(annunciator),
      drive_buffer(
          std::make_shared<ThreadSafeBuffer<RadioPrimitive>>(MAX_ROBOTS_OVER_RADIO)),
      camera_buffer(
          std::make_shared<ThreadSafeBuffer<DetectedRobot>>(MAX_ROBOTS_OVER_RADIO));
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
        if (config < 0 || static_cast<std::size_t>(config) >=
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
    // more than 1 prim
    if (!detbots.empty())
    {
        std::size_t numbots = detbots.size();

        for (std::size_t i = 0; i < numbots; i++)
        {
            DetectedRobot detbot = DetectedRobot();

            detbot.set_timestamp(timestamp);
            detbot.set_ball_position(point(ball.x(), ball.y()));

            detbot.set_robot_id(std::get<0>(detbots[i]));
            detbot.set_robot_position(
                point(std::get<1>(detbots).x(), std::get<1>(detbots[i]).y()));
            detbot.set_robot_orientation(std::get<2>(detbots[i]).toRadians());
            camera_buffer->push(detbot);
        }

        submit_drive_transfer();
    }

    // Update annunciator with detected bots for dead bot detection
    if (!annunciator.beep_dongle.num_slots())
    {
        // Connect signal to beep dongle when annunciator requests it.
        this->annunciator.beep_dongle.connect(
            boost::bind(&MRFDongle::beep, this, ANNUNCIATOR_BEEP_LENGTH_MILLISECONDS));
    }
    annunciator.update_vision_detections(robot_ids);
};

bool MRF::submit_camera_transfer()
{
    if (camera_transfers.size() >= 8)
    {
        LOG(WARNING) << "Camera transfer queue is full, ignoring camera packet"
                     << std::endl;
        return;
    }

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
}

void MRFDongle::send_drive_packet(const std::vector<std::unique_ptr<Primitive>> &prims)
{
    // if there is exisiting transfer, we are currently sending a primitive to each
    // robot, round-robin style. Each robot will receive a primtivie ID,
    // followed by num_robots - 1 primitives.
    //
    // So even though the buffer has a size MAX_ROBOTS_OVER_RADIO, we are full when
    // all the primitives for the existing robots are in the buffer, and stays "full"
    // until all of those msgs are transfered. This is to avoid sending primitives at
    // unkown rates, with this method, we are limitied by the dongles receiving rate (rate
    // from computer to dongle over USB, NOT over dongel to robot radio)
    if (!drive_transfer)
    {
        LOG(WARNING) << "Drive transfer que is full, ignoring new primitives"
                     << std::endl;
        return;
    }

    // more than 1 prim
    if (!prims.empty())
    {
        std::size_t num_prims = prims.size();

        if (num_prims > MAX_ROBOTS_OVER_RADIO)
        {
            throw std::invalid_argument("Too many primitives in vector.");
        }

        // push all drive packets onto buffer and start the transfer
        for (std::size_t i = 0; i != num_prims; ++i)
        {
            drive_buffer->push(encode_primitive(prims[i]));
        }

        submit_drive_transfer();
    }
}

bool MRFDongle::submit_drive_transfer()
{
    // submit drive_packet when possible.
    if (!drive_transfer)
    {
        std::optional<RadioPrimitive> prim = drive_buffer->popLeastRecentlyAddedValue();

        if (!prim)
        {
            std::string drive_packet;
            prim->SerializeToString(&drive_packet);

            drive_transfer.reset(new USB::BulkOutTransfer(device, 1, drive_packet.c_str(),
                                                          drive_packet.length(), 64, 0));

            drive_transfer->signal_done.connect(
                boost::bind(&MRFDongle::handle_drive_transfer_done, this, _1));

            drive_transfer->submit();
        }
        else
        {
            drive_transfer.reset();
        }
    }

    return false;
}

RadioPrimitive MRFDongle::encode_primitive(const std::unique_ptr<Primitive> &prim)
{
    MRFPrimitiveVisitor visitor = MRFPrimitiveVisitor();

    // Visit the primitive.
    prim->accept(visitor);
    RadioPrimitive r_prim = visitor.getRadioPacket();

    // Encode charge state
    // Robots are always charged if the estop is in RUN state; otherwise discharge them.
    switch (estop_state)
    {
        case EStopState::BROKEN:
        case EStopState::STOP:
            // Discharge`
            r_prim.set_estop(true);
            break;
        case EStopState::RUN:
            // Charge
            r_prim.set_estop(false);
            break;
    }

    return r_prim;
}

void MRFDongle::handle_drive_transfer_done(AsyncOperation<void> &op)
{
    // start the next transfer, the transfers will stop if the buffer becomes empty
    op.result();
    submit_drive_transfer();
}

void MRFDongle::handle_camera_transfer_done(AsyncOperation<void> &op)
{
    // start the next transfer, the transfers will stop if the buffer becomes empty
    op.result();
    submit_camera_transfer();
}

void MRFDongle::handle_beep_done(AsyncOperation<void> &)
{
    beep_transfer->result();
    beep_transfer.reset();
    beep(0);
}
