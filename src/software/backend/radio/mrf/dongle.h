#ifndef MRF_DONGLE_H
#define MRF_DONGLE_H

/**
 * Provides access to an MRF24J40 dongle.
 */
#include <boost/signals2.hpp>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <queue>
#include <tuple>
#include <utility>
#include <vector>

#include "shared/proto/tbots_software_msgs.pb.h"
#include "software/backend/radio/mrf/annunciator.h"
#include "software/backend/radio/mrf/send_reliable_message_operation.h"
#include "software/backend/radio/mrf/usb/libusb.h"
#include "software/backend/radio/mrf/util/async_operation.h"
#include "software/backend/radio/mrf/util/noncopyable.h"
#include "software/geom/angle.h"
#include "software/geom/point.h"

/**
 * An operation to send a reliable message.
 */
class SendReliableMessageOperation;

/**
 * The dongle.
 */
class MRFDongle final
{
   public:
    /**
     * Constructs a new MRFDongle.
     *
     * @param config MRF configuration to start dongle in
     * @param annunciator annunciator to publish robot statuses
     */
    explicit MRFDongle(unsigned int config, Annunciator &annunciator);

    /**
     * Destroys an MRFDongle.
     */
    ~MRFDongle();

    /**
     * Given a TbotsProto::PrimitiveSet, constructs a single drive packet to send over
     * radio to all robots.
     *
     * @param prims TbotsProto::PrimitiveSet from HL
     */
    void send_drive_packet(const TbotsProto::PrimitiveSet &prims);

    /**
     * Sends a camera packet over radio to all robots, including vision coordinates of
     * all robots and the ball.
     * @param robots vector of tuples of {robot ID, robot location, robot orientation}
     * @param ball ball location
     * @param timestamp timestamp in seconds when this data was received
     */
    void send_camera_packet(std::vector<std::tuple<uint8_t, Point, Angle>> robots,
                            Point ball, uint64_t timestamp);

    /**
     * Generates an audible beep on the dongle.
     *
     * @param[in] length the length of the beep, in milliseconds (0 to 65,535)
     */
    void beep(unsigned int length);

    /**
     * Returns the channel number on which the dongle is communicating.
     */
    uint8_t channel() const;

    /**
     * Returns the PAN ID on which the dongle is communicating.
     */
    uint16_t pan() const;

    /**
     * The possible positions of the hardware run switch
     */
    enum class EStopState
    {
        /**
         * The switch is not connected properly
         */
        BROKEN,

        /**
         * The switch is in the stop state
         */
        STOP,

        /**
         * The switch is in the run state
         */
        RUN,
    };

    /**
     * The current state of the emergency stop switch.
     */
    EStopState eStopState;

   private:
    // We are currently uncertain if the limit on radio messages is 64 or 100 bytes. So
    // to be safe we assume 64, then leave two bytes for the E-Stop and Feedback requests
    // The robot id is assumed to be set at the start of the packet
    static const size_t MAX_RADIO_PACKET_SIZE = 62;

    friend class SendReliableMessageOperation;

    /* libusb objects for the dongle */
    USB::Context context;
    USB::DeviceHandle device;
    int radio_interface, configuration_altsetting, normal_altsetting;
    std::unique_ptr<USB::InterfaceClaimer> interface_claimer;
    uint8_t channel_;
    uint16_t pan_;

    /**
     * Encode the given primitive and robot id for transmission over radio
     *
     * @param robot_id The robot id to encode for
     * @param prim_proto The primitive to encode
     *
     * @return The serialization of the primitive proto
     */
    std::vector<uint8_t> encode_primitive(unsigned int robot_id,
                                          TbotsProto::Primitive prim_proto);

    /**
     * Attempt to pack and send the given data over libusb to the radio
     *
     * @param data The data to send
     *
     * @throw std::runtime_error If data.size() > MAX_RADIO_PACKET_SIZE
     */
    void submit_drive_transfer(std::vector<uint8_t> data);

    /* Functions that handle encoding and sending drive packets. */
    void handle_drive_transfer_done(AsyncOperation<void> &);
    uint8_t drive_packet[64];
    std::size_t drive_packet_length;
    std::unique_ptr<USB::BulkOutTransfer> drive_transfer;

    /* Camera (vision) packet stuff */
    void handle_camera_transfer_done(
        AsyncOperation<void> &,
        std::list<std::pair<std::unique_ptr<USB::BulkOutTransfer>, uint64_t>>::iterator
            iter);
    std::mutex cam_mtx;
    std::unique_ptr<USB::BulkOutTransfer> camera_transfer;
    std::list<std::pair<std::unique_ptr<USB::BulkOutTransfer>, uint64_t>>
        camera_transfers;

    /* Handling of messages from the robots. */
    std::array<std::unique_ptr<USB::BulkInTransfer>, 32> mdr_transfers;
    std::array<std::unique_ptr<USB::BulkInTransfer>, 32> message_transfers;
    USB::InterruptInTransfer status_transfer;
    std::list<std::unique_ptr<USB::BulkOutTransfer>> unreliable_messages;
    std::queue<uint8_t> free_message_ids;
    boost::signals2::signal<void(uint8_t, uint8_t)> signal_message_delivery_report;

    uint8_t alloc_message_id();
    void free_message_id(uint8_t id);
    void handle_mdrs(AsyncOperation<void> &);
    void handle_message(AsyncOperation<void> &, USB::BulkInTransfer &transfer);
    void handle_status(AsyncOperation<void> &);

    /* Sending of unreliable messages (delivery status unchecked) */
    void send_unreliable(unsigned int robot, unsigned int tries, const void *data,
                         std::size_t len);
    void check_unreliable_transfer(
        AsyncOperation<void> &,
        std::list<std::unique_ptr<USB::BulkOutTransfer>>::iterator iter);

    /* Functions that make annoying dongle beeps. */
    void submit_beep();
    void handle_beep_done(AsyncOperation<void> &);
    std::unique_ptr<USB::ControlNoDataTransfer> beep_transfer;
    unsigned int pending_beep_length;

    Annunciator &annunciator;
};

inline uint8_t MRFDongle::channel() const
{
    return channel_;
}

inline uint16_t MRFDongle::pan() const
{
    return pan_;
}

#endif
