#include "software/jetson_nano/services/network/network.h"

NetworkService::NetworkService(const std::string& ip_address,
                               unsigned short primitive_listener_port,
                               unsigned short robot_status_sender_port, bool multicast)
    : primitive_tracker(ProtoTracker("primitive set"))
{
    sender = std::make_unique<ThreadedProtoUdpSender<TbotsProto::RobotStatus>>(
        ip_address, robot_status_sender_port, multicast);

    udp_listener_primitive_set =
        std::make_unique<ThreadedProtoUdpListener<TbotsProto::PrimitiveSet>>(
            ip_address, primitive_listener_port,
            boost::bind(&NetworkService::primitiveSetCallback, this, _1), multicast);

    radio_listener_primitive_set =
        std::make_unique<ThreadedProtoRadioListener<TbotsProto::PrimitiveSet>>(
            boost::bind(&NetworkService::primitiveSetCallback, this, _1));
}

TbotsProto::PrimitiveSet NetworkService::poll(TbotsProto::RobotStatus& robot_status)
{
    std::scoped_lock lock{primitive_set_mutex};

    robot_status.mutable_network_status()->set_primitive_packet_loss_percentage(
        static_cast<unsigned int>(primitive_tracker.getLossRate() * 100));

    // Rate limit sending of proto based on thunderloop freq
    if (shouldSendNewRobotStatus(robot_status))
    {
        last_breakbeam_state_sent = robot_status.power_status().breakbeam_tripped();
        updatePrimitiveSetLog(robot_status);
        sender->sendProto(robot_status);
        network_ticks = (network_ticks + 1) % ROBOT_STATUS_BROADCAST_RATE_HZ;
    }
    thunderloop_ticks = (thunderloop_ticks + 1) % THUNDERLOOP_HZ;
    return primitive_set_msg;
}

bool NetworkService::shouldSendNewRobotStatus(
    const TbotsProto::RobotStatus& robot_status) const
{
    bool has_motor_fault =
        robot_status.motor_status().front_left().motor_faults_size() > 0 ||
        robot_status.motor_status().front_right().motor_faults_size() > 0 ||
        robot_status.motor_status().back_left().motor_faults_size() > 0 ||
        robot_status.motor_status().back_right().motor_faults_size() > 0;

    bool has_breakbeam_status_changed =
        robot_status.has_power_status() &&
        robot_status.power_status().breakbeam_tripped() != last_breakbeam_state_sent;

    bool require_heartbeat_status_update = (network_ticks / (thunderloop_ticks + 1.0)) <=
                                           ROBOT_STATUS_TO_THUNDERLOOP_HZ_RATIO;

    return has_motor_fault || has_breakbeam_status_changed ||
           require_heartbeat_status_update;
}

void NetworkService::primitiveSetCallback(TbotsProto::PrimitiveSet input)
{
    std::scoped_lock<std::mutex> lock(primitive_set_mutex);
    const uint64_t seq_num = input.sequence_number();

    logNewPrimitiveSet(input);

    primitive_tracker.send(seq_num);
    if (primitive_tracker.isLastValid())
    {
        primitive_set_msg = input;
    }

    float primitive_set_loss_rate = primitive_tracker.getLossRate();
    if (primitive_set_loss_rate > PROTO_LOSS_WARNING_THRESHOLD)
    {
        LOG(WARNING) << "Primitive set loss rate is " << primitive_set_loss_rate * 100
                     << "%";
    }
}

void NetworkService::logNewPrimitiveSet(TbotsProto::PrimitiveSet input)
{
    /* TODO: THIS TRACKS THE RECEIVED PRIMITIVES
     *  - make sure it is in the lock
     *  - log the new PRIMITIVE SET PROTO within a deque if it is valid (see primitive_tracker.send())
     *  - update the timestamp on each primitive set to the current epoch time
     */

    if (primitive_set_rtt.size() >= 50)
    {
        LOG(WARNING) << "Too many primitive sets logged for round-trip calculations, halting log process";
        return;
    }

    if (!primitive_set_rtt.empty()
            && input.sequence_number() <= primitive_set_rtt.back().primitive_sequence_num)
    {
        // If the proto is older than the last received proto, then ignore it
        return;
    }

    NetworkService::RoundTripTime current_round_trip_time;
    current_round_trip_time.primitive_sequence_num = input.sequence_number();
    current_round_trip_time.thunderscope_sent_time_seconds = input.time_sent().epoch_timestamp_seconds();
    current_round_trip_time.thunderloop_recieved_time_seconds = getCurrentEpochTimeInSeconds();

    primitive_set_rtt.emplace_back(current_round_trip_time);
}

void NetworkService::updatePrimitiveSetLog(TbotsProto::RobotStatus &robot_status)
{
    /**
     * TODO: Send return RobotStatus msg to Thunderscope
     *      1) Traverse deque containing all received primitive_sets
     *      2) If the last_handled_primitive_set for current robot_status is stored in the deque:
     *         - delete all earlier sets
     *         - stop & calculate the processing counter
     *         - store the new omit_thunderloop_processing_time_sent = time_sent + processing duration
     */

    uint64_t seq_num = robot_status.last_handled_primitive_set();
    while (seq_num <= primitive_set_rtt.back().primitive_sequence_num)
    {
        if (primitive_set_rtt.front().primitive_sequence_num == seq_num)
        {
            double received_epoch_time_seconds = primitive_set_rtt.front().thunderloop_recieved_time_seconds;
            double processing_time_seconds = getCurrentEpochTimeInSeconds() - received_epoch_time_seconds;

            robot_status.mutable_omit_thunderloop_processing_time_sent()->set_epoch_timestamp_seconds(
                    primitive_set_rtt.front().thunderscope_sent_time_seconds
                    + processing_time_seconds
            );
            return;
        }
        primitive_set_rtt.pop_front();
    }
}

double NetworkService::getCurrentEpochTimeInSeconds()
{
    return std::chrono::duration<double>
            (std::chrono::steady_clock::now().time_since_epoch()).count();
}
