#include "software/backend/output/wifi/communication/robot_communicator.h"

#include <iostream>

#include "shared/proto/primitive.pb.h"
#include "shared/proto/status.pb.h"
#include "shared/proto/vision.pb.h"
#include "software/backend/output/wifi/communication/transfer_media/transfer_medium.h"
#include "software/multithreading/thread_safe_buffer.h"

using boost::asio::ip::udp;
using google::protobuf::Message;

template <class SendProto, class ReceiveProto>
RobotCommunicator<SendProto, ReceiveProto>::RobotCommunicator(
    std::unique_ptr<TransferMedium> medium, MsgSentCallback<SendProto> sent_callback,
    MsgReceivedCallback<ReceiveProto> received_callback)
    : sent_callback(sent_callback),
      received_callback(received_callback),
      in_destructor(false),
      medium(std::move(medium))
{
    // start thread to send data from the buffer
    send_buffer.reset(new ThreadSafeBuffer<SendProto>(10));
    send_thread = std::thread(&RobotCommunicator::send_loop, this, std::ref(send_buffer));

    this->medium->receive_data_async([&](std::string incoming_data) {
        ReceiveProto msg;
        msg.ParseFromString(incoming_data);

        if (received_callback)
            received_callback(msg);
    });
}

template <class SendProto, class ReceiveProto>
RobotCommunicator<SendProto, ReceiveProto>::~RobotCommunicator()
{
    in_destructor_mutex.lock();
    in_destructor = true;
    in_destructor_mutex.unlock();

    send_thread.join();
}

template <class SendProto, class ReceiveProto>
void RobotCommunicator<SendProto, ReceiveProto>::send_proto(const SendProto& proto)
{
    send_buffer->push(proto);
}

template <class SendProto, class ReceiveProto>
void RobotCommunicator<SendProto, ReceiveProto>::send_proto_vector(
    const std::vector<SendProto>& protos)
{
    for (auto proto : protos)
    {
        send_buffer->push(proto);
    }
}


template <class SendProto, class ReceiveProto>
void RobotCommunicator<SendProto, ReceiveProto>::send_loop(
    std::shared_ptr<ThreadSafeBuffer<SendProto>> buffer)
{
    do
    {
        in_destructor_mutex.unlock();

        std::optional<SendProto> new_val =
            buffer->popMostRecentlyAddedValue(Duration::fromSeconds(0.1));

        if (new_val)
        {
            std::string data;
            (*new_val).SerializeToString(&data);

            medium->send_data(data);

            if (sent_callback)
            {
                sent_callback(*new_val);
            }
        }

        in_destructor_mutex.lock();

    } while (!in_destructor);
}

// place all templated communcation msg send/receive pair initializations here
template class RobotCommunicator<PrimitiveMsg, StatusMsg>;
template class RobotCommunicator<VisionMsg, StatusMsg>;
