#include "firmware_new/tools/communication/robot_communicator.h"

#include "firmware_new/tools/communication/transfer_media/transfer_medium.h"
#include "software/multithreading/thread_safe_buffer.h"

using boost::asio::ip::udp;
using google::protobuf::Message;

template <class SendProto, class ReceiveProto>
RobotCommunicator<SendProto, ReceiveProto>::RobotCommunicator(
    const TransferMedium& medium, const MsgSentCallback<SendProto>& sent_callback,
    const MsgReceivedCallback<ReceiveProto>& received_callback)
{
    // connect to the medium
    medium.connect();

    // start thread to send data from the buffer
    send_thread = std::thread(this->send_loop, std::ref(send_buffer));
}

template <class SendProto, class ReceiveProto>
void RobotCommunicator<SendProto, ReceiveProto>::send_proto(const SendProto& proto)
{
    send_buffer.push(proto);
}

template <class SendProto, class ReceiveProto>
void RobotCommunicator<SendProto, ReceiveProto>::send_loop(
    std::shared_ptr<ThreadSafeBuffer<SendProto>> buffer)
{
    do
    {
        in_destructor_mutex.unlock();

        std::optional<T> new_val =
            send_buffer->popMostRecentlyReceivedValue(Duration::fromSeconds(0.1));

        if (new_val)
        {
            medium.send_data((*new_val).SerializeAsString());

            if (sent_callback)
            {
                sent_callback(*new_val);
            }
        }

        in_destructor_mutex.lock();

    } while (!in_destructor);
}
