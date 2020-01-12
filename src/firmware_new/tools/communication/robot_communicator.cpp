#include "firmware_new/tools/robot_communicator.h"

#include "firmware_new/tools/medium/transfer_medium.h"
#include "software/multithreading/thread_safe_buffer.h"

using boost::asio::ip::udp;
using google::protobuf::Message;

template <class SendProto, class ReceiveProto>

RobotCommunicator::RobotCommunicator(std::unique_ptr<TransferMedium> medium,
                                     const MsgSentCallback& sent_callback,
                                     const MsgReceivedCallback& received_callback)
    : medium(medium)
{
    // connect to the medium
    medium->connect();

    // start thread to send data from the buffer
    send_thread = std::thread(this->send_loop, std::ref(send_buffer));

    // start thread to process receive buffer
    receive_thread = std::thread(this->receive_loop, std::ref(recv_buffer));

    virtual std::string receive_data(std::function<void(std::string)>) = 0;
}

RobotCommunicator::send_proto(const SendProto& proto)
{
    send_buffer.push(proto);
}

RobotCommunicator::send_loop(std::shared_ptr<ThreadSafeBuffer<SendProto>> buffer)
{
    do
    {
        in_destructor_mutex.unlock();

        std::optional<T> new_val =
            send_buffer->popMostRecentlyReceivedValue(Duration::fromSeconds(0.1));

        if (new_val)
        {
            medium->send_data((*new_val).SerializeAsString());
            sent_callback(*new_val);
        }

        in_destructor_mutex.lock();

    } while (!in_destructor);
}

RobotCommunicator::receive_loop(std::shared_ptr<ThreadSafeBuffer<ReceiveProto>> buffer)
{
    do
    {
        in_destructor_mutex.unlock();

        std::optional<T> new_val =
            recv_buffer->popMostRecentlyReceivedValue(Duration::fromSeconds(0.1));

        if (new_val)
        {
            medium->send_data((*new_val).SerializeAsString());
            sent_callback(*new_val);
        }

        in_destructor_mutex.lock();

    } while (!in_destructor);
}
