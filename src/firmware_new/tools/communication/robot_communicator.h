#pragma once
#include "boost/array.hpp"
#include "boost/asio.hpp"
#include "boost/bind.hpp"
#include "firmware_new/proto/control.pb.h"
#include "g3log/g3log.hpp"
#include "software/multithreading/thread_safe_buffer.h"

using boost::asio::ip::udp;
using google::protobuf::Message;

/*
 * Broadcasts and listens to protobuf msgs through specified TransferMedium. This class is
 * templated to specify which msg types to send and receive. Any msgs that are received
 * that do not match that type will be dropped.
 *
 * This class takes a TransferMedium as a dependency which indicates how the msg is to be
 * transfered to the Robot (over SPI, UART, ETHERNET, ...)
 *
 * On construction, two thread-safe buffers will be started and a connection will be
 * established. Msgs will be processed until this class is destroyed.
 *
 * NOTE: All receiving is done asynchronously.
 *
 */
template <class SendProto, class ReceiveProto>

using MsgSentCallback     = std::function<void(const SendProto& msg)>;
using MsgReceivedCallback = std::function<void(const ReceiveProto& msg)>;

class RobotCommunicator
{
   public:
    // Enforce TransferMedium dependency injection by deleting the default constructor
    RobotCommunicator() = delete;

    // Delete the copy and assignment operators to avoid doing anything nasty
    // and to protect ownership of the TransferMedium
    RobotCommunicator& operator=(const RobotCommunicator&) = delete;
    RobotCommunicator(const RobotCommunicator&)            = delete;

    virtual ~RobotCommunicator() = default;

    /*
     * Inject a TransferMedium as a dependency, used to transmit protobuf. A unique
     * ptr is used so that it is moved into this class and the class gets full ownership.
     *
     * The callbacks to be executed on send/receive of a msg. Use nullptr for no callback.
     *
     * @param medium TransferMedium for the protobuf to be sent through
     * @param sent_callback The callback to run when a msg has been sent
     * @param received_callback The callback to run when a msg has been received
     */
    RobotCommunicator(std::unique_ptr<TransferMedium> medium,
                      const MsgSentCallback& sent_callback,
                      const MsgReceivedCallback& received_callback);

    /*
     * Send proto over TransferMedium asynchronously, returns immediately.
     * The msg to be sent is loaded in the thread-safe buffer and sent at
     * as soon as the msg reaches the front of the buffer.
     *
     * If the blocking flag is true (by default its false), then the function
     * blocks until the buffer is empty before returning.
     *
     * Callback is executed when the msg sends (if not nullptr)
     *
     * @param proto The msg to send over medium
     * @param blocking Defaults to false, set to true if call needs to be blocking
     */
    void send_proto(const SendProto& proto, bool blocking = false);


   private:
    // function that runs in the send_thread
    void send_loop(std::shared_ptr<ThreadSafeBuffer<SendProto>> buffer);

    // function that runs in the receive_thread
    void receive_loop(std::shared_ptr<ThreadSafeBuffer<ReceiveProto>> buffer);

    // thread to handle sending
    std::thread send_thread;

    // thread to handle receiving
    std::thread receive_thread;

    // medium to transfer data through
    std::unique_ptr<TransferMedium> medium;

    // buffer that holds msgs to be sent
    std::shared_ptr<ThreadSafeBuffer<SendProto>> send_buffer;

    // buffer to hold incoming msgs
    std::shared_ptr<ThreadSafeBuffer<ReceiveProto>> recv_buffer;
};
