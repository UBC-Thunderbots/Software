#pragma once
#include "boost/array.hpp"
#include "boost/asio.hpp"
#include "boost/bind.hpp"
#include "g3log/g3log.hpp"
#include "google/protobuf/message.h"
#include "software/backend/output/wifi/communication/transfer_media/transfer_medium.h"
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
template <class SendProto>
using MsgSentCallback = std::function<void(const SendProto& msg)>;

template <class ReceiveProto>
using MsgReceivedCallback = std::function<void(const ReceiveProto& msg)>;

template <class SendProto, class ReceiveProto>
class RobotCommunicator
{
   public:
    // Enforce TransferMedium dependency injection by deleting the default constructor
    RobotCommunicator() = delete;

    // Delete the copy and assignment operators to avoid doing anything nasty
    // and to protect ownership of the TransferMedium
    RobotCommunicator& operator=(const RobotCommunicator&) = delete;
    RobotCommunicator(const RobotCommunicator&)            = delete;

    virtual ~RobotCommunicator();

    /*
     * Inject a TransferMedium as a dependency, used to transmit protobuf.
     * The callbacks to be executed on send/receive of a msg. Use nullptr for no callback.
     *
     * @param medium TransferMedium for the protobuf to be sent through
     * @param sent_callback The callback to run when a msg has been sent
     * @param received_callback The callback to run when a msg has been received
     */
    RobotCommunicator(std::unique_ptr<TransferMedium> medium,
                      MsgSentCallback<SendProto> sent_callback,
                      MsgReceivedCallback<ReceiveProto> received_callback);

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
     */
    void send_proto(const SendProto& proto);
    void send_proto_vector(const std::vector<SendProto>& protos);


   private:
    // function that runs in the send_thread
    void send_loop(std::shared_ptr<ThreadSafeBuffer<SendProto>> buffer);

    // callbacks
    MsgSentCallback<SendProto> sent_callback;
    MsgReceivedCallback<ReceiveProto> received_callback;

    // thread to handle sending
    std::thread send_thread;

    // flags to handle shutting down thread
    bool in_destructor;

    // mutex to protect in_destructor flag
    std::mutex in_destructor_mutex;

    // medium to transfer data through
    std::unique_ptr<TransferMedium> medium;

    // buffer that holds msgs to be sent
    std::shared_ptr<ThreadSafeBuffer<SendProto>> send_buffer;
};
