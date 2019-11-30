/*
 * This tool sends protobuf over serial to the STM32 to do simple arithmatic. It serves
 * as a placeholder to demonstrate protobuf communication, as well as facilitate fw
 * development.
 *
 * Simply modify robot.proto in firmware_new/proto and run this tool to get access to the
 * updated protobuf fields.
 *
 * We are using proto2 (mostly to be consistent with the ssl proto version)
 * For more info on protobuf, please go here:
 * https://developers.google.com/protocol-buffers/docs/cpptutorial
 *
 * NOTE: We include robot.pb.h which contains the required info to create msgs
 * defined in robot.proto. We link the cc_library w/ c++ headers to this tool.
 * On the microcontroller side we also include robot.pb.h, but we link the
 * cc_library w/ c headers.
 *
 * NOTE: Do not use libusb, it does not work out of the box
 *
 */

#include "boost/asio.hpp"
#include "boost/asio/serial_port.hpp"
#include "boost/crc.hpp"
#include "firmware_new/proto/control.pb.h"
#include "g3log/g3log.hpp"
#include "google/protobuf/util/delimited_message_util.h"

void send_proto_over_serial(boost::asio::serial_port& port,
                            const google::protobuf::Message& proto_msg);

//void receive_proto_over_serial(boost::asio::serial_port& port,
                            //const google::protobuf::Message& proto_msg)

int main()
{
    boost::asio::io_service io;
    boost::asio::serial_port port(io);

    // TODO update udev rules to make this constant?
    port.open("/dev/ttyACM0");
    port.set_option(boost::asio::serial_port_base::baud_rate(115200));

    // verify that the version of the library that we linked against is
    // compatible with the version of the headers we compiled against.
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    // we create a wheel control msg, and request wheel 1 to spin at 100 rpm forwards
    // these wheel profile will be used across multiple wheels
    wheel_speed_msg wheel_control;
    wheel_control.set_rpm(69);
    wheel_control.set_forwards(true);

    // turn two of the wheels on with this profile
    // NOTE that the other two wheels are not being populated
    control_msg control_req;
    control_req.mutable_wheel_1_control()->CopyFrom(wheel_control);
    control_req.mutable_wheel_2_control()->CopyFrom(wheel_control);
    control_req.mutable_wheel_2_control()->CopyFrom(wheel_control);

    for (int k =0; k<1000; k++)
        send_proto_over_serial(port, control_req);

    robot_ack ack;
    //recv_proto_over_serial<robot_ack>(port, ack);

    // shutdown
    //std::promise<void>().get_future().wait();
    port.close();
    google::protobuf::ShutdownProtobufLibrary();

    return 0;
}

/*
 * Given a boost::asio::serial_port and an arbitrary protomsg, serializes the msg
 * and sends it over the port. Assumes the port is open and does not close the port.
 *
 * NOTE: we assume that the receiver is using IDLE Line detection for delimiting. This
 * function can be repeatedly called to stream data
 *
 * @param port The port to send the data over
 * @param proto_msg The msg to serialize and send
 */
void send_proto_over_serial(boost::asio::serial_port& port,
                            const google::protobuf::Message& proto_msg)
{
    // serialize msg
    auto size_of_msg = proto_msg.ByteSizeLong();
    uint8_t msg_buf[size_of_msg];
    proto_msg.SerializeToArray(&msg_buf, size_of_msg);

    // compute and store crc checksum
    boost::crc_32_type result;
    result.process_bytes(msg_buf, size_of_msg);
    uint32_t checksum = result.checksum();

    // we encode the checksum in the first 4 bytes of the msg
    // little endian
    uint8_t send_buf[size_of_msg + 4];
    send_buf[3] = (checksum >> 24) & 0xFF;
    send_buf[2] = (checksum >> 16) & 0xFF;
    send_buf[1] = (checksum >> 8) & 0xFF;
    send_buf[0] = (checksum & 0xFF);

    for (int k = 0; k < size_of_msg; k++)
    {
        send_buf[k+4] = msg_buf[k];
    }

    boost::asio::write(port, boost::asio::buffer(&send_buf, size_of_msg + 4));

    // an "Idle Line" is detected when the received detects the line idle
    // for more than 1/baud_rate. We compute that value and create a blocking
    // delay
    boost::asio::serial_port_base::baud_rate baud_rate;
    port.get_option(baud_rate);

    std::this_thread::sleep_for(std::chrono::milliseconds(
                                    (int)1e6 / baud_rate.value()));
}
