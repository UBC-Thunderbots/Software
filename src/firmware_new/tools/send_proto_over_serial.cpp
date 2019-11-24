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
#include "firmware_new/proto/control.pb.h"
#include "google/protobuf/util/delimited_message_util.h"
#include "g3log/g3log.hpp"

void send_proto_over_serial(boost::asio::serial_port &port, const google::protobuf::Message& proto_msg);

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
    wheel_control.set_rpm(100);
    wheel_control.set_forwards(true);

    // turn two of the wheels on with this profile
    // NOTE that the other two wheels are not being populated
    control_msg control_req;
    control_req.mutable_wheel_1_control()->CopyFrom(wheel_control);
    control_req.mutable_wheel_2_control()->CopyFrom(wheel_control);

    send_proto_over_serial(port, control_req);

    // shutdown
    port.close();
    google::protobuf::ShutdownProtobufLibrary();

    return 0;
}

void send_proto_over_serial(boost::asio::serial_port &port, const google::protobuf::Message& proto_msg){

    auto size_of_msg = proto_msg.ByteSizeLong();
    uint8_t msg_buf[size_of_msg];
    std::cout << "Sending protobuf with size: " << size_of_msg << std::endl;

    // we need to encode the size of the msg being sent over serial into the msg due
    // to the nature of how the micro recieves these values. Regardless of DMA or a purely Interrupt based
    // approach on the stm32, it is difficult to know where the msg begins and ends,
    // (even using pb_decode_delimited and writeDelimitedTo are tricky due to how this data is received
    // on the stm32, as we can't use in/out streams the same way c++ server/clients can)
    // So we consistently send the size of the msg in the first 4 bytes, as our
    // custom "framing" (size is stored in little endian)
    uint8_t send_buf[size_of_msg + 4];
    send_buf[3] = (size_of_msg >> 24) & 0xFF;
    send_buf[2] = (size_of_msg >> 16) & 0xFF;
    send_buf[1] = (size_of_msg >> 8) & 0xFF;
    send_buf[0] = (size_of_msg & 0xFF);

    proto_msg.SerializeToArray(&msg_buf, size_of_msg);

    // append the rest of the message to the length prefixed buffer
    for (int k = 0; k < size_of_msg; k++)
    {
        send_buf[4 + k] = msg_buf[k];
    }

    boost::asio::write(port, boost::asio::buffer(&send_buf, size_of_msg + 4));
    std::cout << "Sent! waiting for ack" << std::endl;

    boost::asio::read(port, boost::asio::buffer(&test, 1));
    ack_msg.ParseFromArray(&indata, insize);
    std::cerr<<"COMING BACK"<<ack_msg.result()<<std::endl;
}
