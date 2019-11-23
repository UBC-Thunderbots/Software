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
#include "firmware_new/proto/robot.pb.h"

int main()
{
    boost::asio::io_service io;
    boost::asio::serial_port port(io);

    port.open("/dev/ttyACM0");
    port.set_option(boost::asio::serial_port_base::baud_rate(115200));

    char c[3];
    char d[3];
    c[0] = 'u';
    c[1] = 'u';
    c[2] = 'u';

    // Verify that the version of the library that we linked against is
    // compatible with the version of the headers we compiled against.
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    robot_msg test_msg;
    robot_ack ack_msg;
    test_msg.set_timestamp(100);
    test_msg.set_operand1(100);
    test_msg.set_operand2(62);

    // Read 1 character into c, this will block
    // forever if no character arrives.
    while (1)
    {
		 boost::asio::streambuf b;
		 std::ostream os(&b);

		// grab the size of the serialized msg, we will also be delimiting by this msg
		 int outsize = 35;
		 int insize = 22;
		 char outdata[outsize];
		 char indata[insize];
         char test;

		 test_msg.SerializeToArray(&outdata, outsize);

		 std::cerr<<"SENDING.........."<<std::endl;
		 boost::asio::write(port, boost::asio::buffer(&outdata, outsize));

         for (int i = 0; i < 22; i++){
		    boost::asio::read(port, boost::asio::buffer(&test, 1));
            std::cerr<<i<<":"<<test<<std::endl;
            indata[i] = test;
         }

		 ack_msg.ParseFromArray(&indata, insize);
		 std::cerr<<"COMING BACK"<<ack_msg.result()<<std::endl;

        //std::cout << "BEFORE WRITE: " << c[0] << c[1] << c[2] << std::endl;
        //boost::asio::write(port, boost::asio::buffer(c, 3));
        //boost::asio::read(port, boost::asio::buffer(&d, 3));
        //std::cout << "AFTER READ: " << d[0] << d[1] << d[2] << std::endl;
    }

    port.close();

    // Optional:  Delete all global objects allocated by libprotobuf.
    google::protobuf::ShutdownProtobufLibrary();

    return 0;
}
