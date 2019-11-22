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

#include "boost/asio/serial_port.hpp" 
#include "boost/asio.hpp" 
#include "firmware_new/proto/robot.pb.h"

int main()
{
    boost::asio::io_service io;
    boost::asio::serial_port port(io);

    port.open("/dev/ttyACM0");
    port.set_option(boost::asio::serial_port_base::baud_rate(115200));

    char c[3];
    char d[3];
    c[0] = 'l';
    c[1] = 'o';
    c[2] = 'l';

     //Read 1 character into c, this will block
      // forever if no character arrives.
      while(1){
            std::cout<<"BEFORE WRITE: "<<c[0]<<c[1]<<c[2]<<std::endl;
            boost::asio::write(port, boost::asio::buffer(c, 3)); 
            std::cout<<"AFTER READ: "<<d[0]<<d[1]<<d[2]<<std::endl;
            boost::asio::read(port, boost::asio::buffer(&d, 3));
      }
           port.close();
               
    return 0;
}
