#include <chrono>
#include <thread>

#include "boost/array.hpp"
#include "boost/asio.hpp"
#include "boost/bind.hpp"
#include "firmware_new/proto/control.pb.h"
#include "firmware_new/tools/communication/robot_communicator.h"
#include "firmware_new/tools/communication/transfer_media/network_medium.h"
#include "g3log/g3log.hpp"
#include "google/protobuf/message.h"
#include "software/multithreading/thread_safe_buffer.h"


using boost::asio::ip::udp;
using google::protobuf::Message;

int main(int argc, char* argv[])
{
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

    int count = 0;

    // create a RobotCommunicator with a NetworkMedium
    RobotCommunicator<control_msg, robot_ack> communicator(
        std::make_unique<NetworkMedium>("192.168.1.140", 42069),
        [&](const control_msg& msg) {
            std::cout << "COMP Txed " << count++ << " msgs " << std::endl;
        },
        [&](const robot_ack& msg) {
            std::cout << "STM32 Rxed " << msg.msg_count() << " msgs " << std::endl;
        });


    while (1)
    {
        communicator.send_proto(control_req);

        // 4000 hz test
        std::this_thread::sleep_for(std::chrono::nanoseconds(250000));
    }

    google::protobuf::ShutdownProtobufLibrary();

    return 0;
}
