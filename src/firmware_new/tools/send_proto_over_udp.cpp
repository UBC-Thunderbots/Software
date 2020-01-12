#include "boost/array.hpp"
#include "boost/asio.hpp"
#include "boost/bind.hpp"
#include "firmware_new/proto/control.pb.h"
#include "g3log/g3log.hpp"
#include "google/protobuf/message.h"
#include "software/multithreading/thread_safe_buffer.h"

using boost::asio::ip::udp;
using google::protobuf::Message;

int main(int argc, char *argv[])

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

// create a RobotCommunicator with a NetworkMedium
RobotCommunicator<control_msg, robot_ack> communicator(std::make_unique<NetworkMedium>(),
                                                       nullptr, nullptr);

// send proto
communicator.send_proto(control_req);

google::protobuf::ShutdownProtobufLibrary();

return 0;
}
