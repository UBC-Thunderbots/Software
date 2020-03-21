#include <chrono>
#include <thread>

#include "boost/array.hpp"
#include "boost/asio.hpp"
#include "boost/bind.hpp"
#include "g3log/g3log.hpp"
#include "google/protobuf/message.h"
#include "shared/constants.h"
#include "shared/proto/primitive.pb.h"
#include "shared/proto/vision.pb.h"
#include "software/ai/primitive/stop_primitive.h"
#include "software/backend/output/wifi/communication/robot_communicator.h"
#include "software/backend/output/wifi/communication/transfer_media/network_medium.h"
#include "software/backend/output/wifi/wifi_output.h"
#include "software/multithreading/thread_safe_buffer.h"


using boost::asio::ip::udp;
using google::protobuf::Message;

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        throw std::invalid_argument(
            "Please provide the interface you wish to multicast over");
    }

    GOOGLE_PROTOBUF_VERIFY_VERSION;

    // we create a wheel control msg, and request wheel 1 to spin at 100 rpm forwards
    // these wheel profile will be used across multiple wheels
    PrimitiveMsg prim;

    // create a WifiOutput with a PrimitiveCommunicator and VisionCommunicator
    WifiOutput wifi_output(
        std::move(std::make_unique<RobotPrimitiveCommunicator>(
            std::make_unique<NetworkMedium>(
                std::string(AI_PRIMITIVE_MULTICAST_ADDRESS) + "%" + std::string(argv[1]),
                AI_PRIMITIVE_MULTICAST_SEND_PORT, AI_PRIMITIVE_UNICAST_LISTEN_PORT),
            nullptr, nullptr)),
        std::move(std::make_unique<RobotVisionCommunicator>(
            std::make_unique<NetworkMedium>(
                std::string(AI_VISION_MULTICAST_ADDRESS) + "%" + std::string(argv[1]),
                AI_VISION_MULTICAST_SEND_PORT, AI_VISION_UNICAST_LISTEN_PORT),
            nullptr, nullptr)));

    while (1)
    {
        auto stopprim = std::make_unique<StopPrimitive>(9, false);
        std::vector<std::unique_ptr<Primitive>> primitives;
        primitives.push_back(std::move(stopprim));
        wifi_output.sendPrimitives(primitives);

        // 200 hz test
        std::this_thread::sleep_for(std::chrono::nanoseconds(5000000));
    }

    google::protobuf::ShutdownProtobufLibrary();

    return 0;
}
