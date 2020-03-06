#include <chrono>
#include <thread>

#include "boost/array.hpp"
#include "boost/asio.hpp"
#include "boost/bind.hpp"
#include "google/protobuf/message.h"
#include "shared/proto/primitive.pb.h"
#include "software/ai/primitive/stop_primitive.h"
#include "software/backend/output/radio/radio_output.h"
#include "software/backend/radio_backend.h"
#include "software/constants.h"

using google::protobuf::Message;

int main(int argc, char* argv[])
{
    RadioOutput radio_output(0, [&](RobotStatus status) {});

    while (1)
    {
        std::cout << "SETING STOP PRIMITIVE" << std::endl;
        auto stopprim = std::make_unique<StopPrimitive>(0, false);

        std::vector<std::unique_ptr<Primitive>> primitives;
        primitives.push_back(std::move(stopprim));

        radio_output.sendPrimitives(primitives);

        // 4000 hz test
        std::cout << "SENT!" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    google::protobuf::ShutdownProtobufLibrary();

    return 0;
}
