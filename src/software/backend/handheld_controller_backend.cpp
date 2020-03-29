#include "software/backend/handheld_controller_backend.h"

#include "software/time/timestamp.h"
#include "software/util/design_patterns/generic_factory.h"
#include "shared/constants.h"
#include "software/backend/output/wifi/communication/transfer_media/network_medium.h"

const std::string HandheldControllerBackend::name = "handheld_controller";


// MAT CHANGE THIS to match your computer
// TODO remove!
const std::string INTERFACE = "eth0";

HandheldControllerBackend::HandheldControllerBackend()
    : HandheldControllerBackend(std::make_shared<const HandheldControllerInputConfig>())
{
}

HandheldControllerBackend::HandheldControllerBackend(
    std::shared_ptr<const HandheldControllerInputConfig> controller_input_config)
    #warning NOT FINALIZED DO NOT LET THIS GET INTO MASTER.
    : wifi_output(
          std::move(std::make_unique<RobotPrimitiveCommunicator>(
              std::make_unique<NetworkMedium>(
                  std::string(AI_PRIMITIVE_MULTICAST_ADDRESS) + INTERFACE,
                  AI_PRIMITIVE_MULTICAST_SEND_PORT, AI_PRIMITIVE_UNICAST_LISTEN_PORT),
              nullptr, nullptr)),
          std::move(std::make_unique<RobotVisionCommunicator>(
              std::make_unique<NetworkMedium>(
                  std::string(AI_VISION_MULTICAST_ADDRESS) + INTERFACE,
                  AI_VISION_MULTICAST_SEND_PORT, AI_VISION_UNICAST_LISTEN_PORT),
              nullptr, nullptr))),

      controller_input_config(controller_input_config)
{
}

void HandheldControllerBackend::onValueReceived(ConstPrimitiveVectorPtr primitives_ptr)
{
    wifi_output.sendPrimitives(*primitives_ptr);

    // Because the DirectVelocityPrimitive bases itself off the robot's
    // internal state, and this state is very prone to drifting due to the
    // lack of camera data, we spoof camera data telling the robot it is
    // always at (0, 0) to prevent drift
    #warning HACK DO NOT LET THIS GET INTO MASTER.
    // Robot robot =
         //Robot(controller_input_config->RobotId()->value(), Point(0, 0), Vector(0, 0),
               //Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    // Team friendly_team = Team(Duration::fromSeconds(1.0), {robot});
    // Ball ball          = Ball(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    // wifi_output.sendVisionPacket(friendly_team, ball);
}

// Register this backend in the genericFactory
static TGenericFactory<std::string, Backend, HandheldControllerBackend> factory;
