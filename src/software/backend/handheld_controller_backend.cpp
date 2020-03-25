#include "software/backend/handheld_controller_backend.h"

#include "software/time/timestamp.h"
#include "software/util/design_patterns/generic_factory.h"

const std::string HandheldControllerBackend::name = "handheld_controller";

HandheldControllerBackend::HandheldControllerBackend()
    : HandheldControllerBackend(std::make_shared<const HandheldControllerInputConfig>())
{
}

HandheldControllerBackend::HandheldControllerBackend(
    std::shared_ptr<const HandheldControllerInputConfig> controller_input_config)
    : radio_output(DEFAULT_RADIO_CONFIG,
                   [this](RobotStatus status) {
                       Subject<RobotStatus>::sendValueToObservers(status);
                   }),
      controller_input_config(controller_input_config)
{
}

void HandheldControllerBackend::onValueReceived(ConstPrimitiveVectorPtr primitives_ptr)
{
    radio_output.sendPrimitives(*primitives_ptr);

    // Because the DirectVelocityPrimitive bases itself off the robot's
    // internal state, and this state is very prone to drifting due to the
    // lack of camera data, we spoof camera data telling the robot it is
    // always at (0, 0) to prevent drift
    Robot robot =
        Robot(controller_input_config->RobotId()->value(), Point(0, 0), Vector(0, 0),
              Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Team friendly_team = Team(Duration::fromSeconds(1.0), {robot});
    Ball ball          = Ball(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    radio_output.sendVisionPacket(friendly_team, ball);
}

// Register this backend in the genericFactory
static TGenericFactory<std::string, Backend, HandheldControllerBackend> factory;
