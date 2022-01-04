#include "proto/tbots_software_msgs.pb.h"
#include "shared/2021_robot_constants.h"
#include "shared/constants.h"
#include "software/jetson_nano/thunderloop.h"
#include "software/logger/logger.h"
#include "software/world/robot_state.h"

int main(int argc, char** argv)
{
    // TODO (#2338) replace with network logger
    LoggerSingleton::initializeLogger("/tmp");

    auto thunderloop =
        Thunderloop(create2021RobotConstants(), create2021WheelConstants());
    thunderloop.run(CONTROL_LOOP_HZ);

    return 0;
}
