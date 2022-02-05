#include "proto/tbots_software_msgs.pb.h"
#include "shared/2021_robot_constants.h"
#include "shared/constants.h"
#include "software/jetson_nano/thunderloop.h"
#include "software/logger/logger.h"
#include "software/world/robot_state.h"

// clang-format off
std::string BANNER =
"        ,/                                                                                       ,/    \n"
"      ,'/         _____ _   _ _   _ _   _ ____  _____ ____  _     ___   ___  ____              ,'/     \n"
"    ,' /         |_   _| | | | | | | \\ | |  _ \\| ____|  _ \\| |   / _ \\ / _ \\|  _ \\           ,' /      \n"
"  ,'  /_____,      | | | |_| | | | |  \\| | | | |  _| | |_ )| |  | | | | | | | |_) |        ,'  /_____, \n"
".'____    ,'       | | |  _  | |_| | |\\  | |_| | |___|  _ <| |__| |_| | |_| |  __/       .'____    ,'  \n"
"     /  ,'         |_| |_| |_|\\___/|_| \\_|____/|_____|_| \\_\\_____\\___/ \\___/|_|               /  ,'    \n"
"    / ,'                                                                                     / ,'      \n"
"   /,'                                                                                      /,'        \n"
"  /'                                                                                       /'          \n";
// clang-format on

int main(int argc, char** argv)
{
    std::cout << BANNER << std::endl;

    // TODO (#2338) replace with network logger
    LoggerSingleton::initializeLogger("/tmp");

    auto thunderloop =
            Thunderloop(CONTROL_LOOP_HZ, create2021RobotConstants(), create2021WheelConstants());
    thunderloop.run();

    return 0;
}
