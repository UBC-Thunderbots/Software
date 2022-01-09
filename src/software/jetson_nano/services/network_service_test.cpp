#include "software/jetson_nano/services/network.h"
#include "software/backend/simulator_backend.h"

#include <gtest/gtest.h>

TEST(NetworkTest, test_receive)
{
    
    //auto network_service = NetworkService("ff02::c3d0:42d2:bb01", VISION_PORT,PRIMITIVE_PORT, ROBOT_STATUS_PORT, true);
    auto simulator_backend = SimulatorBackend(std::make_shared<const ThunderbotsConfig>()->getBackendConfig());

    EXPECT_EQ(180, 180);
}
