#include "software/jetson_nano/thunderloop.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "proto/robot_status_msg.pb.h"
#include "software/jetson_nano/redis/redis_client.h"
#include "software/jetson_nano/services/motor.h"
#include "software/jetson_nano/services/network.h"
#include "software/jetson_nano/services/power.h"

class MockMotorService : public MotorService
{
   public:
    TbotsProto::DriveUnitStatus poll(
        const TbotsProto::DirectControlPrimitive& direct_control) override
    {
        return TbotsProto::DriveUnitStatus();
    }
    MOCK_METHOD(uint8_t, tmc4671ReadWriteByte,
                (uint8_t motor, uint8_t data, uint8_t last_transfer), (override));
    MOCK_METHOD(uint8_t, tmc6100ReadWriteByte,
                (uint8_t motor, uint8_t data, uint8_t last_transfer), (override));
    MOCK_METHOD(void, calibrateEncoder, (uint8_t motor), (override));
    MOCK_METHOD(void, runOpenLoopCalibrationRoutine, (uint8_t motor, size_t num_samples),
                (override));
};

class MockPowerService : public PowerService
{
   public:
    MockPowerService() : PowerService() {}

    TbotsProto::PowerStatus poll(const TbotsProto::PowerControl& control) override
    {
        return TbotsProto::PowerStatus();
    }
};

class MockNetworkService : public NetworkService
{
   public:
    MockNetworkService() : NetworkService("0.0.0.0", 5000, 7000, 8000, true) {}
    std::tuple<TbotsProto::PrimitiveSet, TbotsProto::World> poll(
        const TbotsProto::RobotStatus& robot_status) override
    {
        TbotsProto::PrimitiveSet primitive_set_msg;
        TbotsProto::World world_msg;
        return std::tuple<TbotsProto::PrimitiveSet, TbotsProto::World>{primitive_set_msg,
                                                                       world_msg};
    }
};

class MockRedisClient : public RedisClient
{
   public:
    MockRedisClient() : RedisClient("mock", 0000) {}

    std::string get(const std::string& key) override
    {
        return key_value_set_[key];
    }

    void set(const std::string& key, const std::string& value) override
    {
        key_value_set_[key] = value;
    }

    std::unordered_map<std::string, std::string> key_value_set_;
};


TEST(ThunderloopTest, test_thunderloop_init)
{
    MockMotorService motor_service;
    MockPowerService power_service;
    MockNetworkService network_service;
    MockRedisClient redis_client;
}
