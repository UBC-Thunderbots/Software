#include "software/proto/message_translation/ssl_wrapper.h"

#include <gtest/gtest.h>

TEST(SSLWrapperTest, test_create_empty_message)
{
    auto wrapper_packet = createWrapperPacket(nullptr, nullptr);
    EXPECT_FALSE(wrapper_packet->has_geometry());
    EXPECT_FALSE(wrapper_packet->has_detection());
}

TEST(SSLWrapperTest, test_create_wrapper_only_with_geometry)
{
    auto geometry_data = std::make_unique<SSL_GeometryData>();

    auto wrapper_packet = createWrapperPacket(std::move(geometry_data), nullptr);
    EXPECT_TRUE(wrapper_packet->has_geometry());
    EXPECT_FALSE(wrapper_packet->has_detection());
}

TEST(SSLWrapperTest, test_create_wrapper_only_with_detection_frame)
{
    auto detection_frame = std::make_unique<SSL_DetectionFrame>();

    auto wrapper_packet = createWrapperPacket(nullptr, std::move(detection_frame));
    EXPECT_FALSE(wrapper_packet->has_geometry());
    EXPECT_TRUE(wrapper_packet->has_detection());
}

TEST(SSLWrapperTest, test_create_wrapper_with_all_data)
{
    auto geometry_data   = std::make_unique<SSL_GeometryData>();
    auto detection_frame = std::make_unique<SSL_DetectionFrame>();

    auto wrapper_packet =
        createWrapperPacket(std::move(geometry_data), std::move(detection_frame));
    EXPECT_TRUE(wrapper_packet->has_geometry());
    EXPECT_TRUE(wrapper_packet->has_detection());
}
