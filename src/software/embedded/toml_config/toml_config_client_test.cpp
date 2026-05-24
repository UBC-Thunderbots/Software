#include "software/embedded/toml_config/toml_config_client.h"

#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>

class TomlConfigClientTest : public ::testing::Test
{
   protected:
    void TearDown() override
    {
        if (!config_path_.empty() && std::filesystem::exists(config_path_))
        {
            std::remove(config_path_.c_str());
        }
    }

    std::string config_path_;
};

TEST_F(TomlConfigClientTest, create_with_nonexistent_file_creates_default_config)
{
    config_path_ = "/tmp/toml_config_test_default.toml";
    std::remove(config_path_.c_str());

    TomlConfigClient client(config_path_);

    EXPECT_EQ(client.get("robot_id"), "1");
    EXPECT_EQ(client.get("channel_id"), "0");
    EXPECT_EQ(client.get("network_interface"), "tbotswifi5");
}

TEST_F(TomlConfigClientTest, get_nonexistent_key_returns_empty_string)
{
    config_path_ = "/tmp/toml_config_test_nonexistent_key.toml";
    std::remove(config_path_.c_str());

    TomlConfigClient client(config_path_);

    EXPECT_EQ(client.get("nonexistent_key"), "");
}

TEST_F(TomlConfigClientTest, set_then_get_round_trip)
{
    config_path_ = "/tmp/toml_config_test_round_trip.toml";
    std::remove(config_path_.c_str());

    TomlConfigClient client(config_path_);

    client.set("robot_id", "42");
    EXPECT_EQ(client.get("robot_id"), "42");

    client.set("custom_key", "custom_value");
    EXPECT_EQ(client.get("custom_key"), "custom_value");
}

TEST_F(TomlConfigClientTest, set_no_commit_then_flush_then_get)
{
    config_path_ = "/tmp/toml_config_test_no_commit.toml";
    std::remove(config_path_.c_str());

    TomlConfigClient client(config_path_);

    client.setNoCommit("robot_id", "99");
    EXPECT_EQ(client.get("robot_id"), "99");

    client.flush();

    TomlConfigClient client2(config_path_);
    EXPECT_EQ(client2.get("robot_id"), "99");
}

TEST_F(TomlConfigClientTest, load_existing_file_preserves_values)
{
    config_path_ = "/tmp/toml_config_test_existing.toml";
    std::remove(config_path_.c_str());

    {
        TomlConfigClient client(config_path_);
        client.set("robot_id", "7");
    }

    TomlConfigClient client(config_path_);
    EXPECT_EQ(client.get("robot_id"), "7");
}
