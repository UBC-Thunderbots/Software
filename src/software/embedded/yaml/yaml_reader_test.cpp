#include "software/embedded/yaml/yaml_reader.h"

#include <gtest/gtest.h>

#include <fstream>


TEST(YamlReadertest, test_can_read_basic_config)
{
    const char* file_content =
        "robot_id:             1\n"
        "channel_id:           2\n"
        "network_interface:    eno1\n"
        "kick_coeff:           1.1\n"
        "kick_constant:        1.2\n"
        "chip_pulse_width:     1.3";

    // writing to file
    std::ofstream file;
    file.open("/tmp/testing.yml");
    file << file_content;
    file.close();

    YamlReader reader("/tmp/testing.yml");

    ASSERT_EQ(reader.getValueDouble("kick_constant"), 1.2);
    ASSERT_EQ(reader.getValueDouble("kick_coeff"), 1.1);
    ASSERT_EQ(reader.getValueDouble("chip_pulse_width"), 1.3);
    ASSERT_EQ(reader.getValueString("network_interface"), "eno1");
    ASSERT_EQ(reader.getValueInt("robot_id"), 1);
    ASSERT_EQ(reader.getValueInt("channel_id"), 2);

    // cleanup
    std::remove("/tmp/testing.yml");
}
