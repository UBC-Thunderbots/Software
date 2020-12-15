extern "C"
{
#include "firmware_new/boards/frankie_v1/io/ublox_odinw262_communicator_buffer_utils.h"
}

#include <gtest/gtest.h>
#include <rapidcheck.h>
#include <rapidcheck/state.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

static char linear_buffer[500];
static char circular_buffer[500];

struct UbloxBuffersModel
{
    std::vector<std::string> data;
    int total_data_size_bytes;
};

struct UbloxBuffersSut
{
    char *circular_buffer;
    char *linear_buffer;
    size_t buffer_length;
    size_t current_byte_position;
    size_t last_parsed_byte_position;
    size_t counter_no_wrap;
};

struct ATCommandRequest : rc::state::Command<UbloxBuffersModel, UbloxBuffersSut>
{
    std::string at_command;

    ATCommandRequest()
    {
        std::vector<std::string> valid_at_commands = {
            "AT+UMLA=2,00AAAAAAAA00\\r",
            "AT&W\\r",
            "AT+UBGRCGA=0,3\\r",
            "AT\\r",
            "AT+UFACTORY\\r",
            "AT+CPWROFF\\r",
            "AT+UBRGC=0,0,0\\r",
            "AT+UETHCA=3\\r",
            "AT+UWSC=0,0,0\\r",
        };

        at_command = *rc::gen::elementOf(valid_at_commands);
    }

    void apply(UbloxBuffersModel &model) const override
    {
        model.data.emplace_back(at_command);
    }

    void run(const UbloxBuffersModel &model, UbloxBuffersSut &sut) const override
    {
        if (at_command.length() + sut.last_parsed_byte_position < sut.buffer_length)
        {
            memcpy(sut.circular_buffer + sut.last_parsed_byte_position, &at_command[0],
                   at_command.length());
        }
        else
        {
            size_t buffer_remaining = sut.buffer_length - sut.last_parsed_byte_position;
            memcpy(sut.circular_buffer + sut.last_parsed_byte_position, &at_command[0],
                   buffer_remaining);
            memcpy(sut.circular_buffer, &at_command[0] + buffer_remaining,
                   at_command.length() - buffer_remaining);
        }
        sut.current_byte_position =
            (sut.current_byte_position + at_command.length()) % sut.buffer_length;

        io_ublox_odinw262_communicator_extractResponseFromCircularBuffer(
            sut.last_parsed_byte_position, sut.current_byte_position, sut.buffer_length,
            (uint8_t *)(sut.circular_buffer), (uint8_t *)(sut.linear_buffer));

        std::cerr << "Generated AT Command: " << at_command << std::endl;
        std::cerr << "Extracted AT Command: " << std::string(sut.linear_buffer)
                  << std::endl;
        ;
        RC_ASSERT(at_command == std::string(sut.linear_buffer));

        sut.last_parsed_byte_position = sut.current_byte_position;
    }

    void show(std::ostream &os) const override
    {
        os << "Requesting " << at_command;
    }
};

TEST(UbloxBufferTests, test_extract_response_from_circular_buffer)
{
    rc::check([&] {
        UbloxBuffersSut sut;
        sut.circular_buffer           = circular_buffer;
        sut.linear_buffer             = linear_buffer;
        sut.counter_no_wrap           = 0;
        sut.current_byte_position     = 0;
        sut.last_parsed_byte_position = 0;
        sut.buffer_length             = 500;

        UbloxBuffersModel model;
        rc::state::check(model, sut,
                         rc::state::gen::execOneOfWithArgs<ATCommandRequest>());
    });
}
