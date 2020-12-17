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

#include <numeric>

static char linear_buffer[500];
static char circular_buffer[500];

/**
 * The following test uses the rapidcheck testing framework. Instead of trying to catch
 * edge cases with static tests, we define "commands" which are events that modify the
 * state of the buffers or System Under Test (Sut). We also define properties that should
 * hold true when these commands are applied in any order.
 *
 * rapidcheck then tries to break the defined properties with the given commands. If any
 * of the defined properties break when applying the commands, rapidcheck will log the
 * sequence of commands that lead to that failure, and the test will fail.
 *
 * Read more about it here: https://github.com/emil-e/rapidcheck/
 */

struct UbloxBuffersModel
{
    std::vector<std::string> data;
};

struct UbloxBuffersSut
{
    char *circular_buffer;
    char *linear_buffer;
    size_t buffer_length;
    size_t current_byte_position;
    size_t last_written_byte_position;
    size_t last_parsed_byte_position;
};

struct PlaceDataInCircularBufferCommand
    : rc::state::Command<UbloxBuffersModel, UbloxBuffersSut>
{
    std::string data;

    void apply(UbloxBuffersModel &model) const override
    {
        model.data.emplace_back(data);
    }

    void run(const UbloxBuffersModel &model, UbloxBuffersSut &sut) const override
    {
        if (data.length() + sut.last_written_byte_position < sut.buffer_length)
        {
            memcpy(sut.circular_buffer + sut.last_written_byte_position, &data[0],
                   data.length());
        }
        else
        {
            size_t buffer_remaining = sut.buffer_length - sut.last_written_byte_position;

            memcpy(sut.circular_buffer + sut.last_written_byte_position, &data[0],
                   buffer_remaining);
            memcpy(sut.circular_buffer, &data[0] + buffer_remaining,
                   data.length() - buffer_remaining);
        }

        sut.current_byte_position =
            (sut.current_byte_position + data.length()) % sut.buffer_length;
        sut.last_written_byte_position = sut.current_byte_position;
    }
};

struct ATCommandRequest : PlaceDataInCircularBufferCommand
{
    ATCommandRequest()
    {
        std::vector<std::string> valid_at_commands = {
            "AT+UMLA=2,00AAAAAAAA00\r",
            "AT&W\r",
            "AT+UBGRCGA=0,3\r",
            "AT\r",
            "AT+UFACTORY\r",
            "AT+CPWROFF\r",
            "AT+UBRGC=0,0,0\r",
            "AT+UETHCA=3\r",
            "AT+UWSC=0,0,0\r",
        };

        data = *rc::gen::elementOf(valid_at_commands);
    }

    void run(const UbloxBuffersModel &model, UbloxBuffersSut &sut) const override
    {
        PlaceDataInCircularBufferCommand::run(model, sut);
        RC_ASSERT(UBLOX_RESPONSE_INCOMPLETE ==
                  io_ublox_odinw262_communicator_getUbloxResponseStatusFromCircularBuffer(
                      sut.current_byte_position, sut.buffer_length,
                      (uint8_t *)sut.circular_buffer));
    }

    void show(std::ostream &os) const override
    {
        os << "Robot requesting: " << data;
    }
};

struct ATCommandResposne : PlaceDataInCircularBufferCommand
{
    ATCommandResposne()
    {
        std::vector<std::string> valid_at_reponses = {
            "ERROR\r\n",
            "OK\r\n",
        };

        data = *rc::gen::elementOf(valid_at_reponses);
    }

    void apply(UbloxBuffersModel &model) const override
    {
        model.data.clear();
    }

    void run(const UbloxBuffersModel &model, UbloxBuffersSut &sut) const override
    {
        PlaceDataInCircularBufferCommand::run(model, sut);

        if (data == std::string("ERROR\r\n"))
        {
            RC_ASSERT(
                UBLOX_RESPONSE_ERROR ==
                io_ublox_odinw262_communicator_getUbloxResponseStatusFromCircularBuffer(
                    sut.current_byte_position, sut.buffer_length,
                    (uint8_t *)sut.circular_buffer));
        }
        else if (data == std::string("OK\r\n"))
        {
            RC_ASSERT(
                UBLOX_RESPONSE_OK ==
                io_ublox_odinw262_communicator_getUbloxResponseStatusFromCircularBuffer(
                    sut.current_byte_position, sut.buffer_length,
                    (uint8_t *)sut.circular_buffer));
        }

        io_ublox_odinw262_communicator_extractResponseFromCircularBuffer(
            sut.last_parsed_byte_position, sut.current_byte_position, sut.buffer_length,
            (uint8_t *)(sut.circular_buffer), sut.linear_buffer);

        std::string expected_response =
            std::accumulate(model.data.begin(), model.data.end(), std::string("")) + data;
        RC_ASSERT(expected_response == std::string(sut.linear_buffer));

        sut.last_parsed_byte_position = sut.current_byte_position;
    }
    void show(std::ostream &os) const override
    {
        os << "u-blox ODIN-W262 responding: " << data;
    }
};

struct RandomUbloxGarbage : PlaceDataInCircularBufferCommand
{
    RandomUbloxGarbage()
    {
        std::vector<std::string> random_garbage = {
            "+UETHCAL",
            "+RANDOM",
            "+UO:ASD1",
            "+UU:123",
        };

        data = *rc::gen::elementOf(random_garbage);
    }

    void run(const UbloxBuffersModel &model, UbloxBuffersSut &sut) const override
    {
        PlaceDataInCircularBufferCommand::run(model, sut);
        RC_ASSERT(UBLOX_RESPONSE_INCOMPLETE ==
                  io_ublox_odinw262_communicator_getUbloxResponseStatusFromCircularBuffer(
                      sut.current_byte_position, sut.buffer_length,
                      (uint8_t *)sut.circular_buffer));
    }

    void show(std::ostream &os) const override
    {
        os << "random garbage from ublox: " << data;
    }
};

TEST(UbloxBufferTests, test_extract_response_from_circular_buffer)
{
    auto result = rc::check([&] {
        UbloxBuffersSut sut = {};
        sut.circular_buffer = circular_buffer;
        sut.linear_buffer   = linear_buffer;
        sut.buffer_length   = 500;

        UbloxBuffersModel model;
        rc::state::check(
            model, sut,
            rc::state::gen::execOneOfWithArgs<ATCommandRequest, ATCommandResposne,
                                              RandomUbloxGarbage>());
    });

    // This will not really be useful, however, rc::check should have printed the exact
    // sequence of ATCommandRequest, ATCommandResposne, and RandomUbloxGarbage commands
    // that caused the failure.
    EXPECT_TRUE(result);
}
