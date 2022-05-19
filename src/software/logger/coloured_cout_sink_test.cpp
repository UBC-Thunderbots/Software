#include "software/logger/coloured_cout_sink.h"

#include <gtest/gtest.h>

#include <iostream>
#include <sstream>

#include "software/logger/logger.h"

class ColouredCoutSinkTest : public testing::TestWithParam<std::tuple<LEVELS, FG_Colour>>
{
};
const std::string test_str =
    "Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt "
    "ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation "
    "ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in "
    "reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur "
    "sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id "
    "est laborum.";
const std::string reset_colour_suffix = "\n\033[m\x1B[37m \x1B[m";

TEST_P(ColouredCoutSinkTest, testLogInfo)
{
    std::unique_ptr<g3::LogWorker> logWorker = g3::LogWorker::createLogWorker();
    auto colour_cout_sink_handle             = logWorker->addSink(
        std::make_unique<ColouredCoutSink>(true), &ColouredCoutSink::displayColouredLog);

    // We need to shut down logging started in tbots gtest main to setup the csv sinks
    g3::internal::shutDownLogging();
    g3::initializeLogging(logWorker.get());

    LEVELS level = std::get<0>(GetParam());
    const std::string colour_prefix =
        "\033[" + ColouredCoutSink::colourToString(std::get<1>(GetParam())) + "m";

    testing::internal::CaptureStdout();

    LOG(level) << test_str;
    // wait for asynchronous logger
    sleep(1);
    std::string output = testing::internal::GetCapturedStdout();

    // remove timestamp info from the log message
    output.erase(colour_prefix.length(), output.find("Lorem") - colour_prefix.length());

    EXPECT_EQ(output, colour_prefix + test_str + reset_colour_suffix);
}

TEST_P(ColouredCoutSinkTest, testLogInfoNoDetails)
{
    std::unique_ptr<g3::LogWorker> logWorker = g3::LogWorker::createLogWorker();
    auto colour_cout_sink_handle             = logWorker->addSink(
        std::make_unique<ColouredCoutSink>(false), &ColouredCoutSink::displayColouredLog);

    // We need to shut down logging started in tbots gtest main to setup the csv sinks
    g3::internal::shutDownLogging();
    g3::initializeLogging(logWorker.get());

    LEVELS level = std::get<0>(GetParam());
    const std::string colour_prefix =
        "\033[" + ColouredCoutSink::colourToString(std::get<1>(GetParam())) + "m";

    testing::internal::CaptureStdout();

    LOG(level) << test_str;
    // wait for asynchronous logger
    sleep(1);
    std::string output = testing::internal::GetCapturedStdout();

    EXPECT_EQ(output, colour_prefix + test_str + reset_colour_suffix);
}

// LOG(FATAL) isn't tested because it causes a SIGABRT
INSTANTIATE_TEST_CASE_P(
    All, ColouredCoutSinkTest,
    ::testing::Values(
        std::make_tuple<LEVELS, FG_Colour>(LEVELS(INFO), FG_Colour::WHITE),
        std::make_tuple<LEVELS, FG_Colour>(LEVELS(DEBUG), FG_Colour::GREEN),
        std::make_tuple<LEVELS, FG_Colour>(LEVELS(ROBOT_STATUS), FG_Colour::WHITE),
        std::make_tuple<LEVELS, FG_Colour>(LEVELS(WARNING), FG_Colour::YELLOW)));
