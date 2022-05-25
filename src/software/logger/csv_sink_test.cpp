#include "software/logger/csv_sink.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "software/logger/logger.h"

class CSVSinkTest : public testing::TestWithParam<std::tuple<LEVELS>>
{
};
const std::string logging_dir = "software/logger";
TEST(CSVSinkTest, test_csv_log_appends)
{
    std::unique_ptr<g3::LogWorker> logWorker = g3::LogWorker::createLogWorker();
    auto csv_sink_handle = logWorker->addSink(std::make_unique<CSVSink>(logging_dir),
                                              &CSVSink::appendToFile);
    g3::initializeLogging(logWorker.get());

    LOG(CSV, "test_file.csv") << "t1,t2,t3";
    LOG(CSV, "test_file.csv") << "t7,t8,t9";
    LOG(CSV, "test_file.csv") << "t4,t5,t6\ns1,s2,s3";

    // wait for asynchronous logger
    sleep(1);

    std::ifstream read_test(logging_dir + "/test_file.csv", std::ios::in);
    std::string output((std::istreambuf_iterator<char>(read_test)),
                       std::istreambuf_iterator<char>());
    EXPECT_EQ(output, "t1,t2,t3t7,t8,t9t4,t5,t6\ns1,s2,s3");
}

TEST_P(CSVSinkTest, test_csv_log_levels_not_logging)
{
    std::unique_ptr<g3::LogWorker> logWorker = g3::LogWorker::createLogWorker();
    auto csv_sink_handle = logWorker->addSink(std::make_unique<CSVSink>(logging_dir),
                                              &CSVSink::appendToFile);
    g3::initializeLogging(logWorker.get());

    LOG(std::get<0>(GetParam()), "test_file2.csv") << "n1,n2,n3";

    // wait for asynchronous logger
    sleep(1);

    std::ifstream read_test("software/logger/test_file2.csv", std::ios::in);
    EXPECT_FALSE(read_test.is_open());
}

INSTANTIATE_TEST_CASE_P(All, CSVSinkTest,
                        ::testing::Values(std::make_tuple<LEVELS>(LEVELS(INFO)),
                                          std::make_tuple<LEVELS>(LEVELS(DEBUG)),
                                          std::make_tuple<LEVELS>(LEVELS(ROBOT_STATUS)),
                                          std::make_tuple<LEVELS>(LEVELS(WARNING))));
