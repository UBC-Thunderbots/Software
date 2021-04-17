#include "software/logger/csv_sink.h"

#include <gtest/gtest.h>

#include <experimental/filesystem>
#include <fstream>

#include "software/logger/logger.h"

TEST(CSVSinkTest, test_csv_log)
{
    std::unique_ptr<g3::LogWorker> logWorker = g3::LogWorker::createLogWorker();
    auto csv_sink_handle =
        logWorker->addSink(std::make_unique<CSVSink>(""), &CSVSink::appendToFile);
    g3::initializeLogging(logWorker.get());

    EXPECT_NO_THROW(LOG(CSV, "test_file.csv") << "t1,t2,t3");
    EXPECT_NO_THROW(LOG(CSV, "test_file.csv") << "");
    EXPECT_NO_THROW(LOG(CSV, "test_file.csv") << "t1,t2,t3\ns1,s2,s3");
}
