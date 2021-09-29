#include "software/logger/network_sink.h"

#include <gtest/gtest.h>

#include <experimental/filesystem>
#include <fstream>

#include "software/logger/network_logger.h"

TEST(network_log_test, test_network_log)
{
    std::unique_ptr<g3::LogWorker> logWorker = g3::LogWorker::createLogWorker();
    auto network_sink_handle = logWorker->addSink(std::make_unique<NetworkSink>(0,"oops0",0),
                                                  &NetworkSink::sendToNetwork);
    g3::initializeLogging(logWorker.get());

    LOG(INFO) << "test1";
    LOG(WARNING) << "t2t3";
    LOG(INFO) << "t4,t5,t6\ns1,s2,s3";

    // wait for asynchronous logger
    sleep(1);
}