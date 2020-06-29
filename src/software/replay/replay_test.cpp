#include <google/protobuf/util/message_differencer.h>
#include <gtest/gtest.h>

#include "software/multithreading/subject.h"
#include "software/replay/replay_logger.h"
#include "software/replay/replay_reader.h"

constexpr const char* REPLAY_TEST_PATH_SUFFIX = "software/replay/test_replay";

namespace fs = std::experimental::filesystem;

// hahaha test subject because it's a subject but also a test subject
class TestSubject : public Subject<SensorMsg>
{
   public:
    void sendValue(SensorMsg val)
    {
        sendValueToObservers(val);
    }
};

TEST(ReplayTest, test_spaghetti)
{
    ReplayReader reader(fs::current_path() / REPLAY_TEST_PATH_SUFFIX);
    while (reader.hasNextFrame()) {
        auto frame = reader.getNextFrame();
        std::cout << frame.ssl_vision_msg().detection().t_sent() << std::endl;
    }
}