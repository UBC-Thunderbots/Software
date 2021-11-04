#include "software/multithreading/subject.hpp"

#include <gtest/gtest.h>

#include <thread>

#include "software/multithreading/observer.hpp"

class MockObserver : public Observer<int>
{
   public:
    std::optional<int> getMostRecentValueFromBufferWrapper()
    {
        return popMostRecentlyReceivedValue(Duration::fromSeconds(5));
    }
};

class TestSubject : public Subject<int>
{
   public:
    void sendValue(int i)
    {
        sendValueToObservers(i);
    }
};

TEST(Subject, sendValueToObservers)
{
    TestSubject test_subject;
    auto mock_observer = std::make_shared<MockObserver>();

    test_subject.registerObserver(mock_observer);

    test_subject.sendValue(37);

    std::optional<int> result = mock_observer->getMostRecentValueFromBufferWrapper();
    ASSERT_TRUE(result);
    EXPECT_EQ(37, *result);
}
