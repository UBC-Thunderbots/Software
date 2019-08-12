#include "multithreading/subject.h"
#include "multithreading/observer.h"

#include <gtest/gtest.h>

#include <thread>

class MockObserver : public Observer<int> {
public:
    int getMostRecentValueFromBufferWrapper(){
        return getMostRecentValueFromBuffer();
    }
};

class TestSubject : public Subject<int> {
public:
    void sendValue(int i){
        sendValueToObservers(i);
    }
};

TEST(Subject, sendValueToObservers){
    TestSubject test_subject;
    auto mock_observer = std::make_shared<MockObserver>();

    test_subject.registerObserver(mock_observer);

    test_subject.sendValue(37);

    EXPECT_EQ(37, mock_observer->getMostRecentValueFromBufferWrapper());
}

