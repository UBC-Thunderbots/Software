#include "software/multithreading/observer_subject_adapter.hpp"

#include <gtest/gtest.h>

#include "software/multithreading/observer.hpp"
#include "software/multithreading/subject.hpp"

class TestObserver : public Observer<std::string>
{
   public:
    void receiveValue(std::string val) override
    {
        received_values.emplace_back(std::move(val));
    }
    std::vector<std::string> received_values;
};

class TestSubject : public Subject<int>
{
   public:
    void sendValue(int val)
    {
        sendValueToObservers(val);
    }
};

TEST(ObserverSubjectAdapterTest, test_observer_to_subject_adapter_conversion)
{
    std::vector<int> input_ints               = {0, 1, 2};
    std::vector<std::string> expected_strings = {"0", "1", "2"};

    auto int_to_string_adapter =
        std::make_shared<ObserverSubjectAdapter<int, std::string>>(
            [](const auto& num) { return std::to_string(num); });

    TestSubject subject;
    auto test_observer_ptr = std::make_shared<TestObserver>();

    subject.registerObserver(int_to_string_adapter);
    int_to_string_adapter->registerObserver(test_observer_ptr);

    for (const auto input : input_ints)
    {
        subject.sendValue(input);
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    EXPECT_EQ(expected_strings, test_observer_ptr->received_values);
}
