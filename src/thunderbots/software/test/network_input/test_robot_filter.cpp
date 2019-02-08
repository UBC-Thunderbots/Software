#include <gtest/gtest.h>

#include "network_input/filter/robot_filter.h"

// helpers for tests
namespace
{
    const int ROBOT_ID        = 42;
    const Point POSITION      = Point(6, 9);
    const Angle ORIENTATION   = Angle::zero();
    const double CONFIDENCE   = 100.0;
    const Timestamp TIMESTAMP = Timestamp::fromSeconds(10);

    // helper function to return populated struct with constants above
    SSLRobotData get_default_ssl_robot_data()
    {
        SSLRobotData data;
        data.id          = ROBOT_ID;
        data.position    = POSITION;
        data.orientation = ORIENTATION;
        data.confidence  = CONFIDENCE;
        data.timestamp   = TIMESTAMP;
        return data;
    }
}  // namespace

// test ideal flow, only one SSLRobotData object in the array
// this test only asserts that the data that should be changed is tested
TEST(RobotFilterTest, normal_flow_test)
{
    std::vector<SSLRobotData> test_data;

    // create ideal data struct
    SSLRobotData ideal_data = get_default_ssl_robot_data();

    // create ideal data vector
    test_data.push_back(ideal_data);

    // FUT
    RobotFilter filter_under_test(0);

    // get filtered data
    FilteredRobotData filtered_data;
    filtered_data = filter_under_test.getFilteredData(test_data);

    // these values should be equal, since only one packet was recieved
    // by the filter, it should not mess with the only data it recievied
    EXPECT_EQ(filtered_data.id, ideal_data.id);
    EXPECT_EQ(filtered_data.position, ideal_data.position);
    EXPECT_EQ(filtered_data.orientation, ideal_data.orientation);
    EXPECT_EQ(filtered_data.timestamp, ideal_data.timestamp);
    EXPECT_EQ(filtered_data.cached, false);
}

// test averaging from multiple entries, when the filter recieves multiple
// msgs with data, it should return the average position
TEST(RobotFilterTest, test_fuzzy_position)
{
    std::vector<SSLRobotData> test_data;

    // create fuzzy data struct
    SSLRobotData fuzzy_data = get_default_ssl_robot_data();

    // move the robot around the default position
    for (int x_disp = 0; x_disp <= 2; ++x_disp)
    {
        for (int y_disp = 0; y_disp <= 2; ++y_disp)
        {
            // add fuzzyness to position and angle, append it to test_data
            fuzzy_data.position.set(POSITION.x() + x_disp, POSITION.y() + y_disp);
            test_data.push_back(fuzzy_data);
        }
    }

    // FUT
    RobotFilter filter_under_test(0);

    // get filtered data
    FilteredRobotData filtered_data;
    filtered_data = filter_under_test.getFilteredData(test_data);

    // these values should be untouched
    EXPECT_EQ(filtered_data.id, fuzzy_data.id);
    EXPECT_EQ(filtered_data.orientation, fuzzy_data.orientation);
    EXPECT_EQ(filtered_data.timestamp, fuzzy_data.timestamp);
    EXPECT_EQ(filtered_data.cached, false);

    // should expect to see 7, 10 as ((avg of 6+0, 6+1, 6+2), (avg of 9+1, 9+2, 9+3))
    EXPECT_EQ(filtered_data.position, Point(7, 10));
}

// test with no data, to make sure the cached data is returned so we don't
// temporarily lose the robots
TEST(RobotFilterTest, test_fuzzy_angle)
{
    std::vector<SSLRobotData> test_data;

    // create fuzzy_data struct
    SSLRobotData fuzzy_data = get_default_ssl_robot_data();

    // create ideal data vector
    test_data.push_back(fuzzy_data);
    fuzzy_data.orientation = Angle::full();
    test_data.push_back(fuzzy_data);

    // FUT
    RobotFilter filter_under_test(0);
    FilteredRobotData filtered_data;
    filtered_data = filter_under_test.getFilteredData(test_data);

    // these values should be untouched
    EXPECT_EQ(filtered_data.id, fuzzy_data.id);
    EXPECT_EQ(filtered_data.position, fuzzy_data.position);
    EXPECT_EQ(filtered_data.timestamp, fuzzy_data.timestamp);
    EXPECT_EQ(filtered_data.cached, false);

    // expect to see 180 (avg of 0 and 360)
    EXPECT_EQ(filtered_data.orientation, Angle::half());
}

// this test is to make sure that when no data is recieved, that the cached
// value is returned
TEST(RobotFilterTest, test_no_data_recieved)
{
    // initialize filter cache
    std::vector<SSLRobotData> test_data;
    SSLRobotData some_data = get_default_ssl_robot_data();
    test_data.push_back(some_data);

    // FUT setup
    RobotFilter filter_under_test(0);

    // get filtered data
    FilteredRobotData filtered_data;
    filtered_data = filter_under_test.getFilteredData(test_data);

    // sanity check
    EXPECT_EQ(filtered_data.id, some_data.id);
    EXPECT_EQ(filtered_data.position, some_data.position);
    EXPECT_EQ(filtered_data.orientation, some_data.orientation);
    EXPECT_EQ(filtered_data.timestamp, some_data.timestamp);
    EXPECT_EQ(filtered_data.cached, false);

    // no data test
    test_data.clear();

    // get filtered data
    filtered_data = filter_under_test.getFilteredData(test_data);

    // assert all the previous values were not touched when moved into the cache
    EXPECT_EQ(filtered_data.id, some_data.id);
    EXPECT_EQ(filtered_data.position, some_data.position);
    EXPECT_EQ(filtered_data.orientation, some_data.orientation);
    EXPECT_EQ(filtered_data.timestamp, some_data.timestamp);

    // assert that the cache flag is set
    EXPECT_EQ(filtered_data.cached, true);
}

// test sending the same data with the same timestamp twice and make sure that
// the cached data is returned as that data has already been processed and stored
// to avoid dividing by zero
TEST(RobotFilterTest, test_same_timestamp)
{
    // initialize filter cache
    std::vector<SSLRobotData> test_data;
    SSLRobotData some_data = get_default_ssl_robot_data();
    test_data.push_back(some_data);

    // FUT setup
    RobotFilter filter_under_test(0);

    // get filtered  data
    FilteredRobotData filtered_data;
    filtered_data = filter_under_test.getFilteredData(test_data);

    // sanity check
    EXPECT_EQ(filtered_data.id, some_data.id);
    EXPECT_EQ(filtered_data.position, some_data.position);
    EXPECT_EQ(filtered_data.orientation, some_data.orientation);
    EXPECT_EQ(filtered_data.timestamp, some_data.timestamp);
    EXPECT_EQ(filtered_data.cached, false);

    // get filtered data from same set of test data
    filtered_data = filter_under_test.getFilteredData(test_data);

    // assert all the previous values were not touched when moved into the cache
    EXPECT_EQ(filtered_data.id, some_data.id);
    EXPECT_EQ(filtered_data.position, some_data.position);
    EXPECT_EQ(filtered_data.orientation, some_data.orientation);
    EXPECT_EQ(filtered_data.timestamp, some_data.timestamp);

    // assert that the cache flag is set
    EXPECT_EQ(filtered_data.cached, true);
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
