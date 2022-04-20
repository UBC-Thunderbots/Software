#pragma once

#include <gtest/gtest.h>

struct TbotsGtestMain
{
    // Controls whether the visualizer will be enabled during the tests (if implemented)
    // if false, visualizer does not run during tests if true, running tests are displayed
    // on the visualizer
    static bool enable_visualizer;

    // Controls whether the test should run in real time
    static bool run_sim_in_realtime;

    // Controls whether the AI will be stopped when the test starts only if
    // enable_visualizer is true and the test uses AI
    static bool stop_ai_on_start;

    // Directory to send g3log logger logs
    static std::string runtime_dir;


    // Controls the speed of the simulated test. Values in the range [0.1,1) are the slow
    // down factor i.e 0.1 test_speed will play test 10X slower. Values in the range (1,
    // 10] will play test faster. Default value is 1.
    static double test_speed;
    static bool help;
};
