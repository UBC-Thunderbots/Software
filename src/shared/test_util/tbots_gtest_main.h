#pragma once

#include <gtest/gtest.h>

struct TbotsGtestMain
{
    // Controls whether the visualizer will be enabled during the tests (if implemented)
    // if false, visualizer does not run during tests if true, running tests are displayed
    // on the visualizer
    static bool enable_visualizer;

    // Controls whether the AI will be stopped when the test starts only if
    // enable_visualizer is true and the test uses AI
    static bool stop_ai_on_start;

    // Directory to send g3log logger logs
    static std::string logging_dir;
};
