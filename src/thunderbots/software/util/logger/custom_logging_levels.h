#pragma once

#include <g3log/loglevels.hpp>

// See https://github.com/KjellKod/g3log/blob/master/API.markdown#custom-logging-levels
// for adding custom log levels

// A custom level used to specify messages that contain robot statuses, including various
// diagnostic information like battery voltage and wheel encoder errors. These messages
// are separate because they are shown separately in the Visualizer.
// over radio
const LEVELS ROBOT_STATUS{INFO.value + 1, {"ROBOT_STATUS"}};
