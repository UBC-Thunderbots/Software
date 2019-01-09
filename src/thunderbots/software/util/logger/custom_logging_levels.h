#pragma once

#include <g3log/loglevels.hpp>

// See https://github.com/KjellKod/g3log/blob/master/API.markdown#custom-logging-levels
// for adding custom log levels

// A custom level used to specify messages that should be displayed in the Annunciator
// in the Visualizer. These are typically diagnostic messages received from the robots
// over radio
const LEVELS ANNUNCIATOR{INFO.value + 1, {"ANNUNCIATOR"}};
