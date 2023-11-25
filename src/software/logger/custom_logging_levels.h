#pragma once


// See https://github.com/KjellKod/g3log/blob/master/API.markdown#custom-logging-levels
// for adding custom log levels

// A custom level used to specify messages that contain robot statuses, including various
// diagnostic information like battery voltage and wheel encoder errors. These messages
// are separate because they are shown separately in the FullSystemGUI.
// over radio
const LEVELS ROBOT_STATUS{INFO.value + 1, {"ROBOT_STATUS"}};
const LEVELS CSV{INFO.value + 2, {"CSV"}};
const LEVELS VISUALIZE{INFO.value + 3, {"VISUALIZE"}};
const LEVELS PLOTJUGGLER{INFO.value + 4, {"PLOTJUGGLER"}};
