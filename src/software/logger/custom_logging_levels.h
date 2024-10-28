#pragma once


// See https://github.com/KjellKod/g3log/blob/master/docs/API_custom_formatting.md
// for adding custom log levels

// A custom level used to specify messages that contain robot statuses, including various
// diagnostic information like battery voltage and wheel encoder errors. These messages
// are separate because they are shown separately in the FullSystemGUI.
// over radio
const LEVELS CSV{INFO.value + 1, {"CSV"}};
const LEVELS VISUALIZE{INFO.value + 2, {"VISUALIZE"}};
const LEVELS PLOTJUGGLER{INFO.value + 3, {"PLOTJUGGLER"}};
