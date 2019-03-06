#include "constants.h"

const char* const MRF::ERROR_LT_MESSAGES[] = {
    "charge timeout",
    "wheel 0 motor hot",
    "wheel 1 motor hot",
    "wheel 2 motor hot",
    "wheel 3 motor hot",
    "dribbler motor hot",
    "wheel 0 encoder not commutating",
    "wheel 1 encoder not commutating",
    "wheel 2 encoder not commutating",
    "wheel 3 encoder not commutating",
    "wheel 0 Hall sensor stuck low",
    "wheel 1 Hall sensor stuck low",
    "wheel 2 Hall sensor stuck low",
    "wheel 3 Hall sensor stuck low",
    "dribbler Hall sensor stuck low",
    "wheel 0 Hall sensor stuck high",
    "wheel 1 Hall sensor stuck high",
    "wheel 2 Hall sensor stuck high",
    "wheel 3 Hall sensor stuck high",
    "dribbler Hall sensor stuck high",
};

// Assert that there are the correct number of level-triggered messages in the array of
// chars.
static_assert(sizeof(MRF::ERROR_LT_MESSAGES) / sizeof(*MRF::ERROR_LT_MESSAGES) ==
                  MRF::ERROR_LT_COUNT,
              "Wrong number of level-triggered error messages.");

const char* const MRF::ERROR_ET_MESSAGES[] = {
    "ICB CRC error",
    "receive frame check sequence failure",
    "crashed (core dumped)",
    "crashed (no core dump)",
};

// Assert that there are the correct number of edge-triggered messages in the array of
// chars.
static_assert(sizeof(MRF::ERROR_ET_MESSAGES) / sizeof(*MRF::ERROR_ET_MESSAGES) ==
                  MRF::ERROR_ET_COUNT,
              "Wrong number of edge-triggered error messages.");
