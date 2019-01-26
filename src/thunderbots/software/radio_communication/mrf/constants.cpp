#include "constants.h"

const char* const MRF::ERROR_LT_MESSAGES[] = {
    u8"charge timeout",
    u8"wheel 0 motor hot",
    u8"wheel 1 motor hot",
    u8"wheel 2 motor hot",
    u8"wheel 3 motor hot",
    u8"dribbler motor hot",
    u8"wheel 0 encoder not commutating",
    u8"wheel 1 encoder not commutating",
    u8"wheel 2 encoder not commutating",
    u8"wheel 3 encoder not commutating",
    u8"wheel 0 Hall sensor stuck low",
    u8"wheel 1 Hall sensor stuck low",
    u8"wheel 2 Hall sensor stuck low",
    u8"wheel 3 Hall sensor stuck low",
    u8"dribbler Hall sensor stuck low",
    u8"wheel 0 Hall sensor stuck high",
    u8"wheel 1 Hall sensor stuck high",
    u8"wheel 2 Hall sensor stuck high",
    u8"wheel 3 Hall sensor stuck high",
    u8"dribbler Hall sensor stuck high",
};
static_assert(sizeof(MRF::ERROR_LT_MESSAGES) / sizeof(*MRF::ERROR_LT_MESSAGES) ==
                  MRF::ERROR_LT_COUNT,
              "Wrong number of level-triggered error messages.");

const char* const MRF::ERROR_ET_MESSAGES[] = {
    u8"ICB CRC error",
    u8"receive frame check sequence failure",
    u8"crashed (core dumped)",
    u8"crashed (no core dump)",
};
static_assert(sizeof(MRF::ERROR_ET_MESSAGES) / sizeof(*MRF::ERROR_ET_MESSAGES) ==
                  MRF::ERROR_ET_COUNT,
              "Wrong number of edge-triggered error messages.");
