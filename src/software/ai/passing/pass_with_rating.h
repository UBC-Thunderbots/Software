#pragma once

#include "software/ai/passing/pass.h"

struct PassWithRating
{
    Pass pass;
    double rating;
};

bool operator==(const PassWithRating &lhs, const struct PassWithRating &rhs);

bool operator<(const PassWithRating &lhs, const struct PassWithRating &rhs);