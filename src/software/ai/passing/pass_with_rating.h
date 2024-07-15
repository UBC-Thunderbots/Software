#pragma once

#include "software/ai/passing/base_pass.h"

struct PassWithRating
{
    BasePass pass;
    double rating;
};

bool operator==(const PassWithRating &lhs, const struct PassWithRating &rhs);
