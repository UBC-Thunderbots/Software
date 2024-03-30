#pragma once

#include "software/ai/passing/pass_with_rating.h"
#include "software/world/world.h"

class BasePassGenerator
{
    public:
        virtual ~BasePassGenerator() {}
        virtual PassWithRating getBestPass(const World& world) = 0;
};