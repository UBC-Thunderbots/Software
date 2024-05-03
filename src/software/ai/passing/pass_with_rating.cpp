#include "software/ai/passing/pass_with_rating.h"

bool operator==(const PassWithRating &lhs, const PassWithRating &rhs)
{
    return lhs.rating == rhs.rating &&
           lhs.pass.receiverPoint() == rhs.pass.receiverPoint() &&
           lhs.pass.speed() == rhs.pass.speed();
}

bool operator<(const PassWithRating &lhs, const struct PassWithRating &rhs)
{
    return lhs.rating < rhs.rating;
}
