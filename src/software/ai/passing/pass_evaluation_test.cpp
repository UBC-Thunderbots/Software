#include "software/ai/passing/pass_evaluation.h"

#include <gtest/gtest.h>

#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/ai/passing/pass.h"
#include "software/geom/algorithms/contains.h"
#include "software/world/field.h"

TEST(PassEvaluation, best_pass_over_entire_field)
{
    auto field          = Field::createSSLDivisionBField();
    auto pitch_division = std::make_shared<const EighteenZonePitchDivision>(field);

    std::vector<PassWithRating> passes_with_rating;

    for (unsigned zone_id = 1; zone_id <= pitch_division->getTotalNumberOfZones(); zone_id++)
    {
        passes_with_rating.emplace_back(PassWithRating{std::move(Pass(Point(0, 0), 0)), 0});
    }

    // change zone 8 to have a score of 0.8, which should be the highest
    passes_with_rating[8].rating = 0.8;

    auto pass_eval = PassEvaluation(pitch_division, passes_with_rating, Timestamp::fromSeconds(10));

    ASSERT_EQ(pass_eval.getBestPassOnField(), passes_with_rating[8]);
}


TEST(PassEvaluation, best_pass_in_zones)
{
    auto field          = Field::createSSLDivisionBField();
    auto pitch_division = std::make_shared<const EighteenZonePitchDivision>(field);

    std::vector<PassWithRating> passes_with_rating;

    for (unsigned zone_id = 1; zone_id <= pitch_division->getTotalNumberOfZones(); zone_id++)
    {
        // create passes with arbitrary ratings, but w/ zone 18 having the highest
        passes_with_rating.emplace_back(PassWithRating{std::move(Pass(Point(0, 0), 0)), 0.05 * zone_id});
    }

    auto pass_eval = PassEvaluation(pitch_division, passes_with_rating, Timestamp::fromSeconds(10));

    // we expect the zone with the higher id to have the higher score
    EXPECT_EQ(pass_eval.getBestPassInZones({10, 11, 12}), passes_with_rating[12 - 1]);
    EXPECT_EQ(pass_eval.getBestPassInZones({15, 1, 3}), passes_with_rating[15 - 1]);

    // sanity check that best pass in zones and zone return the same thing when querried w/ 1 zone
    EXPECT_EQ(pass_eval.getBestPassInZones({10}), passes_with_rating[10 - 1]);
    EXPECT_EQ(pass_eval.getBestPassInZones({2}), passes_with_rating[2 - 1]);
    EXPECT_EQ(pass_eval.getBestPassInZones({5}), passes_with_rating[5 - 1]);

    // make sure the evaluation complains when the zone is out of bounds, or empty
    EXPECT_THROW(pass_eval.getBestPassInZones({20}), std::invalid_argument);
    EXPECT_THROW(pass_eval.getBestPassInZones({}), std::invalid_argument);
}

TEST(PassEvaluation, get_pitch_division_and_timestamp)
{
    auto field          = Field::createSSLDivisionBField();
    auto pitch_division = std::make_shared<const EighteenZonePitchDivision>(field);

    std::vector<PassWithRating> passes_with_rating;

    for (unsigned zone_id = 1; zone_id <= pitch_division->getTotalNumberOfZones(); zone_id++)
    {
        passes_with_rating.emplace_back(PassWithRating{std::move(Pass(Point(0, 0), 0)), 0});
    }

    auto pass_eval = PassEvaluation(pitch_division, passes_with_rating, Timestamp::fromSeconds(10));

    EXPECT_EQ(pass_eval.getFieldPitchDivsion(), pitch_division);
    EXPECT_EQ(pass_eval.getEvaluationTime(), Timestamp::fromSeconds(10));

}

bool operator==(const PassWithRating &lhs, const PassWithRating &rhs)
{
    return lhs.rating == rhs.rating &&
           lhs.pass.receiverPoint() == rhs.pass.receiverPoint() &&
           lhs.pass.speed() == rhs.pass.speed();
}
