#include "software/ai/passing/pass_evaluation.hpp"

#include <gtest/gtest.h>

#include <unordered_map>

#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/ai/passing/pass.h"
#include "software/geom/algorithms/contains.h"
#include "software/world/field.h"

class PassEvaluationTest : public ::testing::Test
{
   protected:
    PassEvaluationTest() {}

    void SetUp() override
    {
        passing_config = std::make_shared<PassingConfig>();
        passing_config->getMutableMinPassSpeedMPerS()->setValue(3.5);
        passing_config->getMutableMaxPassSpeedMPerS()->setValue(5.5);
    }

    std::shared_ptr<PassingConfig> passing_config;
};

TEST_F(PassEvaluationTest, best_pass_over_entire_field)
{
    auto field          = Field::createSSLDivisionBField();
    auto pitch_division = std::make_shared<const EighteenZonePitchDivision>(field);

    std::unordered_map<EighteenZoneId, PassWithRating> passes_with_rating;

    for (EighteenZoneId zone_id : allValuesEighteenZoneId())
    {
        passes_with_rating.emplace(
            zone_id, PassWithRating{std::move(Pass(Point(0, 0), Point(0, 0), 0)), 0});
    }

    // change zone 8 to have a score of 0.8, which should be the highest
    passes_with_rating.emplace(
        EighteenZoneId::ZONE_8,
        PassWithRating{std::move(Pass(Point(0, 0), Point(0, 0), 0)), 0.8});

    auto pass_eval = PassEvaluation<EighteenZoneId>(
        pitch_division, passes_with_rating, passing_config, Timestamp::fromSeconds(10));

    ASSERT_EQ(pass_eval.getBestPassOnField(),
              passes_with_rating.at(EighteenZoneId::ZONE_8));
}


TEST_F(PassEvaluationTest, best_pass_in_zones)
{
    auto field          = Field::createSSLDivisionBField();
    auto pitch_division = std::make_shared<const EighteenZonePitchDivision>(field);

    std::unordered_map<EighteenZoneId, PassWithRating> passes_with_rating;

    for (EighteenZoneId zone_id : allValuesEighteenZoneId())
    {
        passes_with_rating.emplace(
            zone_id, PassWithRating{std::move(Pass(Point(0, 0), Point(0, 0), 0)),
                                    0.05 * static_cast<unsigned>(zone_id)});
    }

    auto pass_eval = PassEvaluation<EighteenZoneId>(
        pitch_division, passes_with_rating, passing_config, Timestamp::fromSeconds(10));

    // we expect the zone with the higher id to have the higher score
    EXPECT_EQ(
        pass_eval.getBestPassInZones(
            {EighteenZoneId::ZONE_12, EighteenZoneId::ZONE_10, EighteenZoneId::ZONE_3}),
        passes_with_rating.at(EighteenZoneId::ZONE_12));
    EXPECT_EQ(
        pass_eval.getBestPassInZones(
            {EighteenZoneId::ZONE_15, EighteenZoneId::ZONE_12, EighteenZoneId::ZONE_13}),
        passes_with_rating.at(EighteenZoneId::ZONE_15));

    // sanity check that best pass in zones and zone return the same thing when querried
    // w/ 1 zone
    EXPECT_EQ(pass_eval.getBestPassInZones({EighteenZoneId::ZONE_10}),
              passes_with_rating.at(EighteenZoneId::ZONE_10));
    EXPECT_EQ(pass_eval.getBestPassInZones({EighteenZoneId::ZONE_12}),
              passes_with_rating.at(EighteenZoneId::ZONE_12));
    EXPECT_EQ(pass_eval.getBestPassInZones({EighteenZoneId::ZONE_15}),
              passes_with_rating.at(EighteenZoneId::ZONE_15));
}

TEST_F(PassEvaluationTest, get_pitch_division_and_timestamp)
{
    auto field          = Field::createSSLDivisionBField();
    auto pitch_division = std::make_shared<const EighteenZonePitchDivision>(field);

    std::unordered_map<EighteenZoneId, PassWithRating> passes_with_rating;

    for (EighteenZoneId zone_id : allValuesEighteenZoneId())
    {
        passes_with_rating.emplace(
            zone_id, PassWithRating{std::move(Pass(Point(0, 0), Point(0, 0), 0)), 0});
    }

    auto pass_eval = PassEvaluation<EighteenZoneId>(
        pitch_division, passes_with_rating, passing_config, Timestamp::fromSeconds(10));

    EXPECT_EQ(pass_eval.getFieldPitchDivsion(), pitch_division);
    EXPECT_EQ(pass_eval.getEvaluationTime(), Timestamp::fromSeconds(10));
}
