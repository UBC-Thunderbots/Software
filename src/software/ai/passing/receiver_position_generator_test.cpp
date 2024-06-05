#include "software/ai/passing/receiver_position_generator.hpp"

#include <gtest/gtest.h>

#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/test_util/test_util.h"

class ReceiverPositionGeneratorTest : public testing::Test
{
   public:
    ReceiverPositionGeneratorTest()
        : receiver_position_generator(ReceiverPositionGenerator<EighteenZoneId>(
              std::make_shared<const EighteenZonePitchDivision>(
                  Field::createSSLDivisionBField()),
              TbotsProto::PassingConfig())),
          world(::TestUtil::createBlankTestingWorld())
    {
    }

   protected:
    ReceiverPositionGenerator<EighteenZoneId> receiver_position_generator;
    // Empty world
    WorldPtr world;
};


TEST_F(ReceiverPositionGeneratorTest, getBestPass_2_friendlies) {}
