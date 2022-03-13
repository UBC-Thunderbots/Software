#include "software/ai/hl/stp/play/play_factory.h"

#include <gtest/gtest.h>

#include "software/util/typename/typename.h"

class PlayFactoryTest : public testing::Test
{
   protected:
    std::shared_ptr<const AiConfig> ai_config =
        std::make_shared<const ThunderbotsConfig>()->getAiConfig();
};

TEST_F(PlayFactoryTest, test_shoot_or_pass_play)
{
    TbotsProto::Play play_proto = TbotsProto::Play::ShootOrPassPlay;
    std::unique_ptr<Play> play  = createPlay(play_proto, ai_config);
    EXPECT_EQ(objectTypeName(*play), "ShootOrPassPlay");
}
