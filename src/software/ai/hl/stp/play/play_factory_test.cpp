#include "software/ai/hl/stp/play/play_factory.h"

#include <gtest/gtest.h>

#include "software/util/typename/typename.h"

class PlayFactoryTest : public testing::Test
{
   protected:
    std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr =
        std::make_shared<TbotsProto::AiConfig>();
};

TEST_F(PlayFactoryTest, test_shoot_or_pass_play)
{
    TbotsProto::Play play_proto = TbotsProto::Play();
    play_proto.set_name(TbotsProto::PlayName::ShootOrPassPlay);
    std::unique_ptr<Play> play = createPlay(play_proto, ai_config_ptr);
    EXPECT_EQ(objectTypeName(*play), "ShootOrPassPlay");
}
