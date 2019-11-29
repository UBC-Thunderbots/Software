#include "software/sensor_fusion/refbox_data.h"

#include <gtest/gtest.h>

TEST(RefboxData, test_get_name_of_refbox_constants)
{
    for (int i = 0; i < static_cast<int>(RefboxGameState::REFBOX_GAME_STATE_COUNT); i++)
    {
        try
        {
            RefboxGameState state = static_cast<RefboxGameState>(i);
            name(state);
        }
        catch (std::invalid_argument &)
        {
            ADD_FAILURE() << "Unable to get a name for refbox gamestate " << i
                          << std::endl;
        }
        catch (std::exception &)
        {
            ADD_FAILURE() << "Unexpected exception thrown while trying to get the refbox"
                          << " gamestate name for " << i << std::endl;
        }
    }
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
