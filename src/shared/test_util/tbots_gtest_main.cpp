#include "shared/test_util/tbots_gtest_main.h"

#include <fenv.h>

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/logger/logger.h"

bool TbotsGtestMain::enable_visualizer  = false;
bool TbotsGtestMain::stop_ai_on_start   = false;
std::string TbotsGtestMain::logging_dir = "";
int TbotsGtestMain::speed_slow_down     = 1;


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    feenableexcept(FE_INVALID | FE_OVERFLOW);


    // load command line arguments
    auto args           = std::make_shared<TbotsGtestMainCommandLineArgs>();
    bool help_requested = args->loadFromCommandLineArguments(argc, argv);

    TbotsGtestMain::logging_dir = args->getLoggingDir()->value();

    if (!help_requested)
    {
        TbotsGtestMain::enable_visualizer = args->getEnableVisualizer()->value();
        if (TbotsGtestMain::enable_visualizer)
        {
            TbotsGtestMain::stop_ai_on_start = args->getStopAiOnStart()->value();
            TbotsGtestMain::speed_slow_down  = args->getSpeedSlowDown()->value();
        }
    }

    return RUN_ALL_TESTS();
}
