#include "shared/test_util/tbots_gtest_main.h"

#include <fenv.h>

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/logger/logger.h"

bool TbotsGtestMain::enable_visualizer   = false;
bool TbotsGtestMain::run_sim_in_realtime = false;
bool TbotsGtestMain::stop_ai_on_start    = false;
std::string TbotsGtestMain::logging_dir  = "";
double TbotsGtestMain::test_speed        = 1.0;


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    feenableexcept(FE_INVALID | FE_OVERFLOW);


    // load command line arguments
    auto args           = std::make_shared<TbotsGtestMainCommandLineArgs>();
    bool help_requested = args->loadFromCommandLineArguments(argc, argv);

    TbotsGtestMain::logging_dir = args->getRuntimeDir()->value();

    if (!help_requested)
    {
        TbotsGtestMain::enable_visualizer   = args->getEnableVisualizer()->value();
        TbotsGtestMain::run_sim_in_realtime = args->getRunSimInRealtime()->value();

        if (TbotsGtestMain::enable_visualizer || TbotsGtestMain::run_sim_in_realtime)
        {
            TbotsGtestMain::stop_ai_on_start = args->getStopAiOnStart()->value();
            TbotsGtestMain::test_speed       = args->getTestSpeed()->value();
            // disable floating point errors when using visualizer due to potential
            // floating point errors in QT
            fedisableexcept(FE_INVALID | FE_OVERFLOW);
        }
    }

    return RUN_ALL_TESTS();
}
