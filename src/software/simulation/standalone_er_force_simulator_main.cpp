#include "software/logger/logger.h"
#include "software/simulation/standalone_er_force_simulator.h"

int main()
{
    LoggerSingleton::initializeLogger("/tmp/logs");

    // TODO pass this in as an arg
    auto sim = StandaloneErForceSimulator("/tmp/tbots");

    // This blocks forever without using the CPU
    std::promise<void>().get_future().wait();
}
