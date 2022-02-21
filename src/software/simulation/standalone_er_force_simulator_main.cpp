#include "software/simulation/standalone_er_force_simulator.h"

int main()
{
    auto sim = StandaloneErForceSimulator();

    // This blocks forever without using the CPU
    std::promise<void>().get_future().wait();
}
