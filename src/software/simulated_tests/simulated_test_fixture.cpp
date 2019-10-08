#include <gtest/gtest.h>

#include "software/ai/ai_wrapper.h"
#include "software/backend/simulator_backend.h"
#include "software/gui/visualizer_wrapper.h"
#include "software/typedefs.h"
#include "software/util/logger/init.h"

class SimulatedTest : public ::testing::Test
{
   public:
    void enableVisualizer()
    {
        // TODO: how to do this without argc and argv? (#768)

        //        visualizer = std::make_shared<VisualizerWrapper>(argc, argv);
        //
        //        backend->Subject<World>::registerObserver(visualizer);
        //        ai->Subject<AIDrawFunction>::registerObserver(visualizer);
        //        ai->Subject<PlayInfo>::registerObserver(visualizer);
        //        backend->Subject<RobotStatus>::registerObserver(visualizer);
    }

   protected:
    static void SetUpTestSuite()
    {
        Util::Logger::LoggerSingleton::initializeLogger();
    }

    void SetUp() override
    {
        backend = std::make_shared<SimulatorBackend>();
        ai      = std::make_shared<AIWrapper>();

        backend->Subject<World>::registerObserver(ai);
        ai->Subject<ConstPrimitiveVectorPtr>::registerObserver(backend);
    }

    std::shared_ptr<SimulatorBackend> backend;
    std::shared_ptr<AIWrapper> ai;
    std::shared_ptr<VisualizerWrapper> visualizer;
};

TEST_F(SimulatedTest, sample_integration_test)
{
    // TODO: Implement this (#768)
}
