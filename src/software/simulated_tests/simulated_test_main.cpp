#include "simulated_test_main.h"

bool SimulatedTestMain::enable_visualizer = false;

int main(int argc, char **argv) {
	LoggerSingleton::initializeLogger();
	
	std::cout << "----Running new main----\n";
	
	::testing::GTEST_FLAG(output) = "xml:hello.xml";
	testing::InitGoogleTest(&argc, argv);

	//load command line arguments
	auto args =
		MutableDynamicParameters->getMutableSimulatedTestMainCommandLineArgs();
	bool help_requested = args->loadFromCommandLineArguments(argc, argv);
	
	std::cout << help_requested;
	
	for (int i = 0; i < argc; i++)
	{
		std::cout << "\n"<< i << ":" << argv[i] << std::endl;
	}

	SimulatedTestMain::enable_visualizer = args->enable_visualizer()->value();
	std::cout << "Current visualizer state: " << SimulatedTestMain::enable_visualizer;
	
	return RUN_ALL_TESTS();
}
