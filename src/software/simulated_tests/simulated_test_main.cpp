#include <iostream>
#include <gtest/gtest.h>
#include "software/logger/logger.h"


int main(int argc, char **argv) {
  LoggerSingleton::initializeLogger();
  
  std::cout << "----Running new main----\n";
  ::testing::GTEST_FLAG(output) = "xml:hello.xml";
  testing::InitGoogleTest(&argc, argv);

  for (int i = 0; i < argc; i++)
  {
	std::cout << i << ":" << argv[i] << std::endl;
  }
  
  return RUN_ALL_TESTS();
}
