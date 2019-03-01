#include <gtest/gtest.h>
#include "ai/hl/stp/play/play_factory.h"

TEST(PlayTest, test) {
   auto p = PlayFactory::getRegisteredPlayNames();
   for(auto x : p) {
      std::cout << x << std::endl;
   }
}




