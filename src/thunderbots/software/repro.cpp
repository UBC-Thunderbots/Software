#include "ai/passing/pass.h"
#include <vector>
#include <algorithm>

using namespace AI::Passing;

int main() {
    std::vector<Pass> v;
    int num_paths_to_gen = 500;
    Pass p(Point(0,0), Point(0,0), 0, Timestamp::fromSeconds(0));
    //for (int i = 0; i < 5; i++){
    std::cout << num_paths_to_gen << std::endl;
    for (int i = 0; i < num_paths_to_gen; i++){
        v.emplace_back(p);
    }

    std::sort(v.begin(), v.end(), [](Pass p1, Pass p2) { return p1.passSpeed() < p2.passSpeed(); });
}