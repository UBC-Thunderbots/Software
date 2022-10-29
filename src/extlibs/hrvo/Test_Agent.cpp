#include "Test_Agent.h"

Test_Agent::Test_Agent(double x, double y) {
    point = Point(x, y);
}

Point Test_Agent::position() const {
    return point;
}