#include <gtest/gtest.h>

#include "simulator2d.hpp"
#include "integrator.hpp"

// Simulator2d
TEST(Simulator2d, ConstructorTest) {
    Simulator2d simulator(1, 1000, nullptr);
}

TEST(Simulator2d, ActiveMethodTest) {
    // TODO implement after implementing addObject, delObject
}