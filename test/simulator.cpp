#include <gtest/gtest.h>

#include "simulator2d.hpp"
#include "integrator.hpp"


#include <iostream>
// Simulator2d
TEST(Simulator2d, ConstructorTest) {
    Simulator2d sim(1, 1000, nullptr); // TODO nullptr with actual integrator
}

TEST(Simulator2d, AddDelGetMethodTest) {
    Simulator2d sim(1, 1000, nullptr); // TODO nullptr with actual integrator

    Rigidbody rb = sim.addObject(100, 10, Eigen::Vector2d(17, 12), Eigen::Vector2d(2, -1));
    RigidbodyData rbdata = sim.getObject(rb);

    EXPECT_EQ(rbdata.exists(), true);
    EXPECT_EQ(*rbdata.m, 100);
    EXPECT_EQ(*rbdata.r, 10);
    EXPECT_EQ(*rbdata.pos[0], 17);
    EXPECT_EQ(*rbdata.pos[1], 12);
    EXPECT_EQ(*rbdata.v[0], 2);
    EXPECT_EQ(*rbdata.v[1], -1);
    EXPECT_EQ(*rbdata.a[0], 0);
    EXPECT_EQ(*rbdata.a[1], 0);

    sim.delObject(rb);

    EXPECT_EQ(rbdata.exists(), false);

    sim.addObject(100, 10, Eigen::Vector2d(17, 12), Eigen::Vector2d(2, -1));
    sim.addObject(-100, 10, Eigen::Vector2d(17, 12), Eigen::Vector2d(2, -1));
    // TODO write tests for these
}