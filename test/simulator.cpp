#include <gtest/gtest.h>

#include "simulator2d.hpp"
#include "integrator.hpp"


#include <iostream>
// Simulator2d
TEST(Simulator2d, ConstructorTest) {
    Simulator2d sim(1, 1000, new EulerIntegrator());
}

TEST(Simulator2d, AddDelGetMethodTest) {
    Simulator2d sim(1, 1000, new EulerIntegrator());

    Rigidbody rb1 = sim.addObject(100, 10, Eigen::Vector2d(17, 12), Eigen::Vector2d(2, -1));
    EXPECT_EQ(sim.rb_exists(rb1), true);
    EXPECT_EQ(sim.rb_m(rb1), 100);
    EXPECT_EQ(sim.rb_r(rb1), 10);
    EXPECT_EQ(sim.rb_pos(rb1)(0), 17);
    EXPECT_EQ(sim.rb_pos(rb1)(1), 12);
    EXPECT_EQ(sim.rb_v(rb1)(0), 2);
    EXPECT_EQ(sim.rb_v(rb1)(1), -1);
    EXPECT_EQ(sim.rb_a(rb1)(0), 0);
    EXPECT_EQ(sim.rb_a(rb1)(1), 0);
    sim.delObject(rb1);
    EXPECT_EQ(sim.rb_exists(rb1), false);

    // Check pack-ifier system working
    Rigidbody rb2 = sim.addObject(100, 10, Eigen::Vector2d(17, 12), Eigen::Vector2d(2, -1));
    Rigidbody rb3 = sim.addObject(-100, 7, Eigen::Vector2d(1, 2), Eigen::Vector2d(3, 4));
    EXPECT_EQ(sim.rb_exists(rb2), true);
    EXPECT_EQ(sim.rb_exists(rb3), true);
    EXPECT_EQ(sim.rb_m(rb3), -100);
    EXPECT_EQ(sim.rb_r(rb3), 7);
    EXPECT_EQ(sim.rb_pos(rb3)(0), 1);
    EXPECT_EQ(sim.rb_pos(rb3)(1), 2);
    EXPECT_EQ(sim.rb_v(rb3)(0), 3);
    EXPECT_EQ(sim.rb_v(rb3)(1), 4);
    EXPECT_EQ(sim.rb_a(rb3)(0), 0);
    EXPECT_EQ(sim.rb_a(rb3)(1), 0);
    sim.delObject(rb2);
    EXPECT_EQ(sim.rb_exists(rb2), false);
    EXPECT_EQ(sim.rb_exists(rb3), true);
    EXPECT_EQ(sim.rb_m(rb3), -100);
    EXPECT_EQ(sim.rb_r(rb3), 7);
    EXPECT_EQ(sim.rb_pos(rb3)(0), 1);
    EXPECT_EQ(sim.rb_pos(rb3)(1), 2);
    EXPECT_EQ(sim.rb_v(rb3)(0), 3);
    EXPECT_EQ(sim.rb_v(rb3)(1), 4);
    EXPECT_EQ(sim.rb_a(rb3)(0), 0);
    EXPECT_EQ(sim.rb_a(rb3)(1), 0);
}

TEST(Simulator2d, EulerIntegratorTest) {
    Simulator2d sim(0.1, 1000, new EulerIntegrator());
    Rigidbody rb = sim.addObject(10, 10, Eigen::Vector2d(10, 20), Eigen::Vector2d(-1, 2));

    sim.step();
    EXPECT_EQ(sim.rb_v(rb)(0), -1);
    EXPECT_EQ(sim.rb_v(rb)(1), 2);
    EXPECT_EQ(sim.rb_pos(rb)(0), 9.9);
    EXPECT_EQ(sim.rb_pos(rb)(1), 20.2);
}