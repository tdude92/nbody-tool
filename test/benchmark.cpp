#include <iostream>
#include <chrono>
#include <string>
#include <cstdlib>
#include <Eigen>
#include "nbodytool.hpp"

void benchmark(Simulator2d& sim, int iters, const std::string& name) {
    std::cout << "Benchmarking " << name                  << std::endl
              << "--------------------------------------" << std::endl
              << "Average over " << iters << " iterations."       << std::endl;

    for (int i = 0; i < sim.maxObjects; ++i) {
        sim.addObject(rand() % 100, 0.001, Eigen::Vector2d::Random()*1000, Eigen::Vector2d::Random()*1);
    }

    std::chrono::time_point<std::chrono::steady_clock> start, end;
    double stepAvg = 0, computeForcesAvg = 0;
    for (int i = 0; i < iters; ++i) {
        start = std::chrono::high_resolution_clock::now();
        sim.computeForces();
        end   = std::chrono::high_resolution_clock::now();
        computeForcesAvg += std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        start = std::chrono::high_resolution_clock::now();
        sim.step();
        end   = std::chrono::high_resolution_clock::now();
        stepAvg += std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    }
    std::cout << "computeForces(): " << computeForcesAvg/iters << " ms" << std::endl;
    std::cout << "step(): " << stepAvg/iters << " ms" << std::endl;
}

int main() {
    // Simulator2d 1k Euler Gravitational_Direct
    Simulator2d sim2d_euler_gd_1k(1, 1000, new EulerIntegrator(),
                                  new Gravitational_Direct(0.01, Unit::LightYear, Unit::SolarMass, Unit::JulianMillenium));

    benchmark(sim2d_euler_gd_1k, 5, "Simulator2d Euler Gravitational_Direct 1k");

    return 0;
}