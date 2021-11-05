#include <iostream>
#include <chrono>
#include <string>
#include <cstdlib>
#include <Eigen>
#include "nbodytool.hpp"

void initializeSim(Simulator& sim) {
    for (int i = 0; i < sim.maxObjects; ++i) {
        sim.addObject(rand() % 100, 0.001, Eigen::Vector3d::Random()*10, Eigen::Vector3d::Zero());
    }
}

void benchmark(Simulator& sim, int iters, const std::string& name) {
    std::cout << "Benchmarking " << name                  << std::endl
              << "--------------------------------------" << std::endl
              << "Average over " << iters << " iterations."       << std::endl;

    std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
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
    std::cout << "step(): " << stepAvg/iters << " ms" << std::endl << std::endl;
}

void energyConservationTest(Simulator& sim, int iters, const std::string& name) {
    std::cout << "Testing Energy Conservation: " << name                  << std::endl
              << "--------------------------------------" << std::endl
              << "Over " << iters << " iterations."       << std::endl;
    
    double initialEnergy = sim.totalEnergy();
    std::cout << "Initial Energy: " << initialEnergy << std::endl;

    double avgEnergyChange = 0;
    double prevEnergy = initialEnergy;
    for (int i = 0; i < iters; ++i) {
        sim.computeForces();
        sim.step();
        double currEnergy = sim.totalEnergy();
        double energyChange = currEnergy - prevEnergy;
        avgEnergyChange += energyChange;
        prevEnergy = currEnergy;

        std::cout << "Step " << i + 1 << " | Current Energy: " << currEnergy << " | Change: " << 
                     energyChange << " | Avg. Change: " << avgEnergyChange/(i+1) << std::endl;
    }

    
}

int main() {
    // Simulator2d 1k Euler Gravitational_Direct
    Simulator sim2d_euler_gd_1k(1, 1000, new EulerIntegrator(),
                                new Gravitational_Direct(0.1, Unit::LightYear, Unit::SolarMass, Unit::JulianMillenium));

    Simulator sim2d_euler_gbh_10k(1, 1000, new EulerIntegrator(),
                                 new Gravitational_BarnesHut(1, 0.1, Unit::LightYear, Unit::SolarMass, Unit::JulianMillenium));

    initializeSim(sim2d_euler_gd_1k);
    initializeSim(sim2d_euler_gbh_10k);

    energyConservationTest(sim2d_euler_gbh_10k, 5, "Simulator Euler Gravitational_BarnesHut 10k");

    benchmark(sim2d_euler_gd_1k, 3, "Simulator Euler Gravitational_Direct 1k");
    benchmark(sim2d_euler_gbh_10k, 5, "Simulator Euler Gravitational_BarnesHut 10k");

    return 0;
}
