#include "simulator2d.hpp"

#include <exception>
#include <cstdint>
#include <Eigen>
#include "integrator.hpp"

Simulator2d::Simulator2d(double timeStep, uint64_t maxObjects, Integrator* integrator)
: timeStep(timeStep)
, maxObjects(maxObjects)
, integrator(integrator)
, m(1, maxObjects)
, r(1, maxObjects)
, pos(2, maxObjects)
, v(2, maxObjects)
, a(2, maxObjects) {}


Eigen::Map<Eigen::Matrix<double, 1, Eigen::Dynamic>> Simulator2d::active(Eigen::Matrix<double, 1, Eigen::Dynamic>& mat) {
    // Return map to slice (1, this->nextIdx). Works because SoA is densely packed.
    return Eigen::Map<Eigen::Matrix<double, 1, Eigen::Dynamic>>(&mat(0), 1, this->nextIdx);
}


Eigen::Map<Eigen::Matrix<double, 2, Eigen::Dynamic>> Simulator2d::active(Eigen::Matrix<double, 2, Eigen::Dynamic>& mat) {
    // Return map to slice (1, this->nextIdx). Works because SoA is densely packed.
    return Eigen::Map<Eigen::Matrix<double, 2, Eigen::Dynamic>>(&mat(0), 2, this->nextIdx);
}