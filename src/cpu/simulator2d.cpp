#include "simulator2d.hpp"

#include <exception>
#include "integrator.hpp"

Simulator2d::Simulator2d(double timeStep, uint64_t maxObjects, Integrator* integrator)
: timeStep(timeStep)
, maxObjects(maxObjects)
, integrator(integrator) {
    // Initialize structure of arrays
    
}