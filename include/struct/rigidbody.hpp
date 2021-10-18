#ifndef NBT_RIGIDBODY_HPP
#define NBT_RIGIDBODY_HPP

#include <cstdint>

/**
 *  Simulated objects are represented as the index to
 *  their column in the simulator's structure of arrays.
 */
typedef uint32_t Rigidbody;

/**
 * Struct that contains pointers to the data of a Rigidbody
 * Usually gotten from the Simulator2d::getObject (Simulator2d)
 * and Simulator3d::getObject (Simulator3d) methods.
 */
struct RigidbodyData {
    Eigen::Map<Eigen::Vector3d> pos;
    Eigen::Map<Eigen::Vector3d> v;
    Eigen::Map<Eigen::Vector3d> a;
    const double* m;
    const double* r;
};

#endif