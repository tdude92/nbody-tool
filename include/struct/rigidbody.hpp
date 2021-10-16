#ifndef NBT_RIGIDBODY_HPP
#define NBT_RIGIDBODY_HPP

#include <cstdint>

typedef uint32_t Rigidbody; // Rigidbodies are a column in the structure of arrays in Simulator

struct RigidBodyData {
    Eigen::Map<Eigen::Vector3d> pos;
    Eigen::Map<Eigen::Vector3d> v;
    Eigen::Map<Eigen::Vector3d> a;
    double* m;
    double* r;
};

#endif