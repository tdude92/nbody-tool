#ifndef NBT_RIGIDBODY_HPP
#define NBT_RIGIDBODY_HPP

#include <cstdint>

#define RIGIDBODY_INDEX_NULL -1  //!< Null value for a rigidbody index
#define RIGIDBODY_ID_NULL -1     //!< Null value for a rigidbody id

/**
 *  Simulated objects are represented as an ID, which is mapped to
 *  the index their column in the simulator's structure of arrays.
 */
typedef uint64_t Rigidbody;

/**
 * Typename of rigidbody indices in SoA
 */
typedef Rigidbody RigidbodyIdx;

/**
 * Struct that contains pointers to the data of a Rigidbody
 * Usually gotten from the Simulator2d::getObject (Simulator2d)
 * and Simulator3d::getObject (Simulator3d) methods.
 */
struct RigidbodyData {
    Eigen::Map<Eigen::Vector3d> pos;
    Eigen::Map<Eigen::Vector3d> v;
    Eigen::Map<Eigen::Vector3d> a;
    double* m;
    double* r;
};

#endif