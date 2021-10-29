#ifndef NBT_RIGIDBODY_HPP
#define NBT_RIGIDBODY_HPP

#include <cstdint>

#define RIGIDBODY_IDX_NULL -1  //!< Null value for a rigidbody index
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

#endif