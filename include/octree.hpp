#ifndef NBT_OCTREE_HPP
#define NBT_OCTREE_HPP

#include <Eigen>
#include "rigidbody.hpp"

/**
 * @brief Node struct for Octree.
 */
struct OctreeNode {
    OctreeNode* children[2][2][2]; //<! 3d Array to store child pointers (Axes in order: z y x)

    bool isEmpty;       //<! True if no objects are in this node.
    bool isExternal;    //<! True if this node is external (has no children).

    double xMin, xMax;  //!< Bounds in x dimension
    double yMin, yMax;  //!< Bounds in y dimension
    double zMin, zMax;  //!< Bounds in z dimension

    double totalMass;               //!< Total mass in the region bounded by the node.
    Eigen::Vector3d centerOfMass;   //!< Center of mass of the objects within this node.

    //!< Constructs an OctreeNode object from x, y, z bounds
    OctreeNode(double xMin, double xMax, double yMin, double yMax, double zMin, double zMax);

    //!< Destroys OctreeNode object by destroying children
    ~OctreeNode();

    //!< Recursively adds an object into the subtree that has this node as root.
    void addObject(double m, const Eigen::Ref<const Eigen::Vector3d> pos);

    //<! Deletes all children below this node.
    void prune();
};

#endif