#ifndef NBT_OCTREE_HPP
#define NBT_OCTREE_HPP

#include <Eigen>

/**
 * @brief Node struct for Octree.
 */
struct OctreeNode {
    // TODO OctreeNode document
    OctreeNode* children[8];

    bool isParent;
    bool isEmpty;
    bool isExternal;

    double xMin, xMax;
    double yMin, yMax;
    double zMin, zMax;

    double totalMass;
    Eigen::Vector3d centerOfMass;

    OctreeNode();
};

#endif