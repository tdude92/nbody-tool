#ifndef NBT_OCTREE_HPP
#define NBT_OCTREE_HPP

#include <Eigen>
#include "simulator.hpp"
#include "rigidbody.hpp"

/**
 * @brief Node struct for Octree.
 */
struct OctreeNode { // TODO document OctreeNode
    OctreeNode* children[2][2][2]; // z y x

    bool isEmpty;
    bool isExternal;

    double xMin, xMax;
    double yMin, yMax;
    double zMin, zMax;

    double totalMass;
    Eigen::Vector3d centerOfMass;

    OctreeNode(double xMin, double xMax, double yMin, double yMax, double zMin, double zMax);
    ~OctreeNode();

    void addObject(double m, const Eigen::Ref<const Eigen::Vector3d> pos);
};

#endif