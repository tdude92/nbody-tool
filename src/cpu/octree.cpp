#include "octree.hpp"

/* Utility Functions */
double midpoint(double min, double max) {
    // Returns the midpoint bisecting min and max.
    return (min + max)/2.0;
}

int childIdx(double x, double mid) {
    // Returns 0 if x <  mid
    // Returns 1 if x >= mid
    // The return value is used to find the axis index of the
    // appropriate OctreeNode child of an object.
    return x < mid ? 0 : 1;
}


/* OctreeNode method implementations */

OctreeNode::OctreeNode(double xMin, double xMax, double yMin, double yMax, double zMin, double zMax)
: children() // Initialize children to nullptrs
, isEmpty(true)
, isExternal(true)
, xMin(xMin), xMax(xMax)
, yMin(yMin), yMax(yMax)
, zMin(zMin), zMax(zMax) {}


OctreeNode::~OctreeNode() {
    delete[] this->children;
}


void OctreeNode::addObject(double m, const Eigen::Ref<const Eigen::Vector3d> pos) {
    // Recursively add object to Octree
    if (this->isEmpty) {
        // Node is empty
        // Add object to this node
        this->totalMass = m;
        this->centerOfMass = pos;

        this->isEmpty = false;
    } else if (this->isExternal) {
        // Node is external
        // Compute midpoints
        double xMid = midpoint(this->xMin, this->xMax);
        double yMid = midpoint(this->yMin, this->yMax);
        double zMid = midpoint(this->zMin, this->zMax);

        // Iterate through each child (z, y, x)
        // and determine child_Mins and child_Maxs
        for (int z = 0; z < 2; ++z) {
            // Compute child_zMin, child_zMax
            double child_zMin, child_zMax;
            switch (z) {
                case 0:
                    child_zMin = this->zMin;
                    child_zMax = zMid;
                    break;
                case 1:
                    child_zMin = zMid;
                    child_zMax = this->zMax;
                    break;
            }

            for (int y = 0; y < 2; ++y) {
                // Compute child_zMin, child_zMax
                double child_yMin, child_yMax;
                switch (y) {
                    case 0:
                        child_yMin = this->yMin;
                        child_yMax = yMid;
                        break;
                    case 1:
                        child_yMin = yMid;
                        child_yMax = this->yMax;
                        break;
                }

                for (int x = 0; x < 2; ++x) {
                    // Compute child_zMin, child_zMax
                    double child_xMin, child_xMax;
                    switch (x) {
                        case 0:
                            child_xMin = this->xMin;
                            child_xMax = xMid;
                            break;
                        case 1:
                            child_xMin = xMid;
                            child_xMax = this->xMax;
                            break;
                    }

                    // Construct child
                    this->children[z][y][x] = new OctreeNode(child_xMin, child_xMax,
                                                             child_yMin, child_yMax,
                                                             child_zMin, child_zMax);
                }
            }
        }
 
        // Compute preexisting object child indices
        int pre_xIdx = childIdx(this->centerOfMass(0), xMid);
        int pre_yIdx = childIdx(this->centerOfMass(1), yMid);
        int pre_zIdx = childIdx(this->centerOfMass(2), zMid);

        // Compute new object child indices
        int new_xIdx = childIdx(pos(0), xMid);
        int new_yIdx = childIdx(pos(1), yMid);
        int new_zIdx = childIdx(pos(2), zMid);

        // Recursively add preexisting and new body to children
        this->children[pre_zIdx][pre_yIdx][pre_xIdx]->addObject(this->totalMass, this->centerOfMass);
        this->children[new_zIdx][new_yIdx][new_xIdx]->addObject(m, pos);

        // Update totalMass and centerOfMass
        double newTotalMass = this->totalMass + m;
        this->centerOfMass = (this->totalMass*this->centerOfMass + m*pos)/newTotalMass;
        this->totalMass = newTotalMass;

        this->isExternal = false;
    } else {
        // Node is internal
        // Update totalMass and centerOfMass
        double newTotalMass = this->totalMass + m;
        this->centerOfMass = (this->totalMass*this->centerOfMass + m*pos)/newTotalMass;
        this->totalMass = newTotalMass;

        // Compute midpoints
        double xMid = midpoint(this->xMin, this->xMax);
        double yMid = midpoint(this->yMin, this->yMax);
        double zMid = midpoint(this->zMin, this->zMax);

        // Compute axis indices
        int xIdx = childIdx(pos(0), xMid);
        int yIdx = childIdx(pos(1), yMid);
        int zIdx = childIdx(pos(2), zMid);

        // Recursively add node to child
        this->children[zIdx][yIdx][xIdx]->addObject(m, pos);
    }
}