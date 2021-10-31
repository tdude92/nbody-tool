#include "force_computer.hpp"

#include <Eigen>
#include <cstdint>
#include <cmath>
#include <queue>
#include "octree.hpp"

/* CONSTRUCTORS */
/* CONSTRUCTORS PERFORM UNIT CONVERSIONS */

Gravitational_Direct::Gravitational_Direct(double softening, unit_t l, unit_t m, unit_t t)
: softening(softening)
, G(6.67430e-11/l/l/l*m*t*t) {}


Gravitational_BarnesHut::Gravitational_BarnesHut(double softening, unit_t l, unit_t m, unit_t t)
: root(nullptr)
, softening(softening)
, G(6.67430e-11/l/l/l*m*t*t) {}


/* computeForces() */
void Gravitational_Direct::computeForces(Eigen::Ref<Eigen::Matrix3Xd> a,
                                         const Eigen::Ref<const Eigen::Matrix3Xd>& x,
                                         const Eigen::Ref<const Eigen::RowVectorXd>& m) {
    // Iterate through each pair of objects and compute gravitational force
    uint64_t n = x.cols();
    for (uint64_t i = 0; i < n; ++i) {
        a.col(i).setZero(); // Clear net acceleration

        for (uint64_t j = 0; j < n; ++j) {
            if (i == j) continue;

            Eigen::VectorXd dx = x.col(i) - x.col(j);
            a.col(i) += -G*m(j)*(dx)/std::pow(dx.dot(dx) + softening*softening, 1.5); // Force computation
        }
    }
}


void Gravitational_BarnesHut::computeForces(Eigen::Ref<Eigen::Matrix3Xd> a,
                                            const Eigen::Ref<const Eigen::Matrix3Xd>& x,
                                            const Eigen::Ref<const Eigen::RowVectorXd>& m) {
    // TODO document this
    delete this->root;

    // Get octree root bounds and construct octree root
    Eigen::Matrix<double, 3, 1> minPos = x.rowwise().minCoeff();
    Eigen::Matrix<double, 3, 1> maxPos = x.rowwise().maxCoeff();
    Eigen::Matrix<double, 3, 1> widths = maxPos - minPos;
    double rootWidth = widths.maxCoeff();

    this->root = new OctreeNode(minPos(0), minPos(0) + rootWidth,
                                minPos(1), minPos(1) + rootWidth,
                                minPos(2), minPos(2) + rootWidth);
    
    // Construct Barnes-Hut tree
    for (int i = 0; i < x.cols(); ++i) {
        this->root->addObject(m(i), x.col(i));
    }

    // TODO Compute accelerations
    for (int i = 0; i < x.cols(); ++i) {
    }
}
