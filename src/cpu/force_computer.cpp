#include "force_computer.hpp"

#include <Eigen>
#include <cstdint>
#include <cmath>
#include <queue>
#include <vector>
#include <thread>
#include "units.hpp"
#include "octree.hpp"


/* class DynamicsEngine */

double DynamicsEngine::totalPotentialEnergy(const Eigen::Ref<const Eigen::Matrix3Xd>& x,
                                            const Eigen::Ref<const Eigen::RowVectorXd>& m) {
    // Iterate through each pair of distinct objects and adds their potential energy to a sum.
    double sumPotentialEnergies = 0;
    for (int i = 0; i < x.cols(); ++i) {
        for (int j = 0; j < x.cols(); ++j) {
            if (i == j) continue;
            sumPotentialEnergies += this->pairPotentialEnergy(x.col(i), x.col(j), m(i), m(j));
        }
    }
    return sumPotentialEnergies;
}


double DynamicsEngine::pairPotentialEnergy(const Eigen::Vector3d& x_i,
                                           const Eigen::Vector3d& x_j,
                                           double m_i,
                                           double m_j) {
    // Returns zero as a placeholder.
    // Should be overriden by subclasses to get actual potential energy.
    return 0;
}


/* class Abstract_Direct */

void Abstract_Direct::updateAccelerations(Eigen::Ref<Eigen::Matrix3Xd> a,
                                          const Eigen::Ref<const Eigen::Matrix3Xd>& x,
                                          const Eigen::Ref<const Eigen::RowVectorXd>& m) {
    // Iterate through each pair of objects and compute gravitational force
    uint64_t n = x.cols();
    for (uint64_t i = 0; i < n; ++i) {
        a.col(i).setZero(); // Clear net acceleration

        for (uint64_t j = 0; j < n; ++j) {
            if (i == j) continue;
            this->pairAcceleration(a.col(i), x.col(i), x.col(j), m(i), m(j)); // Force computation
        }
    }
}


/* class Abstract_BarnesHut */

Abstract_BarnesHut::Abstract_BarnesHut(double theta)
: root(nullptr)
, theta(theta) {}


Abstract_BarnesHut::~Abstract_BarnesHut() {
    // this->root guaranteed pointing to OctreeNode or nullptr
    delete this->root;
}


void Abstract_BarnesHut::threadUpdateAccelerations(Eigen::Ref<Eigen::Matrix3Xd> a,
                                                   const Eigen::Ref<const Eigen::Matrix3Xd>& x,
                                                   const Eigen::Ref<const Eigen::RowVectorXd>& m,
                                                   int startIdx, int endIdx) {
    for (int i = startIdx; i < endIdx; ++i) {
        // Set acceleration to zero
        a.col(i).setZero();

        // Iterate through tree using breadth-first traversal
        std::queue<OctreeNode*> bfsQ;
        bfsQ.push(this->root);
        while (bfsQ.size() > 0) {
            // Get current node
            OctreeNode* currNode = bfsQ.front();
            bfsQ.pop();

            // Compute s/d
            double s = currNode->xMax - currNode->xMin;
            double d = (x.col(i) - currNode->centerOfMass).norm();
            if (s/d < theta || currNode->isExternal) {
                // Current node is sufficiently far away from the current object
                this->pairAcceleration(a.col(i), x.col(i), currNode->centerOfMass, m(i), currNode->totalMass); // Force computation
            } else {
                // Current node not sufficiently far from the current object
                // Add currNode's children to queue
                for (int z = 0; z < 2; ++z) {
                    for (int y = 0; y < 2; ++y) {
                        for (int x = 0; x < 2; ++x) {
                            OctreeNode* child = currNode->children[z][y][x];
                            if (!child->isEmpty) bfsQ.push(child);
                        }
                    }
                }
            }
        }
    }
}


void Abstract_BarnesHut::updateAccelerations(Eigen::Ref<Eigen::Matrix3Xd> a,
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

    // Compute accelerations
    int nThreads = std::thread::hardware_concurrency();
    int width = x.cols() / nThreads;
    int remainder = x.cols() % nThreads;
    std::vector<std::thread> threads;
    for (int i = 0; i < nThreads; ++i) {
        int startIdx = i*width;
        int endIdx = (i + 1)*width;
        if (i == nThreads - 1) {
            endIdx += remainder; // Add remaining indices to final thread
        }

        threads.emplace_back(&Abstract_BarnesHut::threadUpdateAccelerations, this, a, x, m, startIdx, endIdx);
    }

    for (int i = 0; i < threads.size(); ++i) {
        threads[i].join();
    }
}


/* class Gravitational_Direct */

Gravitational_Direct::Gravitational_Direct(double softening, unit_t l, unit_t m, unit_t t)
: softening(softening)
, G(6.67430e-11/l/l/l*m*t*t) {}


void Gravitational_Direct::pairAcceleration(Eigen::Ref<Eigen::Vector3d> a_i,
                                            const Eigen::Vector3d& x_i,
                                            const Eigen::Vector3d& x_j,
                                            double m_i, double m_j) {
    Eigen::VectorXd dx = x_i - x_j;
    a_i += -G*m_j*(dx)/std::pow(dx.squaredNorm() + softening*softening, 1.5);
}


double Gravitational_Direct::pairPotentialEnergy(const Eigen::Vector3d& x_i,
                                                 const Eigen::Vector3d& x_j,
                                                 double m_i, double m_j) {
    return -G*m_j*m_i/2.0/(x_j - x_i).norm();
}


/* class Gravitational_BarnesHut */

Gravitational_BarnesHut::Gravitational_BarnesHut(double theta, double softening, unit_t l, unit_t m, unit_t t)
: Abstract_BarnesHut(theta)
, softening(softening)
, G(6.67430e-11/l/l/l*m*t*t) {}


void Gravitational_BarnesHut::pairAcceleration(Eigen::Ref<Eigen::Vector3d> a_i,
                                               const Eigen::Vector3d& x_i,
                                               const Eigen::Vector3d& x_j,
                                               double m_i, double m_j) {
    Eigen::VectorXd dx = x_i - x_j;
    a_i += -G*m_j*(dx)/std::pow(dx.squaredNorm() + softening*softening, 1.5);
}


double Gravitational_BarnesHut::pairPotentialEnergy(const Eigen::Vector3d& x_i,
                                                    const Eigen::Vector3d& x_j,
                                                    double m_i, double m_j) {
    return -G*m_j*m_i/2.0/(x_j - x_i).norm();
}
