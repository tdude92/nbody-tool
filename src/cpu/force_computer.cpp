#include "force_computer.hpp"

#include <cstdint>
#include <cmath>

/* CONSTRUCTORS */
/* CONSTRUCTORS PERFORM UNIT CONVERSIONS */

Gravitational_Direct::Gravitational_Direct(double softening, unit_t l, unit_t m, unit_t t)
: softening(softening)
, G(0.0000000000667430/l/l/l*m*t*t) {}


/* computeForces() */

void Gravitational_Direct::computeForces(Eigen::Ref<Eigen::MatrixXd> a,
                                         const Eigen::Ref<const Eigen::MatrixXd>& x,
                                         const Eigen::Ref<const Eigen::VectorXd>& m) {
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