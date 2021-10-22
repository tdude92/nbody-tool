#include "integrator.hpp"

void EulerIntegrator::integrate(double dt, const Eigen::Ref<const Eigen::MatrixXd>& a,
                                Eigen::Ref<Eigen::MatrixXd> v, Eigen::Ref<Eigen::MatrixXd> x) {
    v += a*dt;
    x += v*dt;
}