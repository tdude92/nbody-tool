#include "integrator.hpp"

/* class EulerIntegrator */

void EulerIntegrator::integrate(double dt, const Eigen::Ref<const Eigen::MatrixXd>& a,
                                Eigen::Ref<Eigen::MatrixXd> v, Eigen::Ref<Eigen::MatrixXd> x) {
    x += v*dt;
    v += a*dt;
}


/* class VerletIntegrator */

void VerletIntegrator::integrate(double dt, const Eigen::Ref<const Eigen::MatrixXd>& a,
                                Eigen::Ref<Eigen::MatrixXd> v, Eigen::Ref<Eigen::MatrixXd> x) {
    if (!this->isFirstIteration) {
        v = v + 0.5*(aPrev + a)*dt;
    } else {
        this->isFirstIteration = false;
    }
    
    x = x + v*dt + 0.5*a*dt*dt;
    aPrev = a;
}
