// TODO write docstrings

#ifndef NBT_INTEGRATOR
#define NBT_INTEGRATOR

#include <Eigen>

/**
 * Abstract function object for integrators
 */
class Integrator {
    public:
        virtual void integrate(double dt,
                               const Eigen::Ref<const Eigen::MatrixXd>& a,
                               Eigen::Ref<Eigen::MatrixXd> v,
                               Eigen::Ref<Eigen::MatrixXd> x) = 0;
};


class EulerIntegrator: public Integrator {
    public:
        void integrate(double dt,
                       const Eigen::Ref<const Eigen::MatrixXd>& a,
                       Eigen::Ref<Eigen::MatrixXd> v,
                       Eigen::Ref<Eigen::MatrixXd> x); //TODO euler
};


class VerletIntegrator: public Integrator {
    public:
        void integrate(double dt,
                       const Eigen::Ref<const Eigen::MatrixXd>& a,
                       Eigen::Ref<Eigen::MatrixXd> v,
                       Eigen::Ref<Eigen::MatrixXd> x); // TODO verlet
};


class RungeKuttaIntegrator: public Integrator {
    public:
        void integrate(double dt,
                       const Eigen::Ref<const Eigen::MatrixXd>& a,
                       Eigen::Ref<Eigen::MatrixXd> v,
                       Eigen::Ref<Eigen::MatrixXd> x); // TODO runge kutta
};

#endif