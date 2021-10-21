// TODO write docstrings

#ifndef NBT_INTEGRATOR
#define NBT_INTEGRATOR

#include <Eigen>

/**
 * Abstract function object for integrators
 */
class Integrator {
    public:
        // NOTE do these need to take map types?
        virtual void integrate(double dt,
                               const Eigen::Ref<const Eigen::Matrix2Xd>& a,
                               Eigen::Ref<Eigen::Matrix2Xd> v,
                               Eigen::Ref<Eigen::Matrix2Xd> x) = 0;
        
        virtual void integrate(double dt,
                               const Eigen::Ref<const Eigen::Matrix3Xd>& a,
                               Eigen::Ref<Eigen::Matrix3Xd> v,
                               Eigen::Ref<Eigen::Matrix3Xd> x) = 0;
};


class EulerIntegrator: public Integrator {
    public:
        void integrate(double dt,
                       const Eigen::Ref<const Eigen::Matrix2Xd>& a,
                       Eigen::Ref<Eigen::Matrix2Xd> v,
                       Eigen::Ref<Eigen::Matrix2Xd> x); //TODO euler 2d
        void integrate(double dt,
                       const Eigen::Ref<const Eigen::Matrix3Xd>& a,
                       Eigen::Ref<Eigen::Matrix3Xd> v,
                       Eigen::Ref<Eigen::Matrix3Xd> x); // TODO euler 3d
};


class VerletIntegrator: public Integrator {
    public:
        void integrate(double dt,
                             const Eigen::Ref<const Eigen::Matrix2Xd>& a,
                             Eigen::Ref<Eigen::Matrix2Xd> v,
                             Eigen::Ref<Eigen::Matrix2Xd> x); // TODO verlet 2d
        void integrate(double dt,
                             const Eigen::Ref<const Eigen::Matrix3Xd>& a,
                             Eigen::Ref<Eigen::Matrix3Xd> v,
                             Eigen::Ref<Eigen::Matrix3Xd> x); // TODO verlet 3d
};


class RungeKuttaIntegrator: public Integrator {
    public:
        void integrate(double dt,
                                 const Eigen::Ref<const Eigen::Matrix2Xd>& a,
                                 Eigen::Ref<Eigen::Matrix2Xd> v,
                                 Eigen::Ref<Eigen::Matrix2Xd> x); // TODO runge kutta 2d
        void integrate(double dt,
                                 const Eigen::Ref<const Eigen::Matrix3Xd>& a,
                                 Eigen::Ref<Eigen::Matrix3Xd> v,
                                 Eigen::Ref<Eigen::Matrix3Xd> x); // TODO runge kutta 3d
};

#endif