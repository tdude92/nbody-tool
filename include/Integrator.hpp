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
                               const Eigen::Matrix<double, 2, Eigen::Dynamic>& a,
                               Eigen::Matrix<double, 2, Eigen::Dynamic>& v,
                               Eigen::Matrix<double, 2, Eigen::Dynamic>& x) = 0;
};


class EulerIntegrator: public Integrator {
    public:
        void integrate(double dt,
                       const Eigen::Matrix<double, 2, Eigen::Dynamic>& a,
                       Eigen::Matrix<double, 2, Eigen::Dynamic>& v,
                       Eigen::Matrix<double, 2, Eigen::Dynamic>& x); //TODO euler 2d
        void integrate(double dt,
                       const Eigen::Matrix<double, 3, Eigen::Dynamic>& a,
                       Eigen::Matrix<double, 3, Eigen::Dynamic>& v,
                       Eigen::Matrix<double, 3, Eigen::Dynamic>& x); // TODO euler 3d
};


class VerletIntegrator: public Integrator {
    public:
        void verletIntegrate(double dt,
                             const Eigen::Matrix<double, 2, Eigen::Dynamic>& a,
                             Eigen::Matrix<double, 2, Eigen::Dynamic>& v,
                             Eigen::Matrix<double, 2, Eigen::Dynamic>& x); // TODO verlet 2d
        void verletIntegrate(double dt,
                             const Eigen::Matrix<double, 3, Eigen::Dynamic>& a,
                             Eigen::Matrix<double, 3, Eigen::Dynamic>& v,
                             Eigen::Matrix<double, 3, Eigen::Dynamic>& x); // TODO verlet 3d
};


class RungeKuttaIntegrator: public Integrator {
    public:
        void rungeKuttaIntegrate(double dt,
                                 const Eigen::Matrix<double, 2, Eigen::Dynamic>& a,
                                 Eigen::Matrix<double, 2, Eigen::Dynamic>& v,
                                 Eigen::Matrix<double, 2, Eigen::Dynamic>& x); // TODO runge kutta 2d
        void rungeKuttaIntegrate(double dt,
                                 const Eigen::Matrix<double, 3, Eigen::Dynamic>& a,
                                 Eigen::Matrix<double, 3, Eigen::Dynamic>& v,
                                 Eigen::Matrix<double, 3, Eigen::Dynamic>& x); // TODO runge kutta 3d
};

#endif