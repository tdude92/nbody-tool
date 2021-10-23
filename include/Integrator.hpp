#ifndef NBT_INTEGRATOR
#define NBT_INTEGRATOR

#include <Eigen>

/**
 * Abstract function object for integrators. Integrators
 * are functional interfaces that update the position and
 * velocity matrices using an acceleration matrix.
 */
class Integrator {
    public:
        /**
         * @brief Computes velocities and positions from accelerations and time step.
         * 
         * @param dt Time step
         * @param a Acceleration matrix
         * @param v Velocity matrix
         * @param x Position matrix
         */
        virtual void integrate(double dt,
                               const Eigen::Ref<const Eigen::MatrixXd>& a,
                               Eigen::Ref<Eigen::MatrixXd> v,
                               Eigen::Ref<Eigen::MatrixXd> x) = 0;
};


/**
 * Performs Euler integration. Fast but inaccurate.
 */
class EulerIntegrator: public Integrator {
    public:
        /**
         * @brief Computes velocities and positions from accelerations and time step.
         * 
         * @param dt Time step
         * @param a Acceleration matrix
         * @param v Velocity matrix
         * @param x Position matrix
         */
        void integrate(double dt,
                       const Eigen::Ref<const Eigen::MatrixXd>& a,
                       Eigen::Ref<Eigen::MatrixXd> v,
                       Eigen::Ref<Eigen::MatrixXd> x);
};


/**
 * Performs Verlet integration. Is symplectic but not as accurate as RungeKuttaIntegrator.
 */
class VerletIntegrator: public Integrator {
    public:
        /**
         * @brief Computes velocities and positions from accelerations and time step.
         * 
         * @param dt Time step
         * @param a Acceleration matrix
         * @param v Velocity matrix
         * @param x Position matrix
         */
        void integrate(double dt,
                       const Eigen::Ref<const Eigen::MatrixXd>& a,
                       Eigen::Ref<Eigen::MatrixXd> v,
                       Eigen::Ref<Eigen::MatrixXd> x); // TODO verlet
};


/**
 * Performs Runge-Kutta 4th order integration method. More accurate than EulerIntegrator but slower.
 */
class RungeKuttaIntegrator: public Integrator {
    public:
        /**
         * @brief Computes velocities and positions from accelerations and time step.
         * 
         * @param dt Time step
         * @param a Acceleration matrix
         * @param v Velocity matrix
         * @param x Position matrix
         */
        void integrate(double dt,
                       const Eigen::Ref<const Eigen::MatrixXd>& a,
                       Eigen::Ref<Eigen::MatrixXd> v,
                       Eigen::Ref<Eigen::MatrixXd> x); // TODO runge kutta
};

#endif