#ifndef NBT_INTEGRATOR
#define NBT_INTEGRATOR

#include <Eigen>

/**
 * Used to identify integrator type used in simulators.s
 */
enum class Integrator {
    Euler = 0,
    Verlet,
    RungeKutta
};

/**
 * @brief 2D Euler integrator
 * 
 * @param dt Time step
 * @param a Accelerations
 * @param v Velocities
 * @param x Positions
 */
void eulerIntegrate(double dt, const Eigen::Matrix<double, 2, Eigen::Dynamic>& a, Eigen::Matrix<double, 2, Eigen::Dynamic>& v, Eigen::Matrix<double, 2, Eigen::Dynamic>& x); //TODO euler 2d

/**
 * @brief 3D Euler integrator
 * 
 * @param dt Time step
 * @param a Accelerations
 * @param v Velocities
 * @param x Positions
 */
void eulerIntegrate(double dt, const Eigen::Matrix<double, 3, Eigen::Dynamic>& a, Eigen::Matrix<double, 3, Eigen::Dynamic>& v, Eigen::Matrix<double, 3, Eigen::Dynamic>& x); // TODO euler 3d


/**
 * @brief 2D Verlet integrator
 * 
 * @param dt Time step
 * @param a Accelerations
 * @param v Velocities
 * @param x Positions
 */
void verletIntegrate(double dt, const Eigen::Matrix<double, 2, Eigen::Dynamic>& a, Eigen::Matrix<double, 2, Eigen::Dynamic>& v, Eigen::Matrix<double, 2, Eigen::Dynamic>& x); // TODO verlet 2d

/**
 * @brief 3D Verlet integrator
 * 
 * @param dt Time step
 * @param a Accelerations
 * @param v Velocities
 * @param x Positions
 */
void verletIntegrate(double dt, const Eigen::Matrix<double, 3, Eigen::Dynamic>& a, Eigen::Matrix<double, 3, Eigen::Dynamic>& v, Eigen::Matrix<double, 3, Eigen::Dynamic>& x); // TODO verlet 3d


/**
 * @brief 2D Runge-Kutta integrator
 * 
 * @param dt Time step
 * @param a Accelerations
 * @param v Velocities
 * @param x Positions
 */
void rungeKuttaIntegrate(double dt, const Eigen::Matrix<double, 2, Eigen::Dynamic>& a, Eigen::Matrix<double, 2, Eigen::Dynamic>& v, Eigen::Matrix<double, 2, Eigen::Dynamic>& x); // TODO runge kutta 2d

/**
 * @brief 3D Runge-Kutta integrator
 * 
 * @param dt Time step
 * @param a Accelerations
 * @param v Velocities
 * @param x Positions
 */
void rungeKuttaIntegrate(double dt, const Eigen::Matrix<double, 3, Eigen::Dynamic>& a, Eigen::Matrix<double, 3, Eigen::Dynamic>& v, Eigen::Matrix<double, 3, Eigen::Dynamic>& x); // TODO runge kutta 3d

#endif