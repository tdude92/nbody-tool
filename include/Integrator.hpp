#ifndef NBT_INTEGRATOR
#define NBT_INTEGRATOR

#include <Eigen>

enum class Integrator {
    Euler = 0,
    Verlet
};

void eulerIntegrate(double dt, const Eigen::Matrix<double, 2, Eigen::Dynamic>& a, Eigen::Matrix<double, 2, Eigen::Dynamic>& v, Eigen::Matrix<double, 2, Eigen::Dynamic>& x); //TODO euler 2d
void eulerIntegrate(double dt, const Eigen::Matrix<double, 3, Eigen::Dynamic>& a, Eigen::Matrix<double, 3, Eigen::Dynamic>& v, Eigen::Matrix<double, 3, Eigen::Dynamic>& x); // TODO euler 3d

void verletIntegrate(double dt, const Eigen::Matrix<double, 2, Eigen::Dynamic>& a, Eigen::Matrix<double, 2, Eigen::Dynamic>& v, Eigen::Matrix<double, 2, Eigen::Dynamic>& x); // TODO verlet 2d
void verletIntegrate(double dt, const Eigen::Matrix<double, 3, Eigen::Dynamic>& a, Eigen::Matrix<double, 3, Eigen::Dynamic>& v, Eigen::Matrix<double, 3, Eigen::Dynamic>& x); // TODO verlet 3d

#endif