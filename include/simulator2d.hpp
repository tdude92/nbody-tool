// TODO write docstrings

#ifndef NBT_SIMULATOR_HPP
#define NBT_SIMULATOR_HPP

#include <Eigen>
#include <array>
#include <vector>
#include <cstdint>

#include "integrator.hpp"
#include "struct/rigidbody.hpp"
#include "struct/quadtree.hpp"

class Simulator2d {
    private:
        const uint32_t maxObjects;
        const Integrator integrator;

        // Function pointer to appropriate integrator
        double (Simulator2d::*integrate)(double dt, const Eigen::Matrix<double, 2, Eigen::Dynamic>& a, Eigen::Matrix<double, 2, Eigen::Dynamic>& v, Eigen::Matrix<double, 2, Eigen::Dynamic>& x);

        // Structure of arrays for object properties
        Eigen::VectorXd m;
        Eigen::VectorXd r;
        Eigen::Matrix<double, 2, Eigen::Dynamic> pos;
        Eigen::Matrix<double, 2, Eigen::Dynamic> v;
        Eigen::Matrix<double, 2, Eigen::Dynamic> a;
    public:
        Simulator2d(uint32_t maxObjects, Integrator integrator = Integrator::Verlet); // TODO constructor 2d
        ~Simulator2d(); // TODO destructor 2d

        RigidBodyData getObject();
        void addObject(double m, const Eigen::Vector3d& p0, const Eigen::Vector3d& v0); // TODO addObject
        void delObject(Rigidbody obj); // TODO delObject
        void combineObject(Rigidbody obj1, Rigidbody obj2s); // TODO combineObject remember to conserve momentum

        void updateForces(); // TODO force update
};

#endif