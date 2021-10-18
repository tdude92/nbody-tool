#ifndef NBT_SIMULATOR_HPP
#define NBT_SIMULATOR_HPP

#include <Eigen>
#include <array>
#include <vector>
#include <cstdint>

#include "integrator.hpp"
#include "struct/rigidbody.hpp"
#include "struct/quadtree.hpp"

/**
 * Simulator object for 2D simulations.
 * Add objects using addObject().
 * Step simulation using step().
 */
class Simulator2d {
    private:
        const uint64_t maxObjects;      //!< Maximum number of objects in the simulation. Sets the dimensions of the sstructure of arrays.
        const Integrator integrator;    //!< Stores type of integrator used in this simulation.
        const double timeStep;          //!< dt value used in integrators.

        /*! Function pointer to appropriate integrator */
        double (Simulator2d::*integrate)(double dt, const Eigen::Matrix<double, 2, Eigen::Dynamic>& a, Eigen::Matrix<double, 2, Eigen::Dynamic>& v, Eigen::Matrix<double, 2, Eigen::Dynamic>& x);

        // Structure of arrays for object properties
        Eigen::Matrix<double, 1, Eigen::Dynamic> m;     //!< Mass of each object packed into a 1 x N vector.
        Eigen::Matrix<double, 1, Eigen::Dynamic> r;     //!< Radius of each object packed into a 1 x N vector.
        Eigen::Matrix<double, 2, Eigen::Dynamic> pos;   //!< 2D position of each object packed into a 2 x N matrix.
        Eigen::Matrix<double, 2, Eigen::Dynamic> v;     //!< 2D velocity of each object packed into a 2 x N matrix.
        Eigen::Matrix<double, 2, Eigen::Dynamic> a;     //!< 2D acceleration of each object packed into a 2 x N matrix.
    public:
        /*! Constructs a Simulator2d object. */
        Simulator2d(uint64_t maxObjects, Integrator integrator = Integrator::Verlet); // TODO constructor 2d
        
        /*! Destroys a Simulator2d object and deallocates all used memory. */
        ~Simulator2d(); // TODO destructor 2d

        /*! Returns a RigidBodyData struct containing pointers to the data of a specific object. */
        RigidbodyData getObject(Rigidbody obj); // TODO object data getter

        /*! Adds an object to the simulation. Can be done during the simulation if the #maxObjects is not met. */
        Rigidbody addObject(double m, const Eigen::Vector3d& p0, const Eigen::Vector3d& v0); // TODO addObject
        
        /*! Deletes an object from the simulation. */
        void delObject(Rigidbody obj); // TODO delObject
        
        /*! Objects combine into obj1 and momentum is conserved. */
        void collideObject(Rigidbody obj1, Rigidbody obj2s); // TODO collideObject remember to conserve momentum

        /*! Computes gravitational force between each object using the Barnes-Hut algorithm. #a is updated. */
        void updateAccelerations(); // TODO force update

        /*! Steps simulation */
        void step(); // TODO step function
};

#endif