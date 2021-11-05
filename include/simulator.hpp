#ifndef NBT_SIMULATOR_HPP
#define NBT_SIMULATOR_HPP

#include <vector>
#include <queue>
#include <cstdint>

#include <Eigen>
#include "integrator.hpp"
#include "force_computer.hpp"
#include "rigidbody.hpp"
#include "octree.hpp"

/**
 * Simulator object to control simulations.
 * Add objects using addObject().
 * Step simulation using step().
 */
class Simulator {
    private:
        Integrator* const integrator;         //!< Integrator used in this simulation.
        DynamicsEngine* const dynamicsEngine; //!< dynamicsEngine use in this simulation.

        double timeStep; //!< dt value used in integrators.

        // Structure of arrays for object properties
        Eigen::Matrix<double, 1, Eigen::Dynamic> m;     //!< Mass of each object packed into a 1 x N vector.
        Eigen::Matrix<double, 1, Eigen::Dynamic> r;     //!< Radius of each object packed into a 1 x N vector.
        Eigen::Matrix<double, 3, Eigen::Dynamic> pos;   //!< 3D position of each object packed into a 3 x N matrix.
        Eigen::Matrix<double, 3, Eigen::Dynamic> v;     //!< 3D velocity of each object packed into a 3 x N matrix.
        Eigen::Matrix<double, 3, Eigen::Dynamic> a;     //!< 3D acceleration of each object packed into a 3 x N matrix.
    
        uint64_t iteration = 0;                 //!< Current iteration of the simulation.
        
        RigidbodyIdx nextIdx = 0;               //!< Index of next available column in the structure of arrays. Also serves as a counter of active objects.
        Rigidbody    nextID = 0;                //!< Next available ID to be assigned to a newly created Rigidbody.
        std::vector<Rigidbody>    idx2id;       //!< Maps index to associated ID
        std::vector<RigidbodyIdx> id2idx;       //!< Maps ID to associated index
        std::queue<Rigidbody> availableUsedIDs; //!< Stores IDs of destroyed objects for reallocation
       
        /*! Returns slice of array structure component with only active objects. */
        Eigen::Ref<Eigen::MatrixXd> active(Eigen::Ref<Eigen::MatrixXd> mat);
    public:
        const Rigidbody maxObjects;         //!< Maximum number of objects in the simulation. Sets the dimensions of the sstructure of arrays.

        /*! Constructs a Simulator object. */
        Simulator(double timeStep, uint64_t maxObjects, Integrator* integrator, DynamicsEngine* dynamicsEngine);
        
        /*! Destroys a Simulator object and deallocates all used memory. */
        ~Simulator();

        /*! Returns sum of kinetic energies of the particles in the system. */
        double totalKineticEnergy();

        /*! Returns sum of potential energies of each particle in the sytem */
        double totalPotentialEnergy();

        /*! Returns the sum of totalKineticEnergy() and totalPotentialEnergy() */
        double totalEnergy();

        /*! Returns sum of moments of inertia of each particle in the system */
        double totalMomentOfInertia();

        /*! Returns number of active objects */
        Rigidbody nObjects();

        /*! Adds an object to the simulation. Can be done during the simulation if the #maxObjects is not met. */
        Rigidbody addObject(double m, double r, const Eigen::Vector3d& p0, const Eigen::Vector3d& v0);
        
        /*! Deletes an object from the simulation. */
        void delObject(Rigidbody id);
        
        /*! Objects combine into id1 and momentum is conserved. */
        void collideObject(Rigidbody id1, Rigidbody id2); // TODO collideObject remember to conserve momentum

        /*! Returns if rigidbody with this id exists. */
        bool rb_exists(Rigidbody id);

        /*! Returns the position vector of a rigidbody. */
        Eigen::Ref<const Eigen::Vector3d> rb_pos(Rigidbody id);

        /*! Returns the velocity vector of a rigidbody. */
        Eigen::Ref<const Eigen::Vector3d> rb_v(Rigidbody id);

        /*! Returns the acceleration vector of a rigidbody. */
        Eigen::Ref<const Eigen::Vector3d> rb_a(Rigidbody id);

        /*! Returns the mass of a rigidbody. */
        double rb_m(Rigidbody id);

        /*! Returns the radius of a rigidbody. */
        double rb_r(Rigidbody id);

        /*! Returns Eigen::Matrix ref of object positions */
        Eigen::Ref<const Eigen::Matrix3Xd> activePos();

        /*! Returns Eigen::Matrix ref of object velocities */
        Eigen::Ref<const Eigen::Matrix3Xd> activeV();

        /*! Returns Eigen::Matrix ref of object accelerations */
        Eigen::Ref<const Eigen::Matrix3Xd> activeA();

        /*! Returns Eigen::Matrix ref of object masses */
        Eigen::Ref<const Eigen::RowVectorXd> activeM();

        /*! Returns Eigen::Matrix ref of object radii */
        Eigen::Ref<const Eigen::RowVectorXd> activeR();

        /*! Computes force between each object using #forceComputer. #a is updated. */
        void computeForces();

        /*! Steps simulation */
        void step();
};

#endif