#ifndef NBT_SIMULATOR_HPP
#define NBT_SIMULATOR_HPP

#include <vector>
#include <queue>
#include <cstdint>

#include <Eigen>
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
        const double timeStep;               //!< dt value used in integrators.
        const Rigidbody maxObjects;          //!< Maximum number of objects in the simulation. Sets the dimensions of the sstructure of arrays.
        const Integrator* const integrator;  //!< Stores type of integrator used in this simulation.

        // Structure of arrays for object properties
        Eigen::Matrix<double, 1, Eigen::Dynamic> m;     //!< Mass of each object packed into a 1 x N vector.
        Eigen::Matrix<double, 1, Eigen::Dynamic> r;     //!< Radius of each object packed into a 1 x N vector.
        Eigen::Matrix<double, 2, Eigen::Dynamic> pos;   //!< 2D position of each object packed into a 2 x N matrix.
        Eigen::Matrix<double, 2, Eigen::Dynamic> v;     //!< 2D velocity of each object packed into a 2 x N matrix.
        Eigen::Matrix<double, 2, Eigen::Dynamic> a;     //!< 2D acceleration of each object packed into a 2 x N matrix.
    
        uint64_t iteration = 0;               //!< Current iteration of the simulation.
        
        RigidbodyIdx nextIdx = 0;               //!< Index of next available column in the structure of arrays. Also serves as a counter of active objects.
        Rigidbody    nextID = 0;                //!< Next available ID to be assigned to a newly created Rigidbody.
        std::vector<Rigidbody>    idx2id;       //!< Maps index to associated ID
        std::vector<RigidbodyIdx> id2idx;       //!< Maps ID to associated index
        std::queue<Rigidbody> availableUsedIDs; //!< Stores IDs of destroyed objects for reallocation
       
        /*! Returns slice of array structure component with only active objects. */
        Eigen::Map<Eigen::Matrix<double, 1, Eigen::Dynamic>> active(Eigen::Matrix<double, 1, Eigen::Dynamic>& mat);
        
        /*! Returns slice of array structure component with only active objects. */
        Eigen::Map<Eigen::Matrix<double, 2, Eigen::Dynamic>> active(Eigen::Matrix<double, 2, Eigen::Dynamic>& mat);
    public:
        /*! Constructs a Simulator2d object. */
        Simulator2d(double timeStep, uint64_t maxObjects, Integrator* integrator);
        
        /*! Destroys a Simulator2d object and deallocates all used memory. */
        ~Simulator2d();

        /*! Returns a RigidBodyData struct containing pointers to the data of a specific object. */
        RigidbodyData getObject(Rigidbody id); // TODO

        /*! Adds an object to the simulation. Can be done during the simulation if the #maxObjects is not met. */
        Rigidbody addObject(double m, double r, const Eigen::Vector3d& p0, const Eigen::Vector3d& v0);
        
        /*! Deletes an object from the simulation. */
        void delObject(Rigidbody id);
        
        /*! Objects combine into id1 and momentum is conserved. */
        void collideObject(Rigidbody id1, Rigidbody id2); // TODO collideObject remember to conserve momentum

        /*! Computes gravitational force between each object using the Barnes-Hut algorithm. #a is updated. */
        void updateAccelerations(); // TODO force update

        /*! Steps simulation */
        void step(); // TODO step function
};

#endif