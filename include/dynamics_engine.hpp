#ifndef NBT_DYNAMICS_ENGINE_HPP
#define NBT_DYNAMICS_ENGINE_HPP

#include <Eigen>
#include "octree.hpp"
#include "units.hpp"

/**
 * Abstract class for Dynamics engines.
 * Used to update the accelerations of a set of
 * particles based on their positions and masses.
 */
class DynamicsEngine {
    public:
        /**
         * @brief Recalculates acceleration matrix using object positions and masses
         * 
         * @param a Acceleration matrix
         * @param x Position matrix
         * @param m Mass vector
         */
        virtual void updateAccelerations(Eigen::Ref<Eigen::Matrix3Xd> a,
                                         const Eigen::Ref<const Eigen::Matrix3Xd>& x,
                                         const Eigen::Ref<const Eigen::RowVectorXd>& m) = 0;
        
        /**
         * @brief Updates the individual acceleration between particles i and j
         * 
         * @param a_i Acceleration of i
         * @param x_i Position of i
         * @param x_j Position of j
         * @param m_i Mass of i
         * @param m_j Mass of j
         */
        virtual void pairAcceleration(Eigen::Ref<Eigen::Vector3d> a_i,
                                      const Eigen::Vector3d& x_i,
                                      const Eigen::Vector3d& x_j,
                                      double m_i,
                                      double m_j) = 0;
        
        /**
         * @brief Returns the potential energy of a system based on particle positions and masses.
         * 
         * @param x Position matrix
         * @param m Mass vector
         * 
         * @return double
         */
        double totalPotentialEnergy(const Eigen::Ref<const Eigen::Matrix3Xd>& x,
                                    const Eigen::Ref<const Eigen::RowVectorXd>& m);
        
        /**
         * @brief Returns the potential energy between a pair of particles i and j.
         *        Default abstract class method returns zero for everything, can be overriden
         *        in custom subclasses.
         * 
         * @param x_i Position of i
         * @param x_j Position of j
         * @param m_i Mass of i
         * @param m_j Mass of j
         * @return double 
         */
        virtual double pairPotentialEnergy(const Eigen::Vector3d& x_i,
                                           const Eigen::Vector3d& x_j,
                                           double m_i,
                                           double m_j);
};


/* ABSTRACT CLASSES FOR N-BODY UPDATE METHODS */
/* Implements DynamicsEngine::updateAccelerations() */


/**
 * Computes force directly between every pair of objects. O(n^2)
 */
class Abstract_Direct: public DynamicsEngine {
    public:
        /**
         * @brief Computes force between each pair of particles individually.
         * 
         * @param a
         * @param x
         * @param m
         */
        void updateAccelerations(Eigen::Ref<Eigen::Matrix3Xd> a,
                                 const Eigen::Ref<const Eigen::Matrix3Xd>& x,
                                 const Eigen::Ref<const Eigen::RowVectorXd>& m) override;
};


/**
 * A multithreaded Barnes-Hut force computer. O(nlogn)
 */
class Abstract_BarnesHut: public DynamicsEngine {
    public:
        OctreeNode* root;       //!< Root node of the Barnes-Hut tree.
        const double theta;     //!< Theta parameter for Barnes-Hut algorithm
        
        /**
         * @brief Construct a Abstract_Direct object.
         * 
         * @param theta Theta parameter for the Barnes-Hut algorithm
         */
        Abstract_BarnesHut(double theta);

        /**
         * @brief Destroy the Abstract_BarnesHut object
         */
        ~Abstract_BarnesHut();
        
        /**
         * @brief Function for threads. Computes the acceleration of bodies from indices startIdx to endIdx (endIdx not included)
         * 
         * @param a
         * @param x 
         * @param m 
         * @param startIdx
         * @param endIdx
         */
        void threadUpdateAccelerations(Eigen::Ref<Eigen::Matrix3Xd> a,
                                      const Eigen::Ref<const Eigen::Matrix3Xd>& x,
                                      const Eigen::Ref<const Eigen::RowVectorXd>& m,
                                      int startIdx,
                                      int endIdx);

        /**
         * @brief Computes forces acting on each particle using the Barnes-Hut algorithm
         * 
         * @param a 
         * @param x 
         * @param m 
         */
        void updateAccelerations(Eigen::Ref<Eigen::Matrix3Xd> a,
                                const Eigen::Ref<const Eigen::Matrix3Xd>& x,
                                const Eigen::Ref<const Eigen::RowVectorXd>& m) override;
};


/* SPECIFIC DYNAMICS ENGINES */
/* Implement DynamicsEngine::pairAccleration() */


class Gravitational_Direct: public Abstract_Direct {
    public:
        const double G;
        const double softening;

        /**
         * @brief Construct a new Gravitational_Direct object
         * 
         * @param softening Softening parameter
         * @param l Unit of length
         * @param m Unit of mass
         * @param t Unit of time
         */
        Gravitational_Direct(double softening, unit_t l = Unit::Meter, unit_t m = Unit::Kilogram, unit_t t = Unit::Second);

        /**
         * @brief Computes acceleration from Newtonian gravitation between two particles i and j
         * 
         * @param a_i 
         * @param x_i 
         * @param x_j 
         * @param m_i 
         * @param m_j 
         */
        void pairAcceleration(Eigen::Ref<Eigen::Vector3d> a_i,
                              const Eigen::Vector3d& x_i,
                              const Eigen::Vector3d& x_j,
                              double m_i,
                              double m_j) override;
        
         /**
         * @brief Returns the gravitational potential energy between a pair of particles i and j.
         * 
         * @param x_i Position of i
         * @param x_j Position of j
         * @param m_i Mass of i
         * @param m_j Mass of j
         * @return double 
         */
        double pairPotentialEnergy(const Eigen::Vector3d& x_i,
                                   const Eigen::Vector3d& x_j,
                                   double m_i,
                                   double m_j) override;
};


class Gravitational_BarnesHut: public Abstract_BarnesHut {
    public:
        const double G;
        const double softening;

        /**
         * @brief Construct a new Gravitational_Direct object
         * 
         * @param theta Theta parameter for the Barnes-Hut algorithm
         * @param softening Softening parameter
         * @param l Unit of length
         * @param m Unit of mass
         * @param t Unit of time
         */
        Gravitational_BarnesHut(double theta, double softening, unit_t l = Unit::Meter, unit_t m = Unit::Kilogram, unit_t t = Unit::Second);

        /**
         * @brief Computes acceleration from Newtonian gravitation between two particles i and j
         * 
         * @param a_i 
         * @param x_i 
         * @param x_j 
         * @param m_i 
         * @param m_j 
         */
        void pairAcceleration(Eigen::Ref<Eigen::Vector3d> a_i,
                              const Eigen::Vector3d& x_i,
                              const Eigen::Vector3d& x_j,
                              double m_i,
                              double m_j) override;
        
         /**
         * @brief Returns the gravitational potential energy between a pair of particles i and j.
         * 
         * @param x_i Position of i
         * @param x_j Position of j
         * @param m_i Mass of i
         * @param m_j Mass of j
         * @return double 
         */
        double pairPotentialEnergy(const Eigen::Vector3d& x_i,
                                   const Eigen::Vector3d& x_j,
                                   double m_i,
                                   double m_j) override;
};


#endif