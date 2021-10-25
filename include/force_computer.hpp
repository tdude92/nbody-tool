#ifndef NBT_FORCES_HPP
#define NBT_FORCES_HPP

#include <Eigen>
#include "units.hpp"

/**
 * Abstract class for force computers.
 * Force computers are functional interfaces
 * that compute accelerations given some parameters.
 */
class ForceComputer {
    public:
        /**
         * @brief Recalculates acceleration matrix using object positions and masses
         * 
         * @param a Acceleration matrix
         * @param x Position matrix
         * @param m Mass vector
         */
        virtual void computeForces(Eigen::Ref<Eigen::MatrixXd> a,
                                   const Eigen::Ref<const Eigen::MatrixXd>& x,
                                   const Eigen::Ref<const Eigen::VectorXd>& m) = 0;
};


/**
 * A force computer that directly computes
 * gravitational force between each pair of objects.
 */
class Gravitational_Direct: public ForceComputer {
    public:
        const double softening; //!< Softening parameter for non-colliding n-body sim.
        const double G;         //!< Gravitational constant. Changes depending on units.
        
        /**
         * @brief Construct a new Gravitational_Direct object. Performs unit conversions on #G.
         * 
         * @param softening Softening parameter
         * @param l Unit of length
         * @param m Unit of mass
         * @param t Unit of time
         */
        Gravitational_Direct(double softening, unit_t l = Unit::Meter, unit_t m = Unit::Kilogram, unit_t t = Unit::Second);
        void computeForces(Eigen::Ref<Eigen::MatrixXd> a,
                           const Eigen::Ref<const Eigen::MatrixXd>& x,
                           const Eigen::Ref<const Eigen::VectorXd>& m);
};


#endif