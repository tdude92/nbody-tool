// TODO document these

#ifndef NBT_FORCES_HPP
#define NBT_FORCES_HPP

#include <Eigen>
#include "units.hpp"

class Forces {
    public:
        virtual void computeForces(Eigen::Ref<Eigen::MatrixXd> a, Eigen::Ref<Eigen::MatrixXd> x) = 0;
};


// TODO
class GravitationalForces_Direct: public Forces {
    public:
        const double G = 0.0000000000667430; // TODO convert units in constructor
        void computeForces(Eigen::Ref<Eigen::MatrixXd> a, Eigen::Ref<Eigen::MatrixXd> x) {

        }
};


#endif