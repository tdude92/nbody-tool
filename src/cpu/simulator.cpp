#include "simulator.hpp"

#include <exception>
#include <cstdint>
#include <limits>
#include <Eigen>
#include "integrator.hpp"
#include "rigidbody.hpp"

Simulator::Simulator(double timeStep, uint64_t maxObjects, Integrator* integrator, ForceComputer* forceComputer)
: timeStep(timeStep)
, maxObjects(maxObjects)
, integrator(integrator)
, forceComputer(forceComputer)
, m(1, maxObjects)
, r(1, maxObjects)
, pos(3, maxObjects)
, v(3, maxObjects)
, a(3, maxObjects)
, id2idx(maxObjects, RIGIDBODY_IDX_NULL)
, idx2id(maxObjects, RIGIDBODY_ID_NULL)
, availableUsedIDs{} {}


Simulator::~Simulator() {
    // Deallocate heap variables
    delete this->integrator;
    delete this->forceComputer;
}


Eigen::Ref<Eigen::MatrixXd> Simulator::active(Eigen::Ref<Eigen::MatrixXd> mat) {
    // Return map to slice (1, this->nextIdx). Works because SoA is densely packed.
    return mat(Eigen::all, Eigen::seq(0, this->nextIdx - 1));
}


Rigidbody Simulator::nObjects() {
    return this->nextIdx;
}


Rigidbody Simulator::addObject(double m, double r, const Eigen::Vector3d& p0, const Eigen::Vector3d& v0) {
    // Update SoA and nextIdx

    // Check for max number of objects reached
    if (this->nextIdx >= this->maxObjects) {
        throw std::overflow_error("ERROR: Max number of objects reached.");
    }

    // Get new object's index in SoA
    RigidbodyIdx idx = this->nextIdx++;

    // If a used id is not available, increment this->nextID and use that
    Rigidbody id;
    if (this->availableUsedIDs.empty()) {
        id = this->nextID++;
    } else {
        id = this->availableUsedIDs.front();
        this->availableUsedIDs.pop();
    }

    // Update id-idx mappings
    this->id2idx[id] = idx;
    this->idx2id[idx] = id;

    // Update structure of arrays
    this->m(idx) = m;
    this->r(idx) = r;
    this->pos(Eigen::all, idx) = p0;
    this->v(Eigen::all,   idx) = v0;
    this->a(Eigen::all,   idx) = Eigen::Vector3d::Zero();

    return id;
}


void Simulator::delObject(Rigidbody id) {
    // Check object exists
    if (!this->rb_exists(id)) {
        throw std::out_of_range("ERROR: Rigidbody id not valid.");
    }

    // Get deleted object's idx
    RigidbodyIdx idx = this->id2idx[id];

    // Get index and id of topmost object (object with greatest index)
    RigidbodyIdx topIdx = this->nextIdx - 1;
    Rigidbody    topID  = this->idx2id[topIdx];

    // Move data at topIdx to idx
    this->m(idx) = this->m(topIdx);
    this->r(idx) = this->r(topIdx);
    this->pos(Eigen::all, idx) = this->pos(Eigen::all, topIdx);
    this->v(Eigen::all,   idx) = this->v(Eigen::all,   topIdx);
    this->a(Eigen::all,   idx) = this->a(Eigen::all,   topIdx);

    // Assign: idx to topID, topID to idx, null to id, null to topIdx
    this->id2idx[topID] = idx;
    this->idx2id[idx] = topID;
    this->id2idx[id] = RIGIDBODY_IDX_NULL;
    this->idx2id[topIdx] = RIGIDBODY_ID_NULL;

    // Push id to used available IDs
    this->availableUsedIDs.push(id);

    // Decrement nextIdx
    this->nextIdx--;
}


bool Simulator::rb_exists(Rigidbody id) {
    if (id < 0 || id >= this->maxObjects || this->id2idx[id] == RIGIDBODY_IDX_NULL) {
        // ID is out of range or does not point to existing object
        return false;
    } else {
        return true;
    }
}


Eigen::Ref<const Eigen::Vector3d> Simulator::rb_pos(Rigidbody id) {
    return this->pos.col(this->id2idx[id]);
}


Eigen::Ref<const Eigen::Vector3d> Simulator::rb_v(Rigidbody id) {
    return this->v.col(this->id2idx[id]);
}


Eigen::Ref<const Eigen::Vector3d> Simulator::rb_a(Rigidbody id) {
    return this->a.col(this->id2idx[id]);
}


double Simulator::rb_m(Rigidbody id) {
    return this->m(this->id2idx[id]);
}


double Simulator::rb_r(Rigidbody id) {
    return this->r(this->id2idx[id]);
}


Eigen::Ref<const Eigen::Matrix3Xd> Simulator::activePos() {
    return this->active(this->pos);
}

Eigen::Ref<const Eigen::Matrix3Xd> Simulator::activeV() {
    return this->active(this->v);
}

Eigen::Ref<const Eigen::Matrix3Xd> Simulator::activeA() {
    return this->active(this->a);
}

Eigen::Ref<const Eigen::RowVectorXd> Simulator::activeM() {
    return this->active(this->m);
}

Eigen::Ref<const Eigen::RowVectorXd> Simulator::activeR() {
    return this->active(this->r);
}

void Simulator::computeForces() {
    this->forceComputer->computeForces(
        this->active(this->a),
        this->active(this->pos),
        this->active(this->m)
    );
}


void Simulator::step() {
    this->integrator->integrate(
        this->timeStep,
        this->active(this->a),
        this->active(this->v),
        this->active(this->pos)
    );
}
