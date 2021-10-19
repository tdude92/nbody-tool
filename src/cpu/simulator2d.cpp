#include "simulator2d.hpp"

#include <exception>
#include <cstdint>
#include <limits>
#include <Eigen>
#include "integrator.hpp"
#include "struct/rigidbody.hpp"

Simulator2d::Simulator2d(double timeStep, uint64_t maxObjects, Integrator* integrator)
: timeStep(timeStep)
, maxObjects(maxObjects)
, integrator(integrator)
, m(1, maxObjects)
, r(1, maxObjects)
, pos(2, maxObjects)
, v(2, maxObjects)
, a(2, maxObjects)
, id2idx(maxObjects, RIGIDBODY_INDEX_NULL)
, idx2id(maxObjects, RIGIDBODY_ID_NULL)
, availableUsedIDs{} {}


Simulator2d::~Simulator2d() {
    // Deallocate heap variables
    delete this->integrator;
}


Eigen::Map<Eigen::Matrix<double, 1, Eigen::Dynamic>> Simulator2d::active(Eigen::Matrix<double, 1, Eigen::Dynamic>& mat) {
    // Return map to slice (1, this->nextIdx). Works because SoA is densely packed.
    return Eigen::Map<Eigen::Matrix<double, 1, Eigen::Dynamic>>(&mat(0), 1, this->nextIdx);
}


Eigen::Map<Eigen::Matrix<double, 2, Eigen::Dynamic>> Simulator2d::active(Eigen::Matrix<double, 2, Eigen::Dynamic>& mat) {
    // Return map to slice (1, this->nextIdx). Works because SoA is densely packed.
    return Eigen::Map<Eigen::Matrix<double, 2, Eigen::Dynamic>>(&mat(0), 2, this->nextIdx);
}


RigidbodyData Simulator2d::getObject(Rigidbody id) {
    RigidbodyIdx idx = this->id2idx[id];
    
    // TODO Populate data
    RigidbodyData data;

    return data;
}


Rigidbody Simulator2d::addObject(double m, double r, const Eigen::Vector3d& p0, const Eigen::Vector3d& v0) {
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


void Simulator2d::delObject(Rigidbody id) {
    // Check object exists
    if (id < 0 || id >= this->maxObjects || this->id2idx[id] == RIGIDBODY_INDEX_NULL) {
        throw std::out_of_range("ERROR: Cannot delete entity: out of range.");
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
    this->id2idx[id] = RIGIDBODY_INDEX_NULL;
    this->idx2id[topIdx] = RIGIDBODY_ID_NULL;

    // Push id to used available IDs
    this->availableUsedIDs.push(id);

    // Decrement nextIdx
    this->nextIdx--;
}
