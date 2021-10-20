#include <Eigen>
#include "struct/rigidbody.hpp"

RigidbodyData::~RigidbodyData() {
    delete[] this->pos;
    delete[] this->v;
    delete[] this->a;
}


bool RigidbodyData::exists() {
    if (*idx != RIGIDBODY_INDEX_NULL) {
        return true;
    } else {
        return false;
    }
}