#include "units.hpp"
#include <iostream>

int main() {
    std::cout << 0.0000000000667430/Unit::Parsec/Unit::Kilometer/Unit::Kilometer*Unit::SolarMass << std::endl;
    return 0;
}

// TODO move to test/