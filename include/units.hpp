#ifndef NBT_UNITS_HPP
#define NBT_UNITS_HPP

// Units defined in terms of SI units
typedef double unit_t;
namespace Unit {
    // Length
    const double Picometer  = 1e-12;
    const double Nanometer  = 1e-9;
    const double Micrometer = 1e-6;
    const double Millimeter = 1e-3;
    const double Centimeter = 1e-2;
    const double Meter = 1.0; // SI
    const double Kilometer = 1e3;
    const double AstronomicalUnit = 149597870700.0;
    const double LightYear = 9460730472580800.0;
    const double Parsec =    30856775814671900.0;

    // Mass
    const double ProtonMass = 1.67262192369e-27;
    const double Nanogram  = 1e-12;
    const double Microgram = 1e-9;
    const double Milligram = 1e-6;
    const double Gram      = 1e-3;
    const double Kilogram = 1.0; // SI
    const double EarthMass = 5.9722e24;
    const double SolarMass = 1.98847e30;

    // Time
    const double Nanosecond  = 1e-9;
    const double Microsecond = 1e-6;
    const double Millisecond = 1e-3;
    const double Second = 1.0; // SI
    const double Hour = 3600.0;
    const double EarthDay = 86400.0;
    const double JulianYear      = 31557600.0;
    const double JulianDecade    = 315576000.0;
    const double JulianCentury   = 3155760000.0;
    const double JulianMillenium = 31557600000.0;
}

#endif