#include "Ellipsoid.h"

using namespace std;

Ellipsoid::Ellipsoid(double x, double y) noexcept : a(x), b(y), c(std::pow(a, 2) / b), alpha((a - b) / a), e1(std::sqrt(std::pow(a, 2) - std::pow(b, 2)) / a), e2(e2 = std::sqrt(std::pow(a, 2) - std::pow(b, 2)) / b)
{}

const std::map<SatelliteSystem, Ellipsoid> Ellipsoid::Ellipsoids { {GPS, WGS_84}, { BDS,CGCS2000 } };