#include "GeoCoordinate.h"
#include "Angle.hpp"

using namespace std;

GeoCoordinate::GeoCoordinate() noexcept :Coordinate() {}

GeoCoordinate::GeoCoordinate(double b, double l, double h) noexcept :Coordinate(b, l, h) {}

GeoCoordinate::GeoCoordinate(const std::vector<double>& blh) :Coordinate(blh) {}

string GeoCoordinate::ToString() const noexcept
{
	return format("{:.8f} {:.8f} {:.4f}", Angle::ToDegree(_elements[0]), Angle::ToDegree(_elements[1]), _elements[2]);
}