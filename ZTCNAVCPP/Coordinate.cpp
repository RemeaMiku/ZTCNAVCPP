#include "Coordinate.h"

Coordinate::Coordinate() noexcept :_elements(3, 0) {}

Coordinate::Coordinate(double a, double b, double c) noexcept :_elements{ a,b,c } {}

Coordinate::Coordinate(const std::vector<double>& vec)
{
	if (vec.size() == 3)
	{
		_elements = vec;
	}
	else
	{
		throw "数组元素个数不为3";
	}
}