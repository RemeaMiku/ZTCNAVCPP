#pragma once
#include "Coordinate.h"

class GeoCoordinate :
	public Coordinate
{
public:
	GeoCoordinate() noexcept;
	GeoCoordinate(double b, double l, double h) noexcept;
	explicit GeoCoordinate(const std::vector<double>& blh);
	inline constexpr double& B() noexcept { return _elements[0]; }
	inline const double& B() const noexcept { return _elements[0]; }
	inline constexpr double& L() noexcept { return _elements[1]; }
	inline const double& L() const noexcept { return _elements[1]; }
	inline constexpr double& H() noexcept { return _elements[2]; }
	inline const double& H() const noexcept { return _elements[2]; }
	std::string ToString() const noexcept override;
};
