#pragma once
#include "Coordinate.h"
#include "GeoCoordinate.h"
#include "Ellipsoid.h"
#include "Angle.hpp"

class CartCoordinate :
	public Coordinate
{
public:
	CartCoordinate() noexcept;
	CartCoordinate(double x, double y, double z) noexcept;
	explicit CartCoordinate(const std::vector<double>& xyz);
	inline double& X() noexcept { return _elements[0]; }
	inline const double& X() const noexcept { return _elements[0]; }
	inline double& Y() noexcept { return _elements[1]; }
	inline const double& Y() const noexcept { return _elements[1]; }
	inline double& Z() noexcept { return _elements[2]; }
	inline const double& Z() const noexcept { return _elements[2]; }
	static double Distance(const CartCoordinate& p1, const CartCoordinate p2) noexcept;
	GeoCoordinate ToBLH(const Ellipsoid& e)const noexcept;
	static CartCoordinate FromBLH(const GeoCoordinate& blh, const Ellipsoid& e) noexcept;
	std::string ToString() const noexcept override;
};
