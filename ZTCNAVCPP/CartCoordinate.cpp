#include "CartCoordinate.h"

using namespace std;
using vec = vector<double>;
using ci = const int&;
using cd = const double&;

CartCoordinate::CartCoordinate() noexcept :Coordinate()
{}

CartCoordinate::CartCoordinate(double x, double y, double z) noexcept :Coordinate(x, y, z)
{}

CartCoordinate::CartCoordinate(const std::vector<double>& xyz) :Coordinate(xyz)
{}

double CartCoordinate::Distance(const CartCoordinate& p1, const CartCoordinate p2) noexcept
{
	return sqrt(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2) + pow(p1[2] - p2[2], 2));
}

GeoCoordinate CartCoordinate::ToBLH(const Ellipsoid& e) const noexcept
{
	const double& x = _elements[0];
	const double& y = _elements[1];
	const double& z = _elements[2];
	double L = abs(x) > 1E-6 ? std::atan2(y, x) : (y > 0 ? PI / 2 : -PI / 2);
	double sqrt = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
	if (sqrt < 1E-6)
	{
		return GeoCoordinate {};
	}
	double t0 = z / sqrt;
	double e1_2 = std::pow(e.e1, 2);
	double e2_2 = std::pow(e.e2, 2);
	double p = e.c * e1_2 / sqrt;
	double k = 1 + e2_2;
	double ti = t0;
	double tj = t0 + p * ti / std::sqrt(k + ti * ti);
	while (std::abs(tj - ti) >= 1E-10)
	{
		ti = tj;
		tj = t0 + p * ti / std::sqrt(k + ti * ti);
	}
	double B = std::atan(tj);
	double N = e.N(B);
	double H = sqrt / std::cos(B) - N;
	return GeoCoordinate(B, L, H);
}

CartCoordinate CartCoordinate::FromBLH(const GeoCoordinate& blh, const Ellipsoid& e) noexcept
{
	double b = blh.B();
	double l = blh.L();
	double h = blh.H();
	double N = e.N(b);
	double sinB = std::sin(b);
	double cosB = std::cos(b);
	double sinL = std::sin(l);
	double cosL = std::cos(l);
	double e1_2 = std::pow(e.e1, 2);
	double temp = (N + h) * cosB;
	return CartCoordinate(temp * cosL, temp * sinL, (N * (1 - e1_2) + h) * sinB);
}

string CartCoordinate::ToString() const noexcept
{
	return format("{:.4f} {:.4f} {:.4f}", _elements[0], _elements[1], _elements[2]);
}