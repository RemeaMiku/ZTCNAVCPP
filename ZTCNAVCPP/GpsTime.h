#pragma once
#include <chrono>
#include <format>

class GpsTime
{
#define CHRONO std::chrono::
	using ns = CHRONO nanoseconds;
	using gpst = CHRONO gps_time<ns>;
public:
	GpsTime() = default;
	explicit GpsTime(double s);
	GpsTime(unsigned int w, double s);
	GpsTime(const gpst& gpsTime);
	inline constexpr unsigned int Week() const noexcept
	{
		return _week;
	}
	inline constexpr double Second() const noexcept
	{
		return _second;
	}
	inline std::string ToString() const
	{
		return std::format("{} {:.3f}", _week, _second);
	}
	double operator -(const GpsTime& gpsTime) const noexcept;
	GpsTime operator -(double s) const;
private:
	unsigned int _week;
	double _second;
};
