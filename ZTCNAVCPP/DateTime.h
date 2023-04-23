#pragma once
#include <chrono>
#include <format>
#include "GpsTime.h"

class DateTime
{
#define CHRONO std::chrono::
	using ns = CHRONO nanoseconds;
	using utc = CHRONO utc_time<ns>;
	using gpst = CHRONO gps_time<ns>;
	using date = CHRONO year_month_day;
public:
	DateTime() = default;
	DateTime(const int& y, const unsigned& mon, const unsigned& d, const unsigned& h, const unsigned& min, const double& s);
	inline constexpr DateTime(const utc& utcTime) noexcept :_time(utcTime)
	{}
	GpsTime ToGpsTime() const noexcept;
	static DateTime FromGpsTime(const GpsTime& gpsTime) noexcept;
	inline std::string ToString() const noexcept
	{
		return std::format("{:%Y/%m/%d %T}", _time);
	}
private:
	utc _time;
};