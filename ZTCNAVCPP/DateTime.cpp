#include "DateTime.h"

using namespace std::chrono;

DateTime::DateTime(const int& y, const unsigned& mon, const unsigned& d, const unsigned& h, const unsigned& min, const double& s)
{
	auto date { month{ mon } / d / y };
	if (date.ok() && h >= 0 && h < 24 && min >= 0 && min < 60 && s >= DBL_EPSILON && s < 60)
	{
		_time = utc_clock::from_sys(static_cast<sys_days>(date) + hours(h) + minutes(min) + ns(llround(s * ns::period::den)));
	}
	else
	{
		throw "输入的时间不合法";
	}
}

GpsTime DateTime::ToGpsTime() const noexcept
{
	return gps_clock::from_utc(_time);
}

DateTime DateTime::FromGpsTime(const GpsTime& gpsTime) noexcept
{
	return gps_clock::to_utc(gpst(weeks(gpsTime.Week()) + ns(llround(gpsTime.Second() * ns::period::den))));
}