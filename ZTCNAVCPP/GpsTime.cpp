#include "GpsTime.h"

using namespace std::chrono;
GpsTime::GpsTime(double s)
{
	if (s >= DBL_EPSILON)
	{
		_week = static_cast<unsigned int>(s / 604800);
		_second = s - _week * 604800;
	}
	else
	{
		throw "输入的时间不合法";
	}
}
GpsTime::GpsTime(unsigned int w, double s)
{
	if (s > -10E-9 && s < 604800)
	{
		_week = w;
		_second = s;
	}
	else
	{
		throw "输入的时间不合法";
	}
}
GpsTime::GpsTime(const gpst& gpsTime)
{
	if (gpsTime > gpst())
	{
		auto duration { gpsTime.time_since_epoch() };
		auto week { floor<weeks>(duration) };
		_week = week.count();
		_second = (duration - week).count() / static_cast<double>(ns::period::den);
	}
	else
	{
		throw "GpsTime周数不能为负值";
	}
}

double GpsTime::operator-(const GpsTime& gpsTime)const noexcept
{
	return (_week - gpsTime._week) * 604800 + _second - gpsTime._second;
}

GpsTime GpsTime::operator-(double s) const
{
	return GpsTime(_week * 604800 + _second - s);
}