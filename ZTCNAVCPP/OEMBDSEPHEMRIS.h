#pragma once
#include "OEMMessageHeader.h"
#include "Ephemeris.h"
#include "Angle.hpp"

class OEMBDSEPHEMRIS :
	public Ephemeris
{
public:
	OEMMessageHeader Header;
	double TimeOfGroupDelay;
	OEMBDSEPHEMRIS(const OEMMessageHeader& hd);
	virtual inline bool IsAvailable(const GpsTime& t) const noexcept override
	{
		return HealthStatus == 0 && abs(t - ReferenceTimeForEphemeris) < 3600;
	}
	virtual CartCoordinate Position(const GpsTime& t) const noexcept override;
	virtual Vector Velocity(const GpsTime& t)const noexcept override;
private:
	virtual inline constexpr double GM() const noexcept override
	{
		return 3.986004418E14;
	}
	virtual inline constexpr double Omega_e() const noexcept override
	{
		return 7.2921150E-5;
	}
	virtual inline constexpr double F() const noexcept override
	{
		return -4.442807301E-10;
	}
};
