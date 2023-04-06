#pragma once
#include "OEMMessageHeader.h"
#include "Ephemeris.h"

class OEMGPSEPHEM :
	public Ephemeris
{
public:
	OEMMessageHeader Header;
	OEMGPSEPHEM(const OEMMessageHeader& hd);
	virtual inline bool IsAvailable(const GpsTime& t) const noexcept { return HealthStatus==0 && abs(t - ReferenceTimeForEphemeris < 7200); }
private:
	virtual inline constexpr double GM()const noexcept override { return 3.986005E14; }
	virtual inline constexpr double Omega_e() const noexcept override { return 7.2921151467E-5; }
	virtual inline constexpr double F() const noexcept override { return -4.442807633E-10; }
};
