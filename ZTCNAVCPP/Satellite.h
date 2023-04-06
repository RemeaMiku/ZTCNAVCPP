#pragma once
#include <format>
#include <map>

/// <summary>
/// Œ¿–«œµÕ≥
/// GPS=0
/// BDS=4
/// </summary>
enum SatelliteSystem
{
	GPS,
	GLONASS,
	SBAS,
	Galileo,
	BDS,
	QZSS,
	NavIC,
	Other
};

class Satellite
{
public:
	unsigned short Id;
	SatelliteSystem System;
	Satellite() = default;
	Satellite(unsigned short prn, SatelliteSystem sys);
	Satellite(const std::string& prn);
	inline bool IsGEO() const noexcept
	{
		return System == BDS && !(Id > 5 && Id < 59);
	}
	inline bool operator <(const Satellite& sat) const noexcept
	{
		return System != sat.System ? System < sat.System : Id < sat.Id;
	}
	inline bool operator ==(const Satellite& sat) const noexcept
	{
		return System == sat.System && Id == sat.Id;
	}
	std::string ToString() const;
	static const std::map<char, SatelliteSystem> SystemNameOfCode;
	static const std::map<SatelliteSystem, char> SystemCodeOfName;
};