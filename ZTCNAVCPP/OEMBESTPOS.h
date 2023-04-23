#pragma once
#include "OEMMessageHeader.h"
#include "GeoCoordinate.h"

class OEMBESTPOS
{
public:
	OEMMessageHeader Header;

	/// <summary>
	/// 定位坐标(rad,rad,m)
	/// </summary>
	GeoCoordinate Position;

	/// <summary>
	/// 纬度标准差(m)
	/// </summary>
	float LatitudeStandardDeviation;

	/// <summary>
	/// 经度标准差(m)
	/// </summary>
	float LongitudeStandardDeviation;

	/// <summary>
	/// 高度标准差(m)
	/// </summary>
	float HeightStandardDeviation;
	char BaseStationId[4];
	unsigned char NumbersOfSatelliteTracked;
	unsigned char NumberOfSatelliteUsedInSolution;
	OEMBESTPOS() = default;
	OEMBESTPOS(const OEMMessageHeader& hd, double lat, double lon, double hgt);
	std::string ToString() const noexcept;
};