#pragma once
#include "OEMMessageHeader.h"
#include "GeoCoordinate.h"

class OEMBESTPOS
{
public:
	OEMMessageHeader Header;

	/// <summary>
	/// ��λ����(rad,rad,m)
	/// </summary>
	GeoCoordinate Position;

	/// <summary>
	/// γ�ȱ�׼��(m)
	/// </summary>
	float LatitudeStandardDeviation;

	/// <summary>
	/// ���ȱ�׼��(m)
	/// </summary>
	float LongitudeStandardDeviation;

	/// <summary>
	/// �߶ȱ�׼��(m)
	/// </summary>
	float HeightStandardDeviation;
	char BaseStationId[4];
	unsigned char NumbersOfSatelliteTracked;
	unsigned char NumberOfSatelliteUsedInSolution;
	OEMBESTPOS() = default;
	OEMBESTPOS(const OEMMessageHeader& hd, double lat, double lon, double hgt);
	std::string ToString() const noexcept;
};