#pragma once
#include<map>
#include"OEMRANGE.h"
#include"GpsTime.h"

class OutlierDetector
{
public:
	std::map <Satellite, SatelliteObservation> Filter(const GpsTime& time, const std::map<Satellite, SatelliteObservation>& observations);
private:
	bool IsOutlier(const Satellite& satellite, const SatelliteObservation& satelliteObservation);
	std::map<Satellite, int> _continuousCountWithoutOutlierOf;
	std::map<Satellite, double> _gfRecordOf;
	std::map<Satellite, double> _mwRecordOf;
	GpsTime _lastTime {};
	std::map <Satellite, SatelliteObservation> _lastRes {};
	inline static constexpr double _interval { 1 };
	inline static constexpr double _dGFLimit { 0.05 };
	inline static constexpr double _dMWLimit { 2.0 };
};

