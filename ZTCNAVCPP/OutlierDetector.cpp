#include "OutlierDetector.h"
using namespace std;


std::map<Satellite, SatelliteObservation> OutlierDetector::Filter(const GpsTime& time, const std::map<Satellite, SatelliteObservation>& observations)
{
	//判断观测连续性
	if (time - _lastTime >= (_interval + DBL_EPSILON))
	{
		_lastTime = time;
		return{};
	}
	map<Satellite, SatelliteObservation> res;
	for (auto& [sat, obs] : observations)
	{
		//双频
		if (obs.Data.size() > 1)
		{
			if (!IsOutlier(sat, obs))
			{
				res[sat] = obs;
			}
		}
	}
	//未出现的卫星连续观测数置0
	for (auto& [sat, count] : _continuousCountWithoutOutlierOf)
	{
		if (!observations.contains(sat))
		{
			count = 0;
		}
	}
	_lastTime = time;
	return res;
}

bool OutlierDetector::IsOutlier(const Satellite& satellite, const SatelliteObservation& satelliteObservation)
{
	auto& ts { SignalTypesOfSatelliteSystems.at(satellite.System) };
	auto& t1 { ts[0] }, & t2 { ts[1] };
	//auto& obs { satelliteObservation[satellite] };
	auto& count = _continuousCountWithoutOutlierOf[satellite];
	auto mw { (satelliteObservation.Melbourne_Wübbena(t1, t2) + count * _mwRecordOf[satellite]) / (count + 1) }, gf { satelliteObservation.GFOfCarrierPhase(t1,t2) };
	bool isOutlier;
	if (count != 0)
	{
		//非首个历元，则直接判断
		isOutlier = abs(mw - _mwRecordOf[satellite]) > _dMWLimit || abs(gf - _gfRecordOf[satellite]) > _dGFLimit;
		//若为粗差，则连续观测数置0
		if (isOutlier)
		{
			count = 0;
		}
		else
		{
			count++;
		}
	}
	else
	{
		//若为首个历元，无法确定而跳过判断，但连续观测历元置1
		isOutlier = true;
		count = 1;
	}
	_mwRecordOf[satellite] = mw;
	_gfRecordOf[satellite] = gf;
	return isOutlier;
}
