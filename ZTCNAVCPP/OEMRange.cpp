#include "OEMRange.h"

using namespace std;

//map<Satellite, int> OEMRANGE::_continuousCountWithoutOutlierOf {};
//map<Satellite, double> OEMRANGE::_gfRecordOf {};
//map<Satellite, double> OEMRANGE::_mwRecordOf {};

OEMRANGE::OEMRANGE(const OEMMessageHeader& hd, unsigned long n) :Header(hd), ObservationNumber(n)
{}

//std::map<Satellite, SatelliteObservation> OEMRANGE::Filter()
//{
//	//判断观测连续性
//	if (Header.Time - _lastTime < (_interval + DBL_EPSILON))
//	{
//		map<Satellite, SatelliteObservation> res;
//		for (auto& [sat, obs] : ObservationOf)
//		{
//			//双频
//			if (obs.Data.size() > 1)
//			{
//				if (!DetectOutlier(sat))
//				{
//					res[sat] = obs;
//				}
//			}
//		}
//		//未出现的卫星连续观测数置0
//		for (auto& [sat, count] : _continuousCountWithoutOutlierOf)
//		{
//			if (!ObservationOf.contains(sat))
//			{
//				count = 0;
//			}
//		}
//		_lastTime = Header.Time;
//		return res;
//	}
//	else
//	{
//		_lastTime = Header.Time;
//		return {};
//	}
//}
//
//bool OEMRANGE::DetectOutlier(const Satellite& sat)
//{
//	auto& ts { SignalTypesOfSatelliteSystems.at(sat.System) };
//	auto& t1 { ts[0] }, & t2 { ts[1] };
//	auto& obs { ObservationOf[sat] };
//	auto& count = _continuousCountWithoutOutlierOf[sat];
//	auto mw { (obs.Melbourne_Wübbena(t1, t2) + count * _mwRecordOf[sat]) / (count + 1) }, gf { obs.GFOfCarrierPhase(t1,t2) };
//	bool isOutlier;
//	if (count != 0)
//	{
//		//非首个历元，则直接判断
//		isOutlier = abs(mw - _mwRecordOf[sat]) > _dMWLimit || abs(gf - _gfRecordOf[sat]) > _dGFLimit;
//		//若为粗差，则连续观测数置0
//		if (isOutlier)
//		{
//			count = 0;
//		}
//		else
//		{
//			count++;
//		}
//	}
//	else
//	{
//		//若为首个历元，无法确定而跳过判断，但连续观测历元置1
//		isOutlier = true;
//		count = 1;
//	}
//	_mwRecordOf[sat] = mw;
//	_gfRecordOf[sat] = gf;
//	return isOutlier;
//}