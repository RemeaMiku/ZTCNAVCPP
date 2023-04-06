#include "OutlierDetector.h"
using namespace std;


std::map<Satellite, SatelliteObservation> OutlierDetector::Filter(const GpsTime& time, const std::map<Satellite, SatelliteObservation>& observations)
{
	//�жϹ۲�������
	if (time - _lastTime >= (_interval + DBL_EPSILON))
	{
		_lastTime = time;
		return{};
	}
	map<Satellite, SatelliteObservation> res;
	for (auto& [sat, obs] : observations)
	{
		//˫Ƶ
		if (obs.Data.size() > 1)
		{
			if (!IsOutlier(sat, obs))
			{
				res[sat] = obs;
			}
		}
	}
	//δ���ֵ����������۲�����0
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
	auto mw { (satelliteObservation.Melbourne_W��bbena(t1, t2) + count * _mwRecordOf[satellite]) / (count + 1) }, gf { satelliteObservation.GFOfCarrierPhase(t1,t2) };
	bool isOutlier;
	if (count != 0)
	{
		//���׸���Ԫ����ֱ���ж�
		isOutlier = abs(mw - _mwRecordOf[satellite]) > _dMWLimit || abs(gf - _gfRecordOf[satellite]) > _dGFLimit;
		//��Ϊ�ֲ�������۲�����0
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
		//��Ϊ�׸���Ԫ���޷�ȷ���������жϣ��������۲���Ԫ��1
		isOutlier = true;
		count = 1;
	}
	_mwRecordOf[satellite] = mw;
	_gfRecordOf[satellite] = gf;
	return isOutlier;
}
