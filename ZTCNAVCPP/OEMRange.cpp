#include "OEMRange.h"

using namespace std;

//map<Satellite, int> OEMRANGE::_continuousCountWithoutOutlierOf {};
//map<Satellite, double> OEMRANGE::_gfRecordOf {};
//map<Satellite, double> OEMRANGE::_mwRecordOf {};

OEMRANGE::OEMRANGE(const OEMMessageHeader& hd, unsigned long n) :Header(hd), ObservationNumber(n)
{}

//std::map<Satellite, SatelliteObservation> OEMRANGE::Filter()
//{
//	//�жϹ۲�������
//	if (Header.Time - _lastTime < (_interval + DBL_EPSILON))
//	{
//		map<Satellite, SatelliteObservation> res;
//		for (auto& [sat, obs] : ObservationOf)
//		{
//			//˫Ƶ
//			if (obs.Data.size() > 1)
//			{
//				if (!DetectOutlier(sat))
//				{
//					res[sat] = obs;
//				}
//			}
//		}
//		//δ���ֵ����������۲�����0
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
//	auto mw { (obs.Melbourne_W��bbena(t1, t2) + count * _mwRecordOf[sat]) / (count + 1) }, gf { obs.GFOfCarrierPhase(t1,t2) };
//	bool isOutlier;
//	if (count != 0)
//	{
//		//���׸���Ԫ����ֱ���ж�
//		isOutlier = abs(mw - _mwRecordOf[sat]) > _dMWLimit || abs(gf - _gfRecordOf[sat]) > _dGFLimit;
//		//��Ϊ�ֲ�������۲�����0
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
//		//��Ϊ�׸���Ԫ���޷�ȷ���������жϣ��������۲���Ԫ��1
//		isOutlier = true;
//		count = 1;
//	}
//	_mwRecordOf[sat] = mw;
//	_gfRecordOf[sat] = gf;
//	return isOutlier;
//}