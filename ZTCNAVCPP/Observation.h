#pragma once
#include <map>
#include <vector>
#include "Satellite.h"
#include "GpsTime.h"

struct ObservationData
{
public:
	/// <summary>
	/// α��۲�ֵ(m)
	/// </summary>
	double		Pseudorange;
	/// <summary>
	/// α���׼��(m)
	/// </summary>
	float		PseudorangeStandardDeviation;
	/// <summary>
	/// �ز���λ�۲�ֵ(m)
	/// </summary>
	double		CarrierPhase;
	/// <summary>
	/// �ز���λ��׼��(m)
	/// </summary>
	float		CarrierPhaseStandardDeviation;
	/// <summary>
	/// ������Ƶ��(Hz)
	/// </summary>
	float		InstantaneousCarrierDopplerFrequency;
	float C_No;
	bool ParityFlag;
};

enum class SignalType
{
	L1CA,
	L2PY,
	B1I,
	B2I,
	B3I,
	OTHERS = -1
};

extern const std::map<SatelliteSystem, std::map<int, SignalType>> SignalTypesOfOEM;
extern const std::map<SatelliteSystem, std::vector<SignalType>> SignalTypesOfSatelliteSystems;
extern const std::map<SignalType, double> FrequencyOfSignalType;
extern inline constexpr double LightSpeed { 2.99792458E8 };

class SatelliteObservation
{
public:
	SatelliteObservation() = default;
	/// <summary>
	/// ����α��IF���
	/// </summary>
	/// <param name="t1">�ź�����1</param>
	/// <param name="t2">�ź�����2</param>
	/// <returns></returns>
	double IFOfPseudorange(const SignalType& t1, const SignalType& t2) const;
	/// <summary>
	/// ����MW���
	/// </summary>
	/// <param name="t1">�ź�����1</param>
	/// <param name="t2">�ź�����2</param>
	/// <returns></returns>
	double Melbourne_W��bbena(const SignalType& t1, const SignalType& t2) const;
	/// <summary>
	/// ������λGF���
	/// </summary>
	/// <param name="t1">�ź�����1</param>
	/// <param name="t2">�ź�����2</param>
	/// <returns></returns>
	double GFOfCarrierPhase(const SignalType& t1, const SignalType& t2) const;
	/// <summary>
	/// ����OEM�ж�Ӧ���ź�����
	/// </summary>
	/// <param name="sys"></param>
	/// <param name="type"></param>
	/// <returns></returns>
	static SignalType GetSignalType(const SatelliteSystem& sys, int type);
	/// <summary>
	/// �ܼ�����λ�۲�ֵת��Ϊ����
	/// </summary>
	/// <param name="phi">����</param>
	/// <param name="f">Ƶ��</param>
	/// <returns>������λ</returns>
	static constexpr inline double ToPhaseMeter(double phi, double f) noexcept
	{
		return phi * LightSpeed / f;
	}
	inline static const double k13 { pow(1561.098E6 / 1268.52E6,2) };
	std::map<SignalType, ObservationData> Data;
};

//class SingleDifferenceObservation
//{
//public:
//	GpsTime Time;
//	std::map<Satellite, SatelliteObservation> Observations;
//	SingleDifferenceObservation(const GpsTime& time, const std::map<Satellite, SatelliteObservation>& rovObservations, const std::map<Satellite, SatelliteObservation>& baseObservations)
//	{
//		Time = time;
//		for (auto& [satellite, baseObs] : baseObservations)
//		{
//			if (!rovObservations.contains(satellite))
//				continue;
//			auto& rovObs = rovObservations.at(satellite);
//			/*if (rovObs.Data.size() < 2)
//				continue;*/
//			auto satObservation = SatelliteObservation {};
//			auto& signalTypes = SignalTypesOfSatelliteSystems.at(satellite.System);
//			for (auto& signalType : signalTypes)
//			{
//				auto& rovData = rovObs.Data.at(signalType);
//				auto& baseData = baseObs.Data.at(signalType);
//				satObservation.Data[signalType] = ObservationData
//				{
//					rovData.Pseudorange - baseData.Pseudorange,
//					0,
//					rovData.CarrierPhase - baseData.CarrierPhase,
//					0,
//					0
//				};
//			}
//			Observations[satellite] = satObservation;
//		}
//	}
//};