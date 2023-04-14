#pragma once
#include <map>
#include <vector>
#include "Satellite.h"
#include "GpsTime.h"

struct ObservationData
{
public:
	/// <summary>
	/// 伪距观测值(m)
	/// </summary>
	double		Pseudorange;
	/// <summary>
	/// 伪距标准差(m)
	/// </summary>
	float		PseudorangeStandardDeviation;
	/// <summary>
	/// 载波相位观测值(m)
	/// </summary>
	double		CarrierPhase;
	/// <summary>
	/// 载波相位标准差(m)
	/// </summary>
	float		CarrierPhaseStandardDeviation;
	/// <summary>
	/// 多普勒频移(Hz)
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
	/// 返回伪距IF组合
	/// </summary>
	/// <param name="t1">信号类型1</param>
	/// <param name="t2">信号类型2</param>
	/// <returns></returns>
	double IFOfPseudorange(const SignalType& t1, const SignalType& t2) const;
	/// <summary>
	/// 返回MW组合
	/// </summary>
	/// <param name="t1">信号类型1</param>
	/// <param name="t2">信号类型2</param>
	/// <returns></returns>
	double Melbourne_Wübbena(const SignalType& t1, const SignalType& t2) const;
	/// <summary>
	/// 返回相位GF组合
	/// </summary>
	/// <param name="t1">信号类型1</param>
	/// <param name="t2">信号类型2</param>
	/// <returns></returns>
	double GFOfCarrierPhase(const SignalType& t1, const SignalType& t2) const;
	/// <summary>
	/// 返回OEM中对应的信号类型
	/// </summary>
	/// <param name="sys"></param>
	/// <param name="type"></param>
	/// <returns></returns>
	static SignalType GetSignalType(const SatelliteSystem& sys, int type);
	/// <summary>
	/// 周计数相位观测值转换为米制
	/// </summary>
	/// <param name="phi">周数</param>
	/// <param name="f">频率</param>
	/// <returns>米制相位</returns>
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