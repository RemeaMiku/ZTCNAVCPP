#pragma once
#include "OEMMessageHeader.h"
#include "Observation.h"
#include "Satellite.h"
#include <map>

//!周跳探测修正
class OEMRANGE
{
public:
	OEMMessageHeader Header;

	/// <summary>
	/// 观测数
	/// </summary>
	unsigned long ObservationNumber;
	OEMRANGE() = default;
	OEMRANGE(const OEMMessageHeader& hd, unsigned long n);

	/// <summary>
	/// 卫星观测值数据
	/// </summary>
	std::map<Satellite, SatelliteObservation> ObservationOf;

	//std::map<Satellite, SatelliteObservation> Filter();
private:
	/*/// <summary>

	/// 粗差探测
	/// </summary>
	/// <param name="sat"></param>
	/// <returns></returns>
	bool DetectOutlier(const Satellite& sat);
	static std::map<Satellite, int> _continuousCountWithoutOutlierOf;
	static std::map<Satellite, double> _gfRecordOf;
	static std::map<Satellite, double> _mwRecordOf;
	inline static GpsTime _lastTime {};
	inline static constexpr double _interval { 1 };
	inline static constexpr double _dGFLimit { 0.05 };
	inline static constexpr double _dMWLimit { 2.0 };*/
};