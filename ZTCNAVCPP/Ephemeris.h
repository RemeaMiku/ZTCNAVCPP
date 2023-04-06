#pragma once
#include "GpsTime.h"
#include "Satellite.h"
#include "CartCoordinate.h"
#include "Vector.h"

class Ephemeris
{
public:
	/// <summary>
	/// 星历参考时间
	/// </summary>
	GpsTime ReferenceTimeForEphemeris;
	/// <summary>
	/// 钟差改正参数参考时间
	/// </summary>
	GpsTime ReferenceTimeForSatelliteClockCorrction;
	Satellite SatelliteOfEphemeris;
	unsigned long HealthStatus;
	double Semi_majorAxis;
	/// <summary>
	/// 平均角速度改正值(rad/s)
	/// </summary>
	double MeanAngularVelocityDifference;
	/// <summary>
	/// 星历参考时刻的平近点角(rad)
	/// </summary>
	double MeanAnomalyOfReferenceTime;
	/// <summary>
	/// 离心率(1)
	/// </summary>
	double Eccentricity;
	/// <summary>
	/// 近地点角距(rad)
	/// </summary>
	double ArgumentOfPerigee;
	/// <summary>
	/// 升交距角的余弦调和改正项的振幅(rad)
	/// </summary>
	double Cos_CorrectionArgumentOfLatitude;
	/// <summary>
	/// 升交距角的正弦调和改正项的振幅(rad)
	/// </summary>
	double Sin_CorrectionArgumentOfLatitude;
	/// <summary>
	/// 轨道半径的余弦调和改正项的振幅(m)
	/// </summary>
	double Cos_CorrectionOrbitRadius;
	/// <summary>
	/// 轨道半径的正弦调和改正项的振幅(m)
	/// </summary>
	double Sin_CorrectionOrbitRadius;
	/// <summary>
	/// 轨道倾角的余弦调和改正项的振幅(rad)
	/// </summary>
	double Cos_CorrectionAngleOfInclination;
	/// <summary>
	/// 轨道倾角的正弦调和改正项的振幅(rad)
	/// </summary>
	double Sin_CorrectionAngleOfInclination;
	/// <summary>
	/// 星历参考时刻的轨道倾角(rad)
	/// </summary>
	double InclinationAngleAtReferenceTime;
	/// <summary>
	/// 轨道倾角的变化率(rad/s)
	/// </summary>
	double RateOfInclinationAngle;
	/// <summary>
	/// 升交点赤经(rad)
	/// </summary>
	double RightAscension;
	/// <summary>
	/// 升交点赤经变化率(rad/s)
	/// </summary>
	double RateOfRightAscension;
	/// <summary>
	/// 钟差(s)
	/// </summary>
	double ClockJitter;
	/// <summary>
	/// 钟漂(s/s)
	/// </summary>
	double ClockWander;
	/// <summary>
	/// 钟漂率(s/s^2)
	/// </summary>
	double ClockWanderSpeed;
	/// <summary>
	/// 获取星历可用性
	/// </summary>
	/// <param name="t">信号发射时刻的Gps时</param>
	/// <returns></returns>
	virtual inline bool IsAvailable(const GpsTime& t) const = 0;
	/// <summary>
	/// 返回Gps时的卫星ECEF坐标
	/// </summary>
	/// <param name="t">信号发射时刻的Gps时</param>
	/// <returns></returns>
	virtual CartCoordinate Position(const GpsTime& t) const;
	/// <summary>
	/// 返回Gps时的卫星钟差
	/// </summary>
	/// <param name="t"></param>
	/// <returns></returns>
	virtual double ClockError(const GpsTime& t) const;
	/// <summary>
	/// 返回Gps时的卫星速度
	/// </summary>
	/// <param name="t"></param>
	/// <returns></returns>
	virtual Vector Velocity(const GpsTime& t) const;
	/// <summary>
	/// 返回Gps时的卫星钟速
	/// </summary>
	/// <param name="t"></param>
	/// <returns></returns>
	virtual double ClockSpeedError(const GpsTime& t) const;	
	virtual inline constexpr double GM() const noexcept = 0;
	virtual inline constexpr double Omega_e() const noexcept = 0;
	virtual inline constexpr double F() const noexcept = 0;
protected:
	Ephemeris() = default;
	double GetE_k(const double& M_k) const;
};