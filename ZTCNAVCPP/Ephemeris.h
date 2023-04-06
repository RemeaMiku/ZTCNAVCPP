#pragma once
#include "GpsTime.h"
#include "Satellite.h"
#include "CartCoordinate.h"
#include "Vector.h"

class Ephemeris
{
public:
	/// <summary>
	/// �����ο�ʱ��
	/// </summary>
	GpsTime ReferenceTimeForEphemeris;
	/// <summary>
	/// �Ӳ���������ο�ʱ��
	/// </summary>
	GpsTime ReferenceTimeForSatelliteClockCorrction;
	Satellite SatelliteOfEphemeris;
	unsigned long HealthStatus;
	double Semi_majorAxis;
	/// <summary>
	/// ƽ�����ٶȸ���ֵ(rad/s)
	/// </summary>
	double MeanAngularVelocityDifference;
	/// <summary>
	/// �����ο�ʱ�̵�ƽ�����(rad)
	/// </summary>
	double MeanAnomalyOfReferenceTime;
	/// <summary>
	/// ������(1)
	/// </summary>
	double Eccentricity;
	/// <summary>
	/// ���ص�Ǿ�(rad)
	/// </summary>
	double ArgumentOfPerigee;
	/// <summary>
	/// ������ǵ����ҵ��͸���������(rad)
	/// </summary>
	double Cos_CorrectionArgumentOfLatitude;
	/// <summary>
	/// ������ǵ����ҵ��͸���������(rad)
	/// </summary>
	double Sin_CorrectionArgumentOfLatitude;
	/// <summary>
	/// ����뾶�����ҵ��͸���������(m)
	/// </summary>
	double Cos_CorrectionOrbitRadius;
	/// <summary>
	/// ����뾶�����ҵ��͸���������(m)
	/// </summary>
	double Sin_CorrectionOrbitRadius;
	/// <summary>
	/// �����ǵ����ҵ��͸���������(rad)
	/// </summary>
	double Cos_CorrectionAngleOfInclination;
	/// <summary>
	/// �����ǵ����ҵ��͸���������(rad)
	/// </summary>
	double Sin_CorrectionAngleOfInclination;
	/// <summary>
	/// �����ο�ʱ�̵Ĺ�����(rad)
	/// </summary>
	double InclinationAngleAtReferenceTime;
	/// <summary>
	/// �����ǵı仯��(rad/s)
	/// </summary>
	double RateOfInclinationAngle;
	/// <summary>
	/// ������ྭ(rad)
	/// </summary>
	double RightAscension;
	/// <summary>
	/// ������ྭ�仯��(rad/s)
	/// </summary>
	double RateOfRightAscension;
	/// <summary>
	/// �Ӳ�(s)
	/// </summary>
	double ClockJitter;
	/// <summary>
	/// ��Ư(s/s)
	/// </summary>
	double ClockWander;
	/// <summary>
	/// ��Ư��(s/s^2)
	/// </summary>
	double ClockWanderSpeed;
	/// <summary>
	/// ��ȡ����������
	/// </summary>
	/// <param name="t">�źŷ���ʱ�̵�Gpsʱ</param>
	/// <returns></returns>
	virtual inline bool IsAvailable(const GpsTime& t) const = 0;
	/// <summary>
	/// ����Gpsʱ������ECEF����
	/// </summary>
	/// <param name="t">�źŷ���ʱ�̵�Gpsʱ</param>
	/// <returns></returns>
	virtual CartCoordinate Position(const GpsTime& t) const;
	/// <summary>
	/// ����Gpsʱ�������Ӳ�
	/// </summary>
	/// <param name="t"></param>
	/// <returns></returns>
	virtual double ClockError(const GpsTime& t) const;
	/// <summary>
	/// ����Gpsʱ�������ٶ�
	/// </summary>
	/// <param name="t"></param>
	/// <returns></returns>
	virtual Vector Velocity(const GpsTime& t) const;
	/// <summary>
	/// ����Gpsʱ����������
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