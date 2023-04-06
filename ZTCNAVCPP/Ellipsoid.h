#pragma once
#include <cmath>
#include <map>
#include "Satellite.h"
/// <summary>
/// ������
/// </summary>
class Ellipsoid
{
public:
	/// <summary>
	/// ������
	/// </summary>
	double a;
	/// <summary>
	/// �̰���
	/// </summary>
	double b;
	/// <summary>
	/// �����ʰ뾶
	/// </summary>
	double c;
	/// <summary>
	/// ����
	/// </summary>
	double alpha;
	/// <summary>
	/// ��һƫ����
	/// </summary>
	double e1;
	/// <summary>
	/// �ڶ�ƫ����
	/// </summary>
	double e2;
	Ellipsoid() = default;
	Ellipsoid(double x, double y) noexcept;
	/// <summary>
	/// ��һ����γ�Ⱥ���
	/// </summary>
	/// <param name="B">���γ��</param>
	/// <returns></returns>
	inline double W(double B) const noexcept
	{
		return std::sqrt(1 - std::pow(e1 * std::sin(B), 2));
	}
	/// <summary>
	/// �ڶ�����γ�Ⱥ���
	/// </summary>
	/// <param name="B">���γ��</param>
	/// <returns></returns>
	inline double V(double B) const noexcept
	{
		return std::sqrt(1 + std::pow(e2 * std::cos(B), 2));
	}
	/// <summary>
	/// î��Ȧ���ʰ뾶
	/// </summary>
	/// <param name="B">���γ��</param>
	/// <returns></returns>
	inline double N(double B) const noexcept
	{
		return a / W(B);
	}
	/// <summary>
	/// ����ϵͳ��Ӧ������
	/// </summary>
	static const std::map<SatelliteSystem, Ellipsoid> Ellipsoids;
};
inline static const Ellipsoid WGS_84 { 6378137, 6356752.3142451795 };
inline static const Ellipsoid CGCS2000 { 6378137, 6356752.31414 };