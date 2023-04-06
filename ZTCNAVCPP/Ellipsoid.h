#pragma once
#include <cmath>
#include <map>
#include "Satellite.h"
/// <summary>
/// 椭球类
/// </summary>
class Ellipsoid
{
public:
	/// <summary>
	/// 长半轴
	/// </summary>
	double a;
	/// <summary>
	/// 短半轴
	/// </summary>
	double b;
	/// <summary>
	/// 极曲率半径
	/// </summary>
	double c;
	/// <summary>
	/// 扁率
	/// </summary>
	double alpha;
	/// <summary>
	/// 第一偏心率
	/// </summary>
	double e1;
	/// <summary>
	/// 第二偏心率
	/// </summary>
	double e2;
	Ellipsoid() = default;
	Ellipsoid(double x, double y) noexcept;
	/// <summary>
	/// 第一基本纬度函数
	/// </summary>
	/// <param name="B">大地纬度</param>
	/// <returns></returns>
	inline double W(double B) const noexcept
	{
		return std::sqrt(1 - std::pow(e1 * std::sin(B), 2));
	}
	/// <summary>
	/// 第二基本纬度函数
	/// </summary>
	/// <param name="B">大地纬度</param>
	/// <returns></returns>
	inline double V(double B) const noexcept
	{
		return std::sqrt(1 + std::pow(e2 * std::cos(B), 2));
	}
	/// <summary>
	/// 卯酉圈曲率半径
	/// </summary>
	/// <param name="B">大地纬度</param>
	/// <returns></returns>
	inline double N(double B) const noexcept
	{
		return a / W(B);
	}
	/// <summary>
	/// 返回系统对应的椭球
	/// </summary>
	static const std::map<SatelliteSystem, Ellipsoid> Ellipsoids;
};
inline static const Ellipsoid WGS_84 { 6378137, 6356752.3142451795 };
inline static const Ellipsoid CGCS2000 { 6378137, 6356752.31414 };