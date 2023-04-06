#pragma once
#include <format>

inline constexpr auto PI{ 3.141592653589793 };

namespace Angle
{
	/// <summary>
	/// 角度制转弧度制
	/// </summary>
	/// <param name="degree"></param>
	/// <returns></returns>
	template<std::floating_point Float>
	inline constexpr Float ToRad(Float degree) noexcept
	{
		return degree * static_cast<Float>(PI) / 180;		
	}
	/// <summary>
	/// 度分秒制转角度制
	/// </summary>
	/// <param name="d"></param>
	/// <param name="m"></param>
	/// <param name="s"></param>
	/// <returns></returns>
	template<std::floating_point Float>
	inline constexpr Float ToDegree(int d, unsigned int m, Float s) noexcept
	{
		return s / static_cast<Float>(3600) + m / static_cast<Float>(60) + d;
	}

	/// <summary>
	/// 弧度制转角度制
	/// </summary>
	/// <param name="rad">输入的角度弧度制</param>
	/// <returns>角度制</returns>
	template<std::floating_point Float>
	inline constexpr Float ToDegree(Float rad) noexcept
	{
		return rad * 180 / static_cast<Float>(PI);
	}
	/// <summary>
	/// 角度制转度分秒制
	/// </summary>
	/// <param name="degree"></param>
	/// <param name="d"></param>
	/// <param name="m"></param>
	/// <param name="s"></param>
	template<std::floating_point Float>
	inline constexpr void ToDegree(Float degree, int d, unsigned int m, Float s) noexcept
	{
		d = static_cast<int>(degree);
		Float minute{ abs((degree - d)) * 60 };
		m = static_cast<unsigned int>(minute);
		s = static_cast<Float>(minute - m) * 60;
	}

	/// <summary>
	/// 格式化构造度分秒字符串
	/// 保留4位小数
	/// </summary>
	/// <param name="d">角度的度值</param>
	/// <param name="m">角度的分值</param>
	/// <param name="s">角度的秒值</param>
	/// <returns>度分秒格式字符串</returns>
	template<std::floating_point Float>
	inline constexpr std::string ToString(int d, unsigned int m, Float s) noexcept
	{		
		return std::format("{}° {}′ {:.4f}″", d, m, s);
	}
};