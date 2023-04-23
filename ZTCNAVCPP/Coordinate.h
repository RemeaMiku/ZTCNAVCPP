#pragma once
#include <vector>
#include <format>

/// <summary>
/// 坐标类
/// </summary>
class Coordinate
{
protected:

	/// <summary>
	/// 坐标值
	/// </summary>
	std::vector<double> _elements;

	/// <summary>
	/// 构造函数，不可被外部实例化
	/// </summary>
	Coordinate() noexcept;
	Coordinate(double a, double b, double c) noexcept;
	Coordinate(const std::vector<double>& vec);
public:

	/// <summary>
	/// 下标引用
	/// </summary>
	/// <param name="i">下标，i=[0,2]</param>
	/// <returns>对应坐标分量引用</returns>
	inline constexpr double& operator [](int i)
	{
		return _elements[i];
	}

	/// <summary>
	/// 下标常量引用
	/// </summary>
	/// <param name="i">下标，i=[0,2]</param>
	/// <returns>对应坐标分量常量</returns>
	inline const double& operator[](int i) const
	{
		return _elements[i];
	}

	inline constexpr double& operator()(int i)
	{
		if (i < 0 || i >= 3)
			throw "下标超限";
		return _elements[i];
	}

	inline const double& operator()(int i) const
	{
		return _elements.at(i);
	}

	inline constexpr std::vector<double> ToArray() const noexcept
	{
		return _elements;
	}

	virtual std::string ToString() const noexcept = 0;
};
