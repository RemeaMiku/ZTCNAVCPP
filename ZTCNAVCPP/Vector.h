#pragma once
#include <vector>
#include <format>
/// <summary>
/// 向量类
/// 最后更新：2022.10.17
/// © 2022 RemeaMiku 版权所有
/// </summary>
class Vector
{
private:
	/// <summary>
	/// 向量元素
	/// </summary>
	std::vector<double> _elements;
	int Dimension;
public:
	Vector();
	explicit Vector(int n);
	explicit Vector(const std::vector<double>& vector);
	/// <summary>
	/// 获取向量元素个数
	/// </summary>
	inline constexpr int Count() const noexcept { return Dimension; }
	/// <summary>
	/// 创建索引
	/// </summary>
	/// <param name="i"></param>
	/// <returns></returns>
	inline constexpr double& operator [](int i) noexcept { return _elements[i]; }
	inline constexpr const double& operator [](int i) const noexcept { return _elements[i]; }

	inline constexpr double& operator()(int i)
	{
		if (i >= 0 && i < Dimension)
		{
			return _elements[i];
		}
		else
		{
			throw "下标超限";
		}
	}

	inline const constexpr double& operator()(int i) const { return _elements.at(i); }

	/// <summary>
	/// 向量点乘
	/// </summary>
	/// <param name="vector1"></param>
	/// <param name="vector2"></param>
	/// <returns></returns>
	double operator *(const Vector& vector2) const;
	/// <summary>
	/// 向量与数相乘
	/// </summary>
	/// <param name="vector"></param>
	/// <param name="num"></param>
	/// <returns></returns>
	Vector operator *(double num) const;
	friend Vector operator *(double num, const Vector& vector2);
	/// <summary>
	/// 计算两向量的叉积
	/// </summary>
	/// <param name="vec1"></param>
	/// <param name="vec2"></param>
	/// <returns></returns>
	Vector CrossProduct(const Vector& vec2);
	/// <summary>
	/// 向量与数相除
	/// </summary>
	/// <param name="vector"></param>
	/// <param name="num"></param>
	/// <returns></returns>
	Vector operator / (double num) const;
	/// <summary>
	/// 向量相加
	/// </summary>
	/// <param name="vector1"></param>
	/// <param name="vector2"></param>
	/// <returns></returns>
	Vector operator + (const Vector& vector2) const;
	/// <summary>
	/// 向量取负
	/// </summary>
	/// <returns></returns>
	const Vector operator -() const;
	/// <summary>
	/// 向量相减
	/// </summary>
	/// <param name="vector1"></param>
	/// <param name="vector2"></param>
	/// <returns></returns>
	Vector operator - (const Vector& vector2) const;
	/// <summary>
	/// 获取向量的模
	/// </summary>
	/// <returns></returns>
	double Norm() const noexcept;
	/// <summary>
	/// 获取单位向量
	/// </summary>
	/// <returns></returns>
	Vector Unitization() const noexcept;
	/// <summary>
	/// 转换为数组
	/// </summary>
	/// <returns></returns>
	inline constexpr std::vector<double> ToArray() const noexcept { return _elements; }
	std::string ToString() const noexcept;
};
