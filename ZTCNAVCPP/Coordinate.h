#pragma once
#include <vector>
#include <format>

/// <summary>
/// ������
/// </summary>
class Coordinate
{
protected:

	/// <summary>
	/// ����ֵ
	/// </summary>
	std::vector<double> _elements;

	/// <summary>
	/// ���캯�������ɱ��ⲿʵ����
	/// </summary>
	Coordinate() noexcept;
	Coordinate(double a, double b, double c) noexcept;
	Coordinate(const std::vector<double>& vec);
public:

	/// <summary>
	/// �±�����
	/// </summary>
	/// <param name="i">�±꣬i=[0,2]</param>
	/// <returns>��Ӧ�����������</returns>
	inline constexpr double& operator [](int i)
	{
		return _elements[i];
	}

	/// <summary>
	/// �±곣������
	/// </summary>
	/// <param name="i">�±꣬i=[0,2]</param>
	/// <returns>��Ӧ�����������</returns>
	inline const double& operator[](int i) const
	{
		return _elements[i];
	}

	inline constexpr double& operator()(int i)
	{
		if (i < 0 || i >= 3)
			throw "�±곬��";
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
