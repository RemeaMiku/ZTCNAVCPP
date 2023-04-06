#pragma once
#include <format>

inline constexpr auto PI{ 3.141592653589793 };

namespace Angle
{
	/// <summary>
	/// �Ƕ���ת������
	/// </summary>
	/// <param name="degree"></param>
	/// <returns></returns>
	template<std::floating_point Float>
	inline constexpr Float ToRad(Float degree) noexcept
	{
		return degree * static_cast<Float>(PI) / 180;		
	}
	/// <summary>
	/// �ȷ�����ת�Ƕ���
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
	/// ������ת�Ƕ���
	/// </summary>
	/// <param name="rad">����ĽǶȻ�����</param>
	/// <returns>�Ƕ���</returns>
	template<std::floating_point Float>
	inline constexpr Float ToDegree(Float rad) noexcept
	{
		return rad * 180 / static_cast<Float>(PI);
	}
	/// <summary>
	/// �Ƕ���ת�ȷ�����
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
	/// ��ʽ������ȷ����ַ���
	/// ����4λС��
	/// </summary>
	/// <param name="d">�ǶȵĶ�ֵ</param>
	/// <param name="m">�Ƕȵķ�ֵ</param>
	/// <param name="s">�Ƕȵ���ֵ</param>
	/// <returns>�ȷ����ʽ�ַ���</returns>
	template<std::floating_point Float>
	inline constexpr std::string ToString(int d, unsigned int m, Float s) noexcept
	{		
		return std::format("{}�� {}�� {:.4f}��", d, m, s);
	}
};