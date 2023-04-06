#include "Vector.h"

using namespace std;
using vec = vector<double>;
using cvec = const vec&;

Vector::Vector() :_elements(), Dimension(0) {}

Vector::Vector(int n) :_elements(n, 0), Dimension(n) {}

Vector::Vector(const vector<double>& vec) :_elements(vec), Dimension(static_cast<int>(vec.size())) {}

double Vector::operator *(const Vector& vector2) const
{
	if (Dimension == vector2.Dimension)
	{
		double res{ 0 };
		for (int i = 0; i < Dimension; ++i)
		{
			res += _elements[i] * vector2[i];
		}
		return res;
	}
	else
	{
		throw "乘法运算维度不匹配";
	}
}

Vector Vector::operator *(double num) const
{
	Vector res(Dimension);
	for (int i = 0; i < Dimension; ++i)
	{
		res[i] = _elements[i] * num;
	}
	return res;
}

Vector operator *(double num, const Vector& vector2)
{
	return vector2 * num;
}

Vector Vector::CrossProduct(const Vector& vec2)
{
	return Vector({ _elements[1] * vec2[2] - _elements[2] * vec2[1],_elements[2] * vec2[0] - _elements[0] * vec2[2],_elements[0] * vec2[1] - _elements[1] * vec2[0] });
}

Vector Vector::operator / (double num) const
{
	if (abs(num) < DBL_EPSILON)
	{
		return *this * (1 / num);
	}
	else
	{
		throw "除数不可为0";
	}
}

Vector Vector::operator + (const Vector& vector2) const
{
	if (Dimension == vector2.Dimension)
	{
		Vector res(Dimension);
		for (int i = 0; i < Dimension; ++i)
		{
			res[i] = _elements[i] + vector2[i];
		}
		return res;
	}
	else
	{
		throw "加减运算维度不匹配！";
	}
}

const Vector Vector::operator-() const
{
	Vector res(Dimension);
	for (int i = 0; i < Dimension; ++i)
	{
		res[i] = -_elements[i];
	}
	return res;
}

Vector Vector::operator - (const Vector& vector2) const
{
	return *this + (-vector2);
}

double Vector::Norm() const noexcept
{
	double sum{ 0 };
	for (auto& element : _elements)
	{
		sum += pow(element, 2);
	}
	return sqrt(sum);
}

Vector Vector::Unitization() const noexcept
{
	return (*this) / Norm();
}

string Vector::ToString() const noexcept
{
	string res{ "" };
	for (auto& element : _elements)
	{
		res += format("{:.4f} ", element);
	}
	return res;
}