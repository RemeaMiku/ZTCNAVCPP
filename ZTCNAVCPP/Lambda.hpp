#pragma once
#include <algorithm>
#include <limits>
#include"Matrix.h"

int Ld(int n, Matrix& Q, Matrix& L, Matrix& D)
{
	Matrix A { Q };
	for (auto i = n - 1; i >= 0; i--)
	{
		if ((D(i, 0) = A(i, i)) <= 0)
			return -1;
		auto a = sqrt(D(i, 0));
		for (auto j = 0; j <= i; j++)
			L(i, j) = A(i, j) / a;
		for (auto j = 0; j <= i - 1; j++)
			for (auto k = 0; k <= j; k++)
				A(j, k) -= L(i, k) * L(i, j);
		for (auto j = 0; j <= i; j++)
			L(i, j) /= L(i, i);
	}
	return 0;
}

void Gauss(int n, Matrix& L, Matrix& Z, int i, int j)
{
	auto mu = lround(L(i, j));
	if (mu != 0)
	{
		for (auto k = i; k < n; k++)
			L(k, j) -= mu * L(k, i);
		for (auto k = 0; k < n; k++)
			Z(k, j) -= mu * Z(k, i);
	}
}

void Perm(int n, Matrix& L, Matrix& D, int j, double del, Matrix& Z)
{
	auto eta = D(j, 0) / del;
	auto lam = D(j + 1, 0) * L(j + 1, j) / del;
	D(j, 0) = eta * D(j + 1, 0);
	D(j + 1, 0) = del;
	for (auto k = 0; k <= j - 1; k++)
	{
		auto a0 = L(j, k);
		auto a1 = L(j + 1, k);
		L(j, k) = -L(j + 1, j) * a0 + a1;
		L(j + 1, k) = eta * a0 + lam * a1;
	}
	L(j + 1, j) = lam;
	for (auto k = j + 2; k < n; k++)
		std::swap(L(k, j), L(k, j + 1));
	for (auto k = 0; k < n; k++)
		std::swap(Z(k, j), Z(k, j + 1));
}

void Reduction(int n, Matrix& L, Matrix& D, Matrix& Z)
{
	auto j = n - 2;
	auto k = n - 2;
	while (j >= 0)
	{
		if (j <= k)
			for (auto i = j + 1; i < n; i++)
				Gauss(n, L, Z, i, j);
		auto del = D(j, 0) + L(j + 1, j) * L(j + 1, j) * D(j + 1, 0);
		if (del + 1E-6 < D(j + 1, 0))
		{
			Perm(n, L, D, j, del, Z);
			k = j;
			j = n - 2;
			continue;
		}
		j--;
	}
}

int Sign(double number)
{
	return number <= 0 ? -1 : 1;
}

int Search(int n, int m, Matrix& L, Matrix& D, Matrix& zs, Matrix& zn, Matrix& s)
{
	int constexpr loopMax = 1E6;
	auto k = n - 1;
	Matrix dist { n,1 };
	Matrix zb { n,1 };
	Matrix z { n,1 };
	Matrix step { n,1 };
	Matrix S { n,n };
	zb(k, 0) = zs(k, 0);
	z(k, 0) = round(zb(k, 0));
	auto y = zb(k, 0) - z(k, 0);
	step(k, 0) = Sign(y);
	auto maxdist { std::numeric_limits<double>::max() };
	auto nn = 0, imax = 0;
	auto finished = false;
	for (auto c = 0; c < loopMax; c++)
	{
		auto newdist = dist(k, 0) + y * y / D(k, 0);
		if (newdist < maxdist)
		{
			if (k != 0)
			{
				dist(--k, 0) = newdist;
				for (auto i = 0; i <= k; i++)
					S(k, i) = S(k + 1, i) + (z(k + 1, 0) - zb(k + 1, 0)) * L(k + 1, i);
				zb(k, 0) = zs(k, 0) + S(k, k);
				z(k, 0) = round(zb(k, 0));
				y = zb(k, 0) - z(k, 0);
				step(k, 0) = Sign(y);
			}
			else
			{
				if (nn < m)
				{
					if (nn == 0 || newdist > s(imax, 0))
						imax = nn;
					for (auto i = 0; i < n; i++)
						zn(i, nn) = z(i, 0);
					s(nn++, 0) = newdist;
				}
				else
				{
					if (newdist < s(imax, 0))
					{
						for (auto i = 0; i < n; i++)
							zn(i, imax) = z(i, 0);
						s(imax, 0) = newdist;
						for (auto i = imax = 0; i < m; i++)
							if (s(imax, 0) < s(i, 0))
								imax = i;
						maxdist = s(imax, 0);
					}
				}
				z(0, 0) += step(0, 0);
				y = zb(0, 0) - z(0, 0);
				step(0, 0) = -step(0, 0) - Sign(step(0, 0));
			}
		}
		else
		{
			if (k == n - 1)
			{
				finished = true;
				break;
			}
			k++;
			z(k, 0) += step(k, 0);
			y = zb(k, 0) - z(k, 0);
			step(k, 0) = -step(k, 0) - Sign(step(k, 0));
		}
	}
	for (auto i = 0; i < m - 1; i++)
	{
		for (auto j = i + 1; j < m; j++)
		{
			if (s(i, 0) < s(j, 0))
				continue;
			std::swap(s(i, 0), s(j, 0));
			for (auto k = 0; k < n; k++)
				std::swap(zn(k, i), zn(k, j));
		}
	}
	return finished ? 0 : -1;
}

std::optional<Matrix> Lambda(Matrix& a, Matrix& Q)
{
	auto n = a.RowsCount();
	if (n <= 0)
		return std::nullopt;
	auto Z = Matrix::Identity(n);
	Matrix L { n,n };
	Matrix D { n,1 };
	auto info = Ld(n, Q, L, D);
	if (info != 0)
		return std::nullopt;
	Reduction(n, L, D, Z);
	auto z = Z * a;
	Matrix E { n,2 };
	Matrix s { 2,1 };
	info = Search(n, 2, L, D, z, E, s);
	if (info != 0)
		return std::nullopt;
	auto ZT = Z.Inverse();
	auto F = ZT * E;
	auto ratio = s(0, 1) / s(0, 0);
	if (ratio < 3)
		return std::nullopt;
	return F;
}