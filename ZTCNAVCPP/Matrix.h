#pragma once
#include <vector>
#include <format>
/// <summary>
/// 矩阵类
/// 最后更新：2022.10.17
/// © 2022 RemeaMiku 版权所有
/// </summary>
class Matrix
{
private:
	/// <summary>
	/// 数组
	/// </summary>
	std::vector<std::vector<double>> _elements;
	/// <summary>
	/// 行数
	/// </summary>
	int _rows;
	/// <summary>
	/// 列数
	/// </summary>
	int _columns;
	/// <summary>
	/// 互换行
	/// </summary>
	/// <param name="mat"></param>
	/// <param name="index"></param>
	/// <param name="index2"></param>
	static void SwapRow(Matrix& mat, int index, int index2) noexcept;
	/// <summary>
	/// 行相加
	/// </summary>
	/// <param name="mat"></param>
	/// <param name="index"></param>
	/// <param name="num"></param>
	static void RowTimes(Matrix& mat, int index, double num) noexcept;
	/// <summary>
	/// 行倍加
	/// </summary>
	/// <param name="mat"></param>
	/// <param name="index"></param>
	/// <param name="num"></param>
	/// <param name="index2"></param>
	static void RowTimesAdd(Matrix& mat, int index, double num, int index2) noexcept;
public:
	/// <summary>
	/// 构造空矩阵
	/// </summary>
	Matrix();
	/// <summary>
	/// 构造已知行数，列数的零矩阵
	/// </summary>
	Matrix(int rows, int columns);
	/// <summary>
	/// 构造以二维数组为模板的矩阵
	/// </summary>
	explicit Matrix(const std::vector<std::vector<double>>& matrix);
	/// <summary>
	/// 获取矩阵对应的二维数组
	/// </summary>
	inline constexpr std::vector<std::vector<double>> ToArray() const noexcept { return _elements; }
	/// <summary>
	/// 二维矩阵的行数
	/// </summary>
	inline constexpr int RowsCount() const noexcept { return _rows; }
	/// <summary>
	/// 二维矩阵的列数
	/// </summary>
	inline constexpr int ColumnsCount() const noexcept { return _columns; }
	/// <summary>
	/// 返回第i+1行可变引用，不做下标检查
	/// </summary>
	inline constexpr std::vector<double>& operator [](int i) { return _elements[i]; }
	/// <summary>
	/// 返回第i+1行常量引用，不做下标检查
	/// </summary>
	inline constexpr const std::vector<double>& operator [](int i) const { return _elements[i]; }
	/// <summary>
	/// 返回第i+1行第j+1列元素可变引用，返回前进行下标检查
	/// </summary>
	/// <param name="i"></param>
	/// <param name="j"></param>
	/// <returns></returns>
	inline constexpr double& operator()(int i,int j)
	{
		if (i >= 0 && i < _rows && j >= 0 && j < _columns) return _elements[i][j];
		else throw "下标超限";
	}
	/// <summary>
	/// 返回第i+1行第j+1列元素常量引用，返回前进行下标检查
	/// </summary>
	/// <param name="i"></param>
	/// <param name="j"></param>
	/// <returns></returns>
	inline constexpr const double& operator()(int i,int j) const { return _elements.at(i).at(j); }
	/// <summary>
	/// 获取指定行数组
	/// </summary>
	/// <param name="i"></param>
	/// <returns></returns>
	inline constexpr std::vector<double> GetRow(int i) const
	{
		if (i >= 0 && i < _rows) return _elements[i];
		else throw "下标超限";
	}
	/// <summary>
	/// 获取指定列数组
	/// </summary>
	/// <param name="j"></param>
	/// <returns></returns>
	std::vector<double> GetColumn(int j) const;
	/// <summary>
	/// 给指定的行赋值
	/// </summary>
	/// <param name="i">行索引</param>
	/// <param name="row"></param>
	void SetRow(int i, const std::vector<double>& row);
	/// <summary>
	/// 给指定的列赋值
	/// </summary>
	/// <param name="j"></param>
	/// <param name="column"></param>
	void SetColumn(int j, const std::vector<double>& column);
	/// <summary>
	/// 二维矩阵加法运算
	/// </summary>
	/// <returns></returns>
	Matrix operator + (const Matrix& matrix2) const;
	void operator+=(const Matrix& matrix2);

	/// <summary>
	/// 取负
	/// </summary>
	/// <returns></returns>
	const Matrix operator -() const;
	/// <summary>
	/// 二维矩阵减法运算
	/// </summary>
	/// <param name="matrix1"></param>
	/// <param name="matrix2"></param>
	/// <returns></returns>
	Matrix operator - (const Matrix& matrix2) const;
	/// <summary>
	/// 二维矩阵乘法运算，矩阵相乘
	/// </summary>
	/// <param name="matrix1"></param>
	/// <param name="matrix2"></param>
	/// <returns></returns>
	Matrix operator *(const Matrix& matrix2) const;
	/// <summary>
	/// 二维矩阵乘法运算,矩阵左乘一个数
	/// </summary>
	/// <param name="matrix1"></param>
	/// <param name="num"></param>
	/// <returns></returns>
	Matrix operator *(double num) const;
	/// <summary>
	/// 二维矩阵乘法运算,矩阵右乘一个数
	/// </summary>
	/// <param name="num"></param>
	/// <param name="matrix1"></param>
	/// <returns></returns>
	friend Matrix operator *(double num, const Matrix& matrix);
	/// <summary>
	/// 二维矩阵除法运算
	/// </summary>
	/// <param name="matrix1"></param>
	/// <param name="matrix2"></param>
	/// <returns></returns>
	Matrix operator / (const Matrix& matrix2) const;
	/// <summary>
	/// 二维矩阵除法运算,矩阵除以一个数
	/// </summary>
	/// <param name="matrix1"></param>
	/// <param name="num"></param>
	/// <returns></returns>
	Matrix operator / (double num) const;
	/// <summary>
	/// 二维矩阵转置运算
	/// </summary>
	/// <returns></returns>
	Matrix Transpose() const noexcept;
	/// <summary>
	/// 二维矩阵求逆运算
	/// </summary>
	/// <returns></returns>
	Matrix Inverse() const;
	/// <summary>
	/// 递归计算方阵的行列式,代数余子式法
	/// </summary>
	/// <param name="matrix"></param>
	/// <returns></returns>
	static double Determinant(const Matrix& matrix);
	/// <summary>
	/// 计算方阵伴随矩阵
	/// </summary>
	/// <param name="matrix"></param>
	/// <returns></returns>
	static Matrix Complement(const Matrix& matrix);
	/// <summary>
	/// 获取n阶单位阵
	/// </summary>
	/// <param name="n"></param>
	/// <returns></returns>
	static Matrix Identity(int n) noexcept;
	/// <summary>
	/// 获取矩阵字符串，默认保留四位小数
	/// </summary>
	/// <returns></returns>
	std::string ToString() const noexcept;
};
