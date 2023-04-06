#include "Matrix.h"
#include <ranges>

using namespace std;
using vec = vector<double>;

void Matrix::SwapRow(Matrix& mat, int index, int index2) noexcept
{
	vec tempRow { mat[index] };
	mat[index] = mat[index2];
	mat[index2] = tempRow;
}

void Matrix::RowTimes(Matrix& mat, int index, double num) noexcept
{
	for (auto& element : mat[index])
	{
		element *= num;
	}
}

void Matrix::RowTimesAdd(Matrix& mat, int index, double num, int index2) noexcept
{
	for (int columnIndex = 0; columnIndex < mat._columns; ++columnIndex)
	{
		mat(index, columnIndex) += num * mat(index2, columnIndex);
	}
}

Matrix::Matrix() :_rows(0), _columns(0), _elements()
{}

Matrix::Matrix(int rows, int columns) :_rows(rows), _columns(columns), _elements(rows, vec(columns, 0))
{}

Matrix::Matrix(const std::vector<std::vector<double>>& matrix)
{
	_rows = static_cast<int>(matrix.size());
	_columns = _rows < 1 ? 0 : static_cast<int>(matrix[0].size());
	for (auto& row : matrix)
	{
		if (row.size() != _columns) throw "��ά����ÿ��Ԫ�ظ��������";
	}
	_elements = vector<vec>(matrix);
}

std::vector<double> Matrix::GetColumn(int j) const
{
	if (j >= 0 && j < _columns)
	{
		vec column(_rows);
		for (auto i = 0; i < _rows; ++i)
		{
			column[i] = _elements[i].at(j);
		}
		return column;
	}
	else
	{
		throw "�±곬��";
	}
}
void Matrix::SetColumn(int j, const std::vector<double>& column)
{
	if (j >= 0 && j < _columns)
	{
		if (column.size() != _rows)
		{
			throw "����Ԫ�ظ��������������";
		}
		for (auto i = 0; i < _rows; ++i)
		{
			_elements[i][j] = column[i];
		}
	}
	else
	{
		throw "�±곬��";
	}
}
void Matrix::SetRow(int i, const std::vector<double>& row)
{
	if (i >= 0 && i < _rows)
	{
		if (row.size() != _columns)
		{
			throw "����Ԫ�ظ��������������";
		}
		_elements[i] = row;
	}
	else
	{
		throw "�±곬��";
	}
}


Matrix Matrix::operator+(const Matrix& matrix2) const
{
	if (_rows == matrix2._rows && _columns == matrix2._columns)
	{
		Matrix res { _rows,_columns };
		for (int i = 0; i < _rows; i++)
		{
			for (int j = 0; j < _columns; j++)
			{
				res[i][j] = _elements[i][j] + matrix2[i][j];
			}
		}
		return res;
	}
	else
	{
		throw "�Ӽ�������������ƥ��";
	}
}

void Matrix::operator+=(const Matrix& matrix2)
{
	if (_rows == matrix2._rows && _columns == matrix2._columns)
	{
		for (int i = 0; i < _rows; i++)
		{
			for (int j = 0; j < _columns; j++)
			{
				_elements[i][j] += matrix2[i][j];
			}
		}
	}
	else
	{
		throw "�Ӽ�������������ƥ��";
	}
}

const Matrix Matrix::operator-() const
{
	Matrix res { _rows,_columns };
	for (int i = 0; i < _rows; i++)
	{
		for (int j = 0; j < _columns; j++)
		{
			res[i][j] = -_elements[i][j];
		}
	}
	return res;
}

Matrix Matrix::operator-(const Matrix& matrix2) const
{
	return *this + (-matrix2);
}

Matrix Matrix::operator*(const Matrix& matrix2) const
{
	if (_columns == matrix2._rows)
	{
		Matrix result { _rows, matrix2._columns };
		for (int row1 = 0; row1 < _rows; ++row1)
		{
			auto row2 { 0 };
			for (int column2 = 0; column2 < matrix2._columns; ++column2)
			{
				double Sum { 0 };
				for (int column1 = 0; column1 < _columns; ++column1)
				{
					Sum += _elements[row1][column1] * matrix2[column1][row2];
				}
				result[row1][column2] = Sum;
				row2++;
			}
		}
		return result;
	}
	else
	{
		throw "�˷�������������ƥ��";
	}
}

Matrix Matrix::operator*(double num) const
{
	Matrix result { _rows, _columns };
	for (int i = 0; i < _rows; ++i)
	{
		for (int j = 0; j < _columns; ++j)
		{
			result[i][j] = _elements[i][j] * num;
		}
	}
	return result;
}

Matrix operator*(double num, const Matrix& matrix)
{
	return matrix * num;
}

Matrix Matrix::operator/(const Matrix& matrix2) const
{
	return *this * matrix2.Inverse();
}

Matrix Matrix::operator/(double num) const
{

	return *this * (1 / num);

}

Matrix Matrix::Transpose() const noexcept
{
	Matrix result { _columns, _rows };
	for (int i = 0; i < _rows; ++i)
	{
		for (int j = 0; j < _columns; ++j)
		{
			result[j][i] = _elements[i][j];
		}
	}
	return result;
}

Matrix Matrix::Inverse() const
{
	if (_rows == _columns) //����
	{
		Matrix mat { _elements };
		Matrix res { Identity(_rows) };
		//�任�õ��������Ǿ���
		for (int rowIndex = 0; rowIndex < _rows - 1; ++rowIndex)
		{
			if (abs(mat[rowIndex][rowIndex]) <= 1E-6)
				//���Խ�Ԫ��Ϊ0�����²��һ���
			{
				auto isSwaped { false };
				for (int rowIndex2 = rowIndex + 1; rowIndex2 < _rows; ++rowIndex2)
				{
					if (abs(mat[rowIndex2][rowIndex]) > 1E-6)
						//����
					{
						SwapRow(mat, rowIndex, rowIndex2);
						SwapRow(res, rowIndex, rowIndex2);
						isSwaped = true;
						break;
					}
				}
				if (!isSwaped)
				{
					throw "��������,�޷�����";
				}
			}
			//���к����Խ���Ԫ��
			double diag { mat[rowIndex][rowIndex] };
			//ʹ���Խ���Ԫ�ػ�Ϊ1
			RowTimes(mat, rowIndex, 1 / diag);
			RowTimes(res, rowIndex, 1 / diag);
			//������м�ȥ���ж�Ӧ�еı���
			for (int rowIndex2 = rowIndex + 1; rowIndex2 < _rows; ++rowIndex2)
			{
				//����
				double d { mat[rowIndex2][rowIndex] };
				RowTimesAdd(mat, rowIndex2, -d, rowIndex);
				RowTimesAdd(res, rowIndex2, -d, rowIndex);
			}
		}
		//������½�Ԫ��
		if (abs(mat[_rows - 1][static_cast<size_t>(_rows - 1)]) > DBL_EPSILON)
		{
			//���½�Ԫ������Ϊ1
			RowTimes(res, _rows - 1, 1 / mat[_rows - 1][static_cast<size_t>(_rows - 1)]);
			//��������������õ���λ����
			for (int rowIndex = _rows - 1; rowIndex > 0; --rowIndex)
			{
				for (int rowIndex2 = rowIndex - 1; rowIndex2 >= 0; --rowIndex2)
				{
					RowTimesAdd(res, rowIndex2, -mat[rowIndex2][rowIndex], rowIndex);
				}
			}
			return res;
		}
		else
		{
			throw "�������ȣ��޷�����";
		}
	}
	else
	{
		throw "�����Ƿ����޷�����";
	}
}

double Matrix::Determinant(const Matrix& matrix)
{
	if (matrix._rows == matrix._columns)
	{
		Matrix mat { matrix };
		if (mat._rows == 1)
		{
			return mat[0][0];
		}
		if (mat._rows == 2)
		{
			return mat[0][0] * mat[1][1] - mat[1][0] * mat[0][1];
		}
		double res { 1 };
		//�任�õ��������Ǿ���
		for (int rowIndex = 0; rowIndex < mat._rows - 1; ++rowIndex)
		{
			if (abs(mat[rowIndex][rowIndex]) <= DBL_EPSILON)
				//���Խ�Ԫ��Ϊ0�����²��һ���
			{
				bool isSwaped { false };
				for (int rowIndex2 = rowIndex + 1; rowIndex2 < mat._rows; ++rowIndex2)
				{
					if (abs(mat[rowIndex2][rowIndex]) > DBL_EPSILON)
						//����
					{
						SwapRow(mat, rowIndex, rowIndex2);
						isSwaped = true;
						break;
					}
				}
				if (!isSwaped)
				{
					return 0;
				}
			}
			//���к����Խ���Ԫ��
			double diag { mat[rowIndex][rowIndex] };
			//ʹ���Խ���Ԫ�ػ�Ϊ1
			RowTimes(mat, rowIndex, 1 / diag);
			//�����res
			res *= diag;
			//������м�ȥ���ж�Ӧ�еı���
			for (int rowIndex2 = rowIndex + 1; rowIndex2 < mat._rows; ++rowIndex2)
			{
				//����
				double d { mat[rowIndex2][rowIndex] };
				RowTimesAdd(mat, rowIndex2, -d, rowIndex);
			}
		}
		res *= mat[mat._rows - 1][static_cast<size_t>(mat._rows - 1)] * pow(-1, mat._rows);
		return res;
	}
	else
	{
		throw "�Ƿ����޷�������ʽ��";
	}
}

Matrix Matrix::Complement(const Matrix& matrix)
{
	if (matrix._rows == matrix._columns) //����
	{
		Matrix result { matrix._rows, matrix._columns };
		for (int i = 0; i < matrix._rows; ++i)
		{
			for (int j = 0; j < matrix._columns; ++j)
			{
				//����aij������ʽ����
				Matrix matrix1 = Matrix(matrix._rows - 1, matrix._columns - 1); //aij������ʽ����
				int row = 0;
				for (int k = 0; k < matrix._rows; ++k)
				{
					int column = 0;
					if (k == i) //ȥ����i��
					{
						continue;
					}
					for (int l = 0; l < matrix._rows; ++l)
					{
						if (l == j) //ȥ����j��
						{
							continue;
						}
						matrix1[row][column++] = matrix[k][l];
					}
					row++;
				}
				result[i][j] = pow(-1, i + j) * Determinant(matrix1);
			}
		}
		return result;
	}
	else
	{
		throw "�����Ƿ����޷���������";
	}
}

Matrix Matrix::Identity(int n) noexcept
{
	Matrix result { n,n };
	for (int i = 0; i < n; ++i)
	{
		result[i][i] = 1;
	}
	return result;
}

std::string Matrix::ToString() const noexcept
{
	string res { "" };
	for (auto& row : _elements)
	{
		for (auto& ele : row)
		{
			res += format("{:12.4f}\t", ele);
		}
		res += '\n';
	}
	return res;
}