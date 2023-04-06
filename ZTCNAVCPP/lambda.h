#pragma once
#ifndef lambda_H

#define lambda_H
/* constants/macros ----------------------------------------------------------*/

#define LOOPMAX     100000           /* maximum count of search loop */

#define SGN(x)      ((x)<=0.0?-1.0:1.0)
#define ROUND(x)    (floor((x)+0.5))
#define SWAP(x,y)   do {double tmp_; tmp_=x; x=y; y=tmp_;} while (0)

#include <stdio.h>
#include <math.h>
#include <memory.h>
#include "Matrix.h"
//#include "Rover.h"

/* LD factorization (Q=L'*diag(D)*L) -----------------------------------------*/
static int LD(int n, const double* Q, double* L, double* D);

/* integer gauss transformation ----------------------------------------------*/
void gauss(int n, double* L, double* Z, int i, int j);

/* permutations --------------------------------------------------------------*/
void perm(int n, double* L, double* D, int j, double del, double* Z);

/* lambda reduction (z=Z'*a, Qz=Z'*Q*Z=L'*diag(D)*L) (ref.[1]) ---------------*/
void reduction(int n, double* L, double* D, double* Z);

/* modified lambda (mlambda) search (ref. [2]) -------------------------------*/
int search(int n, int m, const double* L, const double* D,
    const double* zs, double* zn, double* s);
/* lambda/mlambda integer least-square estimation ------------------------------
* integer least-square estimation. reduction is performed by lambda (ref.[1]),
* and search by mlambda (ref.[2]).
* args   : int    n      I  number of float parameters
*          int    m      I  number of fixed solutions
*          double *a     I  float parameters (n x 1)
*          double *Q     I  covariance matrix of float parameters (n x n)
*          double *F     O  fixed solutions (n x m)
*          double *s     O  sum of squared residulas of fixed solutions (1 x m)
* return : status (0:ok,other:error)
* notes  : matrix stored by column-major order (fortran convension)
*-----------------------------------------------------------------------------*/
int lambda(int n, int m, const double* a, const double* Q, double* F, double* s);
/****************************************************************************
  MatrixInv

  目的：矩阵求逆,采用全选主元高斯-约当法

  参数:
  n      M1的行数和列数
  a      输入矩阵
  b      输出矩阵   b=inv(a)
  返回值：1=正常，0=致命错误

****************************************************************************/

int MatrixInv(int n, double a[], double b[]);

/****************************************************************************
  MatrixMultiply

  目的：矩阵乘法

  参数:
  m,n    Z的行数和列数
  p,q    a的行数和列数
  z      输出矩阵   z=Z*a
  返回值：1=正常，0=致命错误

****************************************************************************/

int MatrixMultiply(const int m1, const int n1, const int m2, const int n2,const double* A,const double* B, double* C);


#endif // !lambda_H
