/*-----------------------------------------------------------------
* Martrix.h : vector&matrix 矩阵创建及基本运算
*
*			Copyright (C) 2025 by J.Y. Liu, All rights reserved.
*
* options : none
*
* reference	: [1]"rtkcmn.c"
*
* version : $Revision: 1.1 $ $Date: 2025/04/09 21:48:06 $
*
* histroy : 2025/04/09  1.0  new
*
*----------------------------------------------------------------*/
#ifndef MATRIX_H
#define MATRIX_H

#include<iostream>
#include <cmath>
#include<vector>
#include <fstream>  
#include <string>
#include <cstdio>
#include <algorithm>
using namespace std;

/* functions of vector ------------------------------------------------*/

void vec_print(const double a[], int len);
int  vec_plus(const double a[], const double b[], double c[], int len);
int  vec_sub(const double a[], const double b[], double c[], int len);
double  vec_dot_mul(const double a[], const double b[], int len);
int  vec_cross_mul(const double a[], const double b[], double c[]);
int  vec_num_mul(const double a[], double b[], double num, int len);
double norm(const double* a, int n);
void Dmemcpy(const double a[], double b[], int l);

/* functions of Matrix ------------------------------------------------*/

void mat_print(const double a[], int row, int col);
int  mat_plus(const double a[], const double b[], double c[], int row, int col);
int  mat_sub(const double a[], const double b[], double c[], int row, int col);
int  mat_mul(const double a[], const double b[], double c[], int m1, int n1, int m2, int n2);
int  mat_trans(const double a[], double a_T[], int row, int col);
void mat_inv(const double* mat, double* matinv, int n);
int mat_Inv_SRS(const int n, double a[]);

/* creat Matrix -------------------------------------------------------*/

double* mat(int n, int m);
void free_matrix(double** mat, int m);
double* zeros(int n, int m);
double* eye(int n);

/* Mat Cal definition ---------------------------------------------------*/
int Rz(double Rz[9], double theta);
int Rx(double Rx[9], double theta);
int Rzdot(double Rzdot[9], double theta, double t_dot);


#endif


