#include "Matrix.h"

using namespace std;
/* vec_print ----------------------------------
* print vector data
*
* args	:	a[]		I	vector
*			len		I	vector lenth
*
* return:	none
*--------------------------------------------*/
void vec_print(const double a[], int len) {
	for (int i = 0; i < len; i++)
	{
		printf("%20.10lf", a[i]);
	}
	printf("\n");
}
/* vec_plus ----------------------------------
* vector plus
*
* args	:	a[]		I	向量1
*			b[]		I	向量2
*			c[]		O	输出向量
*			len		I	向量长度
*
* return:	execution status (0:ok,0>:error)
*--------------------------------------------*/
int  vec_plus(const double a[], const double b[], double c[], int len) {
	for (int i = 0; i < len; i++)
	{
		c[i] = a[i] + b[i];
	}
	return 1;
}
/* vec_sub -------------------------------------------
* 向量减法
*
* args	:	a[]		I	向量1
*			b[]		I	向量2
*			c[]		O	结果向量
*			len		I	向量长度
*
* return:	执行状态 (1: 正常, 0>: 错误)
*----------------------------------------------------*/
int vec_sub(const double a[], const double b[], double c[], int len) {
	for (int i = 0; i < len; i++)
	{
		c[i] = a[i] - b[i];
	}
	return 1;
}
/* vec_dot_mul ----------------------------------
* 向量点乘
* args	:	a[]		I	向量a
*			b[]		I	向量b
*			len		I	向量长度
*
* return:	double点积结果
*--------------------------------------------*/
double  vec_dot_mul(const double a[], const double b[], int len) {
	double sum = 0.0;
	for (int i = 0; i < len; i++)
	{
		sum += a[i] * b[i];
	}
	return sum;
}
/* vec_cross_mul -------------------------------------
* 向量叉乘 (三维)
*
* args	:	a[]		I	向量a (3维)
*			b[]		I	向量b (3维)
*			c[]		O	结果向量 (a × b)
*
* return:	执行状态 (1: 正常)
*----------------------------------------------------*/
int  vec_cross_mul(const double a[], const double b[], double c[]) {
	c[0] = a[1] * b[2] - a[2] * b[1];
	c[1] = a[2] * b[0] - a[0] * b[2];
	c[2] = a[0] * b[1] - a[1] * b[0];
	return 1;
}
/* vec_num_mul ---------------------------------------
* 向量数乘
*
* args	:	a[]		I	输入向量
*			b[]		O	输出向量（a × num）
*			num		I	数值
*			len		I	向量长度
*
* return:	执行状态 (1: 正常)
*----------------------------------------------------*/
int  vec_num_mul(const double a[], double b[], double num, int len) {
	for (int i = 0; i < len; i++)
	{
		b[i] = a[i] * num;
	}
	return 1;
}
/* norm -----------------------------------------------------------------
* euclid norm of vector
*
* args   : double *a        I   vector a (n x 1)
*          int    len       I   lenth of vector
*
* return : ||a|| norm data
*-----------------------------------------------------------------------------*/
extern double norm(const double* a, int len)
{
	double sum = 0;
	for (int i = 0; i < len; i++)
	{
		sum += (a[i] * a[i]);
	}
	return sqrt(sum);
}
/* vec_cpy ------------------------------------------
* 数组复制
* args	:	a[]		I	src
*			b[]		O	out
*			l		I	lenth
* return:	none
*----------------------------------------------------*/
void Dmemcpy(const double a[], double b[], int l)
{
	memcpy(b, a, l * sizeof(double));
}
/* mat_print ------------------------------------------
* 打印矩阵
* args	:	a[]		I	矩阵数据（一维存储）
*			row		I	行数
*			col		I	列数
* return:	none
*----------------------------------------------------*/
void mat_print(const double a[], int row, int col)
{
	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < col; j++)
		{
			printf("%20.10lf ", a[i * col + j]);
		}
		cout << endl;
	}
	cout << endl;
}
/* mat_plus ------------------------------------------
* 矩阵加法
*
* args	:	a[]		I	矩阵a
*			b[]		I	矩阵b
*			c[]		O	结果矩阵
*			row		I	行数
*			col		I	列数
*
* return:	执行状态 (1: 正常)
*----------------------------------------------------*/
int  mat_plus(const double a[], const double b[], double c[], int row, int col) {
	vec_plus(a, b, c, row * col);
	return 1;
}
/* mat_sub -------------------------------------------
* 矩阵减法
*
* args	:	a[]		I	矩阵a
*			b[]		I	矩阵b
*			c[]		O	结果矩阵
*			row		I	行数
*			col		I	列数
*
* return:	执行状态 (1: 正常)
*----------------------------------------------------*/
int  mat_sub(const double a[], const double b[], double c[], int row, int col) {
	vec_sub(a, b, c, row * col);
	return 1;
}
/* mat_mul -------------------------------------------
* 矩阵乘法
* args	:	a[]		I	矩阵a (m1×n1)
*			b[]		I	矩阵b (m2×n2)
*			c[]		O	结果矩阵 (m1×n2)
*			m1,n1,m2,n2 I	矩阵维度
* return:	执行状态 (1: 正常, -1:维度不匹配)
*----------------------------------------------------*/
int  mat_mul(const double a[], const double b[], double c[], int m1, int n1, int m2, int n2) {
	//先检查
	if (n1 != m2) {
		return -1;
	}
	for (int i = 0; i < m1 * n2; i++)
	{
		c[i] = 0;
	}
	// 矩阵乘法运算  
	for (int i = 0; i < m1; i++) {
		for (int j = 0; j < n2; j++) {
			for (int k = 0; k < n1; k++) {
				// 索引计算
				int index_a = i * n1 + k;
				int index_b = k * n2 + j;
				int index_c = i * n2 + j;
				c[index_c] += a[index_a] * b[index_b];
			}
		}
	}
	return 1;
	//进行乘法计算
}
/* mat_trans -----------------------------------------
* 矩阵转置
* args	:	a[]		I	输入矩阵 (m×n)
*			a_T[]	O	输出转置矩阵 (n×m)
*			m		I	行数
*			n		I	列数
* return:	执行状态 (1: 正常)
*----------------------------------------------------*/
int  mat_trans(const double a[], double a_T[], int m, int n) {
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			int index_I = i * n + j;
			int index_O = j * m + i;
			a_T[index_O] = a[index_I];
		}
	}
	return 1;
}
/* mat_inv -------------------------------------------
* 一般矩阵求逆（高斯消元法）
* args	:	mat		I	输入矩阵 (n×n)
*			matinv	O	输出逆矩阵
*			n		I	矩阵维数
* return:	void（失败时退出程序）
*----------------------------------------------------*/
void mat_inv(const double* mat, double* matinv, int dim) {

	double* aug = new double[dim * dim * 2];
	for (int i = 0; i < dim; i++)
	{
		for (int j = 0; j < dim; j++)
		{
			aug[i * 2 * dim + j] = mat[i * dim + j]; 
			aug[i * 2 * dim + j + dim] = (i == j) ? 1.0 : 0.0; 
		}
	}

	for (int i = 0; i < dim; i++)
	{

		double maxEl = fabs(aug[i * 2 * dim + i]);
		int maxRow = i;
		for (int k = i + 1; k < dim; k++)
		{
			if (fabs(aug[k * 2 * dim + i]) > maxEl)
			{
				maxEl = fabs(aug[k * 2 * dim + i]);
				maxRow = k;
			}
		}

		for (int k = 0; k < 2 * dim; k++)
		{
			swap(aug[maxRow * 2 * dim + k], aug[i * 2 * dim + k]);
		}

		if (fabs(aug[i * 2 * dim + i]) < 1e-9)
		{
			cout << "Matrix is singular and cannot be inverted!" << endl;
			delete[] aug;
			exit(1);
		}

		double div = aug[i * 2 * dim + i];
		for (int k = 0; k < 2 * dim; k++)
		{
			aug[i * 2 * dim + k] /= div;
		}

		for (int k = 0; k < dim; k++)
		{
			if (k != i)
			{
				double factor = aug[k * 2 * dim + i];
				for (int j = 0; j < 2 * dim; j++)
				{
					aug[k * 2 * dim + j] -= factor * aug[i * 2 * dim + j];
				}
			}
		}
	}

	for (int i = 0; i < dim; i++)
	{
		for (int j = 0; j < dim; j++)
		{
			matinv[i * dim + j] = aug[i * 2 * dim + j + dim];
		}
	}
	delete[] aug;
}
/* mat_Inv_SRS ----------------------------------------
* 对称正定矩阵求逆
* args	:	n		I	M1的行数和列数
*			a[]		IO	输入矩阵，输出为inv(a)
* return:	1=正常，0=求逆失败
*----------------------------------------------------*/
int mat_Inv_SRS(const int n, double a[])
{
	int i, j, k, m;
	double w, g, b[160] = { 0 };   // 当前最大支持30维

	for (k = 0; k <= n - 1; k++)
	{
		w = a[0];

		if (fabs(w) < 1E-15) /* 主元素接近于0，矩阵不可逆 */
		{
			printf("Divided by 0 in MatrixInv_SRS!\n");
			return (0);
		}

		m = n - k - 1;
		for (i = 1; i <= n - 1; i++)
		{
			g = a[i * n];
			b[i] = g / w;
			if (i <= m) b[i] = -b[i];

			for (j = 1; j <= i; j++)
			{
				a[(i - 1) * n + j - 1] = a[i * n + j] + g * b[j];
			}
		}
		a[n * n - 1] = 1.0 / w;
		for (i = 1; i <= n - 1; i++)
		{
			a[(n - 1) * n + i - 1] = b[i];
		}
	}
	for (i = 0; i <= n - 2; i++)
	{
		for (j = i + 1; j <= n - 1; j++)
		{
			a[i * n + j] = a[j * n + i];
		}
	}

	return (1);
}
/* creat new Matrix -----------------------------------
* 创建一个矩阵
* args	：	row		I	行数
*			lon		I	列数
* return：	一个double数组
* note	:	动态创建强制转化
*----------------------------------------------------*/
extern double* mat(int row, int col) {
	double* p;

	if (row <= 0 || col <= 0) return NULL;
	//动态创建数组：返回类型：double* 内存大小：~
	if (!(p = (double*)malloc(sizeof(double) * row * col))) {
		printf("matrix memory allocation error: row=%d,col=%d\n", row, col);
	}
	return p;

}
/* free_matrix ---------------------------------------
* 释放二维动态矩阵内存
* args	:	mat		I	指针数组
*			m		I	行数
* return:	none
*----------------------------------------------------*/
extern void free_matrix(double** mat, int m) {
	//分布释放，两部分空间
	for (int i = 0; i < m; ++i) {
		delete[] mat[i];
	}
	delete[] mat;
}
double* zeros(int n, int m) {
	double* p;

	if ((p = mat(n, m))) {
		for (n = n * m - 1; n >= 0; n--) p[n] = 0.0;
	}

	return p;
}
extern double* eye(int n)
{
	double* p;

	if ((p = zeros(n, n))) {
		for (int i = 0; i < n; i++) p[i + i * n] = 1.0;
	}
	return p;
}
/* Mat Cl definition ---------------------------------------------------*/
extern int Rz(double Rz[9], double theta)
{
	for (int i = 0; i < 9; i++) { Rz[i] = 0; }
	Rz[0] = cos(theta);
	Rz[1] = sin(theta);
	Rz[3] = -sin(theta);
	Rz[4] = cos(theta);
	Rz[8] = 1;
	return 1;
}
extern int Rx(double Rx[9], double theta)
{
	for (int i = 0; i < 9; i++) { Rx[i] = 0; }
	Rx[0] = 1;
	Rx[4] = cos(theta);
	Rx[5] = sin(theta);
	Rx[7] = -sin(theta);
	Rx[8] = cos(theta);
	return 1;
}
extern int Rzdot(double Rzdot[9], double theta, double t_dot)
{
	double R[9];
	for (int i = 0; i < 9; i++) { R[i] = 0; }
	R[0] = -sin(theta);
	R[1] = cos(theta);
	R[3] = -cos(theta);
	R[4] = -sin(theta);
	R[8] = 0;
	vec_num_mul(R, Rzdot, t_dot, 9);

	return 1;
}
extern void BtPB(const double B[], const double P[], double Nbb[], const int m, const int n);
extern void BtPw(const double B[], const double P[], const double w[], double W[], const int m, const int n);

