#include "rtklib.h"

/* BLH2XYZ -------------------------------------------
* Convert geodetic coordinates (BLH) to Earth-Centered Earth-Fixed (ECEF) coordinates (XYZ)
*
* args   : blh     I   Geodetic coordinates (latitude, longitude, height)
*          xyz     O   Converted Earth-Centered Earth-Fixed coordinates
*          elli    I   Ellipsoid type (e.g., WGS84 or CGCS2000)
*
* return : none
*----------------------------------------------------*/
void BLH2XYZ(const BLHCOOR blh, XYZCOOR& xyz, ELLIPSOID elli)
{
	double R = (elli == CGCS2K) ? R_CGCS2K : R_WGS84;
	double F = (elli == CGCS2K) ? F_CGCS2K : F_WGS84;
	double e2 = 2 * F - F * F;

	xyz.XYZ.x = (blh.BLH.h + R / sqrt(1 - e2 * pow(sin(blh.BLH.b), 2))) * cos(blh.BLH.b) * cos(blh.BLH.l);
	xyz.XYZ.y = (blh.BLH.h + R / sqrt(1 - e2 * pow(sin(blh.BLH.b), 2))) * cos(blh.BLH.b) * sin(blh.BLH.l);
	xyz.XYZ.z = (R / sqrt(1 - e2 * pow(sin(blh.BLH.b), 2)) * (1 - e2) + blh.BLH.h) * sin(blh.BLH.b);
}
/* XYZ2BLH -------------------------------------------
* 将地心地固坐标（XYZ）转换为大地坐标（BLH）
*
* args   : xyzr    I   地心地固坐标（X, Y, Z）
*          blhr    O   转换后的大地坐标（经度、纬度、高程）
*          elli    I   椭球体类型（如WGS84或CGCS2000）
*
* return : none
* note   : 使用迭代法计算大地坐标
*----------------------------------------------------*/
void XYZ2BLH(const XYZCOOR xyzr, BLHCOOR& blhr, ELLIPSOID elli)
{
	double R = (elli == CGCS2K) ? R_CGCS2K : R_WGS84;
	double F = (elli == CGCS2K) ? F_CGCS2K : F_WGS84;
	double e = sqrt(2 * F - F * F);

	int iterator;
	double X, Y, Z, B;
	double dZ, dZ0; //Z的变化量，迭代计算，初值dZ=e^2*Z;
	double N;//卯酉圈半径
	X = xyzr.xyz[0]; Y = xyzr.xyz[1]; Z = xyzr.xyz[2];
	dZ0 = e * e * Z;
	iterator = 10;

	while (true)
	{
		B = (Z + dZ0) / sqrt(X * X + Y * Y + (Z + dZ0) * (Z + dZ0));//Sin(B)
		N = R / sqrt(1 - (e * B) * (e * B));
		dZ = N * e * e * B;
		if (fabs(dZ - dZ0) < 1e-4 || iterator >= 10)
		{
			break;
		}
		dZ0 = dZ;
		iterator++;
	}
	blhr.blh[1] = atan2(Y, X);
	blhr.blh[0] = atan2((Z + dZ), (sqrt(X * X + Y * Y)));
	blhr.blh[2] = sqrt(X * X + Y * Y + (Z + dZ) * (Z + dZ)) - N;

}
/* BLH2EnuMat ---------------------------------------
* 将大地坐标（BLH）转换为ENU矩阵
*
* args   : blhr    I   大地坐标（经度、纬度、高程）
*          enuMAT  O   得到转换后ENU矩阵
*
* return : none
*----------------------------------------------------*/
void BLH2EnuMat(const BLHCOOR blhr, double enuMAT[9])
{
	double B = blhr.BLH.b;
	double L = blhr.BLH.l;
	enuMAT[0] = -sin(B) * cos(L);
	enuMAT[1] = -sin(B) * sin(L);
	enuMAT[2] = cos(B);
	enuMAT[3] = -sin(L);
	enuMAT[4] = cos(L);
	enuMAT[5] = 0.0;
	enuMAT[6] = cos(B) * cos(L);
	enuMAT[7] = cos(B) * sin(L);
	enuMAT[8] = sin(B);
}
/* CompSatElAz --------------------------------------
* 计算卫星的高度角和方位角
*
* args   : P_rec   I   接收机地心地固坐标
*          P_sat   I   卫星地心地固坐标
*          Elev    O   卫星高度角
*          Azim    O   卫星方位角
*          elli    I   椭球体类型（如WGS84或CGCS2000）
*
* return : none
*----------------------------------------------------*/
void CompSatElAz(const XYZCOOR P_rec, const XYZCOOR P_sat, double& Elev, double& Azim, ELLIPSOID elli)
{
	double R[9], denu[3], dxyz[3];
	double x_sat[3], x_rec[3];
	BLHCOOR blh_rec;
	for (int i = 0; i < 3; i++) {
		x_sat[i] = P_sat.xyz[i];
		x_rec[i] = P_rec.xyz[i];
	}
	XYZ2BLH(P_rec, blh_rec, elli);
	BLH2EnuMat(blh_rec, R);
	vec_sub(x_sat, x_rec, dxyz, 3);
	// Transform to ENU
	mat_mul(R, dxyz, denu, 3, 3, 3, 1);
	Elev = asin(denu[2] / sqrt(denu[0] * denu[0] + denu[1] * denu[1] + denu[2] * denu[2]));
	Azim = atan2(Elev, denu[0]);
}

void Comp_dEnu(const XYZCOOR P_real, const XYZCOOR P_cal, NEUCOOR& DENU, ELLIPSOID elli)
{
	double R[9], denu[3], dxyz[3];
	double x_real[3], x_cal[3];
	BLHCOOR blh_real;
	for (int i = 0; i < 3; i++) { x_real[i] = P_real.xyz[i]; x_cal[i] = P_cal.xyz[i]; }
	vec_sub(x_cal, x_real, dxyz, 3);
	XYZ2BLH(P_real, blh_real, elli);
	BLH2EnuMat(blh_real, R);
	mat_mul(R, dxyz, denu, 3, 3, 3, 1);
	for (int i = 0; i < 3; i++) {
		DENU.enu[i] = denu[i];
	}
}
