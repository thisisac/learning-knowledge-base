/*-----------------------------------------------------------------
* rtklib.h : SPP software constans, types and function prototypes
*
*			Copyright (C) 2025 by J.Y. Liu, All rights reserved.
*
* options : none
*
* reference	: [1]"rtkcmn.c"	[2]"RTK_Structs.h"
*
* version : $Revision: 1.1 $ $Date: 2025/04/10 19:20:11 $
*
* histroy : 2025/04/10  1.0
*			2025/05/22  1.1
*			2025/06/24  1.2	new
*
*----------------------------------------------------------------*/
#ifndef RTKLIB_H
#define RTKLIB_H

#include<iostream>
#include<vector>
#include<cmath>
#include <fstream>    
#include <cstdio>
#include"Matrix.h"

using namespace std;

/* const -----------------------------------------------------------------*/

/* Common uints */
#define PAI 3.1415926535898
#define PAI2 (2.0*PAI)                  /* 2pi */
#define Rad (PAI/180.0)                 /* Radians per degree */
#define Deg (180.0/PAI)                 /* Degrees per radian */
#define C_Light 299792458.0				/* Speed of light  [m/s]; IAU 1976  */

/* ECEF params */
#define R_WGS84  6378137.0				/* Radius Earth [m]; WGS-84  */
#define F_WGS84  1.0/298.257223563		/* Flattening; WGS-84   */
#define Omega_WGS 7.2921151467e-5		/*[rad/s], the earth rotation rate */
#define GM_WGS   398600.5e+9			/* [m^3/s^2]; WGS-84 */

#define R_CGCS2K  6378137.0				/* Radius Earth [m]; CGCS2000  */
#define F_CGCS2K  1.0/298.257222101		/* Flattening; CGCS2000   */
#define Omega_CGCS 7.2921150e-5			/*[rad/s], the earth rotation rate */
#define GM_CGCS   398600.4418e+9		/* [m^3/s^2]; CGCS2000  */

/* some constants about GPS satellite signal */
#define  FG1_GPS  1575.42E6             /* L1信号频率 */
#define  FG2_GPS  1227.60E6             /* L2信号频率 */
#define  FG12R    (77/60.0)             /* FG1_Freq/FG2_Freq */
#define  FG12R2   (5929/3600.0)
#define  WL1_GPS  (C_Light/FG1_GPS)
#define  WL2_GPS  (C_Light/FG2_GPS)

/* some constants about Compass satellite signal */
#define  FG1_BDS  1561.098E6               /* B1信号的基准频率 */
#define  FG2_BDS  1207.140E6               /* B2信号的基准频率 */
#define  FG3_BDS  1268.520E6               /* B3信号的基准频率 */
#define  FC12R    (FG1_BDS/FG2_BDS)       /* FG1_BDS/FG2_BDS */
#define  FC12R2   (FC12R*FC12R)           /* FG1_BDS^2/FG2_BDS^2 */
#define  FC13R    (FG1_BDS/FG3_BDS)       
#define  FC13R2   (FC13R*FC13R)			/* FG1_BDS^2/FG3_BDS^2 */
#define  WL1_BDS  (C_Light/FG1_BDS)
#define  WL2_BDS  (C_Light/FG2_BDS)
#define  WL3_BDS  (C_Light/FG3_BDS)    

#define GPST_BDT   14		
#define GPST_BDT_Week 1356
#define MAXCHANNUM 36
#define MAXGPSNUM  32
#define MAXBDSNUM  63
#define MAXSATNUM  95
#define MAXRAWLEN  40960
#define MAXNOVDLEN 20*1024
#define LIM_BDST (3600+300)
#define LIM_GPST (7200+300)
#define DELTATIME  2                      /*基站流动站最大相差时间*/
#define FILEMODE 1

#define MAX_dMW	   0.05
#define MAX_dGF    3
#define PsrDelta   1
#define AdrDelta   1e-3 
#define PsrW       2*PsrDelta*PsrDelta
#define AdrW       2*AdrDelta*AdrDelta


constexpr auto POLYCRC32 = 0xEDB88320u /* CRC32 polynomial */;;

enum GNSSsys { UNKS = 0, GPS, BDS, GLONASS, GALILEO, QZSS };//卫星系统
enum ELLIPSOID { UNK = 0, CGCS2K, WGS84 };//椭圆类型
enum SignalType {
	CODE_NONE = 0, GPS_L1C, GPS_L2P, GPS_L2W, GPS_L5Q, GPS_L1L, GPS_L2C, GLO_L1C, GLO_L2C,
	BDS_B1I_D1, BDS_B1I_D2, BDS_B2I_D1, BDS_B2I_D2, BDS_B3I_D1, BDS_B3I_D2, BDS_B1C, BDS_B2a, BDS_B2b
};

/* structs definitions -----------------------------------------------------------------*/

/* 通用时间定义 */
struct COMMONTIME
{
	short Year;
	unsigned short Month;
	unsigned short Day;
	unsigned short Hour;
	unsigned short Minute;
	double Second;
	COMMONTIME()
	{
		Year = 0;
		Month = Day = Hour = Minute = 0;
		Second = 0.0;
	}
	COMMONTIME(short y, short m, short d, short h, short min, short sec)
	{
		Year = y;
		Month = m;
		Day = d;
		Hour = h;
		Minute = min;
		Second = sec;
	}
	void print_COMMONTIME()
	{
		printf("%d-%d-%d %d:%d:%6.10lf\n", Year, Month, Day, Hour, Minute, Second);
	}
};
/* 简化儒略日定义 */
struct MJDTIME
{
	int Days;
	double FracDay;
	MJDTIME()
	{
		Days = 0;
		FracDay = 0.0;
	}
	MJDTIME(int d, double fd)
	{
		Days = d;
		FracDay = fd;
	}
	void print_MJDTIME()
	{
		printf("MJDTIME: Days %d, FracDay %12.9f\n", Days, FracDay);
	}
};
/* GPS时间定义 */
struct GPSTIME
{
	unsigned short Week;
	double SecOfWeek;
	GPSTIME()
	{
		Week = 0;
		SecOfWeek = 0.0;
	}
	GPSTIME(unsigned short w, double s)
	{
		Week = w;
		SecOfWeek = s;
	}
	void print_GPSTIME()
	{
		printf("week: %d, sow %.3f\n", Week, SecOfWeek);
	}
};
/* CoordConvert -----------------------------------------*/
union  XYZCOOR
{
	//成员共享空间
	//通过XYZ结构体访问,（匿名成员结构体XYZ）
	struct
	{
		double x;
		double y;
		double z;
	}XYZ;
	//通过xyz数组访问
	double xyz[3];
	XYZCOOR()
	{
		xyz[0] = xyz[1] = xyz[2] = 3.0;
	}
	XYZCOOR(double x1, double y1, double z1)
	{
		XYZ.x = x1;
		XYZ.y = y1;
		XYZ.z = z1;
	}
	void print_XYZ()
	{
		printf("X(m):%20.6f, Y(m):%20.6f, Z(m):%20.6f\n", XYZ.x, XYZ.y, XYZ.z);
	}
};
/* 大地坐标系定义 */
union BLHCOOR
{
	struct
	{
		double b;
		double l;
		double h;
	}BLH;
	double blh[3];
	BLHCOOR()
	{
		blh[0] = blh[1] = blh[2] = 0;
	}
	BLHCOOR(double b1, double l1, double h1)
	{
		blh[0] = b1;
		blh[1] = l1;
		blh[2] = h1;
	}
	void print_BLH()
	{
		printf("B(°):%6.3f,	L(°):%6.3f, H(m):%10.3f\n", blh[0] * Deg, blh[1] * Deg, blh[2]);
	}
};
/* 测站地平坐标系定义 */
union NEUCOOR
{
	struct
	{
		double n;
		double e;
		double u;
	}NEU;
	double enu[3];
	NEUCOOR()
	{
		enu[0] = enu[1] = enu[2] = 0;
	}
	NEUCOOR(double n1, double e1, double u1)
	{
		enu[0] = e1;
		enu[1] = n1;
		enu[2] = u1;
	}
	void print_NEU()
	{
		printf("dN(m):%8.3f,  dE(m):%8.3f,  dU(m):%8.3f\n", enu[0], enu[1], enu[2]);
	}
};

/* observation data struct --------------------------------------------------------------------------------*/
struct SIGOBS
{
	SignalType sigtype;
	double psr, adr, dopp;
	SIGOBS()
	{
		sigtype = CODE_NONE;
		psr = adr = dopp = 0.0;
	}
};
/*单个卫星的观测*/
struct SATOBSDATA
{
	short Prn;
	GNSSsys Satsys;
	double P[2], L[2], D[2];
	double cn0[2], LockTime[2];
	unsigned char half[2];
	bool Valid;//false:无法组成双频数据/有周跳/有粗差
	//std::vector<SIGOBS> Sigobs;
	SATOBSDATA()
	{
		Prn = 0;
		Satsys = UNKS;
		for (int i = 0; i < 2; i++)
			P[i] = L[i] = D[i] = 0.0;
		for (int i = 0; i < 1; i++)
			cn0[i] = LockTime[i] = 0.0;
		Valid = false;
	}
};
/*单个卫星的星历*/
struct SATEPHREC
{
	short Prn;
	GNSSsys Satsys;
	GPSTIME TOC, TOE;
	double ClkBias, ClkDrift, ClkDriftRate;
	double IODE, IODC, IODE2;
	double SqrtA, M0, e, OMEGA, i0, omega;
	double Crs, Cuc, Cus, Cic, Cis, Crc;
	double DeltaN, OMEGADot, iDot;
	short SVHealth;
	double TGD1, TGD2;
	SATEPHREC()
	{
		Prn = 0; SVHealth = -1;
		Satsys = UNKS;
		ClkBias = ClkDrift = ClkDriftRate = IODE = IODC = IODE2 = TGD1 = TGD2 = 0.0;
		SqrtA = M0 = e = OMEGA = i0 = omega = DeltaN = iDot = OMEGADot = 0.0;
		Crs = Cuc = Cus = Cic = Cis = Crc = 0.0;
	}
};

/*定位结果*/
struct POSRES
{
	GPSTIME Time;
	double Pos[3]{ 0,0,6400000 }, Vel[3]{ 0,0,0 };
	double PDOP, sigma_pos, sigma_vel;
	double clk[2];//gps,bds
	int SatNum;
	double Qxx[9];
	double std_xyz[3];
	double std_enu[3];
	int GPSNum, BDSNum;

};
struct track_code
{
	SignalType signal_type;
	GNSSsys sys;
	int track, sig_code;
	int p_lock, c_lock, parity, halfc;
	int obs_type;//对于双频观测值的顺序
};
struct MWGF
{
	short Prn;//卫星号
	GNSSsys Sys;
	double MW, GF, PIF;
	int n;

	MWGF()
	{
		Prn = n = 0;
		Sys = UNKS;
		MW = GF = PIF = 0.0;
	}
};
//每颗卫星PVT，中间计算结果
struct SATMIDRES
{
	short Prn;//卫星号
	GNSSsys Sys;
	double Pos[3], Vel[3];
	double SatClkOft, SatClkSft;
	double Elevation, Azimuth;
	double TronCorr;
	double tgd1, tgd2;
	bool Valid;//false=星历过期或没有星历，ture=计算成功
	SATMIDRES()
	{
		for (int i = 0; i < 3; i++)
		{
			Pos[i] = 0.0;
			Vel[i] = 0.0;
		}
		Prn = 0; Sys = UNKS;
		Elevation = PAI / 2.0;
		Azimuth = 0.0;
		tgd1 = tgd2 = 0.0;
		SatClkOft = SatClkSft = TronCorr = 0.0;
		Valid = false;
	}
};
/*历元观测值*/
struct EPOCHOBSDATA
{
	GPSTIME		time;
	short		SatNum;
	SATOBSDATA	SatObs[MAXCHANNUM];
	SATMIDRES  SatPVT[MAXCHANNUM]; // 卫星位置等计算结果，数组索引与SatObs相同
	MWGF       ComObs[MAXCHANNUM];  // 当前历元的组合观测值，数组索引与SatObs相同
	POSRES		Pos;		//接收机定位结果+SPP计算结果
	POSRES		decode_Pos;
	EPOCHOBSDATA()
	{
		SatNum = 0;
	}
};

/* functions ------------------------------------------------------------------------*/

/* NovAtel OEM7 data decodeing 二进制解码 ------------------------------------------------------------------ */
short I2(unsigned char* p);
int I4(unsigned char* p);
unsigned short UI2(unsigned char* p);
unsigned int UI4(unsigned char* p);
float R4(unsigned char* p);
double R8(unsigned char* p);

unsigned int crc32(unsigned char* buff, int len);
SignalType get_type(GNSSsys sys, int code);
int DecodeNovOem7Dat(unsigned char buff[], int& len, EPOCHOBSDATA* obs, SATEPHREC geph[], SATEPHREC beph[], POSRES* pos);
int decode_rangeb_oem7(unsigned char* buff, int len, EPOCHOBSDATA* obs);
int decode_gpsephem(unsigned char* buff, int len, SATEPHREC* geph);
int decode_bdsephem(unsigned char* buff, int len, SATEPHREC* beph);
int decode_psrpos(unsigned char* buff, int len, POSRES* pos);
void decode_track_status(unsigned int track_status, track_code& tc);

/* Time convert functions ------------------------------------------------------------------------*/
void CommonTime2MJDTime(const COMMONTIME* ct, MJDTIME* mt);
void MJDTime2CommonTime(const MJDTIME* mt, COMMONTIME* ct);
void MJDTime2GpsTime(const MJDTIME* mt, GPSTIME* gt);
void GpsTime2MJDTime(const GPSTIME* gt, MJDTIME* mt);
void CommonTime2GpsTime(const COMMONTIME* ct, GPSTIME* gt);
void GpsTime2CommonTime(const GPSTIME* gt, COMMONTIME* ct);
double GetDiffTime(const GPSTIME* GT2, const GPSTIME* GT1);

/* Coord convert functions ------------------------------------------------------------------------*/
void BLH2XYZ(const BLHCOOR blhc, XYZCOOR& xyzc, ELLIPSOID elli = WGS84);
void XYZ2BLH(const XYZCOOR xyzr, BLHCOOR& blhr, ELLIPSOID elli = WGS84);
void BLH2EnuMat(const BLHCOOR blhr, double enuMAT[9]);
/* cal sat pos */
void CompSatElAz(const XYZCOOR P_rec, const XYZCOOR P_sat, double& Elev, double& Azim, ELLIPSOID elli); //卫星高度角方位角计算函数
void Comp_dEnu(const XYZCOOR P_real, const XYZCOOR P_cal, NEUCOOR& DENU, ELLIPSOID elli);  //定位误差计算函数

/* Satellite PVT computation ------------------------------------------------------------------------*/
bool CompSatClkOff(const int Prn, const GNSSsys Sys, const GPSTIME* t, SATEPHREC* GPSEph, SATEPHREC* BDSEph, SATMIDRES* Mid);//钟差
bool CompGPSSatPVT(const int Prn, const GPSTIME* t, const SATEPHREC* Eph, SATMIDRES* Mid);
bool CompBDSSatPVT(const int Prn, const GPSTIME* t, const SATEPHREC* Eph, SATMIDRES* Mid);
void ComputeSatPVTAtSignalTrans(EPOCHOBSDATA& Obs, SATEPHREC* GpsEph, SATEPHREC* BDSEph, double RcvPos[3], SATMIDRES* Mid);//信号发射时刻PVT

/* 误差改正 -------------------------------------------------------------------- */
/* 模型改正 */
void compMWGF(const SATOBSDATA sat, MWGF& mwgf);
double Hopfield(const double H, const double Elev);//对流层
double Klobutchar(const GPSTIME* Time, double Elev, double Azim, double RcvPos[3]);

void DetectOutlier(EPOCHOBSDATA& obs);//粗差探测函数
void Earth_corr(SATMIDRES* Mid, GNSSsys sys, const double dt);

/* 单点定位模块 --------------------------------------------------------------- */
void compLSQ(const double B[], const double P[], const double w[], double x[], double Nbb_inv[],
	int m, int n = 5, bool ifReconstruct = false, bool if_nogps = false);
void SPP_accuracy_ass(POSRES& Res, const EPOCHOBSDATA& obs, const double B[], const double P[],
	const double x[], const double w[], const double Nbb_inv[], const int satnum, const bool ifreconstruct);
bool SPV(EPOCHOBSDATA& obs, POSRES& Res);
bool SPP(EPOCHOBSDATA& obs, SATEPHREC* gpseph, SATEPHREC* bdseph, POSRES& Res);







#endif