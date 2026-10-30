#ifndef NAVFIRST_SPP_H
#define NAVFIRST_SPP_H

#include <iostream>
#include <string>
#include <cmath>
#include <Eigen>
#include "Constants.h"

using namespace std;
using namespace Eigen;

#ifdef __cplusplus
extern "C++" {
#endif

	//时间系统转换
	
	//定义通用时结构体
	//优化1：用short而不用double：年月日时分都是整数，double占8个字节，short占2个字节，short更节省内存
	//优化2：用unsigned short而不用short:月日时分都是正整数，年有可能是负数，所以采用short
	struct CommonTime {
		short Year;//年
		unsigned short Month;//月
		unsigned short Day;//日
		unsigned short Hour;//时
		unsigned short Minute;//分
		double Second;//秒
	};

	//定义简化儒略日结构体
	struct MJDTime {
		int Days;//简化儒略日的整数部分
		double FracDay;//简化儒略日的小数部分
	};

	//定义GPS时结构体
	struct GPSTime {
		unsigned short Week;//GPS周
		double SecOfWeek;//GPS周秒
	};


	//坐标系统转换

    // 空间直角坐标系点的结构体
    typedef struct XYZ {
        double x;
        double y;
        double z;
    } XYZ;

    //大地坐标点的结构体
    typedef struct BLH {
        double B;  // 坐标纬度
        double L; // 坐标经度
        double H;// 坐标高程
    } BLH;

    //地平坐标系点的结构体
    typedef struct NEU {
        double dN;
        double dE;
        double dH;
    }NEU;

    //坐标系的参数
    typedef struct Coord {
        double a;  // 坐标系长半轴
        double le; // 坐标系扁率分母
    } Coord;

	//从16进制文件中解码

	//优势一：枚举将每个GNSS系统的标识定义为具有明确语义的名称，而不是012
	//优势二：枚举限定了可用的值的范围，即只有明确列出的GNSS系统类型是有效的，便于编译减少潜在的错误
	enum GNSSSys { UNKS = 0, GPS, BDS, GLONASS, GALILEO, QZSS };

	//卫星（SAT）观测（OBS）数据（DATA）
	//侧重于接收机与卫星之间的实时信号测量
	typedef struct SATOBSDATA
	{
		short Prn;//伪随机噪声码，用于标识和区分不同的卫星，即每颗卫星的ID。
		GNSSSys System;//表示卫星所属的全球导航卫星系统

		double P[2];//伪距观测值（Pseudorange）数组，包含 2 个元素。
		double L[2];//载波相位观测值（Carrier Phase）数组，包含 2 个元素。
		double D[2]; //多普勒频移（Doppler Frequency）数组

		double cn0[2]; //载波信噪比,用于衡量卫星信号的质量和强度。
		double LockTime[2];//连续跟踪卫星信号的时间（以秒为单位）
		unsigned char half[2];
		bool Valid;//
		SATOBSDATA()
		{
			Prn = 0;
			System = UNKS;
			for (int i = 0; i < 2; i++)
				P[i] = L[i] = D[i] = 0.0;
			Valid = false;
		}
	}SATOBSDATA;

	//卫星（GPS）星历（EPH）记录（RECORD）
	//侧重于提供卫星的轨道信息，是确定卫星位置的基础
	typedef struct GPSEPHREC
	{
		short PRN; //伪随机噪声码，用于标识和区分不同的卫星，即每颗卫星的ID。
		GNSSSys System;//
		GPSTime TOC, TOE;//卫星钟的参考时刻和星历的参考时刻
		double ClkBias, ClkDrift, ClkDriftRate;//钟差钟漂和钟速
		double IODE;//数据星历的发布时间
		double IODC; //钟的数据龄期
		double SqrtA, M0, e, OMEGA, i0, omega;//轨道根数
		double Crs, Cuc, Cus, Cic, Cis, Crc;//轨道摄动参数
		double DeltaN, OMEGADot, iDot;//轨道摄动参数
		int SVHealth;//卫星的健康状态，指示卫星是否正常工作，
		double TGD1, TGD2;//卫星信号的群延迟参数，用于修正卫星信号在穿过电离层和对流层时产生的延迟

		GPSEPHREC() {
			PRN = 0;
			System = UNKS;
			SVHealth = 0;
			ClkBias = ClkDrift = ClkDriftRate = 0.0;
			IODE = IODC = 0.0;
			TGD1 = TGD2 = 0.0;
			SqrtA = e = M0 = OMEGA = i0 = omega = OMEGADot = iDot = DeltaN = 0.0;
			Crs = Cuc = Cus = Cic = Cis = Crc = 0.0;

			// 初始化 TOC 和 TOE
			TOC.Week = TOC.SecOfWeek = 0;
			TOE.Week = TOE.SecOfWeek = 0;
		}
	}GPSEPHREC;

	//定位（POS）结果（RES）结构体
	typedef struct POSRES
	{
		GPSTime Time;//当前时刻
		double Pos[3], Vel[3];//三维位置坐标和三维速度向量
		double RefBLH[3],RefXYZ[3];
		double dXYZ[3];
		double dNEU[3];
		double RcvClkOft[2];               /* 0 为GPS钟差; 1=BDS钟差 */
		double RcvClkSft;
		double PDOP; //位置精度衰减因子
		double SigmaPos, SigmaVel;//位置和速度的标准差
		short  GPSSatNum, BDSSatNum;
		int SatNum;//卫星数量
	}POSRES;

	//卫星位置计算


	//卫星（SAT）位置（P）、速度（V）和钟差（T）等的中间计算结果 */
	typedef struct SATPVT
	{
		double SatPos[3], SatVel[3];//卫星位置和速度
		double SatClkOft, SatClkSft;//卫星钟差和钟速
		double Elevation, Azimuth;//卫星的高度角和方位角
		double TropCorr;//对流层延迟误差
		double Tgd1, Tgd2;//电离层延迟误差、对流层延迟误差
		bool Valid; //false=没有星历或星历过期,true-计算成功
		SATPVT()
		{
			SatPos[0] = SatPos[1] = SatPos[2] = 0.0;
			SatVel[0] = SatVel[1] = SatVel[2] = 0.0;
			Azimuth = 0.0;
			Elevation = pi / 2.0;
			SatClkOft = SatClkSft = 0.0;
			Tgd1 = Tgd2 = TropCorr = 0.0;
			Valid = false;
		}
	}SATPVT;



	//误差改正
	typedef struct MWGF
	{
		short Prn;//卫星号
		GNSSSys Sys;//卫星所在的的系统
		double MW;
		double GF;
		double PIF; //Phase Ionosphere - Free（无电离层）组合观测量
		int n; //平滑计数
		MWGF()
		{
			Prn = n = 0;
			Sys = UNKS;
			MW = GF = PIF = 0.0;
		}
	};


	//历元（EPOCH）观测（OBS）数据（DATA）
	typedef struct EPOCHOBSDATA
	{
		GPSTime Time;//当前历元的GPS时
		short SatNum;//当前历元接收的卫星数
		SATOBSDATA SatObs[MAXCHANNUM];//当前历元接受到的每一个卫星的观测数据
		MWGF ComObs[MAXCHANNUM];
		SATPVT SatPVT[MAXCHANNUM];
		EPOCHOBSDATA()
		{
			SatNum = 0;
		}
	};

	//历元（EPOCH）定位（POS）结果（RES）
	typedef struct RAWDATA
	{
		EPOCHOBSDATA Obs;
		GPSEPHREC GpsEph[MAXGPSNUM];
		GPSEPHREC BdsEph[MAXBDSNUM];
		POSRES Pos;
	};

	/*  每颗卫星的单差观测数据定义  */
	typedef struct SDSATOBS
	{
		short    Prn;
		GNSSSys  System;
		short    Valid;
		double   dP[2], dL[2];   // m
		short    nBas, nRov;   // 存储单差观测值对应的基准和流动站的数值索引号

		SDSATOBS()
		{
			Prn = nBas = nRov = 0;
			System = UNKS;
			dP[0] = dL[0] = dP[1] = dL[1] = 0.0;
			Valid = -1;
		}
	};

	/*  每个历元的单差观测数据定义  */
	typedef struct SDEPOCHOBS
	{
		GPSTime    Time;
		short      SatNum;
		SDSATOBS   SdSatObs[MAXCHANNUM];
		MWGF       SdCObs[MAXCHANNUM];

		SDEPOCHOBS()
		{
			SatNum = 0;
		}
	};

	//函数部分

	//时间系统
	
	//通用时到简化儒略日的转换
	//输入：通用时（年月日时分秒）
	//输出：简化儒略日的整数的小数部分
	int CommonTime2MJDtime(const CommonTime ct, MJDTime* mjd);

	//简化儒略日到通用时的转换
	//输入：简化儒略日的整数的小数部分
	//输出：通用时（年月日时分秒）
	int MJDTime2CommonTime(const MJDTime mjd, CommonTime* ct);

	//简化儒略日到GPS时的转换
	//输入：简化儒略日的整数的小数部分
	//输出：GPS时(GPS周和GPS周秒）
	int MJDTime2GPSTime(const MJDTime mjd, GPSTime* gt);

	//GPS时到简化儒略日的转换
	//输入：GPS时(GPS周和GPS周秒）
	//输出：简化儒略日的整数的小数部分
	int GPSTime2MJDTime(const GPSTime gt, MJDTime* mjd);


	//坐标系统转换
	
	// 返回坐标系椭球扁率f, f=(a-b)/a
	inline double Get_f(Coord& coor) {
		return 1.0 / coor.le;
	}

	// 返回坐标系椭球第一偏心率e, e=sqrt(1-(b/a)?)
	inline double Get_e(double f) {
		return sqrt(1 - pow((1 - f), 2));
	}
	//返回输入纬度下的子午线曲率半径
	inline double Get_M(double a, double phi, double e) {
		return (a * (1 - e * e) / pow((1 - e * e * sin(phi) * sin(phi)), 1.5));
	}

	//返回输入纬度下的卯酉圈曲率半径
	inline double Get_N(double a, double phi, double e) {
		return (a) / pow((1 - e * e * sin(phi) * sin(phi)), 0.5);
	}

	//大地坐标到笛卡尔坐标的转换
	//输入：大地坐标、椭球参数
	//输出：笛卡尔坐标
	int BLH2XYZ(const BLH origin, Coord coord, XYZ& target);

	// XYZ坐标系转换为BLH坐标系
	//输入：笛卡尔坐标、椭球参数
	//输出：大地坐标
	int XYZ2BLH(XYZ origin, Coord coord, BLH* target);

	// 计算转换矩阵 R
	// 输入：测站大地坐标
	// 输出：转换矩阵R
	void BLHToNEUMat(const BLH& blh, double Mat[3][3]);

	// 计算定位误差
	// 输入：卫星xyz坐标，测站xyz坐标,转换矩阵R
	// 输出：dN,dE,dU
	void CompEnuPos(const XYZ& Xs, const XYZ& Xr, double R[3][3], NEU& neu);

	// 计算卫星高度角和方位角
	// 输入：dN,dE,dU
	// 输出：卫星高度角和方位角
	void CompSatElAz(const NEU& dneu, double* Elev, double* Azim);

	int DecodeNovOem7Dat(unsigned char buff[], int& len, EPOCHOBSDATA* obs, GPSEPHREC geph[], GPSEPHREC beph[], POSRES* pos);

	bool CompSatClkOff(const int Prn, const GNSSSys Sys, const GPSTime* t, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, SATPVT* Mid);

	bool CompGPSSatPVT(const int Prn, const GPSTime* t, GPSEPHREC* GPSEph, SATPVT* Mid);

	bool CompBDSSatPVT(const int Prn, GPSTime* t, GPSEPHREC* BDSEph, SATPVT* Mid);

	void EarthRotationCorrection(const Vector3d& receiverPos, SATPVT* satPos, GNSSSys system);
	//利用Hopefield模型进行对流层延迟改正
	//输入：H（测站高度）、Elev（卫星相对于测站的高度角,单位为角度）
	//输出：对流层改正值
	double hopfield(double H, double Elev);

	void CompSatPVTAtSignalTrans(EPOCHOBSDATA* EphObs, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, const Vector3d& receiverPos);

	void DetectOutlier(EPOCHOBSDATA* Obs);

	bool SPP(EPOCHOBSDATA* obs, GPSEPHREC* gpseph, GPSEPHREC* bdseph, POSRES* Res);
	void SPP_Plus(EPOCHOBSDATA* obs, GPSEPHREC* gpseph, GPSEPHREC* bdseph, POSRES* res, const Vector3d& initPos);
	bool SPV(EPOCHOBSDATA& obs, POSRES& Res);

	void CompNEU(POSRES& Res);

	int TimeSynchronization(FILE* FObs_base, FILE* FObs_rover, RAWDATA* base, RAWDATA* rover);
#ifdef __cplusplus
}
#endif

#endif /* NAVFIRST_SPP_H */
#pragma once
