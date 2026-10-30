#include "rtklib.h"

using namespace std;
/* CompSatClkOff ---------------------------------------
* 计算某一颗卫星在某一历元的钟差钟速（无相对论改正）
* args   : Prn      I
*          Sys      I   （GPS/BDS）
*          t        I   卫星发射信号时刻的时间(变量)
*          GPSEph   I   GPS广播星历数组
*          BDSEph   I   BDS广播星历数组
*          Mid      O   中间计算结果:SatClkOft,SatClkSft
*------------------------------------------------------*/
bool CompSatClkOff(const int Prn, const GNSSsys Sys, const GPSTIME* t, SATEPHREC* GPSEph, SATEPHREC* BDSEph, SATMIDRES* Mid)
{
	double mu, omega_e, tk, af[3], MAXtime;
	GPSTIME CurT;
	SATEPHREC* eph = (Sys == GPS) ? (GPSEph + (Prn - 1)) : (BDSEph + (Prn - 1));

	// 卫星不可用
	if (eph->Prn != Prn || eph->SVHealth != 0) {
		Mid->Valid = false;
		return false;
	}

	// 常数设置
	mu = (Sys == GPS) ? GM_WGS : GM_CGCS;
	omega_e = (Sys == GPS) ? Omega_WGS : Omega_CGCS;
	MAXtime = (Sys == GPS) ? LIM_GPST : LIM_BDST;

	// 时间转换为GPS参考
	CurT.Week = (Sys == GPS) ? t->Week : (t->Week - GPST_BDT_Week);
	CurT.SecOfWeek = (Sys == GPS) ? t->SecOfWeek : (t->SecOfWeek - GPST_BDT);

	// 计算与Toc的时间差tk
	tk = GetDiffTime(&CurT, &(eph->TOC));
	if (fabs(tk) > MAXtime) {
		Mid->Valid = false;
		return false;
	}

	// 读取钟差参数
	af[0] = eph->ClkBias;
	af[1] = eph->ClkDrift;
	af[2] = eph->ClkDriftRate;

	// 钟差计算
	Mid->Valid = true;
	Mid->SatClkOft = af[0] + af[1] * tk + af[2] * tk * tk;
	Mid->SatClkSft = af[1] + 2 * af[2] * tk;

	return true;
}
/* CompGPSSatPVT ---------------------------------------
* 计算GPS卫星在某一时刻的位置和钟差
* args   : Prn    I
*          t      I   卫星发射信号时刻的时间(变量)
*          Eph    I   对应卫星的星历
*          Mid    O   中间计算结果:Pos,Vel,Clk钟差，钟速；无发射时刻与地球自转
*------------------------------------------------------*/
bool CompGPSSatPVT(const int Prn, const GPSTIME* t, const SATEPHREC* Eph, SATMIDRES* Mid)
{
	if (Prn<1 || Prn>MAXGPSNUM){
		return false;
	}
	double mu = GM_WGS;
	double omega_e = Omega_WGS;
	Eph += (Prn - 1);

	double A, n0, n, M, tk;
	double E0, E, F;
	double dtr, dot_dtr;
	double v, phi, du, dr, di, u, r, i;
	double x1, y1, xk, yk, zk;
	double Omega, e;
	double COS0, SIN0;
	double sin2phi, cos2phi;
	int inte_num = 0;
	double Edot, phidot, udot, rdot, Idot, Omegadot;
	double x1dot, y1dot, xkdot, ykdot, zkdot;

	/* Position ---------------------------------------------------------------------*/
	A = pow(Eph->SqrtA, 2);
	n0 = sqrt(mu / pow(A, 3));
	tk = GetDiffTime(t, &Eph->TOE);
	n = n0 + Eph->DeltaN;
	M = Eph->M0 + n * tk;
	//迭代计算偏近点角
	E0 = M; E = 0;	//未知数Ek，Mk常数
	e = Eph->e;
	while (fabs(E - E0) > 1e-11)//阈值
	{
		E = E0;
		E0 = M + e * sin(E);
		inte_num += 1;
		if (inte_num > 20)//迭代次数
			break;
	}
	E = E0;
	v = atan2(sin(E) * sqrt(1 - pow(e, 2)), cos(E) - e);
	phi = v + Eph->omega;
	sin2phi = sin(2 * phi);
	cos2phi = cos(2 * phi);
	du = Eph->Cus * sin2phi + Eph->Cuc * cos2phi;
	dr = Eph->Crs * sin2phi + Eph->Crc * cos(2 * phi);
	di = Eph->Cis * sin2phi + Eph->Cic * cos(2 * phi);
	u = phi + du;
	r = A * (1 - e * cos(E)) + dr;
	i = Eph->i0 + Eph->iDot * tk + di;
	//卫星平面位置
	x1 = r * cos(u);
	y1 = r * sin(u);
	Omega = Eph->OMEGA + (Eph->OMEGADot - omega_e) * tk - omega_e * Eph->TOE.SecOfWeek;//改正升交点
	COS0 = cos(Omega), SIN0 = sin(Omega);//计算xyz下位置
	xk = x1 * COS0 - y1 * cos(i) * SIN0;
	yk = x1 * SIN0 + y1 * cos(i) * COS0;
	zk = y1 * sin(i);

	/* Velocity ------------------------------------------------------------------------------------*/
	//钟差相对论改正
	F = -2 * sqrt(mu) / pow(C_Light, 2);
	dtr = F * e * sqrt(A) * sin(E);
	Mid->SatClkOft += dtr;

	Edot = n / (1 - e * cos(E));
	phidot = sqrt(1 - e * e) * Edot / (1 - e * cos(E));
	udot = 2 * (Eph->Cus * cos2phi - Eph->Cuc * sin2phi) * phidot + phidot;
	rdot = A * e * sin(E) * Edot + 2 * (Eph->Crs * cos2phi - Eph->Crc * sin2phi) * phidot;
	Idot = Eph->iDot + 2 * (Eph->Cis * cos2phi - Eph->Cic * sin2phi) * phidot;
	Omegadot = Eph->OMEGADot - omega_e;

	x1dot = rdot * cos(u) - r * udot * sin(u);
	y1dot = rdot * sin(u) + r * udot * cos(u);
	double Mat_r[4] = { x1dot,y1dot,Omegadot,Idot };
	double Rdot[12] = { COS0,-SIN0 * cos(i),-(x1 * SIN0 + y1 * COS0 * cos(i)),y1 * SIN0 * sin(i),
						SIN0,COS0 * cos(i) ,(x1 * COS0 - y1 * SIN0 * cos(i)) ,-y1 * COS0 * sin(i),
						0	 ,sin(i)		 ,0									 ,y1 * cos(i)			};
	double Vk[3];
	mat_mul(Rdot, Mat_r, Vk, 3, 4, 4, 1);
	xkdot = Vk[0];
	ykdot = Vk[1];
	zkdot = Vk[2];

	/* clock rate ----------------------------------------------------------------------------------------------*/
	/*钟速相对论改正*/
	dot_dtr = F * e * cos(E) * Edot;
	Mid->SatClkSft += dot_dtr;

	/* PVT res */
	Mid->Pos[0] = xk;
	Mid->Pos[1] = yk;
	Mid->Pos[2] = zk;
	Mid->Vel[0] = xkdot;
	Mid->Vel[1] = ykdot;
	Mid->Vel[2] = zkdot;
	Mid->tgd1 = Eph->TGD1;

	Eph -= (Prn - 1);
	return true;
}
/* CompBDSSatPVT ---------------------------------------
* 计算BDS卫星在某一时刻的位置和钟差
* args   : 同上
* return : true 成功，false 失败
*------------------------------------------------------*/
bool CompBDSSatPVT(const int Prn, const GPSTIME* t, const SATEPHREC* Eph, SATMIDRES* Mid)
{
	if (Prn<1 || Prn>MAXBDSNUM) {
		return false;
	}
	double mu = GM_CGCS;
	double omega_e = Omega_CGCS;
	Eph += (Prn - 1);//地球椭球设置，指针漂移对应位置

	double A, n0, n, M, tk;
	double E0, E, F;
	double d_tr, d_tr_dot;
	double f, phi, r1, du, dr, di, u, r, i;
	double x1, y1, xk, yk, zk, xgk, ygk, zgk;
	double Omega, e;
	double theta;//omega_e*tk
	int inte_num = 0;
	double RZ[9], RX[9], RZdot[9], RZRX[9], P_GK[3], P_K[3];

	double Edot, phidot, udot, rdot, Idot, Omegadot;
	double sin2phi, cos2phi;
	double x1dot, y1dot, xkdot, ykdot, zkdot;
	double Vk[3], Vgk[3], V1[3], V2[3];


	/* Position ---------------------------------------------------------------------*/
	A = pow(Eph->SqrtA, 2);
	n0 = sqrt(mu / pow(A, 3));			//平均角速度
	GPSTIME t1(t->Week - GPST_BDT_Week, t->SecOfWeek - GPST_BDT);
	tk = GetDiffTime(&t1, &(Eph->TOE));
	n = n0 + Eph->DeltaN;				//对平均角速度改正
	M = Eph->M0 + n * tk;				//平近点角
	E0 = M; E = 0;						//偏近角
	e = Eph->e;
	while (fabs(E - E0) > 1e-11)
	{
		E = E0;
		E0 = M + e * sin(E);
		inte_num += 1;
		if (inte_num > 20)
			break;
	}
	E = E0;
	//真近点角
	f = atan2(sin(E) * sqrt(1 - pow(e, 2)), cos(E) - e);
	//升交角距
	phi = f + Eph->omega;
	//二阶调和改正数
	du = Eph->Cuc * cos(2 * phi) + Eph->Cus * sin(2 * phi);
	dr = Eph->Crc * cos(2 * phi) + Eph->Crs * sin(2 * phi);
	di = Eph->Cic * cos(2 * phi) + Eph->Cis * sin(2 * phi);
	u = phi + du;
	r = A * (1 - e * cos(E)) + dr;
	i = Eph->i0 + Eph->iDot * tk + di;
	// 计算卫星在轨道平面的坐标
	x1 = r * cos(u);
	y1 = r * sin(u);

	if (Prn > 5 && Prn < 59)//MEO/IGSO
	{
		//改正后的升交点经度
		Omega = Eph->OMEGA + (Eph->OMEGADot - omega_e) * tk - omega_e * Eph->TOE.SecOfWeek;
		//地固坐标系
		xk = x1 * cos(Omega) - y1 * cos(i) * sin(Omega);
		yk = x1 * sin(Omega) + y1 * cos(i) * cos(Omega);
		zk = y1 * sin(i);
	}
	else//GEO
	{
		//改正后的升交点经度
		Omega = Eph->OMEGA + Eph->OMEGADot * tk - omega_e * Eph->TOE.SecOfWeek;
		//自定义坐标系下坐标
		P_GK[0] = x1 * cos(Omega) - y1 * cos(i) * sin(Omega);
		P_GK[1] = x1 * sin(Omega) + y1 * cos(i) * cos(Omega);
		P_GK[2] = y1 * sin(i);
		//地固坐标系
		theta = omega_e * tk;
		Rz(RZ, theta);
		Rx(RX, -5.0 * Rad);
		mat_mul(RZ, RX, RZRX, 3, 3, 3, 3);
		mat_mul(RZRX, P_GK, P_K, 3, 3, 3, 1);
		xk = P_K[0];
		yk = P_K[1];
		zk = P_K[2];
	}
	/* Velocity ------------------------------------------------------------------------------------*/
	/*钟差相对论改正*/
	F = -2 * sqrt(mu) / pow(C_Light, 2);
	d_tr = F * e * sqrt(A) * sin(E);
	Mid->SatClkOft += d_tr;

	sin2phi = sin(2 * phi);
	cos2phi = cos(2 * phi);
	Edot = n / (1 - e * cos(E));
	phidot = sqrt(1 - e * e) * Edot / (1 - e * cos(E));
	udot = 2 * (Eph->Cus * cos2phi - Eph->Cuc * sin2phi) * phidot + phidot;
	rdot = A * e * sin(E) * Edot + 2 * (Eph->Crs * cos2phi - Eph->Crc * sin2phi) * phidot;
	Idot = Eph->iDot + 2 * (Eph->Cis * cos2phi - Eph->Cic * sin2phi) * phidot;
	x1dot = rdot * cos(u) - r * udot * sin(u);
	y1dot = rdot * sin(u) + r * udot * cos(u);
	Omegadot = (Prn > 5 && Prn < 59) ? (Eph->OMEGADot - omega_e) : Eph->OMEGADot;
	double cos_O = cos(Omega), sin_O = sin(Omega);
	double Mat_r[4] = { x1dot,y1dot,Omegadot,Idot };
	double Rdot[12] = { cos_O,-sin_O * cos(i),-(x1 * sin_O + y1 * cos_O * cos(i)),y1 * sin_O * sin(i),
			sin_O,cos_O * cos(i),(x1 * cos_O - y1 * sin_O * cos(i)),-y1 * cos_O * sin(i),
			0,sin(i),0,y1 * cos(i) };
	mat_mul(Rdot, Mat_r, Vgk, 3, 4, 4, 1);

	if (Prn > 5 && Prn < 59)//MEO/IGSO
	{
		xkdot = Vgk[0];
		ykdot = Vgk[1];
		zkdot = Vgk[2];
	}
	else//GEO
	{
		Rzdot(RZdot, theta, omega_e);
		mat_mul(RZRX, Vgk, V1, 3, 3, 3, 1);
		mat_mul(RZdot, RX, RZRX, 3, 3, 3, 3);
		mat_mul(RZRX, P_GK, V2, 3, 3, 3, 1);

		vec_plus(V1, V2, Vk, 3);
		xkdot = Vk[0];
		ykdot = Vk[1];
		zkdot = Vk[2];
	}

	/* clock rate ----------------------------------------------------------------------------------------------*/
	/* 钟速相对论改正 */
	d_tr_dot = F * e * cos(E) * Edot;
	Mid->SatClkSft += d_tr_dot;

	/* PVT res */
	Mid->Pos[0] = xk;
	Mid->Pos[1] = yk;
	Mid->Pos[2] = zk;
	Mid->Vel[0] = xkdot;
	Mid->Vel[1] = ykdot;
	Mid->Vel[2] = zkdot;
	Mid->tgd1 = Eph->TGD1;
	Mid->tgd2 = Eph->TGD2;
	Eph -= (Prn - 1);
	return true;
}
/*计算MWGF（窄宽组合）单个卫星处理_用于无电离层组合观测值*/
void compMWGF(const SATOBSDATA sat, MWGF& mwgf)
{
	double f1, f2, b1, b2;
	double L1, L2, P1, P2;
	mwgf.Sys = sat.Satsys;
	mwgf.Prn = sat.Prn;
	f1 = (mwgf.Sys == GPS) ? FG1_GPS : FG1_BDS;
	f2 = (mwgf.Sys == GPS) ? FG2_GPS : FG3_BDS;
	L1 = sat.L[0] ;
	L2 = sat.L[1] ;
	P1 = sat.P[0];
	P2 = sat.P[1];
	mwgf.MW = (f1 * L1 - f2 * L2) / (f1 - f2) - (f1 * P1 + f2 * P2) / (f1 + f2);
	mwgf.GF = L1 - L2;
	mwgf.PIF = (f1 * f1 * P1 - f2 * f2 * P2) / (f1 * f1 - f2 * f2);
}
//double Hopfield(const double H, const double Elev)//对流层
//{
//	//标准气象元素
//	const double H0 = 0.0;//海平面
//	const double T0 = 15 + 273.16;//温度
//	const double p0 = 1013.25;//气压 mbar
//	const double RH0 = 0.5;//相对湿度
//	if (H < -35000 || H>18000)//高度异常
//	{
//		return 0;
//	}
//	//Hopefield模型
//	double RH, p, T, Hf_e, h_w, h_d, K_w, K_d, d_Trop;
//	RH = RH0 * exp(-0.0006396 * (H - H0));
//	p = p0 * pow((1 - 0.0000226 * (H - H0)), 5.225);
//	T = T0 - 0.0065 * (H - H0);
//	Hf_e = RH * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);
//	h_w = 11000;
//	h_d = 40136 + 148.72 * (T0 - 273.16);
//	K_w = 155.2e-7 * 4810 / pow(T, 2) * Hf_e * (h_w - H);
//	K_d = 155.2e-7 * p / T * (h_d - H);
//	//Elev单位为角度
//	d_Trop = K_d / sin((sqrt(pow(Elev * Deg, 2) + 6.25)) * Rad) + K_w / sin((sqrt(pow(Elev * Deg, 2) + 2.25) * Rad));//高度角单位为角度，计算三角函数时转换为弧度
//	return d_Trop;
//}

double Hopfield(const double H, const double Elev)
{
	if (!std::isfinite(H) || !std::isfinite(Elev)) return 0.0;

	// 裁剪高度，避免不合理输入放大
	const double Hc = fmax(-500.0, fmin(10000.0, H));

	// 标准气象参数
	const double H0 = 0.0;
	const double T0 = 15 + 273.16;   // 288.16 K
	const double p0 = 1013.25;       // mbar
	const double RH0 = 0.5;

	// 气象量（数值净化）
	const double RH = RH0 * exp(-0.0006396 * (Hc - H0));
	double base = 1.0 - 0.0000226 * (Hc - H0);
	base = fmax(1e-6, base);                      // 避免负底数
	double p = p0 * pow(base, 5.225);
	double T = T0 - 0.0065 * (Hc - H0);
	T = fmax(180.0, fmin(330.0, T));              // 温度边界，避免 1/T 爆炸

	const double Hf_e = RH * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);
	const double h_w = 11000.0;
	const double h_d = 40136.0 + 148.72 * (T0 - 273.16);

	double K_w = 155.2e-7 * 4810.0 / (T * T) * Hf_e * fmax(0.0, h_w - Hc);
	double K_d = 155.2e-7 * p / T * fmax(0.0, h_d - Hc);

	// 映射函数（防止 sin 极小）
	const double Edeg = Elev * Deg;
	double denom_d = sin(fmax(1e-3, (sqrt(Edeg * Edeg + 6.25)) * Rad));
	double denom_w = sin(fmax(1e-3, (sqrt(Edeg * Edeg + 2.25)) * Rad));

	return (K_d / denom_d) + (K_w / denom_w);
}
double Klobutchar(const GPSTIME* Time, double Elev, double Azim, double RcvPos[3])
{
	return 0;
}

void DetectOutlier(EPOCHOBSDATA& obs)
{
	//一个历元中有多颗卫星，通过循环对其中一颗卫星进行粗差探测。
	//函数内部可先定义MWGF CurComObs[MAXCHANNUM];用于存放当前历元的计算结果，
	//EPOCHOBSDATA结构体中ComObs保存上个历元的平滑结果。
	//对于任意一颗卫星的观测数据，其粗差探测的步骤为
	MWGF CurComObs[MAXCHANNUM];
	for (int i = 0; i < obs.SatNum; i++)
	{
		//1. 检查该卫星的双频伪距和相位数据是否有效和完整，若不全或为0，将Valid标记为false，continue
		if (obs.SatObs[i].P[0] < 1e-9 || obs.SatObs[i].P[1] < 1e-9 || obs.SatObs[i].L[1] < 1e-9 || obs.SatObs[i].L[0] < 1e-9)
		{
			obs.SatObs[i].Valid = false;
			continue;
		}
		//2. 计算当前历元该卫星的GF和MW组合值
		compMWGF(obs.SatObs[i], CurComObs[i]);
		//3. 从上个历元的MWGF数据中查找该卫星的GF和MW组合值
		for (int j = 0; j < MAXCHANNUM; j++) {
			if (obs.ComObs[j].Prn == CurComObs[i].Prn && obs.ComObs[j].Sys == CurComObs[i].Sys)
			{
				//4. 计算当前历元该卫星GF与上一历元对应GF的差值dGF
				//5. 计算当前历元该卫星MW与上一历元对应MW平滑值的差值dMW
				double dMW = CurComObs[i].MW - obs.ComObs[j].MW;
				double dGF = CurComObs[i].GF - obs.ComObs[j].GF;
				//6. 检查dGF和dMW是否超限，限差阈值建议为5cm和3m。若超限，标记为粗差，将Valid标记为
				//false ，若不超限，标记为可用将Valid标记为true，并计算该卫星的MW平滑值
				if (dMW < MAX_dMW && dGF < MAX_dGF)
				{
					CurComObs[i].MW = (obs.ComObs[j].MW * obs.ComObs[j].n + CurComObs[i].MW) / (obs.ComObs[j].n + 1);
					CurComObs[i].n = obs.ComObs[j].n + 1;
					obs.SatObs[i].Valid = true;
				}
				else { obs.SatObs[i].Valid = false; CurComObs[i].n = 0; }
				break;
			}
		}
	}
	//7. 对于可用的观测数据，计算伪距的IF组合观测值，用于SPP 前述已计算，可用的被标记
	//8. 所有卫星循环计算完成之后，将CurComObs内存拷贝到ComObs，即函数运行结束后， ComObs
	//保存了当前历元的GF和MW平滑值。
	memcpy(obs.ComObs, CurComObs, MAXCHANNUM * sizeof(MWGF));
}
//
//void DetectOutlier(EPOCHOBSDATA& obs)
//{
//	MWGF CurComObs[MAXCHANNUM]{};
//	for (int i = 0; i < obs.SatNum; i++)
//	{
//		// 基本可用性检查
//		if (obs.SatObs[i].P[0] <= 0 || obs.SatObs[i].P[1] <= 0 || obs.SatObs[i].L[0] == 0 || obs.SatObs[i].L[1] == 0) {
//			obs.SatObs[i].Valid = false;
//			continue;
//		}
//
//		compMWGF(obs.SatObs[i], CurComObs[i]);
//		bool matched = false;
//
//		for (int j = 0; j < MAXCHANNUM; j++) {
//			if (obs.ComObs[j].Prn == CurComObs[i].Prn && obs.ComObs[j].Sys == CurComObs[i].Sys) {
//				matched = true;
//				double dMW = CurComObs[i].MW - obs.ComObs[j].MW;
//				double dGF = CurComObs[i].GF - obs.ComObs[j].GF;
//				if (fabs(dMW) < MAX_dMW && fabs(dGF) < MAX_dGF) {
//					CurComObs[i].MW = (obs.ComObs[j].MW * obs.ComObs[j].n + CurComObs[i].MW) / (obs.ComObs[j].n + 1);
//					CurComObs[i].n = obs.ComObs[j].n + 1;
//					obs.SatObs[i].Valid = true;
//				}
//				else {
//					obs.SatObs[i].Valid = false;
//					CurComObs[i].n = 0;
//				}
//				break;
//			}
//		}
//
//		// 首次出现或未匹配上一历元：保守认为可用（由后续 SPP 残差再剔除）
//		if (!matched) {
//			obs.SatObs[i].Valid = true;
//			CurComObs[i].n = 1;
//		}
//	}
//	memcpy(obs.ComObs, CurComObs, MAXCHANNUM * sizeof(MWGF));
//}




void Earth_corr(SATMIDRES* Mid, GNSSsys sys, const double dt)
{
	double OMEGAe;
	double RZ[9];
	double Pos[3];
	double Vel[3];
	OMEGAe = (sys == GPS) ? Omega_WGS : Omega_CGCS;
	Dmemcpy(Mid->Pos, Pos, 3);
	Dmemcpy(Mid->Vel, Vel, 3);
	Rz(RZ, OMEGAe * dt);
	mat_mul(RZ, Pos, Mid->Pos, 3, 3, 3, 1);
	mat_mul(RZ, Vel, Mid->Vel, 3, 3, 3, 1);
}
/* ComputeSatPVTAtSignalTrans -----------------------------------------------------------------------------
* 计算信号发射时刻卫星位置计算
* args   : Obs		I	观测数据
*          GpsEph	I	星历数据
*          BDSEph	I	星历数据
*          RcvPos	I	接收机位置
*          Mid		O   中间计算结果:位置\钟差\方位角\高度角\地球自转修正\大气延迟\可用
*----------------------------------------------------------------------------------------------------------*/
void ComputeSatPVTAtSignalTrans(EPOCHOBSDATA& Obs, SATEPHREC* GpsEph, SATEPHREC* BDSEph, double RcvPos[3], SATMIDRES* Mid)
{
	GPSTIME t;
	XYZCOOR Sat_xyz{}, Rec_xyz{};
	BLHCOOR Sat_blh{}, Rec_blh{};
	ELLIPSOID elli;

	for (int n = 0; n < 3; n++) Rec_xyz.xyz[n] = RcvPos[n];

	for (int i = 0; i < Obs.SatNum; i++)
	{
		const int Prn = Obs.SatObs[i].Prn;
		const GNSSsys sys = Obs.SatObs[i].Satsys;

		Mid->Prn = Prn;
		Mid->Sys = sys;

		// 必须是有效观测（至少 P1>0）
		if (!Obs.SatObs[i].Valid || Obs.SatObs[i].P[0] <= 0.0) {
			Mid->Valid = false;
			Mid++;
			continue;
		}

		// 每颗卫星独立的钟差变量（关键：避免跨星串扰）
		double clk = 0.0;

		// 初始发射时刻估计：tr - ρ/c - dt_sv（dt_sv首轮取0）
		const double dt_geom = Obs.SatObs[i].P[0] / C_Light;
		t = GPSTIME(Obs.time.Week, Obs.time.SecOfWeek - dt_geom - clk);

		// 两次迭代钟差，提高发射时刻精度
		if (!CompSatClkOff(Prn, sys, &t, GpsEph, BDSEph, Mid)) { Mid->Valid = false; Mid++; continue; }
		clk = Mid->SatClkOft;
		t = GPSTIME(Obs.time.Week, Obs.time.SecOfWeek - dt_geom - clk);
		if (!CompSatClkOff(Prn, sys, &t, GpsEph, BDSEph, Mid)) { Mid->Valid = false; Mid++; continue; }
		clk = Mid->SatClkOft;
		t = GPSTIME(Obs.time.Week, Obs.time.SecOfWeek - dt_geom - clk);

		// 卫星 PVT（含相对论改正在 Comp*SatPVT 内部累加到 SatClkOft/Sft）
		bool ok = (sys == GPS) ? CompGPSSatPVT(Prn, &t, GpsEph, Mid) : CompBDSSatPVT(Prn, &t, BDSEph, Mid);
		if (!ok) { Mid->Valid = false; Mid++; continue; }

		// 地球自转改正：按信号传播时延旋转卫星位置/速度
		double rho_vec[3]; vec_sub(RcvPos, Mid->Pos, rho_vec, 3);
		const double rho = norm(rho_vec, 3);
		const double dt_rot = rho / C_Light;
		Earth_corr(Mid, sys, dt_rot);

		// 仰角/方位/对流层
		for (int k = 0; k < 3; k++) Sat_xyz.xyz[k] = Mid->Pos[k];
		elli = (sys == GPS) ? WGS84 : CGCS2K;
		XYZ2BLH(Sat_xyz, Sat_blh, elli);
		XYZ2BLH(Rec_xyz, Rec_blh, elli);
		CompSatElAz(Rec_xyz, Sat_xyz, Mid->Elevation, Mid->Azimuth, elli);
		Mid->TronCorr = Hopfield(Rec_blh.blh[2], Mid->Elevation);

		Mid->Valid = true;
		Mid++;
	}
}
//
//void ComputeSatPVTAtSignalTrans(EPOCHOBSDATA& Obs, SATEPHREC* GpsEph, SATEPHREC* BDSEph, double RcvPos[3], SATMIDRES* Mid)
//{
//	double dt;
//	GNSSsys satsys;
//	GPSTIME t;
//	XYZCOOR Sat_xyz, Rec_xyz;
//	BLHCOOR Sat_blh, Rec_blh;
//	ELLIPSOID elli = WGS84;
//	for (int n = 0; n < 3; n++) Rec_xyz.xyz[n] = RcvPos[n];
//
//	for (int i = 0; i < Obs.SatNum; i++)
//	{
//		double clk = 0.0; // 关键：每颗卫星必须重置，避免串扰
//		int Prn = Obs.SatObs[i].Prn;
//		satsys = Obs.SatObs[i].Satsys;
//
//		Mid->Prn = Prn;
//		Mid->Sys = satsys;
//
//		if (!Obs.SatObs[i].Valid) { Mid->Valid = false; Mid++; continue; }
//		if (Obs.SatObs[i].P[0] <= 0 || Obs.SatObs[i].P[1] <= 0) { Mid->Valid = false; Mid++; continue; }
//
//		dt = Obs.SatObs[i].P[0] / C_Light;
//		t = GPSTIME(Obs.time.Week, Obs.time.SecOfWeek - dt - clk);
//
//		if (!CompSatClkOff(Prn, satsys, &t, GpsEph, BDSEph, Mid)) { Mid->Valid = false; Mid++; continue; }
//		clk = Mid->SatClkOft;
//		t = GPSTIME(Obs.time.Week, Obs.time.SecOfWeek - dt - clk);
//		if (!CompSatClkOff(Prn, satsys, &t, GpsEph, BDSEph, Mid)) { Mid->Valid = false; Mid++; continue; }
//		clk = Mid->SatClkOft;
//		t = GPSTIME(Obs.time.Week, Obs.time.SecOfWeek - dt - clk);
//
//		bool ok = (satsys == GPS) ? CompGPSSatPVT(Prn, &t, GpsEph, Mid) : CompBDSSatPVT(Prn, &t, BDSEph, Mid);
//		if (!ok) { Mid->Valid = false; Mid++; continue; }
//
//		// 地球自转改正
//		double res[3];
//		vec_sub(RcvPos, Mid->Pos, res, 3);
//		double dis = norm(res, 3);
//		dt = dis / C_Light;
//		Earth_corr(Mid, satsys, dt);
//
//		// 仰角/对流层
//		for (int j = 0; j < 3; j++) Sat_xyz.xyz[j] = Mid->Pos[j];
//		XYZ2BLH(Sat_xyz, Sat_blh, elli);
//		XYZ2BLH(Rec_xyz, Rec_blh, elli);
//		CompSatElAz(Rec_xyz, Sat_xyz, Mid->Elevation, Mid->Azimuth, elli);
//		Mid->TronCorr = Hopfield(Rec_blh.blh[2], Mid->Elevation);
//
//		Mid->Valid = true;
//		Mid++;
//	}
//}


/*compLSQ: ------------------------------------------------------
* GPS+BDS双系统的最小二乘解算
* args	：	I:	B,P,w,row_B,column_B，if_nogps||if_nobds, if_nogps
*			O:  x,Nbb_
* --------------------------------------------------------------------------------------*/
void compLSQ(const double B[], const double P[], const double w[], double x[], double Nbb_inv[],
	int m, int n, bool ifReconstruct, bool if_nogps)
{
	double* Bt = new double[n * m];
	double* BtP = new double[n * m];
	double Nbb[5 * 5]{}, /*Nbb_[5 * 5]{},*/ W[5]{};
	double Nbb1[4 * 4]{}, /*Nbb1_[4 * 4]{},*/ W1[4]{}, x1[4]{};
	int i, j;

	mat_trans(B, Bt, m, n);
	mat_mul(Bt, P, BtP, 5, m, m, m);
	mat_mul(BtP, B, Nbb, 5, m, m, 5);
	mat_mul(BtP, w, W, 5, m, m, 1);

	auto invert_spd_with_damping = [](double* A, int dim) -> bool {
		// 先尝试直接LDL/Cholesky
		double tmp[25]; // dim<=5，这里足够
		memcpy(tmp, A, sizeof(double) * dim * dim);
		if (mat_Inv_SRS(dim, tmp)) { memcpy(A, tmp, sizeof(double) * dim * dim); return true; }
		// 加阻尼后重试
		double maxd = 0.0;
		for (int i = 0; i < dim; ++i) maxd = std::max(maxd, fabs(A[i * dim + i]));
		double lam = std::max(1e-6, 1e-8 * maxd);
		for (int i = 0; i < dim; ++i) A[i * dim + i] += lam;
		memcpy(tmp, A, sizeof(double) * dim * dim);
		if (mat_Inv_SRS(dim, tmp)) { memcpy(A, tmp, sizeof(double) * dim * dim); return true; }
		return false;
	};

	if (!ifReconstruct)
	{
		// 5×5 直接求逆（SPD + 阻尼），失败则回退为零解
		double Ninv[25];
		memcpy(Ninv, Nbb, sizeof(Ninv));
		if (!invert_spd_with_damping(Ninv, 5)) {
			// 回退：输出零矢量，调用方本轮迭代将不收敛，SPP返回失败
			memset(x, 0, sizeof(double) * 5);
			memset(Nbb_inv, 0, sizeof(double) * 25);
			delete[] Bt; delete[] BtP;
			return;
		}
		Dmemcpy(Ninv, Nbb_inv, 25);
		mat_mul(Nbb_inv, W, x, 5, 5, 5, 1);
	}
	else
	{
		int dt_idx = (if_nogps) ? 4 : 3;  // 当前系统钟差在 Nbb 中的行/列索引（GPS:3, BDS:4）

		// 重构 Nbb 的前 3 行 + dt 当前系统列（共4列），以及 W 前3项
		for (i = 0; i < 3; i++) {
			for (j = 0; j < 3; j++) {
				Nbb1[i * 4 + j] = Nbb[i * 5 + j];              // x,y,z 对应列
			}
			Nbb1[i * 4 + 3] = Nbb[i * 5 + dt_idx];             // 第4列 = 当前系统钟差列
			W1[i] = W[i];                                      // W的前3项
		}
		for (j = 0; j < 3; j++) {
			Nbb1[3 * 4 + j] = Nbb[dt_idx * 5 + j];             // 当前系统钟差行的 x,y,z
		}
		Nbb1[15] = Nbb[dt_idx * 5 + dt_idx];                   // 当前系统钟差行的自身协方差
		W1[3] = W[dt_idx];                                     // W 的当前系统钟差项

		// 4×4 SPD + 阻尼
		double N1inv[16];
		memcpy(N1inv, Nbb1, sizeof(N1inv));
		if (!invert_spd_with_damping(N1inv, 4)) {
			memset(x, 0, sizeof(double) * 5);
			memset(Nbb_inv, 0, sizeof(double) * 25);
			delete[] Bt; delete[] BtP;
			return;
		}
		// x1 = N1inv * W1
		mat_mul(N1inv, W1, x1, 4, 4, 4, 1);
		Dmemcpy(x1, x, 3);
		if (if_nogps) { x[4] = x1[3]; x[3] = 0; }
		else { x[3] = x1[3]; x[4] = 0; }
		// Nbb_inv 仅用于后续精度评定，按 4×4 结果填充左上角
		memset(Nbb_inv, 0, sizeof(double) * 25);
		for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) Nbb_inv[r * 5 + c] = N1inv[r * 4 + c];
		Nbb_inv[(if_nogps ? 4 : 3) * 5 + (if_nogps ? 4 : 3)] = N1inv[15];
	}

	delete[] Bt; delete[] BtP;
}


//void compLSQ(const double B[], const double P[], const double w[], double x[], double Nbb_inv[],
//	int m, int n, bool ifReconstruct, bool if_nogps)
//{
//	auto invert_with_tikhonov = [](const double* A, double* Ainv, int dim) -> bool {
//		double M[25]{}, Ainvt[25]{};
//		double diagSum = 0.0; for (int i = 0; i < dim; ++i) diagSum += A[i * dim + i];
//		double diagAvg = (diagSum == 0.0) ? 1.0 : fabs(diagSum) / dim;
//		const double scales[] = { 0.0, 1e-8, 1e-6, 1e-4, 1e-2 };
//		for (double s : scales) {
//			memcpy(M, A, sizeof(double) * dim * dim);
//			if (s > 0.0) for (int i = 0; i < dim; ++i) M[i * dim + i] += s * diagAvg;
//			if (mat_inv(M, Ainvt, dim) == 0) { memcpy(Ainv, Ainvt, sizeof(double) * dim * dim); return true; }
//		}
//		return false;
//		};
//
//	double* Bt = new double[n * m]();
//	double* BtP = new double[n * m]();
//	double* Wr = new double[n]();
//	double  Nbb[25]{};
//	double  Nbb1[16]{}, W1[4]{}, x1[4]{};
//
//	mat_trans(B, Bt, m, n);
//	mat_mul(Bt, P, BtP, n, m, m, m);
//	mat_mul(BtP, B, Nbb, n, m, m, n);
//	mat_mul(BtP, w, Wr, n, m, m, 1);
//
//	if (!ifReconstruct) {
//		double Ninv[25]{};
//		if (!invert_with_tikhonov(Nbb, Ninv, n)) {
//			memset(x, 0, sizeof(double) * n);
//			memset(Nbb_inv, 0, sizeof(double) * n * n);
//			delete[] Bt; delete[] BtP; delete[] Wr; return;
//		}
//		memcpy(Nbb_inv, Ninv, sizeof(double) * n * n);
//		mat_mul(Ninv, Wr, x, n, n, n, 1);
//	}
//	else {
//		const int dt_idx = if_nogps ? 4 : 3;
//		for (int i = 0; i < 3; i++) {
//			for (int j = 0; j < 3; j++) Nbb1[i * 4 + j] = Nbb[i * n + j];
//			Nbb1[i * 4 + 3] = Nbb[i * n + dt_idx];
//			W1[i] = Wr[i];
//		}
//		for (int j = 0; j < 3; j++) Nbb1[3 * 4 + j] = Nbb[dt_idx * n + j];
//		Nbb1[15] = Nbb[dt_idx * n + dt_idx];
//		W1[3] = Wr[dt_idx];
//
//		double N1inv[16]{};
//		if (!invert_with_tikhonov(Nbb1, N1inv, 4)) {
//			memset(x, 0, sizeof(double) * n);
//			memset(Nbb_inv, 0, sizeof(double) * n * n);
//			delete[] Bt; delete[] BtP; delete[] Wr; return;
//		}
//		mat_mul(N1inv, W1, x1, 4, 4, 4, 1);
//		memset(x, 0, sizeof(double) * n);
//		Dmemcpy(x1, x, 3);
//		if (if_nogps) x[4] = x1[3]; else x[3] = x1[3];
//
//		memset(Nbb_inv, 0, sizeof(double) * n * n);
//		for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) Nbb_inv[r * n + c] = N1inv[r * 4 + c];
//		Nbb_inv[dt_idx * n + dt_idx] = N1inv[15];
//	}
//
//	delete[] Bt; delete[] BtP; delete[] Wr;
//}


bool SPV(EPOCHOBSDATA& obs, POSRES& Res)
{
	double B[MAXCHANNUM * 4]{}, P[MAXCHANNUM * MAXCHANNUM]{}, w[MAXCHANNUM]{}, v[MAXCHANNUM]{};
	double Bt[4 * MAXCHANNUM]{}, BtP[4 * MAXCHANNUM]{}, Nbb[4 * 4]{}, Nbb_[4 * 4]{}, W[MAXCHANNUM]{};
	double vt[1 * MAXCHANNUM]{}, Bx[MAXCHANNUM * 1]{}, vtP[1 * MAXCHANNUM]{};
	double X[4]{ 1,0,0,0 }, d_x[3]{};
	int satnum;
	int i, j;
	double rou, roudot, tdot, b;
	double sigma = 0;

	satnum = 0;
	for (i = 0; i < obs.SatNum; i++)
	{
		if (!obs.SatObs[i].Valid || !obs.SatPVT[i].Valid) continue;
		satnum += 1;  j = satnum - 1;
		//线性化方程
		vec_sub(Res.Pos, obs.SatPVT[i].Pos, d_x, 3);
		rou = norm(d_x, 3);
		roudot = -vec_dot_mul(d_x, obs.SatPVT[i].Vel, 3) / rou;
		//填充B矩阵
		for (int k = 0; k < 3; k++) { B[j * 4 + k] = d_x[k] / rou; }
		B[j * 4 + 3] = 1.0;
		//填充w矩阵
		w[j] = obs.SatObs[i].D[0]  - (roudot - C_Light * obs.SatPVT[i].SatClkSft);
	}
	Dmemcpy(eye(satnum), P, satnum*satnum);

	//卫星数量判断
	if (!(satnum >= 4)) { return false; }
	mat_trans(B, Bt, satnum, 4);
	mat_mul(Bt, P, BtP, 4, satnum, satnum, satnum);
	mat_mul(BtP, B, Nbb, 4, satnum, satnum, 4);
	mat_inv(Nbb, Nbb_, 4);
	mat_mul(BtP, w, W, 4, satnum, satnum, 1);
	mat_mul(Nbb_, W, X, 4, 4, 4, 1);
	Dmemcpy(X, Res.Vel, 3);
	//SIGMA_VEL
	for (i = 0; i < satnum; i++)
	{
		v[i] = B[i * 4 + 0] * X[0] + B[i * 4 + 1] * X[1] + B[i * 4 + 2] * X[2] + X[3] - w[i];
	}
	mat_trans(v, vt, satnum, 1);
	mat_mul(vt, P, vtP, 1, satnum, satnum, satnum);
	sigma = sqrt(vec_dot_mul(vtP, v, satnum) / (satnum - 4));
	Res.sigma_vel = sigma;
	return true;
}
bool SPP(EPOCHOBSDATA& obs, SATEPHREC* gpseph, SATEPHREC* bdseph, POSRES& Res)
{
	//① 设定初始位置𝑋𝑅0
	double Pos[3] = { 0,0,6400000 }, d_x[3];
	double B[MAXCHANNUM * 5]{}, P[MAXCHANNUM * MAXCHANNUM]{}, w[MAXCHANNUM]{};
	double Nbb[5 * 5]{}, Nbb_inv[5 * 5]{};
	double x[5]{ 1,0,0,0,0 }, X[5]{};
	int gpsnum = 0, bdsnum = 0, satnum = 0;
	int prn, iter = 0, lines,j,i;
	double rou, tgd, trop, clk, clk_r, fk = 1 / (1 - 1 / FC13R2);
	bool ifreconstruct, if_nogps;
	GNSSsys sys;
	Dmemcpy(Res.Pos, Pos, 3);//防止未进行初始化
	Dmemcpy(Res.Pos, X, 3);

	do
	{//② 计算信号发射时刻的卫星位置和钟差 𝑋𝑗(𝑡𝑅, 𝑖 − 𝛥𝑡) 、𝑌𝑗(𝑡𝑅, 𝑖 − 𝛥𝑡) 、𝑍𝑗(𝑡𝑅, 𝑖 −𝛥𝑡)、𝛿𝑡𝑖𝑗(𝑡𝑅, 𝑖 − 𝛥𝑡ቁ，计算地球自转改正和对流层延迟𝑇𝑗 等
		memset(B, 0, sizeof(B));
		memset(w, 0, sizeof(w));
		ComputeSatPVTAtSignalTrans(obs, gpseph, bdseph, Pos, obs.SatPVT);
		gpsnum = bdsnum = satnum = 0;
		ifreconstruct = if_nogps = false;
		for (i = 0; i < obs.SatNum; i++)
		{
			if (!obs.SatObs[i].Valid || !obs.SatPVT[i].Valid) {
				continue;
			}
			prn = obs.SatObs[i].Prn, sys = obs.SatObs[i].Satsys;
			trop = obs.SatPVT[i].TronCorr;
			clk = obs.SatPVT[i].SatClkOft;
			//③ 对所有卫星的观测数据进行线性化,建立观测方程
				//① 卫星位置计算失败、观测数据不完整或有粗差，不参与定位计算
				//② 以初始位置为参考，对观测方程线性化，计算观测系数矩阵𝐵𝑛∗5和残差向量𝑤 𝑎 + 𝑏 ∗1，统计参与定位的各系统卫星数和所有卫星数
			vec_sub(Pos, obs.SatPVT[i].Pos, d_x, 3);
			rou = norm(d_x, 3);
			satnum += 1;   j = satnum - 1;//矩阵的构造
			for (int k = 0; k < 3; k++) { B[j * 5 + k] = d_x[k] / rou; }
			if (sys == GPS)
			{
				gpsnum += 1;//更新
				B[j * 5 + 3] = 1;
				tgd = 0;
				clk_r = X[3];
			}
			else if (sys == BDS)
			{
				bdsnum += 1;
				B[j * 5 + 4] = 1;
				tgd = obs.SatPVT[i].tgd1 * fk;
				clk_r = X[4];
			}
			w[j] = obs.ComObs[i].PIF - (rou + clk_r - C_Light * clk + trop + C_Light * tgd);
		}
		Dmemcpy(eye(satnum), P, satnum * satnum);


		//④ 卫星总数是否大于未知参数数量，如果卫星数不足，直接返回定位失败。
		if (!(gpsnum == 0 && bdsnum >= 4 || bdsnum == 0 && gpsnum >= 4 || satnum >= 5)) { return false; }
		ifreconstruct = (gpsnum == 0 && bdsnum >= 4 || bdsnum == 0 && gpsnum >= 4);
		if_nogps = (gpsnum == 0);
		//⑤ 如果GPS或BDS卫星数量为0，重构法方程矩阵N和Y,⑥ 最小二乘求解 x = (𝐁𝑇𝐏𝐁)-1𝑩𝑻𝐏𝐰
		compLSQ(B, P, w, x, Nbb_inv, satnum, 5, ifreconstruct, if_nogps);
		//参数更新
		double temp1[5];
		vec_plus(X, x, temp1, 5);
		Dmemcpy(temp1, X, 5);
		double temp2[3];
		vec_plus(Pos, x, temp2, 3);
		Dmemcpy(temp2, Pos, 3);
		iter += 1;

	} while (norm(x, 3) > 1e-4 && iter < 20);
	//⑦ 检查𝑥ො是否收敛，如果没有收敛，将初始位置设定为Xr=Xr+est
	//⑧ 迭代计算②③④⑤⑥⑦⑧，直到收敛
	if (iter == 20) { return false; }
	Dmemcpy(Pos, Res.Pos, 3);
	//⑨ 定位精度评价，计算PDOP和标准差。
	SPP_accuracy_ass(Res, obs, B, P, x, w, Nbb_inv, satnum, ifreconstruct);

	Res.clk[0] = X[3];
	Res.clk[1] = X[4];
	Res.Time = obs.time;
	Res.SatNum = satnum;
	Res.GPSNum = gpsnum;
	Res.BDSNum = bdsnum;
	return true;

}

//计算Qxx、mxyz、PDOP，sigma_Pos

//void SPP_accuracy_ass(POSRES& Res, const EPOCHOBSDATA& obs, const double B[], const double P[],
//	const double x[], const double w[], const double Nbb_inv[], const int satnum, const bool ifreconstruct)
//{
//	int lines = (ifreconstruct) ? 4 : 5;
//	double PDOP = 0.0;
//	double v[MAXCHANNUM]{};
//	double vt[1 * MAXCHANNUM]{}, vtP[1 * MAXCHANNUM]{}; double vtpv[1]{};
//
//	for (int i = 0; i < satnum; i++) {
//		v[i] = B[i * 5 + 0] * x[0] + B[i * 5 + 1] * x[1] + B[i * 5 + 2] * x[2] + B[i * 5 + 3] * x[3] + B[i * 5 + 4] * x[4] - w[i];
//	}
//
//	mat_trans(v, vt, satnum, 1);
//	mat_mul(vt, P, vtP, 1, satnum, satnum, satnum);
//	mat_mul(vtP, v, vtpv, 1, satnum, satnum, 1);// vtpv = v^T *P*v
//	Res.sigma_pos = sqrt(vtpv[0] / (satnum - lines));
//
//	// 计算Qxx、mxyz、PDOP
//	for (int i = 0; i < 3; i++) {
//		PDOP += Nbb_inv[i * lines + i];
//		Res.std_xyz[i] = sqrt(Nbb_inv[i * lines + i]) * Res.sigma_pos;
//		for (int j = 0; j < 3; j++) { Res.Qxx[i * 3 + j] = Nbb_inv[i * lines + j]; }
//	}
//	Res.PDOP = sqrt(PDOP);
//
//	XYZCOOR P_ref(Res.Pos[0], Res.Pos[1], Res.Pos[2]);
//	BLHCOOR blh_ref;
//	XYZ2BLH(P_ref, blh_ref);
//
//	//double R[9], Rt[9], double RtQ[9], Qnn[9];
//	//BLH2EnuMat(blh_ref, R);
//	//mat_trans(R, Rt, 3, 3);         // R^T
//	//mat_mul(Rt, Res.Qxx, RtQ, 3, 3, 3, 3);
//	//mat_mul(RtQ, R, Qnn, 3, 3, 3, 3);
//
//	//for (int i = 0; i < 3; i++) {
//	//	Res.std_enu[i] = Res.sigma_pos * sqrt(Qnn[i * 3 + i]);  // 东、北、天方向误差
//	//}
//	// 构造 ENU 转换矩阵 R（3x3）
//	double R[9], Rt[9], RtQ[9], Qenu[9];
//	BLH2EnuMat(blh_ref, R);            // R 从 ENU -> XYZ
//	mat_trans(R, Rt, 3, 3);            // Rt 从 XYZ -> ENU
//
//	// Qenu = Rt * Qxx * R
//	mat_mul(Rt, Res.Qxx, RtQ, 3, 3, 3, 3);
//	mat_mul(RtQ, R, Qenu, 3, 3, 3, 3);
//
//	// 提取 ENU 三方向标准差
//	for (int i = 0; i < 3; i++) {
//		Res.std_enu[i] = Res.sigma_pos * sqrt(Qenu[i * 3 + i]);  // σ_E, σ_N, σ_U
//	}
//}

void SPP_accuracy_ass(POSRES& Res, const EPOCHOBSDATA& obs, const double B[], const double P[],
	const double x[], const double w[], const double Nbb_inv[], const int satnum, const bool ifreconstruct)
{
	const int stride = 5;              // Nbb_inv 始终按 5×5 存
	const int dof = ifreconstruct ? 4 : 5;

	double v[MAXCHANNUM]{}, vt[1 * MAXCHANNUM]{}, vtP[1 * MAXCHANNUM]{}, vtpv[1]{};
	for (int i = 0; i < satnum; i++) {
		v[i] = B[i * 5 + 0] * x[0] + B[i * 5 + 1] * x[1] + B[i * 5 + 2] * x[2]
			+ B[i * 5 + 3] * x[3] + B[i * 5 + 4] * x[4] - w[i];
	}
	mat_trans(v, vt, satnum, 1);
	mat_mul(vt, P, vtP, 1, satnum, satnum, satnum);
	mat_mul(vtP, v, vtpv, 1, satnum, satnum, 1);
	Res.sigma_pos = sqrt(std::max(0.0, vtpv[0]) / std::max(1, satnum - dof));

	double PDOP = 0.0;
	for (int i = 0; i < 3; i++) {
		PDOP += Nbb_inv[i * stride + i];
		Res.std_xyz[i] = sqrt(std::max(0.0, Nbb_inv[i * stride + i])) * Res.sigma_pos;
		for (int j = 0; j < 3; j++) Res.Qxx[i * 3 + j] = Nbb_inv[i * stride + j];
	}
	Res.PDOP = sqrt(std::max(0.0, PDOP));

	XYZCOOR P_ref(Res.Pos[0], Res.Pos[1], Res.Pos[2]); BLHCOOR blh_ref;
	XYZ2BLH(P_ref, blh_ref);
	double R[9], Rt[9], RtQ[9], Qenu[9];
	BLH2EnuMat(blh_ref, R);
	mat_trans(R, Rt, 3, 3);
	mat_mul(Rt, Res.Qxx, RtQ, 3, 3, 3, 3);
	mat_mul(RtQ, R, Qenu, 3, 3, 3, 3);
	for (int i = 0; i < 3; i++) Res.std_enu[i] = Res.sigma_pos * sqrt(std::max(0.0, Qenu[i * 3 + i]));
}




int pntpos(EPOCHOBSDATA* obs, SATEPHREC* gpseph, SATEPHREC* bdseph, POSRES* res)
{
	// 外点检测
	DetectOutlier(*obs);

	// 调用 SPP 解算函数
	if (!SPP(*obs, gpseph, bdseph, *res)) {
		cerr << "SPP 解算失败" << endl;
		return -1;  // SPP 解算失败，返回错误码
	}

	// 调用 SPV 解算函数
	if (!SPV(*obs, *res)) {
		cerr << "SPV 解算失败" << endl;
		return -2;  // SPV 解算失败，返回错误码
	}
	return 0;
}

