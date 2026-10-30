#include"Structs.h"
#include"vector_matrix.h"
using namespace std;
//void PrintNEMA(GPSTIME gt, bool isFixed, const double XYZ[], const int nsat, const char OutputFilePath[]);
/*测量更新
*[In] X(n*1),P(n*n),H(m*n),v(m*1),R(m*m)
*[In] n-->3+2*satnum_curr ,Xkk_1-->n*1
*[In] m-->4*satnum_curr,H-->m*n,R-->m*m
*[Out]X(n*1),P(n*n)
*/
int filter(double *x, double *P, const double *H, const double *v, const double *R, int n, int m, FILE* fmout)
{	//测量更新
	double *H_, *PH_, *HPHt, *Nbb_, *K, *KH, *I, *IP, *IPI_, *KRK_, *dX;
	H_ = new double[n*m]; PH_ = new double[n*m]; HPHt = new double[m*m]; Nbb_ = new double[m*m];
	K = new double[n*m];  KH = new double[n*n]; KRK_ = new double[n*n];
	I = new double[n*n];  IPI_ = new double[n*n];	dX = new double[n];

	//2.1 K
	//Pkk_1*H^T
	matrix_trans(H, H_, m, n);
	matrix_multipy(P, H_, PH_, n, n, n, m);
	//(HPH^T+R)^(-1)
	BPBt(H, P, HPHt, m, n);
	vector_addin(HPHt, R, m*m);//HPH^T+R
	if (fmout != nullptr)
	{
		fprintf(fmout, "H\n");
		fprint_matrix(fmout, H, m, n);
		//fprint_matrix(fmout,P, n, n);
		fprintf(fmout, "HPHt\n");
		fprint_matrix(fmout, HPHt, m, m);
		//fprint_matrix(fmout, R, m, m);
	}

	/*print_matrix(v, 1, m);

	print_matrix(R, m, m);*/
	if (matrix_inv(HPHt, Nbb_, m) != 1)
	{
		delete[] H_; delete[] PH_; delete[] HPHt; delete[] Nbb_;
		delete[] K; delete[] KH; delete[] KRK_;
		delete[] I; delete[] IPI_; delete[] dX;
		return -1;
	}
	matrix_multipy(PH_, Nbb_, K, n, m, m, m);

	//2.2 Xk=Xkk_1+K*v
	matrix_multipy(K, v, dX, n, m, m, 1);
	vector_addin(x, dX, n);

	//2.3 P=(I-KH)P(I-KH)^T+KPK^T
	//(I-KH)*P*(I-KH)^T
	matrix_multipy(K, H, KH, n, m, m, n);
	eye(I, n);
	vector_subin(I, KH, n*n);
	BPBt(I, P, IPI_, n, n);
	//KPK^T
	BPBt(K, R, KRK_, n, m);
	vector_plus(IPI_, KRK_, P, n*n);
	if (fmout != nullptr)
	{
		fprintf(fmout, "K\n");
		fprint_matrix(fmout, K, n, m);
		fprintf(fmout, "P after MeasureUpdate\n");
		fprint_matrix(fmout, P, n, n);
	}
	delete[] H_; delete[] PH_; delete[] HPHt; delete[] Nbb_;
	delete[] K; delete[] KH; delete[] KRK_;
	delete[] I; delete[] IPI_; delete[] dX;
	return 1;
}
/*
对状态协方差矩阵P进行初始化
*/
void InitRtkEkf(RAWDAT *raw, POSRES* RovSpp, RTKEKF* ekf)
{
	int i, j, k;
	double wl1, wl2;
	ekf->IsInit = false;
	//双系统没有没有参考星，跳过
	if (ekf->CurDDObs.RefPos[0] == -1 && ekf->CurDDObs.RefPos[1] == -1)return;
	//对xyz的协方差初始化
	ekf->CurDDObs.Ambnum = ekf->CurDDObs.Sats * 2;
	zeros(ekf->P, (3 + 2 * MAXCHANNUM)*(3 + 2 * MAXCHANNUM));
	for (i = 0; i < 3; i++)
	{
		ekf->X[i] = RovSpp->Pos[i];
		ekf->P[i*(ekf->CurDDObs.Ambnum + 3) + i] = 200;
	}
	//对模糊度进行初始化

	for (k = 3, i = 0; i < raw->SdObs.SatNum; i++)
	{
		j = raw->SdObs.SdSatObs[i].System == GPS ? 0 : 1;
		wl1 = j == 0 ? WL1_GPS : WL1_BDS;
		wl2 = j == 0 ? WL2_GPS : WL3_BDS;
		ekf->Index[i] = -1;
		//无参考星，参考星本星
		if (ekf->CurDDObs.RefPos[j] == -1 || ekf->CurDDObs.RefPos[j] == i)continue;
		//周跳、粗差、半周标记
		if (!raw->SdObs.SdSatObs[i].Valid) continue;
		if (!raw->SdObs.SdSatObs[i].freValid[0] || !raw->SdObs.SdSatObs[i].freValid[1])continue;
		int nBas = raw->SdObs.SdSatObs[i].nBas;
		int nRov = raw->SdObs.SdSatObs[i].nRov;
		if (!raw->OBS_rover.SatPVT[nRov].Valid || !raw->OBS_base.SatPVT[nBas].Valid) continue;
		int refidx[2] = { ekf->CurDDObs.RefPos[0],ekf->CurDDObs.RefPos[1] };
		//ekf->X[k] = (-(raw->SdObs.SdSatObs[i].dP[0] - raw->SdObs.SdSatObs[refidx[j]].dP[0]) / wl1
		//	+ (raw->SdObs.SdSatObs[i].dL[0] - raw->SdObs.SdSatObs[refidx[j]].dL[0]));
		//ekf->X[k + 1] = (-(raw->SdObs.SdSatObs[i].dP[1] - raw->SdObs.SdSatObs[refidx[j]].dP[1]) / wl2
		//	+ (raw->SdObs.SdSatObs[i].dL[1] - raw->SdObs.SdSatObs[refidx[j]].dL[1]));
		ekf->P[k*(ekf->CurDDObs.Ambnum + 3) + k] = 400;
		ekf->P[(k + 1)*(ekf->CurDDObs.Ambnum + 3) + k + 1] = 400;
		ekf->Index[i] = k;
		k += 2;

	}
	//print_matrix(ekf->X, 1, ekf->CurDDObs.Ambnum + 3);
	//print_matrix(ekf->P, ekf->CurDDObs.Ambnum + 3, ekf->CurDDObs.Ambnum + 3);
	ekf->IsInit = true;
}

/***
对双差模糊度的 参数 进行时间更新，更新转移矩阵H和过程噪声Q
[In]raw:当前历元的数据
[In]ekf:上一历元的单差，上一历元和当前历元的双差
[Out]H
[Out]Q
[Out]ekf.nPos新升起的卫星在SD中的索引
*/
void UpdateDDAmb(RAWDAT* raw, RTKEKF* ekf, double* H, double* Q)
{
	//1.参考星是否变化？
	//2.卫星升起降落
	int dRef[2] = { -1,-1 };//-1-->参考星没有改变；num-->参考星发生改变，新的参考星（的模糊度）在上一个历元X中的位置（>=3）
	int currIdx[MAXCHANNUM];//当前历元的模糊度的索引
	int k = -1;//
	for (int i = 0; i < MAXCHANNUM; i++) { currIdx[i] = -1; ekf->nPos[i] = -1; }
	//1.参考星是否有变化
	for (int i = 0; i < 2; i++)
	{
		//如果 本历元/上个历元 没有参考星/没有该系统，跳过
		if (ekf->CurDDObs.RefPrn[i] == -1 || ekf->DDObs.RefPrn[i] == -1)continue;
		//如果 参考星没有变化，跳过
		if (ekf->CurDDObs.RefPrn[i] == ekf->DDObs.RefPrn[i]) { dRef[i] = -1; }
		else
		{//参考星变化
			int Prn = ekf->CurDDObs.RefPrn[i];
			GNSSsys sys = (i == 0) ? GPS : BDS;
			//当前历元的参考星，在上一历元的位置
			k = SearchSatInSDSatList(Prn, sys, &(ekf->SDObs));
			//如果参考星在上一个历元就参与了解算
			if (k != -1)dRef[i] = ekf->Index[k];
			//如果该参考卫星是这个历元才可以使用
			//方法1：将Isinit重置为false，重新最小二乘计算
			//方法2：修改参考星选择函数，使这种情况不允许为参考星
		}
	}
	//2.计算位置的卫星
	ekf->nSats = 0;
	int n = 3; int i = 0;
	for (n = 3, i = 0; i < raw->SdObs.SatNum; i++)
	{
		int j = raw->SdObs.SdSatObs[i].System == GPS ? 0 : 1;
		if (raw->SdObs.SdSatObs[i].Valid < 1)continue;
		if (raw->SdObs.SdSatObs[i].freValid[0] < 1 || raw->SdObs.SdSatObs[i].freValid[1] < 1)continue;
		if (ekf->CurDDObs.RefPrn[j] == -1)continue;//无参考星
		if (ekf->CurDDObs.RefPos[j] == i)continue;//当前星为参考星
		int Prn = raw->SdObs.SdSatObs[i].Prn;
		GNSSsys sys = raw->SdObs.SdSatObs[i].System;
		//Prn在上个历元单差组合观测值的索引
		k = SearchSatInSDSatList(Prn, sys, &(ekf->SDObs));
		//2.1新升起
		if (k == -1 || (k != -1 && ekf->Index[k] == -1))
		{
			ekf->nPos[ekf->nSats] = i;
			ekf->nSats++;
			Q[n] = Q[n + 1] = 36.0;
			currIdx[i] = n;
			n += 2;
			continue;
		}
		//2.2原有卫星
		//2.2.1参考星没有变化
		if (dRef[j] == -1)
		{
			H[n*(ekf->DDObs.Sats * 2 + 3) + ekf->Index[k]] = 1.0;
			H[(n + 1)*(ekf->DDObs.Sats * 2 + 3) + ekf->Index[k] + 1] = 1.0;
			Q[n] = Q[n + 1] = 1E-8;
		}
		else//2.2.2参考星变化
		{
			H[n*(ekf->DDObs.Sats * 2 + 3) + ekf->Index[k]] = 1.0;
			H[n*(ekf->DDObs.Sats * 2 + 3) + dRef[j]] = -1.0;
			H[(n + 1)*(ekf->DDObs.Sats * 2 + 3) + ekf->Index[k] + 1] = 1.0;
			H[(n + 1)*(ekf->DDObs.Sats * 2 + 3) + dRef[j] + 1] = -1.0;
			Q[n] = Q[n + 1] = 1E-8;
		}
		currIdx[i] = n;
		n += 2;

	}
	//

	//对index进行更新？
	vector_copy(currIdx, ekf->Index, MAXCHANNUM);

}

/*****************时间更新+测量更新*****************/

/*
对RTK结构体进行时间更新
1.更新滤波状态（流动站位置，双差模糊度）---调用UpdateDDAmb获取Psi,Q
2.时间更新 ---Xkk_1=Phi*Xk_1
3.卫星升起、卫星粗差 ---对这些卫星进行初始化模糊度
*/
bool RTKTimeUpdate(RAWDAT* raw, POSRES* Bas, POSRES* Rov, RTKEKF* ekf, FILE* fmout)
{
	double *Psi, *Q;
	int paramnum = ekf->CurDDObs.Sats * 2 + 3;
	if (ekf->CurDDObs.Sats < 3)return false;
	if (ekf->DDObs.Sats < 3)return false;
	Psi = new double[paramnum*(ekf->DDObs.Sats * 2 + 3)]{ 0.0 };
	Q = new double[paramnum] {0.0};
	double wl1, wl2;
	//xyz
	for (int i = 0; i < 3; i++)
	{
		Psi[i*(ekf->DDObs.Sats * 2 + 3) + i] = 1.0;
		Q[i] = 1296.0;
		//Q[i] = 100;
	}

	UpdateDDAmb(raw, ekf, Psi, Q);
	if (fmout)
	{
		fprint_matrix(fmout, Psi, paramnum, ekf->DDObs.Sats * 2 + 3);
		fprint_matrix(fmout, Q, 1, paramnum);
	}

	//时间更新
	zeros(ekf->X0, 2 * MAXCHANNUM + 3);
	zeros(ekf->P0, (2 * MAXCHANNUM + 3)*(2 * MAXCHANNUM + 3));
	//Xkk_1=H*XK_1（位置+模糊度，位置之后会之间使用SPP结果）
	matrix_multipy(Psi, ekf->X, ekf->X0, paramnum, ekf->DDObs.Sats * 2 + 3, ekf->DDObs.Sats * 2 + 3, 1);
	BPBt(Psi, ekf->P, ekf->P0, paramnum, ekf->DDObs.Sats * 2 + 3);
	for (int i = 0; i < paramnum; i++)ekf->P0[i*paramnum + i] += Q[i];

	//初始化新升起卫星模糊度
	//ekf->nSats:新升起的卫星数
	//ekf->nPos[]:新升起的卫星的位置索引
	for (int i = 0; i < ekf->nSats; i++)
	{
		int k = ekf->nPos[i];
		int j = raw->SdObs.SdSatObs[k].System == GPS ? 0 : 1;
		wl1 = j == 0 ? WL1_GPS : WL1_BDS;
		wl2 = j == 0 ? WL2_GPS : WL3_BDS;

		ekf->X0[ekf->Index[k]] = (raw->SdObs.SdSatObs[k].dL[0] - raw->SdObs.SdSatObs[ekf->CurDDObs.RefPos[j]].dL[0]) -
			(raw->SdObs.SdSatObs[k].dP[0] - raw->SdObs.SdSatObs[ekf->CurDDObs.RefPos[j]].dP[0]) / wl1;
		ekf->X0[ekf->Index[k] + 1] = (raw->SdObs.SdSatObs[k].dL[1] - raw->SdObs.SdSatObs[ekf->CurDDObs.RefPos[j]].dL[1]) -
			(raw->SdObs.SdSatObs[k].dP[1] - raw->SdObs.SdSatObs[ekf->CurDDObs.RefPos[j]].dP[1]) / wl2;
	}
	//更新Rover位置
	vector_copy(Rov->Pos, ekf->X0, 3);
	vector_copy(ekf->X0, ekf->X, 2 * MAXCHANNUM + 3);
	vector_copy(ekf->P0, ekf->P, (2 * MAXCHANNUM + 3)*(2 * MAXCHANNUM + 3));
	ekf->Time = raw->OBS_rover.time;
	
	//
	matrixSub(ekf->P, rtkPrecision.Dxyz, paramnum, paramnum, 3, 3, 0, 0);
	delete[]Psi;
	delete[]Q;
	return true;
}
/*
对RTK进行测量更新

*/
bool RTKMeasUpdate(RAWDAT* raw, POSRES* Bas, POSRES* Rov, RTKEKF* ekf, FILE* fmout)
{

	int paramnum = (2 * ekf->CurDDObs.Sats + 3);
	int measurenum = 4 * ekf->CurDDObs.Sats;
	int num = raw->SdObs.SatNum;
	double *H, *W, *R, *X;
	H = new double[measurenum*paramnum]{ 0.0 };
	W = new double[measurenum] {0.0};
	R = new double[measurenum*measurenum]{ 0.0 };
	X = new double[paramnum] {0.0};

	/*vector_copy(ekf->X, X, 3);
	for (int i = 0, n = 3; i < num; i++)
	{
		int j = raw->SdObs.SdSatObs[i].System == GPS ? 0 : 1;
		//无参考星，参考星本星
		if (ekf->CurDDObs.RefPos[j] == -1 || ekf->CurDDObs.RefPos[j] == i)continue;
		//周跳、粗差、半周标记
		if (!raw->SdObs.SdSatObs[i].Valid) continue;
		if (!raw->SdObs.SdSatObs[i].freValid[0] || !raw->SdObs.SdSatObs[i].freValid[1])continue;
		int nBas = raw->SdObs.SdSatObs[i].nBas;
		int nRov = raw->SdObs.SdSatObs[i].nRov;
		if (!raw->RovEpk.SatPVT[nRov].Valid || !raw->BasEpk.SatPVT[nBas].Valid) continue;

		X[n] = ekf->X[ekf->Index[i]];
		X[n + 1] = ekf->X[ekf->Index[i] + 1];
		n += 2;
	}*/
	vector_copy(ekf->X, X, paramnum);
	//计算卫地距
	double disBas[MAXCHANNUM]{ 0.0 }, disRov[MAXCHANNUM]{ 0.0 };
	int idx;
	for (int i = 0; i < num; i++)
	{
		idx = raw->SdObs.SdSatObs[i].nBas;
		disBas[i] = geo_dis(raw->OBS_base.SatPVT[idx].Pos, Bas->Pos);
		idx = raw->SdObs.SdSatObs[i].nRov;
		disRov[i] = geo_dis(raw->OBS_rover.SatPVT[idx].Pos, Rov->Pos);
	}


	//计算量测矩阵H
	//计算OMC
	//计算观测值的噪声R
	cmp_BWP(disBas, disRov, raw, X, ekf->CurDDObs.DDSatNum[0], ekf->CurDDObs.DDSatNum[1], Rov, H, W, R, false, true);
	if (ekf->CurDDObs.Sats < 4)return false;

	if (fmout != nullptr)
	{
		fprintf(fmout, "time:%d,%lf\n", raw->OBS_base.time.Week, raw->OBS_base.time.SecOfWeek);
		fprintf(fmout, "P after TimeUpdate\n");
		fprint_matrix(fmout, ekf->P, paramnum, paramnum);
	}
	//测量更新
	if (filter(X, ekf->P, H, W, R, paramnum, measurenum, fmout) < 0)
	{
		vector_copy(ekf->X0, ekf->X, paramnum);
		vector_copy(ekf->P0, ekf->P, paramnum*paramnum);
		delete[]H; delete[]W; delete[]R; delete[]X;
		return false;
	}
	//固定
	double *Qnn;
	Qnn = new double[(paramnum - 3)*(paramnum - 3)]{ 0.0 };
	matrixSub(ekf->P, Qnn, paramnum, paramnum, paramnum - 3, paramnum - 3, 3, 3);
	matrixSub(ekf->P, rtkPrecision.Dxyz, paramnum, paramnum, 3, 3, 0, 0);
	if (fmout != nullptr)
	{
		fprintf(fmout, "Qnn\n");
		fprint_matrix(fmout, Qnn, paramnum - 3, paramnum - 3);
	}
	if (lambda(paramnum - 3, 2, ekf->X + 3, Qnn, ekf->CurDDObs.FixedAmb, ekf->CurDDObs.ResAmb) != 0)
	{
		delete[]Qnn; delete[]H; delete[]W; delete[]R; delete[]X;
		return false;
	}
	//将结果赋值给ekf
	ekf->CurDDObs.Ratio = (ekf->CurDDObs.ResAmb[0] > 0.0) ? ekf->CurDDObs.ResAmb[1] / ekf->CurDDObs.ResAmb[0] : 999.0;
	ekf->CurDDObs.Ratio = (ekf->CurDDObs.Ratio > 999.0) ? 999.0 : ekf->CurDDObs.Ratio;
	if (ekf->CurDDObs.Ratio > config.RatioThres)ekf->CurDDObs.bFixed = true;
	else ekf->CurDDObs.bFixed = false;
	vector_copy(X, ekf->X, paramnum);
	vector_copy(X, Rov->Pos, 3);
	delete[]Qnn; delete[]H; delete[]W; delete[]R; delete[]X;
	return true;

}
/*RTK EKF主程序
返回值： -1：未成功进行RTK，结果为SPP；1：成功进行RTK。
*/
int RTKMainEKF(RAWDAT* raw, RTKEKF*ekf, POSRES* Bas, POSRES* Rov, FILE* fmout)
{
	ekf->CurDDObs.reset();
	memcpy(&ekf->CurDDObs, &raw->DDObs, sizeof(DDCOBS));
	int rtkstatus = -2;
	//滤波初始化
	if (!ekf->IsInit)
	{
		if(RTKMainLSQ(*raw, *Bas, *Rov)>-1)
		//if (RTKFloat(raw, Bas, Rov))
		{
			InitRtkEkf(raw, Rov, ekf);
			rtkstatus = 1;
		}
		else rtkstatus=-1;
		
		memcpy(&ekf->DDObs, &ekf->CurDDObs, sizeof(DDCOBS));
		memcpy(&ekf->SDObs, &raw->SdObs, sizeof(SDEPOCHOBS));
		return rtkstatus;
	}

	//滤波时间更新
	if (RTKTimeUpdate(raw, Bas, Rov, ekf))
		//滤波测量更新
	{
		if (RTKMeasUpdate(raw, Bas, Rov, ekf, fmout))
		{
			//固定
			RTKFixed(ekf->CurDDObs.Ambnum, ekf->X, ekf->P, ekf->CurDDObs, Bas, Rov);
		}
		rtkstatus = 1;
	}
	else
	{
		rtkstatus = -1;
	}
	//精度评定
	ErrorEstimate(*Rov);

	memcpy(&ekf->DDObs, &ekf->CurDDObs, sizeof(DDCOBS));
	memcpy(&ekf->SDObs, &raw->SdObs, sizeof(SDEPOCHOBS));
	return rtkstatus;
}


void RTKMAIN(RAWDAT &raw, POSRES &posBas, POSRES &posRov)
{
	FormSDEpochObs(&raw.OBS_rover, &raw.OBS_base, &raw.SdObs);
	DetectCycleSlip(&raw.SdObs, raw);
	DetRefSat(&raw.OBS_rover, &raw.OBS_base, &raw.SdObs, &raw.DDObs);
	if (config.RTKProcMode == 2)//LSQ
	{
		RTKMainLSQ(raw, posBas, posRov);
	}
	else
	{
		RTKMainEKF(&raw, &ekf, &posBas, &posRov);
	}
}
void Menu(const char *fileRov, const char *fileBas, SOCKET socketBas, SOCKET socketRov, const char *filePos)
{
	FILE *fpRov, *fpBas, *fpPos, *fpLog;
	RAWDAT raw;
	POSRES posBas, posRov;
	int sync = 0;
	int quality = 0;

	fopen_s(&fpBas, fileBas, "rb");	//base
	fopen_s(&fpRov, fileRov, "rb");	//rover
	//fopen_s(&fpLog, "./matrixdata.txt", "w");
	fopen_s(&fpPos, filePos, "w");
	if (!fpPos)
	{
		cerr << "[WARNINGS]: CANNOT OPEN result FILE!\n";
	}
	if (fpPos)
	{
		/*fprintf(fpPos, "%4s %14s %19s %19s %19s %19s %19s %19s %13s %13s %13s %11s %11s %11s %11s %11s %11s %6s %6s",
			"Wk", "SOW", "ECEF-X/m", "ECEF-Y/m", "ECEF-Z/m", "REF-ECEF-X/m", "REF-ECEF-Y/m", "REF-ECEF-Z/m",
			"E/m","N/m","U/m","VX","VY","VZ","PDOP","SigmaP","SigmaV","nG" "nB");*/
		fprintf(fpPos, "%17s %3.1f deg\n", "% ElevThreshold :",config.ElevThreshold);
		fprintf(fpPos, "%29s\n","% Q=1:fixed,Q=2:float,Q=5:SPP");

		fprintf(fpPos, "%4s %11s ", "% Wk", "SOW");
		fprintf(fpPos, "%15s %15s %15s ", "Latitude(deg)", "Longitude(deg)", "Height/m");
		//fprintf(fpPos, "%15s %15s %15s ", "ECEF-X/m", "ECEF-Y/m", "ECEF-Z/m");
		fprintf(fpPos, "%8s %4s ", "Quality","n");
		fprintf(fpPos, "%12s %12s %12s ", "BaseLine_dX", "BaseLine_dY", "BaseLine_dZ");
		fprintf(fpPos, "%8s %8s %8s ", "mxx", "myy", "mzz");
		fprintf(fpPos, "%8s %8s %8s ", "mee", "mnn", "muu");
		//fprintf(fpPos, "%4s %4s %8s %8s\n", "nG", "nB", "Quality", "ratio");
		fprintf(fpPos, "%4s %4s %8s\n", "nG", "nB", "ratio");

	}
	//if (!fpLog)
	//{
	//	cerr << "[WARNINGS]: CANNOT OPEN LOG FILE!\n";
	//}
	if (config.IsFileData == 1)
	{
		if (!fpBas || !fpRov) {
			cerr << "[ERROR]: CANNOT OPEN OEM7 FILE!\n";
			return;
		}
	}


	//**********开始解算**********
	do
	{
		if (config.IsFileData == 1)
			sync = GetSynObs(fpBas, fpRov, raw);
		else if (config.IsFileData == 2)
			sync = GetSynObs(socketBas, socketRov, raw);

		double initpos[3] = { 0,0,6400000 };
		if (sync > 0)
		{
			int sucBas = sppMain(raw.OBS_base, raw.GpsEph, raw.BdsEph, posBas);
			int sucRov = sppMain(raw.OBS_rover, raw.GpsEph, raw.BdsEph, posRov);
			if (!sucRov|| !sucBas)
			{
				printf("[ERROR]:SPP failed! Bas:%d Rov:%d\n", sucBas, sucRov);
				continue;
			}
			////确保数据含有BestPos
			//if (geo_dis(raw.BasEpk.pos[0].Pos, initpos) > 100)
			//	vector_copy(raw.BasEpk.pos[0].Pos, posBas.Pos, 3);
			//设置基站坐标
			posBas.Pos[0] = -2267808.4305;
			posBas.Pos[1] = 5009321.5343;
			posBas.Pos[2] = 3221021.5829;
			FormSDEpochObs(&raw.OBS_rover, &raw.OBS_base, &raw.SdObs);
			DetectCycleSlip(&raw.SdObs, raw);
			DetRefSat(&raw.OBS_rover, &raw.OBS_base, &raw.SdObs, &raw.DDObs);
			if (config.RTKProcMode == 2) {
				int rtkstatus=RTKMainLSQ(raw, posBas, posRov);
				if (rtkstatus > -1)
				{
					quality = (rtkstatus == 0) ? 2 : 1;// float/fixed
				}
				else quality = 5;//SPP
				printf("%12.5f,%12.5f,%12.5f,%3d\n", raw.DDObs.dPos[0], raw.DDObs.dPos[1], raw.DDObs.dPos[2], quality);
				
				OutputPos(raw.OBS_base.time, posBas, posRov, raw.DDObs, quality, fpPos);
				//PrintNEMA(raw.BasEpk.time, raw.DDObs.bFixed, posRov.Pos, raw.DDObs.DDSatNum[0] + raw.DDObs.DDSatNum[1], "./result.txt");
			}
			else if (config.RTKProcMode == 1) {
				int rtkstatus = RTKMainEKF(&raw, &ekf, &posBas, &posRov);
				quality = (rtkstatus > -1) ? ((ekf.CurDDObs.bFixed) ? 1 : 2) : 5;

				printf("%12.5f,%12.5f,%12.5f,%3d\n", ekf.CurDDObs.dPos[0], ekf.CurDDObs.dPos[1], ekf.CurDDObs.dPos[2], quality);
				OutputPos(raw.OBS_base.time, posBas, posRov, ekf.CurDDObs, quality, fpPos);
			}
			/*if (quality!=1)
				printf("%4d %11.3lf %4d\n", raw.BasEpk.time.Week, raw.BasEpk.time.SecOfWeek,quality);*/
		}
		memcpy(&raw.BasLast, &raw.OBS_base, sizeof(EPOCHOBS));
		memcpy(&raw.RovLast, &raw.OBS_rover, sizeof(EPOCHOBS));
		//memset(raw.BasEpk.Satobs, 0, sizeof(SATOBS)*MAXCHANNUM);
		//memset(raw.BasEpk.SatPVT, 0, sizeof(SATMIDRES)*MAXCHANNUM);
		//memset(raw.RovEpk.Satobs, 0, sizeof(SATOBS)*MAXCHANNUM);
		//memset(raw.RovEpk.SatPVT, 0, sizeof(SATMIDRES)*MAXCHANNUM);

	} while (sync != -1);
	

	fclose(fpBas);
	fclose(fpRov);
	//fclose(fpLog);
	fclose(fpPos);
}

void OutputPos(GPSTIME Time, POSRES Bas, POSRES Rov, DDCOBS DDObs, int Quality,FILE* fpPos)
{
	//fprintf(fpPos, "%4s %11s ", "% Wk", "SOW");
	//fprintf(fpPos, "%15s %15s %15s ", "Latitude(deg)", "Longitude(deg)", "Height/m");
	//fprintf(fpPos, "%8s %4s ", "Quality", "n");
	//fprintf(fpPos, "%12s %12s %12s ", "BaseLine_dX", "BaseLine_dY", "BaseLine_dZ");
	//fprintf(fpPos, "%8s %8s %8s ", "mxx", "myy", "mzz");
	//fprintf(fpPos, "%8s %8s %8s ", "mee", "mnn", "muu");
	//fprintf(fpPos, "%4s %4s %8s\n", "nG", "nB", "ratio");

	if (Quality !=1)
	{
		vector_subtract(Rov.Pos, Bas.Pos, DDObs.dPos, 3);
	}
	//转化为BLH
	double blh[3]{ 0.0 };
	xyz_to_blh(Rov.Pos, blh);

	fprintf(fpPos, "%4d %11.3lf ",Time.Week, Time.SecOfWeek);
	fprintf(fpPos, "%15.8lf %15.8lf %15.8lf ",blh[0]*Deg, blh[1]*Deg, blh[2]);
	//fprintf(fpPos, "%15.4lf %15.4lf %15.4lf ",Rov.Pos[0], Rov.Pos[1], Rov.Pos[2]);
	fprintf(fpPos, "%8d %4d ", Quality, DDObs.DDSatNum[0] + DDObs.DDSatNum[1]);
	fprintf(fpPos, "%12.5lf %12.5lf %12.5lf ", DDObs.dPos[0], DDObs.dPos[1], DDObs.dPos[2]);
	//fprintf(fpPos, "%4d %4d %8d %8.1lf\n",DDObs.DDSatNum[0], DDObs.DDSatNum[1], Quality, DDObs.Ratio);
	fprintf(fpPos, "%8.5lf %8.5lf %8.5lf ",rtkPrecision.mxyz[0], rtkPrecision.mxyz[1], rtkPrecision.mxyz[2]);
	fprintf(fpPos, "%8.5lf %8.5lf %8.5lf ",rtkPrecision.menu[0], rtkPrecision.menu[1], rtkPrecision.menu[2]);
	fprintf(fpPos, "%4d %4d %8.1lf\n",DDObs.DDSatNum[0], DDObs.DDSatNum[1], DDObs.Ratio);


}
// 仅用于校验码，作为静态函数服务于 void PrintNEMA()
static int NMEACRC(char* buff, int len)
{
	int checknum = 0;
	char* pdata = NULL;
	short i;

	if (!buff)return 0;

	pdata = buff;
	for (i = 0; i < len; i++)
	{
		checknum ^= *(pdata + i);
	}
	return checknum;
}

static const double R2D = Rad;
/**
 * @brief        将结果用NEMA格式进行输出，一次输出一个历元的结果
 * @param[in]    gt			当前历元GPS时间结构体
 * @param[in]    isFixed	传入的坐标是否为固定解坐标
 * @param[in]    XYZ		当前解算的位置坐标
 * @param[in]    nsat		共视卫星数  BDS+GPS
 * @param[in]    OutputFilePath	  输出的文件路径
 * @return       void
 * @note         注意这里GPS周秒转成通用时，ECEF坐标转LLH使用自己的所写的代码实现
 * @par History
 *               2025/03/12, 卫星导航算法课程  new \n
 */
void PrintNEMA(GPSTIME gt, bool isFixed, const double XYZ[], const int nsat, const char OutputFilePath[]) {
	if (!OutputFilePath)return;
	char   sol[1024] = "";
	memset(sol, 0, 1024);
	double BLH[3]{}, min;
	int    checkNum = 0, checkStart = 0, checkEnd = 0, deg = 0, solLen = 0, QNMEA = 5;
	static bool bHead1 = true;
	COMMONTIME ymd;			// 年月日时分秒的转换
	//GT_CT(gt, ymd, 0);      //注意跳秒
	xyz_to_blh(XYZ, BLH);      //XYZ是解算出流动站ecef坐标,默认输出的BLH是弧度制
	isFixed ? QNMEA = 4 : QNMEA = 5;   //这里是针对RTK，如果是SPP结果，设QNMEA为1即可

	if (bHead1)
	{
		solLen += sprintf(sol + solLen, "$ACCUR,1,12,0,%04d%02d%02d,,123206.00,,,,,,,,,,*", ymd.Year, ymd.Month, ymd.Day);
		checkNum = NMEACRC(sol + 1, solLen - 2); // 校验和
		solLen += sprintf(sol + solLen, "%X\n", checkNum);
		bHead1 = false;
	}

	checkStart = solLen + 1; //起始下标
	checkEnd = solLen + 1; //结束下标

	solLen += sprintf(sol + solLen, "$GNGGA,");

	// 时间: 时分秒
	solLen += sprintf(sol + solLen, "%02d%02d%05.2f,", ymd.Hour, ymd.Minute, ymd.Second);

	// 纬度 ddmm.mmmmmm 度分 
	deg = (int)(fabs(BLH[0]) * R2D);
	min = (fabs(BLH[0] * R2D) - deg) * 60;
	if (BLH[0] > 0) solLen += sprintf(sol + solLen, "%d%09.6f,N,", deg, min);
	else solLen += sprintf(sol + solLen, "%d%09.6f,S,", deg, min);

	// 经度 dddmm.mmmmmm 度分
	deg = (int)(fabs(BLH[1]) * R2D);
	min = (fabs(BLH[1] * R2D) - deg) * 60;
	if (BLH[1] > 0) solLen += sprintf(sol + solLen, "%d%09.6f,E,", deg, min);
	else solLen += sprintf(sol + solLen, "%d%09.6f,W,", deg, min);

	// 解状态
	solLen += sprintf(sol + solLen, "%d,", QNMEA);

	// 卫星数
	solLen += sprintf(sol + solLen, "%2d,", nsat);

	// HDOP
	solLen += sprintf(sol + solLen, "%3.1f,", 0.0);

	// 高程
	solLen += sprintf(sol + solLen, "%5.3f,M,", BLH[2]);

	// 高程异常
	solLen += sprintf(sol + solLen, "%5.3f,M,", 0.0);

	solLen += sprintf(sol + solLen, ",*");

	// 校验和：$与*之间所有字符ASCII码的校验和(各字节做异或运算，得到校验和后，再转换16进制格式的ASCII字符。)
	checkEnd = solLen - 2; //-2:*与下标
	checkNum = NMEACRC(sol + checkStart, checkEnd - checkStart + 1);
	solLen += sprintf(sol + solLen, "%X\n", checkNum);


	static FILE* fpNEMA = NULL;
	if (fpNEMA == NULL)
	{
		fopen_s(&fpNEMA, OutputFilePath, "w");
	}
	if (fpNEMA != NULL) {
		fprintf(fpNEMA, sol);
		fflush(fpNEMA);
	}
}

void ErrorEstimate(POSRES Rov,bool isLSQ)
{
	double Dxyz[9]{ 0.0 };
	double Denu[9]{ 0.0 };
	double blh[3]{ 0.0 };
	double enuMAT[9]{ 0.0 };
	if (isLSQ)
	{//LSQ时，传入rtkPrecision的是Qxyz，需乘以sigma转化为Dxyz
		vector_num_product(rtkPrecision.Dxyz, Dxyz, rtkPrecision.sigma, 9);
	}
	else
	{
		vector_copy(rtkPrecision.Dxyz, Dxyz, 9);
	}
	//xyz-->blh-->enuMAT
	xyz_to_blh(Rov.Pos, blh);
	blh_to_enuMAT(blh, enuMAT);
	//Dxyz-->Denu
	BPBt(enuMAT, Dxyz, Denu, 3, 3);

	for (int i = 0; i < 3; i++)
	{
		rtkPrecision.mxyz[i] = sqrt(Dxyz[i * 3 + i]);
		rtkPrecision.menu[i] = sqrt(Denu[i * 3 + i]);
	}
}