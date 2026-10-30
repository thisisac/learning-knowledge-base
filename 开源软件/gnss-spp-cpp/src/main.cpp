#include <iostream>
#include "rtklib.h"
#include"sockets.h"

using namespace std;

int main()
{

	SATEPHREC gpseph[MAXGPSNUM];
	SATEPHREC bdseph[MAXBDSNUM];
	EPOCHOBSDATA obs;
	POSRES Res;
	XYZCOOR P_ref, P_cal;
	NEUCOOR denu;

	int i;
	int LenRead, LenDecode = 0;
	unsigned char buff[MAXRAWLEN];
	unsigned char BUFF[MAXRAWLEN];

	//-------------------------------读取文件解码--------------------

	FILE* file;
	FILE* fout;
	fopen_s(&file, "D:\\viual stidio\\code\\RTKNav\\data\\oem719-202504011900-1.bin", "rb");
	fopen_s(&fout, "RESfile.txt", "w");
	fprintf(fout, "Week      SOW		  X			  Y			 Z			 refX		  refY		  refZ		");
	fprintf(fout, "   Vx	  Vy      Vz	  PDOP	  sigmaP	sigmaV	  GPSnum BDSNum    Gclk    Bclk		");
	fprintf(fout, "sigmax	sigmay	sigmaz	sigmaE	sigmaN	sigmaU: \n");
	if (!file)
	{
		return -1;
	}
	while (!feof(file))
	{
		LenRead = fread(buff + LenDecode, sizeof(unsigned char), MAXRAWLEN - LenDecode, file);
		LenDecode += LenRead;
		i = DecodeNovOem7Dat(buff, LenDecode, &obs, gpseph, bdseph, &obs.decode_Pos);
		if (i == -1)
		{
			break;
		}
		DetectOutlier(obs);
		printf("---------------------------------------------------------------------------------------\n");

		obs.time.print_GPSTIME();
		//ComputeGPSSatOrbitAtSignalTrans(obs, gpseph, bdseph,Pos, obs.SatPVT);
		if (SPP(obs, gpseph, bdseph, Res))
		{
			SPV(obs, Res);
			printf("POS:");
			vec_print(Res.Pos, 3);
			printf("VEL:");
			vec_print(Res.Vel, 3);
			printf("Vel_Sigma:%f\n", Res.sigma_vel);
			printf("Sigma:%f  pdop:%f  gpsclock:%f  bdsclock:%f\n\n", Res.sigma_pos, Res.PDOP, Res.clk[0], Res.clk[1]);

			//计算每个历元的定位误差值，并转换到ENU坐标系

			fprintf(fout, "%4d %11.3lf %15.4lf %15.4lf %15.4lf %15.4lf %15.4lf %15.4lf %8.3lf %8.3lf %8.3lf %8.3lf %8.3lf %8.3lf  %6d  %6d %8.3lf %8.3lf",
				obs.time.Week, obs.time.SecOfWeek, Res.Pos[0], Res.Pos[1], Res.Pos[2], obs.decode_Pos.Pos[0], obs.decode_Pos.Pos[1], obs.decode_Pos.Pos[2],
				Res.Vel[0], Res.Vel[1], Res.Vel[2], Res.PDOP, Res.sigma_pos, Res.sigma_vel, Res.GPSNum, Res.BDSNum,Res.clk[0],Res.clk[1]);
			fprintf(fout, "%8.3lf %8.3lf %8.3lf %8.3lf %8.3lf %8.3lf\n",
				Res.std_xyz[0], Res.std_xyz[1], Res.std_xyz[2], Res.std_enu[0], Res.std_enu[1], Res.std_enu[2]);
		}
		memset(obs.SatObs, 0, sizeof(SATOBSDATA) * MAXCHANNUM);
		memset(obs.SatPVT, 0, sizeof(SATMIDRES) * MAXCHANNUM);
	}
	fclose(file);
	fclose(fout);
}


//-------------------------------读取文件解码--------------------


//
//	SOCKET NetGps;
//	FILE* fout;
//	const char* file_out = "RESsocket.txt";
//	fopen_s(&fout, file_out, "w");
//	if (fout == nullptr) { cerr << "Failed to open the file" << endl; }
//	fprintf(fout, "Week      SOW		  X			  Y			 Z			 refX		  refY		  refZ		");
//	fprintf(fout, "   Vx	  Vy      Vz	  PDOP	  sigmaP	sigmaV	  GPSnum BDSNum    Gclk    Bclk		");
//	fprintf(fout, "sigmax	sigmay	sigmaz	sigmaE	sigmaN	sigmaU: \n");
//	if (OpenSocket(NetGps, "47.114.134.129", 7190) == false)
//		//if (OpenSocket(NetGps, "8.148.22.229", 4002) == false)
//	{
//		printf("This ip & port was not opened.\n");
//		return 0;
//	}
//	while (1)
//	{
//		Sleep(999);
//		if ((LenRead = recv(NetGps, (char*)(buff), MAXRAWLEN, 0)) > 0)
//		{
//			memcpy(BUFF + LenDecode, buff, LenRead);
//			LenDecode += LenRead;
//			i = DecodeNovOem7Dat(BUFF, LenDecode, &obs, gpseph, bdseph, &obs.decode_Pos);
//			DetectOutlier(obs);
//			printf("---------------------------------------------------------------------------------------\n");
//
//			obs.time.print_GPSTIME();
//			if (SPP(obs, gpseph, bdseph, Res))
//			{
//				SPV(obs, Res);
//
//				//输出数据
//				printf("POS:");
//				vec_print(Res.Pos, 3);
//				printf("VEL:");
//				vec_print(Res.Vel, 3);
//				printf("Vel_sigma:%f\n", Res.sigma_vel);
//				printf("SIGMA:%f  PDOP:%f  GPSclock:%f  BDSclock:%f\n\n", Res.sigma_pos, Res.PDOP, Res.clk[0], Res.clk[1]);
//
//				fprintf(fout, "%4d %11.3lf %15.4lf %15.4lf %15.4lf %15.4lf %15.4lf %15.4lf %8.3lf %8.3lf %8.3lf %8.3lf %8.3lf %8.3lf  %6d  %6d %8.3lf %8.3lf",
//					obs.time.Week, obs.time.SecOfWeek, Res.Pos[0], Res.Pos[1], Res.Pos[2], obs.decode_Pos.Pos[0], obs.decode_Pos.Pos[1], obs.decode_Pos.Pos[2],
//					Res.Vel[0], Res.Vel[1], Res.Vel[2], Res.PDOP, Res.sigma_pos, Res.sigma_vel, Res.GPSNum, Res.BDSNum, Res.clk[0], Res.clk[1]);
//				fprintf(fout, "%8.3lf %8.3lf %8.3lf %8.3lf %8.3lf %8.3lf\n",
//					Res.std_xyz[0], Res.std_xyz[1], Res.std_xyz[2], Res.std_enu[0], Res.std_enu[1], Res.std_enu[2]);
//			}
//			memset(obs.SatObs, 0, sizeof(SATOBSDATA) * MAXCHANNUM);
//			memset(obs.SatPVT, 0, sizeof(SATMIDRES) * MAXCHANNUM);
//		}
//	}
//	CloseSocket(NetGps);
//	fclose(fout);
//	return 0;
//}
//
