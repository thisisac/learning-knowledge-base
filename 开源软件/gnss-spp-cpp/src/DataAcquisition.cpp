#include "rtklib.h"

using namespace std;
//I4
int I4(unsigned char* p)
{
	int i;
	memcpy(&i, p, 4);
	return i;
}
//I2
short I2(unsigned char* p)
{
	short result;
	result = static_cast<short>((p[0] & 0xFF) |
		(p[1] << 8 & 0xFF00));
	return result;
}
//UI4
unsigned int UI4(unsigned char* p)
{
	unsigned int u;
	memcpy(&u, p, 4);
	return u;
}
//UI2
unsigned short UI2(unsigned char* p)
{
	unsigned short u;
	memcpy(&u, p, 2);
	return u;
}
//R8
double R8(unsigned char* p)
{
	double r;
	memcpy(&r, p, 8);
	return r;
}
//R4
float R4(unsigned char* p)
{
	float r;
	memcpy(&r, p, 4);
	return r;
}
/* DecodeNovOem7Dat ----------------------------------
* Decode OEM7 binary message into observation data, ephemeris data, or position data.
*
* args   : buff    I/O   Buffer containing the data to be decoded
*          len     I/O   Length of the buffer (updated after decoding)
*          obs     O    Decoded observation data
*          geph    O    GPS ephemeris data
*          beph    O    BeiDou ephemeris data
*          pos     O    Position data (if available)
*
* return : int     0 if successful, non-zero for errors
*----------------------------------------------------*/
int DecodeNovOem7Dat(unsigned char buff[], int& len, EPOCHOBSDATA* obs, SATEPHREC geph[], SATEPHREC beph[], POSRES* pos)
{
	//设计要求：
		//1. 输入buff缓冲区，总字节数len，buff中可能有多组消息，循环解码每
		//一条消息，在文件模式下，解码得到观测值后，函数返回；在实时模
		//式下，解码所有消息，直到剩余的字节不能组成一条消息。
		//2. 剩余的字节要和下一组要读取的字节组成一个buff，避免数据丢失

	int i, MsgLen, MsgId, Status;

	i = 0;
	while (true)
	{
		//设计思路：
		//1. 设置循环变量i = 0，开始查找AA 44 12同步字符
		for (; i < len - 2; i++) {
			if (buff[i] == 0xAA && buff[i + 1] == 0x44 && buff[i + 2] == 0x12) break;
		}
		//2. 找到同步字符后，获取消息头长度的字符28字节，若字节数量不足即
		// i + 28 > len，跳出循环（break）6步
		if (i + 28 > len)break;

		//3. 从消息头中解码消息长度MsgLen和消息类型MsgID，获得整条消息
		//buff[i, i + 28 + MsgLen + 4]，若字节数量不足即i + 28 + MsgLen + 4 > len，
		//则跳出循环（break）6步
		MsgId = UI2(buff + i + 4);
		MsgLen = UI2(buff + i + 8);//对maglen进行校验

		if (i + 28 + MsgLen + 4 > len) break;
		//4. CRC检验，若不通过，跳过同步字符3个字节，即i = i + 3，返回到第1步continue;
		if (crc32(buff + i, 28 + MsgLen) != UI4(buff + i + 28 + MsgLen)) {
			i += 3;
			continue;
		}
		//5. 根据消息ID，调用对应的解码函数，若解码得到观测值，跳出循环至
		//第6步；否则，跳过整条消息，即i = i + 28 + MsgLen + 4，返回第1步
		Status = 0;
		switch (MsgId)
		{
		case 43:
			decode_rangeb_oem7(buff + i, MsgLen, obs);
			Status = 1;//读取状态历元变化，循环可跳出
			break;
		case 7:
			decode_gpsephem(buff + i, MsgLen, geph);
			break;
		case 1696:
			decode_bdsephem(buff + i, MsgLen, beph);
			break;
		case 42:
			decode_psrpos(buff + i, MsgLen, pos);
			break;
		default:
			printf("Unkown message ID: 0x%04X\n", MsgId);
			break;
		}
		i = i + 28 + MsgLen + 4;
		if (Status == 1 && FILEMODE == 1) break;
		//设置文件读取状态Status，为1时候则退出
	}
	//6. 循环结束后，将不足一条消息的剩余字节，拷贝至buff缓冲区的开始
	//处，将len设置为剩余字节数量，并返回给主函数。
	memcpy(buff, buff + i, len - i);
	len = len - i;
	return 0;
}
/* crc32 ----------------------------------
* Calculate for a buffer.
* args   : buff    I   buffer
*          len     I   Length
* return : unsigned int  CRC32 checksum value
*----------------------------------------------------*/
unsigned int crc32(unsigned char* buff, int len)
{
	int i, j;
	unsigned int crc = 0;
	for (i = 0; i < len; i++)
	{
		crc ^= buff[i];
		for (j = 0; j < 8; j++)
		{
			if (crc & 1) crc = (crc >> 1) ^ POLYCRC32;
			else crc >>= 1;
		}
	}
	return crc;
}
/* get_type ----------------------------------
* Return the signal type
* args   : sys     I   GNSS system (GPS or BDS)
*          code    I   Signal code (e.g., 0, 5, 9)
* return : SignalType   Corresponding signal type
*----------------------------------------------------*/
SignalType get_type(GNSSsys sys, int code)
{
	if (sys == GPS)
	{
		switch (code)
		{
		case 0:return GPS_L1C; break;
		case 5:return GPS_L2P; break;
		case 9:return GPS_L2W; break;
		case 14:return GPS_L5Q; break;
		case 16:return GPS_L1L; break;
		case 17:return GPS_L2C; break;
		default:break;
		}
	}
	else if (sys == BDS)
	{
		switch (code)
		{
		case 0:return BDS_B1I_D1; break;
		case 4:return BDS_B1I_D2; break;
		case 1:return BDS_B2I_D1; break;
		case 5:return BDS_B2I_D1; break;
		case 2:return BDS_B3I_D1; break;
		case 6:return BDS_B3I_D2; break;
		case 7:return BDS_B1C; break;
		case 9:return BDS_B2a; break;
		case 11:return BDS_B2b; break;
		default:break;
		}
	}
	return CODE_NONE;
}
/* decode_rangeb_oem7 ---------------------------------------------------------------
* Decode range data from OEM7 and store
* args   : buff    I   buffer containing OEM7 message
*          len     I   Length
*          obs     O   Observation data
* return : int     1 if decoding was successful, 0 if no valid data found
*---------------------------------------------------------------------------------*/
int decode_rangeb_oem7(unsigned char* buff, int len, EPOCHOBSDATA* obs)
{

	int i, j, n = 0, k = -1, ObsNum, Freq, Prn;
	GNSSsys sys;
	double wl;
	int PhaseLockFlag, CodeLockedFlag, ParityFlag, SatSystem, SigType;
	unsigned int ChanStatus;

	unsigned char* p = buff + 28;

	//	1. 从消息头中解码得到观测时刻，该时刻为接收机钟表面时，用GPSTIME结
	//	构体表示。(文件头部分)
	obs->time.Week = UI2(buff + 14);
	obs->time.SecOfWeek = UI4(buff + 16) * 1E-3;
	//	2. 解码得到观测值数量，为所有卫星所有信号观测值的总数。（数据块部分）
	ObsNum = UI4(p);  // 观测卫星数量
	memset(obs->SatObs, 0, MAXCHANNUM * sizeof(SATOBSDATA));

	//	3. 对所有信号观测值进行循环解码(观测值obs数据)
	for (i = 0, p += 4; i < ObsNum; i++, p += 44)
	{
		//	① 解码得到跟踪状态标记，从中取出Phase lock flag/Code locked 
		//	flag/Parity known flag/Satellite system/signal type等数据
		ChanStatus = UI4(p + 40);//此处p+4的原因
		ParityFlag = (ChanStatus >> 11) & 0x01;
		PhaseLockFlag = (ChanStatus >> 10) & 0x01;
		CodeLockedFlag = (ChanStatus >> 12) & 0x01;
		SatSystem = (ChanStatus >> 16) & 0x07;
		SigType = (ChanStatus >> 21) & 0x1F;
		//	② 如果卫星系统不是GPS或BDS，continue至①

		//	③ 如果GPS卫星的信号类型不是L1 C/A或者L2P（Y），BDS卫星不是B1I
		//	或B3I， continue至①，并记录信号频率类型，第一频率s=0，第二频
		//	率s=1；
		if (SatSystem == 0) {
			sys = GPS;
			if (SigType == 0) {
				Freq = 0; wl = WL1_GPS;
			}
			else if (SigType == 9) { Freq = 1; wl = WL2_GPS; }
			else continue;
		}
		else if (SatSystem == 4) {
			sys = BDS;
			if (SigType == 0 || SigType == 4) {
				Freq = 0; wl = WL1_BDS;
			}
			else if (SigType == 2 || SigType == 6) { Freq = 1; wl = WL3_BDS; }
			else continue;
		}
		else continue;

		//	④ 解码得到卫星号Prn以及卫星系统号，在当前观测值结构体中进行搜索，
		//	如果找到相同的卫星，将解码的观测值填充到该卫星对应的数组中；如
		//	果在当前已解码的卫星数据中没有发现，则填充到现有数据的末尾。
		Prn = UI2(p);
		for (j = 0; j < MAXCHANNUM; j++) {
			if (obs->SatObs[j].Satsys == sys && obs->SatObs[j].Prn == Prn) {
				n = j; break;
			}
			if (obs->SatObs[j].Prn == 0) {
				k = n = j; break;
			}
		}
		obs->SatObs[n].Prn = Prn;
		obs->SatObs[n].Satsys = sys;
		obs->SatObs[n].P[Freq] = CodeLockedFlag == 1 ? R8(p + 4) : 0.0;
		obs->SatObs[n].L[Freq] = -wl * (PhaseLockFlag == 1 ? R8(p + 16) : 0.0);
		obs->SatObs[n].D[Freq] = -wl * R4(p + 28);
		obs->SatObs[n].cn0[Freq] = R4(p + 32);
		obs->SatObs[n].LockTime[Freq] = R4(p + 36);
		obs->SatObs[n].half[Freq] = ParityFlag;
	}
	//	4. 统计有效的卫星数量
	obs->SatNum = k + 1;
	return 1;
}
/* decode_gpsephem ----------------------------------------------------------------
* Decode GPS ephemeris data from OEM7 and store
* args   : buff    I   buffer containing OEM7 message
*          len     I   Length
*          geph    O   GPS ephemeris data
* return : int     1 if decoding was successful, 0 if no valid data found
*---------------------------------------------------------------------------------*/
int decode_gpsephem(unsigned char* buff, int len, SATEPHREC* geph)
{
	if (buff == NULL)	return 0;
	unsigned char* p = buff + 28;
	SATEPHREC* eph;

	int prn = UI4(p);
	if (prn < 1 || prn >= MAXBDSNUM) { return -1; }

	eph = geph + prn - 1;//更加易读,指针操作

	eph->Prn = prn;
	eph->Satsys = GPS;
	eph->SVHealth = UI4(p + 12);
	eph->IODE = UI4(p + 16);
	eph->IODE2 = UI4(p + 20);
	eph->TOC.Week = eph->TOE.Week = UI4(p + 24);
	eph->TOE.SecOfWeek = R8(p + 32);
	eph->SqrtA = sqrt(R8(p + 40));
	eph->DeltaN = R8(p + 48);
	eph->M0 = R8(p + 56);
	eph->e = R8(p + 64);
	eph->omega = R8(p + 72);
	eph->Cuc = R8(p + 80);
	eph->Cus = R8(p + 88);
	eph->Crc = R8(p + 96);
	eph->Crs = R8(p + 104);
	eph->Cic = R8(p + 112);
	eph->Cis = R8(p + 120);
	eph->i0 = R8(p + 128);
	eph->iDot = R8(p + 136);
	eph->OMEGA = R8(p + 144);
	eph->OMEGADot = R8(p + 152);
	eph->IODC = UI4(p + 160);
	eph->TOC.SecOfWeek = R8(p + 164);
	eph->TGD1 = R8(p + 172);
	eph->ClkBias = R8(p + 180);
	eph->ClkDrift = R8(p + 188);
	eph->ClkDriftRate = R8(p + 196);
	return 1;
}
/* decode_bdsephem -----------------------------------------------------------------
* Decode BeiDou ephemeris data from OEM7 and store
* args   : buff    I   buffer containing OEM7 message
*          len     I   Length
*          beph    O   BDS ephemeris data
* return : int     1 if decoding was successful, 0 if no valid data found
*---------------------------------------------------------------------------------*/
int decode_bdsephem(unsigned char* buff, int len, SATEPHREC* beph)
{
	if (buff == NULL)	return 0;
	unsigned char* p = buff + 28;
	SATEPHREC* eph;

	int prn = UI4(p);
	if (prn < 1 || prn >= MAXBDSNUM) { return -1; }

	eph = beph + prn - 1;//更加易读,指针操作

	eph->Prn = prn;
	eph->Satsys = BDS;
	eph->TOC.Week = eph->TOE.Week = UI4(p + 4);
	eph->SVHealth = UI4(p + 16);
	eph->TGD1 = R8(p + 20);
	eph->TGD2 = R8(p + 28);
	eph->IODC = UI4(p + 36);
	eph->TOC.SecOfWeek = UI4(p + 40);
	eph->ClkBias = R8(p + 44);
	eph->ClkDrift = R8(p + 52);
	eph->ClkDriftRate = R8(p + 60);
	eph->IODE = UI4(p + 68);
	eph->TOE.SecOfWeek = UI4(p + 72);
	eph->SqrtA = R8(p + 76);
	eph->e = R8(p + 84);
	eph->omega = R8(p + 92);
	eph->DeltaN = R8(p + 100);
	eph->M0 = R8(p + 108);
	eph->OMEGA = R8(p + 116);
	eph->OMEGADot = R8(p + 124);
	eph->i0 = R8(p + 132);
	eph->iDot = R8(p + 140);
	eph->Cuc = R8(p + 148);
	eph->Cus = R8(p + 156);
	eph->Crc = R8(p + 164);
	eph->Crs = R8(p + 172);
	eph->Cic = R8(p + 180);
	eph->Cis = R8(p + 188);
	return 1;
}
/* decode_psrpos ------------------------------------------------------------
* Decode position solution from OEM7 and store
* args   : buff    I   buffer OEM7 msg
*          len     I   Length
*          pos     O   Position data to store
* return : int     1 if decoding was successful, 0 if no valid data found
*----------------------------------------------------------------------------*/
int decode_psrpos(unsigned char* buff, int len, POSRES* pos)
{
	//时间
	unsigned char* p = buff + 28;

	pos->Time.Week = UI2(p - 28 + 14);
	pos->Time.SecOfWeek = R8(p - 28 + 16) * 0.001;
	//坐标xyz
	BLHCOOR blh((R8(p + 8)) / Deg, (R8(p + 16)) / Deg, (R8(p + 24) + R4(p + 32)));
	XYZCOOR xyz;
	BLH2XYZ(blh, xyz, WGS84);
	for (int i = 0; i < 3; i++)
	{
		pos->Pos[i] = xyz.xyz[i];
	}
	return 1;
}
