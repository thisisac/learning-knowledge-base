#include "rtklib.h"

//我所使用的同步函数
//short SynRTKObs(FILE* Bin, FILE* Rin, RAWDATA* rawdata)
//{
//    double dt;
//    int lenR_base = 0, lenR_rover = 0;                // 本次读入长度
//    static int lenD_base = 0, lenD_rover = 0;         // 缓冲区内剩余长度（由解码器回传维护）
//    static unsigned char buff_base[MAXRAWLEN], buff_rover[MAXRAWLEN];
//    static GPSTIME last_rover = { 0,0 }, last_base = { 0,0 }; // 上一次输出的历元时间，用于“新历元”判定
//
//    // 1) 读取流动站：直到解出一帧“新历元”观测
//    while (true)
//    {
//        lenR_rover = (int)fread(buff_rover + lenD_rover, 1, MAXRAWLEN - lenD_rover, Rin);
//        if (lenR_rover <= 0) return -1; // EOF/错误
//
//        lenD_rover += lenR_rover;
//
//        // 解码：lenD_rover 将被回传为“未解码剩余字节数”
//        DecodeNovOem7Dat(buff_rover, lenD_rover, &rawdata->OBS_rover,
//            rawdata->GpsEph, rawdata->BdsEph, &rawdata->OBS_rover.decode_Pos);
//
//        // 判定是否得到“新历元”
//        if (rawdata->OBS_rover.SatNum > 0 &&
//            (rawdata->OBS_rover.time.Week != last_rover.Week ||
//                fabs(rawdata->OBS_rover.time.SecOfWeek - last_rover.SecOfWeek) > 1e-9))
//        {
//            last_rover = rawdata->OBS_rover.time;
//            break; // 拿到一帧流动站观测
//        }
//
//        // 防止异常数据撑爆缓冲
//        if (lenD_rover >= MAXRAWLEN) lenD_rover = 0;
//    }
//
//    // 2) 若基站已有有效时间，则先比较时间差，满足限差直接返回同步
//    if (rawdata->OBS_base.time.Week != 0 || rawdata->OBS_base.time.SecOfWeek != 0) {
//        dt = GetDiffTime(&rawdata->OBS_rover.time, &rawdata->OBS_base.time); // dt = rover - base
//        if (fabs(dt) < 0.1) return 1;   // 已同步（限差内）
//        if (dt < -0.5)      return 0;   // 基站时间比流动站新（rover 落后），外层应先推进流动站
//        // dt >= +0.5：基站落后，需要继续读取基站以追赶
//    }
//
//    // 3) 推进基站：直到追上或与流动站在限差内
//    while (true)
//    {
//        lenR_base = (int)fread(buff_base + lenD_base, 1, MAXRAWLEN - lenD_base, Bin);
//        if (lenR_base <= 0) return -1; // EOF/错误
//
//        lenD_base += lenR_base;
//
//        // 解码：lenD_base 将被回传为“未解码剩余字节数”
//        DecodeNovOem7Dat(buff_base, lenD_base, &rawdata->OBS_base,
//            rawdata->GpsEph, rawdata->BdsEph, &rawdata->OBS_base.decode_Pos);
//
//        // 判定是否得到“新历元”
//        if (rawdata->OBS_base.SatNum > 0 &&
//            (rawdata->OBS_base.time.Week != last_base.Week ||
//                fabs(rawdata->OBS_base.time.SecOfWeek - last_base.SecOfWeek) > 1e-9))
//        {
//            last_base = rawdata->OBS_base.time;
//
//            dt = GetDiffTime(&rawdata->OBS_rover.time, &rawdata->OBS_base.time); // dt = rover - base
//            if (fabs(dt) < 0.1) return 1;  // 同步成功
//            if (dt < -0.5)      return 0;  // 基站时间比流动站新（rover 落后），交回外层，下次先推进流动站
//            // dt >= +0.5：基站仍落后，继续读基站
//        }
//
//        if (lenD_base >= MAXRAWLEN) lenD_base = 0;
//    }
//}

bool ReadNextEpoch(FILE* fin,
    unsigned char* buf, int& lenRemain,
    EPOCHOBSDATA* outObs,
    SATEPHREC* geph, SATEPHREC* beph,
    GPSTIME& lastTime)
{
    while (true) {
        // 读入追加到剩余区
        int n = (int)fread(buf + lenRemain, 1, MAXRAWLEN - lenRemain, fin);
        if (n <= 0) return false; // EOF/错误
        lenRemain += n;

        // 解码
        DecodeNovOem7Dat(buf, lenRemain, outObs, geph, beph, &outObs->best_pos);

        // 拿到新历元才返回
        if (outObs->SatNum > 0 &&
            (outObs->time.Week != lastTime.Week ||
                fabs(outObs->time.SecOfWeek - lastTime.SecOfWeek) > 1e-9))
        {
            lastTime = outObs->time;
            return true;
        }
    }
}

bool ReadNextEpochSock(SOCKET s,
    unsigned char* buf, int& lenRemain,
    EPOCHOBSDATA* outObs,
    SATEPHREC* geph, SATEPHREC* beph,
    GPSTIME& lastTime)
{
    while (true) {
        // 读入追加到剩余区
        int n = recv(s, (char*)(buf)+lenRemain, MAXRAWLEN - lenRemain, 0);
        if (n <= 0) return false; // EOF/错误
        lenRemain += n;

        // 解码
        DecodeNovOem7Dat(buf, lenRemain, outObs, geph, beph, &outObs->best_pos);

        // 拿到新历元才返回
        if (outObs->SatNum > 0 &&
            (outObs->time.Week != lastTime.Week ||
                fabs(outObs->time.SecOfWeek - lastTime.SecOfWeek) > 1e-9))
        {
            lastTime = outObs->time;
            return true;
        }
    }
}


short SynRTKObs(FILE* Bin, FILE* Rin, RAWDATA* rawdata)
{
    // 独立缓冲与剩余长度
    static unsigned char buff_base[MAXRAWLEN], buff_rover[MAXRAWLEN];
    static int lenD_base = 0, lenD_rover = 0;
    static GPSTIME last_rover{ 0,0 }, last_base{ 0,0 };

    const double SYNC_TOL = cfg.syn_tol;     // 同步容差（秒）
    // 1) 先推进流动站到一个新历元
    if (!ReadNextEpoch(Rin, buff_rover, lenD_rover,
        &rawdata->OBS_rover, rawdata->GpsEph, rawdata->BdsEph,
        last_rover)) {
        return -1; // EOF/错误
    }

    // 2) 若基站已有时间，先判差
    if (rawdata->OBS_base.SatNum > 0) {
        double dt = GetDiffTime(&rawdata->OBS_rover.time, &rawdata->OBS_base.time); // rover - base
        if (fabs(dt) < SYNC_TOL) return 1; // 已同步
        if (dt < -0.5) return 0;            // 基站领先，交回外层先推进 rover
        // 否则：基站落后，继续推进基站
    }

    // 3) 推进基站，直到追齐或超前
    while (true) {
        if (!ReadNextEpoch(Bin, buff_base, lenD_base,
            &rawdata->OBS_base, rawdata->GpsEph, rawdata->BdsEph,
            last_base)) {
            return -1; // EOF/错误
        }
        double dt = GetDiffTime(&rawdata->OBS_rover.time, &rawdata->OBS_base.time); // rover - base
        if (fabs(dt) < SYNC_TOL) return 1; // 同步成功
        if (dt < -0.5) return 0;           // 基站仍领先，交回外层推进 rover
        // dt >= +0.5：基站还落后，继续读基站
    }
}
// 端口文件时间同步（与 SynRTKObs 类似，但使用独立静态状态，便于并行/切换端口文件）
// 返回：1=已同步；0=应先推进流动站；-1=EOF/错误

short SynRTKObs_Sock(SOCKET BaseSock, SOCKET RoverSock, RAWDATA* rawdata)
{
    // 独立缓冲与剩余长度（静态跨调用，分别对应基站/流动站）
    static unsigned char buff_base[MAXRAWLEN], buff_rover[MAXRAWLEN];
    static int  len_base_rem = 0, len_rover_rem = 0;
    static GPSTIME last_rover{ 0,0 }, last_base{ 0,0 };

    const double SYNC_TOL = cfg.syn_tol; // 同步容差（秒）

    // 1) 先推进流动站到一个新历元
    if (!ReadNextEpochSock(RoverSock, buff_rover, len_rover_rem,
        &rawdata->OBS_rover, rawdata->GpsEph, rawdata->BdsEph, last_rover)) {
        return -1; // 连接关闭/错误
    }

    // 2) 若基站已有时间，先判差
    if (rawdata->OBS_base.SatNum > 0) {
        const double dt = GetDiffTime(&rawdata->OBS_rover.time, &rawdata->OBS_base.time); // rover - base
        if (fabs(dt) < SYNC_TOL) return 1; // 已同步
        if (dt < -0.5)           return 0; // 基站领先，外层先推进 rover
        // 否则基站落后，继续推进基站
    }

    // 3) 推进基站，直到追齐或超前
    while (true) {
        if (!ReadNextEpochSock(BaseSock, buff_base, len_base_rem,
            &rawdata->OBS_base, rawdata->GpsEph, rawdata->BdsEph, last_base)) {
            return -1; // 连接关闭/错误
        }
        const double dt = GetDiffTime(&rawdata->OBS_rover.time, &rawdata->OBS_base.time); // rover - base
        if (fabs(dt) < SYNC_TOL) return 1; // 同步成功
        if (dt < -0.5)           return 0; // 基站仍领先，交回外层推进 rover
        // dt >= +0.5：基站还落后，继续读
    }
}

void FormSDEpochObs(const EPOCHOBSDATA* EpkRover, const EPOCHOBSDATA* EpkBase, SD_EPOCHOBSDATA* SDObs)
{
    int satsum = 0;
    SDObs->SatNum = 0;
    SDObs->Time = EpkRover->time;
    //根据流动站卫星数进行循环，对于每颗流动站的卫星
    for (int i = 0; i < EpkRover->SatNum; i++)
    {
        //检查卫星号和系统号是否正常，卫星观测值是否完整，如果不正常，continue
        if (EpkRover->SatObs[i].Valid == false)continue;
        for (int j = 0; j < EpkBase->SatNum; j++)
        {
            //在基站观测值中查找与该流动站相同的卫星，如果未找到，continue
            if (EpkBase->SatObs[j].Prn == EpkRover->SatObs[i].Prn && EpkBase->SatObs[j].Satsys == EpkRover->SatObs[i].Satsys)
            {
                SDObs->SdSatObs[satsum].nRov = i;
                SDObs->SdSatObs[satsum].nBas = j;
                SDObs->SdSatObs[satsum].Prn = EpkBase->SatObs[j].Prn;
                SDObs->SdSatObs[satsum].Satsys = EpkBase->SatObs[j].Satsys;
                SDObs->SdSatObs[satsum].dP[0] = EpkRover->SatObs[i].P[0] - EpkBase->SatObs[j].P[0];
                SDObs->SdSatObs[satsum].dL[0] = EpkRover->SatObs[i].L[0] - EpkBase->SatObs[j].L[0];
                SDObs->SdSatObs[satsum].dP[1] = EpkRover->SatObs[i].P[1] - EpkBase->SatObs[j].P[1];
                SDObs->SdSatObs[satsum].dL[1] = EpkRover->SatObs[i].L[1] - EpkBase->SatObs[j].L[1];
                satsum++;
                break;
            }
        }
    }
    SDObs->SatNum = satsum;
}



void compSDMWGF(const SD_SATOBS satobs, MWGF& mwgf)
{
    double f1, f2, b1, b2;
    double L1, L2, P1, P2;

    mwgf.Sys = satobs.Satsys;
    mwgf.Prn = satobs.Prn;
    f1 = (mwgf.Sys == GPS) ? FG1_GPS : FG1_BDS;
    f2 = (mwgf.Sys == GPS) ? FG2_GPS : FG3_BDS;

    L1 = satobs.dL[0];
    L2 = satobs.dL[1];
    P1 = satobs.dP[0];
    P2 = satobs.dP[1];
    mwgf.MW = (f1 * L1 - f2 * L2) / (f1 - f2) - (f1 * P1 + f2 * P2) / (f1 + f2);
    mwgf.GF = L1 - L2;
    mwgf.PIF = (f1 * f1 * P1 - f2 * f2 * P2) / (f1 * f1 - f2 * f2);
}

//void DetectCycleSlip(SD_EPOCHOBSDATA& SDobs, RAWDATA Rawobs)
//{
//    MWGF CurComObs[MAXCHANNUM]{};  // 当前历元的MW/GF结果与平滑状态
//    int nRov, nBas;
//    for (int i = 0; i < SDobs.SatNum; i++)
//    {
//        SD_SATOBS& sd = SDobs.SdSatObs[i];
//        nRov = SDobs.SdSatObs[i].nRov;
//        nBas = SDobs.SdSatObs[i].nBas;
//        if (nRov < 0 || nBas < 0 ||!Rawobs.OBS_rover.SatPVT[nRov].Valid ||!Rawobs.OBS_base.SatPVT[nBas].Valid) 
//        {
//            SDobs.SdSatObs[i].Valid = false; continue;
//        }
//
//        // 1) 数据完整性：单差可为负，
//        if (fabs(SDobs.SdSatObs[i].dP[0]) < 1e-9 || fabs(SDobs.SdSatObs[i].dP[1]) < 1e-9 ||
//            fabs(SDobs.SdSatObs[i].dL[1]) < 1e-9 || fabs(SDobs.SdSatObs[i].dL[0]) < 1e-9)
//        {
//            SDobs.SdSatObs[i].Valid = false;
//            continue;
//        }
//
//        //locktime判断
//        const bool lock_ok =
//            Rawobs.OBS_rover.SatObs[nRov].LockTime[0] >= 6 && Rawobs.OBS_rover.SatObs[nRov].LockTime[1] >= 6 &&
//            Rawobs.OBS_base.SatObs[nBas].LockTime[0] >= 6 && Rawobs.OBS_base.SatObs[nBas].LockTime[1] >= 6;
//        if (!lock_ok) { SDobs.SdSatObs[i].Valid = false; continue; }
//
//
//        // 2) 计算当前GF/MW
//        compSDMWGF(sd, CurComObs[i]);
//
//        // 3) 查找上一历元平滑结果（按 Prn+Sys）
//        bool found = false;
//        for (int j = 0; j < MAXCHANNUM; j++)
//        {
//            if (SDobs.SdCObs[j].Prn == CurComObs[i].Prn &&
//                SDobs.SdCObs[j].Sys == CurComObs[i].Sys)
//            {
//                found = true;
//
//                const double dMW = CurComObs[i].MW - SDobs.SdCObs[j].MW;
//                const double dGF = CurComObs[i].GF - SDobs.SdCObs[j].GF;
//
//                // 4) 判阈（绝对值）
//                if (fabs(dMW) < MAX_dMW && fabs(dGF) < MAX_dGF)
//                {
//                    const int nprev = SDobs.SdCObs[j].n;
//                    CurComObs[i].MW = (SDobs.SdCObs[j].MW * nprev + CurComObs[i].MW) / (nprev + 1);
//                    CurComObs[i].n = nprev + 1;
//                    SDobs.SdSatObs[i].Valid = true;
//                }
//                else
//                {
//                    // 对齐“正确代码”：周跳时本历元不参与平滑，置 n=0
//                    SDobs.SdSatObs[i].Valid = false;
//                    CurComObs[i].n = 0;
//                }
//                break;
//            }
//        }
//
//        // 5) 首次出现或上一历元n==0（等价于未有效记录）：初始化为 n=1，认为本历元可用
//        // 新星：先记录平滑，不立即放行（避免第一历元噪声入解）
//        if (!found) {
//            CurComObs[i].n = 1;
//            SDobs.SdSatObs[i].Valid = false; // 等下一历元再用
//        }
//    }
//
//    // 6) 更新平滑缓存
//    memcpy(SDobs.SdCObs, CurComObs, MAXCHANNUM * sizeof(MWGF));
//}

void DetectCycleSlip(SD_EPOCHOBSDATA& SDobs, RAWDATA Rawobs)
{
    MWGF CurComObs[MAXCHANNUM]{};  // 当前历元的MW/GF结果与平滑状态

    for (int i = 0; i < SDobs.SatNum; i++)
    {
        SD_SATOBS& sd = SDobs.SdSatObs[i];
        const int nRov = sd.nRov;
        const int nBas = sd.nBas;

        // 基础索引/星历有效性检查
        if (nRov < 0 || nBas < 0 ||
            !Rawobs.OBS_rover.SatPVT[nRov].Valid ||
            !Rawobs.OBS_base.SatPVT[nBas].Valid)
        {
            sd.Valid = false;
            continue;
        }

        // 半周标记检查（0=坏，有半周；1=好）。任一端任一频点出现 half=0 判为不可用
        const bool half_ok =
            Rawobs.OBS_rover.SatObs[nRov].half[0] == 1 &&
            Rawobs.OBS_rover.SatObs[nRov].half[1] == 1 &&
            Rawobs.OBS_base.SatObs[nBas].half[0] == 1 &&
            Rawobs.OBS_base.SatObs[nBas].half[1] == 1;
        if (!half_ok) {
            sd.Valid = false;
            // 若上一历元存在该星平滑记录，将其“冻结”：n=0 表示需重新初始化
            // （无需显式修改 SdCObs，这里只是不参与本历元平滑；下一历元作为新星处理）
            continue;
        }

        // 原始单差数据完整性
        if (fabs(sd.dP[0]) < 1e-9 || fabs(sd.dP[1]) < 1e-9 ||
            fabs(sd.dL[0]) < 1e-9 || fabs(sd.dL[1]) < 1e-9)
        {
            sd.Valid = false;
            continue;
        }

        // 锁定时间（用于排除刚出现的周跳）
        const bool lock_ok =
            Rawobs.OBS_rover.SatObs[nRov].LockTime[0] >= 6 &&
            Rawobs.OBS_rover.SatObs[nRov].LockTime[1] >= 6 &&
            Rawobs.OBS_base.SatObs[nBas].LockTime[0] >= 6 &&
            Rawobs.OBS_base.SatObs[nBas].LockTime[1] >= 6;
        if (!lock_ok) {
            sd.Valid = false;
            continue;
        }

        // 计算当前历元 Melbourne-Wubbena / Geometry-Free
        compSDMWGF(sd, CurComObs[i]);

        // 查找上一历元平滑记录
        bool found_prev = false;
        for (int j = 0; j < MAXCHANNUM; j++)
        {
            if (SDobs.SdCObs[j].Prn == CurComObs[i].Prn &&
                SDobs.SdCObs[j].Sys == CurComObs[i].Sys)
            {
                found_prev = true;

                // 判定变化量
                const double dMW = CurComObs[i].MW - SDobs.SdCObs[j].MW;
                const double dGF = CurComObs[i].GF - SDobs.SdCObs[j].GF;

                // 若在阈值内 => 平滑更新；否则视为周跳（半周标记也会提前过滤）
                if (fabs(dMW) < 0.25 && fabs(dGF) < 0.05) {
                    const int nprev = SDobs.SdCObs[j].n;
                    CurComObs[i].MW = (SDobs.SdCObs[j].MW * nprev + CurComObs[i].MW) / (nprev + 1);
                    CurComObs[i].n = nprev + 1;
                    sd.Valid = true;
                }
                else {
                    // 周跳/异常：不参与平滑，本历元记为 n=0，标记不可用
                    CurComObs[i].n = 0;
                    sd.Valid = false;
                }
                break;
            }
        }

        // 首次出现（上一历元无记录）：初始化平滑计数，但先不给 Valid（等下一历元再放行）
        if (!found_prev) {
            CurComObs[i].n = 1;
            sd.Valid = false;
        }
    }

    // 写回平滑缓存
    memcpy(SDobs.SdCObs, CurComObs, sizeof(CurComObs));
}





void DetRefSat(const EPOCHOBSDATA* EpkR, const EPOCHOBSDATA* EpkB, SD_EPOCHOBSDATA* SDObs, DD_OBSDATA* DDObs)
{
    int i, j, n;
    double Sum[2] = { 0.0 }, MaxSum[2] = { 0.0 };
    int RefPrn[2] = { -1 }, RefIndex[2] = { -1 };
    int nRov, nBas;
    // short BDSsat[MAXBDSNUM], GPSsat[MAXGPSNUM],SDbds[MAXBDSNUM],SDgps[MAXGPSNUM];//储存GPS、BD索引号
    // short bestBDS[2], bestGPS[2];//基准星号，在原始观测值中的索引，为单差观测值中的索引
    // double maxBDScn0=0.0, maxGPScn0=0.0;

    /*基准星选取*/
    for (int i = 0; i < SDObs->SatNum; i++)
    {
        nBas = SDObs->SdSatObs[i].nBas;
        nRov = SDObs->SdSatObs[i].nRov;

        double elevDeg = EpkR->SatPVT[nRov].Elevation * Deg;
        if (elevDeg < cfg.elev_threshold) continue; // 按配置

        if (!SDObs->SdSatObs[i].Valid || !EpkR->SatPVT[nRov].Valid || !EpkB->SatPVT[nBas].Valid || !EpkB->SatObs[nBas].Valid || !EpkR->SatObs[nRov].Valid) continue;

        if (EpkR->SatObs[nRov].LockTime[0] < 6 || EpkR->SatObs[nRov].LockTime[1] < 6 ||
            EpkB->SatObs[nBas].LockTime[0] < 6 || EpkB->SatObs[nBas].LockTime[1] < 6) continue;

        n = SDObs->SdSatObs[i].Satsys == GPS ? 0 : 1;
        Sum[n] = EpkR->SatPVT[nRov].Elevation + EpkB->SatObs[nBas].cn0[0] + EpkB->SatObs[nBas].cn0[1]+ EpkR->SatObs[nRov].cn0[0] + EpkR->SatObs[nRov].cn0[1];

        if (Sum[n] > MaxSum[n]) {
            MaxSum[n] = Sum[n];
            RefPrn[n] = SDObs->SdSatObs[i].Prn;
            RefIndex[n] = i;
        }
    }

    for (i = 0; i < 2; i++) {
        if (MaxSum[i] < 200.0)
        {
            DDObs->RefPos[i] = -1;
            DDObs->RefPrn[i] = 0;
        }
        else
        {
            DDObs->RefPos[i] = RefIndex[i];
            DDObs->RefPrn[i] = RefPrn[i];
        }
    }
}

static inline double lambdaOf(int sys, int fidx) {
    // fidx: 0=L1/B1, 1=L2/B2 （按频点顺序）
    if (sys == GPS) {
        return (fidx == 0 ? WL1_GPS : WL2_GPS);
    }
    else { // BDS）
        return (fidx == 0 ? WL1_BDS : WL2_BDS);
    }
}
/* RTKFloat ----------------------------------
* RTK float solution
*
* args	:	rawdata		I/O	原始观测值
*					    I	单差观测值
*			ResB/ResR   I	spp定位初值中间结果
*					    I	双差基准星选取结果
*
* return:	execution status (0:ok,0>:error)
*--------------------------------------------*/
bool rtk_float_solution(RAWDATA* rawdata, POSRES* ResB, POSRES* ResR)
{

    //  1.设置基站和流动站位置初值
    //	2. 计算GPS和BDS双差卫星数（单双频混用：各系统每个频率的双差卫星数）
    const int nG = rawdata->DD_Obs.DDSatNum[0]; // GPS “非参考星”数
    const int nB = rawdata->DD_Obs.DDSatNum[1]; // 其它系统“非参考星”数
    const int nSat = nG + nB;
    if (nSat <= 4) return false;
    const int m = 4 * nSat;                // 每星4行：P1,P2,L1,L2
    const int n = 3 + 2 * nSat;            // 3个位置 + 每星两频模糊度

    double* W = new double[m]();                // 右端项w
    double* B = new double[m * n]();            // 设计矩阵
    double* P = new double[m * m]();            // 权阵（或R，根据配置）
    double* Bt = new double[n * m]();            // 设计矩阵
    double* Nbb = new double[n * n]();            // 正规方程矩阵 BᵀPB
    double* BtP = new double[n * m]();
    double* Nbb_inv = new double[n * n]();         // (BᵀPB)⁻¹
    double* BtPW = new double[n]();                // 右端向量 BᵀPW
    double* X = new double[n]();                // 当前估计（pos + amb）
    double* x = new double[n]();                // 本轮改正
    double* Q = new double[n * n]();            // 参数协方差

    //memset(W, 0, sizeof(double) * m);
    //memset(B, 0, sizeof(double) * m * n);
    //memset(P, 0, sizeof(double) * m * m);
    //memset(Bt, 0, sizeof(double) * n * m);
    //memset(Nbb, 0, sizeof(double) * n * n);
    //memset(BtP, 0, sizeof(double) * n * m);
    //memset(Nbb_inv, 0, sizeof(double) * n * n);
    //memset(BtPW, 0, sizeof(double) * n);
    //memset(X, 0, sizeof(double) * n);
    //memset(x, 0, sizeof(double) * n);
    //memset(Q, 0, sizeof(double) * n * n);

    Dmemcpy(ResR->Pos, X, 3);   //初始流动站坐标

    const int num = rawdata->SD_Obs.SatNum;

    //	3. 计算基站坐标到所有卫星的几何距离'
    double DistanceB[MAXBDSNUM + MAXGPSNUM]{}, DistanceR[MAXBDSNUM + MAXGPSNUM]{};

    for (int i = 0; i < num; i++)
    {
        int idxB = rawdata->SD_Obs.SdSatObs[i].nBas;
        double dx[3];
        vec_sub(rawdata->OBS_base.SatPVT[idxB].Pos, ResB->Pos, dx, 3);
        DistanceB[i] = norm(dx, 3);
    }
    //	4. 计算流动站到参考星的几何距离

    int iter = 0;

    do
    {
        for (int i = 0; i < num; i++)
        {
            int idxR = rawdata->SD_Obs.SdSatObs[i].nRov;
            double dx[3];
            vec_sub(rawdata->OBS_rover.SatPVT[idxR].Pos, ResR->Pos, dx, 3);
            DistanceR[i] = norm(dx, 3);
        }
        // 仅首轮迭代初始化模糊度初值（避免反复重置）
        const bool initAmbThisIter = (iter == 0);

        memset(W, 0, sizeof(double) * m);
        memset(B, 0, sizeof(double) * m * n);
        Dmemcpy(eye(m), P, m * m);
        memset(Bt, 0, sizeof(double) * n * m);
        memset(Nbb, 0, sizeof(double) * n * n);
        memset(BtP, 0, sizeof(double) * n * m);
        memset(Nbb_inv, 0, sizeof(double) * n * n);
        memset(BtPW, 0, sizeof(double) * n);
        memset(x, 0, sizeof(double) * n);

        int nosat[2]{ 0, nG };

        double lam1[2]{ WL1_GPS,WL1_BDS };
        double lam2[2]{ WL2_GPS,WL3_BDS };
        //	5. 对单差观测值进行循环  填充 B 和 W
        for (int i = 0; i < num; i++)
        {
            SD_SATOBS sd = rawdata->SD_Obs.SdSatObs[i];
            int SYSn = (sd.Satsys == GPS) ? 0 : 1;
            //	① 参考星不用计算，存在半周不参与计算
			if (!sd.Valid) continue;//没有加上半波跳变检测
            if (sd.Prn == rawdata->DD_Obs.RefPrn[SYSn]) continue;    // 跳过参考星
            if (!rawdata->OBS_rover.SatPVT[sd.nRov].Valid || !rawdata->OBS_base.SatPVT[sd.nBas].Valid) continue;


            // 组合索引（跨系统累加的“非参考星”序号），统一用于 X、B、W
            const int kTot = (nosat[0] + nosat[1] - nG); // 已装入的“非参考星”总数

            if (initAmbThisIter) {

                int RefIdx[2] = { rawdata->DD_Obs.RefPos[0], rawdata->DD_Obs.RefPos[1] };
   
                X[3 + kTot * 2 + 0] = (-(sd.dP[0] - rawdata->SD_Obs.SdSatObs[RefIdx[SYSn]].dP[0]) + (sd.dL[0] - rawdata->SD_Obs.SdSatObs[RefIdx[SYSn]].dL[0]));
                X[3 + kTot * 2 + 1] = (-(sd.dP[1] - rawdata->SD_Obs.SdSatObs[RefIdx[SYSn]].dP[1]) + (sd.dL[1] - rawdata->SD_Obs.SdSatObs[RefIdx[SYSn]].dL[1]));
            }
            //	② 线性化双差观测方程得到B矩阵和W向量
           

            // 参考星与该星的索引
            int refSD = rawdata->DD_Obs.RefPos[SYSn];
            int refposR = rawdata->SD_Obs.SdSatObs[refSD].nRov;
            int satposR = sd.nRov;

            // 写入 B：该星的4行，对位置偏导相同
            const int rowBase = (nosat[SYSn] * 4);
            for (int k = 0; k < 4; k++) {
                for (int j = 0; j < 3; j++) {
                    B[(rowBase + k) * n + j] =
                        (ResR->Pos[j] - rawdata->OBS_rover.SatPVT[satposR].Pos[j]) / DistanceR[i]
                        - (ResR->Pos[j] - rawdata->OBS_rover.SatPVT[refposR].Pos[j]) / DistanceR[refSD];
                }
            }
            // 相位两行的模糊度列m：使用统一索引 kTot
            B[(rowBase + 2) * n + (3 + 2 * kTot + 0)] = lam1[SYSn];
            B[(rowBase + 3) * n + (3 + 2 * kTot + 1)] = lam2[SYSn];

            // W：伪距两行、相位两行（相位=λ*(DDL - N) - ρDD；伪距=DDP - ρDD）
            const double ddN1 = X[3 + kTot * 2 + 0];
            const double ddN2 = X[3 + kTot * 2 + 1];
            const double rhoDD = (DistanceR[i] - DistanceR[refSD]) - (DistanceB[i] - DistanceB[refSD]);
            double w1 = (rawdata->SD_Obs.SdSatObs[i].dP[0] - rawdata->SD_Obs.SdSatObs[refSD].dP[0]) - rhoDD;
            double w2= (rawdata->SD_Obs.SdSatObs[i].dP[1] - rawdata->SD_Obs.SdSatObs[refSD].dP[1]) - rhoDD;
            double w3=  rawdata->SD_Obs.SdSatObs[i].dL[0] - rawdata->SD_Obs.SdSatObs[refSD].dL[0] - lam1[SYSn]*ddN1 - rhoDD;
            double w4=  rawdata->SD_Obs.SdSatObs[i].dL[1] - rawdata->SD_Obs.SdSatObs[refSD].dL[1] - lam2[SYSn]*ddN2 - rhoDD;
            W[rowBase + 0] = w1;
            W[rowBase + 1] = w2;
            W[rowBase + 2] = w3;
            W[rowBase + 3] = w4;
            // 下一颗非参考星
            nosat[SYSn]++;
        }

        const int usedG = max(0, min(nosat[0], nG));
        const int usedB = max(0, min(nosat[1] - nG, nB));

        // 清零，再按两块（GPS、BDS）填充
        memset(P, 0, sizeof(double) * m * m);

        // 噪声（米）：伪距/相位
        const double sigmaP = cfg.sigP;
        const double sigmaL = cfg.sigL;

        auto fill_block = [&](int rowOffset, int cnt) {
            if (cnt <= 0) return;
            const int total = cnt + 1; // 含参考星

            // 权值：对角 (total-1)/(total·2σ²)，非对角 −1/(total·2σ²)
            const double wP_off = 1.0 / (total * 2.0 * sigmaP * sigmaP);
            const double wL_off = 1.0 / (total * 2.0 * sigmaL * sigmaL);
            const double wP_diag = (total - 1) * wP_off;
            const double wL_diag = (total - 1) * wL_off;

            for (int i = 0; i < cnt; ++i) {
                for (int j = 0; j < cnt; ++j) {
                    const bool diag = (i == j);
                    const double pval = diag ? wP_diag : -wP_off;
                    const double lval = diag ? wL_diag : -wL_off;

                    // 同频同型：P1/P2
                    P[(rowOffset + i * 4 + 0) * m + (rowOffset + j * 4 + 0)] = pval;
                    P[(rowOffset + i * 4 + 1) * m + (rowOffset + j * 4 + 1)] = pval;
                    // 相位 L1/L2
                    P[(rowOffset + i * 4 + 2) * m + (rowOffset + j * 4 + 2)] = lval;
                    P[(rowOffset + i * 4 + 3) * m + (rowOffset + j * 4 + 3)] = lval;
                }
            }
            };

        // 注意：你的行布局是 GPS 从 0 起，BDS 从 4*nG 起（不是 4*usedG）
        fill_block(/*rowOffset=*/0,       /*cnt=*/usedG);
        fill_block(/*rowOffset=*/4 * nG,  /*cnt=*/usedB);
        //mat_print(B, m, n);

        //	7. 最小二乘解算
        mat_trans(B, Bt, m, n);
        mat_mul(Bt, P, BtP, n, m, m, m);
        mat_mul(BtP, B, Nbb, n, m, m, n);
        mat_mul(BtP, W, BtPW, n, m, m, 1);
        mat_inv(Nbb, Nbb_inv, n);
        mat_mul(Nbb_inv, BtPW, x, n, n, n, 1);
        vec_plus(X, x, X, n);
        Dmemcpy(X, ResR->Pos, 3);
        iter++;
    } while (norm(x, 3) > 1e-3 && iter < 10);

    //	9. 流动站位置增量大于阈值或迭代次数小于阈值，返回4迭代计算，否则浮点解计算完成。
    if (norm(x, 3) > 1e-3 && iter >= 10){
        delete[] W; delete[] B; delete[] P; delete[] Bt; delete[] Nbb; delete[] BtP;
        delete[] Nbb_inv; delete[] BtPW; delete[] Q; delete[] X; delete[] x;
        return false;
    }

    //	10. 保存双差浮点解模糊度及其协因数矩阵，用于LAMBDA模糊度固定
    Dmemcpy(Nbb_inv, Q, n* n);
    Dmemcpy(Q, rawdata->DD_Obs.Q, n* n);
    Dmemcpy(X + 3, rawdata->DD_Obs.floatAmb, 2 * nSat);
    //vec_print(X, n);
    vec_sub(X, ResB->Pos, rawdata->DD_Obs.dPos, 3);

    //	11. 精度评价
    double sigma[1];
    double* v = new double[m]();
    double* Bx = new double[m]();
    double* vt = new double[m]();
    double* vtP = new double[m]();
    memset(v, 0, sizeof(double) * m);
    memset(Bx, 0, sizeof(double)* m);
    memset(vt, 0, sizeof(double)* m);
    memset(vtP, 0, sizeof(double)* m);
	//vec_print(W, m);
    mat_mul(B, x, Bx, m, n, n, 1);  
    vec_sub(W, Bx, v, m); // v = W - Bx
    mat_trans(v, vt, m, 1);
    mat_mul(vt, P, vtP, 1, m, m, m);
    mat_mul(vtP, v, sigma, 1, m, m, 1);

    RTKRES.sigma_pos = sqrt(sigma[0] / (2 * (nSat) - 3));    //中误差

    //11.2 Qxyz:最小二乘中，将Qxzy储存到Dxyz中，可根据sigma进行转换。
    //		如果固定，可以将固定后的再储存
// 直接将 Q 的左上 3x3 拷贝到 AccessRes
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            RTKRES.dxyz[i * 3 + j] = Q[i * (n) + j];
        }
    }

    // 释放
    delete[] v; delete[] Bx; delete[] vt; delete[] vtP;
    delete[] W; delete[] B; delete[] P; delete[] Bt; delete[] Nbb; delete[] BtP;
    delete[] Nbb_inv; delete[] BtPW; delete[] Q; delete[] X; delete[] x;
    return true;
}

int rtk_fix_solution(int Ambnum, double* X, double* Q, DD_OBSDATA& DDObs, POSRES* Bas, POSRES* Rov)
{
    // 输入检查
    DDObs.bFixed = false;
    if (Ambnum <= 0 || !X || !Q || !Bas || !Rov) return -1;

    const int n = 3 + Ambnum;          // 参数维度：3(pos)+Amb
    const double RatioThres = cfg.ratio_threshold;

    // 1) 提取 Qnn（Amb×Amb）与模糊度向量 n_hat（cycles）
    double* Qnn = new double[Ambnum * Ambnum]();
    for (int r = 0; r < Ambnum; ++r) {
        for (int c = 0; c < Ambnum; ++c) {
            Qnn[r * Ambnum + c] = Q[(3 + r) * n + (3 + c)];
        }
    }


    double* n_hat = new double[Ambnum]();
    Dmemcpy(X + 3, n_hat, Ambnum);     // cycles
	//vec_print(n_hat, Ambnum);
	//mat_print(Qnn, Ambnum, Ambnum);
    
    // 2) LAMBDA 整周搜索（Qnn 单位=cycles^2，n_hat 单位=cycles）
    if (lambda(Ambnum, 2, n_hat, Qnn, DDObs.FixedAmb, DDObs.ResAmb) != 0) {
        delete[] Qnn; delete[] n_hat;
        return -1; // LAMBDA 失败
    }

    // 3) 比率检验
    if (DDObs.ResAmb[0] > 1e-12 && DDObs.ResAmb[1] > 0.0) {
        DDObs.Ratio = DDObs.ResAmb[1] / DDObs.ResAmb[0];
    } else {
        // 不可计算时给 0，表示无法通过固定判定
        DDObs.Ratio = 0.0;
    }
    DDObs.bFixed = (DDObs.Ratio >= RatioThres);
    if (!DDObs.bFixed) {
        delete[] Qnn; delete[] n_hat;
        return 0; // 未过比率，不固定
    }

    double* Qnn_ = new double[Ambnum * Ambnum]();
	mat_inv(Qnn, Qnn_, Ambnum);



    double* Qnn_inv = new double[Ambnum * Ambnum]();
    if (DDObs.bFixed)
    {
        // 3) Qnn^-1
        double* Qnn_inv = new double[Ambnum * Ambnum]();
        if (mat_inv(Qnn, Qnn_inv, Ambnum) != 0) {
            delete[] Qnn; delete[] n_hat; delete[] Qnn_inv;
            return 0;
        }

        // 4) ΔN = n_hat − n_fix（cycles）
        double* dN = new double[Ambnum]();
        vec_sub(n_hat, DDObs.FixedAmb, dN, Ambnum);

        // 5) Qxn = Q[0:3, 3:3+Amb]（3×Amb）
        double* Qxn = new double[3 * Ambnum]();
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < Ambnum; ++c)
                Qxn[r * Ambnum + c] = Q[r * n + (3 + c)];

        // 6) Qxn·Qnn^-1（3×Amb）
        double* QxnQnni = new double[3 * Ambnum]();
        mat_mul(Qxn, Qnn_inv, QxnQnni, 3, Ambnum, Ambnum, Ambnum);

        // 7) dPos = (Qxn·Qnn^-1)·ΔN（3×1）
        double dPos[3]{ 0.0,0.0,0.0 };
        mat_mul(QxnQnni, dN, dPos, 3, Ambnum, Ambnum, 1);

        // 8) 位置回代与写回
        X[0] -= dPos[0]; X[1] -= dPos[1]; X[2] -= dPos[2];
        for (int i = 0; i < Ambnum; ++i) X[3 + i] = DDObs.FixedAmb[i];

        Rov->Pos[0] = X[0]; Rov->Pos[1] = X[1]; Rov->Pos[2] = X[2];
        DDObs.dPos[0] = X[0] - Bas->Pos[0];
        DDObs.dPos[1] = X[1] - Bas->Pos[1];
        DDObs.dPos[2] = X[2] - Bas->Pos[2];

        // 9) Qxx_fix = Qxx − Qxn·Qnn^-1·Qnx
        double Qxx[9]{ 0.0 }, dQxx[9]{ 0.0 }, Qxx_fix[9]{ 0.0 };
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                Qxx[r * 3 + c] = Q[r * n + c];

        double* Qnx = new double[Ambnum * 3]();
        for (int r = 0; r < Ambnum; ++r)
            for (int c = 0; c < 3; ++c)
                Qnx[r * 3 + c] = Q[(3 + r) * n + c];

        mat_mul(QxnQnni, Qnx, dQxx, 3, Ambnum, Ambnum, 3);
        for (int i = 0; i < 9; ++i) Qxx_fix[i] = Qxx[i] - dQxx[i];

        for (int i = 0; i < 9; ++i) RTKRES.dxyz[i] = Qxx_fix[i];

        // 释放
        delete[] Qnn; delete[] n_hat; delete[] Qnn_inv;
        delete[] dN; delete[] Qxn; delete[] QxnQnni; delete[] Qnx;

        return 1;
    }
    return 0;

}

int rtkpos_lsq(RAWDATA& raw, POSRES& ResB, POSRES& ResR)
{
    // 1) 流动站 SPP 初值
    if (pntpos(&raw.OBS_rover, raw.GpsEph, raw.BdsEph, &ResR) != 0) return -1;
    if (pntpos(&raw.OBS_base, raw.GpsEph, raw.BdsEph, &ResB) != 0) return -1;
    //利用bp进行解算
    ResB = raw.OBS_base.best_pos;
    // 2) 单差/周跳/参考星
    FormSDEpochObs(&raw.OBS_rover, &raw.OBS_base, &raw.SD_Obs);
    if (raw.SD_Obs.SatNum <= 0) return -1;

    DetectCycleSlip(raw.SD_Obs, raw);
    DetRefSat(&raw.OBS_rover, &raw.OBS_base, &raw.SD_Obs, &raw.DD_Obs);
    if (raw.DD_Obs.RefPos[0] < 0 && raw.DD_Obs.RefPos[1] < 0) return -1;

    // 3) 统计非参考星数
    raw.DD_Obs.DDSatNum[0] = raw.DD_Obs.DDSatNum[1] = 0;
    int refGpsIdx = raw.DD_Obs.RefPos[0];
    int refBdsIdx = raw.DD_Obs.RefPos[1];
    for (int i = 0; i < raw.SD_Obs.SatNum; i++) {
        SD_SATOBS& sd = raw.SD_Obs.SdSatObs[i];
        if (!sd.Valid) continue;
        if (sd.Satsys == GPS) {
            if (refGpsIdx >= 0 && i == refGpsIdx) continue;
            raw.DD_Obs.DDSatNum[0]++;
        }
        else if (sd.Satsys == BDS) {
            if (refBdsIdx >= 0 && i == refBdsIdx) continue;
            raw.DD_Obs.DDSatNum[1]++;
        }
    }
    raw.DD_Obs.Sats = raw.DD_Obs.DDSatNum[0] + raw.DD_Obs.DDSatNum[1];
    if (raw.DD_Obs.Sats < 1) return -1;

    // 4) 浮点解
    if (!rtk_float_solution(&raw, &ResB, &ResR)) return -1;

    // 5) 固定解（LAMBDA + 比率检验），仅当有待定模糊度时尝试
    const int Ambnum = 2 * raw.DD_Obs.Sats;
    if (Ambnum <= 0) return 0;

    const int n = 3 + Ambnum;
    double* Xfull = new double[n]();
    // X = [PosR(3), N_float(Ambnum)]
    Dmemcpy(ResR.Pos, Xfull, 3);
    Dmemcpy(raw.DD_Obs.floatAmb, Xfull + 3, Ambnum);

    raw.DD_Obs.bFixed = false;
    const int retFix = rtk_fix_solution(Ambnum, Xfull, raw.DD_Obs.Q, raw.DD_Obs, &ResB, &ResR);

    delete[] Xfull;

    // 返回：1=固定成功；0=固定未通过；-1=失败（已在上面返回）
    return (retFix == 1 && raw.DD_Obs.bFixed) ? 1 : 0;
}

/* EKF: 内部状态与工具 ======================== */

// 简单卡尔曼量测更新（与样例等价的 H/K 计算）
int ekf_filter(double* x, double* P, const double* H, const double* v, const double* R, int n, int m)
{
    // 分配
    double* Ht = new double[n * m]();      // H^T
    double* PHt = new double[n * m]();      // P*H^T
    double* HPHt = new double[m * m]();
    double* S = new double[m * m]();      // S = HPH^T + R
    double* SInv = new double[m * m]();      // S^{-1}
    double* K = new double[n * m]();      // K = PH^T S^{-1}
    double* Kv = new double[n]();          // K*v
    double* KH = new double[n * n]();      // K*H
    double* I = new double[n * n]();      // I
    double* ImKH = new double[n * n]();      // I-KH
    double* ImKH_P = new double[n * n]();    // (I-KH)P
    double* KRKt = new double[n * n]();      // K R K^T
    double* Rt = new double[m * m]();      // R^T（对称时=R）
    double* KRt = new double[n * m]();      // K R
    double* Kt = new double[m * n]();      // K^T
    // H^T
    mat_trans(H, Ht, m, n);
    // PH^T
    mat_mul(P, Ht, PHt, n, n, n, m);
    // S = H P H^T + R

    mat_mul(H, P, S, m, n, n, n);            // S = H*P
    mat_mul(S, Ht, HPHt, m, n, n, m);        // HPH^T
	mat_plus(HPHt, R, S, m , m);
    // S^{-1}
    if (mat_inv(S, SInv, m) != 0) {
        delete[] Ht; delete[] PHt; delete[] S; delete[] SInv; delete[] K; delete[] Kv; delete[] KH; delete[] I;
        delete[] ImKH; delete[] ImKH_P; delete[] KRKt; delete[] Rt; delete[] KRt; delete[] Kt; delete[] HPHt;
        return -1;
    }
    // K = PH^T S^{-1}
    mat_mul(PHt, SInv, K, n, m, m, m);
    // x = x + K v
    mat_mul(K, v, Kv, n, m, m, 1);
  /*  vec_print(v, m);*/

    vec_plus(x, Kv, x, n);
    // P = (I-KH)P(I-KH)^T + K R K^T
    mat_mul(K, H, KH, n, m, m, n);
    for (int i = 0; i < n * n; ++i) I[i] = 0.0;
    for (int i = 0; i < n; ++i) I[i * n + i] = 1.0;
    for (int i = 0; i < n * n; ++i) ImKH[i] = I[i] - KH[i];
    // (I-KH)P
    mat_mul(ImKH, P, ImKH_P, n, n, n, n);
    // (I-KH)P(I-KH)^T
    double* ImKH_t = new double[n * n]();
    mat_trans(ImKH, ImKH_t, n, n);
    double* P_new = new double[n * n]();
    mat_mul(ImKH_P, ImKH_t, P_new, n, n, n, n);
    // K R K^T
    mat_trans(R, Rt, m, m);
    mat_mul(K, R, KRt, n, m, m, m);
    mat_trans(K, Kt, n, m);
    mat_mul(KRt, Kt, KRKt, n, m, m, n);
    for (int i = 0; i < n * n; ++i) P[i] = P_new[i] + KRKt[i];

    // 释放
    delete[] Ht; delete[] PHt; delete[] S; delete[] SInv; delete[] K; delete[] Kv; delete[] KH; delete[] I;
    delete[] ImKH; delete[] ImKH_P; delete[] KRKt; delete[] Rt; delete[] KRt; delete[] Kt; delete[] HPHt;
    delete[] ImKH_t; delete[] P_new;
    return 0;
}


// 将当前“非参考星”列表（按GPS后BDS顺序）填充到输出数组，返回个数
int collect_used_sats(const RAWDATA& raw, int outIdxSD[], int outSys[], int outPrn[])
{
    int n = 0;
    int refGps = raw.DD_Obs.RefPos[0];
    int refBds = raw.DD_Obs.RefPos[1];

    // 先GPS
    for (int i = 0; i < raw.SD_Obs.SatNum; ++i) {
        SD_SATOBS sd = raw.SD_Obs.SdSatObs[i];
        if (!sd.Valid) continue;
        if (sd.Satsys != GPS) continue;
        if (refGps >= 0 && i == refGps) continue;
        outIdxSD[n] = i;
        outSys[n] = 0;
        outPrn[n] = sd.Prn;
        n++;
    }
    // 再BDS
    for (int i = 0; i < raw.SD_Obs.SatNum; ++i) {
        SD_SATOBS sd = raw.SD_Obs.SdSatObs[i];
        if (!sd.Valid) continue;
        if (sd.Satsys != BDS) continue;
        if (refBds >= 0 && i == refBds) continue;
        outIdxSD[n] = i;
        outSys[n] = 1;
        outPrn[n] = sd.Prn;
        n++;
    }
    return n;
}

/* EKF 主流程 ======================== */


// 基于卡尔曼滤波的RTK定位
int rtkpos_ekf(RAWDATA& raw, POSRES& ResB, POSRES& ResR)
{
    // 1) 流动站 SPP 初值
    if (pntpos(&raw.OBS_rover, raw.GpsEph, raw.BdsEph, &ResR) != 0) return -1;
    int firstTry = pntpos(&raw.OBS_base, raw.GpsEph, raw.BdsEph, &ResB);
    if (pntpos(&raw.OBS_base, raw.GpsEph, raw.BdsEph, &ResB) != 0) return -1;

    //利用bp进行解算
    ResB = raw.OBS_base.best_pos;
    // 2) 单差/周跳/参考星
    FormSDEpochObs(&raw.OBS_rover, &raw.OBS_base, &raw.SD_Obs);
    if (raw.SD_Obs.SatNum <= 0) return -1;

    DetectCycleSlip(raw.SD_Obs, raw);
    DetRefSat(&raw.OBS_rover, &raw.OBS_base, &raw.SD_Obs, &raw.DD_Obs);
    if (raw.DD_Obs.RefPos[0] < 0 && raw.DD_Obs.RefPos[1] < 0) return -1;

    // 3) 统计非参考星数
    int idxSD[MAXCHANNUM]{}, sysAB[MAXCHANNUM]{}, prnAB[MAXCHANNUM]{};
    raw.DD_Obs.DDSatNum[0] = raw.DD_Obs.DDSatNum[1] = 0;
    const int refGps = raw.DD_Obs.RefPos[0];
    const int refBds = raw.DD_Obs.RefPos[1];

    int used = 0;
    // GPS
    for (int i = 0; i < raw.SD_Obs.SatNum; ++i) {
        SD_SATOBS sd = raw.SD_Obs.SdSatObs[i];
        if (!sd.Valid) continue;
        if (sd.Satsys != GPS) continue;
        if (refGps >= 0 && i == refGps) continue;

        // 高度角门控(仅新星)这个地方设计有点复杂
        bool wasTracked = false;
        if (g_ekf.inited) {
            for (int t = 0; t < g_ekf.usedSatNum; ++t) {
                if (g_ekf.usedPrn[t] == sd.Prn && g_ekf.usedSys[t] == 0) { wasTracked = true; break; }
            }
        }
        if (!wasTracked) {
            int nRov = sd.nRov;
            if (nRov >= 0 && raw.OBS_rover.SatPVT[nRov].Valid) {
                double elevDeg = raw.OBS_rover.SatPVT[nRov].Elevation * Deg;
                if (elevDeg < cfg.elev_threshold) {
                    // 仰角未达到阈值：暂不加入跟踪列表（等待以后“升起”）
                    continue;
                }
            }
            else {
                // 没有有效卫星姿态也不加入
                continue;
            }
        }
        idxSD[used] = i;
        sysAB[used] = 0;
        prnAB[used] = sd.Prn;
        used++;
        raw.DD_Obs.DDSatNum[0]++;
    }
    // BDS
    for (int i = 0; i < raw.SD_Obs.SatNum; ++i) {
        SD_SATOBS sd = raw.SD_Obs.SdSatObs[i];
        if (!sd.Valid) continue;
        if (sd.Satsys != BDS) continue;
        if (refBds >= 0 && i == refBds) continue;

        // 高度角判断(新星)
        bool wasTracked = false;
        if (g_ekf.inited) {
            for (int t = 0; t < g_ekf.usedSatNum; ++t) {
                if (g_ekf.usedPrn[t] == sd.Prn && g_ekf.usedSys[t] == 1) { wasTracked = true; break; }
            }
        }
        if (!wasTracked) {

            if (sd.nRov >= 0 && raw.OBS_rover.SatPVT[sd.nRov].Valid) {
                double elevDeg = raw.OBS_rover.SatPVT[sd.nRov].Elevation * Deg;
                if (elevDeg < cfg.elev_threshold) {
                    continue;
                }
            }
            else {
                continue;
            }
        }
        idxSD[used] = i;
        sysAB[used] = 1;
        prnAB[used] = sd.Prn;
        used++;
        raw.DD_Obs.DDSatNum[1]++;
    }
    raw.DD_Obs.Sats = used;
    if (used <= 0) return -1;

    // 4) 量维度
    const int m = 4 * used;                 // P1,P2,L1,L2
    const int n = 3 + 2 * used;             // XYZ + 每星2个模糊度

    // 5) 几何距离
    double* DistanceR = new double[raw.SD_Obs.SatNum]();
    double* DistanceB = new double[raw.SD_Obs.SatNum]();
    for (int i = 0; i < raw.SD_Obs.SatNum; ++i) {
        const int iR = raw.SD_Obs.SdSatObs[i].nRov;
        const int iB = raw.SD_Obs.SdSatObs[i].nBas;
        double dR[3], dBv[3];
        vec_sub(raw.OBS_rover.SatPVT[iR].Pos, ResR.Pos, dR, 3);
        vec_sub(raw.OBS_base.SatPVT[iB].Pos, ResB.Pos, dBv, 3);
        DistanceR[i] = norm(dR, 3);
        DistanceB[i] = norm(dBv, 3);
    }


    // 6) 对齐/初始化 EKF 状态
    bool ambReset[MAXCHANNUM]{}; // 记录本历元哪些模糊度被“重置”（新星或周跳）
    if (!g_ekf.inited) 
    {
        // 初始化为零
        fill(begin(g_ekf.X), end(g_ekf.X), 0.0);
        fill(begin(g_ekf.P), end(g_ekf.P), 0.0);

        // 位置
        g_ekf.X[0] = ResR.Pos[0];
        g_ekf.X[1] = ResR.Pos[1];
        g_ekf.X[2] = ResR.Pos[2];
        for (int i = 0; i < 3; ++i) g_ekf.P[i * n + i] =49.0; // 初始位置方差7m

        // 模糊度初始化（cycles），N ≈ -(DDP)/λ + DDL；P 取较大先验 ((4/λ)^2)
        for (int k = 0; k < used; ++k) {
            const int iSD = idxSD[k];
            const int sys = sysAB[k];
            const int ref = (sys == 0 ? refGps : refBds);
            const double lam1 = (sys == 0 ? WL1_GPS : WL1_BDS);
            const double lam2 = (sys == 0 ? WL2_GPS : WL3_BDS);

            g_ekf.X[3 + 2 * k + 0] = (-(raw.SD_Obs.SdSatObs[iSD].dP[0] - raw.SD_Obs.SdSatObs[ref].dP[0]) + (raw.SD_Obs.SdSatObs[iSD].dL[0] - raw.SD_Obs.SdSatObs[ref].dL[0])) / lam1;
            g_ekf.X[3 + 2 * k + 1] = (-(raw.SD_Obs.SdSatObs[iSD].dP[1] - raw.SD_Obs.SdSatObs[ref].dP[1]) + (raw.SD_Obs.SdSatObs[iSD].dL[1] - raw.SD_Obs.SdSatObs[ref].dL[1])) / lam2;

            g_ekf.P[(3 + 2 * k + 0) * n + (3 + 2 * k + 0)] = 16.0;//4
            g_ekf.P[(3 + 2 * k + 1) * n + (3 + 2 * k + 1)] = 16.0;

            ambReset[k] = true; // 首次历元视作“新星”
        }

        g_ekf.usedSatNum = used;
        for (int k = 0; k < used; ++k) {
            g_ekf.usedPrn[k] = prnAB[k];
            g_ekf.usedSys[k] = sysAB[k];
        }
        g_ekf.refGps = refGps;
        g_ekf.refBds = refBds;
        g_ekf.inited = true;
    }
    else 
    {
        // 对齐到新顺序：老星无周跳继承 X/P，新星或周跳重置
        bool refChanged = (g_ekf.refGps != refGps) || (g_ekf.refBds != refBds);

        double Xnew[3 + 2 * MAXCHANNUM]{};
        double Pnew[(3 + 2 * MAXCHANNUM) * (3 + 2 * MAXCHANNUM)]{};
        const int nOld = 3 + 2 * g_ekf.usedSatNum;

        // 位置直接继承（方差至少保持200）
        Xnew[0] = g_ekf.X[0]; 
        Xnew[1] = g_ekf.X[1]; 
        Xnew[2] = g_ekf.X[2];
        for (int i = 0; i < 3; ++i)
            Pnew[i * n + i] = g_ekf.P[i * nOld + i];

        for (int k = 0; k < used; ++k) {
            int oldIdx = -1;
            for (int t = 0; t < g_ekf.usedSatNum; ++t) {
                if (g_ekf.usedPrn[t] == prnAB[k] && g_ekf.usedSys[t] == sysAB[k]) // 在旧列表中查找同 PRN+SYS的旧卫星记录到oldIdx
                { 
                    oldIdx = t; break; 
                }
            }
            const int iSD = idxSD[k];
            const int sys = sysAB[k];
            const int ref = (sys == 0 ? refGps : refBds);
            const double lam1 = (sys == 0 ? WL1_GPS : WL1_BDS);
            const double lam2 = (sys == 0 ? WL2_GPS : WL3_BDS);
            bool slip = false;
            const int nR = raw.SD_Obs.SdSatObs[iSD].nRov;
            const int nB = raw.SD_Obs.SdSatObs[iSD].nBas;
            if (nR >= 0 && nB >= 0) { // 周跳 任一端任一频点锁定时间 < 6 视作周跳
                slip = (raw.OBS_rover.SatObs[nR].LockTime[0] < 6 ||
                        raw.OBS_rover.SatObs[nR].LockTime[1] < 6 ||
                        raw.OBS_base.SatObs[nB].LockTime[0] < 6 || 
                        raw.OBS_base.SatObs[nB].LockTime[1] < 6);
            }

            if (oldIdx >= 0 && !slip && !refChanged) {
                // 无周跳老星：继承模糊度
                Xnew[3 + 2 * k + 0] = g_ekf.X[3 + 2 * oldIdx + 0];
                Xnew[3 + 2 * k + 1] = g_ekf.X[3 + 2 * oldIdx + 1];
                Pnew[(3 + 2 * k + 0) * n + (3 + 2 * k + 0)] =
                    g_ekf.P[(3 + 2 * oldIdx + 0) * nOld + (3 + 2 * oldIdx + 0)];
                Pnew[(3 + 2 * k + 1) * n + (3 + 2 * k + 1)] =
                    g_ekf.P[(3 + 2 * oldIdx + 1) * nOld + (3 + 2 * oldIdx + 1)];
                ambReset[k] = false;
            }
            else {
                // 新星或周跳：重置 N，并给先验方差,同上
                Xnew[3 + 2 * k + 0] = ((raw.SD_Obs.SdSatObs[iSD].dL[0] - raw.SD_Obs.SdSatObs[ref].dL[0]) - (raw.SD_Obs.SdSatObs[iSD].dP[0] - raw.SD_Obs.SdSatObs[ref].dP[0])) / lam1;
                Xnew[3 + 2 * k + 1] = ((raw.SD_Obs.SdSatObs[iSD].dL[1] - raw.SD_Obs.SdSatObs[ref].dL[1]) - (raw.SD_Obs.SdSatObs[iSD].dP[1] - raw.SD_Obs.SdSatObs[ref].dP[1])) / lam2;
                Pnew[(3 + 2 * k + 0) * n + (3 + 2 * k + 0)] = 16;
                Pnew[(3 + 2 * k + 1) * n + (3 + 2 * k + 1)] = 16;

                ambReset[k] = true;
            }
        }

        // 覆盖到 g_ekf
        fill(begin(g_ekf.X), end(g_ekf.X), 0.0);
        fill(begin(g_ekf.P), end(g_ekf.P), 0.0);
        for (int i = 0; i < n; ++i) g_ekf.X[i] = Xnew[i];
        for (int r = 0; r < n; ++r)
            for (int c = 0; c < n; ++c)
                g_ekf.P[r * n + c] = Pnew[r * n + c];

        g_ekf.usedSatNum = used;
        for (int k = 0; k < used; ++k) 
        { 
            g_ekf.usedPrn[k] = prnAB[k];
            g_ekf.usedSys[k] = sysAB[k];
        }
        g_ekf.refGps = refGps;
        g_ekf.refBds = refBds;
    }

    // 7) 时间更新
    // 状态转移矩阵：位置使用常速度模型，模糊度保持不变
    // 过程噪声Q设置：
    double pos_Q = cfg.pos_Q; // 位置过程噪声：5m方差
    double amb_Q = cfg.amb_Q; // 模糊度过程噪声：微小量

    for (int i = 0; i < 3; ++i)
        g_ekf.P[i * n + i] += pos_Q;

    for (int k = 0; k < used; ++k) {
        if (!ambReset[k]) {
            // 无周跳模糊度：加微小过程噪声
            g_ekf.P[(3 + 2 * k + 0) * n + (3 + 2 * k + 0)] += amb_Q;
            g_ekf.P[(3 + 2 * k + 1) * n + (3 + 2 * k + 1)] += amb_Q;
        }
        // 有周跳或新星的模糊度已经在重置时设置了较大方差，不再额外加噪声
    }

    // 8) 量测矩阵 B、残差 w、量测协方差 Pm
    double* B = new double[m * n]();
    double* w = new double[m]();
    double* R = new double[m * m]();

    for (int k = 0; k < used; ++k) {
        const int iSD = idxSD[k];
        const int sys = sysAB[k];
        const int refSD = (sys == 0 ? refGps : refBds);
        const int rowBase = 4 * k;

        const int iR = raw.SD_Obs.SdSatObs[iSD].nRov;
        const int iRrf = raw.SD_Obs.SdSatObs[refSD].nRov;

        // 简化位置偏导计算（四行相同）
        for (int row = 0; row < 4; ++row) {
            for (int j = 0; j < 3; ++j) {
                B[(rowBase + row) * n + j] =
                    (g_ekf.X[j] - raw.OBS_rover.SatPVT[iR].Pos[j]) / DistanceR[iSD] -
                    (g_ekf.X[j] - raw.OBS_rover.SatPVT[iRrf].Pos[j]) / DistanceR[refSD];
            }
        }
        // 相位对 N 的偏导：
        const double lam1 = (sys == 0 ? WL1_GPS : WL1_BDS);
        const double lam2 = (sys == 0 ? WL2_GPS : WL3_BDS);
        // 相位两行的模糊度列m：使用统一索引 kTot
        B[(rowBase + 2) * n + (3 + 2 * k + 0)] = lam1;
        B[(rowBase + 3) * n + (3 + 2 * k + 1)] = lam2;

        // 残差（观测-模型）伪距两行、相位两行（相位=λ*(DDL - N) - ρDD；伪距=DDP - ρDD）
        const double ddN1 = g_ekf.X[3 + 2 * k + 0];
        const double ddN2 = g_ekf.X[3 + 2 * k + 1];
        const double rhoDD = (DistanceR[iSD] - DistanceR[refSD]) - (DistanceB[iSD] - DistanceB[refSD]);
        w[rowBase + 0] = (raw.SD_Obs.SdSatObs[iSD].dP[0] - raw.SD_Obs.SdSatObs[refSD].dP[0]) - rhoDD;
        w[rowBase + 1] = (raw.SD_Obs.SdSatObs[iSD].dP[1] - raw.SD_Obs.SdSatObs[refSD].dP[1]) - rhoDD;
        w[rowBase + 2] = (raw.SD_Obs.SdSatObs[iSD].dL[0] - raw.SD_Obs.SdSatObs[refSD].dL[0]) - rhoDD - lam1 * ddN1;
        w[rowBase + 3] = (raw.SD_Obs.SdSatObs[iSD].dL[1] - raw.SD_Obs.SdSatObs[refSD].dL[1]) - rhoDD - lam2 * ddN2;
    }

    // 量测协方差（参考相关块）

    const double sigP = cfg.sigP;
    const double sigL = cfg.sigL;
    int cntGps = 0, cntBds = 0;
    for (int k = 0; k < used; ++k) (sysAB[k] == 0 ? cntGps : cntBds)++;
    memset(R, 0, sizeof(double)* m* m);

    auto fill_R_block = [&](int rowOffset, int cnt) {
        if (cnt <= 0) return;
        const int total = cnt + 1; // 含参考星

        // 权值：对角 (total-1)/(total·2sigma方)，非对角 −1/(total·2sigma方)
        const double wP_off = (2.0 * sigP * sigP);
        const double wL_off = (2.0 * sigL * sigL);
        const double wP_diag = (total - 1)*wP_off;
        const double wL_diag = (total - 1)* wL_off;

        for (int i = 0; i < cnt; ++i) {
            for (int j = 0; j < cnt; ++j) {
                const bool diag = (i == j);
                const double PP = diag ? wP_diag : wP_off;
                const double LL = diag ? wL_diag : wL_off;

                // 同频同型：P1/P2
                R[(rowOffset + 4 * i + 0) * m + (rowOffset + 4 * j + 0)] = PP;
                R[(rowOffset + 4 * i + 1) * m + (rowOffset + 4 * j + 1)] = PP;
                // 相位 L1/L2
                R[(rowOffset + 4 * i + 2) * m + (rowOffset + 4 * j + 2)] = LL;
                R[(rowOffset + 4 * i + 3) * m + (rowOffset + 4 * j + 3)] = LL;
            }
        }
    };
    
    fill_R_block(0, cntGps);
    fill_R_block(4 * cntGps, cntBds);

    // 9) 量测更新
    if (ekf_filter(g_ekf.X, g_ekf.P, B, w, R, n, m) != 0) {
        delete[] DistanceR; delete[] DistanceB;
        delete[] B; delete[] w; delete[] R;
        return rtkpos_lsq(raw, ResB, ResR);
        printf("测量更新失败");
    }

    // 10) 输出与固定解尝试
    ResR.Pos[0] = g_ekf.X[0];
    ResR.Pos[1] = g_ekf.X[1];
    ResR.Pos[2] = g_ekf.X[2];
    raw.DD_Obs.Sats = used;
    raw.DD_Obs.DDSatNum[0] = cntGps;
    raw.DD_Obs.DDSatNum[1] = cntBds;

    //提取定位信息
    double* Xn = new double[n]();
    double* Qn = new double[n * n]();
    for (int i = 0; i < n; ++i) Xn[i] = g_ekf.X[i];

    //vec_print(Xn, n);

    for (int r = 0; r < n; ++r)
        for (int c = 0; c < n; ++c)
            Qn[r * n + c] = g_ekf.P[r * n + c];
    Dmemcpy(Qn, raw.DD_Obs.Q, n * n);
    Dmemcpy(Xn + 3, raw.DD_Obs.floatAmb, 2 * used);

    raw.DD_Obs.bFixed = false;
    const int retFix = rtk_fix_solution(2 * used, Xn, raw.DD_Obs.Q, raw.DD_Obs, &ResB, &ResR);

    delete[] Xn; delete[] Qn;
    delete[] DistanceR; delete[] DistanceB;
    delete[] B; delete[] w; delete[] R;
    if (retFix == 0){
        g_ekf.inited= false;
    }
    return (retFix == 1 && raw.DD_Obs.bFixed) ? 1 : 0;
}

/* 统一入口 ======================== */

// 统一入口：按配置选择 EKF 或 LSQ；必要时回退
int rtkpos(RAWDATA& raw, POSRES& ResB, POSRES& ResR)
{
    if (cfg.rtk_mode == 1) {
        int ret = rtkpos_ekf(raw, ResB, ResR);
        return ret;          // 1=固定,0=浮点

    }
    else {
        int ret = rtkpos_lsq(raw, ResB, ResR);
        return ret;
    }
}
