#include "SPP.h"
#include <iomanip>

//计算信号发射时刻
double CompSatSignalTransTime(double tb, double P, double satclk)
{
	double t = tb - P / C_Light - satclk;
    return t;
}

void CompSatPVTAtSignalTrans(EPOCHOBSDATA* EphObs, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, const Vector3d& receiverPos) {
    //Vector3d receiverPos(-2267335.9727,5008647.9932,3222372.6377);
    for (int i = 0; i < EphObs->SatNum; i++) {
        GPSTime SignalTrans;
        SATPVT Mid;
        bool clkOffValid = false;
        //传进来的EphObs是指针，SatPVT[i].Valid会保存上一步结果
        //因此即使初始化为false也要在这里重置一下false
        EphObs->SatPVT[i].Valid = false;

        SignalTrans.Week = EphObs->Time.Week;
        SignalTrans.SecOfWeek = EphObs->Time.SecOfWeek;

        // 设置初始卫星钟差为0
        double satclk = 0;
        // 计算卫星发射时刻
        SignalTrans.SecOfWeek = CompSatSignalTransTime(EphObs->Time.SecOfWeek, EphObs->SatObs[i].P[0], satclk);

        // 计算卫星钟差
        // 定义一个循环，迭代计算两次，提高计算精度
        for (int iter = 0; iter < 2; iter++) {
            // 调用计算卫星钟差的函数
            if (!CompSatClkOff(EphObs->SatObs[i].Prn, EphObs->SatObs[i].System, &SignalTrans, GPSEph, BDSEph, &Mid)) {
                break; // 如果计算失败，跳出循环
            }
            satclk = Mid.SatClkOft;
            // 根据计算得到的钟差更新时间 t
            SignalTrans.SecOfWeek = CompSatSignalTransTime(EphObs->Time.SecOfWeek, EphObs->SatObs[i].P[0], satclk);
            clkOffValid = true;
        }

        // 只有在clkOffValid为true时才进行后续计算和输出
        if (clkOffValid) {
            // 以信号发射时刻调用卫星位置和钟差函数，计算卫星位置、速度和钟差钟速等
            if (EphObs->SatObs[i].System == GPS) {
                CompGPSSatPVT(EphObs->SatObs[i].Prn, &SignalTrans, GPSEph, &Mid);
            }
            else {
                CompBDSSatPVT(EphObs->SatObs[i].Prn, &SignalTrans, BDSEph, &Mid);
            }

            // 对卫星位置进行地球自转改正
            EarthRotationCorrection(receiverPos, &Mid, EphObs->SatObs[i].System);

            // 更新 SatPVT 数组
            EphObs->SatPVT[i] = Mid;
            EphObs->SatPVT[i].Valid = true;

            // 将 Vector3d 转换为 XYZ 结构体
            XYZ rcv_xyz = { receiverPos(0),receiverPos(1), receiverPos(2) };
            Coord coord = { 6378137.0, 298.257222101 };
            BLH rcv_blh;
            // 调用 XYZ2BLH 函数进行坐标转换
            XYZ2BLH(rcv_xyz, coord, &rcv_blh);
            double Mat[3][3];
            BLHToNEUMat(rcv_blh, Mat);
            XYZ sat_xyz = { EphObs->SatPVT[i].SatPos[0],EphObs->SatPVT[i].SatPos[1] ,EphObs->SatPVT[i].SatPos[2]};
            NEU neu;
            CompEnuPos(sat_xyz, rcv_xyz, Mat, neu);
            CompSatElAz(neu, &EphObs->SatPVT[i].Elevation, &EphObs->SatPVT[i].Azimuth);
            //cout << EphObs->SatPVT[i].Elevation * Deg << endl;
            EphObs->SatPVT[i].TropCorr = hopfield(rcv_blh.H, EphObs->SatPVT[i].Elevation);
            //在这里添加输出语句，输出卫星位置和速度
            //if (EphObs->SatObs[i].System == GPS) {
            //    cout << "卫星系统：GPS" << endl;
            //    cout << "卫星编号: G" << setw(2) << setfill('0') << EphObs->SatObs[i].Prn << endl;
            //    cout << "位置:X=" << fixed << setprecision(3) << Mid.SatPos[0] << " Y=" << fixed << setprecision(3) << Mid.SatPos[1] << " Z=" << fixed << setprecision(3) << Mid.SatPos[2] << endl;
            //    cout << "速度:Vx=" << fixed << setprecision(4) << Mid.SatVel[0] << " Vy=" << fixed << setprecision(4) << Mid.SatVel[1] << " Vz=" << fixed << setprecision(4) << Mid.SatVel[2] << endl;
            //    cout << "时钟偏移: " << scientific << setprecision(6) << Mid.SatClkOft << endl;
            //    cout << "*************************" << endl;
            //}
            //if (EphObs->SatObs[i].System == BDS) {
            //    cout << "卫星系统：BDS" << endl;
            //    cout << "卫星编号: C" << setw(2) << setfill('0') << EphObs->SatObs[i].Prn << endl;
            //    cout << "位置:X=" << fixed << setprecision(3) << Mid.SatPos[0] << " Y=" << fixed << setprecision(3) << Mid.SatPos[1] << " Z=" << fixed << setprecision(3) << Mid.SatPos[2] << endl;
            //    cout << "速度:Vx=" << fixed << setprecision(4) << Mid.SatVel[0] << " Vy=" << fixed << setprecision(4) << Mid.SatVel[1] << " Vz=" << fixed << setprecision(4) << Mid.SatVel[2] << endl;
            //    cout << "时钟偏移: " << scientific << setprecision(6) << Mid.SatClkOft << endl;
            //    cout << "*************************" << endl;
            //}
        }
    }
}

bool SPP(EPOCHOBSDATA* obs, GPSEPHREC* gpseph, GPSEPHREC* bdseph, POSRES* Res)
{
    //设置初始位置
    Vector3d Pos(-2267335.9727, 5008647.9932, 3222372.6377);
    //构建矩阵
    Vector3d d_XYZ;
    //设计矩阵B
    MatrixXd B;
    B.setZero();
    //权阵P，后假设为单位阵
    MatrixXd P;
    //
    VectorXd l;

    VectorXd W(5);
    MatrixXd Nbb(5, 5);
    //后面的Qxx会用
    MatrixXd Nbb_inv;
    //解算xyz+2个钟差的变化量
    VectorXd dx(5);
    //解算xyz+2个钟差的估计值
    VectorXd X(5);
    // 初始化 X 和 dx 为零向量

    X.setZero();
    dx.setZero();

    //n用于区分必要观测数
    double n;
    int gpsnum, bdsnum, satnum;

    int i, j, idx, prn;
    int iter = 0;
    double rou, tgd, trop;
    //卫星钟差和接收机钟差
    double clk, clk_r;
    double PDOP = 0.0;
    GNSSSys sys;

    do
    {
        //将卫星数量重置为0
        gpsnum = bdsnum = satnum = 0;

        // 统计有效卫星数量
        vector<int> validIndices;
        for (i = 0; i < obs->SatNum; i++) {
            if (obs->SatObs[i].Valid && obs->SatPVT[i].Valid) {
                satnum++;
                validIndices.push_back(i);
            }
        }

        // 动态调整 B 和 l 矩阵的大小
        B.conservativeResize(satnum, 5);
        l.conservativeResize(satnum);

        // 观测方程线性化
        for (i = 0, j = 0; i < satnum; i++)
        {
            idx = validIndices[i]; // 使用有效卫星索引

            //确定是哪颗卫星，并累加卫星数量
            prn = obs->SatObs[idx].Prn;
            sys = obs->SatObs[idx].System;

            // 线性化填充矩阵B
            // 获取PIF，ρ，接收机钟差(m)，卫星钟差(s),对流层延迟，tgd,用来构建l
            d_XYZ = Pos - Map<Vector3d>(obs->SatPVT[idx].SatPos);
            rou = d_XYZ.norm();

            B(j, 0) = d_XYZ(0) / rou;
            B(j, 1) = d_XYZ(1) / rou;
            B(j, 2) = d_XYZ(2) / rou;

            //区分GPS和BDS，累加卫星个数
            if (sys == GPS)
            {
                gpsnum++;
                B(j, 3) = 1;
                B(j, 4) = 0;
                tgd = 0;
                //接收机钟差
                clk_r = X(3);
            }
            else if (sys == BDS)
            {
                bdsnum++;
                B(j, 3) = 0;
                B(j, 4) = 1;
                tgd = obs->SatPVT[idx].Tgd1 * fk;
                //接收机钟差
                clk_r = X(4);
            }

            trop = obs->SatPVT[idx].TropCorr;
            clk = obs->SatPVT[idx].SatClkOft;
            l(j) = obs->ComObs[idx].PIF - (rou + clk_r - C_Light * clk + trop + C_Light * tgd);

            j++;
        }

        P = MatrixXd::Identity(satnum, satnum);

        // 卫星数量判断
        if (!(gpsnum == 0 && bdsnum >= 4 || bdsnum == 0 && gpsnum >= 4 || satnum >= 5))
        {
            return false;
        }
        //如果GPS或BDS卫星数量为0，重构法方程矩阵N和Y
        bool reconstruct = (gpsnum == 0 && bdsnum >= 4 || bdsnum == 0 && gpsnum >= 4);
        bool nogps = (gpsnum == 0);

        // 计算 Nbb 和 W
        //拆开计算，后面Nbb有用
        Nbb = B.transpose() * P * B;
        //cout << Nbb << endl;
        Nbb_inv = Nbb.inverse();
        //cout << Nbb_inv << endl;
        W = B.transpose() * P * l;

        MatrixXd Nbb_temp;
        VectorXd W_temp;

        if (!reconstruct)
        {
            // 如果不需要重构法方程矩阵
            Nbb_temp = Nbb;
            W_temp = W;
            n = 5;
        }
        else
        {
            n = 4;
            // 如果需要重构法方程矩阵
            if (nogps)
            {
                // 删除第4行和第4列（索引从0开始，对应的是第4个元素）
                Nbb_temp.resize(4, 4);
                W_temp.resize(4);

                // 复制前3行和前3列
                Nbb_temp.block(0, 0, 3, 3) = Nbb.block(0, 0, 3, 3);

                // 复制第5行和第5列（索引4）到新矩阵的第4行和第4列（索引3）
                Nbb_temp.block(3, 3, 1, 1) = Nbb.block(4, 4, 1, 1);

                // 复制第5行的前3列到新矩阵的第4行
                Nbb_temp.row(3).head(3) = Nbb.row(4).head(3);

                // 复制前3行的第5列到新矩阵的第4列
                Nbb_temp.col(3).head(3) = Nbb.col(4).head(3);

                // 复制W向量的前3个元素和第5个元素
                W_temp.head(3) = W.head(3);
                W_temp(3) = W(4);
            }
            else
            {
                // 删除第5行和第5列（索引从0开始，对应的是第5个元素）
                Nbb_temp.resize(4, 4);
                W_temp.resize(4);

                // 复制前4行和前4列
                Nbb_temp = Nbb.topLeftCorner(4, 4);

                // 复制W向量的前4个元素
                W_temp = W.head(4);
            }
        }

        // 最小二乘求解
        MatrixXd Nbb_inv_temp = Nbb_temp.inverse();
        Nbb_inv = Nbb_inv_temp;
        VectorXd x_temp = Nbb_inv_temp * W_temp;
        dx.setZero();

        if (reconstruct)
        {
            dx.head(3) = x_temp.head(3);  // 更新位置增量
            if (nogps)
            {
                dx(4) = x_temp(3);
            }
            else
            {
                dx(3) = x_temp(3);
            }
        }
        else
        {
            dx = x_temp;
        }

        // 位置更新
        X += dx;
        Pos += dx.head<3>();
        iter++;

        // 计算信号发射时刻的卫星位置和钟差计算,地球自转改正和对流层延迟𝑇
        CompSatPVTAtSignalTrans(obs, gpseph, bdseph, Pos);
    } while (dx.head<3>().norm() > 1e-6 || iter < 10);

    for (int i = 0; i < 3; ++i) {
        Res->Pos[i] = Pos(i);
    }
    Res->RcvClkOft[0] = X(3);
    Res->RcvClkOft[1] = X(4);

    //cout << Nbb_inv << endl;
    //计算单位权中误差
    for (i = 0; i < 3; i++)
    {
        PDOP += Nbb_inv(i, i);
    }

    VectorXd v(satnum);
    //for (int i = 0; i < satnum; ++i)
    //{
    //    v(i) = B(i, 0) * X(0) + B(i, 1) * X(1) + B(i, 2) * X(2) + B(i, 3) * X(3) + B(i, 4) * X(4) - l(i);
    //}
    //cout << X << endl;
    v = B * dx - l;
    //cout << v << endl;
    double vtpv = v.transpose() * P * v;
    Res->SigmaPos = sqrt(vtpv / (satnum - n));
    Res->Time = obs->Time;
    Res->SatNum = satnum;
    Res->GPSSatNum = gpsnum;
    Res->BDSSatNum = bdsnum;
    Res->PDOP = sqrt(PDOP);

    return true;
}

bool SPV(EPOCHOBSDATA& obs, POSRES& Res)
{
    // 定义矩阵和向量
    MatrixXd B; // 4 x satnum
    MatrixXd P; // satnum x satnum
    VectorXd l; // satnum x 1
    VectorXd v; // satnum x 1
    MatrixXd Nbb; // 4 x 4
    MatrixXd Nbb_inv; // 4 x 4
    VectorXd X; // 4 x 1
    VectorXd d_x(3); // 3 x 1
    VectorXd ResVel(3); // 用于存储速度结果

    // 找出所有有效卫星的索引
    vector<int> validIndices;
    for (int i = 0; i < obs.SatNum; i++)
    {
        if (obs.SatObs[i].Valid && obs.SatPVT[i].Valid)
        {
            validIndices.push_back(i);
        }
    }

    int satnum = validIndices.size();

    // 如果有效卫星数量不足4颗，则无法进行定位
    if (satnum < 4)
    {
        return false;
    }

    // 初始化矩阵和向量
    B.resize(satnum,4);
    l.resize(satnum);
    P.setIdentity(satnum, satnum);

    // 填充B矩阵和w向量
    for (int j = 0; j < satnum; j++)
    {
        int i = validIndices[j];

        // 线性化方程
        d_x << obs.SatPVT[i].SatPos[0] - Res.Pos[0],
            obs.SatPVT[i].SatPos[1] - Res.Pos[1],
            obs.SatPVT[i].SatPos[2] - Res.Pos[2];
        double rou = d_x.norm();

        VectorXd satVel(3);
        satVel << obs.SatPVT[i].SatVel[0],
            obs.SatPVT[i].SatVel[1],
            obs.SatPVT[i].SatVel[2];

        // 计算 roudot
        double roudot = d_x.dot(satVel) / rou;

        // 填充B矩阵
        //B.row(j) << d_x / rou, 1.0;
                // 填充B矩阵
        B.block<1, 3>(j, 0) = d_x / rou;
        B(j, 3) = 1.0;

        // 填充w矩阵
        l(j) = obs.SatObs[i].D[0] - (roudot - C_Light * obs.SatPVT[i].SatClkSft);
    }
    //cout << B << endl;
    //cout << w << endl;
    // 计算Nbb = B' * P * B
    Nbb = B.transpose() * P * B;

    // 计算Nbb的逆矩阵
    Nbb_inv = Nbb.inverse();

    // 计算W = B' * P * w
    VectorXd W = B.transpose() * P * l;

    // 计算X = Nbb_inv * W
    X = Nbb_inv * W;
  /*  cout << X << endl;*/
    // 提取速度分量
    ResVel << X(0), X(1), X(2);
    Res.Vel[0] = ResVel(0);
    Res.Vel[1] = ResVel(1);
    Res.Vel[2] = ResVel(2);

    // 计算SIGMA_VEL
    v = B * X - l;

    double vtpv = v.transpose() * P * v;
    Res.SigmaVel = sqrt(vtpv / (satnum - 4));

    return true;
}

void CompNEU(POSRES& Res) {
    XYZ Cal = { Res.Pos[0],Res.Pos[1], Res.Pos[2]};
    XYZ Ref = { Res.RefXYZ[0],Res.RefXYZ[1], Res.RefXYZ[2]};
    Coord coord = { 6378137.0, 298.257223563 };
    BLH blh;
    NEU neu;
    double R[3][3];
    XYZ2BLH(Cal, coord, &blh);
    BLHToNEUMat(blh, R);
    CompEnuPos(Cal, Ref, R, neu);

    Res.dNEU[0] = neu.dN;
    Res.dNEU[1] = neu.dE;
    Res.dNEU[2] = neu.dH;

    Res.dXYZ[0] = Res.Pos[0] - Res.RefXYZ[0];
    Res.dXYZ[1] = Res.Pos[1] - Res.RefXYZ[1];    
    Res.dXYZ[2] = Res.Pos[2] - Res.RefXYZ[2];
}

// 从文件读一条完整历元，返回已用字节数；返回 0 表示成功解出一帧，返回 -1 表示 EOF 或错误
int ReadOneEpoch(FILE* fp,unsigned char* buff,int& lenD,EPOCHOBSDATA* obs,GPSEPHREC* gpsEph,GPSEPHREC* bdEph,POSRES* pos)
{
    while (true) {
        int remaining = MAXRAWLEN - lenD;
        if (remaining == 0) return -1;          // 缓冲区满

        size_t lenR = fread(buff + lenD, 1, remaining, fp);
        if (lenR == 0) {
            if (feof(fp)) return -1;            // 正常读完
            if (ferror(fp)) return -1;          // 读取出错
        }
        lenD += lenR;

        int used = DecodeNovOem7Dat(buff, lenD, obs, gpsEph, bdEph, pos);
        if (used == 0) {                        // 成功解出一帧
            memmove(buff, buff + used, lenD - used);
            lenD -= used;
            return 0;
        }
        if (lenD == MAXRAWLEN) return -1;       // 单帧太大
    }
}

int TimeSynchronization(FILE* FObs_base, FILE* FObs_rover, RAWDATA* base, RAWDATA* rover)
{
    static unsigned char buff_base[MAXRAWLEN], buff_rover[MAXRAWLEN];
    static int lenD_base = 0, lenD_rover = 0;
    static int base_initialized = 0;

    // 1. 读一条流动站历元
    if (ReadOneEpoch(FObs_rover, buff_rover, lenD_rover,
        &rover->Obs,
        rover->GpsEph,
        rover->BdsEph,
        &rover->Pos) != 0)
        return -1;

    // 2. 若基站未初始化，先读一条
    if (!base_initialized) {
        if (ReadOneEpoch(FObs_base, buff_base, lenD_base,
            &base->Obs,
            base->GpsEph,
            base->BdsEph,
            &base->Pos) != 0)
            return -1;
        base_initialized = 1;
    }

    // 3. 时间差计算
    double dt = (rover->Obs.Time.Week - base->Obs.Time.Week) * 604800.0+ (rover->Obs.Time.SecOfWeek - base->Obs.Time.SecOfWeek);
    if (fabs(dt) < 0.01) return 1;
    if (dt < -0.5) return 0;   // rover 太早，放弃

    // 4. 基站继续读直到时间匹配
    while (true) {
        if (ReadOneEpoch(FObs_base, buff_base, lenD_base,
            &base->Obs,
            base->GpsEph,
            base->BdsEph,
            &base->Pos) != 0)
            return -1;

        double dt = (rover->Obs.Time.Week - base->Obs.Time.Week) * 604800.0 + (rover->Obs.Time.SecOfWeek - base->Obs.Time.SecOfWeek);
        if (fabs(dt) < 0.01) return 1;
        if (dt < -0.5) return 0;
    }
}
void SPP_Plus(EPOCHOBSDATA* obs,GPSEPHREC* gpseph,GPSEPHREC* bdseph,POSRES* res,const Vector3d& initPos)
{
    CompSatPVTAtSignalTrans(obs, gpseph, bdseph, initPos);
    DetectOutlier(obs);
    if (!SPP(obs, gpseph, bdseph, res)) {
        cerr << "SPP 解算失败" << endl;
    }
    if (!SPV(*obs, *res)) {
        cerr << "SPV 解算失败" << endl;
    }
    CompNEU(*res);
}