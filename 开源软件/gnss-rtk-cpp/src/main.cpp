#include <iostream>
#include "rtklib.h"

RESofRTK RTKRES;
EKFState g_ekf;
int main()
{
    // 读取配置（不存在则使用默认值）
    load_rtk_config("rtk.conf");

    const char* basePath =cfg.base_file.c_str();
    const char* rovPath =cfg.rover_file.c_str();
    const int isPort = cfg.isPort;

    FILE* fBase = nullptr;
    FILE* fRov = nullptr;
    // 仅文件模式需要打开观测文件
    if (isPort != 2) {
        if (fopen_s(&fBase, basePath, "rb") != 0 || fopen_s(&fRov, rovPath, "rb") != 0 || !fBase || !fRov) {
            cerr << "无法打开观测文件:\n  base=" << basePath << "\n  rover=" << rovPath << endl;
            if (fBase) fclose(fBase);
            if (fRov)  fclose(fRov);
            return -1;
        }
    }

    FILE* fout = nullptr;
    if (fopen_s(&fout, cfg.output_file.c_str(), "w") != 0 || !fout) {
        cerr << "无法打开输出文件 " << cfg.output_file << endl;
        if (fBase) fclose(fBase);
        if (fRov)  fclose(fRov);
        return -1;
    }

    // 端口模式下打开 socket
    SOCKET sBase, sRov;
    if (isPort == 2) {

        if (!OpenSocket(sRov, "8.148.22.229", 7002) ||

            //!OpenSocket(sBase, "8.148.22.229", 7002)) {
            !OpenSocket(sBase, "47.114.134.129", 7190)) {
            cerr << "Socket连接失败\n";
            return -1;
        }

        FILE* fout = nullptr;
        if (fopen_s(&fout, "rtk_socket_result.txt", "w") != 0 || !fout) {
            cerr << "无法打开输出文件\n";
            CloseSocket(sBase); CloseSocket(sRov);
            return -1;
        }

    }

    // 写 rtkplot 头
    rtkplotHEAD(fout);

    RAWDATA raw{};
    POSRES ResB{}, ResR{};
    printf("Week  SOW         GPS(NR) BDS(NR) Ref(GPS,BDS)   Fixed  Ratio    Base_X            Base_Y            Base_Z            Rover_X           Rover_Y           Rover_Z           dX               dY               dZ            |b|\n");
    printf("------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");


    while (true) {
        int ret;
        if (isPort == 2)
        {
            Sleep(999);
            int sync = SynRTKObs_Sock(sBase, sRov, &raw);
            if (sync == -1) {
                cout << "连接断开，退出\n";
                break;
            }
            if (sync != 1) continue; // 未同步继续循环

            ret = rtkpos(raw, ResB, ResR);
            if (ret < 0) continue;

        }
        if (isPort != 2)
        {
            
            int sync = SynRTKObs(fBase, fRov, &raw);
            if (sync != 1) {

                if (sync == -1 || (isPort != 2 && (feof(fBase) || feof(fRov)))) {

                    cout << "处理结束\n";
                    break;
                }
                continue;
            }
            ret = rtkpos(raw, ResB, ResR);// 1=固定，0=浮点，-1失
            if (ret < 0) continue;

        }

        write2rtkplot(fout, raw, ResB, ResR);

        const int refGpsIdx = raw.DD_Obs.RefPos[0];
        const int refBdsIdx = raw.DD_Obs.RefPos[1];
        const int refGpsPrn = (refGpsIdx >= 0 ? raw.SD_Obs.SdSatObs[refGpsIdx].Prn : 0);
        const int refBdsPrn = (refBdsIdx >= 0 ? raw.SD_Obs.SdSatObs[refBdsIdx].Prn : 0);
        const char* mode = (ret == 1 && raw.DD_Obs.bFixed) ? "FIX" : "FLT";

        printf("%4d %10.3lf   %3d     %3d      (%02d,%02d)     %s  %6.2f   %15.4lf %15.4lf %15.4lf %15.4lf %15.4lf %15.4lf %15.4lf %15.4lf %15.4lf %12.4lf\n",
            raw.OBS_base.time.Week, raw.OBS_base.time.SecOfWeek,
            raw.DD_Obs.DDSatNum[0], raw.DD_Obs.DDSatNum[1],
            refGpsPrn, refBdsPrn, mode, raw.DD_Obs.Ratio,
            ResB.Pos[0], ResB.Pos[1], ResB.Pos[2], ResR.Pos[0], ResR.Pos[1], ResR.Pos[2],
            raw.DD_Obs.dPos[0], raw.DD_Obs.dPos[1], raw.DD_Obs.dPos[2], RTKRES.sigma_pos);
    }

    if (fBase) fclose(fBase);
    if (fRov)  fclose(fRov);
    fclose(fout);
    if (isPort == 2) {
        CloseSocket(sBase);
        CloseSocket(sRov);
    }
    return 0;
}