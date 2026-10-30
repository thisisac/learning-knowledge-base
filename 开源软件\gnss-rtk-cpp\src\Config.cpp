#include "rtklib.h"
#include <cctype>

RTKConfig cfg; // 全局实例

void rtkplotHEAD(FILE* fp) {
    if (!fp) return;
    fprintf(fp,
        "%% rtkplot compatible solution file\n"
        "%% columns: time(UTC) lat(deg) lon(deg) h(m) Q ns sdn(m) sde(m) sdu(m) sdne(m) sdeu(m) sdun(m) age(s) ratio baseline(m)\n"
        "%% Q:1=FIX,2=FLOAT\n");
}

static void GPST2UTC(const GPSTIME& gt, char* buf, size_t sz) {
    // 这里简单用 GPS 周与周内秒直接输出，
    // 输出格式：Week,SOW 或 YYYY/MM/DD hh:mm:ss.sss
    COMMONTIME ct;
    GpsTime2CommonTime(&gt, &ct);
    int ms = (int)floor((gt.SecOfWeek - floor(gt.SecOfWeek)) * 1000.0 + 0.5);
    double secs = floor(gt.SecOfWeek) - (ct.Hour * 3600 + ct.Minute * 60) + ct.Second; // ct.Second 已是 double
    // 格式化
    snprintf(buf, sz, "%04d/%02d/%02d %02d:%02d:%02d.%03d",
        ct.Year, ct.Month, ct.Day, ct.Hour, ct.Minute,
        (int)floor(ct.Second), ms);
}


static void Ecef2ENU(const double Cov[9], const double xyz[3], double enuCov[9]) {
    // 若协方差全零，直接返回0
    bool Erro = true;
    for (int i = 0; i < 9; ++i) if (fabs(Cov[i]) > 0.0) { Erro = false; break; }
    if (Erro) {
        for (int i = 0; i < 9; ++i) enuCov[i] = 0.0;
        return;
    }

    BLHCOOR blh;
    XYZCOOR xyzc(xyz[0], xyz[1], xyz[2]);
    XYZ2BLH(xyzc, blh, WGS84); // 得到 (b,l,h) 弧度

    const double phi = blh.BLH.b; // 纬度(rad
    const double lam = blh.BLH.l; // 经度(rad

    // ECEF->ENU 旋转矩阵 R (行：N,E,U；列：X,Y,Z)
    double R[9];
    const double sinp = sin(phi), cosp = cos(phi), sinl = sin(lam), cosl = cos(lam);
    R[0] = -sinl;         R[1] = cosl;        R[2] = 0.0;
    R[3] = -sinp * cosl;    R[4] = -sinp * sinl;   R[5] = cosp;
    R[6] = cosp * cosl;    R[7] = cosp * sinl;   R[8] = sinp;

    // enuCov = R * Cov * R^T
    double RC[9]{ 0 }, Rt[9]{ 0 };
    // RC = R * Cov
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            double sum = 0;
            for (int k = 0; k < 3; ++k) sum += R[r * 3 + k] * Cov[k * 3 + c];
            RC[r * 3 + c] = sum;
        }
    }
    // R^T
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            Rt[r * 3 + c] = R[c * 3 + r];
    // enuCov
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            double sum = 0;
            for (int k = 0; k < 3; ++k) sum += RC[r * 3 + k] * Rt[k * 3 + c];
            enuCov[r * 3 + c] = sum;
        }
    }
}


void write2rtkplot(FILE* fp, const RAWDATA& raw, const POSRES& ResB, const POSRES& ResR) {
    if (!fp) return;

    // 质量标志
    int Q = raw.DD_Obs.bFixed ? 1 : 2;

    // 卫星数（使用的双差非参考星 + 参考星存在的个数）
    int ns = raw.DD_Obs.Sats;
    if (raw.DD_Obs.RefPos[0] >= 0) ns++;
    if (raw.DD_Obs.RefPos[1] >= 0) ns++;

    // 位置(ECEF -> LLH)
    XYZCOOR xyzR(ResR.Pos[0], ResR.Pos[1], ResR.Pos[2]);
    BLHCOOR blhR;
    XYZ2BLH(xyzR, blhR, WGS84);  // blhR.BLH.b,l 弧度

    double latDeg = blhR.BLH.b * Deg;
    double lonDeg = blhR.BLH.l * Deg;
    double hEll = blhR.BLH.h; // 椭球高

    // 协方差（ECEF）来自 RTKRES.dxyz3x3
    double CovXYZ[9];
    for (int i = 0; i < 9; ++i) CovXYZ[i] = RTKRES.dxyz[i];

    double CovENU[9];
    Ecef2ENU(CovXYZ, ResR.Pos, CovENU);

    double sdn = (CovENU[0 * 3 + 0] > 0) ? sqrt(CovENU[0 * 3 + 0]) : 0.0;
    double sde = (CovENU[1 * 3 + 1] > 0) ? sqrt(CovENU[1 * 3 + 1]) : 0.0;
    double sdu = (CovENU[2 * 3 + 2] > 0) ? sqrt(CovENU[2 * 3 + 2]) : 0.0;
    double sdne = CovENU[0 * 3 + 1];
    double sdeu = CovENU[1 * 3 + 2];
    double sdun = CovENU[2 * 3 + 0];

    // age（差分龄期）：暂置 0
    double age = 0.0;

    // ratio
    double ratio = raw.DD_Obs.Ratio;

    // 基线长度
    double blen = sqrt(raw.DD_Obs.dPos[0] * raw.DD_Obs.dPos[0] +
        raw.DD_Obs.dPos[1] * raw.DD_Obs.dPos[1] +
        raw.DD_Obs.dPos[2] * raw.DD_Obs.dPos[2]);

    // 时间
    char tbuf[64];
    GPST2UTC(raw.OBS_rover.time, tbuf, sizeof(tbuf));

    fprintf(fp,
        "%s %14.9f %14.9f %10.4f %d %2d %7.4f %7.4f %7.4f %9.5f %9.5f %9.5f %4.1f %6.2f %9.3f\n",
        tbuf, latDeg, lonDeg, hEll, Q, ns,
        sdn, sde, sdu,
        sdne, sdeu, sdun,
        age, ratio, blen);
}


static inline void trim(std::string& s) {
    size_t b = 0, e = s.size();
    while (b < e && std::isspace((unsigned char)s[b])) ++b;
    while (e > b && std::isspace((unsigned char)s[e - 1])) --e;
    s = s.substr(b, e - b);
}

bool load_rtk_config(const string& path) {
    FILE* fp = nullptr;
    if (fopen_s(&fp, path.c_str(), "r") != 0 || !fp) {
        // 未找到文件，保留默认值
        return false;
    }
    char line[512];
    while (fgets(line, sizeof(line), fp)) {
        string s(line);
        trim(s);
        if (s.empty() || s[0] == '#' || s[0] == ';') continue;
        auto pos = s.find('=');
        if (pos ==string::npos) continue;
        string key = s.substr(0, pos);
        string val = s.substr(pos + 1);
        trim(key); trim(val);
        if (val.empty()) continue;

        // 映射到结构体
        if (key == "sigP")            cfg.sigP = atof(val.c_str());
        else if (key == "sigL")            cfg.sigL = atof(val.c_str());
        else if (key == "pos_Q")           cfg.pos_Q = atof(val.c_str());
        else if (key == "amb_Q")           cfg.amb_Q = atof(val.c_str());
        else if (key == "base_file")       cfg.base_file = val;
        else if (key == "rover_file")      cfg.rover_file = val;
        else if (key == "output_file")     cfg.output_file = val;
        else if (key == "isPort")          cfg.isPort = atoi(val.c_str());
        else if (key == "rtk_mode")        cfg.rtk_mode = atoi(val.c_str());
        else if (key == "syn_tol")         cfg.syn_tol = atof(val.c_str());
        else if (key == "ratio_threshold") cfg.ratio_threshold = atof(val.c_str());
        else if (key == "elev_threshold")  cfg.elev_threshold = atof(val.c_str());
        // 未知键忽略
    }
    fclose(fp);
    if (cfg.sigP <= 0) cfg.sigP = 0.3;
    if (cfg.sigL <= 0) cfg.sigL = 0.01;
    if (cfg.pos_Q < 0) cfg.pos_Q = 25.0;
    if (cfg.amb_Q < 0) cfg.amb_Q = 1e-10;
    if (cfg.syn_tol <= 0) cfg.syn_tol = 0.10;
    if (cfg.elev_threshold < 0) cfg.elev_threshold = 0.0;
    if (cfg.elev_threshold > 85.0) cfg.elev_threshold = 85.0;
    printf("[cfg] 加载完成:\n");
    printf("  sigP=%.3f sigL=%.3f pos_Q=%.3f amb_Q=%g\n", cfg.sigP, cfg.sigL, cfg.pos_Q, cfg.amb_Q);
    printf("  base_file=%s\n", cfg.base_file.c_str());
    printf("  rover_file=%s\n", cfg.rover_file.c_str());
    printf("  output_file=%s\n", cfg.output_file.c_str());
    printf("  isPort=%d rtk_mode=%d syn_tol=%.3f ratio_threshold=%.3f elev_threshold=%.3f\n",
        cfg.isPort, cfg.rtk_mode, cfg.syn_tol, cfg.ratio_threshold, cfg.elev_threshold);

    return true;
}