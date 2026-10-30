#include "rtklib.h"

//TimeConvert
void CommonTime2MJDTime(const COMMONTIME* ct, MJDTIME* mt)
{
    short y, m; y = m = 0;
    if (ct->Month <= 2)
    {
        y = ct->Year - 1;
        m = ct->Month + 12;
    }
    else
    {
        y = ct->Year;
        m = ct->Month;
    }
    mt->Days = int(365.25 * y) + int(30.6001 * (m + 1)) + int(ct->Day) - 679019;
    mt->FracDay = (double(ct->Hour) + double(ct->Minute) / 60 + ct->Second / 3600) / 24;
}
void MJDTime2CommonTime(const MJDTIME* mt, COMMONTIME* ct)
{
    int a, b, c, d, e, Y, M, D, H, Min;
    double JD, S;
    JD = mt->Days + mt->FracDay + 2400000.5;
    a = int(JD + 0.5);
    b = a + 1537;
    c = int((b - 122.1) / 365.25);
    d = int(365.25 * c);
    e = int((b - d) / 30.6001);
    M = e - 1 - 12 * int(e / 14);//月
    Y = c - 4715 - int((7 + M) / 10);//年
    D = b - d - int(30.6001 * e);//日
    H = int(mt->FracDay * 24);//时
    Min = int((mt->FracDay * 24 - H) * 60);//分
    S = double((((mt->FracDay * 24 - H) * 60) - Min) * 60);//秒
    ct->Year = Y;
    ct->Month = M;
    ct->Day = D;
    ct->Hour = H;
    ct->Minute = Min;
    ct->Second = S;
}
void MJDTime2GpsTime(const MJDTIME* mt, GPSTIME* gt)
{
    //GPS时的起点为1980年1月6日0点，其MJD时间表示为44244
    gt->Week = int((mt->Days + mt->FracDay - 44244) / 7);
    gt->SecOfWeek = (mt->Days - 44244 - gt->Week * 7 + mt->FracDay) * 86400.0;
}
void GpsTime2MJDTime(const GPSTIME* gt, MJDTIME* mt)
{
    double MJD;
    MJD = 44244 + gt->Week * 7 + gt->SecOfWeek / 86400;
    mt->Days = int(MJD);
    mt->FracDay = MJD - mt->Days;
}
void CommonTime2GpsTime(const COMMONTIME* ct, GPSTIME* gt)
{
    MJDTIME mt;
    CommonTime2MJDTime(ct, &mt);
    MJDTime2GpsTime(&mt, gt);
}
void GpsTime2CommonTime(const GPSTIME* gt, COMMONTIME* ct)
{
    MJDTIME mt;
    GpsTime2MJDTime(gt, &mt);
    MJDTime2CommonTime(&mt, ct);
}
/* GetDiffTime -------------------------------------------------------------
* execute command line by operating system
* args   :  GPSTIME* gt2    I   gpstime2
*           GPSTIME* gt1    I   gpstime1
* return :  Difference of gpstime1&gpstime2
*-----------------------------------------------------------------------------*/
double GetDiffTime(const GPSTIME* gt1, const GPSTIME* gt2)
{
    int week = gt1->Week - gt2->Week;
    double sec = gt1->SecOfWeek - gt2->SecOfWeek;
    sec += 604800 * week;
    return sec;
}