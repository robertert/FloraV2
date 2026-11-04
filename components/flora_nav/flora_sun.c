#include <math.h>

#include "flora_sun.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define RAD(d) ((d) * M_PI / 180.0)
#define DEG(r) ((r) * 180.0 / M_PI)

flora_sun_pos_t flora_sun_position(int64_t unix_s, double lat_deg, double lon_deg)
{
    /* Wiek julianski od J2000. */
    double jd = (double)unix_s / 86400.0 + 2440587.5;
    double jc = (jd - 2451545.0) / 36525.0;

    double L = fmod(280.46646 + jc * (36000.76983 + jc * 0.0003032), 360.0);  /* sr. dlugosc */
    double M = 357.52911 + jc * (35999.05029 - 0.0001537 * jc);                /* sr. anomalia */
    double e = 0.016708634 - jc * (0.000042037 + 0.0000001267 * jc);           /* mimosrod */

    double C = sin(RAD(M)) * (1.914602 - jc * (0.004817 + 0.000014 * jc))
             + sin(RAD(2 * M)) * (0.019993 - 0.000101 * jc)
             + sin(RAD(3 * M)) * 0.000289;
    double omega    = 125.04 - 1934.136 * jc;
    double app_long = L + C - 0.00569 - 0.00478 * sin(RAD(omega));

    double obliq0 = 23.0 + (26.0 + (21.448 - jc * (46.815 + jc * (0.00059 - jc * 0.001813))) / 60.0) / 60.0;
    double obliq  = obliq0 + 0.00256 * cos(RAD(omega));

    double decl = asin(sin(RAD(obliq)) * sin(RAD(app_long)));

    double vy = tan(RAD(obliq / 2.0));
    vy *= vy;
    double eq_time = 4.0 * DEG(vy * sin(2 * RAD(L))
                               - 2 * e * sin(RAD(M))
                               + 4 * e * vy * sin(RAD(M)) * cos(2 * RAD(L))
                               - 0.5 * vy * vy * sin(4 * RAD(L))
                               - 1.25 * e * e * sin(2 * RAD(M)));   /* minuty */

    double minutes_utc = fmod((double)unix_s, 86400.0) / 60.0;
    if (minutes_utc < 0) minutes_utc += 1440.0;
    double tst = fmod(minutes_utc + eq_time + 4.0 * lon_deg, 1440.0);
    if (tst < 0) tst += 1440.0;
    double ha = tst / 4.0 < 0 ? tst / 4.0 + 180.0 : tst / 4.0 - 180.0;   /* kat godzinny */

    double lat = RAD(lat_deg);
    double cos_zen = sin(lat) * sin(decl) + cos(lat) * cos(decl) * cos(RAD(ha));
    if (cos_zen > 1.0) cos_zen = 1.0;
    if (cos_zen < -1.0) cos_zen = -1.0;
    double zen = acos(cos_zen);

    double az;
    double den = cos(lat) * sin(zen);
    if (fabs(den) < 1e-9) {
        az = 180.0;   /* slonce w zenicie albo biegun - azymut nieokreslony */
    } else {
        double a = (sin(lat) * cos(zen) - sin(decl)) / den;
        if (a > 1.0) a = 1.0;
        if (a < -1.0) a = -1.0;
        a = DEG(acos(a));
        az = ha > 0 ? fmod(a + 180.0, 360.0) : fmod(540.0 - a, 360.0);
    }

    flora_sun_pos_t out = { .azimuth_deg = az, .elevation_deg = 90.0 - DEG(zen) };
    return out;
}
