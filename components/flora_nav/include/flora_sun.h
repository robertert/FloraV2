/**
 * @file flora_sun.h
 * @brief Pozycja slonca na niebie (algorytm NOAA) do kompasu slonecznego.
 *
 * Kompas sloneczny mierzy azymut slonca WZGLEDEM GLOWICY. Zeby dostac z
 * tego kurs, trzeba znac azymut slonca WZGLEDEM POLNOCY w tej chwili:
 *
 *     kurs_geograficzny = azymut_z_obliczen - azymut_zmierzony
 *
 * Algorytm NOAA (wzory z arkusza NOAA Solar Calculator) daje ok. 0,01 st
 * w latach 1800-2100 - z ogromnym zapasem wobec szumu krzyza VEML (~10 st).
 * Bez refrakcji: przy elewacji ponizej ok. 10 st i tak nie uzywamy pomiaru.
 *
 * Liczone w double: dzien julianski ma ~7 cyfr przed przecinkiem, float
 * zgubilby minuty. Na S3 double jest programowy, ale to wywolanie raz na
 * kilka sekund.
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double azimuth_deg;    /**< CW od polnocy geograficznej, 0..360 */
    double elevation_deg;  /**< nad horyzontem, ujemna w nocy */
} flora_sun_pos_t;

/**
 * @param unix_s   czas UTC w sekundach od 1970
 * @param lat_deg  szerokosc geograficzna, polnoc dodatnia
 * @param lon_deg  dlugosc geograficzna, wschod dodatni
 */
flora_sun_pos_t flora_sun_position(int64_t unix_s, double lat_deg, double lon_deg);

#ifdef __cplusplus
}
#endif
