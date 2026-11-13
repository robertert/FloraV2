/**
 * @file flora_navlog.h
 * @brief Format nagrania przejazdu: wejscia flora_navcore_step, linia po linii.
 *
 * Bryla (hub_rec) zapisuje, host/replay.c czyta - formaty tylko tutaj,
 * zeby nie mogly sie rozjechac. Tekst, przecinki, jedna linia = jeden rekord.
 * Linie A, S, G naleza do taktu zamknietego NASTEPNA linia E.
 *
 *   V,<wersja>                                  naglowek
 *   P,<mm_l>,<mm_r>,<track>,<cam_x>,<cam_y>,<lat>,<lon>,<decl>
 *                                               parametry (start i kazda zmiana)
 *   C                                           mapa palikow: wyczysc
 *   M,<id>,<x_mm>,<y_mm>                        mapa palikow: palik
 *   R,<x_mm>,<y_mm>,<th_deg>                    cmd/pose_reset (przed nastepnym E)
 *   A,<age_us>,<kurs_cdeg>,<tempo_cdps>,<flagi>,<sigma_cdeg>,<quality>
 *                                               ATTITUDE; flagi: 1 kurs wazny, 2 magnetometr
 *   S,<unix_s>,<azymut_cdeg>,<lux N>,<lux E>,<lux S>,<lux W>,<valid>
 *   G,<n>[,<id>,<odl_mm>,<azymut_cdeg>,<quality>]*n
 *   E,<t_us>,<zliczenia L>,<zliczenia P>        enkodery; zamyka takt
 *
 * W MQTT (rec/data) kazda wiadomosc zaczyna sie linia "#<numer>" - luka
 * w numeracji to zgubiona wiadomosc. Liczby calkowite tam, gdzie robot ma
 * calkowite (zliczenia, centystopnie), zeby odtworzenie bylo bit w bit.
 */
#pragma once

#define FLORA_NAVLOG_VERSION 1

#define FLORA_NAVLOG_ATT_HEADING_VALID 1
#define FLORA_NAVLOG_ATT_MAG_VALID     2

#define FLORA_NAVLOG_FMT_V "V,%d\n"
#define FLORA_NAVLOG_FMT_P "P,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n"
#define FLORA_NAVLOG_FMT_C "C\n"
#define FLORA_NAVLOG_FMT_M "M,%u,%.9g,%.9g\n"
#define FLORA_NAVLOG_FMT_R "R,%.9g,%.9g,%.9g\n"
#define FLORA_NAVLOG_FMT_A "A,%ld,%u,%d,%u,%u,%u\n"
#define FLORA_NAVLOG_FMT_S "S,%lld,%u,%.9g,%.9g,%.9g,%.9g,%u\n"
#define FLORA_NAVLOG_FMT_E "E,%lld,%lld,%lld\n"
/* G ma zmienna dlugosc: "G,<n>" i dla kazdego ",<id>,<odl>,<azymut>,<q>". */
#define FLORA_NAVLOG_FMT_G_HEAD "G,%u"
#define FLORA_NAVLOG_FMT_G_ITEM ",%u,%u,%d,%u"
