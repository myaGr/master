#ifndef GNSS_TDCP_H
#define GNSS_TDCP_H
#include "asg_rtcm_decode.h"
typedef struct
{
    unsigned char sat; /* satellite*/
    unsigned char chann_num;
    unsigned char SNR[NFREQ + NEXOBS]; /* signal strength (0.25 dBHz) */
    unsigned char LLI[NFREQ + NEXOBS]; /* loss of lock indicator */
    unsigned char code[NFREQ + NEXOBS]; /* code indicator (CODE_???) */
    double L[NFREQ + NEXOBS]; /* observation data carrier-phase (cycle) */
    double P[NFREQ + NEXOBS]; /* observation data pseudorange (m) */
    float  D[NFREQ + NEXOBS]; /* observation data doppler frequency (Hz) */
    double sat_pos_clk[4];/* sat_pos_clk[0~2] represent the satellite position and sat_pos_clk[3] represent satellite clock*/
    double sat_v[4];/* sat_v[0~2] represent the satellite velocity and sat_v[3] represent satellite clock drift*/
    float ele;
    float azimuth;
} asg_sat_tdcp;

typedef struct
{
    asg_gtime_t obs_time;
    uint8_t fix_quality;
    double site_blh[3];
    double site_v_enu[3];
    int sat_num;
    asg_sat_tdcp sat_data[MAXOBS];
}asg_epoch_data_tdcp;

typedef struct
{
    uint64_t utc_timestamp; /* ms */
    uint16_t utc_year;
    uint16_t utc_month;
    uint16_t utc_day;
    uint16_t utc_hour;
    uint16_t utc_minute;
    double   utc_second;

    uint16_t week; /* GPS week number */
    double   tow; /* time of week */
    uint8_t  leapsec; /* leap second */

    uint8_t fix_quality; /* 0/1/2/4/5 */

    double lon; /* Longitude (deg) */
    double lat; /* Latitude (deg) */
    float alt;

    float lonstd; /* unit: m */
    float latstd; /* unit: m */
    float altstd; /* unit: m */

    float vel_n; /* NED north velocity (m/s) */
    float vel_e; /* NED east velocity (m/s) */
    float vel_d; /* NED down velocity (m/s) */

    float vel_n_std; /* unit: m/s */
    float vel_e_std; /* unit: m/s */
    float vel_d_std; /* unit: m/s */

    float age; /* diff age (s) */

    uint16_t sv_used;
    uint16_t sv_trked;
    uint16_t sv_fixed;

    float gdop;
    float pdop;
    float hdop;
    float vdop;
    float tdop;

    char nmea_rmc[256];
    char nmea_gga[256];
    char nmea_gsv_num;
    char nmea_gsv[24][256];
    char nmea_gst[256];
    double quasi_geoid_h;//add new field to represent quasi-geoid height
    float avg_CN0;//add new field to represent average CN0
    uint8_t CN040;
} asensing_tdcp_pos_t;

void updateCurEpochData(const asg_obs_t* cur_obs);
void pushBaseTDCPdata(const asg_epoch_data_tdcp* base_tdcp_data, const asensing_tdcp_pos_t* tdcp_pos);
int getRealTimePos(asensing_tdcp_pos_t* rover_tdcp_pos);
#endif