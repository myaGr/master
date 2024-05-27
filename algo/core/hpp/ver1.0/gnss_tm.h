#ifndef __GNSS__TM__H__
#define __GNSS__TM__H__

#include "gnss_types.h"
#include "gnss.h"

#define MAX_CLOCK_FREQ_DRIFT       (float)(750*GPS_L1_WAVELENGTH) //m/s  //750Hz(0.5pm)
#define LEAP_NUM                    24
#ifdef __cplusplus
extern "C" {
#endif

	enum
	{
		GPS_TOW_FIRST = 1,          /* GPS first get TOW */
		GLN_TOW_FIRST = 2,          /* GLN first get TOW */
		BDS_TOW_FIRST = 3,          /* BDS first get TOW */
		GAL_TOW_FIRST = 4
	};

	typedef struct
	{
		double          tauGPS[GNSS_MAX_MODE];                                  /* system time bias reference to GPS without HW bias */
		double          hwBias[GNSS_MAX_MODE];                                  /* HW bias with GPS as reference */
		double          hwBiasLocal[GNSS_MAX_MODE];
	} GNSS_TIMESYSPARAM;

	////////////////////////////////////////////////////////////////////////
	void gnss_tm_init(void);
	GNSS_TIME* gnss_tm_get_time(void);
	GNSS_TIMESYSPARAM* gnss_tm_get_sysparam(void);
	void gnss_tm_propagate(double tor, uint8_t flt_reverse);
	void gnss_tm_set_gps(uint16_t week, double tow);
	void gnss_tm_set_bds(uint16_t week, double tow);
	void gnss_tm_set_gln(uint16_t NT,uint16_t N4,double tow);
	void gnss_tm_set_gal(uint16_t week, double tow);
	void gnss_tm_gps2utc(UtcTimeType* p_utc, uint16_t gpsWeek, double rcvr_time);
	void gnss_tm_utc2bds(UtcTimeType* p_utc, uint16_t* bdsWeek, double* rcvr_time);
	void gnss_tm_utc2gln(UtcTimeType* p_utc, uint16_t* NT, uint16_t* N4, double* rcvr_time);
	void gnss_tm_leap_secs_adjust(GNSS_TIME* pTime,meas_blk_t* pMeas);
	uint8_t gnss_tm_check_bias_status(uint8_t gnssMode);
	int gnss_tm_compile_time(double* tow);	
	uint32_t gnss_tm_leap_secs(uint8_t gnssMode, UtcTimeType* p_utc);
#ifdef __cplusplus
}
#endif


#endif