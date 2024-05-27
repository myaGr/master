#ifndef __GNSS__SD__DEC__H__
#define __GNSS__SD__DEC__H__


#include "gnss_sd_dec_type.h"
#include "gnss_sd_nm.h"
//#include "gps_android.h"
#include "gnss_api_def.h"

#define abs(x) ((x) < 0 ? (-(x)) : (x))  //return abs

#ifdef __cplusplus
extern "C" {
#endif

	//*********************************************************
	//agnss offline files decoding
	uint8_t gnss_sd_agnssDec_logFile(FILE *fid);

	void *get_eph_offline( uint8_t GNSS_MODE, uint8_t prn, double tor );

	//*********************************************************
	//rawbits offline files decoding
	uint8_t gnss_sd_msgDec_logFile( FILE *fid );

	//*********************************************************
	//rawbits online files decoding,for SPRD interface
	uint8_t gnss_sd_msgDec(const void *msgBits);

	//*********************************************************
	//rawbits online files decoding,for Qualcomm interface
	uint8_t gnss_sd_msgDec_N(const void *msgBits);

#ifdef __cplusplus
}
#endif

#endif