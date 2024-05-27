#ifndef __GNSS__SD__POS__H__
#define __GNSS__SD__POS__H__

#include "gnss.h"
#include "gnss_sd.h"
#include "gnss_sd_nm.h"
#include "gnss_sys_api.h"
#include "gnss_tm.h"

#ifdef __cplusplus
extern "C" {
#endif

	////////////////////////////////////////////////////////////////
	uint8_t gps_sd_satpos_e( GPS_EPH_INFO* pEph, double t, uint8_t sv_id, ECEF* svp, double* dClk,double* dopp_corr, uint32_t freq_index);

	uint8_t gln_sd_sv_pos_e(uint8_t doExpireCheck,GLN_EPH_INFO* pEph, double t, ECEF* svpos, double* dClk,double* dopp_corr);

	uint8_t bds_sd_sv_pos_e( BDS_EPH_INFO* pEph, double t, ECEF* svpos, uint8_t GEO, double* dClk,double* dopp_corr,uint32_t freq_index);

	double gnss_Sd_Pos_e(  uint8_t gnssMode, uint8_t sv_id, double t, ECEF* svp,double* dopp_corr, uint32_t freq_index);

	double gnss_Sd_Pos_e1(uint8_t gnssMode, uint8_t sv_id, double t, ECEF* svp, double* dopp_corr, uint32_t freq_index);

	uint8_t gps_sd_satpos_clk_dop(GPS_EPH_INFO* pEph, double t, double* dClk,double* dopp_corr, uint32_t freq_index);

	uint8_t gln_sd_sv_clk_dop(uint8_t doExpireCheck, GLN_EPH_INFO* pEph, double t, double* dClk,double* dopp_corr);

	uint8_t gal_sd_sv_clk_dop(GAL_EPH_INFO* pEph,double t, double* dClk,double* dopp_corr, uint32_t freq_index);

	uint8_t bds_sd_sv_clk_dop(BDS_EPH_INFO* pEph, double t, double* dClk,double* dopp_corr, uint32_t freq_index);

	void gnss_Construct_R(double dt,double *R);

#ifdef __cplusplus
}
#endif


#endif