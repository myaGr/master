#ifndef __GNSS_SD__H__
#define __GNSS_SD__H__

#include "gnss.h"
#include "gnss_types.h"
#include "gnss_rtk.h"
#include "gnss_mode.h"

#define DCOS_PERIOD               ((double)1.0)
#define MCORR_PERIOD              ((double)1.0)
#define ALM_USE_AGE               (4)
#define SV_INVISIBLE_THRES        (-5)
#define REL_HUMI                  (0.7)             /* relative humidity for saastamoinen model */
#ifdef __cplusplus
extern "C" {
#endif

#define     DGNSS_STATUS_NONE           (0)        // there is no rtd correction
#define     DGNSS_STATUS_NEW_EPH        (1<<1)     // this correction is based on new eph
#define     DGNSS_STATUS_OLD_EPH        (1<<2)     // this correction is based on old eph

#define     MEAS_SAVE_NUM               6

	/* structure used to hold measurement correction data */
	typedef struct
	{
		uint8_t       iono_status;
		uint8_t       newWAASIono;
		uint8_t       valid;
		float      trop_corr;          /* tropospheric correction (meters)    */
		float      erot_corr;          /* earth rotation correction (meters) */
		float      iono_corr;          /* computed iono correction (meters)  */
#ifdef USED_IN_MC262M
		float      iono_sigma;         /* WAAS provides an estimate of the   */
		float      meas_corr;          /* measurement correction (meters)    */
#endif
		float      azimuth;            /* satellite azimuth (semi-circles)   */
#ifdef USED_IN_MC262M
		float      meas_corr_dot;
#endif
		float      erot_corr_dot;
		float      time;
	} MCORR;
	/* structure used to hold intermediate measurement correction data */
	typedef struct
	{
		float      flat;
		float      flong;
		float      survel;
		float      tropo_alt;
		double      Zdry;  /* Dry Zenith tropo component from Hopfield Model */
		double      Zwet;  /* Wet Zenith tropo component from Hopfield Model */
	} MCPREP;
	/* structure used to hold east-north-up transforms */
	typedef struct
	{
		float
			ce,     // north.z
			et1,    // east.x
			et2,    // east.y
			nt1,    // north.x
			nt2,    // north.y
			utx,    // up.x
			uty,    // up.y
			utz;    // up.z
	} TRANS;

	typedef struct
	{
        int32_t   cycleSlipCnt;
	#ifdef USED_IN_MC262M
		float   codeStd;
	#endif
		double   tor;
		double   biasAdjMs;
		double   driftAdj;
		double   pr;
		double   dr;
	#ifdef USED_IN_MC262M
		double   cr;           /* carrier phase range */
	#endif
        double   tot;
		double   PRKfTest;
	}MEAS_SAVE;
	typedef struct
	{
		uint8_t            gnssMode;
		uint8_t            prn;
		uint8_t            freq_index;//for dual_freq
#ifdef USED_IN_MC262M
		uint8_t            sat_type;
#endif
		uint8_t            isReAcq : 1;
		uint8_t            prWithErr : 1;
		uint8_t            isBadPr : 1;
		uint8_t            updateSumDiff :1;
		uint8_t			  fit_flag : 1;//fitting flag: 1:success;0:fail
		uint8_t            drUdFlag;
		uint8_t            prUdFlag;
		uint8_t            measBackCnt;
		uint8_t            prRCnt;
		uint8_t            badPrCnt;
		uint8_t            goodPrCnt;
		float			  toe;//time epoch of EPH
		uint8_t            last_prflag;
		double           prDiffSum;
#ifdef USED_IN_MC262M
		double           prErrSum;
#endif
		double           range;
		double           torReAcq;
#ifdef USED_IN_MC262M
		double           torPrErr;
#endif
		double           torBadPr;
		double           prR;
		/*******
		* last_pr, last_dr and last_tor were used for 1ms bias Jump
		*******/
		double           last_pr;
		double           last_dr;
		double           last_tor;
		MEAS_SAVE     measBack[MEAS_SAVE_NUM];

		D_COS         d_cos;
		ECEF          ecef;
		MCORR         mcorr;
		double			  svt[2];

		double			  fit_tm_start;//start time of fitting
		double			  B_XYZ_pos[12];//fitting coefficient of Pos,3*4=12��3->X Y Z;4:three oder equation ,four unknown parameters
		double			  B_XYZ_vel[12];//fitting coefficient of Vel,3*4=12��3->X Y Z;4:three oder equation ,four unknown parameters
#if 0
		uint8_t			  reqEphFlag;
		int64_t			  lastRequestTime;
		double			  reqEphThresT;
#endif
	}sat_data_t;

	typedef struct 
	{
		uint8_t           Mcorr_init : 1;
		uint8_t           geoId_init : 1;
		uint8_t			 sat_data_idx[MAXSAT][AGGNSS_MAX_FREQ_NUM + 1];
		float          udcos[3];                     // user direction cosine
		sat_data_t*  sat_data[MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM];       // satellite data
		double          sd_geoid_corr;                // GeoId Height correction
		double          geoId_LLA[3];                 // LLA according to GeoId
		double          last_mcorr_tor;
		MCPREP       sd_flla;                      // intermediate data for measurement correction
		TRANS        sd_enu_trans;                 // east-north-up transforms
	}Sd_t;

	/* satellite position and velocity calculation */
	double gnss_Sd_Pos_e(uint8_t gnssMode,uint8_t sv_id,double t,ECEF* svp,double* dopp_corr, uint32_t freq_index);
	double gnss_Sd_Clk(uint8_t gnssMode, uint8_t sv_id, double t, double* dopp_corr, uint32_t freq_index);
	double gnss_Sd_Clk_Corr(uint8_t gnssMode, uint8_t sv_id, uint32_t freq_index);
	double gnss_Sd_Pos_e1(uint8_t gnssMode, uint8_t sv_id, double t, ECEF* svp, double* dopp_corr, uint32_t freq_index);
	uint8_t gnss_Sd_Pos_a(uint8_t gnssMode,uint8_t sv_id,uint16_t week,double t,ECEF* svp);
	uint8_t gnss_Sd_Pos(uint8_t gnssMode,uint8_t sv_id,uint16_t week,double t,ECEF* svp, uint32_t freq_index);

	/* satellite channel management */
	int16_t gnss_sd_svch_get(uint8_t gnssMode, uint8_t sv, uint32_t freq_index);
	void gnss_sd_svch_rm(uint8_t gnssMode, uint8_t sv);
	void gnss_sd_svch_insert(uint8_t gnssMode, uint8_t sv, uint32_t freq_index);
	void gnss_sd_add_sv(uint8_t gnssMode, uint8_t sv, uint32_t freq_index);
	sat_data_t* gnss_sd_get_sv_data(uint8_t gnssMode, uint8_t sv, uint32_t freq_index);
	void gnss_sd_save_meas(sat_data_t* sp, gnss_meas_t* pSvMeas, double tor, double biad_adj, double drift_adj);
	void gnss_sd_clear_meas(Sd_t* p_Sd);
	/* satellite IONO and TROP correction */
	void gnss_sd_mcorr_prep(void* user_ecef, void* user_lla, TRANS* tr, MCPREP* mc, double* geoid_corr);
	uint8_t gnss_sd_get_iono_delay(uint8_t gnssMode,uint32_t freq_index ,double tow, double latitude, double longitude, double elevation, double azimuth, double* ionodelay);
	void gnss_sd_sat_iontrocorr(uint8_t gnssMode, uint8_t prn, double* userlla, D_COS* dc, MCORR* mc, uint32_t freq_index);
	uint8_t gnss_sd_get_pos_clc_sbas(uint8_t gnssMode, uint8_t prn, double *rs, double *dts);
	/* SD module API */
	void gnss_Sd_Init(Sd_t* sd);
	void gnss_Sd_Main(Sd_t* sd,void* upvt);
	void gnss_Sd_Del(void);

	/* RTD data API */
	void gnss_sd_rtcm_init(Rtcm_data_t* p_Rtd);
	void gnss_sd_rtcm_del(Rtcm_data_t* p_Rtd);
	void gnss_sd_rtd_glb_chck(meas_blk_t* pMeas,Rtcm_data_t* p_rtcm, PE_MODES* pMode);
	dgnss_t* gnss_sd_get_sv_rtd(uint8_t gnssMode,uint8_t prn);
	void gnss_sd_rtcm_rtd_proc(rtcm_data_pack_t* rawData);
	void gnss_sd_rm_sv_rtd(uint8_t gnssMode, uint8_t prn);
    void gnss_sd_rtcm_rtk_proc(rtcm_data_pack_t* rawData);
	uint8_t gnss_sd_prn_check(uint8_t gnssMode,int16_t prn);

	double gnss_sd_geoidh(const double *pos);
	double gnss_sd_get_geoId(void);
	//void gnss_sd_set_diffage(double age);
#ifdef __cplusplus
}
#endif


#endif
