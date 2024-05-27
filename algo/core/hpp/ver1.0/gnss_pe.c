/************************************************************
* Copyrights(C)
* All rights Reserved
* �ļ����ƣ�gnss_pe.c
* �ļ�������
* �汾�ţ�1.0.0
* ���ߣ�
* ���ڣ�08/08
************************************************************/

#include <math.h>

#include "macro.h"
#include "gnss.h"
#include "gnss_ls.h"
#include "gnss_kf.h"
#include "gnss_pe.h"
#include "gnss_mode.h"
#include "gnss_config.h"
#include "gnss_sd.h"
#include "gnss_qos.h"
#include "gnss_tm.h"
#include "gnss_math.h"
#include "gnss_sd_nm.h"
#include "gnss_sd_dec.h"
#include "gnss_comm.h"
#include "gnss_back.h"
#include "gnss_common.h"

//#include "gnss_hsm_re.h"
#include "gnss_rtk.h"
//#include "gnss_hsm_data.h"
#include "gnss_hsm_lite_api.h"
#include "gnss_cfd.h"
#include "gnss_config_default.h"

//#define  OFFLINE_INFO_SAVE   //store offline info

#undef MODULE_TAG
#define MODULE_TAG OBJ_PE


//[DEFAULT_DCB_TABLE] discription
double  DEFAULT_DCB_TABLE[10][8] = {
{  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	 0.0},	//SUB_CHIPTYPE_ANYTYPE = 0,	
{  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	 0.0},	//SUB_CHIPTYPE_UBLOX_M8 = 1,	
{ 7.32,	 3.03,	10.71,	 4.71,	  0.0,	  0.0,	  0.0,	 0.0},	//SUB_CHIPTYPE_UBLOX_F9 = 2,L1_L2: GPS  GLN  BDS  GAL,	L1_L5: GPS  GLN  BDS  GAL
{  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	 0.0},	//SUB_CHIPTYPE_ST_8090 = 3,	
{  2.3,	  0.0,	 -3.0,   -9.0,	 -6.8,	  0.0,	 -6.5,	-8.2},	//SUB_CHIPTYPE_ST_8100 = 4,	L1_L2: GPS  GLN  BDS  GAL,	L1_L5: GPS  GLN  BDS  GAL//ST firmware 9.1.1
{  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	 0.0},	//SUB_CHIPTYPE_MXT_900B = 5,	
{  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	 0.0},	//SUB_CHIPTYPE_MXT_900D = 6,	
{  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	 0.0},	//SUB_CHIPTYPE_QCOM_SF = 7,	
{  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	 0.0},	//SUB_CHIPTYPE_QCOM_DF = 8,	
{  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	  0.0,	 0.0} }; //SUB_CHIPTYPE_BRCM_47755 = 9

/* pe config */
Gnss_Cfg_t				g_pe_cfg;

/* define the raw measurements */
meas_blk_t* gpz_Meas;

/* Define LS working area used for GNSS position */
Ls_t* gpz_Ls;
static LS_PVT_INFO*     gpz_ls_pvt_data;

/* Define KF working area used for GNSS position */
Kf_t*      gpz_kf_data;
static Kf_t* p_Kf;
static KF_PVT_INFO*      gpz_kf_pvt_data;
static KF_PVT_INFO*      gpz_kf_pvt_back;

/* user working Modes */
PE_MODES                peMode;
uncertainty_t			g_pvtunc;

// satellite date
extern Sd_t             Sd_data;
Sd_t* p_Sd = &Sd_data;

// User output position
USER_PVT*                gpz_user_pvt;
USER_PVT                user_pvt_backup;

// RTD Data
#if 0
extern Rtcm_data_t      g_rtcm_data;
Rtcm_data_t* p_rtcm = &g_rtcm_data;
#else
Rtcm_data_t* p_rtcm = NULL;

#endif

// PE request assisted data for command
PEReqCommand_t          peReqCommand;
// Global value used to save fusionRslt
//AGFusionResults         DRFusionRslt;

/*EpochCnt count the total epoch count of measurements  */
PE_Context_t            peContext;

//position engine work state monitor 
PeStateMonitor_t        peState;

/* history KF information */
history_Info            HisInfo;

/* The scenario information, updated after PVT in every epoch. */
unc_scene_t				      g_unc_scene;


/* feedback infomation from FLP*/
VDR_Gnss_Feedback_t     vdrFeedbackInfo;

extern gnss_nm_t* p_Nm;

extern double           rtdTimeGap;

int32_t fullReqFlag = 0;

static uint32_t triaCnt = 0;

/* Max elapse time for setting pos approx */
static float  const  FIX_LIFETIME[N_DYN] =
{
    (float)180,                                   /* Undefined */
    (float)300,                                   /* LAND */
    (float)600,                                   /* SEA */
    (float)60,                                    /* AIR */
    (float)1.0e12f,                               /* STATIONARY */
    (float)600                                    /* AUTOMOBILE */
};

/*First fix flag */
uint8_t firstFix;

/* reverse process flag for PPK process */
static uint8_t ppk_reverse = 0;   //0: forward   1:reverse
//illegal area flag
uint8_t illegalArea;//0--China Local, 1-- outside of China

extern const uint8_t clockBiasIdx[BIAS_NUM][BIAS_NUM];
extern KF_INFO  kfInfo;
extern GNSS_TIME* p_gnssTime;
extern GNSS_TIMESYSPARAM* p_gnssTimeSysParam;
extern void gnss_Pe_Log_Meas(meas_blk_t* pMeas);
extern rtk_t rtkctrl;            //rtk control and results
extern proc_ctrl_t g_proc;

extern uint32_t Leaplist[LEAP_NUM];
extern uint32_t LeapListInUse[LEAP_NUM];
extern double leaps[MAXLEAPS + 1][7];
extern double leapsInUse[MAXLEAPS + 1][7];

float realVel = 0.0;   // reference horizon velocity
double realAlt = 0.0;   // reference altitude
double realPosErr = 0.0;    // position error of current epoch 
uint8_t weekCheckNum = 0;
uint8_t timeResetFlag = 0;

void gnss_Pe_set_ppk_filter_direction(uint8_t reverse)
{
  ppk_reverse = (reverse == 0 ? 0 : 1);
}

uint8_t gnss_Pe_get_ppk_filter_direction()
{
  return ppk_reverse;
}

static Gnss_Test_Cfg_t global_cfg;

void Gnss_PE_DefaultCfg(Gnss_Cfg_t* cfg)
{
  rtk_algo_cfg_t* rtk_cfg = &cfg->rtk_cfg;

  cfg->ppk_mode = DFLTCFG_PE_PPK_MODE;
  cfg->enable_pe = TRUE;
  cfg->cno_mask = DFLTCFG_PE_CN0_MASK;
  cfg->altLowLimit = DFLTCFG_PE_ALT_LOW_LIMIT;
  cfg->altHighLimit = DFLTCFG_PE_ALT_HIGH_LIMIT;
  cfg->enable_indoor = DFLTCFG_PE_ENABLE_INDOOR;
  cfg->enable_waas = DFLTCFG_PE_ENABLE_WAAS;
  cfg->pdop_mask = DFLTCFG_PE_PDOP_MASK;
  cfg->dynamics = DFLTCFG_PE_DYNAMICS;
  cfg->pos_res_thres = DFLTCFG_PE_POS_RES_THRES;
  cfg->ele_mask = DFLTCFG_PE_ELE_MASK;
  cfg->start_mode = DFLTCFG_PE_START_MODE;
  cfg->chipType = DFLTCFG_PE_MEAS_TYPE;
  cfg->sub_chipType = DFLTCFG_PE_SUB_CHIPTYPE;
  cfg->rinexDataType = DFLTCFG_PE_RINEX_DATA_TYPE;
  cfg->meas_type_mask = DFLTCFG_PE_MEAS_TYPE_MASK;
  cfg->automobile = DFLTCFG_PE_AUTO_MOBILE;
  cfg->applyScenario = DFLTCFG_PE_APPLY_SCENE;
  cfg->codeStdPhone = DFLTCFG_PE_CODE_STD_PHONE;
  cfg->codeStdAuto = DFLTCFG_PE_CODE_STD_AUTO;
  cfg->carrStdPhone = DFLTCFG_PE_CARR_STD_PHONE;
  cfg->carrStdAuto = DFLTCFG_PE_CARR_STD_AUTO;
  cfg->rtd_usage_flag = DFLTCFG_PE_RTD_USAGE_FLAG;
  cfg->gnss_usage_flag = DFLTCFG_PE_GNSS_USAGE_FLAG;

  cfg->meas_rate = DFLCFG_SYS_MEAS_RATE;
  cfg->applyScenario = DFLTCFG_PE_APPLY_SCENE;

  rtk_cfg->nav_sys_gps = (DFLTCFG_RTK_NAV_SYS_MASK & RTK_NAV_MASK_BIT_GPS) == 0 ? 0 : 1;
  rtk_cfg->nav_sys_sbas = (DFLTCFG_RTK_NAV_SYS_MASK & RTK_NAV_MASK_BIT_SBAS) == 0 ? 0 : 1;
  rtk_cfg->nav_sys_glonass = (DFLTCFG_RTK_NAV_SYS_MASK & RTK_NAV_MASK_BIT_GLN) == 0 ? 0 : 1;
  rtk_cfg->nav_sys_qzss = (DFLTCFG_RTK_NAV_SYS_MASK & RTK_NAV_MASK_BIT_QZSS) == 0 ? 0 : 1;
  rtk_cfg->nav_sys_galileo = (DFLTCFG_RTK_NAV_SYS_MASK & RTK_NAV_MASK_BIT_GAL) == 0 ? 0 : 1;
  rtk_cfg->nav_sys_compass = (DFLTCFG_RTK_NAV_SYS_MASK & RTK_NAV_MASK_BIT_BDS) == 0 ? 0 : 1;
  rtk_cfg->enable_first_static = DFLTCFG_RTK_ENABLE_FIRST_STATIC;
  rtk_cfg->bds_armode = DFLTCFG_RTK_ENABLE_BDS_ARMODE;
  rtk_cfg->glo_armode = DFLTCFG_RTK_ENABLE_GLN_ARMODE;
  rtk_cfg->out_rinex_file = DFLTCFG_RTK_OUTPUT_RINEX_FILE;
  rtk_cfg->debug_level = DFLTCFG_RTK_DEBUG_LEVEL;
  rtk_cfg->freq_combine = DFLTCFG_RTK_FREQ_COMBINE;
  rtk_cfg->process_type = DFLTCFG_RTK_PROCESS_TYPE;
  rtk_cfg->pos_mode = RTK_POS_MODE_KINEMATIC;
  rtk_cfg->ar_mode = DFLTCFG_RTK_ARMODE;
  rtk_cfg->enable_rtd = DFLTCFG_RTK_ENABLE_RTD;
  rtk_cfg->ar_lock_cnt = DFLTCFG_RTK_AR_LOCK_CNT;
  rtk_cfg->ar_min_fix = DFLTCFG_RTK_AR_MIN_FIX;
  rtk_cfg->ar_out_cnt = DFLTCFG_RTK_AR_OUT_CNT;
  rtk_cfg->ar_elevation_mask = DFLTCFG_RTK_AR_ELE_MASK;
  rtk_cfg->ar_threshold = DFLTCFG_RTK_AR_THRES;
  rtk_cfg->elevation_mask_hold = DFLTCFG_RTK_ELE_MASK_HOLD;
  rtk_cfg->elevation_mask = DFLTCFG_RTK_ELE_MASK;
  rtk_cfg->max_age = DFLTCFG_RTK_MAX_AGE;
  rtk_cfg->rej_thres_innovation = DFLTCFG_RTK_REJ_THRES_INNO;
  rtk_cfg->err_ratio_1 = DFLTCFG_RTK_ERR_RATIO_1;
  rtk_cfg->err_ratio_2 = DFLTCFG_RTK_ERR_RATIO_2;
  rtk_cfg->err_phase_factor_a = DFLTCFG_RTK_ERR_PHASE_FACTOR_A;
  rtk_cfg->err_phase_factor_b = DFLTCFG_RTK_ERR_PHASE_FACTOR_B;
  rtk_cfg->err_phase_factor_c = DFLTCFG_RTK_ERR_PHASE_FACTOR_C;
  rtk_cfg->err_phase_doppler_freq = DFLTCFG_RTK_ERR_PHASE_DOPP_FREQ;
  rtk_cfg->init_state_std_bias = DFLTCFG_RTK_INIT_STATE_STD_BIAS;
  rtk_cfg->proc_noise_std_bias = DFLTCFG_RTK_PROC_NOISE_STD_BIAS;
}

void gnss_Pe_Init(const Gnss_Cfg_t* cfg)
{
  firstFix = FALSE;
  illegalArea = FALSE;

  memset(&peContext, 0, sizeof(peContext));
  memset(&peReqCommand, 0, sizeof(peReqCommand));
  memset(&peState, 0, sizeof(peState));

  memcpy(&g_pe_cfg, cfg, sizeof(g_pe_cfg));
  /*Init Meas block */
  gpz_Meas = OS_MALLOC_FAST(sizeof(meas_blk_t));

  /* malloc */
  gpz_ls_pvt_data = OS_MALLOC_FAST(sizeof(LS_PVT_INFO));
  gpz_Ls = OS_MALLOC_FAST(sizeof(Ls_t));
  gpz_kf_pvt_data = OS_MALLOC_FAST(sizeof(KF_PVT_INFO));
  gpz_kf_pvt_back = OS_MALLOC_FAST(sizeof(KF_PVT_INFO));
  gpz_kf_data = OS_MALLOC_FAST(sizeof(Kf_t));
  gpz_user_pvt = OS_MALLOC_FAST(sizeof(USER_PVT));
  if (NULL == gpz_Meas || NULL == gpz_ls_pvt_data || NULL == gpz_Ls || NULL == gpz_kf_pvt_data ||
      NULL == gpz_kf_pvt_back || NULL == gpz_kf_data || NULL == gpz_user_pvt)
  {
    OS_FREE(gpz_Meas);
    OS_FREE(gpz_ls_pvt_data);
    OS_FREE(gpz_Ls);
    OS_FREE(gpz_kf_pvt_data);
    OS_FREE(gpz_kf_pvt_back);
    OS_FREE(gpz_kf_data);
    OS_FREE(gpz_user_pvt);
    g_pe_cfg.enable_pe = FALSE;
    GLOGE("GNSS PVT ver1.0 init failed, cant alloc memory \n");
    return;
  }

  if (g_pe_cfg.is_playback)
    triaCnt++;

  /* Init LS Module */
  gnss_Ls_Init(gpz_Ls);
  gpz_Ls->ls_Pvt_Info = gpz_ls_pvt_data;
  gpz_Ls->pCfg = &g_pe_cfg;

  /* Init KF Module */
  p_Kf = gpz_kf_data;
  gnss_Kf_Init(p_Kf);
  p_Kf->lsBlk = gpz_Ls;
  p_Kf->ls_Pvt_Info = gpz_ls_pvt_data;
  p_Kf->kf_Pvt_Info = gpz_kf_pvt_data;
  p_Kf->kf_Pvt_Back = gpz_kf_pvt_back;
  p_Kf->pCfg = &g_pe_cfg;
  p_Kf->meas_blk = gpz_Meas;

  /* Time module init */
  gnss_tm_init();

  /* SD module init */
  gnss_Sd_Init(p_Sd);
  gnss_sd_nm_init_glo_chn();

  /*NM init*/
  memset(p_Nm->week_decoded, 0, sizeof(p_Nm->week_decoded));

  /* RTD data init */
  gnss_sd_rtcm_init(p_rtcm);

  /* Mode init */
  gnss_Mode_Init(&peMode);

  memset(&peReqCommand, 0, sizeof(PEReqCommand_t));

  if (!g_pe_cfg.is_playback)
  {/* User PVT init */
    memset(gpz_user_pvt, 0, sizeof(USER_PVT));
    gpz_user_pvt->have_position = HAVE_POS_NONE;

    memset(&user_pvt_backup, 0, sizeof(USER_PVT));
    user_pvt_backup.have_position = HAVE_POS_NONE;
  }
  else
  {
    if (triaCnt == 1)
    {
      /* User PVT init */
      memset(gpz_user_pvt, 0, sizeof(USER_PVT));
      gpz_user_pvt->have_position = HAVE_POS_NONE;

      memset(&user_pvt_backup, 0, sizeof(USER_PVT));
      user_pvt_backup.have_position = HAVE_POS_NONE;
    }
    else
    {
      user_pvt_backup.have_position = HAVE_POS_INIT;
      user_pvt_backup.ecef.have_position = POS_FRM_NULL;
      user_pvt_backup.lla.have_position = POS_FRM_NULL;
      memcpy(gpz_user_pvt, &user_pvt_backup, sizeof(USER_PVT));
    }
  }
  memset(gpz_user_pvt, 0, sizeof(USER_PVT));

#ifdef ASG_USE_RTK_ALGO
  //rtk module init
  gnss_rtk_init(&cfg->rtk_cfg);
#endif

  memcpy(&LeapListInUse[0], &Leaplist[0], sizeof(uint32_t) * LEAP_NUM);
  memcpy(&leapsInUse[0][0], &leaps[0][0], sizeof(double) * (MAXLEAPS + 1) * (7));
  gnss_Pe_Set_HW_Bias(NULL);

  // SPP
  if (fabs(p_gnssTimeSysParam->hwBias[GLN_MODE]) > 1.0)
  {
    p_Kf->biasDiff[GLN_MODE] = p_gnssTimeSysParam->hwBias[GLN_MODE];
  }
  if (fabs(p_gnssTimeSysParam->hwBias[BDS_MODE]) > 1.0)
  {
    p_Kf->biasDiff[BDS_MODE] = p_gnssTimeSysParam->hwBias[BDS_MODE];
  }
  if (fabs(p_gnssTimeSysParam->hwBias[GAL_MODE]) > 1.0)
  {
    p_Kf->biasDiff[GAL_MODE] = p_gnssTimeSysParam->hwBias[GAL_MODE];
  }
  // RTD
  if (fabs(p_gnssTimeSysParam->hwBiasLocal[GLN_MODE]) > 1.0)
  {
    p_Kf->biasDiffLocal[GLN_MODE] = p_gnssTimeSysParam->hwBiasLocal[GLN_MODE];
  }
  if (fabs(p_gnssTimeSysParam->hwBiasLocal[BDS_MODE]) > 1.0)
  {
    p_Kf->biasDiffLocal[BDS_MODE] = p_gnssTimeSysParam->hwBiasLocal[BDS_MODE];
  }
  if (fabs(p_gnssTimeSysParam->hwBiasLocal[GAL_MODE]) > 1.0)
  {
    p_Kf->biasDiffLocal[GAL_MODE] = p_gnssTimeSysParam->hwBiasLocal[GAL_MODE];
  }
  // bias difference quality check
  if (g_pe_cfg.chipType == QCOM)
  {
    if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_BRCM_47755)
    {
      // GPS-GLN bias
      if (p_Kf->biasDiff[GLN_MODE] > (DEFAULT_GPSGLN_HW_BIAS_BRCOM + 200.0)
        || p_Kf->biasDiff[GLN_MODE] < (DEFAULT_GPSGLN_HW_BIAS_BRCOM - 200.0))
      {
        p_Kf->biasDiff[GLN_MODE] = 0.0;
      }
      if (p_Kf->biasDiffLocal[GLN_MODE] > (DEFAULT_GPSGLN_HW_BIAS_BRCOM + 200.0)
        || p_Kf->biasDiffLocal[GLN_MODE] < (DEFAULT_GPSGLN_HW_BIAS_BRCOM - 200.0))
      {
        p_Kf->biasDiffLocal[GLN_MODE] = 0.0;
      }

      // GPS-BDS bias
      if (p_Kf->biasDiff[BDS_MODE] > (DEFAULT_GPSBDS_HW_BIAS_BRCOM + 200.0)
        || p_Kf->biasDiff[BDS_MODE] < (DEFAULT_GPSBDS_HW_BIAS_BRCOM - 200.0))
      {
        p_Kf->biasDiff[BDS_MODE] = 0.0;
      }
      if (p_Kf->biasDiffLocal[BDS_MODE] > (DEFAULT_GPSBDS_HW_BIAS_BRCOM + 200.0)
        || p_Kf->biasDiffLocal[BDS_MODE] < (DEFAULT_GPSBDS_HW_BIAS_BRCOM - 200.0))
      {
        p_Kf->biasDiffLocal[BDS_MODE] = 0.0;
      }
    }
    else
    {
      // GPS-GLN bias
      if (p_Kf->biasDiff[GLN_MODE] > (DEFAULT_GPSGLN_HW_BIAS + 200.0)
        || p_Kf->biasDiff[GLN_MODE] < (DEFAULT_GPSGLN_HW_BIAS - 200.0))
      {
        p_Kf->biasDiff[GLN_MODE] = 0.0;
      }
      if (p_Kf->biasDiffLocal[GLN_MODE] > (DEFAULT_GPSGLN_HW_BIAS + 200.0)
        || p_Kf->biasDiffLocal[GLN_MODE] < (DEFAULT_GPSGLN_HW_BIAS - 200.0))
      {
        p_Kf->biasDiffLocal[GLN_MODE] = 0.0;
      }

      // GPS-BDS bias
      if (p_Kf->biasDiff[BDS_MODE] > (DEFAULT_GPSBDS_HW_BIAS + 200.0)
        || p_Kf->biasDiff[BDS_MODE] < (DEFAULT_GPSBDS_HW_BIAS - 200.0))
      {
        p_Kf->biasDiff[BDS_MODE] = 0.0;
      }
      if (p_Kf->biasDiffLocal[BDS_MODE] > (DEFAULT_GPSBDS_HW_BIAS + 200.0)
        || p_Kf->biasDiffLocal[BDS_MODE] < (DEFAULT_GPSBDS_HW_BIAS - 200.0))
      {
        p_Kf->biasDiffLocal[BDS_MODE] = 0.0;
      }
    }
  }
  else if (g_pe_cfg.chipType == SPRD)
  {
    // GPS-GLN bias
    if (p_Kf->biasDiff[GLN_MODE] > (DEFAULT_GPSGLN_HW_BIAS_SPRD + 200.0)
      || p_Kf->biasDiff[GLN_MODE] < (DEFAULT_GPSGLN_HW_BIAS_SPRD - 200.0))
    {
      p_Kf->biasDiff[GLN_MODE] = 0.0;
    }
    if (p_Kf->biasDiffLocal[GLN_MODE] > (DEFAULT_GPSGLN_HW_BIAS_SPRD + 200.0)
      || p_Kf->biasDiffLocal[GLN_MODE] < (DEFAULT_GPSGLN_HW_BIAS_SPRD - 200.0))
    {
      p_Kf->biasDiffLocal[GLN_MODE] = 0.0;
    }
  }

  //memset(&DRFusionRslt,0,sizeof(AGFusionResults));
  memset(&vdrFeedbackInfo, 0, sizeof(VDR_Gnss_Feedback_t));
  memset(&g_pvtunc, 0, sizeof(uncertainty_t));
}

/***********************************************************************
* ��������: gnss_Pe_Close
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�10/08/
***********************************************************************/
void gnss_Pe_Close(void)
{
  // free
  OS_FREE(gpz_Meas);
  OS_FREE(gpz_ls_pvt_data);
  OS_FREE(gpz_Ls);
  OS_FREE(gpz_kf_pvt_data);
  OS_FREE(gpz_kf_pvt_back);
  OS_FREE(gpz_kf_data);
  OS_FREE(gpz_user_pvt);

  // delete SD module
  gnss_Sd_Del();
#ifdef AG_GNSS_RTK_FUNCTION_IMPL
  //rtk module deinit
  gnss_rtk_free();
#endif
}

/***********************************************************************
* ��������: gnss_Pe_Get_RTD_Status
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�11/9/
***********************************************************************/
uint8_t gnss_Pe_Get_RTD_Status(void)
{
  return gpz_Meas->rtdUsed;
}

/***********************************************************************
* ��������: gnss_Pe_Get_RTD_Status
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�11/9/
***********************************************************************/
uint8_t gnss_Pe_Get_SvRtd_Status(uint8_t gnssMode, uint8_t prn)
{
  uint32_t      i;
  for (i = 0; i < gpz_Meas->measCnt; i++)
  {
    if (gpz_Meas->meas[i].gnssMode == gnssMode && gpz_Meas->meas[i].prn == prn)
    {
      return ((gpz_Meas->meas[i].status >> 2) & 0x1);
    }
  }
  return FALSE;
}

/***********************************************************************
* Description: gnss_Pe_Get_Kf
* Input:
* Output:
* Return:
* Author: 
* Date: 05/19/
***********************************************************************/
Kf_t* gnss_Pe_Get_Kf(void)
{
  return p_Kf;
}

/***********************************************************************
* Description: gnss_Pe_Get_PeState
* Input:
* Output:
* Return:
* Author: 
* Date: 10/19/
***********************************************************************/
void gnss_Pe_Get_PeState(PeStateMonitor_t* p)
{
  (*p) = peState;
}

void gnss_Pe_Time_Judge(GNSS_TIME* pTime)
{
  uint8_t GPS_FLAG = 0, BDS_FLAG = 0, GAL_FLAG = 0, RESET_FLAG = 0;
  uint8_t GPS_VALID = 0, BDS_VALID = 0, GAL_VALID = 0;

  if (!pTime->init)
  {
    return;
  }

  if (timeResetFlag)
  {
    return;
  }

  if (pTime->tor > 900.0 && (pTime->week[GPS_MODE] != pTime->week_save[GPS_MODE] ||
    pTime->week[GPS_MODE] != (pTime->week_save[BDS_MODE] + 1356)) && pTime->week_save[GPS_MODE] > 1024 &&
    pTime->week_save[BDS_MODE] > 692)
  {
    // before corss week
    if (pTime->week_save[GPS_MODE] == (pTime->week_save[BDS_MODE] + 1356) &&
      pTime->week_save[GPS_MODE] == (pTime->week_save[GAL_MODE] + 1024) &&
      pTime->week[GPS_MODE] != pTime->week_save[GPS_MODE])
    {
      // boradcase check success
      timeResetFlag = 1;
      pTime->init = FALSE;
      pTime->week[GPS_MODE] = pTime->week_save[GPS_MODE];
      GLOGI("week reset to %d", pTime->week[GPS_MODE]);
    }
    else if (pTime->week_save[GPS_MODE] == (pTime->week_save[BDS_MODE] + 1356) &&
      pTime->week[GPS_MODE] != pTime->week_save[GPS_MODE])
    {
      if (++weekCheckNum > 60)
      {
        timeResetFlag = 1;
        pTime->init = FALSE;
        pTime->week[GPS_MODE] = pTime->week_save[GPS_MODE];
        weekCheckNum = 0;
        GLOGI("week reset to %d", pTime->week[GPS_MODE]);
      }
    }
    else if (pTime->week_save[GPS_MODE] == (pTime->week_save[GAL_MODE] + 1024) &&
      pTime->week[GPS_MODE] != pTime->week_save[GPS_MODE])
    {
      if (++weekCheckNum > 60)
      {
        timeResetFlag = 1;
        pTime->init = FALSE;
        pTime->week[GPS_MODE] = pTime->week_save[GPS_MODE];
        weekCheckNum = 0;
        GLOGI("week reset to %d", pTime->week[GPS_MODE]);
      }
    }
    else if ((pTime->week_save[BDS_MODE] + 1356) == (pTime->week_save[GAL_MODE] + 1024) &&
      pTime->week[GPS_MODE] != (pTime->week_save[BDS_MODE] + 1356))
    {
      if (++weekCheckNum > 60)
      {
        timeResetFlag = 1;
        pTime->init = FALSE;
        pTime->week[GPS_MODE] = pTime->week_save[BDS_MODE] + 1356;
        pTime->week_save[GPS_MODE] = pTime->week_save[BDS_MODE] + 1356;
        weekCheckNum = 0;
        GLOGI("week reset to %d", pTime->week[GPS_MODE]);
      }
    }
    else
    {
      weekCheckNum = 0;
    }
  }
}
/***********************************************************************
* ��������: gnss_Pe_Set_Time
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�10/08/
***********************************************************************/
void gnss_Pe_Set_Time(GNSS_TIME* pTime, double tor)
{
  uint16_t     week = 0, N4 = 0, NT = 0;
  uint32_t     i;
  double       deltaT;
  double       torTemp = 0.0;

  if (pTime->init)
  {
    return;
  }

  /*
  1. Set GPS tor and convert to GLN tor
  2. week_save is set when decoding from broadcast or AGNSS EPH
  3. Initial time stamp for time module when there is tor
  */
  if (pTime->timeInitSrc == TM_INIT_GROM && pTime->week_save[GPS_MODE] == 0)
  {
    pTime->week_save[GPS_MODE] = pTime->week[GPS_MODE];
  }

  /*
  time init flow: GPS --> GLN --> BDS --> GAL
  */
  if (pTime->week_save[GPS_MODE] > 0 && pTime->weekCheckNum[GPS_MODE] >= 1 && tor > 0.0)
  {
    week = pTime->week_save[GPS_MODE];
    if (g_pe_cfg.chipType == QCOM || g_pe_cfg.chipType == SPRD || g_pe_cfg.chipType == MTK)
    {
      // Find one GPS satellite's tot as tor 
      for (i = 0; i < gpz_Meas->measCnt; i++)
      {
        if (gpz_Meas->meas[i].gnssMode == GPS_MODE && (gpz_Meas->meas[i].measState & AGGNSS_MEASUREMENT_STATE_TOW_DECODED) != 0)
        {
          //gnss_tm_set_gps(week,(gpz_Meas->meas[i].tot + 0.080));
          torTemp = g_pe_cfg.chipType == MTK ? gpz_Meas->meas[i].tot : (gpz_Meas->meas[i].tot + 0.080);
          break;
        }
      }
      if (i >= gpz_Meas->measCnt) return;
    }
    else
    {
      torTemp = tor;
    }
#if 0
    if (pTime->timeInitSrc == TM_INIT_GROM || pTime->timeInitSrc == TM_INIT_AGNSS_L)
    {
      deltaT = torTemp - pTime->rcvr_time[GPS_MODE];

      if (deltaT < -SECS_IN_WEEK / 2.0)  //now we only consider semi-week gap for cross-week
      {
        week++;
      }
      else if (deltaT > SECS_IN_WEEK / 2.0)
      {
        week--;
      }
      SYS_LOGGING(OBJ_PE, LOG_INFO, "Time set: Week=%d is load from GROM/AGNSS_L", pTime->week_save[GPS_MODE]);
    }
#endif
    SYS_LOGGING(OBJ_PE, LOG_INFO, "Time was set with tor(%14.6f),week:%d", torTemp, week);
    gnss_tm_set_gps(week, torTemp);
    pTime->week_save[GPS_MODE] = 0;
    pTime->tor = tor;
    pTime->startTor = tor;
    if (g_pe_cfg.chipType == MTK)
    {
      pTime->lastGpsTor = pTime->gpsTor;
    }
  }
  else if (pTime->N4_save > 0 && pTime->NT_save > 0)
  {
    N4 = pTime->N4_save;
    NT = pTime->NT_save;

    // Find one GLN satellite's tot as tor 
    for (i = 0; i < gpz_Meas->measCnt; i++)
    {
      if (gpz_Meas->meas[i].gnssMode == GLN_MODE && (gpz_Meas->meas[i].measState & AGGNSS_MEASUREMENT_STATE_GLO_TOD_DECODED) != 0)
      {
        //gnss_tm_set_gln(pTime->NT_save,pTime->N4_save,(p_Meas->meas[i].tot + 0.080));
        if (g_pe_cfg.chipType != UBLOX)
        {
          torTemp = gpz_Meas->meas[i].tot + 0.080;
        }
        else
        {
          torTemp = gpz_Meas->meas[i].tot + gpz_Meas->meas[i].pseudoRange_raw / CLIGHT;
          if (fabs(torTemp - (int)torTemp - tor + (int)tor) > 1e-9)  continue;
        }
        break;
      }
    }
    if (i >= gpz_Meas->measCnt) return;

    if (pTime->timeInitSrc == TM_INIT_GROM || pTime->timeInitSrc == TM_INIT_AGNSS_L)
    {
      deltaT = torTemp - pTime->rcvr_time[GLN_MODE];
      if (deltaT < -SECS_IN_DAY / 2.0)
      {
        NT++;
        if (NT > DAY_IN_FOUR_YEAR)
        {
          NT -= DAY_IN_FOUR_YEAR;
          N4++;
        }
      }
      else if (deltaT > SECS_IN_DAY / 2.0)
      {
        if (NT > 1)
        {
          NT -= 1;
        }
        else
        {
          NT = DAY_IN_FOUR_YEAR;
          N4 -= 1;
        }
      }
      SYS_LOGGING(OBJ_PE, LOG_INFO, "Time set: N4=%d NT=%d is load from GROM/AGNSS_L", pTime->N4_save, pTime->NT_save);
    }
    SYS_LOGGING(OBJ_PE, LOG_INFO, "Time was set with tod(%14.6f),N4:%d,NT:%d", torTemp, N4, NT);
    gnss_tm_set_gln(NT, N4, torTemp);
    pTime->N4_save = 0;
    pTime->NT_save = 0;
    pTime->tor = tor;
    pTime->startTor = tor;
  }
  else if (pTime->week_save[BDS_MODE] > 0 && pTime->weekCheckNum[BDS_MODE] >= 1)
  {
    week = pTime->week_save[BDS_MODE];
    if (g_pe_cfg.chipType == UBLOX)
    {
      torTemp = tor - 14.0;
      if (torTemp < 0.0)
      {
        torTemp += (double)SECS_IN_WEEK;
        week--;
      }
    }
    else
    {
      // Find one BDS satellite's tot as tor 
      for (i = 0; i < gpz_Meas->measCnt; i++)
      {
        if (gpz_Meas->meas[i].gnssMode == BDS_MODE && (gpz_Meas->meas[i].measState & AGGNSS_MEASUREMENT_STATE_TOW_DECODED) != 0)
        {
          //gnss_tm_set_bds(pTime->week_save[BDS_MODE],(gpz_Meas->meas[i].tot + 0.080));
          torTemp = gpz_Meas->meas[i].tot + 0.080;
          break;
        }
      }
      if (i >= gpz_Meas->measCnt) return;
    }
#if 0
    if (pTime->timeInitSrc == TM_INIT_GROM || pTime->timeInitSrc == TM_INIT_AGNSS_L)
    {
      deltaT = torTemp - pTime->rcvr_time[BDS_MODE];

      if (deltaT < -SECS_IN_WEEK / 2.0)  //now we only consider semi-week gap for cross-week
      {
        week++;
      }
      else if (deltaT > SECS_IN_WEEK / 2.0)
      {
        week--;
      }
      SYS_LOGGING(OBJ_PE, LOG_INFO, "Time set: Week=%d is load from GROM", pTime->week_save[BDS_MODE]);
    }
#endif
    GLOGI("Time was set with tor(%14.6f),week:%d", torTemp, week);
    gnss_tm_set_bds(week, torTemp);
    pTime->week_save[BDS_MODE] = 0;
    pTime->tor = tor;
    pTime->startTor = tor;
  }
  else if (pTime->week_save[GAL_MODE] > 0 && pTime->weekCheckNum[GAL_MODE] >= 1)
  {
    week = pTime->week_save[GAL_MODE];
    if (g_pe_cfg.chipType == UBLOX)
    {
      torTemp = tor; //gps tow == gal tow
    }
    else
    {
      // Find one GAL satellite's tot as tor 
      for (i = 0; i < gpz_Meas->measCnt; i++)
      {
        if (gpz_Meas->meas[i].gnssMode == GAL_MODE && (gpz_Meas->meas[i].measState & AGGNSS_MEASUREMENT_STATE_TOW_DECODED) != 0)
        {
          //gnss_tm_set_gal(pTime->week_save[GAL_MODE],(gpz_Meas->meas[i].tot + 0.080));
          torTemp = gpz_Meas->meas[i].tot + 0.080;
          break;
        }
      }
      if (i >= gpz_Meas->measCnt) return;
    }
#if 0
    if (pTime->timeInitSrc == TM_INIT_GROM || pTime->timeInitSrc == TM_INIT_AGNSS_L)
    {
      deltaT = torTemp - pTime->rcvr_time[GAL_MODE];

      if (deltaT < -SECS_IN_WEEK / 2.0)  //now we only consider semi-week gap for cross-week
      {
        week++;
      }
      else if (deltaT > SECS_IN_WEEK / 2.0)
      {
        week--;
      }
      SYS_LOGGING(OBJ_PE, LOG_INFO, "Time set: Week=%d is load from GROM", pTime->week_save[GAL_MODE]);
    }
#endif
    GLOGI("Time was set with tor(%14.6f),week:%d", torTemp, week);
    gnss_tm_set_gal(week, torTemp);
    pTime->week_save[GAL_MODE] = 0;
    pTime->tor = tor;
    pTime->startTor = tor;
  }

  /* Init receive time, receive time(in ns) and bias uncertainty */
  if (pTime->init)
  {
    kfInfo.tor = tor;
    pTime->last_time_ns = pTime->time_ns;
    for (i = 0; i < GNSS_MAX_MODE; i++)
    {
      pTime->rcvr_time_ns[i] = (int64_t)(pTime->rcvr_time[i] * 1e9);
      pTime->rcvr_time_subNs[i] = (pTime->rcvr_time[i] * 1e9) - pTime->rcvr_time_ns[i];
      pTime->bias_unc[i] = (float)(LIGHT_MSEC * 10.0);
    }

#if 0
    gtime_t time = gnss_gpst2time(pTime->week[GPS_MODE], pTime->rcvr_time[GPS_MODE]);
    if (time.time < 1577433000)
    {
      DEFAULT_DCB_TABLE[g_pe_cfg.sub_chipType][4] = -46.7;
      DEFAULT_DCB_TABLE[g_pe_cfg.sub_chipType][5] = 0.0;
      DEFAULT_DCB_TABLE[g_pe_cfg.sub_chipType][6] = -37.2;
      DEFAULT_DCB_TABLE[g_pe_cfg.sub_chipType][7] = -45.0;
    }
#endif
    for (i = 0; i < GNSS_MAX_MODE * 2; i++)
    {
      pTime->dcb[i] = DEFAULT_DCB_TABLE[g_pe_cfg.sub_chipType][i];
    }
  }
#if 0
  /* check user position status */
  if (gpz_user_pvt->have_position == HAVE_POS_INIT)
  {
    deltaT = pTime->rcvr_time[GPS_MODE] - gpz_user_pvt->posfix_t;
    deltaT += (pTime->week[GPS_MODE] - gpz_user_pvt->posfix_wn) * 604800.0;
    if (fabs(deltaT) > POSITION_HOLD_TIME)
    {
      gpz_user_pvt->have_position = HAVE_POS_NONE;
    }
  }
#endif
}

/***********************************************************************
* ��������: gnss_Pe_Pre_Proc
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/02/
***********************************************************************/
void gnss_Pe_Rtd_Select(meas_blk_t* pMeas)
{
  uint32_t           i;
  uint32_t           cno35Cnt = 0;
  gnss_meas_t* pSvMeas;
  if (pMeas->rtdUsed)
  {
    for (i = 0; i < GNSS_MAX_MODE; i++)
    {
      cno35Cnt += pMeas->Cno35Cnt[i];
    }
    if (cno35Cnt < 10)
    {
      return;
    }
    for (i = 0; i < pMeas->measCnt; i++)
    {
      pSvMeas = &pMeas->meas[i];
      if (pSvMeas->prn == 0) continue;
      if ((pSvMeas->status & 0x3) == 0) continue;
      if (pSvMeas->cno < 35)
      {
        pSvMeas->status &= 0xF8;
        SYS_LOGGING(OBJ_PE, LOG_INFO, "%s rej:%02d %03d", __FUNCTION__, pSvMeas->gnssMode, pSvMeas->prn);
      }
    }
  }
}

PEReqCommand_t* gnss_Pe_Get_ReqCommand(void)
{
  return &peReqCommand;
}
void gnss_Pe_Reset_ReqCommand(PEReqCommand_t* peReqCom)
{
  peReqCom->flag = FALSE;
  peReqCom->gnss = 0;
  peReqCom->reqType = 0;
}
void gnss_Pe_Set_ReqCommand(uint8_t gnssMode, uint8_t prn, int32_t dataType, PEReqCommand_t* peReqCom)
{
#if 0
  double            dt;
  sat_data_t* sp = NULL;
  uint8_t             isQzssMode = 0;

  if (prn)
  {
    sp = gnss_sd_get_sv_data(gnssMode, prn, 0);
    if (sp == NULL)
    {
      sp = gnss_sd_get_sv_data(gnssMode, prn, 1);
      if (sp == NULL)
      {
        sp = gnss_sd_get_sv_data(gnssMode, prn, 2);
        if (sp == NULL)
        {
          return;
        }
      }
    }
  }

  if (gnssMode == GPS_MODE)
  {
    if ((prn >= MIN_QZSS_PRN) && (prn <= MAX_QZSS_PRN)) {
      isQzssMode = 1;
    }
    else {
      isQzssMode = 0;
    }
  }

  /* Has Finish one request */
  if (peReqCom->reqFlag[gnssMode][dataType - 1])
  {
    if ((gnssMode == GPS_MODE) && (isQzssMode))
    {
      dt = (double)(p_gnssTime->time_ns - peReqCom->lastRequestTime_Qzss[dataType - 1]) * 1e-9;
    }
    else
    {
      dt = (double)(p_gnssTime->time_ns - peReqCom->lastRequestTime[gnssMode][dataType - 1]) * 1e-9;
    }

    if (gnssMode == GLN_MODE)
    {
      if (dt < -43200.0)
      {
        dt += 86400.0;
      }
    }
    else
    {
      if (dt < -302400.0)
      {
        dt += 604800.0;
      }
    }
    if (fabs(dt) < 20)
    {
      return;
    }

    if (sp != NULL)
    {
      dt = (double)(p_gnssTime->time_ns - sp->lastRequestTime) * 1e-9;
      if ((fabs(dt) < sp->reqEphThresT) && (sp->lastRequestTime > 0))
      {
        if (sp->reqEphFlag)
        {
          if (sp->reqEphThresT < 3600)
          {
            sp->reqEphThresT = sp->reqEphThresT + 300;
          }
          sp->reqEphFlag = FALSE;
        }
        SYS_LOGGING(OBJ_PE, LOG_INFO, "abnormal eph request: %10.4f,%10.4f,%f,%d,%d", (double)p_gnssTime->time_ns * 1e-9, sp->lastRequestTime * 1e-9, sp->reqEphThresT, sp->gnssMode, sp->prn);
        return;
      }

      sp->reqEphFlag = TRUE;
      sp->lastRequestTime = p_gnssTime->time_ns;
    }
  }

  peReqCom->flag = TRUE;
  if ((gnssMode == GPS_MODE) && (isQzssMode))
  {
    peReqCom->reqFlag_Qzss[dataType - 1] = TRUE;
    peReqCom->lastRequestTime_Qzss[dataType - 1] = p_gnssTime->time_ns;
  }
  else
  {
    peReqCom->reqFlag[gnssMode][dataType - 1] = TRUE;
    peReqCom->lastRequestTime[gnssMode][dataType - 1] = p_gnssTime->time_ns;
  }

  SYS_LOGGING(OBJ_PE, LOG_INFO, "%s,%10.4f,%d,%d,%d", __FUNCTION__, (double)p_gnssTime->time_ns * 1e-9, gnssMode, prn, dataType);
  /* set GNSS mode */
  switch (gnssMode)
  {
  case GPS_MODE: {
    if (isQzssMode)
    {
      peReqCom->gnss |= PE_GNSS_QZS; break;
    }
    else
    {
      peReqCom->gnss |= PE_GNSS_GPS; break;
    }
  }
  case GLN_MODE: {peReqCom->gnss |= PE_GNSS_GLO; break; }
  case BDS_MODE: {peReqCom->gnss |= PE_GNSS_BDS; break; }
  case GAL_MODE: {peReqCom->gnss |= PE_GNSS_GAL; break; }
  default:break;
  }

  if (dataType == REQ_EPH)
  {
    peReqCom->reqType |= PE_REQ_REF_TIME;
  }

  /*set Request data type */
  if (gnssMode == GPS_MODE)
  {
    if (isQzssMode)
    {
      switch (dataType)
      {
      case REQ_LOC: {peReqCom->reqType |= PE_REQ_REF_LOC; break; }
      case REQ_TIME: {peReqCom->reqType |= PE_REQ_REF_TIME; break; }
      case REQ_EPH: {peReqCom->reqType |= PE_REQ_QZS_EPH; break; }
      case REQ_ALM: {peReqCom->reqType |= PE_REQ_QZS_ALM; break; }
      case REQ_IONO: {peReqCom->reqType |= PE_REQ_QZS_IONO; break; }
      case REQ_UTC: {peReqCom->reqType |= PE_REQ_QZS_UTC; break; }
      default: break;
      }
    }
    else
    {
      switch (dataType)
      {
      case REQ_LOC: {peReqCom->reqType |= PE_REQ_REF_LOC; break; }
      case REQ_TIME: {peReqCom->reqType |= PE_REQ_REF_TIME; break; }
      case REQ_EPH: {peReqCom->reqType |= PE_REQ_GPS_EPH; break; }
      case REQ_ALM: {peReqCom->reqType |= PE_REQ_GPS_ALM; break; }
      case REQ_IONO: {peReqCom->reqType |= PE_REQ_GPS_IONO; break; }
      case REQ_UTC: {peReqCom->reqType |= PE_REQ_GPS_UTC; break; }
      default: break;
      }
    }

  }
  else if (gnssMode == GLN_MODE)
  {
    switch (dataType)
    {
    case REQ_LOC: {peReqCom->reqType |= PE_REQ_REF_LOC; break; }
    case REQ_TIME: {peReqCom->reqType |= PE_REQ_REF_TIME; break; }
    case REQ_EPH: {peReqCom->reqType |= PE_REQ_GLO_EPH; break; }
    case REQ_ALM: {peReqCom->reqType |= PE_REQ_GLO_ALM; break; }
    case REQ_GLO_AUX: {peReqCom->reqType |= PE_REQ_GLO_AUX; break; }
    case REQ_UTC: {peReqCom->reqType |= PE_REQ_GLO_UTC; break; }
    default: break;
    }
  }
  else if (gnssMode == BDS_MODE)
  {
    switch (dataType)
    {
    case REQ_LOC: {peReqCom->reqType |= PE_REQ_REF_LOC; break; }
    case REQ_TIME: {peReqCom->reqType |= PE_REQ_REF_TIME; break; }
    case REQ_EPH: {peReqCom->reqType |= PE_REQ_BDS_EPH; break; }
    case REQ_ALM: {peReqCom->reqType |= PE_REQ_BDS_ALM; break; }
    case REQ_IONO: {peReqCom->reqType |= PE_REQ_BDS_IONO; break; }
    case REQ_UTC: {peReqCom->reqType |= PE_REQ_BDS_UTC; break; }
    default: break;
    }
  }
  else if (gnssMode == GAL_MODE)
  {
    switch (dataType)
    {
    case REQ_LOC: {peReqCom->reqType |= PE_REQ_REF_LOC; break; }
    case REQ_TIME: {peReqCom->reqType |= PE_REQ_REF_TIME; break; }
    case REQ_EPH: {peReqCom->reqType |= PE_REQ_GAL_EPH; break; }
    case REQ_ALM: {peReqCom->reqType |= PE_REQ_GAL_ALM; break; }
    case REQ_IONO: {peReqCom->reqType |= PE_REQ_GAL_IONO; break; }
    case REQ_UTC: {peReqCom->reqType |= PE_REQ_GAL_UTC; break; }
    default: break;
    }
  }
  else
  {

  }
#endif
}

void gnss_Pe_Full_Agnss_Request()
{
#if 0
  gnss_Pe_Set_ReqCommand(GPS_MODE, 0, REQ_LOC, &peReqCommand);
  gnss_Pe_Set_ReqCommand(GPS_MODE, 0, REQ_TIME, &peReqCommand);
  gnss_Pe_Set_ReqCommand(GPS_MODE, 0, REQ_EPH, &peReqCommand);
  gnss_Pe_Set_ReqCommand(GPS_MODE, 0, REQ_ALM, &peReqCommand);
  gnss_Pe_Set_ReqCommand(GPS_MODE, 0, REQ_IONO, &peReqCommand);
  gnss_Pe_Set_ReqCommand(GPS_MODE, 0, REQ_UTC, &peReqCommand);

  gnss_Pe_Set_ReqCommand(GPS_MODE, MIN_QZSS_PRN, REQ_LOC, &peReqCommand);
  gnss_Pe_Set_ReqCommand(GPS_MODE, MIN_QZSS_PRN, REQ_TIME, &peReqCommand);
  gnss_Pe_Set_ReqCommand(GPS_MODE, MIN_QZSS_PRN, REQ_EPH, &peReqCommand);
  gnss_Pe_Set_ReqCommand(GPS_MODE, MIN_QZSS_PRN, REQ_ALM, &peReqCommand);
  gnss_Pe_Set_ReqCommand(GPS_MODE, MIN_QZSS_PRN, REQ_IONO, &peReqCommand);
  gnss_Pe_Set_ReqCommand(GPS_MODE, MIN_QZSS_PRN, REQ_UTC, &peReqCommand);

  gnss_Pe_Set_ReqCommand(GLN_MODE, 0, REQ_LOC, &peReqCommand);
  gnss_Pe_Set_ReqCommand(GLN_MODE, 0, REQ_TIME, &peReqCommand);
  gnss_Pe_Set_ReqCommand(GLN_MODE, 0, REQ_EPH, &peReqCommand);
  gnss_Pe_Set_ReqCommand(GLN_MODE, 0, REQ_ALM, &peReqCommand);
  gnss_Pe_Set_ReqCommand(GLN_MODE, 0, REQ_GLO_AUX, &peReqCommand);
  gnss_Pe_Set_ReqCommand(GLN_MODE, 0, REQ_UTC, &peReqCommand);

  gnss_Pe_Set_ReqCommand(BDS_MODE, 0, REQ_LOC, &peReqCommand);
  gnss_Pe_Set_ReqCommand(BDS_MODE, 0, REQ_TIME, &peReqCommand);
  gnss_Pe_Set_ReqCommand(BDS_MODE, 0, REQ_EPH, &peReqCommand);
  gnss_Pe_Set_ReqCommand(BDS_MODE, 0, REQ_ALM, &peReqCommand);
  gnss_Pe_Set_ReqCommand(BDS_MODE, 0, REQ_IONO, &peReqCommand);
  gnss_Pe_Set_ReqCommand(BDS_MODE, 0, REQ_UTC, &peReqCommand);

  gnss_Pe_Set_ReqCommand(GAL_MODE, 0, REQ_EPH, &peReqCommand);
  gnss_Pe_Set_ReqCommand(GAL_MODE, 0, REQ_ALM, &peReqCommand);
  gnss_Pe_Set_ReqCommand(GAL_MODE, 0, REQ_IONO, &peReqCommand);
  gnss_Pe_Set_ReqCommand(GAL_MODE, 0, REQ_UTC, &peReqCommand);
#endif
}
static void gnss_Pe_Proc_Ctrl(meas_blk_t* pMeas)
{
  uint8_t                    index = 0, k = 1;

  g_proc.flag = 0xFC00;

  if (g_pe_cfg.meas_rate != MEAS_RATE_2HZ && g_pe_cfg.meas_rate != MEAS_RATE_5HZ && g_pe_cfg.meas_rate != MEAS_RATE_10HZ)
  {
    return;
  }

  g_proc.rtk_status = g_proc.rtk_status << 1;
  g_proc.rtk_status |= (rtkctrl.kfstatus >> 4) & 0x01;

  if (g_pe_cfg.meas_rate == MEAS_RATE_2HZ)
  {
    index = (int)round(pMeas->tor * 2) % 2;
    g_proc.lambda_epoch = 0xFFFE;
    g_proc.hold_epoch = 0xFFFE;
  }
  else if (g_pe_cfg.meas_rate == MEAS_RATE_5HZ)
  {
    index = (int)round(pMeas->tor * 5) % 5;
    g_proc.lambda_epoch = 0xFFE4;
    g_proc.hold_epoch = 0xFFE8;
  }
  else if (g_pe_cfg.meas_rate == MEAS_RATE_10HZ)
  {
    index = (int)round(pMeas->tor * 10) % 10;
    g_proc.lambda_epoch = 0xFC20;
    g_proc.hold_epoch = 0xFC40;
  }

  g_proc.flag = 0x0001 << index;

  if (sumofbit1(g_proc.rtk_status, k) == 0)
  {
    g_proc.flag = 0x0001;
  }
  else if ((rtkctrl.kfcnt <= 10) && (rtkctrl.kfcnt > 0))
  {
    g_proc.flag = 0x0001;
  }
  else if (rtkctrl.kfstatus & RTK_KF_RESET_AMB)
  {
    g_proc.flag = 0x0001;
  }
}
/***********************************************************************
* ��������: gnss_Pe_Pre_Proc
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/02/
***********************************************************************/
static void gnss_Pe_Pre_Proc(meas_blk_t* pMeas, PE_MODES* pMode)
{
  uint8_t        FreqLoopFlag = TRUE;
  uint32_t       i, j;
  int32_t        indx;
  double         dt = 0, dopp_corr = 0.0, tot;
  gnss_meas_t    measTmp, * pSvMeas;
  sat_data_t* sp;
  sat_data_t* sp_L1;
  GNSS_TIME* pTime;
  double         dtClk = 0.0, dClk = 0.0;;
  ECEF           svp;
  dgnss_t* pSvRtd;
  double         range, /*range1,*/ toa;
  double         satPosCorr[3];

  /* Default no need to request any assisted data */
#if 0
  gnss_Pe_Reset_ReqCommand(&peReqCommand);

  //full request check
  if (fullReqFlag == 0 && (pMeas->Cno20Cnt >= 1 || pMeas->measCnt >= 4))
  {
    gnss_Pe_Full_Agnss_Request();
    fullReqFlag = 1;
    SYS_LOGGING(OBJ_PE, LOG_INFO, "the first and only full request, fullReqFlag=%d", fullReqFlag);
  }
#endif

  pTime = gnss_tm_get_time();

  gnss_Qos_avgCno(pMeas);

  /* Set time */
  if (pMeas->avgCno >= 20 && pMeas->measCnt >= 4)
  {
    gnss_Pe_Time_Judge(pTime);
    gnss_Pe_Set_Time(pTime, pMeas->tor);
  }

  /* arrange the data according to cno */
  for (i = 0; i < pMeas->measCnt; i++)
  {
    for (j = i + 1; j < pMeas->measCnt; j++)
    {
      if (pMeas->meas[i].cno < pMeas->meas[j].cno)
      {
        memcpy(&measTmp, &(pMeas->meas[i]), sizeof(gnss_meas_t));
        memcpy(&(pMeas->meas[i]), &(pMeas->meas[j]), sizeof(gnss_meas_t));
        memcpy(&(pMeas->meas[j]), &measTmp, sizeof(gnss_meas_t));
      }
    }
  }

  /* IF ST8090/ST8100, PR and tot error caused by 20ms need to be corrected */
  if (g_pe_cfg.chipType == UBLOX && (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8090 || g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100))
  {
    gnss_Pe_20ms_Jump(pMeas);
  }

  /* If SPRD, then do PR jump detection */
  if (g_pe_cfg.chipType == MTK || g_pe_cfg.chipType == UBLOX)
  {
    /* 1ms bias jump detection */
    gnss_Pe_Bias_Jump(pMeas);
  }

  if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100 || g_pe_cfg.sub_chipType == SUB_CHIPTYPE_HD9310)
  {
    gnss_Pe_Drift_Jump(pMeas);
  }

  /* time propagation */
  gnss_tm_propagate(pMeas->tor, IS_PPK_REVERSE);

#ifdef USED_IN_MC262M
  /* IF QCOM, then calculate PR */
  if ((g_pe_cfg.chipType == QCOM || g_pe_cfg.chipType == SPRD) && pTime->init == TRUE)
  {
    for (i = 0; i < pMeas->measCnt; i++)
    {
      pSvMeas = &(pMeas->meas[i]);
      if (pSvMeas->prn == 0) continue;
      if (pSvMeas->status & 0x1)
      {
        if ((pSvMeas->gnssMode == GPS_MODE || pSvMeas->gnssMode == BDS_MODE || pSvMeas->gnssMode == GAL_MODE) && ((pSvMeas->tot - pTime->rcvr_time[pSvMeas->gnssMode]) > SECS_IN_WEEK / 2.0))
        {
          //pSvMeas->pseudoRange = (pTime->rcvr_time[pSvMeas->gnssMode] - pSvMeas->tot+SECS_IN_WEEK) * LIGHT_SEC;
          pSvMeas->pseudoRange = (SECS_IN_WEEK * 1e9 - pSvMeas->received_sv_time_in_ns) * LIGHT_NSEC;
          pSvMeas->pseudoRange += pTime->rcvr_time_ns[pSvMeas->gnssMode] * LIGHT_NSEC;
        }
        else if (pSvMeas->gnssMode == GLN_MODE && ((pSvMeas->tot - pTime->rcvr_time[pSvMeas->gnssMode]) > SECS_IN_DAY / 2.0))
        {
          //pSvMeas->pseudoRange = (pTime->rcvr_time[pSvMeas->gnssMode] - pSvMeas->tot+SECS_IN_DAY) * LIGHT_SEC;
          pSvMeas->pseudoRange = (SECS_IN_DAY * 1e9 - pSvMeas->received_sv_time_in_ns) * LIGHT_NSEC;
          pSvMeas->pseudoRange += pTime->rcvr_time_ns[pSvMeas->gnssMode] * LIGHT_NSEC;
        }
        else
        {
          //pSvMeas->pseudoRange = (pTime->rcvr_time[pSvMeas->gnssMode] - pSvMeas->tot) * LIGHT_SEC;
          pSvMeas->pseudoRange = (pTime->rcvr_time_ns[pSvMeas->gnssMode] - pSvMeas->received_sv_time_in_ns) * LIGHT_NSEC;
        }
      }
      else
      {
        pSvMeas->pseudoRange = -1.0;
      }
      if (pSvMeas->pseudoRange < 0.0 || pSvMeas->pseudoRange > 0.5 * LIGHT_SEC)
      {
        pSvMeas->status &= 0xF0;//not use PR/DR/CR
      }
      pSvMeas->pseudoRange1 = pSvMeas->pseudoRange;
      pSvMeas->pseudoRange_raw = pSvMeas->pseudoRange;
    }
  }
#endif

  //CNO Mask check
  gnss_Qos_Cno_Mask(pMeas);

  /* Determine if use RTD correction */
  gnss_sd_rtd_glb_chck(pMeas, p_rtcm, pMode);

#ifdef USED_IN_MC262M
  /* Check if MP signal */
  gnss_Qos_MP(pMeas);
#endif

  if (firstFix == FALSE || g_pe_cfg.rtd_usage_flag == 0)
  {
    if (p_rtcm != NULL) {
      pMeas->rtdUsed = FALSE;
      p_rtcm->rtd.RtdUseFlag = FALSE;
      p_rtcm->rtd.RtdEnterCnt = 0;
    }
  }

FREQ_LOOP_TAG:
  /* Add satellite, PR correction */
  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &(pMeas->meas[i]);

    if (g_pe_cfg.meas_rate > MEAS_RATE_1HZ)
    {
      if (FreqLoopFlag == TRUE && pSvMeas->freq_index != 0)
      {
        continue;
      }
      if (FreqLoopFlag == FALSE && pSvMeas->freq_index == 0)
      {
        continue;
      }
    }

    if (pSvMeas->prn == 0 || (pSvMeas->prn >= 93 && pSvMeas->gnssMode == GLN_MODE)) {
      continue;
    }

    /*prn validty check*/
    indx = gnss_sv_Idx(pSvMeas->gnssMode, pSvMeas->prn);
    if (indx < 0)
    {
      pSvMeas->status &= 0xF8;
      SYS_LOGGING(OBJ_PE, LOG_INFO, "Invalid sat prn: %d, %d", pSvMeas->gnssMode, pSvMeas->prn);
      continue;
    }

    // Add new satellite
    gnss_sd_add_sv(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);
    sp = gnss_sd_get_sv_data(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);
    pSvRtd = gnss_sd_get_sv_rtd(pSvMeas->gnssMode, pSvMeas->prn);

    if (sp == NULL)
    {
      pSvMeas->status &= 0xF8;
      pSvMeas->sv_info.eph_status = FALSE;
      SYS_LOGGING(OBJ_PE, LOG_ERROR, "There is no sp for gnssMode(%02d),prn(%03d),freq(%d)", pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);
      continue;
    }
    // save current PR and DR
    if (pSvMeas->pseudoRange > 0.0)
    {
      sp->last_pr = pSvMeas->pseudoRange;
      sp->last_dr = pSvMeas->pseudoRangeRate;
      sp->last_tor = pMeas->tor;
      sp->last_prflag = pSvMeas->prAdjustFlag;
    }

    /* Elevation angle mask */
    if (sp->d_cos.elev > 0.0 && (sp->d_cos.elev * RAD2DEG) < g_pe_cfg.ele_mask)
    {
      pSvMeas->status &= 0xF8;
      SYS_LOGGING(OBJ_PE, LOG_INFO, "Rej %d %d because low ele", pSvMeas->gnssMode, pSvMeas->prn);
      continue;
    }
    // day or week roll-over
    if (pSvMeas->tot < 0.0)
    {
      if (pSvMeas->gnssMode == GLN_MODE)
      {
        pSvMeas->tot += (double)SECS_IN_DAY;
      }
      else
      {
        pSvMeas->tot += (double)SECS_IN_WEEK;
      }
    }
    // calculation satellite position and clock correction
    tot = pSvMeas->tot;    // satellite time
    if (g_pe_cfg.chipType == MTK)
    {
      tot = pTime->rcvr_time[pSvMeas->gnssMode] - pSvMeas->pseudoRange / LIGHT_SEC;
      if (tot < 0.0)
      {
        if (pSvMeas->gnssMode == GLN_MODE)
        {
          tot += (double)SECS_IN_DAY;
        }
        else
        {
          tot += (double)SECS_IN_WEEK;
        }
      }
    }
    svp.have_spos = FRM_NULL;
    if (pTime->init)
    {
      // only when TOW decoded, then calculate satellite position 
      if (pSvMeas->measState & AGGNSS_MEASUREMENT_STATE_TOW_DECODED ||
        pSvMeas->measState & AGGNSS_MEASUREMENT_STATE_GLO_TOD_DECODED)
      {
        sp_L1 = gnss_sd_get_sv_data(pSvMeas->gnssMode, pSvMeas->prn, 0);
        if (sp_L1 != NULL && sp_L1->ecef.have_spos == FRM_EPH && pSvMeas->freq_index > 0 && g_pe_cfg.meas_rate > MEAS_RATE_1HZ)
        {
          if (pSvMeas->gnssMode == GLN_MODE)
          {
            dtClk = -sp_L1->svt[0];
            dopp_corr = -sp_L1->svt[1];
          }
          else
          {
            dtClk = sp_L1->svt[0];
            dopp_corr = sp_L1->svt[1];
          }
          tot = sp_L1->ecef.t;

          dClk = gnss_Sd_Clk_Corr(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);
          dtClk += dClk;
          tot += dClk;

          for (j = 0; j < 3; j++)
          {
            svp.p[j] = sp_L1->ecef.p[j];
            svp.v[j] = sp_L1->ecef.v[j];
          }
          svp.have_spos = FRM_EPH;
        }
        else
        {
          dtClk = gnss_Sd_Clk(pSvMeas->gnssMode, pSvMeas->prn, tot, &dopp_corr, pSvMeas->freq_index);
          if (pSvMeas->gnssMode == GLN_MODE)
          {
            tot += dtClk;
          }
          else
          {
            tot -= dtClk;
          }

          if (g_pe_cfg.ppk_mode)
          {
            dtClk = gnss_Sd_Pos_e1(pSvMeas->gnssMode, pSvMeas->prn, tot, &svp, &dopp_corr, pSvMeas->freq_index);
          }
          else
          {
            dtClk = gnss_Sd_Pos_e(pSvMeas->gnssMode, pSvMeas->prn, tot, &svp, &dopp_corr, pSvMeas->freq_index);
          }
        }

        if (pSvMeas->gnssMode == GLN_MODE)
        {
          sp->svt[0] = -dtClk;
          sp->svt[1] = -dopp_corr;
        }
        else
        {
          sp->svt[0] = dtClk;
          sp->svt[1] = dopp_corr;
        }

        if (svp.have_spos == FRM_EPH)
        {
          // satellite position assignment
          for (j = 0; j < 3; j++)
          {
            pSvMeas->sv_info.p[j] = svp.p[j];
            pSvMeas->sv_info.v[j] = svp.v[j];
            sp->ecef.p[j] = svp.p[j];
            sp->ecef.v[j] = svp.v[j];
          }

          sp->ecef.t = tot;
          sp->ecef.have_spos = svp.have_spos;

          range = gnssClcSqrtAminusB_DBL(gpz_user_pvt->ecef.pos, pSvMeas->sv_info.p, 3);
          //calculate the direction cosine:
          if (range > 0.0)
          {
            toa = range / LIGHT_SEC;
            gnssEarthRotateCorr(pSvMeas->sv_info.p, satPosCorr, toa);
            pSvMeas->range = gnssClcSqrtAminusB_DBL(gpz_user_pvt->ecef.pos, satPosCorr, 3);
            pSvMeas->sv_info.dcos[0] = ((gpz_user_pvt->ecef.pos[0] - satPosCorr[0]) / pSvMeas->range);
            pSvMeas->sv_info.dcos[1] = ((gpz_user_pvt->ecef.pos[1] - satPosCorr[1]) / pSvMeas->range);
            pSvMeas->sv_info.dcos[2] = ((gpz_user_pvt->ecef.pos[2] - satPosCorr[2]) / pSvMeas->range);
          }
          pSvMeas->sv_info.fltElev = (float)(sp->d_cos.elev * RAD2DEG);
          pSvMeas->sv_info.fltAz = (float)(sp->d_cos.az * RAD2DEG);

          pSvMeas->sv_info.eph_status = TRUE;
          // satellite clock bias correction
          if (pSvMeas->gnssMode == GLN_MODE)
          {
            pSvMeas->pseudoRange -= (double)(dtClk * LIGHT_SEC);
            if (g_pe_cfg.meas_type_mask & MEAS_TYPE_MASK_DR)
            {
              pSvMeas->pseudoRangeRate -= dopp_corr * LIGHT_SEC;
            }
          }
          else
          {
            pSvMeas->pseudoRange += (double)(dtClk * LIGHT_SEC);
            if (g_pe_cfg.meas_type_mask & MEAS_TYPE_MASK_DR)
            {
              pSvMeas->pseudoRangeRate += dopp_corr * LIGHT_SEC;
            }
          }

          //DCB correction
          /*
          if (pSvMeas->freq_index == 1)
          {
              pSvMeas->pseudoRange += DEFAULT_DCB_TABLE[g_pe_cfg.sub_chipType][pSvMeas->gnssMode];
          }
          if (pSvMeas->freq_index == 2)
          {
              pSvMeas->pseudoRange += DEFAULT_DCB_TABLE[g_pe_cfg.sub_chipType][pSvMeas->gnssMode + 4];
          }
          */
          // save PR  
          gnss_sd_save_meas(sp, pSvMeas, pMeas->tor, pMeas->msec_adj[pSvMeas->gnssMode], pMeas->drift_adj);
          gnss_Qos_DRSmoothPR(pSvMeas);

#ifdef USED_IN_MC262M
          pSvMeas->pseudoRange1 = pSvMeas->pseudoRange;
          pSvMeas->pseudoRange1 -= (sp->mcorr.iono_corr + sp->mcorr.trop_corr);
          pSvMeas->smoothedPR -= (sp->mcorr.iono_corr + sp->mcorr.trop_corr);
#endif
          if (g_pe_cfg.meas_type_mask & MEAS_TYPE_MASK_DR)
          {
            pSvMeas->pseudoRangeRate += sp->mcorr.erot_corr_dot;
          }
          // IONO and TROP correction
          if (pMeas->rtdUsed == FALSE)
          {
            /* result may be changed */
            pSvMeas->pseudoRange -= ((double)sp->mcorr.iono_corr + sp->mcorr.trop_corr);
          }
          else
          {
            if ((pSvMeas->status & 0x4) == 0 || (pSvRtd == NULL))
            {
              pSvMeas->status &= 0xFE;
              SYS_LOGGING(OBJ_PE, LOG_INFO, "Disable satellite(%02d,%02d) without rtd data when using RTD", pSvMeas->gnssMode, pSvMeas->prn);
            }
            else
            {
              if (pSvMeas->gnssMode == GPS_MODE)
              {
                dt = timediff(pTime->GPSTime, pSvRtd->time);
              }
              else if (pSvMeas->gnssMode == GLN_MODE)
              {
                dt = timediff(pTime->GLNTime, pSvRtd->time);
              }
              else if (pSvMeas->gnssMode == BDS_MODE)
              {
                dt = timediff(pTime->BDSTime, pSvRtd->time);
              }

              if (dt > SECS_IN_WEEK / 2.0) dt -= (double)SECS_IN_WEEK;
              else if (dt < -SECS_IN_WEEK / 2.0) dt += (double)SECS_IN_WEEK;

              if (fabs(dt) > RTD_AGE)
              {
                pSvMeas->status &= 0xFE;
                SYS_LOGGING(OBJ_PE, LOG_INFO, "Disable satellite(%02d,%02d) when PRC is old", pSvMeas->gnssMode, pSvMeas->prn);
              }
              else
              {
                pSvMeas->pseudoRange += (pSvRtd->prc + 0 * pSvRtd->rrc * dt);
                SYS_LOGGING(OBJ_PE, LOG_INFO, "Use RTD: %8.4f,%02d,%02d,%10.5f", pTime->rcvr_time[0], pSvMeas->gnssMode, pSvMeas->prn, pSvRtd->prc);
              }
            }
          }
        }
        else
        {
          pSvMeas->status &= 0xF8;
          pSvMeas->sv_info.eph_status = FALSE;
          sp->ecef.have_spos = svp.have_spos;
          SYS_LOGGING(OBJ_PE, LOG_INFO, "There is no SVP for gnss(%02d) prn(%02d)", pSvMeas->gnssMode, pSvMeas->prn);
        }
        /*
        1. If can't calculate SV position, if no EPH or expired EPH, then apply EPH request
        2. If can't calculate SV position, if there is valid EPH, then apply EPH request
        */
        if (gnss_Sd_Nm_Check_NoEph(pSvMeas->gnssMode, pSvMeas->prn) == TRUE)
        {
          gnss_Pe_Set_ReqCommand(pSvMeas->gnssMode, pSvMeas->prn, REQ_EPH, &peReqCommand);
          if (pSvMeas->gnssMode == GLN_MODE)
          {
            gnss_Pe_Set_ReqCommand(GLN_MODE, pSvMeas->prn, REQ_GLO_AUX, &peReqCommand);
          }
        }
      }
#if 0
      GLOGW("sv check: mode %d prn %03d f %d, P %.3f %.3f %.3f, V %.3f %.3f %.3f, tot %.10f dt %.3f",
        pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index, pSvMeas->sv_info.p[0], pSvMeas->sv_info.p[1],
        pSvMeas->sv_info.p[2], pSvMeas->sv_info.v[0], pSvMeas->sv_info.v[1], pSvMeas->sv_info.v[2], tot, dtClk * CLIGHT);
#endif
    }
    else
    {
      // Before time init, if there is no EPH or expired EPH, then apply EPH request
      if (gnss_Sd_Nm_Check_NoEph(pSvMeas->gnssMode, pSvMeas->prn))
      {
        gnss_Pe_Set_ReqCommand(pSvMeas->gnssMode, pSvMeas->prn, REQ_EPH, &peReqCommand);
      }
      pSvMeas->status &= 0xF8;
      pSvMeas->sv_info.eph_status = FALSE;
      SYS_LOGGING(OBJ_PE, LOG_INFO, "There is no time for gnss(%02d) prn(%02d)", pSvMeas->gnssMode, pSvMeas->prn);
    }
  }

  if (g_pe_cfg.meas_rate > MEAS_RATE_1HZ)
  {
    if (FreqLoopFlag == TRUE)
    {
      FreqLoopFlag = FALSE;
      goto FREQ_LOOP_TAG;
    }
  }

  if (g_pe_cfg.chipType == QCOM)
  {
    gnss_Pe_Time_Jump(pMeas);
  }

  /* MP reject strategy with average cno check */
  gnss_Pe_PRDR_Num(pMeas);
#ifdef USED_IN_MC262M
  if (pMeas->avgCno > 10)
  {
    for (i = 0; i < pMeas->measCnt; i++)
    {
      pSvMeas = &(pMeas->meas[i]);
      if (g_pe_cfg.chipType == SPRD)
      {
        if (pSvMeas->mpIndicator == 1 && pSvMeas->usedInChip == FALSE)
        {
          pSvMeas->status &= 0xFE;
          SYS_LOGGING(OBJ_PE, LOG_INFO, "MP Reject %02d %03d", pSvMeas->gnssMode, pSvMeas->prn);
        }
      }
      else if (g_pe_cfg.chipType == QCOM)
      {

      }
    }
  }
#endif
}

/***********************************************************************
* ��������: gnss_Pe_Time_Jump
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�02/18/
***********************************************************************/
void gnss_Pe_Time_Jump(meas_blk_t* pMeas)
{
  uint8_t            flag;
  int8_t             index[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM];
  uint32_t           i, j = 0, k;
  double             torDelta;
  double             totDelta[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, totDeltaAbs[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, totDeltaStd = 0.0;
  double             totDeltaSum = 0.0, median, rejectK = 20.0;
  gnss_meas_t*       pSvMeas;
  sat_data_t*        sp;
  GNSS_TIME*         pTime;

  torDelta = pMeas->tor - pMeas->last_tor;
  pTime = gnss_tm_get_time();
  pTime->time_ns_bias = 0.0;
  pTime->isLargeTimeJmp = FALSE;

  if (peContext.epochCnt < 2) return;
  memset(index, -1, sizeof(int8_t) * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM);
  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &(pMeas->meas[i]);
    if (pSvMeas->prn == 0 || (pSvMeas->status & 0x1) == 0) continue;
    sp = gnss_sd_get_sv_data(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);
    if (sp == NULL || sp->measBackCnt < 2) continue;
    if (sp->measBack[1].tor == pMeas->last_tor && (pMeas->tor == sp->measBack[0].tor))
    {
      totDelta[j] = sp->measBack[0].tot - sp->measBack[1].tot;
      if (totDelta[j] < -SECS_IN_WEEK / (2.0))
      {
        totDelta[j] = totDelta[j] + (double)SECS_IN_WEEK;
      }
      else if (totDelta[j] > (double)SECS_IN_WEEK / (2.0))
      {
        totDelta[j] = totDelta[j] - (double)SECS_IN_WEEK;
      }

      index[j] = 1;
      j++;
    }
  }
  if (j == 0)return;

  flag = gnss_MAD_DBL(totDelta, totDeltaAbs, j, &median);
  if (flag == FALSE) return;

  k = j;
  /* reject out-lies according to MAD */
  for (i = 0; i < j; i++)
  {
    if (totDeltaAbs[i] > rejectK * (median / 0.6745))
    {
      index[i] = -1;
      k--;
    }
  }
  /* calculate sum */
  for (i = 0; i < j; i++)
  {
    if (index[i] == -1) continue;
    totDeltaSum += totDelta[i];
  }
  if (k > 0)
  {
    totDeltaSum /= k;
    for (i = 0; i < j; i++)
    {
      if (index[i] == -1) continue;
      totDeltaStd += (totDelta[i] - totDeltaSum) * (totDelta[i] - totDeltaSum);
    }
    totDeltaStd = sqrt(totDeltaStd / k);
    if (firstFix && fabs(pTime->bias[GPS_MODE] * (double)1.0 / LIGHT_MSEC) < 0.5)
    {
      if (fabs(torDelta - totDeltaSum) > 1e-5)
      {
        //pTime->time_ns_bias = totDeltaSum - torDelta;
        pTime->isLargeTimeJmp = TRUE;
        SYS_LOGGING(OBJ_PE, LOG_INFO, "Abnormal Time_Ns Jump:%10.7f", totDeltaSum - torDelta);
      }
    }
    SYS_LOGGING(OBJ_PE, LOG_INFO, "Time_ns Monitor:%14.6f,%14.6f,%14.6f,%.3f", torDelta, totDeltaSum, fabs(torDelta - totDeltaSum), totDeltaStd);
  }

}
/***********************************************************************
* ��������: gnss_Pe_Localtow_JumpMs
* ���������
* ���������
* ����ֵ��
* ���ߣ�none
* ���ڣ�12/11/
***********************************************************************/
static double gnss_Pe_Localtow_JumpMs(meas_blk_t* pMeas, GNSS_TIME* pTime)
{
  double dt = 0.0, jump = 0.0;

  dt = pMeas->tor - pMeas->last_tor;
  /*forward filter mode*/
  if (!IS_PPK_REVERSE)
  {
    if (dt < -SECS_IN_WEEK / 2) //cross week
    {
      dt = dt + (double)SECS_IN_WEEK;
    }
    if (dt < 0.0 || dt > 30.0)
    {
      return 0.0;
    }
  }
  else
  {
    if (dt > SECS_IN_WEEK / 2) //cross week
    {
      dt = dt - (double)SECS_IN_WEEK;
    }
    if (dt > 0.0 || dt < -30.0)
    {
      return 0.0;
    }
  }

  jump = dt - pTime->dt;

  if (fabs(jump * LIGHT_SEC) < 0.01 * LIGHT_MSEC)
  {
    return 0.0;
  }

  return -jump * 1000.0;
}
/***********************************************************************
* ��������: gnss_Pe_BiasMsec_offset
* ���������
* ���������
* ����ֵ��
* ���ߣ�none
* ���ڣ�12/11/
***********************************************************************/
static void gnss_Pe_BiasMsec_offset(GNSS_TIME* pTime)
{
  int32_t satMode = 0;

  if (pTime->msec_correction[GPS_MODE] != 0.0)
  {
    for (satMode = 0; satMode < GNSS_MAX_MODE; satMode++)
    {
      if (GPS_MODE == satMode) continue;
      if (pTime->bias[satMode] != 0.0 && (pTime->msec_correction[satMode] == 0.0 ||
        fabs(pTime->msec_correction[GPS_MODE] - pTime->msec_correction[satMode]) > 0.01))
      {
        pTime->msec_correction[satMode] = pTime->msec_correction[GPS_MODE];
      }
    }
  }
  else if (pTime->msec_correction[BDS_MODE] != 0.0)
  {
    for (satMode = 0; satMode < GNSS_MAX_MODE; satMode++)
    {
      if (BDS_MODE == satMode) continue;
      if (pTime->bias[satMode] != 0.0 && (pTime->msec_correction[satMode] == 0.0 ||
        fabs(pTime->msec_correction[BDS_MODE] - pTime->msec_correction[satMode]) > 0.01))
      {
        pTime->msec_correction[satMode] = pTime->msec_correction[BDS_MODE];
      }
    }
  }
  else if (pTime->msec_correction[GLN_MODE] != 0.0)
  {
    for (satMode = 0; satMode < GNSS_MAX_MODE; satMode++)
    {
      if (GLN_MODE == satMode) continue;
      if (pTime->bias[satMode] != 0.0 && (pTime->msec_correction[satMode] == 0.0 ||
        fabs(pTime->msec_correction[GLN_MODE] - pTime->msec_correction[satMode]) > 0.01))
      {
        pTime->msec_correction[satMode] = pTime->msec_correction[GLN_MODE];
      }
    }
  }
  else if (pTime->msec_correction[GAL_MODE] != 0.0)
  {
    for (satMode = 0; satMode < GNSS_MAX_MODE; satMode++)
    {
      if (GAL_MODE == satMode) continue;
      if (pTime->bias[satMode] != 0.0 && (pTime->msec_correction[satMode] == 0.0 ||
        fabs(pTime->msec_correction[GAL_MODE] - pTime->msec_correction[satMode]) > 0.01))
      {
        pTime->msec_correction[satMode] = pTime->msec_correction[GAL_MODE];
      }
    }
  }
}
void gnss_Pe_20ms_Jump(meas_blk_t* pMeas)
{
  uint32_t         i = 0;
  double           pos[3] = { 0.0 }, detPos[3] = { 0.0 }, calRange = 0.0, pr_diff = 0.0, bitNum = 0.0, lambda = 0.0;
  int32_t          delta_week = 0, sat = 0;
  double           delta_t = 0;
  //gtime_t        time;
  GNSS_TIME*       pTime;
  gnss_meas_t*     pSvMeas;
  sat_data_t*      sp;

  pTime = gnss_tm_get_time();

  if (pTime->init == FALSE)
  {
    goto Lab_checkbias;
  }

  if (gpz_user_pvt->have_position > FIX_SOURCE_INIT)
  {
    delta_t = pTime->rcvr_time[GPS_MODE] - gpz_user_pvt->posfix_t;
    delta_week = (int32_t)(pTime->week[GPS_MODE] - gpz_user_pvt->posfix_wn);
    delta_t += (double)SECS_IN_WEEK * delta_week;

    pos[0] = gpz_user_pvt->ecef.pos[0];
    pos[1] = gpz_user_pvt->ecef.pos[1];
    pos[2] = gpz_user_pvt->ecef.pos[2];
  }
  else
  {
    goto Lab_checkbias;
  }


  if (fabs(delta_t) < POSITION_HOLD_TIME)
  {
    for (i = 0; i < pMeas->measCnt; i++)
    {
      pSvMeas = &(pMeas->meas[i]);
      if (fabs(pSvMeas->pseudoRange) < 1e-3) continue;
      sp = gnss_sd_get_sv_data(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);
      if (sp == NULL || sp->ecef.have_spos == FRM_NULL) continue;

      detPos[0] = sp->ecef.p[0] - pos[0];
      detPos[1] = sp->ecef.p[1] - pos[1];
      detPos[2] = sp->ecef.p[2] - pos[2];
      calRange = gnssClcSqrtSum_DBL(detPos, 3);

      pr_diff = -(float)(calRange - pSvMeas->pseudoRange);
      if (fabs(pr_diff) > CLIGHT * 0.01)
      {
        bitNum = pr_diff / CLIGHT / 0.02;
        sat = satno(GNSS_MODE2SYS(pSvMeas->gnssMode, pSvMeas->prn), pSvMeas->prn);
        lambda = satwavelen(sat, pSvMeas->freq_index, NULL);
        if (fabs(round(bitNum) - bitNum) < 0.1 || (round(bitNum) > 1 && fabs(round(bitNum) - bitNum) < 0.3))
        {
          pSvMeas->pseudoRange -= round(bitNum) * CLIGHT * 0.02;
          pSvMeas->pseudoRange_raw = pSvMeas->pseudoRange;
          pSvMeas->tot += round(bitNum) * 0.02;
          if (DBL_ISNT_EQUAL(lambda, 0.0) && DBL_ISNT_EQUAL(pSvMeas->carrierPhase, 0.0) && round((pSvMeas->carrierPhase * lambda - pSvMeas->pseudoRange_raw) / (CLIGHT * 0.02)) == round(bitNum))
          {
            pSvMeas->carrierPhase -= round(bitNum) * CLIGHT * 0.02 / lambda;
          }
          pSvMeas->prAdjustFlag = 1;
          GLOGI("ST 20ms offset check: tor %.3f, gnssMode %d  prn %03d  fidx %d, nbit=%.6f  round_err_ms=%.6f, phase %.3f",
            pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index, bitNum, (round(bitNum) - bitNum) * 20,
            round((pSvMeas->carrierPhase * lambda - pSvMeas->pseudoRange_raw) / (CLIGHT * 0.02)));
        }
      }
    }
  }

Lab_checkbias:
  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &(pMeas->meas[i]);
    /* Wrong PR detected */
    if (pSvMeas->pseudoRange < 0.0 || DBL_IS_EQUAL(pSvMeas->pseudoRange, 0.0) || pSvMeas->pseudoRange>1.0E9)
    {
      pSvMeas->status &= 0xF0;
    }
  }
}
/***********************************************************************
* ��������: gnss_Pe_Bias_Jump
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�10/13/
***********************************************************************/
void gnss_Pe_Bias_Jump(meas_blk_t* pMeas)
{
  uint8_t                flag = 0, madflag = 0, msec_adjustcnt = 0, adjustcnt = 0;
  uint8_t                idx, mscorrect_cnt[GNSS_MAX_MODE] = { 0 };
  uint8_t                cnt, satcheck_cnt;
  uint8_t                satMode;
  uint32_t               i;
  gnss_meas_t* pSvMeas;
  GNSS_TIME* pTime;
  sat_data_t* sp;
  double               avg;
  double               dt, timeLimit = 1.5, timeLimit1 = 4.5;
  double               det_range[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM];
  double               std_range, avgdt;
  double               temp_detrange, temp_dr, bitNum;
  pTime = gnss_tm_get_time();

  if (!pTime || pTime->init == FALSE) return;

  memcpy(pTime->last_bias, pTime->bias, GNSS_MAX_MODE * sizeof(double));

  if (gpz_Ls->ls_Pvt_Info->fix_status != FIX_STATUS_NEW && p_Kf->kf_Pvt_Info->kfFixStatus != FIX_STATUS_NEW)
  {
    return;
  }

  if (peMode.tunnelMode.mode == TRUE)
  {
    timeLimit = 50;
  }
  for (satMode = 0; satMode < GNSS_MAX_MODE; satMode++)
  {
    pTime->msec_correction[satMode] = 0;
    avg = 0;
    cnt = 0;
    satcheck_cnt = 0;
    //std_range = 0;
    avgdt = 0;
    madflag = 0;

    for (i = 0; i < pMeas->measCnt; i++)
    {
      pSvMeas = &pMeas->meas[i];
      if (pSvMeas->gnssMode != satMode || pSvMeas->pseudoRange < 0.0)
      {
        continue;
      }
      sp = gnss_sd_get_sv_data(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);
      if (sp == NULL) continue;
      if (fabs(sp->last_pr) < 1e-3) continue;
      if (pSvMeas->prAdjustFlag == 1 && sp->last_prflag == 0) continue;
      dt = pMeas->tor - sp->last_tor;

      /* time gap validity check*/
      if (!IS_PPK_REVERSE && (dt <= 0.0 || dt > timeLimit)) continue;
      if (IS_PPK_REVERSE && (dt >= 0.0 || dt < -timeLimit)) continue;

      if (cnt > 0 && fabs(dt) > avgdt * 5 / cnt) continue;
      avgdt += fabs(dt);
      /*20ms check again*/
      temp_detrange = pSvMeas->pseudoRange - (sp->last_pr + pSvMeas->pseudoRangeRate * dt);
      bitNum = temp_detrange / CLIGHT / 0.02;
      if (fabs(round(bitNum) - bitNum) < 0.1 && fabs(bitNum) > 1.0) continue;

      det_range[cnt] = temp_detrange;
      avg += det_range[cnt++];
    }
    mscorrect_cnt[satMode] = cnt;

  Lab_checkbias:
    if (cnt >= 1)
    {
      /*add protection only one satellite,cno constraint*/
      if (cnt == 1 && pMeas->measCnt == 1 && pMeas->meas[0].cno < 30) continue;

      /*when one sat,check bias adjust*/
      if (cnt == 1 && pMeas->measCnt > 1)
      {
        for (i = 0; i < pMeas->measCnt; i++)
        {
          pSvMeas = &pMeas->meas[i];
          if (pSvMeas->gnssMode != satMode || pSvMeas->pseudoRange < 0.0)
          {
            continue;
          }
          sp = gnss_sd_get_sv_data(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);
          if (sp == NULL) continue;
          if (fabs(sp->last_pr) < 1e-3) continue;
          dt = pMeas->tor - sp->last_tor;
          if (dt > timeLimit && dt < timeLimit1 && (!DBL_IS_EQUAL(pSvMeas->pseudoRangeRate, 0.0) || !DBL_IS_EQUAL(sp->last_dr, 0.0)))
          {
            temp_detrange = pSvMeas->pseudoRange - sp->last_pr;
            temp_dr = DBL_IS_EQUAL(pSvMeas->pseudoRangeRate, 0.0) ? sp->last_dr : pSvMeas->pseudoRangeRate;
            bitNum = temp_detrange / (temp_dr * dt);
            if (fabs(round(bitNum) - bitNum) < 0.2)
            {
              satcheck_cnt++;
            }
          }
        }
        if (satcheck_cnt >= cnt)
        {
          SYS_LOGGING(OBJ_PE, LOG_INFO, "Only One Sat Bias Check Fail");
          continue;
        }
      }
      avg /= cnt;
      std_range = 0.0;
      for (i = 0; i < cnt; i++)
      {
        std_range += (det_range[i] - avg) * (det_range[i] - avg);
      }
      std_range = sqrt(std_range / cnt);
      if (std_range < 2e3)
      {
        if (avg > (double)(0.995 * LIGHT_MSEC) && avg < (double)(1.005 * LIGHT_MSEC))
        {
          pTime->msec_correction[satMode] = 1.0;
          SYS_LOGGING(OBJ_PE, LOG_INFO, "gnssMode(%02d) has +1ms tor jump,avg:%-12.4f,std:%-12.4f", satMode, avg, std_range);
        }
        else if (avg > -(double)(1.005 * LIGHT_MSEC) && avg < -(double)(0.995 * LIGHT_MSEC))
        {
          pTime->msec_correction[satMode] = -1.0;
          SYS_LOGGING(OBJ_PE, LOG_INFO, "gnssMode(%02d) has -1ms tor jump,avg:%-12.4f,std:%-12.4f", satMode, avg, std_range);
        }
        else if (fabs(avg) > 0.01 * LIGHT_MSEC)
        {
          pTime->msec_correction[satMode] = avg / LIGHT_MSEC;
        }
        else if ((g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100 || g_pe_cfg.sub_chipType == SUB_CHIPTYPE_HD9310) && (fabs(avg) <= 0.01 * LIGHT_MSEC))
        {
          if (fabs(avg) > 30.0 && fabs(std_range) < 10.0 && DBL_ISNT_EQUAL(std_range, 0.0))
          {
            pTime->msec_correction[satMode] = avg / LIGHT_MSEC;
          }
          else if (fabs(avg) > 10.0 && fabs(std_range) < 5.0 && DBL_ISNT_EQUAL(std_range, 0.0))
          {
            pTime->msec_correction[satMode] = avg / LIGHT_MSEC;
          }
        }
      }
      else if (madflag == 0)
      {
        double medianout, dataabs[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, K1 = 0.0;
        uint8_t cnt1 = 0;
        GLOGW("Ab-normal range difference std:gnssMode(%02d),avg(%-12.4f),std(%-12.4f),%s", satMode, avg, std_range, __FUNCTION__);
        //screen outliers
        gnss_MAD_DBL(det_range, dataabs, cnt, &medianout);
        K1 = 0.5 * LIGHT_MSEC * (medianout / 0.6745);
        avg = 0.0;
        for (i = 0; i < cnt; i++)
        {
          if (dataabs[i] > K1)
          {
            GLOGW("Ab-normal delta range: %.3f,%s", det_range[i], __FUNCTION__);
            continue;
          }
          avg += det_range[i];
          det_range[cnt1++] = det_range[i];
        }
        cnt = cnt1;
        madflag = 1;
        goto Lab_checkbias;
      }
    }

  }

  if (g_pe_cfg.chipType == UBLOX && g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100)
  {
    gnss_Pe_BiasMsec_offset(pTime);
  }
  else if (g_pe_cfg.chipType == UBLOX && g_pe_cfg.sub_chipType == SUB_CHIPTYPE_HD9310)
  {
    for (satMode = 0; satMode < GNSS_MAX_MODE; satMode++)
    {
      if (fabs(pTime->msec_correction[satMode]) > 0.0 && mscorrect_cnt[satMode] > 3)
      {
        adjustcnt++;
      }
      if (fabs(pTime->msec_correction[satMode]) >= 1.0)
      {
        msec_adjustcnt++;
      }
    }
    if (msec_adjustcnt > 0 || adjustcnt >= 2)
    {
      gnss_Pe_BiasMsec_offset(pTime);
    }
  }

  /*if (GNSS_MAX_MODE >= 4)*/ {
    GLOGI("bias jump detect: %.6f %.6f %.6f %.6f bias: %.3f %.3f %.3f %.3f mscorrect_cnt:%d %d %d %d",
      pTime->msec_correction[0], pTime->msec_correction[1], pTime->msec_correction[2], pTime->msec_correction[3],
      pTime->bias[0], pTime->bias[1], pTime->bias[2], pTime->bias[3],
      mscorrect_cnt[0], mscorrect_cnt[1], mscorrect_cnt[2], mscorrect_cnt[3]);
  }

  if (pTime->init && g_pe_cfg.chipType != UBLOX)
  {
    /* TOR calculation when bias jump happened */

    if (pTime->msec_correction[GPS_MODE] == 0.0) {
      // GLONASS
      if (pTime->msec_correction[GLN_MODE] == 1.0)
      {
        pTime->rcvr_time[GLN_MODE] += BIAS_ADJUST_TIME;
        SYS_LOGGING(OBJ_PE, LOG_INFO, "GPS had no jump,GLN had +1ms jump");
      }
      else if (pTime->msec_correction[GLN_MODE] == -1.0)
      {
        pTime->rcvr_time[GLN_MODE] -= BIAS_ADJUST_TIME;
        SYS_LOGGING(OBJ_PE, LOG_INFO, "GPS had no jump,GLN had -1ms jump");
      }
      else if (pTime->msec_correction[GLN_MODE] == 0.0)
      {
        SYS_LOGGING(OBJ_PE, LOG_INFO, "GPS had no jump,GLN had no jump");
      }

    }
    else if (pTime->msec_correction[GPS_MODE] == 1.0)
    {
      // GLONASS
      if (pTime->msec_correction[GLN_MODE] == 0.0)
      {
        pTime->rcvr_time[GLN_MODE] -= BIAS_ADJUST_TIME;
        SYS_LOGGING(OBJ_PE, LOG_INFO, "GPS had +1ms jump,GLN had no jump");
      }
      else if (pTime->msec_correction[GLN_MODE] == 1.0)
      {
        SYS_LOGGING(OBJ_PE, LOG_INFO, "GPS and GLN both had +1ms jump");
      }
      else if (pTime->msec_correction[GLN_MODE] == -1.0)
      {
        SYS_LOGGING(OBJ_PE, LOG_INFO, "GPS had +1ms jump, GLN had -1ms jump");
      }

    }
    else if (pTime->msec_correction[GPS_MODE] == -1.0)
    {
      // GLONASS
      if (pTime->msec_correction[GLN_MODE] == 0.0)
      {
        pTime->rcvr_time[GLN_MODE] += BIAS_ADJUST_TIME;
        SYS_LOGGING(OBJ_PE, LOG_INFO, "GPS had -1ms jump,GLN had no jump");
      }
      else if (pTime->msec_correction[GLN_MODE] == -1.0)
      {
        SYS_LOGGING(OBJ_PE, LOG_INFO, "GPS and GLN both had -1ms jump");
      }
      else if (pTime->msec_correction[GLN_MODE] == 1.0)
      {
        SYS_LOGGING(OBJ_PE, LOG_INFO, "GPS had -1ms jump, GLN had +1ms jump");
      }
    }
  }

  if (g_pe_cfg.chipType != UBLOX) //for ublox type, correct ms_corr for each system
  {

    if (pTime->msec_correction[GPS_MODE] != 0.0 && fabs(pTime->msec_correction[GPS_MODE]) != 1e-3)
    {
      if (IS_MEAS_GLN_USED((&g_pe_cfg)))
      {
        pTime->msec_correction[GLN_MODE] = pTime->msec_correction[GPS_MODE];
      }
      if (IS_MEAS_BDS_USED((&g_pe_cfg)))
      {
        pTime->msec_correction[BDS_MODE] = pTime->msec_correction[GPS_MODE];
      }
    }
    else if (pTime->msec_correction[GLN_MODE] != 0.0 && fabs(pTime->msec_correction[GLN_MODE]) != 1.0 && IS_MEAS_GPS_USED((&g_pe_cfg)))
    {
      pTime->msec_correction[GPS_MODE] = pTime->msec_correction[GLN_MODE];
    }
    else if (pTime->msec_correction[BDS_MODE] != 0.0 && fabs(pTime->msec_correction[BDS_MODE]) != 1.0 && IS_MEAS_GPS_USED((&g_pe_cfg)))
    {
      pTime->msec_correction[GPS_MODE] = pTime->msec_correction[BDS_MODE];
    }

  }

  for (satMode = 0; satMode < GNSS_MAX_MODE; satMode++)
  {
    if (pTime->msec_correction[satMode] == 0.0)
    {
      continue;
    }

    // TM bias adjust
    pTime->bias[satMode] += pTime->msec_correction[satMode] * (double)(BIAS_ADJUST_TIME * LIGHT_SEC);

    // LS bias adjust
    idx = gpz_Ls->lsCtrl.biasIndx[satMode];
    if (idx > 0)
    {
      gpz_Ls->bias[idx - 1] += pTime->msec_correction[satMode] * (double)(BIAS_ADJUST_TIME * LIGHT_SEC);
      SYS_LOGGING(OBJ_PE, LOG_INFO, "LS bias adjust for gnssMode(%02d), bias idx(%02d),bias value(%6.4f)ms",
        satMode, idx, pTime->msec_correction[satMode]);
    }
    else
    {
      gpz_Ls->bias[satMode] += pTime->msec_correction[satMode] * (double)(BIAS_ADJUST_TIME * LIGHT_SEC);
      SYS_LOGGING(OBJ_PE, LOG_INFO, "LS bias adjust for gnssMode(%02d), bias idx(%02d),bias value(%6.4f)ms",
        satMode, idx, pTime->msec_correction[satMode]);
    }


    // KF bias adjust when KF is running
    if (kfInfo.status & KF_RUN)
    {
      idx = clockBiasIdx[satMode][p_Kf->kf_ctrl.biasLinkFlag];
      kfInfo.X[satMode + 6] += pTime->msec_correction[idx - 6] * (double)(BIAS_ADJUST_TIME * LIGHT_SEC);
      gpz_kf_pvt_data->clkBias_last[satMode] += (float)(pTime->msec_correction[satMode] * (double)(BIAS_ADJUST_TIME * LIGHT_SEC));
      SYS_LOGGING(OBJ_PE, LOG_INFO, "KF bias adjust for gnssMode(%02d), bias idx(%02d),bias value(%6.4f)ms",
        satMode, idx, pTime->msec_correction[satMode]);
    }

    // clear saved DR and PR
    flag = 1;
  }

  if (flag)
  {
    memcpy(pMeas->msec_adj, pTime->msec_correction, GNSS_MAX_MODE * sizeof(double));
  }

  if (flag && g_pe_cfg.chipType != UBLOX)
  {
    gnss_sd_clear_meas(p_Sd);
  }

}

void gnss_Pe_Drift_Jump(meas_blk_t* pMeas)
{
  uint8_t                flag = 0, madflag = 0;
  //uint8_t                idx;
  uint8_t                cnt;
  uint8_t                cnt1 = 0;
  uint32_t               i;
  gnss_meas_t* pSvMeas;
  GNSS_TIME* pTime;
  sat_data_t* sp;
  double               avg, avg_threshold = 50.0;
  double               dt, timeLimit = 1.5;
  double               det_range[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM];
  double               std_range, avgdt;
  double               medianout, dataabs[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, K1 = 0.0;

  pTime = gnss_tm_get_time();

  if (!pTime || pTime->init == FALSE) return;
  if (gpz_Ls->ls_Pvt_Info->fix_status != FIX_STATUS_NEW && p_Kf->kf_Pvt_Info->kfFixStatus != FIX_STATUS_NEW)
  {
    return;
  }

  if (peMode.tunnelMode.mode == TRUE)
  {
    timeLimit = 50;
  }

  if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100)
  {
    avg_threshold = 10.0;
  }
  if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_HD9310)
  {
    avg_threshold = 5.0;
  }

  avg = 0;
  cnt = 0;
  //std_range = 0;
  avgdt = 0;
  madflag = 0;
  pTime->drift_correction = 0;

  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &pMeas->meas[i];
    if (fabs(pSvMeas->pseudoRangeRate) < 1e-6)
    {
      continue;
    }
    sp = gnss_sd_get_sv_data(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);
    if (sp == NULL) continue;
    if (fabs(sp->last_dr) < 1e-6) continue;
    dt = pMeas->tor - sp->last_tor;

    /* time gap validity check*/
    if (!IS_PPK_REVERSE && (dt <= 0.0 || dt > timeLimit)) continue;
    if (IS_PPK_REVERSE && (dt >= 0.0 || dt < -timeLimit)) continue;

    if (cnt > 0 && fabs(dt) > avgdt * 5 / cnt) continue;
    avgdt += fabs(dt);
    det_range[cnt] = pSvMeas->pseudoRangeRate - sp->last_dr;
    avg += det_range[cnt++];
  }

Lab_checkbias:
  if (cnt > 1)
  {
    avg /= cnt;
    std_range = 0.0;
    for (i = 0; i < cnt; i++)
    {
      std_range += (det_range[i] - avg) * (det_range[i] - avg);
    }
    std_range = sqrt(std_range / cnt);
    if ((fabs(avg) > avg_threshold) && (std_range < 1))
    {
      pTime->drift_correction = avg;
      SYS_LOGGING(OBJ_PE, LOG_INFO, "drift has jump,avg:%-12.4f,std:%-12.4f", avg, std_range);
    }
    else if ((fabs(avg) > 10 * avg_threshold) && (std_range < 2))
    {
      pTime->drift_correction = avg;
      SYS_LOGGING(OBJ_PE, LOG_INFO, "drift has jump,avg:%-12.4f,std:%-12.4f", avg, std_range);
    }
    else if ((std_range > 5) && (madflag == 0))
    {
      GLOGW("Ab-normal drift range difference std:avg(%-12.4f),std(%-12.4f),%s", avg, std_range, __FUNCTION__);
      //screen outliers
      gnss_MAD_DBL(det_range, dataabs, cnt, &medianout);
      K1 = 10 * (medianout / 0.6745);
      avg = 0.0;
      for (i = 0; i < cnt; i++)
      {
        if (dataabs[i] > K1)
        {
          GLOGW("Ab-normal drift delta range: %.3f,%s", det_range[i], __FUNCTION__);
          continue;
        }
        avg += det_range[i];
        det_range[cnt1++] = det_range[i];
      }
      cnt = cnt1;
      madflag = 1;
      goto Lab_checkbias;
    }
  }

  pTime->drift += (float)pTime->drift_correction;
  pMeas->drift_adj = pTime->drift_correction;
  gpz_Ls->drift += (float)pTime->drift_correction;

  // KF bias adjust when KF is running
  if (kfInfo.status & KF_RUN)
  {
    kfInfo.X[10] += pTime->drift_correction;
    gpz_kf_pvt_data->clkDrift_last += (float)pTime->drift_correction;
    SYS_LOGGING(OBJ_PE, LOG_INFO, "KF drift adjust,drift value(%6.4f)", pTime->drift_correction);
  }
}

/***********************************************************************
* ��������: gnss_Pe_Set_HW_Bias
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�03/14/
***********************************************************************/
void gnss_Pe_Set_HW_Bias(GNSS_TIMESYSPARAM* pTimeParam)
{
  if (pTimeParam == NULL)
  {
    if (g_pe_cfg.automobile == 0)
    {
      if (g_pe_cfg.chipType == QCOM)
      {
        if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_BRCM_47755)
        {
          p_Kf->biasDiff[GLN_MODE] = DEFAULT_GPSGLN_HW_BIAS_BRCOM;
          p_Kf->biasDiff[BDS_MODE] = DEFAULT_GPSBDS_HW_BIAS_BRCOM;
          p_Kf->biasDiff[GAL_MODE] = DEFAULT_GPSGAL_HW_BIAS_BRCOM;
          p_Kf->biasDiffLocal[GLN_MODE] = DEFAULT_GPSGLN_HW_BIAS_BRCOM;
          p_Kf->biasDiffLocal[BDS_MODE] = DEFAULT_GPSBDS_HW_BIAS_BRCOM;
          p_Kf->biasDiffLocal[GAL_MODE] = DEFAULT_GPSGAL_HW_BIAS_BRCOM;
        }
        else
        {
          p_Kf->biasDiff[GLN_MODE] = DEFAULT_GPSGLN_HW_BIAS;
          p_Kf->biasDiff[BDS_MODE] = DEFAULT_GPSBDS_HW_BIAS;
          p_Kf->biasDiffLocal[GLN_MODE] = DEFAULT_GPSGLN_HW_BIAS;
          p_Kf->biasDiffLocal[BDS_MODE] = DEFAULT_GPSBDS_HW_BIAS;
        }
      }

    }
    else
    {
      if (g_pe_cfg.chipType == QCOM)
      {
        if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_BRCM_47755)
        {
          p_Kf->biasDiff[GLN_MODE] = DEFAULT_GPSGLN_HW_BIAS_BRCOM;
          p_Kf->biasDiff[BDS_MODE] = DEFAULT_GPSBDS_HW_BIAS_BRCOM;
          p_Kf->biasDiff[GAL_MODE] = DEFAULT_GPSGAL_HW_BIAS_BRCOM;
          p_Kf->biasDiffLocal[GLN_MODE] = DEFAULT_GPSGLN_HW_BIAS_BRCOM;
          p_Kf->biasDiffLocal[BDS_MODE] = DEFAULT_GPSBDS_HW_BIAS_BRCOM;
          p_Kf->biasDiffLocal[GAL_MODE] = DEFAULT_GPSGAL_HW_BIAS_BRCOM;
        }
        else
        {
          p_Kf->biasDiff[GLN_MODE] = DEFAULT_GPSGLN_HW_BIAS_AUTO;
          p_Kf->biasDiffLocal[GLN_MODE] = DEFAULT_GPSGLN_HW_BIAS_AUTO;
        }
      }
      else if (g_pe_cfg.chipType == MTK)
      {
        p_Kf->biasDiff[BDS_MODE] = DEFAULT_GPSBDS_HW_BIAS_AUTO_MTK;
        p_Kf->biasDiffLocal[BDS_MODE] = DEFAULT_GPSBDS_HW_BIAS_AUTO_MTK;
      }
      else if (g_pe_cfg.chipType == UBLOX && g_pe_cfg.sub_chipType == SUB_CHIPTYPE_UBLOX_F9)
      {
        p_Kf->biasDiff[BDS_MODE] = DEFAULT_GPSBDS_HW_BIAS_AUTO_F9;
        p_Kf->biasDiffLocal[BDS_MODE] = DEFAULT_GPSBDS_HW_BIAS_AUTO_F9;
        p_Kf->biasDiff[GLN_MODE] = DEFAULT_GPSGLN_HW_BIAS_AUTO_F9;
        p_Kf->biasDiffLocal[GLN_MODE] = DEFAULT_GPSGLN_HW_BIAS_AUTO_F9;
        p_Kf->biasDiff[GAL_MODE] = DEFAULT_GPSGAL_HW_BIAS_AUTO_F9;
        p_Kf->biasDiffLocal[GAL_MODE] = DEFAULT_GPSGAL_HW_BIAS_AUTO_F9;
      }
      else if (g_pe_cfg.chipType == UBLOX && g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100)
      {
        p_Kf->biasDiff[BDS_MODE] = DEFAULT_GPSBDS_HW_BIAS_AUTO_ST8100;
        p_Kf->biasDiffLocal[BDS_MODE] = DEFAULT_GPSBDS_HW_BIAS_AUTO_ST8100;
        p_Kf->biasDiff[GAL_MODE] = DEFAULT_GPSGAL_HW_BIAS_AUTO_ST8100;
        p_Kf->biasDiffLocal[GAL_MODE] = DEFAULT_GPSGAL_HW_BIAS_AUTO_ST8100;
      }
    }

    memcpy(p_Kf->biasDiffInit, p_Kf->biasDiff, GNSS_MAX_MODE * sizeof(double));
    memcpy(p_Kf->biasDiffLocalInit, p_Kf->biasDiffLocal, GNSS_MAX_MODE * sizeof(double));

    return;
  }

  // RTD bias load
  if (pTimeParam->hwBias[GLN_MODE] != 0.0)
  {
    p_Kf->biasDiff[GLN_MODE] = pTimeParam->hwBias[GLN_MODE];
  }
  else
  {
    if (g_pe_cfg.automobile == 0)
    {
      if (g_pe_cfg.chipType == QCOM)
      {
        if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_BRCM_47755)
        {
          p_Kf->biasDiff[GLN_MODE] = DEFAULT_GPSGLN_HW_BIAS_BRCOM;
        }
        else
        {
          p_Kf->biasDiff[GLN_MODE] = DEFAULT_GPSGLN_HW_BIAS;
        }
      }
    }
    else
    {
      if (g_pe_cfg.chipType == QCOM)
      {
        if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_BRCM_47755)
        {
          p_Kf->biasDiff[GLN_MODE] = DEFAULT_GPSGLN_HW_BIAS_BRCOM;
        }
        else
        {
          p_Kf->biasDiff[GLN_MODE] = DEFAULT_GPSGLN_HW_BIAS_AUTO;
        }
      }
    }

    p_Kf->biasDiffInit[GLN_MODE] = p_Kf->biasDiff[GLN_MODE];
  }
  if (pTimeParam->hwBias[BDS_MODE] != 0.0)
  {
    p_Kf->biasDiff[BDS_MODE] = pTimeParam->hwBias[BDS_MODE];
  }
  else
  {
    if (g_pe_cfg.automobile == 0)
    {
      if (g_pe_cfg.chipType == QCOM)
      {
        if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_BRCM_47755)
        {
          p_Kf->biasDiff[BDS_MODE] = DEFAULT_GPSBDS_HW_BIAS_BRCOM;
        }
        else
        {
          p_Kf->biasDiff[BDS_MODE] = DEFAULT_GPSBDS_HW_BIAS;
        }
      }
    }
    else
    {
      if (g_pe_cfg.chipType == MTK)
      {
        p_Kf->biasDiff[BDS_MODE] = DEFAULT_GPSBDS_HW_BIAS_AUTO_MTK;
      }
    }

    p_Kf->biasDiffInit[BDS_MODE] = p_Kf->biasDiff[BDS_MODE];
  }
  if (pTimeParam->hwBias[GAL_MODE] != 0.0)
  {
    p_Kf->biasDiff[GAL_MODE] = pTimeParam->hwBias[GAL_MODE];
  }
  else
  {
    if (g_pe_cfg.automobile == 0)
    {
      if (g_pe_cfg.chipType == QCOM && g_pe_cfg.sub_chipType == SUB_CHIPTYPE_BRCM_47755)
      {
        p_Kf->biasDiff[GAL_MODE] = DEFAULT_GPSGAL_HW_BIAS_BRCOM;
      }
    }

    p_Kf->biasDiffInit[GAL_MODE] = p_Kf->biasDiff[GAL_MODE];
  }

  // SPP bias load
  if (pTimeParam->hwBiasLocal[GLN_MODE] != 0.0)
  {
    p_Kf->biasDiffLocal[GLN_MODE] = pTimeParam->hwBiasLocal[GLN_MODE];
  }
  else
  {
    if (g_pe_cfg.automobile == 0)
    {
      if (g_pe_cfg.chipType == QCOM)
      {
        if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_BRCM_47755)
        {
          p_Kf->biasDiffLocal[GLN_MODE] = DEFAULT_GPSGLN_HW_BIAS_BRCOM;
        }
        else
        {
          p_Kf->biasDiffLocal[GLN_MODE] = DEFAULT_GPSGLN_HW_BIAS;
        }
      }
    }
    else
    {
      if (g_pe_cfg.chipType == QCOM)
      {
        if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_BRCM_47755)
        {
          p_Kf->biasDiffLocal[GLN_MODE] = DEFAULT_GPSGLN_HW_BIAS_BRCOM;
        }
        else
        {
          p_Kf->biasDiffLocal[GLN_MODE] = DEFAULT_GPSGLN_HW_BIAS_AUTO;
        }
      }
    }

    p_Kf->biasDiffLocalInit[GLN_MODE] = p_Kf->biasDiffLocal[GLN_MODE];
  }
  if (pTimeParam->hwBiasLocal[BDS_MODE] != 0.0)
  {
    p_Kf->biasDiffLocal[BDS_MODE] = pTimeParam->hwBiasLocal[BDS_MODE];
  }
  else
  {
    if (g_pe_cfg.automobile == 0)
    {
      if (g_pe_cfg.chipType == QCOM)
      {
        if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_BRCM_47755)
        {
          p_Kf->biasDiffLocal[BDS_MODE] = DEFAULT_GPSBDS_HW_BIAS_BRCOM;
        }
        else
        {
          p_Kf->biasDiffLocal[BDS_MODE] = DEFAULT_GPSBDS_HW_BIAS;
        }
      }
    }
    else
    {
      if (g_pe_cfg.chipType == MTK)
      {
        p_Kf->biasDiffLocal[BDS_MODE] = DEFAULT_GPSBDS_HW_BIAS_AUTO_MTK;
      }
    }

    p_Kf->biasDiffLocalInit[BDS_MODE] = p_Kf->biasDiffLocal[BDS_MODE];
  }

  if (pTimeParam->hwBiasLocal[GAL_MODE] != 0.0)
  {
    p_Kf->biasDiffLocal[GAL_MODE] = pTimeParam->hwBiasLocal[GAL_MODE];
  }
  else
  {
    if (g_pe_cfg.automobile == 0)
    {
      if (g_pe_cfg.chipType == QCOM && g_pe_cfg.sub_chipType == SUB_CHIPTYPE_BRCM_47755)
      {
        p_Kf->biasDiffLocal[GAL_MODE] = DEFAULT_GPSGAL_HW_BIAS_BRCOM;
      }
    }

    p_Kf->biasDiffLocalInit[GAL_MODE] = p_Kf->biasDiffLocal[GAL_MODE];
  }
}

/***********************************************************************
* ��������: gnss_Pe_Set_Bias
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/21/
***********************************************************************/
void gnss_Pe_Set_Bias(Ls_t* pLs, Kf_t* pKf)
{
  uint32_t 		   i, j;
  KF_PVT_INFO* kfPvtInfo;
  LS_PVT_INFO* lsPvtInfo;
  GNSS_TIME* pTime;
  double 		   bias_diff;
  meas_blk_t* pMeas;

  pTime = gnss_tm_get_time();
  pMeas = pKf->meas_blk;

  kfPvtInfo = pKf->kf_Pvt_Info;
  lsPvtInfo = pKf->ls_Pvt_Info;


  /* If KF is new, then update LS bias and drift, including uncertainty */
  if (kfPvtInfo->kfFixStatus == FIX_STATUS_NEW)
  {
    pLs->drift = kfPvtInfo->clkDrift;
    pLs->drift_unc = kfPvtInfo->kfErrEst.drift;
    // Time Module update
    for (i = 0; i < BIAS_NUM; i++)
    {
      if (kfPvtInfo->biasValid[i])
      {
        pTime->bias[i] = kfPvtInfo->clkBias[i];
        pTime->bias_unc[i] = kfPvtInfo->kfErrEst.bias[i];
        pTime->timeStatus[i] = TM_STATUS_ACCU;
        pLs->bias_unc[i] = kfPvtInfo->kfErrEst.bias[i];
        pLs->bias[i] = kfPvtInfo->clkBias[i];
      }
      if (kfPvtInfo->biasValid[i] == 0 && g_pe_cfg.chipType == UBLOX && (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100 || g_pe_cfg.sub_chipType == SUB_CHIPTYPE_HD9310))
      {
        if (DBL_IS_EQUAL(pTime->last_bias[i], 0.0)) continue;
        pTime->bias[i] = kfPvtInfo->clkBias[i];
      }
    }
    gnss_Pe_DCB_Smooth(pLs, pKf);
    pTime->drift = kfPvtInfo->clkDrift;
    pTime->drift_unc = kfPvtInfo->kfErrEst.drift;
    pTime->driftStatus = DRIFT_FIX;
    pTime->biasSrc = BIAS_KF;
  }
  else if (lsPvtInfo->fix_status == FIX_STATUS_NEW)
  {
    pLs->drift_unc = (float)(pLs->DoP.tDOP / LIGHT_SEC * 3);

    // Time Module update
    for (i = 0; i < BIAS_NUM; i++)
    {
      if (lsPvtInfo->biasValid[i] == 1)
      {
        pTime->bias[i] = lsPvtInfo->clkBias[i];
        pTime->bias_unc[i] = lsPvtInfo->err.tErr;
        pTime->timeStatus[i] = TM_STATUS_ACCU;
        pLs->bias_unc[i] = pLs->err_est.tErr;
      }

      if ((gpz_Ls->pos_res < 8.0) && (pMeas->avgCno > 30) && (gpz_Ls->meas->measCnt > 15))
      {
        if ((lsPvtInfo->dcbValid[i] == 1) && (fabs(lsPvtInfo->clkDcb[i] - DEFAULT_DCB_TABLE[g_pe_cfg.sub_chipType][i]) < 20))
        {
          pTime->dcb[i] = lsPvtInfo->clkDcb[i];
        }

        if ((lsPvtInfo->dcbValid[i + GNSS_MAX_MODE] == 1) && (fabs(lsPvtInfo->clkDcb[i + GNSS_MAX_MODE] - DEFAULT_DCB_TABLE[g_pe_cfg.sub_chipType][i + GNSS_MAX_MODE]) < 20))
        {
          pTime->dcb[i + GNSS_MAX_MODE] = lsPvtInfo->clkDcb[i + GNSS_MAX_MODE];
        }
      }

      /*use biasValid sys to adjust biasNonValid sys*/
      if (lsPvtInfo->biasValid[i] == 0 && ((g_pe_cfg.chipType == UBLOX && (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8090 ||
        g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100 || g_pe_cfg.sub_chipType == SUB_CHIPTYPE_HD9310)) || (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_QCOM_855)))
      {
        //bias_diff = 0.0;
        for (j = 0; j < BIAS_NUM; j++)
        {
          if (i == j) continue;
          if (DBL_IS_EQUAL(pTime->last_bias[i], 0.0))
          {
            /*GPS bias valid*/
            if (lsPvtInfo->biasValid[j] == 1 && j == GPS_MODE)
            {
              pTime->bias[i] = pTime->bias[j] - pKf->biasDiff[i];
              break;
            }
          }
          else
          {
            if (DBL_IS_EQUAL(pTime->last_bias[j], 0.0) || DBL_IS_EQUAL(pTime->bias[j], 0.0)) continue;
            if (lsPvtInfo->biasValid[j] == 1)
            {
              bias_diff = (pTime->last_bias[j] - lsPvtInfo->clkBias[j]);
              if (fabs(bias_diff) > 30.0)
              {
                pTime->bias[i] -= bias_diff / LIGHT_MSEC * (double)(BIAS_ADJUST_TIME * LIGHT_SEC);
                break;
              }
            }
          }
        }
      }
    }
    pTime->drift = lsPvtInfo->clkDrift;
    pTime->drift_unc = pLs->drift_unc;
    if (lsPvtInfo->isDriftValid)
    {
      pTime->driftStatus = DRIFT_FIX;
    }
    pTime->biasSrc = BIAS_LS;
  }
  else
  {
    pTime->biasSrc = BIAS_NONE;
  }

  // save bias difference between different satellite systems with GPS as reference
  p_gnssTimeSysParam->hwBias[GLN_MODE] = p_Kf->biasDiff[GLN_MODE];
  p_gnssTimeSysParam->hwBias[BDS_MODE] = p_Kf->biasDiff[BDS_MODE];
  p_gnssTimeSysParam->hwBias[GAL_MODE] = p_Kf->biasDiff[GAL_MODE];

  p_gnssTimeSysParam->hwBiasLocal[GLN_MODE] = p_Kf->biasDiffLocal[GLN_MODE];
  p_gnssTimeSysParam->hwBiasLocal[BDS_MODE] = p_Kf->biasDiffLocal[BDS_MODE];
  p_gnssTimeSysParam->hwBiasLocal[GAL_MODE] = p_Kf->biasDiffLocal[GAL_MODE];
  SYS_LOGGING(OBJ_PE, LOG_INFO, "GPS bias unc:%f,Gln bias unc:%f, BDS bias unc:%f", pTime->bias_unc[0], pTime->bias_unc[1], pTime->bias_unc[2]);
  SYS_LOGGING(OBJ_PE, LOG_INFO, "ptime_dcb:%14.6f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f", kfPvtInfo->tor,
    pTime->dcb[0], pTime->dcb[1], pTime->dcb[2], pTime->dcb[3], pTime->dcb[4], pTime->dcb[5], pTime->dcb[6], pTime->dcb[7]);
}


/***********************************************************************
* ��������: gnss_Pe_BiasNum
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�10/02/
***********************************************************************/
void gnss_Pe_BiasNum(meas_blk_t* pMeas)
{
  uint32_t 			i;
  gnss_meas_t* pSvMeas;

  pMeas->satModeCnt = 0;
  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &pMeas->meas[i];
    if (pSvMeas->prn == 0) continue;
    if ((pSvMeas->status & 0x1) == 0) continue;

    switch (pSvMeas->gnssMode)
    {
    case GPS_MODE:
    {
      pMeas->satModeCnt |= 0x1;
      if (pSvMeas->freq_index == 1)
      {
        pMeas->satModeCnt |= 0x10;
      }
      else if (pSvMeas->freq_index == 2)
      {
        pMeas->satModeCnt |= 0x100;
      }
      break;
    }
    case GLN_MODE:
    {
      pMeas->satModeCnt |= 0x2;
      if (pSvMeas->freq_index == 1)
      {
        pMeas->satModeCnt |= 0x20;
      }
      else if (pSvMeas->freq_index == 2)
      {
        pMeas->satModeCnt |= 0x200;
      }
      break;
    }
    case BDS_MODE:
    {
      pMeas->satModeCnt |= 0x4;
      if (pSvMeas->freq_index == 1)
      {
        pMeas->satModeCnt |= 0x40;
      }
      else if (pSvMeas->freq_index == 2)
      {
        pMeas->satModeCnt |= 0x400;
      }
      break;
    }
    case GAL_MODE:
    {
      pMeas->satModeCnt |= 0x8;
      if (pSvMeas->freq_index == 1)
      {
        pMeas->satModeCnt |= 0x80;
      }
      else if (pSvMeas->freq_index == 2)
      {
        pMeas->satModeCnt |= 0x800;
      }
      break;
    }
    default: {break; }
    }
  }

  pMeas->biasNum = 0;
  for (i = 0; i < BIAS_NUM; i++)
  {
    pMeas->biasNum += (pMeas->satModeCnt >> i) & 0x1;
    pMeas->biasIndx[i] = (pMeas->biasNum) * ((pMeas->satModeCnt >> i) & 0x1);
    if (pMeas->fre2_satcnt[i] <= DCB_SAT_LIMIT && ((pMeas->satModeCnt >> (i + 4)) & 0x1))
    {
      pMeas->satModeCnt = (pMeas->satModeCnt & (~(1 << (i + 4))));
    }
    if (pMeas->fre2_satcnt[i + GNSS_MAX_MODE] <= DCB_SAT_LIMIT && ((pMeas->satModeCnt >> (i + 8)) & 0x1))
    {
      pMeas->satModeCnt = (pMeas->satModeCnt & (~(1 << (i + 8))));
    }
  }
}


/***********************************************************************
* ��������: gnss_Pe_PRDR_Num
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�10/02/
***********************************************************************/
void gnss_Pe_PRDR_Num(meas_blk_t* pMeas)
{
  uint32_t         i;
  int32_t         idx;

  pMeas->validPrNum = 0;
  pMeas->validDrNum = 0;
  pMeas->validSatnum = 0;
  pMeas->trckedSatNum = 0;
  memset(pMeas->prNumEachSystem, 0, sizeof(uint8_t) * GNSS_MAX_MODE);
  for (i = 0; i < MAX_PRN_ALL_MODE; i++)
  {
    pMeas->satmask[i] = 0;
  }
  uint8_t sattrck[MAX_PRN_ALL_MODE] = {0};
  for (i = 0; i < pMeas->measCnt; i++)
  {
    if (pMeas->meas[i].prn == 0)
    {
      continue;
    }
    idx = gnss_sv_Idx(pMeas->meas[i].gnssMode, pMeas->meas[i].prn);
    if (idx > 0)
    {
      sattrck[idx]++;
    }
    if (pMeas->meas[i].status & 0x1)
    {
      if (idx < 0) continue;

      if (pMeas->meas[i].freq_index == 0)
      {
        pMeas->satmask[idx] |= 0x1;
      }
      if (pMeas->meas[i].freq_index == 1)
      {
        pMeas->satmask[idx] |= 0x2;
      }
      if (pMeas->meas[i].freq_index == 2)
      {
        pMeas->satmask[idx] |= 0x4;
      }
      pMeas->prNumEachSystem[pMeas->meas[i].gnssMode]++;
      pMeas->validPrNum++;
    }
    if (pMeas->meas[i].status & 0x2) pMeas->validDrNum++;
  }
  for (i = 0; i < MAX_PRN_ALL_MODE; i++)
  {
    if (sattrck[i] != 0)
    {
      pMeas->trckedSatNum++;
    }
    if (pMeas->satmask[i] & 0x7)
      pMeas->validSatnum++;
  }
}


/***********************************************************************
* ��������: gnss_Pe_Dop
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�10/02/
***********************************************************************/
void gnss_Pe_Dop(meas_blk_t* pMeas, DOP_TYPE* dop, uint8_t flag)
{
  uint8_t				 /*row, */colume;
  uint32_t 			 i, j, k, cnt = 0;
  USER_PVT* user = NULL;
  float 			 TDOP = 0, PDOP = 0, DOPS[3];
  float 			 Ecef2lla[7][7];
  float 			 ceg[3][3];
  double 			 detPos[3], range;
  gnss_meas_t* pSvMeas;

  float* HMatrix = (float*)Sys_Malloc(sizeof(float) * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * LS_PARA_NUM);
  float* invR = (float*)Sys_Malloc(sizeof(float) * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * LS_PARA_NUM);
  float* DoPMatrix = (float*)Sys_Malloc(sizeof(float) * LS_PARA_NUM * LS_PARA_NUM);

  if ((HMatrix == NULL) || (invR == NULL) || (DoPMatrix == NULL))
  {
    Sys_Free(HMatrix);
    Sys_Free(invR);
    Sys_Free(DoPMatrix);
    return;
  }

  memset(HMatrix, 0, sizeof(float) * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * LS_PARA_NUM);
  memset(invR, 0, sizeof(float) * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * LS_PARA_NUM);
  memset(DoPMatrix, 0, sizeof(float) * LS_PARA_NUM * LS_PARA_NUM);

  // memset(&user,0,sizeof(USER_PVT));
  // gnss_Pe_Get_PosFix(&user);
  user = gnss_Pe_Get_User_Pvt();
  if (user->have_position >= HAVE_POS_OLD || user->have_position < HAVE_POS_APPX)
  {
    dop->hDOP = 0;
    dop->pDOP = 0;
    dop->tDOP = 0;
    dop->vDOP = 0;
    Sys_Free(HMatrix); Sys_Free(invR); Sys_Free(DoPMatrix);
    return;
  }
  // 1. calculate biasNum and biasIndx
  gnss_Pe_BiasNum(pMeas);

  //row = pMeas->measCnt;

  /*
  flag = 1  --> PR
  flag = 2  --> DR
  */
  if (flag == 1)
  {
    colume = 3 + pMeas->biasNum;
  }
  else
  {
    colume = 4;
  }
  // 2. calculate H matrix
  memset(HMatrix, 0, sizeof(float) * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * LS_PARA_NUM);
  for (i = 0; i < pMeas->measCnt; i++)
  {
    if ((pMeas->meas[i].status & flag) == 0)
    {
      continue;
    }
    pSvMeas = &pMeas->meas[i];

    /* calculate direction cosine */
    detPos[0] = pSvMeas->sv_info.p[0] - user->ecef.pos[0];
    detPos[1] = pSvMeas->sv_info.p[1] - user->ecef.pos[1];
    detPos[2] = pSvMeas->sv_info.p[2] - user->ecef.pos[2];
    range = gnssClcSqrtSum_DBL(detPos, 3);

    HMatrix[cnt * LS_PARA_NUM + 0] = (float)(detPos[0] / range);
    HMatrix[cnt * LS_PARA_NUM + 1] = (float)(detPos[1] / range);
    HMatrix[cnt * LS_PARA_NUM + 2] = (float)(detPos[2] / range);
    if (flag == 1)
    {
      HMatrix[cnt * LS_PARA_NUM + 2 + pMeas->biasIndx[pSvMeas->gnssMode]] = 1.0;
    }
    else
    {
      HMatrix[cnt * LS_PARA_NUM + 3] = 1.0;
    }

    cnt++;
  }

  // 3. QR Factorize
  if (!gnss_QR_Factorize((float(*)[15])HMatrix, cnt, colume))
  {
    dop->hDOP = 0;
    dop->vDOP = 0;
    dop->pDOP = 0;
    dop->tDOP = 0;
    Sys_Free(HMatrix); Sys_Free(invR); Sys_Free(DoPMatrix);
    return;
  }

  // 4. Inverse matrix of R
  memset(invR, 0, sizeof(float) * LS_PARA_NUM * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM);
  gnss_Inverse_UpMatrix((float(*)[15])HMatrix, (float(*)[15])invR, colume);

  // 5. DOP Matrix calculation
  memset(DoPMatrix, 0, sizeof(float) * LS_PARA_NUM * LS_PARA_NUM);
  gnss_UpMatrix_Square((float(*)[15])invR, (float(*)[15])DoPMatrix, colume);

  /* 6. Calculate DOPS */
  if (flag == 1)
  {
    for (i = 0; i < pMeas->biasNum; i++)
    {
      TDOP += DoPMatrix[(3 + i) * LS_PARA_NUM + 3 + i];
    }
  }
  else
  {
    TDOP = DoPMatrix[3 * LS_PARA_NUM + 3];
  }
  PDOP = DoPMatrix[0 * LS_PARA_NUM + 0] + DoPMatrix[1 * LS_PARA_NUM + 1] + DoPMatrix[2 * LS_PARA_NUM + 2];
  if (TDOP < 0.0 || PDOP < 0.0)
  {
    SYS_LOGGING(OBJ_LS, LOG_ERROR, "Wrong DOP calculation %s(%d)", __FUNCTION__, __LINE__);
    dop->hDOP = 0;
    dop->vDOP = 0;
    dop->pDOP = 0;
    dop->tDOP = 0;
    Sys_Free(HMatrix); Sys_Free(invR); Sys_Free(DoPMatrix);
    return;
  }
  else
  {
    dop->tDOP = (float)sqrt(TDOP);
    dop->pDOP = (float)sqrt(PDOP);
  }
  /* 7. Get the transition matrix */
  gnssGetEcef2EnuMatrix(ceg, user->lla.pos);
  memset(Ecef2lla, 0, sizeof(float) * 7 * 7);
  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      Ecef2lla[i][j] = ceg[i][j];
    }
  }
  for (i = 0; i < pMeas->biasNum; i++)
  {
    Ecef2lla[3 + i][3 + i] = 1;
  }
  for (i = 0; i < 3; i++)
  {
    DOPS[i] = 0;
    for (j = 0; j < 3; j++)
    {
      for (k = 0; k < 3; k++)
      {
        DOPS[i] += Ecef2lla[i][j] * DoPMatrix[j * LS_PARA_NUM + k] * Ecef2lla[i][k];
      }
    }
  }
  dop->hDOP = (float)sqrtf(DOPS[0] + DOPS[1]);
  dop->vDOP = (float)sqrtf(DOPS[2]);

  Sys_Free(HMatrix); Sys_Free(invR); Sys_Free(DoPMatrix);
}


/***********************************************************************
* ��������: gnss_Pe_Dop_Check
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�03/12/
***********************************************************************/
uint8_t gnss_Pe_Dop_Check(meas_blk_t* pMeas, uint8_t idx, uint8_t flag)
{
  uint8_t            status;
  DOP_TYPE      dop1, dop2;

  if (g_pe_cfg.meas_rate > MEAS_RATE_1HZ && (pMeas->Cno35Cnt[GPS_MODE] + pMeas->Cno35Cnt[BDS_MODE]) >= 12 && pMeas->avgCno >= 35 && (pMeas->prDiffStd_1 > 0.0 && pMeas->prDiffStd_1 <= 5.0)
      && peMode.userSceData.isUnderEleRoad == FALSE && gpz_user_pvt->meas_use_info.smallPrDiffRate > 0.85
      && (rtkctrl.sol.qflag & (QFLAG_PVT_PRRES_GOOD | QFLAG_PVT_KFPOS_GOOD)) == (QFLAG_PVT_PRRES_GOOD | QFLAG_PVT_KFPOS_GOOD))
  {
    return TRUE;
  }
  else if (pMeas->measCnt > 24 && (pMeas->Cno35Cnt[GPS_MODE] + pMeas->Cno35Cnt[BDS_MODE]) >= 12 && pMeas->avgCno >= 34
           && peMode.userSceData.isUnderEleRoad == FALSE && gpz_user_pvt->meas_use_info.smallPrDiffRate > 0.80
           && (rtkctrl.sol.qflag & (QFLAG_PVT_PRRES_GOOD | QFLAG_PVT_KFPOS_GOOD)) == (QFLAG_PVT_PRRES_GOOD | QFLAG_PVT_KFPOS_GOOD))
  {
    return TRUE;
  }

  gnss_Pe_Dop(pMeas, &dop1, flag);
  status = pMeas->meas[idx].status;
  pMeas->meas[idx].status &= 0xF8;
  gnss_Pe_Dop(pMeas, &dop2, flag);
  pMeas->meas[idx].status = status;

  if (dop2.pDOP == 0.0 || (dop2.pDOP > 4.0 && (dop2.pDOP / dop1.pDOP) > 1.40) || (dop2.pDOP / dop1.pDOP) > 1.80)
  {
    return FALSE;
  }

  return TRUE;
}
/***********************************************************************
* ��������: gnss_Pe_Init_HisInfo
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�05/11/
***********************************************************************/
void gnss_Pe_Init_HisInfo(const history_Info* p)
{
  if (p)
  {
    if (!g_pe_cfg.is_playback) {
      HisInfo = (*p);
    }
    else
    {
      if (triaCnt == 1)
      {
        HisInfo = (*p);
      }
    }
  }
}
/***********************************************************************
* ��������: gnss_Pe_Init_UsrPos
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�04/12/
***********************************************************************/
void gnss_Pe_Init_UsrPos(const USER_PVT* p)
{
  if (p)
  {
    if (!g_pe_cfg.is_playback)
    {
      memcpy(gpz_user_pvt, p, sizeof(USER_PVT));
      gpz_user_pvt->have_position = HAVE_POS_INIT;
      gpz_user_pvt->ecef.have_position = POS_FRM_NULL;
      gpz_user_pvt->lla.have_position = POS_FRM_NULL;

      memcpy(&user_pvt_backup, gpz_user_pvt, sizeof(USER_PVT));
    }
    else
    {
      if (triaCnt == 1)
      {
        memcpy(gpz_user_pvt, p, sizeof(USER_PVT));
        gpz_user_pvt->have_position = HAVE_POS_INIT;
        gpz_user_pvt->ecef.have_position = POS_FRM_NULL;
        gpz_user_pvt->lla.have_position = POS_FRM_NULL;

        memcpy(&user_pvt_backup, gpz_user_pvt, sizeof(USER_PVT));
      }
    }
  }
}

meas_blk_t* gnss_Pe_Get_Meas(void)
{
  return gpz_Meas;
}

/***********************************************************************
* ��������: gnss_Pe_Get_PosFix ������ȡ��λ���
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/21/
***********************************************************************/
void gnss_Pe_Get_PosFix(USER_PVT* pvt)
{
  memcpy(pvt, gpz_user_pvt, sizeof(USER_PVT));
}

USER_PVT* gnss_Pe_Get_User_Pvt()
{
  return gpz_user_pvt;
}
/***********************************************************************
* ��������: gnss_Pe_Get_IllegalArea_Status ������ȡ��λ�����ж�
* ���������
* ���������flag: 0--China Local; 1- out side of China Local
* ����ֵ��
* ���ߣ�
* ���ڣ�09/21/
***********************************************************************/
void gnss_Pe_Get_RtdIllegalArea_Status(uint8_t* flag)
{
  if (flag)
  {
    *flag = illegalArea;
  }
}

void gnss_Pe_Get_HisInfo(history_Info* p)
{
  if (p)
  {
    (*p) = HisInfo;
  }
}
/***********************************************************************
* ��������: gnss_Pe_Update_HisInfo
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�05/10/
***********************************************************************/
static void gnss_Pe_Update_HisInfo(history_Info* p)
{
  uint8_t        flag = FALSE;
  double         dis = 0.0;
  if (p)
  {
    p->context = peMode.userContext.context;
    p->kfFixStatus = gpz_kf_pvt_data->kfFixSource;
    p->kfPosRes = p_Kf->posRes;
    p->kfPosResStd = p_Kf->posResStd;
    p->kfVelRes = p_Kf->velRes;
    p->kfVelResStd = p_Kf->velResStd;
    p->kfRunEpochCnt = p_Kf->kfCnt;

    // save good altitude information
    if (gpz_kf_pvt_data->kfFixStatus == FIX_STATUS_NEW && p_Kf->posResStd < 30.0 && p_Kf->posResStd > 0.0
        && gpz_kf_pvt_data->kfWeek > 0 && gpz_kf_pvt_data->kfFixSource == FIX_SOURCE_3D && gpz_ls_pvt_data->fix_dim == DIM_3D)
    {
      /* check If the LS and KF result match with each other */
      dis = gnssCalPosDis(gpz_kf_pvt_data->kfLLApos, gpz_ls_pvt_data->LLApos, 1);
      if (dis < 20.0 && fabs(gpz_kf_pvt_data->kfLLApos[2] - gpz_ls_pvt_data->LLApos[2]) < 20.0 && gpz_Ls->pos_res < 30.0 && gpz_Meas->validPrNum >= 10)
      {
        flag = TRUE;
      }
      else
      {
        flag = FALSE;
      }
      /* If underEle, then don't save height*/
      if (peMode.userSceData.isUnderEleRoad)
      {
        flag = FALSE;
      }
    }
    /* Need update altitude */
    if (flag == TRUE)
    {
      if (fabs(p->preciseAlt) < 1e-3)
      {
        p->preciseAlt = gpz_kf_pvt_data->kfLLApos[2];
      }
      else
      {
        if (gpz_Ls->pos_res < 15.0 && p_Kf->posResStd < 15.0 && gpz_Meas->validPrNum >= 15)
        {
          if (fabs(p->preciseAlt - gpz_kf_pvt_data->kfLLApos[2]) > 10.0)
          {
            p->preciseAlt = gpz_kf_pvt_data->kfLLApos[2];
          }
          else if (peMode.staticData.staticFlag)
          {
            p->preciseAlt = 15 * p->preciseAlt / 16 + gpz_kf_pvt_data->kfLLApos[2] / 16;
          }
          else
          {
            p->preciseAlt = 3 * p->preciseAlt / 4 + gpz_kf_pvt_data->kfLLApos[2] / 4;
          }
        }
        else if (peMode.staticData.staticFlag)
        {
          p->preciseAlt = 31 * p->preciseAlt / 32 + gpz_kf_pvt_data->kfLLApos[2] / 32;
        }
        else
        {
          p->preciseAlt = 7 * p->preciseAlt / 8 + gpz_kf_pvt_data->kfLLApos[2] / 8;
        }
      }

      p->towForAlt = gpz_kf_pvt_data->kfTow;
      p->weekForAlt = gpz_kf_pvt_data->kfWeek;
      p->llaPosForAlt[0] = gpz_kf_pvt_data->kfLLApos[0];
      p->llaPosForAlt[1] = gpz_kf_pvt_data->kfLLApos[1];
      p->llaPosForAlt[2] = gpz_kf_pvt_data->kfLLApos[2];
#if defined(PLAYBACK_MODE)
      SYS_LOGGING(OBJ_PE, LOG_INFO, "%s,%14.4f,%6.2f,%6.2f,%03d,%10.4f,%02d", __FUNCTION__, p->towForAlt, p->preciseAlt, realAlt, gpz_Meas->avgCno, gpz_Ls->pos_res, peMode.staticData.staticFlag);
#endif
    }
  }
}
/***********************************************************************
* ��������: gnss_Pe_PrDiffStd_Calc
* ���������
* ���������
* ����ֵ��
* ���ߣ�jh
* ���ڣ�09/07/
***********************************************************************/
void gnss_Pe_PrDiffStd_Calc(meas_blk_t* pMeas, float* prDiff, uint8_t prDiffNum, uint8_t* index)
{
  uint8_t              flag, cnt;
  float             prDiffAvg, median;
  float             diffAbs[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 };

  cnt = prDiffNum;

  if (prDiffNum > 6)
  {
    flag = gnss_MAD(prDiff, diffAbs, prDiffNum, &median);
    if (flag)
    {
      gnss_Sort_WithIndx_1(diffAbs, prDiff, index, prDiffNum);
      if (diffAbs[prDiffNum - 1] > 15 * (median / 0.6745) && (diffAbs[prDiffNum - 1] - median) / (diffAbs[prDiffNum - 3] - median) > 7.5 &&
        fabs(prDiff[prDiffNum - 1]) > 200)
      {
        cnt--;
      }
      if (diffAbs[prDiffNum - 2] > 15 * (median / 0.6745) && (diffAbs[prDiffNum - 2] - median) / (diffAbs[prDiffNum - 3] - median) > 7.5 &&
        fabs(prDiff[prDiffNum - 2]) > 200)
      {
        cnt--;
      }
      prDiffNum = cnt;
    }
  }

  if (prDiffNum >= 5)
  {
    gnss_math_fstd(prDiff, prDiffNum, &prDiffAvg, &(pMeas->prDiffStd_1));
  }
  else
  {
    pMeas->prDiffStd_1 = -1.0;
  }
}

static void gnss_Pe_PrDiffStd_Calc_ForFlp(float* prRes, uint8_t prResNum, float* std)
{
  uint8_t 		i, prDiffNum = 0, cnt, flag, baseNum;
  int8_t 		index[MAX_MEAS_NUM];
  float     avg, median;
  float* prDiff = NULL;
  float* diffAbs = NULL;

  if (prRes == NULL || std == NULL) {
    return;
  }

  if (prResNum == 0)
  {
    *std = -1.0;
    return;
  }

  prDiff = (float*)Sys_Calloc(prResNum, sizeof(float));
  diffAbs = (float*)Sys_Calloc(prResNum, sizeof(float));

  if (prDiff == NULL || diffAbs == NULL)
  {
    Sys_Free(prDiff);
    Sys_Free(diffAbs);
    return;
  }

  for (i = 0; i < prResNum; i++)
  {
    index[prDiffNum] = -1;
    if (fabs(prRes[i]) < 400)
    {
      prDiff[prDiffNum] = prRes[i];
      index[prDiffNum] = i;
      prDiffNum++;
    }
  }

  cnt = prDiffNum;

  if (prDiffNum > 20)
  {
    baseNum = 5;
  }
  else
  {
    baseNum = 3;
  }

  flag = gnss_MAD(prDiff, diffAbs, prDiffNum, &median);
  if ((prDiffNum > 6) && (flag == TRUE))
  {
    gnss_Sort_WithIndx_1(diffAbs, prDiff, (uint8_t*)index, prDiffNum);
    if (diffAbs[prDiffNum - 1] > 30 * (median / 0.6745) && (diffAbs[prDiffNum - 1] - median) / (diffAbs[prDiffNum - baseNum] - median) > 10 &&
      fabs(prDiff[prDiffNum - 1]) > 150)
    {
      cnt--;
    }
    else if (diffAbs[prDiffNum - 1] > 60 * (median / 0.6745) && (diffAbs[prDiffNum - 1] - median) / (diffAbs[prDiffNum - baseNum] - median) > 4 &&
      fabs(prDiff[prDiffNum - 1]) > 150)
    {
      cnt--;
    }
    else if (diffAbs[prDiffNum - 1] > 45 * (median / 0.6745) && (diffAbs[prDiffNum - 1] - median) / (diffAbs[prDiffNum - baseNum] - median) > 20 &&
      fabs(prDiff[prDiffNum - 1]) > 80)
    {
      cnt--;
    }

    if (diffAbs[prDiffNum - 2] > 30 * (median / 0.6745) && (diffAbs[prDiffNum - 2] - median) / (diffAbs[prDiffNum - baseNum] - median) > 10 &&
      fabs(prDiff[prDiffNum - 2]) > 150)
    {
      cnt--;
    }
    else if (diffAbs[prDiffNum - 2] > 45 * (median / 0.6745) && (diffAbs[prDiffNum - 2] - median) / (diffAbs[prDiffNum - baseNum] - median) > 20 &&
      fabs(prDiff[prDiffNum - 2]) > 80)
    {
      cnt--;
    }

    if (baseNum >= 5)
    {
      if (diffAbs[prDiffNum - 3] > 30 * (median / 0.6745) && (diffAbs[prDiffNum - 3] - median) / (diffAbs[prDiffNum - baseNum] - median) > 10 &&
        fabs(prDiff[prDiffNum - 3]) > 150)
      {
        cnt--;
      }
      else if (diffAbs[prDiffNum - 3] > 45 * (median / 0.6745) && (diffAbs[prDiffNum - 3] - median) / (diffAbs[prDiffNum - baseNum] - median) > 20 &&
        fabs(prDiff[prDiffNum - 3]) > 80)
      {
        cnt--;
      }

      if (diffAbs[prDiffNum - 4] > 30 * (median / 0.6745) && (diffAbs[prDiffNum - 4] - median) / (diffAbs[prDiffNum - baseNum] - median) > 10 &&
        fabs(prDiff[prDiffNum - 4]) > 150)
      {
        cnt--;
      }
      else if (diffAbs[prDiffNum - 4] > 45 * (median / 0.6745) && (diffAbs[prDiffNum - 4] - median) / (diffAbs[prDiffNum - baseNum] - median) > 20 &&
        fabs(prDiff[prDiffNum - 4]) > 80)
      {
        cnt--;
      }
    }

    prDiffNum = cnt;
  }

  gnss_math_fstd(prDiff, prDiffNum, &avg, std);
  Sys_Free(prDiff);
  Sys_Free(diffAbs);
  return;
}


void gnss_Pe_Dcb_Check(meas_blk_t* pMeas)
{
  uint8_t                freq_idx, madflag;
  uint8_t                idx, idx_meas[MAX_MEAS_NUM];
  uint8_t                cnt, cnt1 = 0;
  uint8_t                satMode;
  uint32_t               i;
  gnss_meas_t* pSvMeas;
  GNSS_TIME* pTime;
  double               avg[AGGNSS_MAX_FREQ_NUM + 1] = { 0.0 };
  double               diff[MAX_MEAS_NUM];
  double               std_range[AGGNSS_MAX_FREQ_NUM + 1] = { 0.0 };
  double               medianout, dataabs[MAX_MEAS_NUM] = { 0.0 };
  double               diff_thres;
  Kf_t* p_Kf_temp;

  p_Kf_temp = gnss_Pe_Get_Kf();

  pTime = gnss_tm_get_time();
  if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_HD9310 || g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100)
  {
    diff_thres = 6.0;
  }
  else
  {
    diff_thres = 20;
  }

  for (satMode = 0; satMode < GNSS_MAX_MODE; satMode++)
  {
    for (freq_idx = 0; freq_idx < AGGNSS_MAX_FREQ_NUM + 1; freq_idx++)
    {
      avg[freq_idx] = 0;
      cnt = 0;
      std_range[freq_idx] = 0;
      madflag = 0;

      memset(idx_meas, 0, sizeof(uint8_t) * MAX_MEAS_NUM);

      if (pTime->dcb_valid_flag[satMode][freq_idx])
      {
        continue;
      }

      for (i = 0; i < pMeas->measCnt; i++)
      {
        pSvMeas = &pMeas->meas[i];
        if (pSvMeas->gnssMode != satMode || (fabs(pSvMeas->pr_diff) < 1e-6) || (pSvMeas->freq_index != freq_idx))
        {
          continue;
        }

        diff[cnt] = pSvMeas->pr_diff;
        idx_meas[cnt] = i;
        avg[freq_idx] += diff[cnt++];
        pSvMeas->dcbValid = 1;
      }

    Lab_checkbias:
      if (cnt > 3)
      {
        avg[freq_idx] /= cnt;
        std_range[freq_idx] = 0.0;
        for (i = 0; i < cnt; i++)
        {
          std_range[freq_idx] += (diff[i] - avg[freq_idx]) * (diff[i] - avg[freq_idx]);
        }
        std_range[freq_idx] = sqrt(std_range[freq_idx] / cnt);
        if ((std_range[freq_idx] > 10) && (madflag == 0))
        {
          cnt1 = 0;
          avg[freq_idx] = 0.0;

          //screen outliers
          gnss_MAD_DBL(diff, dataabs, cnt, &medianout);
          avg[freq_idx] = 0.0;
          for (i = 0; i < cnt; i++)
          {
            if (dataabs[i] > 5 * (medianout / 0.6745))
            {
              pMeas->meas[idx_meas[cnt]].dcbValid = 0;
              continue;
            }
            avg[freq_idx] += diff[i];
            diff[cnt1++] = diff[i];
          }
          cnt = cnt1;
          madflag = 1;
          goto Lab_checkbias;
        }
      }
    }

    for (freq_idx = 1; freq_idx < AGGNSS_MAX_FREQ_NUM + 1; freq_idx++)
    {

      if (pTime->dcb_valid_flag[satMode][freq_idx])
      {
        continue;
      }

      if ((std_range[0] > DBL_EPSILON) && (std_range[freq_idx] > DBL_EPSILON) && (std_range[0] < 10) && (std_range[freq_idx] < 10) && (fabs(avg[freq_idx] - avg[0]) > diff_thres))
      {
        if (freq_idx == 1 && (fabs(avg[freq_idx] - avg[0]) < 20))
        {
          continue;
        }
        //if (freq_idx == 2 && (std_range[0] > 2 || std_range[freq_idx]>2)&&(g_pe_cfg.sub_chipType==SUB_CHIPTYPE_HD9310))
        //{
        //	continue;
        //}

        if (p_Kf_temp->fre1_satcnt[satMode] > 0)
        {
          for (i = 0; i < pMeas->measCnt; i++)
          {
            pSvMeas = &pMeas->meas[i];
            if (pSvMeas->gnssMode != satMode || (pSvMeas->freq_index != freq_idx))
            {
              continue;
            }
            pSvMeas->pr_diff += (float)(avg[0] - avg[freq_idx]);
          }

          pTime->dcb[satMode + (freq_idx - 1) * GNSS_MAX_MODE] += avg[0] - avg[freq_idx];

          if (kfInfo.status & KF_RUN)
          {
            kfInfo.X[11 + satMode + (freq_idx - 1) * GNSS_MAX_MODE] = pTime->dcb[satMode + (freq_idx - 1) * GNSS_MAX_MODE];
          }

          SYS_LOGGING(OBJ_PE, LOG_INFO, "use pr_diff to update dcb:%14.6f,%2d,%2d,%10.4f", pTime->tor, freq_idx, satMode, pTime->dcb[satMode + (freq_idx - 1) * GNSS_MAX_MODE]);
        }
        else if (p_Kf_temp->fre2_satcnt[satMode] > 0)
        {
          for (i = 0; i < pMeas->measCnt; i++)
          {
            pSvMeas = &pMeas->meas[i];
            if (pSvMeas->gnssMode != satMode || (pSvMeas->freq_index != 0))
            {
              continue;
            }
            pSvMeas->pr_diff -= (float)(avg[0] - avg[freq_idx]);
          }

          pTime->dcb[satMode + (freq_idx - 1) * GNSS_MAX_MODE] += avg[0] - avg[freq_idx];

          // TM bias adjust
          pTime->bias[satMode] += pTime->dcb[satMode + (freq_idx - 1) * GNSS_MAX_MODE];

          // KF bias adjust when KF is running
          if (kfInfo.status & KF_RUN)
          {
            idx = clockBiasIdx[satMode][p_Kf_temp->kf_ctrl.biasLinkFlag];
            kfInfo.X[idx] += pTime->dcb[satMode + (freq_idx - 1) * GNSS_MAX_MODE];
            kfInfo.X[11 + satMode + (freq_idx - 1) * GNSS_MAX_MODE] = pTime->dcb[satMode + (freq_idx - 1) * GNSS_MAX_MODE];
            gpz_kf_pvt_data->clkBias_last[satMode] += pTime->dcb[satMode + (freq_idx - 1) * GNSS_MAX_MODE];;
          }

          SYS_LOGGING(OBJ_PE, LOG_INFO, "use pr_diff to update dcb and bias:%14.6f,%2d,%2d,%10.4f", pTime->tor, freq_idx, satMode, pTime->dcb[satMode + (freq_idx - 1) * GNSS_MAX_MODE]);
        }
      }
      else if ((std_range[0] > DBL_EPSILON) && (std_range[freq_idx] > DBL_EPSILON) && (std_range[0] < 5) && (std_range[freq_idx] < 5) && (fabs(avg[freq_idx] - avg[0]) < 5))
      {
        //pTime->dcb_valid_flag[satMode][freq_idx] = 1;
        SYS_LOGGING(OBJ_PE, LOG_INFO, "confirm that dcb initial value is valid:%14.6f,%2d,%2d,%10.4f", pTime->tor, freq_idx, satMode, pTime->dcb[satMode + (freq_idx - 1) * GNSS_MAX_MODE]);
      }
    }
  }
}
void gnss_Pe_DCB_Smooth(Ls_t* pLs, Kf_t* pKf)
{
  GNSS_TIME* pTime;
  KF_PVT_INFO* kfPvtInfo;
  int32_t            i, gap;
  double            delta_DCB;
  meas_blk_t* pMeas;

  pTime = gnss_tm_get_time();
  kfPvtInfo = pKf->kf_Pvt_Info;
  pMeas = pKf->meas_blk;
  gap = 1;

  for (i = 0; i < BIAS_NUM * 2; i++)
  {
    if (pMeas->avgCno <= 30) continue;
    if (pKf->posRes >= 8 || pLs->pos_res >= 8) continue;
    if (pKf->meas_blk->measCnt <= 15)  continue;
    if (pKf->fre2_satcnt[i] <= DCB_SAT_LIMIT) continue;
    //if dcb is default ,update 
    if (DBL_IS_EQUAL(pTime->dcb[i], DEFAULT_DCB_TABLE[g_pe_cfg.sub_chipType][i]) && fabs(kfPvtInfo->dcb[i]) > 0.0)
    {
      if (fabs(kfPvtInfo->dcb[i] - DEFAULT_DCB_TABLE[g_pe_cfg.sub_chipType][i]) < 20)
      {
        pTime->dcb[i] = kfPvtInfo->dcb[i];
      }
    }

    delta_DCB = fabs(kfPvtInfo->dcb[i] - pTime->dcb[i]);

    //DCB smooth
    if (pKf->kfCnt < 10)
    {
      if (pKf->posRes < 8 && pLs->pos_res < 8)
      {
        if (delta_DCB > 10)
        {
          gap = -1;
        }
        else if (delta_DCB > 5)
        {
          gap = 20;
        }
        else if (delta_DCB > 2)
        {
          gap = 10;
        }
        else if (delta_DCB > 0)
        {
          gap = 5;
        }
      }
      else
      {
        gap = -1;
      }
    }
    else
    {
      if (pKf->posRes < 5 && pLs->pos_res < 5)
      {
        if (delta_DCB > 8)
        {
          gap = -1;
        }
        if (delta_DCB > 5)
        {
          gap = 30;
        }
        else if (delta_DCB > 2)
        {
          gap = 20;
        }
        else if (delta_DCB > 0)
        {
          gap = 10;
        }
      }
      else
      {
        gap = -1;
      }
    }
    if (gap > 0)
    {
      pTime->dcb[i] = pTime->dcb[i] * (gap - 1.0) / gap + kfPvtInfo->dcb[i] / gap;
    }
  }
}
/***********************************************************************
* ��������: gnss_Pe_Ins_Vel_Check   ���FLP�������ٶ���Ч��
* ���������
* ���������
* ����ֵ��
* ���ߣ�none
* ���ڣ�09/24/
***********************************************************************/
uint8_t gnss_Pe_Ins_Vel_Check(const VDR_Gnss_Feedback_t* pVdrFeedInfo, const uint32_t uMode, const USER_PVT* puser)
{
  uint8_t          valid = FALSE, uReturn = FALSE, i = 0;
  float            fAccIne = 0, fDistance_XYZ = 0, fGnssVelDiff[4] = { 0 }, fGnssPosDiff = 0, fGnssVel = 0;
  //double         dGnssPosDiff[4] = { 0 };
  GNSS_TIME* pTime = NULL;
  gtime_t          gtutcTime;
  int64_t          timeStampOfUtc = 0;
  float            fThre_Diff_Vel = 15.0, fThre_Max_Vel = 40.0, fThre_Pos = 400.0, fThre_Acc = 5.0;

  if (pVdrFeedInfo == NULL) return uReturn;

  pTime = gnss_tm_get_time();
  gtutcTime = gpst2utc(pTime->GPSTime);
  timeStampOfUtc = (long long)((long long)gtutcTime.time * 1000 + gtutcTime.sec * 1000);

  if ((fabs((double)timeStampOfUtc - pVdrFeedInfo->timeStampOfUtc) < 30.0) && pVdrFeedInfo->uInsVelOK)
  {
    valid |= VDR_FEEDBACK_OK;
  }

  if (uMode == VDR_FEEDBACK_STATIC)//static   1
  {
    GLOGI("%s %8.3f %lld %lld %lld %d %8.4f", __FUNCTION__, pTime->tor, timeStampOfUtc, pVdrFeedInfo->timeStampOfUtc, 
      timeStampOfUtc - pVdrFeedInfo->timeStampOfUtc, pVdrFeedInfo->uInsVelOK, pVdrFeedInfo->heading);
  }

  if (valid)
  {
    if (uMode == VDR_FEEDBACK_STATIC)//static   1
    {
      uReturn = (pVdrFeedInfo->uInsVelOK & VDR_FEEDBACK_STATIC) == VDR_FEEDBACK_STATIC ? TRUE : FALSE;
    }
    else if (uMode == VDR_FEEDBACK_VEL)// vel   2
    {
      uReturn = (pVdrFeedInfo->uInsVelOK & VDR_FEEDBACK_VEL) == VDR_FEEDBACK_VEL ? TRUE : FALSE;
      if (uReturn && puser)
      {
        for (i = 0; i < 3; i++)
        {
          fGnssVelDiff[i] = puser->ecef.vel[i] - pVdrFeedInfo->fvelEcef[i];
        }
        fGnssVelDiff[3] = sqrtf(fGnssVelDiff[0] * fGnssVelDiff[0] + fGnssVelDiff[1] * fGnssVelDiff[1] + fGnssVelDiff[2] * fGnssVelDiff[2]);
        if ((fGnssVelDiff[3] < fThre_Diff_Vel) && fabs(pVdrFeedInfo->fvelEcef[0]) < fThre_Max_Vel && 
          fabs(pVdrFeedInfo->fvelEcef[1]) < fThre_Max_Vel && fabs(pVdrFeedInfo->fvelEcef[2]) < fThre_Max_Vel)
        {
          uReturn = TRUE;
        }
        else
        {
          uReturn = FALSE;
          GLOGI("Feedback vel Abornamal! %lld %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f", timeStampOfUtc, fGnssVelDiff[3], puser->ecef.vel[0], pVdrFeedInfo->fvelEcef[0],
            puser->ecef.vel[1], pVdrFeedInfo->fvelEcef[1], puser->ecef.vel[2], pVdrFeedInfo->fvelEcef[2]);
        }
      }
    }
    else if (uMode == VDR_FEEDBACK_POS)//pos    3
    {
      uReturn = (pVdrFeedInfo->uInsVelOK & VDR_FEEDBACK_POS) == VDR_FEEDBACK_POS ? TRUE : FALSE;
      if (uReturn && puser)
      {
        fGnssPosDiff = gnssCalPosDis((double*)pVdrFeedInfo->llaPos, (double*)puser->lla.pos, 1);
        if (fGnssPosDiff < fThre_Pos)
        {
          uReturn = TRUE;
        }
        else
        {
          uReturn = FALSE;
          GLOGI("Feedback Pos Abornamal! %lld %8.4f", timeStampOfUtc, fGnssPosDiff);
        }
      }
      //uReturn = FALSE;
    }
    else if (uMode == VDR_FEEDBACK_ODO2GNSSVEL)//odo2Gnss vel   4
    {
      //uReturn = (pVdrFeedInfo->uInsVelOK & VDR_FEEDBACK_ODO2GNSSVEL) == VDR_FEEDBACK_ODO2GNSSVEL ? TRUE : FALSE;
      uReturn = FALSE;
    }
    else if (uMode == VDR_FEEDBACK_ACCINE)//acc    5
    {
      uReturn = (pVdrFeedInfo->uInsVelOK & VDR_FEEDBACK_ACCINE) == VDR_FEEDBACK_ACCINE ? TRUE : FALSE;
      fAccIne = (float)sqrtf(pVdrFeedInfo->fAccIne[0] * pVdrFeedInfo->fAccIne[0] + pVdrFeedInfo->fAccIne[1] * pVdrFeedInfo->fAccIne[1] + pVdrFeedInfo->fAccIne[2] * pVdrFeedInfo->fAccIne[2]);
      if (uReturn)
      {
        if (fAccIne < fThre_Acc && fAccIne > 0.0001)
        {
          uReturn = TRUE;
        }
        else
        {
          uReturn = FALSE;
          GLOGI("Feedback Acc Abornamal! %lld %8.5f %8.5f", timeStampOfUtc, fAccIne, pVdrFeedInfo->fAccIne[0]);
        }
      }
    }
    else if (uMode == VDR_FEEDBACK_RTK)//rtk   6
    {
      //uReturn = (pVdrFeedInfo->uInsVelOK & VDR_FEEDBACK_RTK) == VDR_FEEDBACK_RTK ? TRUE : FALSE;
      uReturn = FALSE;
    }
    else if (uMode == VDR_FEEDBACK_HEADING)//Heading   7
    {
      //uReturn = (pVdrFeedInfo->uInsVelOK & VDR_FEEDBACK_HEADING) == VDR_FEEDBACK_HEADING ? TRUE : FALSE;
      uReturn = FALSE;
    }
    else if (uMode == VDR_FEEDBACK_ELE)//Under Ele   8
    {
      uReturn = (pVdrFeedInfo->uInsVelOK & VDR_FEEDBACK_ELE) == VDR_FEEDBACK_ELE ? TRUE : FALSE;
      //uReturn = FALSE;
    }
    else if (uMode == VDR_FEEDBACK_DELTAX)//Deltx  9
    {
      uReturn = (pVdrFeedInfo->uInsVelOK & VDR_FEEDBACK_DELTAX) == VDR_FEEDBACK_DELTAX ? TRUE : FALSE;
      if (uReturn && puser)
      {
        fDistance_XYZ = (float)sqrtf(pVdrFeedInfo->fDistance_XYZ[0] * pVdrFeedInfo->fDistance_XYZ[0] + 
          pVdrFeedInfo->fDistance_XYZ[1] * pVdrFeedInfo->fDistance_XYZ[1] + pVdrFeedInfo->fDistance_XYZ[2] * pVdrFeedInfo->fDistance_XYZ[2]);
        fGnssVel = sqrtf(puser->lla.vel[0] * puser->lla.vel[0] + puser->lla.vel[1] * puser->lla.vel[1]);
        if (fDistance_XYZ > 0.0001f && fDistance_XYZ < fThre_Max_Vel && fabs((double)fGnssVel - fDistance_XYZ) < fThre_Diff_Vel)
        {
          uReturn = TRUE;
        }
        else
        {
          uReturn = FALSE;
          GLOGI("Feedback fDistance_XYZ Abornamal! %lld %8.5f %8.5f", timeStampOfUtc, fDistance_XYZ, fabs((double)fGnssVel - fDistance_XYZ));
        }
      }
      //uReturn = FALSE;
      //if (uReturn && sqrt(pFlpFeedInfo->fDistance_XYZ[0] * pFlpFeedInfo->fDistance_XYZ[0] + pFlpFeedInfo->fDistance_XYZ[1] * pFlpFeedInfo->fDistance_XYZ[1]) > 0.01)
      //{
      //	uReturn = TRUE;
      //}
      //else
      //{
      //	uReturn = FALSE;
      //}

    }
    else
    {
      uReturn = FALSE;
    }
  }

  return uReturn;
}
/***********************************************************************
* ��������: gnss_Pe_Diff ����в�
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�10/11/
***********************************************************************/
void gnss_Pe_Diff(meas_blk_t* pMeas, Kf_t* pKf, VDR_Gnss_Feedback_t* pVdrInfo)
{
  uint8_t       prDiffNum = 0;
  uint8_t       index[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 };
  uint32_t      i;
  float         prDiff[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 };
  float         user_vel[3] = { 0.0 };
  double        detPos[3], detVel[3];
  double        calRange, calDoppler;
  double        dt;
  double        biasDiffUse[GNSS_MAX_MODE] = { 0.0 };
  double        duser_pos[3] = { 0 };
  gnss_meas_t*  pSvMeas;
  sat_data_t*   sp;
  GNSS_TIME*    pTime;
  USER_PVT      user;
  VDR_FEEDBACKFLAG* p;
#if defined(PLAYBACK_MODE)
  float             fVelError[4] = { 0 }, fVelGnssError[4] = { 0 };
  float             FeedbackenuVel[4] = { 0 }, GnssenuVel[4] = { 0 };
  float	            FeedbackenuPos[8] = { 0 }, GnssenuPos[8] = { 0 };
  double            dXYZINSREFDiff[4] = { 0 }, dDisIns = 0, dDisRefs = 0, dRefXYZ[3] = { 0 };
  double            dAttError[3] = { 0 };
  static double     dRefPosLast[3] = { 0 };
#endif 

  pTime = gnss_tm_get_time();
  pMeas->hasDiff = FALSE;
  p = &(peMode.feedbackFlag);
  memset(p, 0, sizeof(VDR_FEEDBACKFLAG));

  if (pTime->init == FALSE)
  {
    pMeas->prDiffStd_1 = -1.0;
    return;
  }

  gnss_Pe_Get_PosFix(&user);

  if (user.ecef.have_position == POS_FRM_NULL)
  {
    pMeas->prDiffStd_1 = -1.0;
    return;
  }
  if (pVdrInfo)
  {
    p->uVelVdrFeedback = gnss_Pe_Ins_Vel_Check(pVdrInfo, VDR_FEEDBACK_VEL, &user);
    p->uAccVdrFeedback = gnss_Pe_Ins_Vel_Check(pVdrInfo, VDR_FEEDBACK_ACCINE, &user);
    p->uPosVdrFeedback = gnss_Pe_Ins_Vel_Check(pVdrInfo, VDR_FEEDBACK_POS, &user);
    p->uEleVdrFeedback = gnss_Pe_Ins_Vel_Check(pVdrInfo, VDR_FEEDBACK_ELE, &user);
    p->uStaticVdrfeedback = gnss_Pe_Ins_Vel_Check(pVdrInfo, VDR_FEEDBACK_STATIC, &user);
    p->uOdoVdrfeedback = gnss_Pe_Ins_Vel_Check(pVdrInfo, VDR_FEEDBACK_ODO2GNSSVEL, &user);
    p->uDeltxVdrFeedback = gnss_Pe_Ins_Vel_Check(pVdrInfo, VDR_FEEDBACK_DELTAX, &user);
  }

  if (p->uVelVdrFeedback)
  {
    for (i = 0; i < 3; i++) user_vel[i] = pVdrInfo->fvelEcef[i];
  }
  else
  {
    memcpy(user_vel, user.ecef.vel, sizeof(user_vel));
    if (p->uAccVdrFeedback && p->uEleVdrFeedback && !peMode.staticData.staticFlag)
    {
      for (i = 0; i < 3; i++)
      {
        user_vel[i] =(float)(user_vel[i] + pVdrInfo->fAccIne[i] * pTime->dt);
      }
    }
  }
  if (p->uPosVdrFeedback)
  {
    pos2ecef(pVdrInfo->llaPos, duser_pos);
  }
  else
  {
    memcpy(duser_pos, user.ecef.pos, sizeof(duser_pos));
  }
#if defined(PLAYBACK_MODE)

  if (pMeas->hpRefQflag[1])
  {
    fVelError[0] = (float)(user_vel[0] - pMeas->hpRefEcef[3]);
    fVelError[1] = (float)(user_vel[1] - pMeas->hpRefEcef[4]);
    fVelError[2] = (float)(user_vel[2] - pMeas->hpRefEcef[5]);
    gnssConvEcef2EnuVel(fVelError, FeedbackenuVel, pMeas->hpRefLla);
    FeedbackenuVel[3] = sqrtf(FeedbackenuVel[0] * FeedbackenuVel[0] + FeedbackenuVel[1] * FeedbackenuVel[1]);

    fVelGnssError[0] = (float)(user.ecef.vel[0] - pMeas->hpRefEcef[3]);
    fVelGnssError[1] = (float)(user.ecef.vel[1] - pMeas->hpRefEcef[4]);
    fVelGnssError[2] = (float)(user.ecef.vel[2] - pMeas->hpRefEcef[5]);
    gnssConvEcef2EnuVel(fVelGnssError, GnssenuVel, pMeas->hpRefLla);
    GnssenuVel[3] = sqrtf(GnssenuVel[0] * GnssenuVel[0] + GnssenuVel[1] * GnssenuVel[1]);


    FeedbackenuPos[0] = (float)(duser_pos[0] - pMeas->hpRefEcef[0]);
    FeedbackenuPos[1] = (float)(duser_pos[1] - pMeas->hpRefEcef[1]);
    FeedbackenuPos[2] = (float)(duser_pos[2] - pMeas->hpRefEcef[2]);
    gnssConvEcef2EnuVel(FeedbackenuPos, FeedbackenuPos + 4, pMeas->hpRefLla);
    FeedbackenuPos[7] = sqrtf(FeedbackenuPos[4] * FeedbackenuPos[4] + FeedbackenuPos[5] * FeedbackenuPos[5]);

    GnssenuPos[0] = (float)(user.ecef.pos[0] - pMeas->hpRefEcef[0]);
    GnssenuPos[1] = (float)(user.ecef.pos[1] - pMeas->hpRefEcef[1]);
    GnssenuPos[2] = (float)(user.ecef.pos[2] - pMeas->hpRefEcef[2]);
    gnssConvEcef2EnuVel(GnssenuPos, GnssenuPos + 4, pMeas->hpRefLla);
    GnssenuPos[7] = sqrtf(GnssenuPos[4] * GnssenuPos[4] + GnssenuPos[5] * GnssenuPos[5]);

    dDisIns = sqrt((double)pVdrInfo->fDistance_XYZ[0] * pVdrInfo->fDistance_XYZ[0] + 
      (double)pVdrInfo->fDistance_XYZ[1] * pVdrInfo->fDistance_XYZ[1] + (double)pVdrInfo->fDistance_XYZ[2] * pVdrInfo->fDistance_XYZ[2]);

    dRefXYZ[0] = (pMeas->hpRefEcef[0] - dRefPosLast[0]);
    dRefXYZ[1] = (pMeas->hpRefEcef[1] - dRefPosLast[1]);
    dRefXYZ[2] = (pMeas->hpRefEcef[2] - dRefPosLast[2]);
    dDisRefs = sqrt(dRefXYZ[0] * dRefXYZ[0] + dRefXYZ[1] * dRefXYZ[1] + dRefXYZ[2] * dRefXYZ[2]);

    dXYZINSREFDiff[0] = pVdrInfo->fDistance_XYZ[0] - dRefXYZ[0];
    dXYZINSREFDiff[1] = pVdrInfo->fDistance_XYZ[1] - dRefXYZ[1];
    dXYZINSREFDiff[2] = pVdrInfo->fDistance_XYZ[2] - dRefXYZ[2];
    dXYZINSREFDiff[3] = dDisIns - dDisRefs;
    dRefPosLast[0] = pMeas->hpRefEcef[0];
    dRefPosLast[1] = pMeas->hpRefEcef[1];
    dRefPosLast[2] = pMeas->hpRefEcef[2];

    dAttError[2] = pVdrInfo->heading - pMeas->hpRefAtt[2];

    if (dAttError[2] > 360)
    {
      dAttError[2] = dAttError[2] - 360;
    }
    else if (dAttError[2] < -360)
    {
      dAttError[2] = dAttError[2] + 360;
    }
  }
  GLOGI("PE Use INS vel Pos to calculate DR diff:%8.3f %lld %d %d %d %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f  %8.4f %8.4f %8.4f %8.4f %8.4f",
    pMeas->tor, vdrFeedbackInfo.uInsVelOK, p->uVelVdrFeedback, p->uPosVdrFeedback, p->uAccVdrFeedback,
    FeedbackenuVel[3], GnssenuVel[3], FeedbackenuPos[7], GnssenuPos[7],
    FeedbackenuVel[0], FeedbackenuVel[1], FeedbackenuVel[2],
    GnssenuVel[0], GnssenuVel[1], GnssenuVel[2], dXYZINSREFDiff[0], dXYZINSREFDiff[1], dXYZINSREFDiff[2], dXYZINSREFDiff[3], dAttError[2]);
#endif // !


  //memcpy(duser_pos, user.ecef.pos, sizeof(duser_pos));
  pMeas->hasDiff = TRUE;

  //load  bias diff for use
  for (i = 0; i < GNSS_MAX_MODE; i++)
  {
    if (pMeas->rtdUsed && pKf->biasDiffUpCnt[i] >= BIASDIFF_UPCNT_THRES)
    {
      biasDiffUse[i] = pKf->biasDiff[i];
    }
    else if (!pMeas->rtdUsed && pKf->biasDiffLocalUpCnt[i] >= BIASDIFF_UPCNT_THRES)
    {
      biasDiffUse[i] = pKf->biasDiffLocal[i];
    }
    else
    {
      biasDiffUse[i] = 0.0;
    }
  }

  // PR and DR Diff calculation
  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &pMeas->meas[i];
    if (pSvMeas->prn == 0) continue;
    // by-pass invalid PR
    if (pSvMeas->pseudoRange < 0.0)
    {
      pSvMeas->status &= 0xFE;
      continue;
    }
    if (pSvMeas->sv_info.eph_status == FALSE)
    {
      continue;
    }

    detPos[0] = pSvMeas->sv_info.p[0] - duser_pos[0];//(-2851076.588);
    detPos[1] = pSvMeas->sv_info.p[1] - duser_pos[1];//4657647.890;
    detPos[2] = pSvMeas->sv_info.p[2] - duser_pos[2];//3284182.303;
    calRange = gnssClcSqrtSum_DBL(detPos, 3);

    dt = calRange / LIGHT_SEC;
    detPos[0] = (pSvMeas->sv_info.p[0] + (WGS84_OMEGDOTE * pSvMeas->sv_info.p[1] * dt) - duser_pos[0]);
    detPos[1] = (pSvMeas->sv_info.p[1] - (WGS84_OMEGDOTE * pSvMeas->sv_info.p[0] * dt) - duser_pos[1]);
    detPos[2] = pSvMeas->sv_info.p[2] - duser_pos[2];
    calRange = gnssClcSqrtSum_DBL(detPos, 3);

#if 1
    /*
    (1) GPS + GLN + BDS, GPS + GLN or GPS + BDS
    (2) GLN + BDS
    */
    if (gnss_tm_check_bias_status(GPS_MODE) == TRUE && pKf->kf_Pvt_Info->biasValid[GPS_MODE] == TRUE)
    {
      if (fabs(biasDiffUse[pSvMeas->gnssMode]) > 0)
      {
        pSvMeas->pr_diff = -(float)(calRange + pTime->bias[GPS_MODE] - biasDiffUse[pSvMeas->gnssMode] - pSvMeas->pseudoRange);
      }
      else
      {
        pSvMeas->pr_diff = -(float)(calRange + pTime->bias[pSvMeas->gnssMode] - pSvMeas->pseudoRange);
      }
    }
    else if (gnss_tm_check_bias_status(GLN_MODE) == TRUE && pKf->kf_Pvt_Info->biasValid[GLN_MODE] == TRUE && fabs(biasDiffUse[GLN_MODE]) > 0)
    {
      if (pSvMeas->gnssMode == GPS_MODE)
      {
        pSvMeas->pr_diff = -(float)(calRange + pTime->bias[GLN_MODE] + biasDiffUse[GLN_MODE] - pSvMeas->pseudoRange);
      }
      else if (pSvMeas->gnssMode == BDS_MODE && fabs(biasDiffUse[pSvMeas->gnssMode]) > 0)
      {
        pSvMeas->pr_diff = -(float)(calRange + pTime->bias[GLN_MODE] + biasDiffUse[GLN_MODE] - biasDiffUse[pSvMeas->gnssMode] - pSvMeas->pseudoRange);
      }
      else if (pSvMeas->gnssMode == GAL_MODE && fabs(biasDiffUse[pSvMeas->gnssMode]) > 0)
      {
        pSvMeas->pr_diff = -(float)(calRange + pTime->bias[GLN_MODE] + biasDiffUse[GLN_MODE] - biasDiffUse[pSvMeas->gnssMode] - pSvMeas->pseudoRange);
      }
      else
      {
        pSvMeas->pr_diff = -(float)(calRange + pTime->bias[pSvMeas->gnssMode] - pSvMeas->pseudoRange);
      }
    }
    else if (gnss_tm_check_bias_status(BDS_MODE) == TRUE && pKf->kf_Pvt_Info->biasValid[BDS_MODE] == TRUE && fabs(biasDiffUse[BDS_MODE]) > 0)
    {
      if (pSvMeas->gnssMode == GPS_MODE)
      {
        pSvMeas->pr_diff = -(float)(calRange + pTime->bias[BDS_MODE] + biasDiffUse[BDS_MODE] - pSvMeas->pseudoRange);
      }
      else if (pSvMeas->gnssMode == GLN_MODE && fabs(biasDiffUse[pSvMeas->gnssMode]) > 0)
      {
        pSvMeas->pr_diff = -(float)(calRange + pTime->bias[BDS_MODE] + biasDiffUse[BDS_MODE] - biasDiffUse[pSvMeas->gnssMode] - pSvMeas->pseudoRange);
      }
      else if (pSvMeas->gnssMode == GAL_MODE && fabs(biasDiffUse[pSvMeas->gnssMode]) > 0)
      {
        pSvMeas->pr_diff = -(float)(calRange + pTime->bias[BDS_MODE] + biasDiffUse[BDS_MODE] - biasDiffUse[pSvMeas->gnssMode] - pSvMeas->pseudoRange);
      }
      else
      {
        pSvMeas->pr_diff = -(float)(calRange + pTime->bias[pSvMeas->gnssMode] - pSvMeas->pseudoRange);
      }
    }
    else if (gnss_tm_check_bias_status(GAL_MODE) == TRUE && pKf->kf_Pvt_Info->biasValid[GAL_MODE] == TRUE && fabs(biasDiffUse[GAL_MODE]) > 0)
    {
      if (pSvMeas->gnssMode == GPS_MODE)
      {
        pSvMeas->pr_diff = -(float)(calRange + pTime->bias[GAL_MODE] + biasDiffUse[GAL_MODE] - pSvMeas->pseudoRange);
      }
      else if (pSvMeas->gnssMode == GLN_MODE && fabs(biasDiffUse[pSvMeas->gnssMode]) > 0)
      {
        pSvMeas->pr_diff = -(float)(calRange + pTime->bias[GAL_MODE] + biasDiffUse[GAL_MODE] - biasDiffUse[pSvMeas->gnssMode] - pSvMeas->pseudoRange);
      }
      else if (pSvMeas->gnssMode == BDS_MODE && fabs(biasDiffUse[pSvMeas->gnssMode]) > 0)
      {
        pSvMeas->pr_diff = -(float)(calRange + pTime->bias[GAL_MODE] + biasDiffUse[GAL_MODE] - biasDiffUse[pSvMeas->gnssMode] - pSvMeas->pseudoRange);
      }
      else
      {
        pSvMeas->pr_diff = -(float)(calRange + pTime->bias[pSvMeas->gnssMode] - pSvMeas->pseudoRange);
      }
    }
    else
    {
      pSvMeas->pr_diff = -(float)(calRange + pTime->bias[pSvMeas->gnssMode] - pSvMeas->pseudoRange);
    }


    if (pSvMeas->freq_index == 1)
    {
      pSvMeas->pr_diff = (float)(pSvMeas->pr_diff + (pTime->dcb[pSvMeas->gnssMode]));
    }
    else if (pSvMeas->freq_index == 2)
    {
      pSvMeas->pr_diff = (float)(pSvMeas->pr_diff + (pTime->dcb[pSvMeas->gnssMode + GNSS_MAX_MODE]));
    }

    /* PR DIFF statistic */
    if (fabs(pSvMeas->pr_diff) < 20.0)
    {
      pMeas->prDiff20Cnt++;
      if (fabs(pSvMeas->pr_diff) < 10.0)
      {
        pMeas->prDiff10Cnt++;
      }
    }

#else 
    pSvMeas->pr_diff = -(float)(calRange + pTime->bias[pSvMeas->gnssMode] - pSvMeas->pseudoRange);
#endif
    if (gnss_tm_check_bias_status(pSvMeas->gnssMode) == TRUE)
    {
      pSvMeas->prDiffValid = 1;
    }
    else
    {
      pSvMeas->prDiffValid = 0;
    }

    if (pSvMeas->prDiffValid && fabs(pSvMeas->pr_diff) < 400)
    {
      prDiff[prDiffNum] = pSvMeas->pr_diff;
      index[prDiffNum] = prDiffNum;
      prDiffNum++;

      if (fabs(pSvMeas->pr_diff) > 5.0)
      {
        pMeas->prDiff5Cnt++;
      }
    }

    pSvMeas->dr_diff = 0.0;
    if (g_pe_cfg.meas_type_mask & MEAS_TYPE_MASK_DR)
    {
      detVel[0] = (double)pSvMeas->sv_info.v[0] - user_vel[0];
      detVel[1] = (double)pSvMeas->sv_info.v[1] - user_vel[1];
      detVel[2] = (double)pSvMeas->sv_info.v[2] - user_vel[2];

      calDoppler = (detVel[0] * detPos[0] + detVel[1] * detPos[1] + detVel[2] * detPos[2]) / calRange;
      pSvMeas->dr_diff = (float)(calDoppler + pTime->drift - pSvMeas->pseudoRangeRate);

    }
    sp = gnss_sd_get_sv_data(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);
    if (sp != NULL)
    {
      GLOGD("gnss_Pe_Diff:%10.3f %u %02d %03d %3u  %d %14.3f %8.3f  %6.2f %6.2f %6.2f %6.2f",
        pTime->tor, pSvMeas->freq_index, pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->measState & 0xFF,
        pSvMeas->prDiffValid, pSvMeas->pr_diff, pSvMeas->dr_diff, pSvMeas->sv_info.fltAz, pSvMeas->sv_info.fltElev,
        sp->mcorr.iono_corr, sp->mcorr.trop_corr);
    }
#ifdef USED_IN_MC262M
    if (g_pe_cfg.chipType == SPRD && pMeas->avgCno > 18 && pMeas->measCnt > 5)
    {
      if (fabs(pSvMeas->dr_diff) > 2.00 && pSvMeas->mpIndicator == 1 && pSvMeas->usedInChip == 0)
      {
        pSvMeas->status &= 0xFD;
      }
    }
#endif
  }

  pMeas->prDiffNum = prDiffNum;

  //calculate pr diff std for downtown mode detection
  gnss_Pe_PrDiffStd_Calc(pMeas, prDiff, prDiffNum, index);

  gnss_Pe_Dcb_Check(pMeas);
}

/***********************************************************************
* ��������: gnss_Pe_Pos_Extrapolate
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�10/03/
***********************************************************************/
void gnss_Pe_Pos_Extrapolate(USER_PVT* pvt)
{
  uint8_t          uAccVdrFeedback = FALSE;
  uint8_t          UeleFeedback = FALSE, uDeltaxFeedback = FALSE;
  uint32_t         i;
  int16_t          k;
  double           dt;
  GNSS_TIME* pTime;

  pTime = gnss_tm_get_time();
  /* compute time since last fix (if one has been done) */
  if (pvt->posfix_t > 0)
  {
    dt = pTime->rcvr_time[GPS_MODE] - pvt->posfix_t;
    k = pTime->week[GPS_MODE] - pvt->posfix_wn;

    if (k == 1)
    {
      dt += (double)SECS_IN_WEEK;
    }
    else if (k == -1)
    {
      dt -= (double)SECS_IN_WEEK;
    }

    if (!IS_PPK_REVERSE && ((k < 0) || (k > 1) || (dt < 0) || (dt > (double)SECS_IN_WEEK)))
    {
      dt = LIGHT_SEC;
    }
    else if (IS_PPK_REVERSE && ((k < -1) || (k > 0) || (dt > 0) || (dt < -(double)SECS_IN_WEEK)))
    {
      dt = LIGHT_SEC;
    }
  }
  else
  {
    dt = LIGHT_SEC;
  }

  // position propagation when dt is within 5 seconds
  if (pvt->have_position >= HAVE_POS_APPX && pvt->have_position < HAVE_POS_OLD)
  {
    if (fabs(dt) < (double)5.0)
    {
      UeleFeedback = gnss_Pe_Ins_Vel_Check(&vdrFeedbackInfo, VDR_FEEDBACK_ELE, pvt);
      //uAccFlpFeedback = gnss_Pe_Ins_Vel_Check(&flpFeedbackInfo, FLP_FEEDBACK_ACCINE, pvt);
      uDeltaxFeedback = gnss_Pe_Ins_Vel_Check(&vdrFeedbackInfo, VDR_FEEDBACK_DELTAX, pvt);

      if (UeleFeedback && uDeltaxFeedback && fabs(dt) < (double)1.1)
      {
        for (i = 0; i < 3; i++) pvt->ecef.pos[i] += vdrFeedbackInfo.fDistance_XYZ[i];			//TODO: 990ms not 1000ms	
      }
      else
      {
        for (i = 0; i < 3; i++)
        {
          pvt->ecef.pos[i] += pvt->ecef.vel[i] * dt;
        }
        //if (uAccFlpFeedback)
        //{
        //	for (i = 0; i < 3; i++)
        //	{
        //		pvt->ecef.pos[i] += 0.5 * flpFeedbackInfo.fAccIne[i] * dt * dt;
        //	}
        //}
      }
      // convert ECEF to LLA
      gnssConvEcef2Lla(pvt->ecef.pos, pvt->lla.pos);
      gnssConvEcef2EnuVel(pvt->ecef.vel, pvt->lla.vel, pvt->lla.pos);

      // update new fix time
      pvt->posfix_t = pTime->rcvr_time[GPS_MODE];
      pvt->posfix_wn = pTime->week[GPS_MODE];
    }
  }

  /* calculate time gap since last LS or KF position fix */
  dt = pTime->rcvr_time[GPS_MODE] - pvt->ecef.posfix_t;
  dt += (pTime->week[GPS_MODE] - pvt->ecef.posfix_wn) * 604800.0;
  // position status check
  if (pvt->pos_approx == FALSE &&
    (pvt->have_position == HAVE_POS_FIX2D || pvt->have_position == HAVE_POS_FIX3D))
  {
    if ((float)fabs(dt) > FIX_LIFETIME[g_pe_cfg.dynamics])
    {
      pvt->pos_approx = TRUE;
      pvt->have_position = HAVE_POS_OLD;
      SYS_LOGGING(OBJ_PE, LOG_INFO, "Position status changed to old (pos_approx = %d)", pvt->pos_approx);
    }
  }
}

/***********************************************************************
* ��������: gnss_Pe_Law_Chck
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�10/30/
***********************************************************************/
static void gnss_Pe_Law_Chck(USER_PVT* pvt)
{
  // Height limit
  if ((pvt->lla.pos[2] >= g_pe_cfg.altHighLimit || pvt->lla.pos[2] <= g_pe_cfg.altLowLimit)
    && (pvt->ecef.have_position == POS_FRM_LS || pvt->ecef.have_position == POS_FRM_KF))
  {
    goto LAW_CLEAN;
  }
  else
  {
    return;
  }
#if 0
  // speed control
  if (pvt->velocity >= 250 && pvt->lla.have_position != POS_FRM_NULL)
  {
    goto LAW_CLEAN;
  }
  else
  {
    return;
  }
#endif
LAW_CLEAN:
  memset(&(pvt->lla), 0, sizeof(USER_LLA));
  memset(&(pvt->ecef), 0, sizeof(USER_ECEF));
  pvt->heading = 0;
  pvt->velocity = 0;
  pvt->have_position = 0;
  pvt->diff_status = DIFF_SOL_STATUS_NONE;
  pvt->pos_fusion_mode = FUSION_TYPE_INVALID;

  SYS_LOGGING(OBJ_PE, LOG_ERROR, "WARNING: Law limited usage!");
}


/***********************************************************************
* ��������: gnss_Pe_Accuracy_Cal
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�11/18/
***********************************************************************/
void gnss_Pe_Accuracy_Cal(USER_PVT* pvt)
{
  //uint8_t     temp = 0;
  float    accuracy;

  /* Accuracy calculation according to KF error */
  accuracy = (float)pvt->posErr.h * gpz_Ls->DoP.hDOP;

  if (gpz_Ls->pos_res > 1.0)
  {
    accuracy = (float)(gpz_Ls->pos_res) * gpz_Ls->DoP.hDOP / (float)2.0;
    if (accuracy <= 0)
    {
      return;
    }

    while (accuracy < 10)
    {
      accuracy = accuracy * 3;
    }
  }
  else
  {
    if (accuracy < 5)
    {
      accuracy = accuracy * 10;
    }
    else if (accuracy < 10)
    {
      accuracy = accuracy * 5;
    }
  }

  if (accuracy < 5)
  {
    accuracy += 5;
  }
  /*
  temp = (uint8_t)(accuracy / 5);
  accuracy = (float)(temp * 5);
  */
  pvt->accuracy = accuracy;
}
/***********************************************************************
* ��������: gnss_Pe_illegalArea_Dec
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�10/03/
***********************************************************************/
uint8_t gnss_Pe_illegalArea_Dec(double* llaPos)
{
  uint8_t               i, flag = FALSE;
  uint8_t               isInCNArea = FALSE, isInTWNArea = FALSE, isKORJAPArea = FALSE, isSEASIAArea = FALSE;
  double              TWN_LAT[2] = { 21.89121944,25.31141666 }, TWN_LON[2] = { 119.99230277,122.01197222 };
  double              CN_LAT[2] = { 18.13523888, 53.54999999 }, CN_LON[2] = { 73.66666666,135.04166666 };
  double              KOR_JAP_LAT[2] = { 25.93138888,38.81108055 }, KOR_JAP_LON[2] = { 124.33333333,148.11751111 };
  double              SE_ASIA_LAT[2] = { 8.51688333,21.08333333 }, SE_ASIA_LON[2] = { 66.54999999, 108.13250 };

  for (i = 0; i < 2; i++)
  {
    TWN_LAT[i] *= DEG2RAD;
    TWN_LON[i] *= DEG2RAD;
    CN_LAT[i] *= DEG2RAD;
    CN_LON[i] *= DEG2RAD;
    KOR_JAP_LAT[i] *= DEG2RAD;
    KOR_JAP_LON[i] *= DEG2RAD;
    SE_ASIA_LAT[i] *= DEG2RAD;
    SE_ASIA_LON[i] *= DEG2RAD;
  }

  if (llaPos == NULL)
  {
    return flag;
  }

  if (fabs(llaPos[0]) < 1e-5 && fabs(llaPos[1]) < 1e-5)
  {
    return flag;
  }

  if (llaPos[0] <= CN_LAT[1] && llaPos[0] >= CN_LAT[0] && llaPos[1] <= CN_LON[1] && llaPos[1] >= CN_LON[0])
  {
    isInCNArea = TRUE;
  }
  if (llaPos[0] <= TWN_LAT[1] && llaPos[0] >= TWN_LAT[0] && llaPos[1] <= TWN_LON[1] && llaPos[1] >= TWN_LON[0])
  {
    isInTWNArea = TRUE;
  }
  if (llaPos[0] <= KOR_JAP_LAT[1] && llaPos[0] >= KOR_JAP_LAT[0] && llaPos[1] <= KOR_JAP_LON[1] && llaPos[1] >= KOR_JAP_LON[0])
  {
    isKORJAPArea = TRUE;
  }
  if (llaPos[0] <= SE_ASIA_LAT[1] && llaPos[0] >= SE_ASIA_LAT[0] && llaPos[1] <= SE_ASIA_LON[1] && llaPos[1] >= SE_ASIA_LON[0])
  {
    isSEASIAArea = TRUE;
  }
  if (!isInCNArea || isInTWNArea || isKORJAPArea || isSEASIAArea)
  {
    flag = TRUE;
  }
  else
  {
    flag = FALSE;
  }

  return flag;
}
/***********************************************************************
* ��������: gnss_Pe_illegalArea
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�10/03/
***********************************************************************/
void gnss_Pe_illegalArea(KF_PVT_INFO kf_pvt, LS_PVT_INFO ls_pvt, uint8_t* kfFixState, uint8_t* lsFixState)
{
  uint8_t               area_flag, kf_flag, ls_flag;

  kf_flag = *kfFixState;
  ls_flag = *lsFixState;


  if (kf_flag)
  {
    area_flag = gnss_Pe_illegalArea_Dec(kf_pvt.kfLLApos);
    if (area_flag)
    {
      kf_flag = FALSE;
      illegalArea = TRUE;
    }
  }

  if (ls_flag)
  {
    area_flag = gnss_Pe_illegalArea_Dec(ls_pvt.LLApos);
    if (area_flag)
    {
      ls_flag = FALSE;
      illegalArea = TRUE;
    }
  }

  *kfFixState = kf_flag;
  *lsFixState = ls_flag;
}

/***********************************************************************
* ��������: gnss_pe_fill_fusion_mode
* �������: USER_PVT structure pointer
* �������: fill the pos_fusion_mode field in PE/RTK output.
* ����ֵ: none
* ���ߣ�
* ���ڣ�12/18/
***********************************************************************/
void gnss_pe_fill_fusion_mode(USER_PVT* pvt)
{
  switch (pvt->diff_status)
  {
  case DIFF_SOL_STATUS_STANDALONE:
  {
    pvt->pos_fusion_mode = FUSION_TYPE_GNSSONLY;
    break;
  }
  case DIFF_SOL_STATUS_RTD:
  {
    pvt->pos_fusion_mode = FUSION_TYPE_DGNSS;
    break;
  }
  case DIFF_SOL_STATUS_RTK_FLOAT:
  {
    pvt->pos_fusion_mode = FUSION_TYPE_RTK_FLOAT;
    break;
  }
  case DIFF_SOL_STATUS_RTK_FIX:
  {
    pvt->pos_fusion_mode = FUSION_TYPE_RTK_FIX;
    break;
  }
  case DIFF_SOL_STATUS_RTK_PROPAGATE:
  {
    pvt->pos_fusion_mode = FUSION_TYPE_RTK_PROPAGATE;
    break;
  }
  default:
  {
    pvt->pos_fusion_mode = FUSION_TYPE_INVALID;
    break;
  }
  }
}
/***********************************************************************
* ��������: gnss_Pe_Cal_HorizEllipseUnc
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�03/12/
***********************************************************************/
void gnss_Pe_Cal_HorizEllipseUnc(float varX, float varY, float varXY, USER_PVT* pvt)
{
  float  k, phi2;

  if (pvt == NULL)  return;

  pvt->ellipseUncSemiMajor = 0.0;
  pvt->ellipseUncSemiMinor = 0.0;
  pvt->ellipseUncOrientation = 0.0;

  if (varX < 1e-6 || varY < 1e-6)
  {
    return;
  }

  k = (float)sqrtf((varX - varY) * (varX - varY) + 4 * varXY * varXY);

  pvt->ellipseUncSemiMajor = (float)0.5 * (varX + varY + k);
  pvt->ellipseUncSemiMinor = (float)0.5 * (varX + varY - k);
  phi2 = (float)atan2(2.0 * varXY, ((double)varX - varY));
  if (phi2 < 0)
  {
    phi2 += (float)(2 * PI);
  }
  pvt->ellipseUncOrientation = (float)(phi2 / 2.0 * RAD2DEG);

  pvt->ellipseUncSemiMajor = (float)sqrt(pvt->ellipseUncSemiMajor);
  pvt->ellipseUncSemiMinor = (float)sqrt(pvt->ellipseUncSemiMinor);
}

/***********************************************************************
* ��������: gnss_Pe_PosErr_Calc
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�04/27/
***********************************************************************/
void gnss_Pe_PosErr_Calc(double* pos_ecef, double* ref_ecef, double* posErr)
{
  uint8_t                i;
  double               delta[3] = { 0.0 }, ref_lla[3] = { 0.0 }, posErr_enu[3] = { 0.0 }, dis = 0.0;

  for (i = 0; i < 3; i++)
  {
    delta[i] = pos_ecef[i] - ref_ecef[i];
    dis += delta[i] * delta[i];
  }

  dis = sqrt(dis);
  ecef2pos(ref_ecef, ref_lla);
  ecef2enu(ref_lla, delta, posErr_enu);
  for (i = 0; i < 3; i++)
  {
    posErr[i] = fabs(posErr_enu[i]);
  }
  GLOGD("distance:%f", dis);
}

/* spp solution confidence evaluate ----------------------------------------------------
*
* args   : none
* return :
* author : none
* note   : fits confidence of spp sol that not overwrited by rtk sol
*-------------------------------------------------------------------------------------*/
static void gnss_Pe_Spp_Confidence(USER_PVT* pvt)
{
  int32_t       i = 0;
  int32_t       satn = pvt->usedSvNum;
  int32_t       ue = peMode.userSceData.isUnderEleRoad;
  double       cfdhw[3] = { 0.0 };
  double       cfduw[3] = { 0.0 };
  /*kf pos_res*/
  double       skf = 100.00;
  double       sq2 = sqrt(2);
  /*overall horizontal/upward confidence*/
  const double overallhcfd[3] = { 3.4682,26.8835,58.6219 };
  const double overallucfd[3] = { 5.0203,17.5969,30.1745 };

  if (pvt->diff_status == DIFF_SOL_STATUS_NONE)
  {
    return;
  }

  skf = p_Kf->posRes;

  if (satn <= 7 && skf <= 1.0)
  {
    cfdhw[0] = 0.90; cfdhw[1] = 0.26; cfdhw[2] = 0.82;
    cfduw[0] = 0.50; cfduw[1] = 0.52; cfduw[2] = 0.62;
  }
  if (satn <= 7 && (skf > 1.0 && skf <= 2.2))
  {
    cfdhw[0] = 2.95; cfdhw[1] = 1.72; cfdhw[2] = 0.85;
    cfduw[0] = 0.97; cfduw[1] = 1.03; cfduw[2] = 0.86;
  }
  if (satn <= 7 && (skf > 2.2 && skf <= 4.9))
  {
    cfdhw[0] = 5.90; cfdhw[1] = 1.62; cfdhw[2] = 1.02;
    cfduw[0] = 1.22; cfduw[1] = 1.12; cfduw[2] = 0.84;
  }
  if (satn <= 7 && skf > 4.9)
  {
    cfdhw[0] = 11.54; cfdhw[1] = 2.29; cfdhw[2] = 1.49;
    cfduw[0] = 2.46; cfduw[1] = 1.04; cfduw[2] = 0.98;
  }
  if ((satn > 7 && satn <= 10) && skf <= 1.0)
  {
    cfdhw[0] = 0.63; cfdhw[1] = 0.16; cfdhw[2] = 0.11;
    cfduw[0] = 0.80; cfduw[1] = 0.34; cfduw[2] = 0.30;
  }
  if ((satn > 7 && satn <= 10) && (skf > 1.0 && skf <= 2.2) && ue == 0)
  {
    cfdhw[0] = 0.72; cfdhw[1] = 0.24; cfdhw[2] = 0.54;
    cfduw[0] = 0.84; cfduw[1] = 0.41; cfduw[2] = 0.47;
  }
  if ((satn > 7 && satn <= 10) && (skf > 2.2 && skf <= 4.9) && ue == 0)
  {
    cfdhw[0] = 1.25; cfdhw[1] = 0.64; cfdhw[2] = 0.48;
    cfduw[0] = 1.08; cfduw[1] = 0.90; cfduw[2] = 0.88;
  }
  if ((satn > 7 && satn <= 10) && (skf > 4.9 && skf <= 13.0) && ue == 0)
  {
    cfdhw[0] = 1.50; cfdhw[1] = 0.83; cfdhw[2] = 0.58;
    cfduw[0] = 1.19; cfduw[1] = 0.99; cfduw[2] = 0.96;
  }
  if ((satn > 7 && satn <= 10) && (skf > 1.0 && skf <= 2.2) && ue == 1)
  {
    cfdhw[0] = 1.43; cfdhw[1] = 0.49; cfdhw[2] = 0.51;
    cfduw[0] = 1.38; cfduw[1] = 0.67; cfduw[2] = 1.04;
  }
  if ((satn > 7 && satn <= 10) && (skf > 2.2 && skf <= 4.9) && ue == 1)
  {
    cfdhw[0] = 2.58; cfdhw[1] = 1.48; cfdhw[2] = 0.90;
    cfduw[0] = 4.08; cfduw[1] = 1.55; cfduw[2] = 1.04;
  }
  if ((satn > 7 && satn <= 10) && (skf > 4.9 && skf <= 13.0) && ue == 1)
  {
    cfdhw[0] = 9.01; cfdhw[1] = 1.83; cfdhw[2] = 0.97;
    cfduw[0] = 2.80; cfduw[1] = 1.42; cfduw[2] = 1.05;
  }
  if ((satn > 7 && satn <= 10) && skf > 13.0)
  {
    cfdhw[0] = 9.39; cfdhw[1] = 2.17; cfdhw[2] = 1.37;
    cfduw[0] = 2.09; cfduw[1] = 1.39; cfduw[2] = 1.10;
  }
  if ((satn > 10 && satn <= 14) && skf <= 1.0)
  {
    cfdhw[0] = 0.52; cfdhw[1] = 0.13; cfdhw[2] = 0.09;
    cfduw[0] = 0.50; cfduw[1] = 0.24; cfduw[2] = 0.27;
  }
  if ((satn > 10 && satn <= 14) && (skf > 1.0 && skf <= 2.2))
  {
    cfdhw[0] = 0.76; cfdhw[1] = 0.18; cfdhw[2] = 0.24;
    cfduw[0] = 0.60; cfduw[1] = 0.41; cfduw[2] = 0.38;
  }
  if ((satn > 10 && satn <= 14) && (skf > 2.2 && skf <= 13.0))
  {
    cfdhw[0] = 1.26; cfdhw[1] = 0.40; cfdhw[2] = 0.68;
    cfduw[0] = 1.88; cfduw[1] = 1.17; cfduw[2] = 1.00;
  }
  if ((satn > 10 && satn <= 14) && (skf > 13.0))
  {
    cfdhw[0] = 2.50; cfdhw[1] = 1.24; cfdhw[2] = 1.12;
    cfduw[0] = 2.08; cfduw[1] = 1.21; cfduw[2] = 1.10;
  }
  if (satn > 14 && skf <= 1.0)
  {
    cfdhw[0] = 0.57; cfdhw[1] = 0.10; cfdhw[2] = 0.06;
    cfduw[0] = 0.52; cfduw[1] = 0.21; cfduw[2] = 0.17;
  }
  if (satn > 14 && (skf > 1.0 && skf <= 2.2))
  {
    cfdhw[0] = 0.65; cfdhw[1] = 0.13; cfdhw[2] = 0.09;
    cfduw[0] = 0.46; cfduw[1] = 0.25; cfduw[2] = 0.27;
  }
  if (satn > 14 && (skf > 2.2 && skf <= 13.0))
  {
    cfdhw[0] = 1.74; cfdhw[1] = 0.33; cfdhw[2] = 0.20;
    cfduw[0] = 1.06; cfduw[1] = 0.70; cfduw[2] = 0.99;
  }
  if (satn > 14 && skf > 13.0)
  {
    cfdhw[0] = 2.15; cfdhw[1] = 0.39; cfdhw[2] = 0.41;
    cfduw[0] = 1.82; cfduw[1] = 1.14; cfduw[2] = 1.00;
  }
  pvt->accu.accu1_east = (float)(overallhcfd[0] * cfdhw[0] / sq2);
  pvt->accu.accu1_north = (float)(overallhcfd[0] * cfdhw[0] / sq2);
  pvt->accu.accu1_up = (float)(overallucfd[0] * cfduw[0]);

  pvt->accu.accu2_east = (float)(overallhcfd[1] * cfdhw[1] / sq2);
  pvt->accu.accu2_north = (float)(overallhcfd[1] * cfdhw[1] / sq2);
  pvt->accu.accu2_up = (float)(overallucfd[1] * cfduw[1]);

  pvt->accu.accu3_east = (float)(overallhcfd[2] * cfdhw[2] / sq2);
  pvt->accu.accu3_north = (float)(overallhcfd[2] * cfdhw[2] / sq2);
  pvt->accu.accu3_up = (float)(overallucfd[2] * cfduw[2]);
  pvt->accu.accu_type = 0;//fusion_mode_to_accu_indicator(pvt->pos_fusion_mode)
    pvt->accu.valid = TRUE;
}
#if 0
/*QX solution confidence analysis ----------------------------------------------------
*
* args   : none
* return :
* author : none
*-------------------------------------------------------------------------------------*/
static void gnss_pe_spp_confidence_analysis()
{
  double pos_err[3] = { 0.0 };
  if (gpz_Meas->hpRefQflag[1])
  {
    gnss_Pe_PosErr_Calc(gpz_user_pvt->ecef.pos, gpz_Meas->hpRefEcef, pos_err);
    if (gpz_user_pvt->diff_status == DIFF_SOL_STATUS_STANDALONE)
    {
      trace(3, "spp %10.4f herr %10.4f uerr %10.4f sigmals %10.4f sigmakf %10.4f satn %2d OS %d DT %d UE %d PDR %d hcdf %.4f %.4f %.4f ucdf %.4f %.4f %.4f\n", gpz_Meas->tor,
        sqrt(pos_err[0] * pos_err[0] + pos_err[1] * pos_err[1]), pos_err[2], gpz_Ls->pos_res, p_Kf->posRes, gpz_user_pvt->usedSvNum, peMode.userSceData.isOpenSky, peMode.userSceData.isDownTown,
        peMode.userSceData.isUnderEleRoad, peMode.userSceData.isPDRMode, sqrt(gpz_user_pvt->accu.accu1_east * gpz_user_pvt->accu.accu1_east + gpz_user_pvt->accu.accu1_north * gpz_user_pvt->accu.accu1_north),
        sqrt(gpz_user_pvt->accu.accu2_east * gpz_user_pvt->accu.accu2_east + gpz_user_pvt->accu.accu2_north * gpz_user_pvt->accu.accu2_north), sqrt(gpz_user_pvt->accu.accu3_east * gpz_user_pvt->accu.accu3_east + gpz_user_pvt->accu.accu3_north * gpz_user_pvt->accu.accu3_north),
        gpz_user_pvt->accu.accu1_up, gpz_user_pvt->accu.accu2_up, gpz_user_pvt->accu.accu3_up);
    }
  }
}
#endif

/***********************************************************************
* ��������: gnss_Pe_Pos_Confidence_Set
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�04/27/
***********************************************************************/
void gnss_Pe_Pos_Confidence_Set(USER_PVT* pvt)
{
  float                     acc1_horiz_min, acc2_horiz_min, acc3_horiz_min;
  float                     acc1_up_min, acc2_up_min, acc3_up_min;
#if defined(PLAYBACK_MODE)
  double                     pos_err[3] = { 0.0 };
#endif
  navigation_accuracy_t* pos_acc;

  if (pvt == NULL) return;

  pos_acc = &(pvt->accu);
  memset(pos_acc, 0, sizeof(navigation_accuracy_t));

  if (pvt->diff_status == DIFF_SOL_STATUS_STANDALONE || pvt->diff_status == DIFF_SOL_STATUS_RTK_PROPAGATE)
  {
    acc1_horiz_min = 3.0f;
    acc2_horiz_min = 10.0f;
    acc3_horiz_min = 25.0f;

    acc1_up_min = 5.0f;
    acc2_up_min = 15.0f;
    acc3_up_min = 30.0f;

  }
  else if (pvt->diff_status == DIFF_SOL_STATUS_RTD)
  {
    acc1_horiz_min = 1.0f;
    acc2_horiz_min = 3.0f;
    acc3_horiz_min = 10.0f;

    acc1_up_min = 1.5f;
    acc2_up_min = 4.5f;
    acc3_up_min = 15.0f;

  }
  else if (pvt->diff_status == DIFF_SOL_STATUS_RTK_FLOAT)
  {
    acc1_horiz_min = 0.5f;
    acc2_horiz_min = 2.0f;
    acc3_horiz_min = 7.5f;

    acc1_up_min = 1.0f;
    acc2_up_min = 3.0f;
    acc3_up_min = 10.0f;
  }
  else if (pvt->diff_status == DIFF_SOL_STATUS_RTK_FIX)
  {
    acc1_horiz_min = 0.1f;
    acc2_horiz_min = 1.0f;
    acc3_horiz_min = 5.0f;

    acc1_up_min = 0.2f;
    acc2_up_min = 1.5f;
    acc3_up_min = 7.5f;
  }
  else
  {
    return;
  }

  pos_acc->accu1_east = pvt->posErr.lon_err;
  pos_acc->accu1_north = pvt->posErr.lat_err;
  pos_acc->accu1_up = pvt->posErr.alt_err;
  if (pos_acc->accu1_east < acc1_horiz_min)  pos_acc->accu1_east = acc1_horiz_min;
  if (pos_acc->accu1_north < acc1_horiz_min)  pos_acc->accu1_north = acc1_horiz_min;
  if (pos_acc->accu1_up < acc1_up_min)  pos_acc->accu1_up = acc1_up_min;

  pos_acc->accu2_east = pvt->posErr.lon_err * 5;
  pos_acc->accu2_north = pvt->posErr.lat_err * 5;
  pos_acc->accu2_up = pvt->posErr.alt_err * 5;
  if (pos_acc->accu2_east < acc2_horiz_min)  pos_acc->accu2_east = acc2_horiz_min;
  if (pos_acc->accu2_north < acc2_horiz_min)  pos_acc->accu2_north = acc2_horiz_min;
  if (pos_acc->accu2_up < acc2_up_min)  pos_acc->accu2_up = acc2_up_min;

  pos_acc->accu3_east = pvt->posErr.lon_err * 10;
  pos_acc->accu3_north = pvt->posErr.lat_err * 10;
  pos_acc->accu3_up = pvt->posErr.alt_err * 10;
  if (pos_acc->accu3_east < acc3_horiz_min)  pos_acc->accu3_east = acc3_horiz_min;
  if (pos_acc->accu3_north < acc3_horiz_min)  pos_acc->accu3_north = acc3_horiz_min;
  if (pos_acc->accu3_up < acc3_up_min)  pos_acc->accu3_up = acc3_up_min;

  pos_acc->valid = TRUE;

#if defined(PLAYBACK_MODE)
  if (gpz_Meas->hpRefQflag[1])
  {
    gnss_Pe_PosErr_Calc(pvt->ecef.pos, gpz_Meas->hpRefEcef, pos_err);
    if (pvt->diff_status == DIFF_SOL_STATUS_STANDALONE)
    {
      gnss_util_trace(3, "spp %10.4f herr %10.4f uerr %10.4f sigmals %10.4f sigmakf %10.4f satn %2d OS %d DT %d UE %d PDR %d hcdf %.4f %.4f %.4f ucdf %.4f %.4f %.4f\n", gpz_Meas->tor,
        sqrt(pos_err[0] * pos_err[0] + pos_err[1] * pos_err[1]), pos_err[2], gpz_Ls->pos_res, p_Kf->posRes, pvt->usedSvNum, peMode.userSceData.isOpenSky, peMode.userSceData.isDownTown,
        peMode.userSceData.isUnderEleRoad, peMode.userSceData.isPDRMode, sqrtf(pvt->accu.accu1_east * pvt->accu.accu1_east + pvt->accu.accu1_north * pvt->accu.accu1_north),
        sqrtf(pvt->accu.accu2_east * pvt->accu.accu2_east + pvt->accu.accu2_north * pvt->accu.accu2_north), 
        sqrtf(pvt->accu.accu3_east * pvt->accu.accu3_east + pvt->accu.accu3_north * pvt->accu.accu3_north),
        pvt->accu.accu1_up, pvt->accu.accu2_up, pvt->accu.accu3_up);
    }
  }

  GLOGI("Pos Acc Set:Fix Type = %d %14.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f", pvt->diff_status, pvt->posfix_t,
    pos_acc->accu1_east, pos_acc->accu1_north, pos_acc->accu1_up,
    pos_acc->accu2_east, pos_acc->accu2_north, pos_acc->accu2_up,
    pos_acc->accu3_east, pos_acc->accu3_north, pos_acc->accu3_up,
    pos_err[0], pos_err[1], pos_err[2]);
#endif
}

/***********************************************************************
* ��������: gnss_Pe_Fill_MeasUse_Info
* ���������
* ���������
* ����ֵ��
* ���ߣ�none
* ���ڣ�06/27/
***********************************************************************/
void gnss_Pe_Fill_MeasUse_Info(meas_blk_t* pMeas, USER_PVT* pvt)
{
  uint8_t               i, flag;
  uint8_t               crNum = 0, goodCrNum = 0, prUseNum = 0, drUseNum = 0, measCnt1 = 0, cnt = 0;
  uint8_t               pr_res_num = 0;
  uint8_t               dr_res_num = 0;
  uint8_t               smallPrDiffNum = 0;
  uint64_t              pr_uesd_mask = 0, dr_uesd_mask = 0;
  float                 diffAvg = 0, avg = 0;
  float                 usedRes[MAX_MEAS_NUM] = { 0 };
  float                 pr_residual[MAX_MEAS_NUM] = { 0.0 };
  float                 dr_residual[MAX_MEAS_NUM] = { 0.0 };
  MEAS_USE_INFO* pMeasUseInfo;
  gnss_meas_t* pSvMeas;

  if (pMeas == NULL || pvt == NULL) return;

  pMeasUseInfo = &(pvt->meas_use_info);

  if (pvt->diff_status == DIFF_SOL_STATUS_NONE)
  {
    memset(pMeasUseInfo, 0, sizeof(MEAS_USE_INFO));
    return;
  }

  memset(&(pvt->meas_use_info), 0, sizeof(pvt->meas_use_info));
  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &(pMeas->meas[i]);
    if (!pSvMeas->prDiffValid)
    {
      continue;
    }

    if (fabs(pSvMeas->post_prdiff) > 1e-6)
    {
      pr_residual[pr_res_num] = pSvMeas->post_prdiff;
    }
    else
    {
      pr_residual[pr_res_num] = pSvMeas->pr_diff;
    }
    if (pSvMeas->status & 0x01)
    {
      pr_uesd_mask |= ((uint64_t)1 << pr_res_num);
      prUseNum++;
    }
    pr_res_num++;

    if (fabs(pSvMeas->post_drdiff) > 1e-6)
    {
      dr_residual[dr_res_num] = pSvMeas->post_drdiff;
    }
    else
    {
      dr_residual[dr_res_num] = pSvMeas->dr_diff;
    }
    if (pSvMeas->status & 0x2)
    {
      dr_uesd_mask |= ((uint64_t)1 << dr_res_num);
      drUseNum++;
    }
    dr_res_num++;

    if (pSvMeas->freq_index == 0)
    {
      measCnt1++;

      if (pSvMeas->cycleSlipCount == 0)
      {
        goodCrNum++;
      }

      if (pSvMeas->cycleSlipCount != 4)
      {
        crNum++;
      }
    }
  }


  /* 1.calculate small pr diff num */
  smallPrDiffNum = 0;
  pMeasUseInfo->smallPrDiffRate = 0.0;
  flag = gnss_median(pr_residual, pr_res_num, &diffAvg);
  if (flag)
  {
    for (i = 0; i < pr_res_num; i++)
    {
      if (fabs((double)pr_residual[i] - diffAvg) < 10.0)
      {
        smallPrDiffNum++;
      }
    }
  }

  if (pr_res_num > 0)
  {
    pMeasUseInfo->smallPrDiffRate = (float)smallPrDiffNum / (float)pr_res_num;
  }

  /* calculate gnss pr diff std */
  gnss_Pe_PrDiffStd_Calc_ForFlp(pr_residual, pr_res_num, &pMeasUseInfo->prDiffStd);

#if 0
  GLOGW("Para for FLP: tor %.3f, small rate %.3f, std %.3f, res %.3f, LS res %.3f; Para for GNSS: ele %d small %d, std %.3f,",
    pMeas->tor, pMeasUseInfo->smallPrDiffRate, pMeasUseInfo->prDiffStd, p_Kf->posRes, p_Kf->ls_Pvt_Info->pos_res, peMode.userSceData.isUnderEleRoad, pMeas->isSmallDiff, pMeas->prDiffStd_1);
#endif

  /* 2.calculated gnss pr diff std of gnss uesd sv */
  cnt = 0;
  for (i = 0; i < pr_res_num; i++)
  {
    if (pr_uesd_mask & ((uint64_t)1 << i))
    {
      usedRes[cnt++] = pr_residual[i];
    }
  }

  if (cnt > 1)
  {
    gnss_math_fstd(usedRes, cnt, &avg, &pMeasUseInfo->usedPrResStd);
  }
  else
  {
    pMeasUseInfo->usedPrResStd = -1.0;
  }

  /* 3. calculate gnss dr res std of gnss used sv */
  cnt = 0;
  for (i = 0; i < dr_res_num; i++)
  {
    if (dr_uesd_mask & ((uint64_t)1 << i))
    {
      usedRes[cnt++] = dr_residual[i];
    }
  }

  if (cnt > 1)
  {
    gnss_math_fstd(usedRes, cnt, &avg, &pMeasUseInfo->usedDrResStd);
  }
  else
  {
    pMeasUseInfo->usedDrResStd = -1.0;
  }

  pMeasUseInfo->cr_rate = (measCnt1 != 0) ? (float)crNum / (float)measCnt1 : 0;
  pMeasUseInfo->good_cr_rate = (measCnt1 != 0) ? (float)goodCrNum / (float)measCnt1 : 0;
  pMeasUseInfo->prNum = p_Kf->kf_ctrl.prNumFLP;
  pMeasUseInfo->prRejNum = p_Kf->kf_ctrl.prRejNum;
  pMeasUseInfo->prDewNum = p_Kf->kf_ctrl.prDewNum;
  pMeasUseInfo->drNum = p_Kf->kf_ctrl.drNum;
  pMeasUseInfo->drRejNum = p_Kf->kf_ctrl.drRejNum;
  pMeasUseInfo->drDewNum = p_Kf->kf_ctrl.drDewNum;
  pMeasUseInfo->posRes = p_Kf->posRes;
  pMeasUseInfo->velRes = p_Kf->velRes;
}

/***********************************************************************
* ��������: gnss_Pe_PosErr_Convert
* ���������
* ���������
* ����ֵ��
* ���ߣ�none
* ���ڣ�07/02/
***********************************************************************/
void gnss_Pe_PosErr_Convert(USER_PVT* pvt, float* posVar_xy)
{
  float         posVarX, posVarY;
  float         templ, errENU[3], horz_acc, posCovar, up_acc;

  if (pvt == NULL || posVar_xy == NULL) return;

  if (pvt->diff_status == DIFF_SOL_STATUS_NONE)
  {
    return;
  }

  posVarX = pvt->posErr.lon_err;
  posVarY = pvt->posErr.lat_err;
  templ = (float)sqrtf(posVarX * posVarX + posVarY * posVarY);
  if (templ < 1e-8 || !pvt->accu.valid)
  {
    *posVar_xy = 999.9f;
    pvt->accuracy = 999.9f;
    pvt->posErr.lat_err = 999.9f;
    pvt->posErr.lon_err = 999.9f;
    pvt->posErr.alt_err = 999.9f;
    pvt->posErr.h = 999.9f;
    pvt->posErr.v = 999.9f;
    pvt->posErr.p = 999.9f;
  }

  horz_acc = pvt->accu.accu1_east * (float)sqrt(2.0);
  up_acc = pvt->accu.accu1_up;

  errENU[0] = posVarX * horz_acc / templ;
  errENU[1] = posVarY * horz_acc / templ;
  errENU[2] = up_acc;

  posCovar = *posVar_xy;

  *posVar_xy = (float)(posCovar * (horz_acc / templ) * (horz_acc / templ));
  pvt->accuracy = horz_acc;
  pvt->posErr.lat_err = errENU[1];
  pvt->posErr.lon_err = errENU[0];
  pvt->posErr.alt_err = errENU[2];
  pvt->posErr.h = horz_acc;
  pvt->posErr.v = up_acc;
  pvt->posErr.p = (float)sqrtf(horz_acc * horz_acc + up_acc * up_acc);
}
/***********************************************************************
* ��������: gnss_Pe_Fill_PosFix
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�10/03/
***********************************************************************/
void gnss_Pe_Fill_PosFix(USER_PVT* pvt, meas_blk_t* pMeas)
{
  uint8_t           k;
  uint8_t           kfFixStatus = FALSE;
  uint8_t           lsFixStatus = FALSE;
  uint8_t           usedprnum = 0;
  float             posVarX, posVarY, posVarXY = 0.0;
  double            dt = 0.0, dt1 = 0.0;
  double            Nh;
  int64_t           dtInt;
  GNSS_TIME* pTime;
  uint8_t           measFlag = pMeas->invalidMeas;

  g_proc.dt = 0.0;
  g_pvtunc.vflag = 0;

  if (gpz_kf_pvt_data->kfFixStatus == FIX_STATUS_NEW && gpz_kf_pvt_data->kfFixSource >= FIX_SOURCE_APPX && !measFlag)
  {
    kfFixStatus = TRUE;
  }

  if (gpz_ls_pvt_data->fix_status == FIX_STATUS_NEW && gpz_ls_pvt_data->fix_dim == DIM_3D && !measFlag)
  {
    lsFixStatus = TRUE;
  }

  firstFix = gpz_kf_pvt_data->kf_had_firstFix;
  pvt->diff_age = rtdTimeGap;
  pvt->geoidal_sep = gnss_sd_get_geoId();

  pTime = gnss_tm_get_time();

  if (kfFixStatus == TRUE || lsFixStatus == TRUE)
  {
    dt1 = pTime->bias[GPS_MODE] / LIGHT_SEC;
    dt = pTime->rcvr_time[GPS_MODE] - dt1;
    dt *= 1e6;
    dtInt = (int64_t)dt;
    dtInt -= (int64_t)((int64_t)(dtInt / 1e5) * 1e5);
    if (dtInt < 5e4)
    {
      dt = -(double)dtInt / 1e6;
    }
    else
    {
      dt = (1e5 - (double)dtInt) / 1e6;
    }
  }
  g_proc.dt = dt;
  if (kfFixStatus == TRUE)
  {
    for (k = 0; k < 3; k++)
    {
      //pvt->lla.pos[k] = gpz_kf_pvt_data->kfLLApos[k];
      if ((g_proc.flag & 0xFC01) > 0)
      {
        if (((g_proc.last_flag & 0xFC01) == 0) && (g_proc.last_flag > 0))
        {
          g_proc.pos[k] = pvt->ecef.pos[k];
        }
        pvt->ecef.pos[k] = gpz_kf_pvt_data->ecefPos[k] + gpz_kf_pvt_data->ecefVel[k] * dt;
      }
      else
      {
        pvt->ecef.pos[k] += gpz_kf_pvt_data->ecefVel[k] * dt;
      }
      pvt->ecef.vel[k] = gpz_kf_pvt_data->ecefVel[k];
      pvt->lla.vel[k] = gpz_kf_pvt_data->kfENUvel[k];
    }
    gnssConvEcef2Lla(pvt->ecef.pos, pvt->lla.pos);

    //calculate altitude to Mean sea level
    Nh = gnss_sd_get_geoId();
    pvt->altitudeMsl = pvt->lla.pos[2] - Nh;

    // fix time
    pvt->posfix_t = pTime->rcvr_time[GPS_MODE] + dt;
    pvt->posfix_wn = pTime->week[GPS_MODE];
    if (pvt->posfix_t < 0.0)
    {
      pvt->posfix_t += 604800;
      pvt->posfix_wn--;
    }
    else if (pvt->posfix_t >= 604800)
    {
      pvt->posfix_t -= 604800;
      pvt->posfix_wn++;
    }
    // Fill ECEF position
    pvt->ecef.have_position = POS_FRM_KF;
    pvt->lla.have_position = POS_FRM_KF;
    pvt->ecef.posfix_t = pvt->posfix_t;
    pvt->ecef.posfix_wn = pvt->posfix_wn;

    // heading and speed
    pvt->heading = (float)(gpz_kf_pvt_data->kfHeadingDir * RAD2DEG);
    if (pvt->heading < 0.0)
    {
      pvt->heading += 360.0;
    }

    pvt->velocity = gpz_kf_pvt_data->kfHeadingVel;
    pvt->accuracy = gpz_kf_pvt_data->kfErrEst.h;

    // Error fill
    pvt->posErr = gpz_kf_pvt_data->kfErrEst;
    if (peMode.userSceData.isPDRMode)
    {
      pvt->accuracy = (float)p_Kf->posRes;
      pvt->posErr.lat_err = (float)(p_Kf->posRes / sqrt(2.0));
      pvt->posErr.lon_err = (float)(p_Kf->posRes / sqrt(2.0));
    }
    else
    {
      gnss_Pe_Accuracy_Cal(pvt);
    }

    posVarXY = gpz_kf_pvt_data->enPosCovar;

    //calculate horizon speed unc
    if (pvt->velocity > 1e-4)
    {
      pvt->velocityUnc = (pvt->lla.vel[0] * pvt->lla.vel[0] * pvt->posErr.ve_err * pvt->posErr.ve_err + pvt->lla.vel[1] * pvt->lla.vel[1] * pvt->posErr.vn_err * pvt->posErr.vn_err) / (pvt->lla.vel[0] * pvt->lla.vel[0] + pvt->lla.vel[1] * pvt->lla.vel[1]);
      pvt->velocityUnc = (float)sqrtf(pvt->velocityUnc);
    }
    else
    {
      pvt->velocityUnc = (float)sqrtf(pvt->posErr.ve_err * pvt->posErr.ve_err + pvt->posErr.vn_err * pvt->posErr.vn_err);
    }

    //calculate heading unc
    if (pvt->velocity > 0.5)
    {
      pvt->headingUnc = (pvt->lla.vel[1] * pvt->lla.vel[1] * pvt->posErr.ve_err * pvt->posErr.ve_err + pvt->lla.vel[0] * pvt->lla.vel[0] * pvt->posErr.vn_err * pvt->posErr.vn_err) / (pvt->lla.vel[0] * pvt->lla.vel[0] + pvt->lla.vel[1] * pvt->lla.vel[1]);
      //pvt->headingUnc /= (pvt->lla.vel[0] * pvt->lla.vel[0] + pvt->lla.vel[1] * pvt->lla.vel[1]);
      pvt->headingUnc = (float)(sqrtf(pvt->headingUnc) / 2.0 * RAD2DEG);
    }
    else
    {
      pvt->headingUnc = 359.9999f;
    }


    // total used satellites and average CNo
    pvt->usedSvNum = 0;
    pvt->fixedSvNum = 0;
    pvt->avgCNO = 0;
    pvt->CN040 = 0;
    for (k = 0; k < gpz_Meas->measCnt; k++)
    {
      if ((gpz_Meas->meas[k].status & 0x1) != 0)
      {
        usedprnum++;
        pvt->avgCNO += gpz_Meas->meas[k].cno;
      }
      if (gpz_Meas->meas[k].cno >= 40)
      {
        pvt->CN040++;
      }
    }
    if (usedprnum) pvt->avgCNO /= usedprnum;
    gnss_Pe_PRDR_Num(gpz_Meas);
    pvt->usedSvNum = gpz_Meas->validSatnum;
    //pvt->CN040 = gpz_Meas->Cno40Cnt;

    if (gpz_kf_pvt_data->kfFixSource == FIX_SOURCE_3D)
    {
      pvt->have_position = HAVE_POS_FIX3D;
    }
    else if (gpz_kf_pvt_data->kfFixSource == FIX_SOURCE_2D)
    {
      pvt->have_position = HAVE_POS_FIX2D;
    }
    else if (gpz_kf_pvt_data->kfFixSource == FIX_SOURCE_APPX)
    {
      pvt->have_position = HAVE_POS_APPX;
    }

    // calculate DOPs
    gnss_Pe_Dop(gpz_Meas, &(pvt->DOP), 1);
  }
  else if (lsFixStatus == TRUE)
  {
    SYS_LOGGING(OBJ_PE, LOG_INFO, "%14.6f LS VALID", pTime->tor);
    for (k = 0; k < 3; k++)
    {
      //pvt->lla.pos[k] = gpz_ls_pvt_data.lla[k];
      pvt->ecef.pos[k] = gpz_ls_pvt_data->ecefPos[k] + dt * gpz_ls_pvt_data->ecefVel[k];
      pvt->ecef.vel[k] = gpz_ls_pvt_data->ecefVel[k];
      pvt->lla.vel[k] = gpz_ls_pvt_data->enuVel[k];
    }
    gnssConvEcef2Lla(pvt->ecef.pos, pvt->lla.pos);

    //calculate altitude to Mean sea level
    Nh = gnss_sd_get_geoId();
    pvt->altitudeMsl = pvt->lla.pos[2] - Nh;

    // fix time
    pvt->posfix_t = pTime->rcvr_time[GPS_MODE] + dt;
    pvt->posfix_wn = pTime->week[GPS_MODE];

    // Fill ECEF position
    pvt->ecef.have_position = POS_FRM_LS;
    pvt->lla.have_position = POS_FRM_LS;
    pvt->ecef.posfix_t = pTime->rcvr_time[GPS_MODE] + dt;
    pvt->ecef.posfix_wn = pTime->week[GPS_MODE];

    // heading and speed
    pvt->heading = (float)(gpz_ls_pvt_data->heading * RAD2DEG);
    if (pvt->heading < 0.0)
    {
      pvt->heading += 360.0;
    }
    pvt->velocity = (float)sqrtf(gpz_ls_pvt_data->enuVel[0] * gpz_ls_pvt_data->enuVel[0] +
                                gpz_ls_pvt_data->enuVel[1] * gpz_ls_pvt_data->enuVel[1]);
    pvt->accuracy = gpz_ls_pvt_data->err.hErr;

    //calculate horizon speed unc
    if (pvt->velocity > 1e-4)
    {
      pvt->velocityUnc = (pvt->lla.vel[0] * pvt->lla.vel[0] * pvt->posErr.ve_err * pvt->posErr.ve_err + pvt->lla.vel[1] * pvt->lla.vel[1] * pvt->posErr.vn_err * pvt->posErr.vn_err) / (pvt->lla.vel[0] * pvt->lla.vel[0] + pvt->lla.vel[1] * pvt->lla.vel[1]);
      pvt->velocityUnc = (float)sqrt(pvt->velocityUnc);
    }
    else
    {
      pvt->velocityUnc = (float)sqrtf(pvt->posErr.ve_err * pvt->posErr.ve_err + pvt->posErr.vn_err * pvt->posErr.vn_err);
    }

    //calculate heading unc
    if (pvt->velocity > 0.5)
    {
      pvt->headingUnc = (pvt->lla.vel[1] * pvt->lla.vel[1] * pvt->posErr.ve_err * pvt->posErr.ve_err + pvt->lla.vel[0] * pvt->lla.vel[0] * pvt->posErr.vn_err * pvt->posErr.vn_err) / (pvt->lla.vel[0] * pvt->lla.vel[0] + pvt->lla.vel[1] * pvt->lla.vel[1]);
      //pvt->headingUnc /= (pvt->lla.vel[0] * pvt->lla.vel[0] + pvt->lla.vel[1] * pvt->lla.vel[1]);
      pvt->headingUnc = (float)(sqrt(pvt->headingUnc) / 2.0 * RAD2DEG);
    }
    else
    {
      pvt->headingUnc = 359.9999f;
    }

    //calculate horizontal elliptical uncertainty
    posVarX = gpz_ls_pvt_data->err.lonErr * gpz_ls_pvt_data->err.lonErr;
    posVarY = gpz_ls_pvt_data->err.latErr * gpz_ls_pvt_data->err.latErr;
    gnss_Pe_Cal_HorizEllipseUnc(posVarX, posVarY, 0, pvt);

    if (gpz_ls_pvt_data->fix_dim == DIM_3D)
    {
      pvt->have_position = HAVE_POS_FIX3D;
    }
    else if (gpz_ls_pvt_data->fix_dim == DIM_2D)
    {
      pvt->have_position = HAVE_POS_FIX2D;
    }
    // calculate DOPs
    gnss_Pe_Dop(gpz_Meas, &(pvt->DOP), 1);
  }
  else
  {
    //if (pTime->init)
    //{
    //	pvt->posfix_t = pTime->rcvr_time[GPS_MODE];
    //	pvt->posfix_wn = pTime->week[GPS_MODE];
    //}   

    /* set user position status to old if not init position */
    if (pvt->have_position > HAVE_POS_INIT)
    {
      pvt->have_position = HAVE_POS_OLD;
    }

#if 0
    /* check if need change position status to NONE */
    if (pvt->have_position > HAVE_POS_NONE && pTime->init == TRUE)
    {
      dt = pTime->rcvr_time[GPS_MODE] - pvt->posfix_t;
      dt += (pTime->week[GPS_MODE] - pvt->posfix_wn) * 604800.0;
      if (fabs(dt) > POSITION_HOLD_TIME)
      {
        pvt->have_position = HAVE_POS_NONE;
      }
    }
#endif

    pvt->ecef.have_position = POS_FRM_NULL;
    pvt->lla.have_position = POS_FRM_NULL;

    SYS_LOGGING(OBJ_PE, LOG_INFO, "%14.6f NO VALID POS(%d)", pTime->tor, firstFix);
  }

  pvt->enterTunnelFlag = peMode.tunnelMode.mode;
  pvt->exitTunnelFlag = peMode.tunnelMode.exitTunnelFlag;

  if (pvt->ecef.have_position == POS_FRM_KF)
  {
    pvt->diff_status = gpz_Meas->rtdUsed ? DIFF_SOL_STATUS_RTD : DIFF_SOL_STATUS_STANDALONE;
  }
  else
  {
    pvt->diff_status = DIFF_SOL_STATUS_NONE;
  }

  gnss_pe_fill_fusion_mode(pvt);

  /* set position fix confidence */
  //gnss_Pe_Spp_Confidence(pvt);
  gnss_pe_unc_predict(&g_pvtunc);

  /* re-calc pos err based based on pos confidence*/
  gnss_Pe_PosErr_Convert(pvt, &posVarXY);

  /*calculate horizontal elliptical uncertainty */
  if (pvt->diff_status != DIFF_SOL_STATUS_NONE)
  {
    posVarX = pvt->posErr.lon_err * pvt->posErr.lon_err;
    posVarY = pvt->posErr.lat_err * pvt->posErr.lat_err;
    gnss_Pe_Cal_HorizEllipseUnc(posVarX, posVarY, posVarXY, pvt);
  }

  /*fill gnss meas residual info*/
  gnss_Pe_Fill_MeasUse_Info(gpz_Meas, pvt);

  // time is used in RTCM decode
  if (pTime->init)
  {
    pTime->GPSTime = timeadd(pTime->GPSTime, -dt1);
    pTime->GPSTime = timeadd(pTime->GPSTime, dt);
    pvt->time = pTime->GPSTime;
    pvt->utcTime = gpst2utc(pvt->time);
    //pvt->timeStamp = (long long)((long long)pvt->utcTime.time * 1000 + pvt->utcTime.sec * 1000);
    pvt->timeStampOfUtc = (long long)((long long)pvt->utcTime.time * 1000 + pvt->utcTime.sec * 1000);
    SYS_LOGGING(OBJ_PE, LOG_INFO, "%s,%lld,%02d", __FUNCTION__, pvt->timeStampOfUtc, pvt->have_position);

    if (lsFixStatus && kfFixStatus)
    {
      {
        double dLat, dLon;
        dLat = (gpz_kf_pvt_data->kfLLApos[0] - gpz_ls_pvt_data->lla[0]) * (180 / PI) * 111319;
        dLon = (gpz_kf_pvt_data->kfLLApos[1] - gpz_ls_pvt_data->lla[1]) * (180 / PI) * 111133;
        SYS_LOGGING(OBJ_PE, LOG_INFO, "LSKF DIFF:%14.7f,%12.3f, %12.3f", pTime->rcvr_time[GPS_MODE], dLat, dLon);
      }
    }
  }

  // back information to GROM
  if (kfFixStatus == TRUE)
  {
    gnss_Pe_Get_PosFix(&user_pvt_backup);
    // save filter history information
    gnss_Pe_Update_HisInfo(&HisInfo);
  }
  // law check
  gnss_Pe_Law_Chck(pvt);
}

/***********************************************************************
* ��������: gnss_Pe_Find_SvMeas
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�05/10/
***********************************************************************/
uint8_t gnss_Pe_Find_SvMeas(uint8_t gnssMode, uint8_t prn)
{
  uint32_t    i;
  for (i = 0; i < gpz_Meas->measCnt; i++)
  {
    if (gpz_Meas->meas[i].gnssMode == gnssMode && gpz_Meas->meas[i].prn == prn)
    {
      return TRUE;
    }
  }
  return FALSE;
}
/***********************************************************************
* ��������: gnss_Pe_AbnormalTot_Dec
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�07/12/
***********************************************************************/
void gnss_Pe_AbnormalTot_Dec(meas_blk_t* pMeas, double* tot_save_gps, double* tot_save_bds)
{
  double                  tot_diff_1, tot_diff_2;

  tot_diff_1 = tot_save_gps[0] - tot_save_gps[1];
  tot_diff_2 = tot_save_bds[0] - tot_save_bds[1];

  if (fabs(tot_diff_1) > 0.1 || fabs(tot_diff_2) > 0.1)
  {
    return;
  }

  tot_diff_1 = tot_save_gps[0] - tot_save_bds[0];
  tot_diff_2 = tot_save_gps[1] - tot_save_bds[1];


  if (tot_diff_1 > 14.5 && tot_diff_2 > 14.5)
  {
    pMeas->invalidMeas = TRUE;
    SYS_LOGGING(OBJ_PE, LOG_WARNING, "Detect Wrong Tot Diff Between GPS and BDS at: %8.3f, gps Tot = %10.4f, bds Tot = %10.4f", pMeas->tor, tot_save_gps[0], tot_save_bds[0]);
  }
}

double gnss_Pe_Cal_EpochDt(meas_blk_t* pMeas)
{
  if (!pMeas) return 0.0;
  return gnss_calc_algo_epoch_dt(pMeas->tor, pMeas->last_tor);
}
/* adr convert to carrier phase -------------------------------------------------------------
*
* args   : none
* return :
* author : none
*-----------------------------------------------------------------------------*/
static void gnss_Pe_ADR_to_CarrierPhase(const GnssChannelMeas_t* pChannelMeas, gnss_meas_t* pSvMeas)
{
  int fcn;
  double lam;

  pSvMeas->carrierPhase = 0.0;
  if (g_pe_cfg.chipType != QCOM)
  {
    return;
  }

  fcn = (pSvMeas->gnssMode == GLN_MODE) ? gnss_Sd_Nm_GetGlnChanN(pSvMeas->prn) : 0;
  lam = ag_getwavelen(GNSS_MODE2SYS(pSvMeas->gnssMode, pSvMeas->prn), pSvMeas->freq_index, fcn);

  if (pSvMeas->gnssMode == GLN_MODE && lam == 0.0 && pChannelMeas->carrier_frequency_hz > 1.0E9)
  {
    lam = CLIGHT / pChannelMeas->carrier_frequency_hz;
  }

  if (lam <= 0.0) return;

  pSvMeas->carrierPhase = -pChannelMeas->accumulated_delta_range_m / lam;

  if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_BRCM_47755)
  {
    pSvMeas->carrierPhase = -pSvMeas->carrierPhase;
  }

#ifdef USED_IN_MC262M
  pSvMeas->carrierUnc = pChannelMeas->accumulated_delta_range_uncertainty_m;
#endif

  pSvMeas->cycleSlipCount = 0x3;
  if (pChannelMeas->accumulated_delta_range_state & AGGNSS_ADR_STATE_VALID)
  {
    pSvMeas->cycleSlipCount = 0;
  }
  else
  {
    return;
  }
  if (pChannelMeas->accumulated_delta_range_state & AGGNSS_ADR_STATE_CYCLE_SLIP)
  {
    pSvMeas->cycleSlipCount = 0x1;
  }
  if (!(pChannelMeas->accumulated_delta_range_state & AGGNSS_ADR_STATE_HALF_CYCLE_RESOLVED)) //for half cycle slip
  {
    pSvMeas->cycleSlipCount |= 0x2;
  }
  if (pChannelMeas->accumulated_delta_range_state & AGGNSS_ADR_STATE_RESET)
  {
    pSvMeas->cycleSlipCount = 0x3;
  }

}
/***********************************************************************
* ��������: gnss_Pe_Meas_Convert
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�10/14/
***********************************************************************/
#ifdef USED_IN_MC262M
static void gnss_Pe_Meas_Convert(const AGGnss_Data_t* pRawData, meas_blk_t* pMeas)
{
  //uint8_t                    flag = FALSE;
  uint8_t                    gpsTotSaveNum = 0, bdsTotSaveNum = 0;
  uint32_t 				  i, j;
  gnss_meas_t* pSvMeas;
  const AGGnss_Measurement_t* pSvRawData;
  const GnssChannelMeas_t* pChannelMeas;
  GNSS_TIME* pTime;
  double                   last_tor = pMeas->tor;
  double                   tot_save_gps[2] = { 0.0 }, tot_save_bds[2] = { 0.0 };//only save gps or bds tot;

#ifdef _WIN32
  struct _timeb timebuffer;
  _ftime_s(&timebuffer);
  gpz_user_pvt->timeStamp = timebuffer.time * 1000 + timebuffer.millitm;
#elif defined(GNUC_MACRO)
  struct timeval        tv;
  if (gettimeofday(&tv, NULL) == 0)
  {
    gpz_user_pvt->timeStamp = (long long)(tv.tv_sec * 1000.0 + tv.tv_usec / 1000.0);
  }
#else 
  gpz_user_pvt->timeStamp = 0;
#endif

  peContext.epochCnt++;          // calculate total measurements block number 

  pTime = gnss_tm_get_time();
  memset(pTime->torStatus, 0, GNSS_MAX_MODE * sizeof(uint8_t));

  memset(pMeas, 0, sizeof(meas_blk_t));
  pMeas->last_tor = last_tor;

  pMeas->tor = (double)pRawData->clock.time_ns * (double)(1e-9);
  pMeas->clock = pRawData->clock;
  pTime->time_ns = pRawData->clock.time_ns;

  //gpz_user_pvt->timeStamp_meas_inject = pRawData->clock.time_stamp;
  SYS_LOGGING(OBJ_PE, LOG_INFO, "GNSSCLOCK:%14.4f,%20.5f,%20.4f,%20d,%d", pRawData->clock.time_ns * (double)(1e-9), pRawData->clock.bias_ns * (double)(1e-9), pRawData->clock.drift_nsps, pMeas->clock.time_stamp, pRawData->measurement_count);

  for (i = 0; i < pRawData->measurement_count; i++)
  {
    pSvRawData = &pRawData->measurements[i];
#ifdef AG_GNSS_RTK_FUNCTION_IMPL
    for (j = 0; j < AGGNSS_MAX_FREQ_NUM; j++)
#else		
    for (j = 0; j < 1; j++)
#endif		   	
    {
      pChannelMeas = &pSvRawData->channel_meas[j];

      // CNO check
      if (fabs(pChannelMeas->c_n0_dbhz) < 0.001)
      {
        continue;
      }

      // GNSS satellite system check
      if ((pSvRawData->constellation == AGGNSS_CONSTELLATION_GPS && IS_MEAS_GPS_MASKED(&g_pe_cfg)) ||
        (pSvRawData->constellation == AGGNSS_CONSTELLATION_GLONASS && IS_MEAS_GLN_MASKED(&g_pe_cfg)) ||
        (pSvRawData->constellation == AGGNSS_CONSTELLATION_BEIDOU && IS_MEAS_BDS_MASKED(&g_pe_cfg)) ||
        (pSvRawData->constellation == AGGNSS_CONSTELLATION_GALILEO && IS_MEAS_GAL_MASKED(&g_pe_cfg)) ||
        (pSvRawData->constellation == AGGNSS_CONSTELLATION_QZSS && IS_MEAS_QZS_MASKED(&g_pe_cfg)))
      {
        continue;
      }

      /* reject QZSS measurement because of agnss server nonsupport, but use it in ppk mode */
#if 0
      if (pSvRawData->constellation == AGGNSS_CONSTELLATION_QZSS && g_pe_cfg.ppk_mode == 0)
      {
        continue;
      }
#endif
#if 0
      /* reject Beidou svid>14 measurement because of agnss server nonsupport */
      if (pSvRawData->constellation == AGGNSS_CONSTELLATION_BEIDOU)
      {
        if (pSvRawData->svid > 14)
        {
          continue;
        }
      }
#endif

      pSvMeas = &pMeas->meas[pMeas->measCnt++];

      pSvMeas->prn = (uint8_t)pSvRawData->svid;
      pSvMeas->freq_index = j;//freq_index
      pSvMeas->codeDetStd = (float)(pChannelMeas->received_sv_time_uncertainty_in_ns * LIGHT_SEC * (double)(1e-9));
      pSvMeas->dopplerStd = (float)pChannelMeas->pseudorange_rate_uncertainty_mps;

      if (pSvRawData->constellation == AGGNSS_CONSTELLATION_GPS ||
        pSvRawData->constellation == AGGNSS_CONSTELLATION_QZSS)
      {
        pSvMeas->gnssMode = GPS_MODE;
        if (pSvRawData->constellation == AGGNSS_CONSTELLATION_QZSS)
        {
          pSvMeas->sat_type = SBS_SAT;
        }
        else
        {
          pSvMeas->sat_type = MEO_SAT;
        }
      }
      else if (pSvRawData->constellation == AGGNSS_CONSTELLATION_GLONASS)
      {
        pSvMeas->sat_type = MEO_SAT;
        pSvMeas->gnssMode = GLN_MODE;
      }
      else if (pSvRawData->constellation == AGGNSS_CONSTELLATION_BEIDOU)
      {
        pSvMeas->gnssMode = BDS_MODE;
        if (pSvMeas->prn <= 5)
        {
          pSvMeas->sat_type = GEO_SAT;
        }
        else
        {
          pSvMeas->sat_type = MEO_SAT;
        }
      }
      else if (pSvRawData->constellation == AGGNSS_CONSTELLATION_GALILEO)
      {
        pSvMeas->gnssMode = GAL_MODE;
        pSvMeas->sat_type = MEO_SAT;
      }
      else
      {
        pMeas->measCnt--;
        SYS_LOGGING(OBJ_PE, LOG_ERROR, "Ab-normal meas constellation(%03d) prn(%03d)", pSvRawData->constellation, pSvRawData->svid);
        continue;
      }


      pSvMeas->tot = (double)pChannelMeas->received_sv_time_in_ns * (double)1e-9;
      pSvMeas->received_sv_time_in_ns = pChannelMeas->received_sv_time_in_ns;
      pSvMeas->cno = (uint32_t)(pChannelMeas->c_n0_dbhz + 0.5);
      pSvMeas->mpIndicator = pChannelMeas->multipath_indicator;
      pSvMeas->pseudoRangeRate = pChannelMeas->pseudorange_rate_mps;
#ifdef AG_GNSS_RTK_FUNCTION_IMPL
      if (fabs(pSvMeas->pseudoRangeRate) < 1.0e-4 &&
        (pSvRawData->flags & AGGNSS_MEASUREMENT_HAS_DOPPLER_SHIFT) &&
        fabs(pChannelMeas->doppler_shift_hz) > 1.0e-4)
      {
        int fcn = (pSvMeas->gnssMode == GLN_MODE) ? gnss_Sd_Nm_GetGlnChanN(pSvMeas->prn) : 0;
        pSvMeas->pseudoRangeRate = -pChannelMeas->doppler_shift_hz * ag_getwavelen(GNSS_MODE2SYS(pSvMeas->gnssMode, pSvMeas->prn), j, fcn);
      }
#endif
      pSvMeas->status = 0;

      if (pSvRawData->flags & AGGNSS_MEASUREMENT_HAS_PSEUDORANGE)
      {
        pSvMeas->status |= PE_MEAS_VALID_PR;
      }
      if ((pSvRawData->flags & AGGNSS_MEASUREMENT_HAS_DOPPLER_SHIFT) &&
        fabs(pSvMeas->pseudoRangeRate) > 1e-4)
      {
        pSvMeas->status |= PE_MEAS_VALID_DR;
      }

      pSvMeas->measState = pChannelMeas->state;
      pSvMeas->carrierFreqHz = pChannelMeas->carrier_frequency_hz;
      pSvMeas->carrierPhase = (double)pChannelMeas->carrier_cycles + pChannelMeas->carrier_phase;
      pSvMeas->carrierUnc = pChannelMeas->carrier_phase_uncertainty; //unit: cycle
      pSvMeas->cycleSlipCount = pChannelMeas->LLI & 0xFF;
#ifdef AG_GNSS_RTK_FUNCTION_IMPL
      if ((pSvRawData->flags & AGGNSS_MEASUREMENT_HAS_CARRIER_PHASE) == 0 && fabs(pChannelMeas->accumulated_delta_range_m) > 1.0e-3)
      {
        gnss_Pe_ADR_to_CarrierPhase(pChannelMeas, pSvMeas);
      }
#endif

      if ((pSvRawData->flags & (AGGNSS_MEASUREMENT_HAS_CARRIER_PHASE)) ||
        fabs(pSvMeas->carrierPhase) > 0.05)
      {
        pSvMeas->status |= PE_MEAS_VALID_L1;
      }
      if (pSvMeas->carrierPhase == 0.0)
      {
        pSvMeas->cycleSlipCount = 4;    //self defined
      }
      // check if any MP signal detected
      if (pSvMeas->mpIndicator == 1)
      {
        pMeas->isMPDetected = 1;
      }

      if (g_pe_cfg.chipType == SPRD || g_pe_cfg.chipType == UBLOX)
      {
        pSvMeas->usedInChip = pSvRawData->used_in_fix;

#if 0
        // If MP and not used, then reject this satellite
        if (pSvMeas->mpIndicator == 1 && pSvMeas->usedInChip == FALSE)
        {
          pSvMeas->status &= 0xFE;
          SYS_LOGGING(OBJ_PE, LOG_INFO, "MP Reject %02d %03d", pSvMeas->gnssMode, pSvMeas->prn);
        }
#endif

        pSvMeas->pseudoRange = pChannelMeas->pseudorange_m;
        GNSS_ASSERT(pSvMeas->pseudoRange > 0.0 && pSvMeas->pseudoRange < 1.0E9);
        if (!(g_pe_cfg.chipType == UBLOX && (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8090 || g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100)))
        {
          /* Wrong PR detected */
          if (pSvMeas->pseudoRange < 0.0 || pSvMeas->pseudoRange>1.0E9)
          {
            pSvMeas->status &= 0xF0;
          }
        }

        if (pSvMeas->gnssMode == GLN_MODE && pSvMeas->prn >= 93)
        {
          pSvMeas->status &= 0xF0;
        }

        /* tot state check */
        if (pSvMeas->gnssMode == GPS_MODE && (pSvMeas->measState & (AGGNSS_MEASUREMENT_STATE_TOW_DECODED)) == 0)
        {
          pSvMeas->status &= 0xF0;
        }
        else if (pSvMeas->gnssMode == GLN_MODE && (pSvMeas->measState & AGGNSS_MEASUREMENT_STATE_TOW_DECODED) == 0
          && (pSvMeas->measState & (AGGNSS_MEASUREMENT_STATE_GLO_TOD_DECODED)) == 0)
        {
          pSvMeas->status &= 0xF0;
        }
        else if (pSvMeas->gnssMode == BDS_MODE && (pSvMeas->measState & (AGGNSS_MEASUREMENT_STATE_TOW_DECODED)) == 0)
        {
          pSvMeas->status &= 0xF0;
        }
        else if (pSvMeas->gnssMode == GAL_MODE && (pSvMeas->measState & (AGGNSS_MEASUREMENT_STATE_TOW_DECODED)) == 0)
        {
          pSvMeas->status &= 0xF0;
        }

        if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_UBLOX_F9 /*|| g_pe_cfg.sub_chipType == SUB_CHIPTYPE_UBLOX_M8*/)
        {
          pSvMeas->codeDetStd = (float)pChannelMeas->pseudorange_uncertainty_m;
          pSvMeas->dopplerStd = (float)pChannelMeas->pseudorange_rate_uncertainty_mps;
        }

#if AGGNSS_MAX_FREQ_NUM == 1
        GLOGI("GNSS TOT:%02d %3d %16.10f %16.10f %16.4f %16.4f %18.3f %4d %3d", pSvMeas->gnssMode, pSvMeas->prn,
          pSvMeas->tot, pMeas->tor, pSvMeas->pseudoRange, pSvMeas->pseudoRangeRate, pSvMeas->carrierPhase, pSvMeas->cycleSlipCount, pSvMeas->cno);
#else
        if (pSvMeas->freq_index == 0) {
          GLOGI("TOT:%d%3d %10.3f %4lld %14.3f %14.3f %9.3f %9.3f %15.3f %15.3f %d %2d %d %2d %x %x %5.3f %5.3f %5d %5d", pSvMeas->gnssMode, pSvMeas->prn,
            pSvMeas->tot, pSvRawData->channel_meas[1].pseudorange_m < 0.001 ? 0 : pSvRawData->channel_meas[1].received_sv_time_in_ns - pSvRawData->channel_meas[0].received_sv_time_in_ns,
            pSvMeas->pseudoRange, pSvRawData->channel_meas[1].pseudorange_m,
            pSvMeas->pseudoRangeRate, pSvRawData->channel_meas[1].pseudorange_rate_mps,
            pSvMeas->carrierPhase, (double)pSvRawData->channel_meas[1].carrier_cycles + pSvRawData->channel_meas[1].carrier_phase,
            pSvMeas->cycleSlipCount, pSvMeas->cno, pSvRawData->channel_meas[1].carrier_phase == 0.0 ? 4 : (pSvRawData->channel_meas[1].LLI & 0xFF),
            (int32_t)(pSvRawData->channel_meas[1].c_n0_dbhz + 0.5),
            (pChannelMeas->LLI >> 8) & 0xFF, (pSvRawData->channel_meas[1].LLI >> 8) & 0xFF, pSvMeas->carrierUnc, pSvRawData->channel_meas[1].carrier_phase_uncertainty,
            pSvRawData->reserved1[1], pSvRawData->reserved1[2]);
        }
#endif
        // set tor status
        if ((pSvMeas->measState & (AGGNSS_MEASUREMENT_STATE_TOW_DECODED)) ||
          (pSvMeas->measState & (AGGNSS_MEASUREMENT_STATE_GLO_TOD_DECODED)))
        {
          pTime->torStatus[pSvMeas->gnssMode] = TRUE;
        }
      }
      else if (g_pe_cfg.chipType == QCOM)
      {
        pSvMeas->usedInChip = 1;

        /* GLONASS frequency number to svid */
        if (pSvMeas->gnssMode == GLN_MODE && pSvMeas->prn >= 93)
        {
          pSvMeas->status &= 0xF0;
        }
        /* tot state check */
        if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_BRCM_47755)
        {
          if ((pSvMeas->gnssMode == GPS_MODE || pSvMeas->gnssMode == BDS_MODE || pSvMeas->gnssMode == GAL_MODE) && (pSvMeas->measState & AGGNSS_MEASUREMENT_STATE_TOW_KNOWN))
          {
            pSvMeas->measState |= AGGNSS_MEASUREMENT_STATE_TOW_DECODED;
          }
          else if (pSvMeas->gnssMode == GLN_MODE && (pSvMeas->measState & AGGNSS_MEASUREMENT_STATE_GLO_TOD_KNOWN))
          {
            pSvMeas->measState |= AGGNSS_MEASUREMENT_STATE_GLO_TOD_DECODED;
          }
        }

        if (pSvMeas->gnssMode == GPS_MODE && (pSvMeas->measState & (AGGNSS_MEASUREMENT_STATE_TOW_DECODED)) == 0)
        {
          pSvMeas->status &= 0xF0;
        }
        else if (pSvMeas->gnssMode == GLN_MODE && (pSvMeas->measState & AGGNSS_MEASUREMENT_STATE_TOW_DECODED) == 0
          && (pSvMeas->measState & (AGGNSS_MEASUREMENT_STATE_GLO_TOD_DECODED)) == 0)
        {
          pSvMeas->status &= 0xF0;
        }
        else if (pSvMeas->gnssMode == BDS_MODE && (pSvMeas->measState & (AGGNSS_MEASUREMENT_STATE_TOW_DECODED)) == 0)
        {
          pSvMeas->status &= 0xF0;
        }
        else if (pSvMeas->gnssMode == GAL_MODE && (pSvMeas->measState & (AGGNSS_MEASUREMENT_STATE_TOW_DECODED)) == 0)
        {
          pSvMeas->status &= 0xF0;
        }

        if ((pSvMeas->status & 0x1) && (pSvMeas->gnssMode == GPS_MODE) && gpsTotSaveNum < 2)
        {
          tot_save_gps[gpsTotSaveNum] = pSvMeas->tot;
          gpsTotSaveNum++;
        }
        else if ((pSvMeas->status & 0x1) && (pSvMeas->gnssMode == BDS_MODE) && bdsTotSaveNum < 2)
        {
          tot_save_bds[bdsTotSaveNum] = pSvMeas->tot;
          bdsTotSaveNum++;
        }

        GLOGI("MEASLOG: %1d%3d %4x %14.6f %17.9f %d, %7.3f %7.3f %7.3f, %10.3f %16.3f %2d %2d", pSvMeas->gnssMode, pSvMeas->prn,
          pSvMeas->measState, pMeas->tor, pSvMeas->tot, pSvMeas->mpIndicator, fabs(pSvMeas->codeDetStd) > 1000.0 ? 0.0 : pSvMeas->codeDetStd,
          fabs(pSvMeas->dopplerStd) > 1000.0 ? 0.0 : pSvMeas->dopplerStd, fabs(pSvMeas->carrierUnc) > 1000.0 ? 0.0 : pSvMeas->carrierUnc,
          pSvMeas->pseudoRangeRate, pSvMeas->carrierPhase, pSvMeas->cno, pSvMeas->cycleSlipCount);


        // set tor status

        if ((pSvMeas->measState & (AGGNSS_MEASUREMENT_STATE_TOW_DECODED)) ||
          (pSvMeas->measState & (AGGNSS_MEASUREMENT_STATE_GLO_TOD_DECODED)))
        {
          pTime->torStatus[pSvMeas->gnssMode] = TRUE;
        }

      }
      else
      {

      }

      pSvMeas->pseudoRange_raw = pSvMeas->pseudoRange;
      pSvMeas->pseudoRangeRate_raw = pSvMeas->pseudoRangeRate;
      // cno calculate
      if ((pSvMeas->status == 0x3 || pSvMeas->status == 0xB) || (pSvMeas->status == 0x9 && !(g_pe_cfg.meas_type_mask & MEAS_TYPE_MASK_DR)))
      {
        if ((pSvMeas->cno >= 35) && (pSvMeas->freq_index < 1))
        {
          pMeas->Cno35Cnt[pSvMeas->gnssMode]++;
        }
        if (pSvMeas->cno >= 20)
        {
          pMeas->Cno20Cnt++;
        }
        if (pSvMeas->cno >= 45)
        {
          pMeas->Cno45Cnt++;
        }
      }
    }
  }
  if (gpsTotSaveNum == 2 && bdsTotSaveNum == 2)
  {
    gnss_Pe_AbnormalTot_Dec(pMeas, tot_save_gps, tot_save_bds);
  }
}
#endif


/***********************************************************************
* ��������: gnss_Pe_Bias_Monitor
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�04/01/
***********************************************************************/
void gnss_Pe_Bias_Monitor(Kf_t* p_KfBack)
{
  uint8_t			 i, satMode, cno35Cnt = 0;
  uint8_t			 goodMeas = FALSE;
  Ls_t* p_Ls;
  meas_blk_t* pMeas;
  float 		   ls_pos_thres;
  uint8_t			 ls_svnum_thres, ls_validnum_thres;
  double 		   UD_ISB_thres;

  p_Ls = p_KfBack->lsBlk;
  pMeas = p_KfBack->meas_blk;

  ls_pos_thres = 10.0;
  ls_svnum_thres = 5;
  ls_validnum_thres = 5;
  UD_ISB_thres = 30.0;

  gnss_pe_cal_prdiff_std(pMeas);

  for (i = 0; i < GNSS_MAX_MODE; i++)
  {
    cno35Cnt += pMeas->Cno35Cnt[i];
  }
  if (pMeas->avgCno > 35 && cno35Cnt > 8 && pMeas->prDiffStd_2 < 5)
  {
    goodMeas = TRUE;
  }

  if (!firstFix && goodMeas && p_KfBack->kfCnt > 1000)
  {
    ls_pos_thres = 40.0;
    ls_svnum_thres = 3;
    ls_validnum_thres = 3;
    UD_ISB_thres = 20;
  }

  // 1. check bias number 
  if (p_Ls->lsCtrl.lsBiasNum < 2 && (p_Ls->lsCtrl.prNum - p_Ls->lsCtrl.lsBiasNum) < 6)
  {
    return;
  }

  // 2. check LS POS status 
  if (p_Ls->posCvg == FALSE)
  {
    return;
  }

  // 3. check position residual 
  if (p_Ls->pos_res > ls_pos_thres)
  {
    return;
  }


  // 4. Check valid GPS number
  if (p_Ls->lsCtrl.validSvNum[GPS_MODE] < ls_svnum_thres)
  {
    return;
  }

  // 5. average cno check
  if (pMeas->avgCno < 28 || cno35Cnt < 4)
  {
    return;
  }

  // 6. calculate biasDiff
  if (pMeas->rtdUsed)
  {
    for (satMode = GLN_MODE; satMode < GNSS_MAX_MODE; satMode++)
    {
      if (p_Ls->lsCtrl.validSvNum[satMode] < 5) continue;
      if (p_Ls->ls_Pvt_Info->biasValid[satMode] == 0) continue;

      if (g_pe_cfg.chipType == QCOM)
      {
        if (fabs(p_Ls->bias[GPS_MODE] - p_Ls->bias[satMode]) > 2000 && satMode == GLN_MODE) continue;
        if (fabs(p_Ls->bias[GPS_MODE] - p_Ls->bias[satMode]) > 1200 && satMode == BDS_MODE) continue;
      }

      if (p_KfBack->biasDiff[satMode] == 0.0 ||
        (fabs(p_Ls->bias[GPS_MODE] - p_Ls->bias[satMode] - p_KfBack->biasDiff[satMode]) > 20 && goodMeas && g_pe_cfg.chipType == SPRD))
      {
        p_KfBack->biasDiff[satMode] = p_Ls->bias[GPS_MODE] - p_Ls->bias[satMode];
      }
      else
      {
        p_KfBack->biasDiff[satMode] = p_KfBack->biasDiff[satMode] * 31.0 / 32.0 + (p_Ls->bias[GPS_MODE] - p_Ls->bias[satMode]) / 32.0;
      }

      p_KfBack->biasDiffUpCnt[satMode]++;
      if (p_KfBack->biasDiffUpCnt[satMode] > BIASDIFF_UPCNT_THRES)
      {
        p_KfBack->biasDiffUpCnt[satMode] = BIASDIFF_UPCNT_THRES;
      }
    }

    SYS_LOGGING(OBJ_KF, LOG_INFO, "%s,RTD: %14.6f,%14.6f,%14.6f,%14.6f,%14.6f,%14.6f", __FUNCTION__, (p_Ls->bias[GPS_MODE] - p_Ls->bias[GLN_MODE]), p_KfBack->biasDiff[GLN_MODE], kfInfo.X[6] - kfInfo.X[7],
      (p_Ls->bias[GPS_MODE] - p_Ls->bias[BDS_MODE]), p_KfBack->biasDiff[BDS_MODE], kfInfo.X[6] - kfInfo.X[8]);
  }

  for (satMode = GLN_MODE; satMode < GNSS_MAX_MODE; satMode++)
  {
    if (p_Ls->lsCtrl.validSvNum[satMode] < ls_validnum_thres) continue;
    if (p_Ls->ls_Pvt_Info->biasValid[satMode] == 0) continue;
    if (pMeas->prDiffStd_3[satMode] > 7.5 || pMeas->prDiffStd_3[satMode] < 0.0) continue;
    if (g_pe_cfg.chipType == QCOM)
    {
      if (fabs(p_Ls->biasLocal[GPS_MODE] - p_Ls->biasLocal[satMode]) > 2000 && satMode == GLN_MODE) continue;
      if (fabs(p_Ls->biasLocal[GPS_MODE] - p_Ls->biasLocal[satMode]) > 1200 && satMode == BDS_MODE) continue;
    }
    if (p_KfBack->biasDiffLocal[satMode] == 0.0 ||
      (fabs(p_Ls->biasLocal[GPS_MODE] - p_Ls->biasLocal[satMode] - p_KfBack->biasDiffLocal[satMode]) > 20 && goodMeas && g_pe_cfg.chipType == SPRD))
    {
      p_KfBack->biasDiffLocal[satMode] = p_Ls->biasLocal[GPS_MODE] - p_Ls->biasLocal[satMode];
    }
    else
    {
      if (((p_KfBack->biasDiffLocalUpCnt[satMode] == 0) || (!firstFix)) && ((fabs(p_KfBack->biasDiffLocal[satMode]) < DBL_EPSILON) || (fabs(p_Ls->biasLocal[GPS_MODE] - p_Ls->biasLocal[satMode] - p_KfBack->biasDiffLocal[satMode]) > UD_ISB_thres)))
      {
        p_KfBack->biasDiffLocal[satMode] = p_Ls->biasLocal[GPS_MODE] - p_Ls->biasLocal[satMode];
        SYS_LOGGING(OBJ_KF, LOG_INFO, "%s,use calculated bias to initialize biasDiffLocal:%d %f", __FUNCTION__, satMode, p_KfBack->biasDiffLocal[satMode]);
      }
      if (firstFix)
      {
        if (p_KfBack->kfCnt < 10)
        {
          p_KfBack->biasDiffLocal[satMode] = p_KfBack->biasDiffLocal[satMode] * 7.0 / 8.0 + (p_Ls->biasLocal[GPS_MODE] - p_Ls->biasLocal[satMode]) / 8.0;
        }
        else if (p_KfBack->kfCnt < 20)
        {
          p_KfBack->biasDiffLocal[satMode] = p_KfBack->biasDiffLocal[satMode] * 15.0 / 16.0 + (p_Ls->biasLocal[GPS_MODE] - p_Ls->biasLocal[satMode]) / 16.0;
        }
        else
        {
          p_KfBack->biasDiffLocal[satMode] = p_KfBack->biasDiffLocal[satMode] * 31.0 / 32.0 + (p_Ls->biasLocal[GPS_MODE] - p_Ls->biasLocal[satMode]) / 32.0;
        }
      }
      else /*if(goodMeas)*/
      {
        p_KfBack->biasDiffLocal[satMode] = p_KfBack->biasDiffLocal[satMode] * 1.0 / 2.0 + (p_Ls->biasLocal[GPS_MODE] - p_Ls->biasLocal[satMode]) / 2.0;
      }
    }

    p_KfBack->biasDiffLocalUpCnt[satMode]++;
    if (p_KfBack->biasDiffLocalUpCnt[satMode] > BIASDIFF_UPCNT_THRES)
    {
      p_KfBack->biasDiffLocalUpCnt[satMode] = BIASDIFF_UPCNT_THRES;
    }
  }
  SYS_LOGGING(OBJ_KF, LOG_INFO, "%s,LOC: %14.6f,%14.6f,%14.6f,%14.6f,%14.6f,%14.6f", __FUNCTION__,
    (p_Ls->biasLocal[GPS_MODE] - p_Ls->biasLocal[GLN_MODE]), p_KfBack->biasDiffLocal[GLN_MODE], kfInfo.X[6] - kfInfo.X[7],
    (p_Ls->biasLocal[GPS_MODE] - p_Ls->biasLocal[BDS_MODE]), p_KfBack->biasDiffLocal[BDS_MODE], kfInfo.X[6] - kfInfo.X[8]);

}


void gnss_Pe_Set_LsBiasDiff(Kf_t* p_KfBack, Ls_t* p_Ls)
{
  memcpy(p_Ls->biasDiff, p_KfBack->biasDiff, 4 * sizeof(double));
  memcpy(p_Ls->biasDiffLocal, p_KfBack->biasDiffLocal, 4 * sizeof(double));
  memcpy(p_Ls->biasDiffUpCnt, p_KfBack->biasDiffUpCnt, 4 * sizeof(uint8_t));
  memcpy(p_Ls->biasDiffLocalUpCnt, p_KfBack->biasDiffLocalUpCnt, 4 * sizeof(uint8_t));
}
/***********************************************************************
* ��������: gnss_Pe_PosBias_Detect_KfRes
* ���������
* ���������
* ����ֵ��
* ���ڣ�04/24/
***********************************************************************/
void gnss_Pe_PosBias_Detect_KfRes(Kf_t* p_KfBack, PosBiasDet_KfRes* pKfResDec)
{
  uint8_t                i;
  double                 posResSmooth = 0;

  if (p_KfBack == NULL || pKfResDec == NULL)
  {
    return;
  }

  pKfResDec->posIsBias = FALSE;

  if (p_KfBack->kf_Pvt_Info->kfFixStatus != FIX_STATUS_NEW || p_KfBack->posResStd < 0)
  {
    pKfResDec->posResCnt = 0;
    pKfResDec->largePosResStdCnt = 0;
    return;
  }

  //smooth KF pos residual std
  if (pKfResDec->posResCnt < MAX_SMOOTH_NUM)
  {
    pKfResDec->posResBack[pKfResDec->posResCnt] = p_KfBack->posResStd;
    pKfResDec->posResCnt++;
  }
  else
  {
    for (i = 0; i < MAX_SMOOTH_NUM - 1; i++)
    {
      pKfResDec->posResBack[i] = pKfResDec->posResBack[i + 1];
    }

    pKfResDec->posResBack[MAX_SMOOTH_NUM - 1] = p_KfBack->posResStd;
  }

  if (pKfResDec->posResCnt == MAX_SMOOTH_NUM)
  {
    posResSmooth = gnssClcAvg_DBL(pKfResDec->posResBack, MAX_SMOOTH_NUM);
    pKfResDec->smoothPosResStd = posResSmooth;
  }
  else
  {
    pKfResDec->smoothPosResStd = p_KfBack->posResStd;
  }

  if (pKfResDec->smoothPosResStd > 50.0 && p_KfBack->posRes > 50 &&
    !peMode.staticData.staticFlag)
  {
    pKfResDec->largePosResStdCnt++;
  }
  else
  {
    pKfResDec->largePosResStdCnt = 0;
  }

  if (pKfResDec->largePosResStdCnt >= 5 &&
    ((pKfResDec->posResetTime > 0 && (p_KfBack->kf_Pvt_Info->tor - pKfResDec->posResetTime) > 30) || (pKfResDec->posResetTime == 0)))
  {
    pKfResDec->posIsBias = TRUE;
    pKfResDec->posResetTime = p_KfBack->kf_Pvt_Info->tor;
    pKfResDec->largePosResStdCnt = 0;
    SYS_LOGGING(OBJ_PE, LOG_ERROR, "Detect large position bias with KF Res: %8.3f", pKfResDec->posResetTime);
  }

  /*if(pKf_PosBias->posHasBias)
  {
  p_Kf->enlargeQ = TRUE;
  p_Kf->QFactor = (4.0*pKf_PosBias->smoothPosResStd/35.0)*(4.0*pKf_PosBias->smoothPosResStd/35.0);
  if(p_Kf->QFactor>100)
  {
  p_Kf->QFactor = 100;
  }
  pKf_PosBias->posHasBias = FALSE;
  }*/
}
/***********************************************************************
* ��������: gnss_Pe_PosBias_Detect_PrDiff
* ���������
* ���������
* ����ֵ��
* ���ڣ�08/30/
***********************************************************************/
void gnss_Pe_PosBias_Detect_PrDiff(meas_blk_t* pMeas, PosBiasDet_PrDiff* prDiffDec, Kf_t* p_KfBack)
{
  if (prDiffDec == NULL || pMeas == NULL)
  {
    return;
  }

  prDiffDec->posIsBias = FALSE;

  if (pMeas->prDiffStd_2 < 0)
  {
    prDiffDec->prDiffBackCnt = 0;
    prDiffDec->smoothPrDiffStd = -1.0;
  }
  else
  {
    gnss_math_data_smooth(pMeas->prDiffStd_2, prDiffDec->prDiffBack, &(prDiffDec->prDiffBackCnt), MAX_SMOOTH_NUM, &(prDiffDec->smoothPrDiffStd));
  }

  if (pMeas->avgPrDiffDis < 0)
  {
    prDiffDec->prDisBackCnt = 0;
    prDiffDec->smoothPrDisAvg = -1.0;
  }
  else
  {
    gnss_math_data_smooth(pMeas->avgPrDiffDis, prDiffDec->prDisBack, &(prDiffDec->prDisBackCnt), MAX_SMOOTH_NUM, &(prDiffDec->smoothPrDisAvg));
  }

  if (pMeas->prDiffStd_a < 0)
  {
    prDiffDec->prDiffBackCnt_a = 0;
    prDiffDec->smoothPrDiffStd_a = -1.0;
  }
  else
  {
    gnss_math_data_smooth(pMeas->prDiffStd_a, prDiffDec->prDiffBack_a, &(prDiffDec->prDiffBackCnt_a), MAX_SMOOTH_NUM, &(prDiffDec->smoothPrDiffStd_a));
  }

  if (prDiffDec->smoothPrDiffStd > 70 && prDiffDec->smoothPrDisAvg > 60 &&
    prDiffDec->smoothPrDiffStd_a > 30 && !peMode.staticData.staticFlag && (p_KfBack->kf_ctrl.prRejNum + p_KfBack->kf_ctrl.prDewNum) > 0)
  {
    prDiffDec->largeDiffCnt = prDiffDec->largeDiffCnt + 2;
  }
  else if (((prDiffDec->smoothPrDiffStd > 55 && prDiffDec->smoothPrDisAvg > 40) ||
    (prDiffDec->smoothPrDiffStd > 50 && prDiffDec->smoothPrDisAvg > 50)) &&
    prDiffDec->smoothPrDiffStd_a > 20 && !peMode.staticData.staticFlag && (p_KfBack->kf_ctrl.prRejNum + p_KfBack->kf_ctrl.prDewNum) > 0)
  {
    if (prDiffDec->largeDiffCnt < 5)
    {
      prDiffDec->largeDiffCnt++;
    }
  }
  else

  {
    prDiffDec->largeDiffCnt = 0;
  }

  /*if(p_Kf->kf_Pvt_Info->kfFixStatus == FIX_STATUS_NEW && ref_pos[0] > 0)*/
  {
    //pos_err = gnssCalPosDis(p_Kf->kf_Pvt_Info->kfLLApos,ref_pos,TRUE);

    SYS_LOGGING(OBJ_PE, LOG_INFO, "PR diff std: %8.3f %10.4f %10.4f %10.4f %10.4f", pMeas->tor, pMeas->prDiffStd_2, prDiffDec->smoothPrDiffStd, prDiffDec->smoothPrDisAvg, prDiffDec->smoothPrDiffStd_a);
  }

  if (prDiffDec->largeDiffCnt >= 5 && ((prDiffDec->decEpoch > 0 &&
    (pMeas->tor - prDiffDec->decEpoch) > 30) || (prDiffDec->decEpoch == 0)))
  {
    prDiffDec->posIsBias = TRUE;
    prDiffDec->decEpoch = pMeas->tor;
    prDiffDec->largeDiffCnt = 0;

    SYS_LOGGING(OBJ_PE, LOG_ERROR, "Detect large position bias with abnormal pr diff: %8.3f", prDiffDec->decEpoch);
  }
}

/***********************************************************************
* ��������: gnss_Pe_PosBias_Detect_KfState
* ���������
* ���������
* ����ֵ��
* ���ڣ�08/28/
***********************************************************************/
void gnss_Pe_PosBias_Detect_KfState(Kf_t* p_KfState, PosBiasDet_KfState* kfStateDec)
{
  uint8_t    measBadNum = 0;
  uint8_t    posBiasFlag = FALSE;

  measBadNum = p_KfState->kf_ctrl.prRejNum + p_KfState->kf_ctrl.prDewNum;

  kfStateDec->posIsBias = FALSE;

  if (p_KfState->kf_Pvt_Info->kfFixStatus == FIX_STATUS_NEW &&
    (double)((double)measBadNum / p_KfState->kf_ctrl.prNum) > 0.6 && p_KfState->kf_ctrl.prNum > 5 && !peMode.staticData.staticFlag)
  {
    if (kfStateDec->abKfStateCnt < 5)
    {
      kfStateDec->abKfStateCnt++;
    }
  }
  else
  {
    kfStateDec->abKfStateCnt = 0;
  }

  if (kfStateDec->abKfStateCnt >= 5 && ((kfStateDec->decEpoch > 0 && (p_KfState->meas_blk->tor - kfStateDec->decEpoch) > 30) || (kfStateDec->decEpoch == 0)))
  {
    kfStateDec->posIsBias = TRUE;
    kfStateDec->decEpoch = p_KfState->meas_blk->tor;
    kfStateDec->abKfStateCnt = 0;

    SYS_LOGGING(OBJ_PE, LOG_ERROR, "Detect large position bias with abnormal KF state: %8.3f", kfStateDec->decEpoch);
  }
}
#if 0
/***********************************************************************
* ��������: gnss_Pe_PosBias_Detect_GoodMeas
* ���������
* ���������
* ����ֵ��
* ���ߣ�jh
* ���ڣ�08/28/
***********************************************************************/
void gnss_Pe_PosBias_Detect_GoodMeas(meas_blk_t* pMeas, Kf_t* p_Kf, PosBiasDet_GoodMeas* goodMeasDec)
{
  uint8_t                   goodMeasCnt = 0, index[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 }, biasDiffUpdate[GNSS_MAX_MODE];
  uint32_t                  i, cnoThres;
  float                  prDiff[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 };
  gnss_meas_t* pSvMeas;
  sat_data_t* sp;

  if (peMode.staticData.staticFlag)
  {
    return;
  }

  for (i = GLN_MODE; i < GNSS_MAX_MODE; i++)
  {
    if (pMeas->rtdUsed && p_Kf->biasDiffUpCnt[i] >= BIASDIFF_UPCNT_THRES)
    {
      biasDiffUpdate[i] = TRUE;
    }
    else if (!pMeas->rtdUsed && p_Kf->biasDiffLocalUpCnt[i] >= BIASDIFF_UPCNT_THRES)
    {
      biasDiffUpdate[i] = TRUE;
    }
  }

  if (peMode.userSceData.isDownTown || peMode.userSceData.isUnderEleRoad)
  {
    cnoThres = 24;
  }
  else
  {
    cnoThres = 28;
  }

  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &(pMeas->meas[i]);
    if (pSvMeas->prn == 0 || pSvMeas->status == 0) continue;
    if ((pSvMeas->status & 0x1) == 0) continue;
    sp = gnss_sd_get_sv_data(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);
    if (sp == NULL) continue;

    if (pSvMeas->cno >= cnoThres && pSvMeas->stdPRDRDiff > 0 && pSvMeas->stdPRDRDiff < 5.0
      && fabs(pSvMeas->sumPRDRDiff) > 0 && fabs(pSvMeas->sumPRDRDiff) < 15 && sp->prR < 40)
    {
      if (pSvMeas->gnssMode != GPS_MODE && !biasDiffUpdate[pSvMeas->gnssMode])
      {
        continue;
      }
      index[goodMeasCnt] = i;
      prDiff[goodMeasCnt] = pSvMeas->pr_diff;
      goodMeasCnt++;
      SYS_LOGGING(OBJ_PE, LOG_INFO, "PE Detect Good Meas: %14.6f,%02d,%02d,%02d,%10.4f,%10.4f,%10.4f,%10.4f", pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->cno, pSvMeas->stdPRDRDiff, pSvMeas->sumPRDRDiff, pSvMeas->pr_diff, pSvMeas->pr_err);
    }
  }
}
#endif
/***********************************************************************
* ��������: gnss_Pe_PosRecover_Check
* ���������
* ���������
* ����ֵ��
* ���ߣ�jh
* ���ڣ�08/28/
***********************************************************************/
void gnss_Pe_PosRecover_Check(double  tor, PeStateMonitor_t* p_state, PosBiasDet_PrDiff* prDiffDec, PosBiasDet_KfRes* pKfResDec)
{
  if (!p_state->posHasBias)
  {
    p_state->posRecoverCnt = 0;
    return;
  }

  if (prDiffDec->smoothPrDiffStd > 0 && prDiffDec->smoothPrDiffStd < 40 &&
    prDiffDec->smoothPrDisAvg > 0 && prDiffDec->smoothPrDisAvg < 30 &&
    prDiffDec->smoothPrDiffStd_a > 0 && prDiffDec->smoothPrDiffStd_a < 15)
  {
    p_state->posRecoverCnt++;
  }
  else if (pKfResDec->smoothPosResStd > 0 && pKfResDec->smoothPosResStd < 20)
  {
    p_state->posRecoverCnt++;
  }
  else
  {
    p_state->posRecoverCnt = 0;
  }

  if (p_state->posRecoverCnt >= 5)
  {
    p_state->posHasBias = FALSE;
    p_state->posRecoverCnt = 0;
    SYS_LOGGING(OBJ_PE, LOG_INFO, "Pos Bias Recover Is Detected: %10.4f", tor);
  }
  else if (p_state->posHasBias && (tor - p_state->posBiasDecTime) > 60)
  {
    p_state->posHasBias = FALSE;
    SYS_LOGGING(OBJ_PE, LOG_INFO, "Pos Bias Flag Is Reset: %10.4f", tor);
  }
}
/***********************************************************************
* ��������: gnss_Pe_State_Monitor
* ���������
* ���������
* ����ֵ��
* ���ߣ�jh
* ���ڣ�08/28/
***********************************************************************/
void gnss_Pe_State_Monitor(meas_blk_t* pMeas, Kf_t* p_KfBack, PeStateMonitor_t* p_state)
{
  PosBiasDet_KfRes* p_KfResDec = &(p_state->kfResPosBiasDect);
  PosBiasDet_PrDiff* p_PrDiffDec = &(p_state->prDiffPosBiasDect);
  PosBiasDet_KfState* p_KfStateDec = &(p_state->kfStatePosBiasDect);

  gnss_Pe_PosBias_Detect_KfRes(p_KfBack, p_KfResDec);

  gnss_Pe_PosBias_Detect_PrDiff(pMeas, p_PrDiffDec, p_KfBack);

  gnss_Pe_PosBias_Detect_KfState(p_KfBack, p_KfStateDec);

#if 0
  if (p_KfResDec->posIsBias)
  {
    p_state->posHasBias = p_KfResDec->posIsBias;
    p_state->posBiasDecTime = p_KfResDec->posResetTime;
    p_state->posRecoverCnt = 0;
  }
  else if (p_PrDiffDec->posIsBias)
  {
    p_state->posHasBias = p_PrDiffDec->posIsBias;
    p_state->posBiasDecTime = p_PrDiffDec->decEpoch;
    p_state->posRecoverCnt = 0;
  }
  else if (p_KfStateDec->posIsBias)
  {
    p_state->posHasBias = p_KfStateDec->posIsBias;
    p_state->posBiasDecTime = p_KfStateDec->decEpoch;
    p_state->posRecoverCnt = 0;
  }
  gnss_Pe_PosRecover_Check(pMeas->tor, p_state, p_PrDiffDec, p_KfResDec);

  if (p_state->posHasBias)
  {
    p_Kf->state = KF_STATE_POSBIAS;
  }

  SYS_LOGGING(OBJ_PE, LOG_INFO, "PE State Monitor: %8.3f,%d", pMeas->tor, p_state->posHasBias);
#endif
}


/***********************************************************************
* ��������: gnss_Pe_PosStatus_Check
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�11/02/
***********************************************************************/
uint8_t gnss_Pe_PosStatus_Check(meas_blk_t* pMeas)
{
  uint8_t               i, j, k = 0, n, sum;
  uint8_t               azStatus[4] = { 0 };
  float              az, azSave[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, delta;
  float* p, * p1;
  gnss_meas_t* pSvMeas;
  sat_data_t* sp;

  /* There is no valid PR DIFF */
  if (!pMeas->hasDiff)
  {
    return FALSE;
  }

  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &(pMeas->meas[i]);

    if (pSvMeas->prn == 0) continue;
    sp = gnss_sd_get_sv_data(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);
    if (sp == NULL) continue;
    az = (float)(RAD2DEG * sp->d_cos.az);

    if (fabs(pSvMeas->pr_diff) < 10.0 && fabs(pSvMeas->pr_diff) > 0.0)
    {
      if (az >= 0 && az < 90) azStatus[0] = TRUE;
      else if (az >= 90 && az < 180) azStatus[1] = TRUE;
      else if (az >= 180 && az < 270)azStatus[2] = TRUE;
      else azStatus[3] = TRUE;

      azSave[k] = az;
      k++;
    }
  }
  sum = azStatus[0] + azStatus[1] + azStatus[2] + azStatus[3];

  /* If more than 8 good measurements, just return TRUE */
  if (k >= 8)
  {
    return TRUE;
  }

  if ((sum == 4) && k >= 2)
  {
    n = k * (k - 1) / 2;
    p = (float*)Sys_Calloc(n, sizeof(float));
    if (p == NULL) return FALSE;
    p1 = p;
    for (i = 0; i < k - 1; i++)
    {
      for (j = i + 1; j < k; j++)
      {
        delta = azSave[i] - azSave[j];
        if (delta > 180) delta -= 360;
        else if (delta < -180) delta += 360;
        delta = (float)fabs(delta);
        (*p1++) = delta;
      }
    }
    p1 = p;
    gnss_Sort_WithIndx(p1, NULL, n);
    j = 0;
    for (i = 0; i < n; i++)
    {
      if (p[i] >= 120) j++;
    }
    if (j >= 2 && gpz_kf_pvt_data->kfHeadingVel >= 2.0)
    {
#if defined(PLAYBACK_MODE)
      SYS_LOGGING(OBJ_PE, LOG_INFO, "%s,%14.6f,%6.2f,%02d,%02d", __FUNCTION__, pMeas->tor, realPosErr, peMode.staticData.staticFlag,
        peMode.userSceData.isUnderEleRoad);
#endif
    }

    if (p) Sys_Free(p);
  }

  return FALSE;
}

void gnss_Pe_Init_Mcorr(meas_blk_t* pMeas, Ls_t* pLs)
{
  uint32_t            i;
  int32_t            indx;
  gnss_meas_t* pSvMeas;
  sat_data_t* sp;
  LS_PVT_INFO* pLsPvtInfo = NULL;
  USER_PVT       pvt;

  pLsPvtInfo = pLs->ls_Pvt_Info;

  if (pLsPvtInfo->valid && (!p_Sd->Mcorr_init))
  {
    for (i = 0; i < 3; i++)
    {
      pvt.ecef.pos[i] = gpz_ls_pvt_data->ecefPos[i];
    }
    gnssConvEcef2Lla(pvt.ecef.pos, pvt.lla.pos);

    if (gpz_ls_pvt_data->fix_dim == DIM_3D)
    {
      pvt.have_position = HAVE_POS_FIX3D;
    }
    else if (gpz_ls_pvt_data->fix_dim == DIM_2D)
    {
      pvt.have_position = HAVE_POS_FIX2D;
    }

    gnss_Sd_Main(p_Sd, (void*)&pvt);

    for (i = 0; i < pMeas->measCnt; i++)
    {
      pSvMeas = &(pMeas->meas[i]);
      if (pSvMeas->prn == 0 || (pSvMeas->prn >= 93 && pSvMeas->gnssMode == GLN_MODE)) continue;

      /*prn validty check*/
      indx = gnss_sv_Idx(pSvMeas->gnssMode, pSvMeas->prn);
      if (indx < 0)
      {
        continue;
      }

      // Add new satellite
      gnss_sd_add_sv(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);
      sp = gnss_sd_get_sv_data(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);

      if (sp == NULL)
      {
        continue;
      }

      /* Elevation angle mask */
      if (sp->d_cos.elev > 0.0 && (sp->d_cos.elev * RAD2DEG) < g_pe_cfg.ele_mask)
      {
        continue;
      }

      pSvMeas->pseudoRange -= ((double)sp->mcorr.iono_corr + sp->mcorr.trop_corr);
#ifdef USED_IN_MC262M
      pSvMeas->pseudoRange1 -= (sp->mcorr.iono_corr + sp->mcorr.trop_corr);
#endif
    }
  }
}

void gnss_Pe_Feedback_Ins_Meas(const meas_blk_t* pMeas, USER_PVT* pvt, gnss_FeedbackInsMeasBlock_t* pz_CurrentFeedbackInsMeas)
{
  uint8_t u_i = 0;
  gnss_meas_t* pSvMeas;
  GNSS_TIME* pTime;

  pTime = gnss_tm_get_time();

  if (NULL != pz_CurrentFeedbackInsMeas)
  {
    cmn_InitGnssMeasFeedbackInsBlock(pz_CurrentFeedbackInsMeas);
    pz_CurrentFeedbackInsMeas->u_FeedbackMeasType = GNSS_FEEDBACK_PR_DOPPLER;

    for (; u_i < pMeas->measCnt; u_i++)
    {
      pSvMeas = (gnss_meas_t*)&(pMeas->meas[u_i]);
      if ((pz_CurrentFeedbackInsMeas->u_MeasCount) < MAX_FEEDBACK_INS_MEAS_NUM && pSvMeas->freq_index == 0 && pSvMeas->pseudoRange > 0.0)
      {
        if (fabs(pSvMeas->sv_info.p[0]) > 1e-3)
        {
          pz_CurrentFeedbackInsMeas->z_GnssFeedbackInsMeasUnit[pz_CurrentFeedbackInsMeas->u_MeasCount].d_pseudorange = pSvMeas->pseudoRange - pTime->bias[pSvMeas->gnssMode];
        }
        pz_CurrentFeedbackInsMeas->z_GnssFeedbackInsMeasUnit[pz_CurrentFeedbackInsMeas->u_MeasCount].f_doppler = (float)pSvMeas->pseudoRangeRate - pTime->drift;

        double range = 0.0;
        if (fabs(pvt->ecef.pos[0] > 1e-3))
        {
          range = gnssClcSqrtAminusB_DBL(&pvt->ecef.pos[0], &pSvMeas->sv_info.p[0], 3);
        }
        else
        {
          range = pz_CurrentFeedbackInsMeas->z_GnssFeedbackInsMeasUnit[pz_CurrentFeedbackInsMeas->u_MeasCount].d_pseudorange;
        }

        double satPoscorr[3] = { 0.0 };
        double toa = range / LIGHT_SEC;
        gnssEarthRotateCorr(&pSvMeas->sv_info.p[0], satPoscorr, toa);

        uint8_t u_constellation = C_GNSS_NONE;
        switch (pSvMeas->gnssMode)
        {
        case GPS_MODE:
          if (pSvMeas->prn <= 32)
          {
            u_constellation = C_GNSS_GPS;
          }
          else
          {
            u_constellation = C_GNSS_QZS;
          }
          break;
        case GLN_MODE:
          u_constellation = C_GNSS_GLO;
          break;
        case BDS_MODE:
          u_constellation = C_GNSS_BDS3;
          break;
        case GAL_MODE:
          u_constellation = C_GNSS_GAL;
          break;
        default:break;
        }
        pz_CurrentFeedbackInsMeas->z_GnssFeedbackInsMeasUnit[pz_CurrentFeedbackInsMeas->u_MeasCount].u_constellation = u_constellation;
        pz_CurrentFeedbackInsMeas->z_GnssFeedbackInsMeasUnit[pz_CurrentFeedbackInsMeas->u_MeasCount].u_svid = pSvMeas->prn;
        pz_CurrentFeedbackInsMeas->z_GnssFeedbackInsMeasUnit[pz_CurrentFeedbackInsMeas->u_MeasCount].u_cn0 = pSvMeas->cno;
        pz_CurrentFeedbackInsMeas->z_GnssFeedbackInsMeasUnit[pz_CurrentFeedbackInsMeas->u_MeasCount].d_pr_var = pSvMeas->prNoise;
        pz_CurrentFeedbackInsMeas->z_GnssFeedbackInsMeasUnit[pz_CurrentFeedbackInsMeas->u_MeasCount].d_dr_var = pSvMeas->drNoise;
        pz_CurrentFeedbackInsMeas->z_GnssFeedbackInsMeasUnit[pz_CurrentFeedbackInsMeas->u_MeasCount].u_obs_valid = pSvMeas->status & 0x03;
        pz_CurrentFeedbackInsMeas->z_GnssFeedbackInsMeasUnit[pz_CurrentFeedbackInsMeas->u_MeasCount].f_elevation = (float)(pSvMeas->sv_info.fltElev * DEG2RAD);
        pz_CurrentFeedbackInsMeas->z_GnssFeedbackInsMeasUnit[pz_CurrentFeedbackInsMeas->u_MeasCount].f_azimuth = (float)(pSvMeas->sv_info.fltAz * DEG2RAD);
        for (int k = 0; k < 3; k++)
        {
          pz_CurrentFeedbackInsMeas->z_GnssFeedbackInsMeasUnit[pz_CurrentFeedbackInsMeas->u_MeasCount].f_unit_dir_vect[k] = (float)pSvMeas->sv_info.dcos[k];
          pz_CurrentFeedbackInsMeas->z_GnssFeedbackInsMeasUnit[pz_CurrentFeedbackInsMeas->u_MeasCount].d_sat_pos[k] = satPoscorr[k];
          pz_CurrentFeedbackInsMeas->z_GnssFeedbackInsMeasUnit[pz_CurrentFeedbackInsMeas->u_MeasCount].d_sat_vel[k] = pSvMeas->sv_info.v[k];
        }
        pz_CurrentFeedbackInsMeas->u_MeasCount++;
      }
    }

    SYS_LOGGING(OBJ_PE, LOG_INFO, "OBS NUM,%hhu", pz_CurrentFeedbackInsMeas->u_MeasCount);
    if (pvt->have_position > HAVE_POS_NONE)
    {
      pz_CurrentFeedbackInsMeas->d_CurLLA[0] = pvt->lla.pos[0] * RAD2DEG;
      pz_CurrentFeedbackInsMeas->d_CurLLA[1] = pvt->lla.pos[1] * RAD2DEG;
      pz_CurrentFeedbackInsMeas->d_CurLLA[2] = pvt->lla.pos[2];
    }

    pz_CurrentFeedbackInsMeas->f_QuasiGeoidHeight = (float)pvt->geoidal_sep;
  }
}

static void gnss_Pe_Set_Scene_Tag_Unc(const asg_obs_t* asg_obs)
{
  int i = 0, j = 0, cs_cnt = 0, meas_cnt = 0, sv_num = 0;
  meas_blk_t* pmeas = ag_pe_get_meas_unc();
  unc_scene_t* scene_tag = &g_unc_scene;
  USER_PVT* pvt = gpz_user_pvt;
  memset(scene_tag, 0, sizeof(unc_scene_t));
  if (!pmeas) return;
  if (!asg_obs) return;

  if (asg_obs->n > 0) {
    for (i = 0; i < asg_obs->n && i < MAXOBS; i++) {
      switch (asg_obs->data[i].sys)
      {
      case SYS_GPS:
      {
        scene_tag->gps_num++;
        if (asg_obs->data[i].SNR[0] > 0) {
          scene_tag->gps_L1_num++;
        }
        if (asg_obs->data[i].SNR[1] > 0) {
          scene_tag->gps_L5_num++;
        }
        break;
      }
      case SYS_QZS:
      {
        scene_tag->qzss_num++;
        if (asg_obs->data[i].SNR[0] > 0) {
          scene_tag->qzs_L1_num++;
        }
        if (asg_obs->data[i].SNR[1] > 0) {
          scene_tag->qzs_L5_num++;
        }
        break;
      }
      case SYS_GLO: scene_tag->gln_num++; break;
      case SYS_GAL: scene_tag->gal_num++; break;
      case SYS_CMP: scene_tag->bds_num++; break;
      default: break;
      }

      for (j = 0; j < AGGNSS_MAX_FREQ_NUM; j++) {
        if (asg_obs->data[i].SNR[j] > 0) {
          sv_num++;
          break;
        }
      }

      for (j = 0; j < AGGNSS_MAX_FREQ_NUM; j++) {
        if ((g_pe_cfg.sub_chipType != SUB_CHIPTYPE_QCOM_DF) && asg_obs->data[i].SNR[j] > 0) {
          meas_cnt++;
          if ((asg_obs->data[i].LLI[j] & 0xFF) != 0) {
            cs_cnt++;
          }
        }
      }
    }

    scene_tag->cycle_clip_ratio = (float)(100.0 * cs_cnt / meas_cnt);
  }

  scene_tag->utc_timestamp_ms = pvt->timeStampOfUtc;
  scene_tag->avg_cn0 = pmeas->avgCno;
  scene_tag->used_sv_num = sv_num;
  scene_tag->hdop = pvt->DOP.hDOP;
  scene_tag->pdop = pvt->DOP.pDOP;
}

static void gnss_Pe_CleanVdrFeedback(VDR_Gnss_Feedback_t* pvdrFeedbackInfo)
{
  memset(pvdrFeedbackInfo, 0, sizeof(VDR_Gnss_Feedback_t));
}

/***********************************************************************
* ��������: gnss_Pe_Exec
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/02/
***********************************************************************/
void gnss_Pe_Exec(meas_blk_t* pMeas, asg_obs_t* asg_obs, gnss_FeedbackInsMeasBlock_t* pz_CurrentFeedbackInsMeas)
{
  if (!g_pe_cfg.enable_pe)
  {
    GLOGE("GNSS PVT ver1.0 Tor %.6f, enable pe failed\n", pMeas->tor);
    return;
  }
  GLOGW("PVT tor %.6f, MeasCnt %2d", pMeas->tor, pMeas->measCnt);

  /* 5Hz Meas, 1Hz PVT */
  gnss_Pe_Proc_Ctrl(pMeas);

  /* Pre-process of measurements */
  gnss_Pe_Pre_Proc(pMeas, &peMode);

  /* This function is used for leap second adjust */
  gnss_tm_leap_secs_adjust(p_gnssTime, pMeas);

  /* position propagation */
  gnss_Pe_Pos_Extrapolate(gpz_user_pvt);

  /* calculation DIff */
  gnss_Pe_Diff(pMeas, p_Kf, &vdrFeedbackInfo);

  /* calculate valid PR number */
  gnss_Pe_PRDR_Num(pMeas);

  /* QoS of Measurements */
  gnss_Qos_Main(pMeas, p_Kf, &peState);

  if ((g_proc.flag & 0xFC01) > 0)
  {
    /* Mode detection */
    gnss_Mode_Detection(pMeas, &peMode, gpz_kf_pvt_data);

    /* Init LS bias DIff */
    gnss_Pe_Set_LsBiasDiff(p_Kf, gpz_Ls);

    /* LS Process */
    gnss_Ls_Main(pMeas, gpz_Ls);

    /*Initial measurement corrections*/
    gnss_Pe_Init_Mcorr(pMeas, gpz_Ls);

    /*static mode detection */
    gnss_Static_Mode_Detection(p_Kf, &peMode);

    /* BIAS Monitor */
    gnss_Pe_Bias_Monitor(p_Kf);

    /* KF process */
    gnss_Kf_Main(pMeas, p_Kf);

    /* position filter state monitor */
    gnss_Pe_State_Monitor(pMeas, p_Kf, &peState);

    /* Bias and Drift update */
    gnss_Pe_Set_Bias(gpz_Ls, p_Kf);
  }

  /* Position fix log */
  gnss_Pe_Fill_PosFix(gpz_user_pvt, pMeas);

  /* Record scenario tag information. */
  gnss_Pe_Set_Scene_Tag_Unc(asg_obs);

  if ((g_proc.flag & 0xFC01) > 0)
  {
    /* SD process */
    gnss_Sd_Main(p_Sd, NULL);
  }

  /* PE data log */
  gnss_Pe_Report(pMeas, gpz_Ls, p_Kf);

  /** feeedback INS meas */
  gnss_Pe_Feedback_Ins_Meas(pMeas, gpz_user_pvt, pz_CurrentFeedbackInsMeas);

  /* rtk process */
  //gnss_rtk_process(pMeas, asg_obs);

  g_proc.last_flag = g_proc.flag;

  gnss_Pe_CleanVdrFeedback(&vdrFeedbackInfo);
}

/***********************************************************************
* ��������: gnss_Pe_Meas_Proc
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�10/08/
***********************************************************************/
void gnss_Pe_Meas_Proc(const void* msg, void* user, uint32_t* size)
{
#ifdef USED_IN_MC262M
  // measurement data convert
  gnss_Pe_Meas_Convert(pRawData, &rawMeas);
#endif

  // local measurement process
  gnss_Pe_Exec(gpz_Meas, NULL, NULL);

  if (user && size)
  {
    memcpy(user, (void*)gpz_user_pvt, sizeof(USER_PVT));
    *size = sizeof(USER_PVT);
  }
}
/***********************************************************************
* ��������: gnss_Pe_Iono_Proc
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�10/14/
***********************************************************************/
void gnss_Pe_Iono_Proc(const void* msg)
{
  uint8_t                            gnssMode;
  ION_INFO                      ionoLocal;
  const GnssIonosphericModelMessage_t* ionoRawData;

  memset(&ionoLocal, 0, sizeof(ionoLocal));
  ionoRawData = (const GnssIonosphericModelMessage_t*)msg;

  ionoLocal.alpha_0 = ionoRawData->alfa0;
  ionoLocal.alpha_1 = ionoRawData->alfa1;
  ionoLocal.alpha_2 = ionoRawData->alfa2;
  ionoLocal.alpha_3 = ionoRawData->alfa3;

  ionoLocal.beta_0 = ionoRawData->beta0;
  ionoLocal.beta_1 = ionoRawData->beta1;
  ionoLocal.beta_2 = ionoRawData->beta2;
  ionoLocal.beta_3 = ionoRawData->beta3;

  if (ionoRawData->constellation == AGGNSS_CONSTELLATION_GPS ||
    ionoRawData->constellation == AGGNSS_CONSTELLATION_QZSS)
  {
    gnssMode = GPS_MODE;
  }
  else if (ionoRawData->constellation == AGGNSS_CONSTELLATION_BEIDOU)
  {
    gnssMode = BDS_MODE;
  }
  else
  {
    gnssMode = GPS_MODE;
  }

  /* When there is IONO data injection, just save them */
  gnss_Sd_Nm_AddIono(gnssMode, &ionoLocal);
}

/***********************************************************************
* ��������: gnss_Pe_NavBit_Proc
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�10/14/
***********************************************************************/
void gnss_Pe_NavBit_Proc(const void* msg)
{
#if defined(_WIN32) || defined(GNUC_MACRO)

  const AGGnss_NavigationMessage_t* rawBits;

  rawBits = (const AGGnss_NavigationMessage_t*)msg;

  //save offline log
#ifdef OFFLINE_INFO_SAVE
  gnss_sd_info_save(SAVE_INFO_RAWBITS, msg);
#endif
  if (g_pe_cfg.chipType == QCOM)
  {
    //decode the raw message
    if (gnss_sd_msgDec_N(rawBits) == FALSE) {
      SYS_LOGGING(OBJ_SD, LOG_WARNING, "data bits decoded failed!, %s %d", __FUNCTION__, __LINE__);
    }
  }
  else if (g_pe_cfg.chipType == SPRD)
  {
    //decode the raw message
    if (gnss_sd_msgDec(rawBits) == FALSE) {
      SYS_LOGGING(OBJ_SD, LOG_WARNING, "data bits decoded failed!, %s %d", __FUNCTION__, __LINE__);
    }
  }
#endif
}

/***********************************************************************
* ��������: gnss_Pe_Week_Check
* ���������
* ���������
* ����ֵ��
* ���ߣ�none
* ���ڣ�07/31/
***********************************************************************/
void gnss_Pe_Week_Check(uint16_t week, double tow, GNSS_TIME* gnssTime)
{
  uint16_t    week_tmp;
  double    dt = 0.0;

  if (gnssTime == NULL) return;

  if (gnssTime->init == FALSE) return;

  dt = tow - gnssTime->rcvr_time[GPS_MODE];
  week_tmp = week;

  if (dt > SECS_IN_WEEK / 2.0)
  {
    week_tmp += 1;
  }
  else if (dt < -SECS_IN_WEEK / 2.0)
  {
    week_tmp -= 1;
  }

  if (week_tmp != gnssTime->week[GPS_MODE])
  {
    gnssTime->init = FALSE;
    GLOGI("GNSS week check fail");
  }
}

/***********************************************************************
* ��������: gnss_Pe_Rtcm_Proc
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�10/31/
***********************************************************************/
void gnss_Pe_Rtcm_Proc(void* msg)
{
  rtcm_data_pack_t* rawData = (rtcm_data_pack_t*)msg;

#ifdef OFFLINE_INFO_SAVE
  gnss_sd_info_save(SAVE_INFO_RTCM, msg);
#endif

#ifdef AG_GNSS_RTD_FUNCTION_IMPL
  gnss_sd_rtcm_rtd_proc(rawData);
#else //AG_GNSS_RTK_FUNCTION_IMPL
  gnss_sd_rtcm_rtk_proc(rawData);
#endif
}


/**
* @author: liyan.wang
* @version:
* @param:
* @deprecated:
* @return:
* @see:
* @todo:
* @bug:
*/


void gnss_Pe_Get_Kf_Data(Kf_t* pkfData) {
  memcpy(pkfData, p_Kf, sizeof(Kf_t));
}

void gnss_Pe_Get_Ls_Data(Ls_t* plsData) {
  memcpy(plsData, gpz_Ls, sizeof(Ls_t));
}

uint16_t gnss_calc_meas_avg_cn0()
{
  uint32_t i;
  uint32_t acc_cn0 = 0;

  if (gpz_Meas->measCnt > MAX_MEAS_NUM || gpz_Meas->measCnt == 0) {
    return 0;
  }

  for (i = 0; i < gpz_Meas->measCnt; i++) {
    acc_cn0 += gpz_Meas->meas[i].cno;
  }

  return (uint16_t)(acc_cn0 / gpz_Meas->measCnt);
}

double gnss_calc_algo_epoch_dt(double tor, double last_tor)
{
  double dt = 0.0;

  dt = tor - last_tor;
  if ((dt < 0.0) && (dt > -SECS_IN_WEEK / 2.0))
  {
    dt = 1.0;
  }
  else if (dt < -SECS_IN_WEEK / 2.0)
  {
    dt = dt + (double)SECS_IN_WEEK;
  }

  if ((last_tor < 1e-3) && (dt > 10.0))
  {
    dt = 0.0;
  }

  return dt;

}

USER_PVT* gnss_get_last_pvt()
{
  return gpz_user_pvt;
}

meas_blk_t* ag_pe_get_meas_unc()
{
  return gpz_Meas;
}

unc_scene_t* ag_pe_get_scene_tag_unc()
{
  return &g_unc_scene;
}

void gnss_pe_cal_prdiff_std(meas_blk_t* pMeas)
{
  gnss_meas_t* pSvMeas;
  float            diff[MAX_MEAS_NUM] = { 0.0 }, prDiffAvg;
  uint32_t            i, satMode;
  uint32_t            totalCnt = 0;

  for (satMode = 0; satMode < GNSS_MAX_MODE; satMode++)
  {
    totalCnt = 0;
    for (i = 0; i < pMeas->measCnt; i++)
    {
      pSvMeas = &pMeas->meas[i];
      if (pSvMeas->gnssMode != satMode) continue;
      if (pSvMeas->prn == 0 || pSvMeas->prDiffValid == 0) continue;
      if ((pSvMeas->status & 0x1) == 0) continue;
      if (DBL_IS_EQUAL(pSvMeas->pr_diff, 0.0)) continue;

      diff[totalCnt] = (float)pSvMeas->pr_diff;
      totalCnt++;
    }
    gnss_math_fstd(diff, totalCnt, &prDiffAvg, &(pMeas->prDiffStd_3[satMode]));
    for (i = 0; i < totalCnt; i++)
    {
      diff[i] = 0.0;
    }
  }
}

