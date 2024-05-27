/**@file        loc_core_api.c
 * @brief       Location engine core api source file, it's unavaliable for user
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/04/25  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "cmn_utils.h"
#include "gnss_common.h"

#include "mw_ipctask.h"
#include "mw_alloc.h"
#include "sm_api.h"
#include "hpp_api.h"
#include "dcp_api.h"
#include "ppp_api.h"
#include "sd_api.h"
#include "vdr_api.h"
#include "ort_api.h"
#include "ekf_api.h"
#include "mw_log.h"
#include "mw_alloc.h"

#include "loc_core_api.h"
#include "loc_core_cfg.h"
#include "loc_core_report.h"
#include "rtcm_dec.h"

loc_core_api_ctrl_t gz_loc_core_api_ctrl;

/**
 * @brief     Location Engine API for Register VLT callback functions
 * @param[in] cb - the callback function package
 * @return    None
 */
int32_t loc_api_Register_Callback(loc_api_callback_t* cb)
{
  if (NULL == cb)
  {
    return FALSE;
  }

  loc_report_SetCallback(cb);

  return TRUE;
}

/**
 * @brief     Location Engine API for Register Memory pool
 * @param[in] pool_mem   - Memory pools pointer array
 * @param[in] pool_size  - Memory pools size array
 * @param[in] pool_count - Memory pools count
 * @return    None
 */
int32_t loc_api_Register_MemoryPool(loc_api_MemoryRegister_t* pz_MemoryRegister)
{
  BOOL ret = mw_os_alloc_init(pz_MemoryRegister);

  return ret;
}

/**
 * @brief Location Engine API for Register configurations
 * @return      None
 */
int32_t loc_api_Register_Config(loc_api_config_para_group_t* pz_api_config_para_grp)
{
  loc_cfg_initialize();
  loc_cfg_SetConfigParamGroup(pz_api_config_para_grp);

  return TRUE;
}

/**
 * @brief Location Engine API for start
 * @return      None
 */
int32_t loc_api_Initialize()
{
  loc_ConfigParamGroup_t* pz_loc_ConfigParamGroup = loc_cfg_getConfigParamGroup();
  if (NULL == pz_loc_ConfigParamGroup)
  {
    return FALSE;
  }

  gz_loc_core_api_ctrl.u_bInit = TRUE;

  uint8_t u_TaskStatus = TASK_STATUS_DISABLE;

  if (pz_loc_ConfigParamGroup->ipc_cfg.b_multiTask)
  {
    u_TaskStatus = TASK_STATUS_ENABLE;
  }
  else if (pz_loc_ConfigParamGroup->ipc_cfg.b_singleTask)
  {
    u_TaskStatus = TASK_STATUS_SINGLE;
  }
  else
  {
    u_TaskStatus = TASK_STATUS_DISABLE;
  }

  ipctask_Init();

  ipctask_SetThreadId2TaskId(get_thread_id(), TASK_INDEX_USER);

  if (pz_loc_ConfigParamGroup->log_cfg.b_enable)
  {
    ipctask_Start(TASK_INDEX_LOG, log_task, u_TaskStatus);
    log_api_Init(&pz_loc_ConfigParamGroup->log_cfg);
  }

  if (pz_loc_ConfigParamGroup->sm_cfg.b_enable)
  {
    ipctask_Start(TASK_INDEX_SM, sm_task, u_TaskStatus);
    sm_api_Init();
    sm_api_Start();
  }

  if (pz_loc_ConfigParamGroup->hpp_cfg.b_enable)
  {
    ipctask_Start(TASK_INDEX_HPP, hpp_task, u_TaskStatus);
    hpp_api_Init();
    hpp_api_Start();
  }

  if (pz_loc_ConfigParamGroup->dcp_cfg.b_enable)
  {
    ipctask_Start(TASK_INDEX_DCP, dcp_task, u_TaskStatus);
    dcp_api_Init();
    dcp_api_Start();
  }

  if (pz_loc_ConfigParamGroup->ppp_cfg.b_enable)
  {
    ipctask_Start(TASK_INDEX_PPP, ppp_task, u_TaskStatus);
    ppp_api_Init();
    ppp_api_Start();
  }

  if (pz_loc_ConfigParamGroup->sd_cfg.b_enable)
  {
    ipctask_Start(TASK_INDEX_SD, sd_task, u_TaskStatus);
    sd_api_Init();
    sd_api_Start();
  }

  if (pz_loc_ConfigParamGroup->vdr_cfg.b_enable)
  {
    ipctask_Start(TASK_INDEX_VDR, vdr_task, u_TaskStatus);
    vdr_api_Init();
    vdr_api_Start();
    ipctask_Start(TASK_INDEX_EKF, ekf_task, u_TaskStatus);
    ekf_api_Init();
    ekf_api_Start();
  }

  if (pz_loc_ConfigParamGroup->ort_cfg.b_enable)
  {
    ipctask_Start(TASK_INDEX_ORT, ort_task, u_TaskStatus);
    ort_api_Init();
    ort_api_Start();
  }

  return TRUE;
}

/**
 * @brief Location Engine API for release
 * @return      None
 */
int32_t loc_api_Release()
{
  gz_loc_core_api_ctrl.u_bInit = FALSE;

  sm_api_Release();
  hpp_api_Release();
  dcp_api_Release();
  ppp_api_Release();
  sd_api_Release();
  vdr_api_Release();
  log_api_Release();

  loc_report_ReleaseMonitor();

  return TRUE;
}

/**
 * @brief     Inject GNSS Receiver measurement by RTCM format
 * @param[in] data - pointer to date buffer
 * @param[in] len - buffer length
 * @return    None
 */
void loc_api_InjectRcvMeasRTCM(uint8_t* data, uint32_t len)
{
  if (FALSE == gz_loc_core_api_ctrl.u_bInit)
  {
    LOGE(TAG_SM, "Loc Engine isn't init");
    return;
  }

  /* Pack a IPC by injected data */
  ipc_t ipc = { 0 };
  ipctask_PackDataToIpc(&ipc, TASK_INDEX_SM, C_M_SM_RCV_OBS_RTCM, data, len);
  log_ipc(&ipc);
  sm_RcvMeasRTCM_Put(&ipc);

  return;
}

/**
 * @brief     Inject GNSS Reference station observed Correction measurement
              using RTCM format
 * @param[in] data - pointer to date buffer
 * @param[in] len - buffer length
 * @return    None
 * @note      The correction measurement is from NRTK service
 */
void loc_api_InjectRefCorrRTCM(uint8_t* data, uint32_t len)
{
  if (FALSE == gz_loc_core_api_ctrl.u_bInit)
  {
    return;
  }

  /* Pack a IPC by injected data */
  ipc_t ipc = {0};
  ipctask_PackDataToIpc(&ipc, TASK_INDEX_SM, C_M_SM_REF_CORR_RTCM, data, len);
  log_ipc(&ipc);
  sm_RefCorrRTCM_Put(&ipc);
  
  return;
}

/**
 * @brief     Inject GNSS Receiver measurement using measurement block
 * @param[in] p_data - pointer to GNSS Receiver measurement
 * @param[in] q_length - length of GNSS Receiver measurement
 * @return    None
 */
void loc_api_InjectRcvMeasBlock(uint8_t* p_data, uint32_t q_length)
{
  if (FALSE == gz_loc_core_api_ctrl.u_bInit)
  {
    LOGE(TAG_SM, "Loc Engine isn't init");
    return;
  }

  if ((sizeof(GnssMeasBlock_t) != q_length) ||
    (NULL == p_data))
  {
    return;
  }

  GnssMeasBlock_t* pz_GnssMeasBlock = (GnssMeasBlock_t*)p_data;
  sm_api_RcvMeasBlk_Put(pz_GnssMeasBlock);

  return;
}
static void gnssApiToGPSEph(const loc_api_GpsEphemeris_t* api_eph, GpsEphemeris_t* eph)
{
  eph->svid = api_eph->svid;
  eph->week = api_eph->week;
  eph->accuracy = api_eph->accuracy;
  eph->health = api_eph->health;
  eph->iodc = api_eph->iodc;
  eph->code = api_eph->code;
  eph->fit = api_eph->fit;
  eph->iode = api_eph->iode;

  eph->OmegaDot = api_eph->OmegaDot;

  eph->i0 = api_eph->i0;
  eph->idot = api_eph->idot;
  eph->DeltaN = api_eph->DeltaN;
  eph->crc = api_eph->crc;
  eph->crs = api_eph->crs;
  eph->cuc = api_eph->cuc;
  eph->cus = api_eph->cus;
  eph->cic = api_eph->cic;
  eph->cis = api_eph->cis;
  eph->af0 = api_eph->af0;
  eph->af1 = api_eph->af1;
  eph->af2 = api_eph->af2;
}
static void gnssApiToBDSEph(const loc_api_BdsEphemeris_t* api_eph, BdsEphemeris_t* eph)
{
  eph->svid = api_eph->svid;
  eph->week = api_eph->week;
  eph->accuracy = api_eph->accuracy;
  eph->health = api_eph->health;
  eph->iodc = api_eph->iodc;
  eph->code = api_eph->code;
  eph->fit = api_eph->fit;
  eph->iode = api_eph->iode;

  eph->tgdB1I = api_eph->tgdB1I;
  eph->tgdB2I = api_eph->tgdB2I;
  eph->toc = api_eph->toc;
  eph->toe = api_eph->toe;
  eph->sqrt_A = api_eph->sqrt_A;
  eph->e = api_eph->e;
  eph->M0 = api_eph->M0;
  eph->Omega0 = api_eph->Omega0;
  eph->Omega = api_eph->Omega;
  eph->OmegaDot = api_eph->OmegaDot;

  eph->i0 = api_eph->i0;
  eph->idot = api_eph->idot;
  eph->DeltaN = api_eph->DeltaN;
  eph->crc = api_eph->crc;
  eph->crs = api_eph->crs;
  eph->cuc = api_eph->cuc;
  eph->cus = api_eph->cus;
  eph->cic = api_eph->cic;
  eph->cis = api_eph->cis;
  eph->af0 = api_eph->af0;
  eph->af1 = api_eph->af1;
  eph->af2 = api_eph->af2;
}
static void gnssApiToGALEph(const loc_api_GalEphemeris_t* api_eph, GalEphemeris_t* eph)
{
  eph->svid = api_eph->svid;
  eph->week = api_eph->week;
  eph->accuracy = api_eph->accuracy;
  eph->E1health = api_eph->E1health;
  eph->E1DVS = api_eph->E1DVS;
  eph->E5bhealth = api_eph->E5bhealth;
  eph->E5bDVS = api_eph->E5bDVS;
  eph->iodc = api_eph->iodc;
  eph->code = api_eph->code;
  eph->fit = api_eph->fit;
  eph->iode = api_eph->iode;

  eph->tgdE1E5a = api_eph->tgdE1E5a;
  eph->tgdE1E5b = api_eph->tgdE1E5b;
  eph->toc = api_eph->toc;
  eph->toe = api_eph->toe;
  eph->sqrt_A = api_eph->sqrt_A;
  eph->e = api_eph->e;
  eph->M0 = api_eph->M0;
  eph->Omega0 = api_eph->Omega0;
  eph->Omega = api_eph->Omega;
  eph->OmegaDot = api_eph->OmegaDot;

  eph->i0 = api_eph->i0;
  eph->idot = api_eph->idot;
  eph->DeltaN = api_eph->DeltaN;
  eph->crc = api_eph->crc;
  eph->crs = api_eph->crs;
  eph->cuc = api_eph->cuc;
  eph->cus = api_eph->cus;
  eph->cic = api_eph->cic;
  eph->cis = api_eph->cis;
  eph->af0 = api_eph->af0;
  eph->af1 = api_eph->af1;
  eph->af2 = api_eph->af2;
}
/**
 * @brief     Inject GNSS API Ephemeris data
 * @param[in] p_data - pointer to GNSS Ephemeris data
 * @param[in] q_length - length of GNSS Ephemeris data
 * @return    None
 */
void loc_api_InjectApiEphemeris(uint8_t* p_data, uint32_t q_length)
{
  if (FALSE == gz_loc_core_api_ctrl.u_bInit)
  {
    LOGE(TAG_SM, "Loc Engine isn't init");
    return;
  }

  if ((sizeof(loc_api_GnssEphemeris_t) != q_length) ||
     (NULL == p_data))
  {
    return;
  }

  loc_api_GnssEphemeris_t* loc_Ephemeris = (loc_api_GnssEphemeris_t*)p_data;

  gnss_Ephemeris_t z_Ephemeris;
  BOOL q_isInjectEph = TRUE;
  z_Ephemeris.u_version = loc_Ephemeris->u_version;
  
  tm_cvt_SetGpst(&z_Ephemeris.z_toeOfGpst, loc_Ephemeris->w_weekOfToe, loc_Ephemeris->q_towMsecOfToe * TIME_MSEC_INV);
  tm_cvt_SetGpst(&z_Ephemeris.z_tocOfGpst, loc_Ephemeris->w_weekOfToc, loc_Ephemeris->q_towMsecOfToc * TIME_MSEC_INV);

  switch (loc_Ephemeris->u_constellation)
  {
  case LOC_API_GNSS_GPS:
    gnssApiToGPSEph(&loc_Ephemeris->eph.z_gpsEph, &z_Ephemeris.eph.z_gpsEph);
    z_Ephemeris.u_constellation = C_GNSS_GPS;
    break;
  case LOC_API_GNSS_GLO:
    //memcpy(&z_api_GnssEph.eph.z_gloEph, &pz_GnssEph->eph.z_gloEph, sizeof(GloEphemeris_t)); // TBD
    z_Ephemeris.u_constellation = C_GNSS_GLO;
    q_isInjectEph = FALSE;
    break;
  case LOC_API_GNSS_BDS:
    gnssApiToBDSEph(&loc_Ephemeris->eph.z_bdsEph, &z_Ephemeris.eph.z_bdsEph);
    z_Ephemeris.u_constellation = C_GNSS_BDS3;
    if (loc_Ephemeris->eph.z_bdsEph.svid < 19) z_Ephemeris.u_constellation = C_GNSS_BDS2;
    break;
  case LOC_API_GNSS_GAL:
    gnssApiToGALEph(&loc_Ephemeris->eph.z_galEph, &z_Ephemeris.eph.z_galEph);
    z_Ephemeris.u_constellation = C_GNSS_GAL;
    break;
  case LOC_API_GNSS_QZS:
    gnssApiToGPSEph(&loc_Ephemeris->eph.z_qzsEph, &z_Ephemeris.eph.z_qzsEph);
    z_Ephemeris.u_constellation = C_GNSS_QZS;
    break;
  default:
    q_isInjectEph = FALSE;
    break;
  }
  
  if (TRUE == q_isInjectEph)
  {
    hpp_api_Ephemeris_Put(&z_Ephemeris);
    sd_api_Ephemeris_Put(&z_Ephemeris);
  }

  return;
}

/**
 * @brief Inject GNSS Ephemeris data
 * @param[in] p_data - pointer to GNSS Ephemeris data
 * @param[in] q_length - length of GNSS Ephemeris data
 * @return None
 */
void loc_api_InjectEphemeris(uint8_t* p_data, uint32_t q_length)
{
  if (FALSE == gz_loc_core_api_ctrl.u_bInit)
  {
    LOGE(TAG_SM, "Loc Engine isn't init");
    return;
  }

  if ((sizeof(gnss_Ephemeris_t) != q_length) ||
      (NULL == p_data))
  {
    return;
  }

  gnss_Ephemeris_t* pz_Ephemeris = (gnss_Ephemeris_t*)p_data;
  hpp_api_Ephemeris_Put(pz_Ephemeris);
  sd_api_Ephemeris_Put(pz_Ephemeris);

  return;
}

/**
 * @brief     Inject GNSS Reference station observed Correction measurement
              using measurement block
 * @param[in] p_data - pointer to GNSS Reference station observe
 * @param[in] q_length - length of GNSS Reference station observe
 * @return    None
 * @note      The correction measurement is from NRTK service as measurement block
 */
void loc_api_InjectRefCorrMeasBlk(uint8_t* p_data, uint32_t q_length)
{
  if (FALSE == gz_loc_core_api_ctrl.u_bInit)
  {
    LOGE(TAG_SM, "Loc Engine isn't init");
    return;
  }

  if ((sizeof(GnssCorrBlock_t) != q_length) ||
    (NULL == p_data))
  {
    return;
  }

  GnssCorrBlock_t* pz_GnssCorrBlock = (GnssCorrBlock_t*)p_data;
  hpp_api_RefCorrBlk_Put(pz_GnssCorrBlock);
  return;
}

/**
 * @brief     Inject Orient Receiver measurement using measurement block
 * @param[in] p_data - pointer to GNSS Receiver measurement
 * @param[in] q_length - length of GNSS Receiver measurement
 * @return    None
 */
void loc_api_InjectOrtMeasBlock(uint8_t* p_data, uint32_t q_length)
{
  if (FALSE == gz_loc_core_api_ctrl.u_bInit)
  {
    LOGE(TAG_SM, "Loc Engine isn't init");
    return;
  }

  if ((sizeof(GnssMeasBlock_t) != q_length) ||
    (NULL == p_data))
  {
    return;
  }

  GnssMeasBlock_t* pz_GnssMeasBlock = (GnssMeasBlock_t*)p_data;
  sm_api_OrtMeasBlk_Put(pz_GnssMeasBlock);

  return;
}

/**
 * @brief     Inject Orient Receiver measurement by RTCM format
 * @param[in] data - pointer to Orient Receiver measurement
 * @param[in] len - length of Orient Receiver measurement
 * @return    None
 */
void loc_api_InjectOrtMeasRTCM(uint8_t* data, uint32_t len)
{
  if (FALSE == gz_loc_core_api_ctrl.u_bInit)
  {
    LOGE(TAG_SM, "Loc Engine isn't init");
    return;
  }

  /* Pack a IPC by injected data */
  ipc_t ipc = { 0 };
  ipctask_PackDataToIpc(&ipc, TASK_INDEX_SM, C_M_SM_TWIN_RCV_OBS_RTCM, data, len);
  log_ipc(&ipc);
  sm_RcvTwinMeasRTCM_Put(&ipc);

  return;
}

/**
 * @brief     Inject IMU data for INS module
 * @param[in] p_data - pointer to IMU data
 * @param[in] q_length - length of IMU data
 * @return    None
 */
void loc_api_InjectImuDataToIns(uint8_t* p_data, uint32_t q_length)
{
  if (FALSE == gz_loc_core_api_ctrl.u_bInit)
  {
    LOGE(TAG_SM, "Loc Engine isn't init");
    return;
  }

  if ((sizeof(loc_api_imu_data_t) != q_length) ||
    (NULL == p_data))
  {
    return;
  }

  loc_api_imu_data_t* pz_loc_ImuData = (loc_api_imu_data_t*)p_data;
  ins_ImuData_t z_ImuData = { 0 };

  z_ImuData.q_tow_msec = pz_loc_ImuData->q_tow_msec;
  for (uint8_t i = 0; i < 3; i++)
  {
    z_ImuData.f_gyro[i] = pz_loc_ImuData->f_gyro[i];
    z_ImuData.f_accl[i] = pz_loc_ImuData->f_accl[i];
  }
  z_ImuData.f_temperature = pz_loc_ImuData->f_temperature;

  vdr_api_ImuData_Put(&z_ImuData);

  return;
}

void loc_api_InjectWheelDataToIns(uint8_t* p_data, uint32_t q_length)
{
  /*if (FALSE == gz_loc_core_api_ctrl.u_bInit)
  {
    LOGE(TAG_SM, "Loc Engine isn't init");
    return;
  }*/
  ins_WheelData_t z_WheelData;
  if ((sizeof(loc_api_wheel_t) != q_length) ||
    (NULL == p_data))
  {
	LOGE(TAG_SM, "loc_api_InjectWheelDataToIns fail\n");
    return;
  }
  loc_api_wheel_t* pz_loc_WheelData = (loc_api_wheel_t*)p_data;
  LOGI(TAG_SM, "loc_api_InjectWheelDataToIns timestamp %lld fl %d fr %d rl %d rr %d angle %f gear %d\n",pz_loc_WheelData->t_timestamp,pz_loc_WheelData->q_fl_whpulse,pz_loc_WheelData->q_fr_whpulse, \
    pz_loc_WheelData->q_rl_whpulse,pz_loc_WheelData->q_rr_whpulse,pz_loc_WheelData->f_angle_front,pz_loc_WheelData->e_gear);
    z_WheelData.u_version = 0;
	z_WheelData.t_timestamp = pz_loc_WheelData->t_timestamp;
	z_WheelData.q_fl_whpulse = pz_loc_WheelData->q_fl_whpulse;
	z_WheelData.q_fr_whpulse = pz_loc_WheelData->q_fr_whpulse;
	z_WheelData.q_rl_whpulse = pz_loc_WheelData->q_rl_whpulse;
	z_WheelData.q_rr_whpulse = pz_loc_WheelData->q_rr_whpulse;
	z_WheelData.f_angle_front = pz_loc_WheelData->f_angle_front;
	z_WheelData.f_angle_rear = pz_loc_WheelData->f_angle_rear;
	z_WheelData.f_odometer = pz_loc_WheelData->f_odometer;
	z_WheelData.e_gear = pz_loc_WheelData->e_gear;
	z_WheelData.u_selfck = pz_loc_WheelData->u_selfck;

	vdr_api_WheelData_Put(&z_WheelData);

}
/**
 * @brief  Inject GNSS fix information for INS module
 * @param[in] p_data - pointer to GNSS fix information
 * @param[in] q_length - length of GNSS fix information
 * @return
 */
void loc_api_InjectGnssFixToIns(uint8_t* p_data, uint32_t q_length)
{
  if (FALSE == gz_loc_core_api_ctrl.u_bInit)
  {
    LOGE(TAG_SM, "Loc Engine isn't init");
    return;
  }

  if ((sizeof(GnssFixType) != q_length) ||
    (NULL == p_data))
  {
    return;
  }
  
  GnssFixType* pz_GnssFix = (GnssFixType*)p_data;
  ins_GnssFix_t z_GnssFix = { 0 };

  z_GnssFix.t_timeStampOfUtc = (uint64_t)pz_GnssFix->ItowPos;
  z_GnssFix.d_lat = pz_GnssFix->Lat_deg;
  z_GnssFix.d_lon = pz_GnssFix->Lon_deg;
  z_GnssFix.f_alt = pz_GnssFix->Alt_m;
  z_GnssFix.f_latstd = pz_GnssFix->LatStd;
  z_GnssFix.f_lonstd = pz_GnssFix->LonStd;
  z_GnssFix.f_altstd = pz_GnssFix->AltStd;
  z_GnssFix.f_vn = pz_GnssFix->VelN_mps;
  z_GnssFix.f_ve = pz_GnssFix->VelE_mps;
  z_GnssFix.f_vd = pz_GnssFix->VelD_mps;
  z_GnssFix.f_vnstd = pz_GnssFix->VelNstd;
  z_GnssFix.f_vestd = pz_GnssFix->VelEstd;
  z_GnssFix.f_vdstd = pz_GnssFix->VelDstd;
  z_GnssFix.f_heading = (float)(atan2(z_GnssFix.f_ve, z_GnssFix.f_vn) * RAD2DEG);
  z_GnssFix.f_heading = z_GnssFix.f_heading < 0 ? 360 + z_GnssFix.f_heading : z_GnssFix.f_heading;
  z_GnssFix.f_speed = sqrtf(z_GnssFix.f_vn * z_GnssFix.f_vn + z_GnssFix.f_ve * z_GnssFix.f_ve);
  z_GnssFix.d_tow = pz_GnssFix->SyncTime_ms;
  z_GnssFix.t_timestamp = (uint64_t)pz_GnssFix->SyncTime_ms;
  z_GnssFix.f_avgsnr = pz_GnssFix->avg_CN0;
  z_GnssFix.w_week = pz_GnssFix->GPSWeek;
  z_GnssFix.f_hor_accu = pz_GnssFix->HAcc;
  z_GnssFix.u_satused = pz_GnssFix->SvNumMst;

  //z_GnssFix.u_msg_type = (GnssMsgType_Enum)GnssData.RecMsgEkf;
  //z_GnssFix.f_heading_deg = pz_GnssFix->Heading_deg;
  //z_GnssFix.f_heading_std = pz_GnssFix->HeadingStd;
  //z_GnssFix.f_pitch_deg = pz_GnssFix->Pitch_deg;
  //z_GnssFix.f_pitch_std = pz_GnssFix->PitchStd;

  switch (pz_GnssFix->PosFlag)
  {
  case GNSS_STATE_NARROW_INT:
    z_GnssFix.u_postype = GNSS_FIX_FLAG_FIXED;
    break;
  case GNSS_STATE_NARROW_FLOAT:
    z_GnssFix.u_postype = GNSS_FIX_FLAG_FLOATING;
    break;
  case GNSS_STATE_PSRDIFF:
    z_GnssFix.u_postype = GNSS_FIX_FLAG_DGNSS;
    break;
  case GNSS_STATE_SINGLE:
    z_GnssFix.u_postype = GNSS_FIX_FLAG_SPS;
    break;
  default:
    z_GnssFix.u_postype = GNSS_FIX_FLAG_INVALID;
    break;
  }

  vdr_api_GnssFix_Put(&z_GnssFix);

  return;
}

/**
 * @brief     Inject PPS time data
 * @param[in] pz_pps - pointer to loc_api_pps_t
 * @return    None
 */
void loc_api_InjectPPS(loc_api_pps_t* pz_pps)
{
  return;
}

/********************Memory Pool Manager Module API***************************/

/**
 * @brief Create a memory pool instance
 * @param[in] pointer to loc_api_MemoryPoolManager_t
 * @return TRUE - Success FALSE - Fail
 */
void loc_api_MemoryPoolCreate(loc_api_MemoryPoolManager_t* pz_MemoryPoolManager)
{
  mw_api_MemoryPoolCreate(pz_MemoryPoolManager);
  return;
}

/**
 * @brief Destroy the memory pool instance
 * @param[in] pointer to loc_api_MemoryPoolManager_t
 * @return None
 */
void loc_api_MemoryPool_Destroy(loc_api_MemoryPoolManager_t* pz_MemoryPoolManager)
{
  mw_api_MemoryPool_Destroy(pz_MemoryPoolManager);
  return;
}

/**
 * @brief Malloc of a memory pool
 * @param[in] pointer to loc_api_MemoryPoolManager_t
 * @param[in] memory size
 * @return Pointer to memory
 */
void* loc_api_MemoryPool_Malloc(loc_api_MemoryPoolManager_t* pz_MemoryPoolManager, uint32_t size)
{
  return mw_api_MemoryPool_Malloc(pz_MemoryPoolManager, size);
}

/**
 * @brief Free of a memory pool
 * @param[in] pointer to loc_api_MemoryPoolManager_t
 * @param[in] pointer need to free
 * @return
 */
void loc_api_MemoryPool_Free(loc_api_MemoryPoolManager_t* pz_MemoryPoolManager, void* ptr)
{
  mw_api_MemoryPool_Free(pz_MemoryPoolManager, ptr);
  return;
}


/**
* @brief Location Engine API for get version
* @return      Gnss version string
*/
const char* loc_api_GetVersion()
{
    static char s_aVersion[128];
    uint32_t q_len = 0;

    memset(s_aVersion,0,128);
    q_len = get_version(s_aVersion);
    s_aVersion[q_len] = '\n';
    return s_aVersion;
}



/********************End Memory Pool Manager Module API***********************/
