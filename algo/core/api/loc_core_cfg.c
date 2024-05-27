/**@file        loc_core_cfg.c
 * @brief       Location engine configuration
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/05/21  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#include "cmn_utils.h"
#include "os_mutex.h"
#include "loc_core_api.h"
#include "loc_core_cfg.h"
#include "mw_log.h"
#include "mw_ipctask.h"

typedef struct {
  uint8_t              b_init;
  mutex_t              mtx;
  loc_api_config_para_group_t  loc_api_config_para;
  loc_ConfigParamGroup_t config;
} loc_InternalConfiguraParam_t;

static loc_InternalConfiguraParam_t gz_loc_internal_config = {0};

/**
 * @brief location core configuration initialze
 * @return None
 */
void loc_cfg_initialize()
{
  gz_loc_internal_config.b_init = TRUE;
  mutex_init(&gz_loc_internal_config.mtx, NULL);
}

/**
 * @brief Convert user set config mask to location core internal config parameter
 * @param[in]   user_cfg_mask
 * @param[in]   pz_loc_cfg_param_grp
 * @return      None
 */
void loc_cfg_CvtUserParamToInternal(const loc_api_config_para_group_t* user_cfg_para_grp,
  loc_ConfigParamGroup_t* pz_loc_cfg_param_grp)
{
  if (any_Ptrs_Null(2, user_cfg_para_grp, pz_loc_cfg_param_grp))
  {
    return;
  }

  /** LOG configuration */
  pz_loc_cfg_param_grp->log_cfg.b_enable =
    M_IS_SET_MASK(user_cfg_para_grp->para[6].t_mask, LOC_API_CFG_MASK_6_LOG_ENABLE);
  pz_loc_cfg_param_grp->log_cfg.u_logTextLevelDebug =
    M_IS_SET_MASK(user_cfg_para_grp->para[6].t_mask, LOC_API_CFG_MASK_6_LOG_TEXT_LEVEL_DEBUG);
  pz_loc_cfg_param_grp->log_cfg.u_logTextLevelInfo =
    M_IS_SET_MASK(user_cfg_para_grp->para[6].t_mask, LOC_API_CFG_MASK_6_LOG_TEXT_LEVEL_INFO);
  pz_loc_cfg_param_grp->log_cfg.u_logTextLevelWarning =
    M_IS_SET_MASK(user_cfg_para_grp->para[6].t_mask, LOC_API_CFG_MASK_6_LOG_TEXT_LEVEL_WARNING);
  pz_loc_cfg_param_grp->log_cfg.u_logTextLevelError =
    M_IS_SET_MASK(user_cfg_para_grp->para[6].t_mask, LOC_API_CFG_MASK_6_LOG_TEXT_LEVEL_ERROR);
  pz_loc_cfg_param_grp->log_cfg.u_logTextOutputIpc =
    M_IS_SET_MASK(user_cfg_para_grp->para[6].t_mask, LOC_API_CFG_MASK_6_LOG_TEXT_OUTPUT_IPC);
  pz_loc_cfg_param_grp->log_cfg.u_logTextOutputRaw =
    M_IS_SET_MASK(user_cfg_para_grp->para[6].t_mask, LOC_API_CFG_MASK_6_LOG_TEXT_OUTPUT_RAW);
  pz_loc_cfg_param_grp->log_cfg.u_logIpcMaskMode1 =
    M_IS_SET_MASK(user_cfg_para_grp->para[6].t_mask, LOC_API_CFG_MASK_6_LOG_IPC_MASK_MODE1);
  pz_loc_cfg_param_grp->log_cfg.u_logIpcMaskMode2 =
    M_IS_SET_MASK(user_cfg_para_grp->para[6].t_mask, LOC_API_CFG_MASK_6_LOG_IPC_MASK_MODE2);
  pz_loc_cfg_param_grp->log_cfg.u_logIpcMaskMode3 =
    M_IS_SET_MASK(user_cfg_para_grp->para[6].t_mask, LOC_API_CFG_MASK_6_LOG_IPC_MASK_MODE3);
  pz_loc_cfg_param_grp->log_cfg.u_logIpcMaskMode4 =
      M_IS_SET_MASK(user_cfg_para_grp->para[6].t_mask, LOC_API_CFG_MASK_6_LOG_IPC_MASK_MODE4);
  pz_loc_cfg_param_grp->log_cfg.u_logIpcMaskMode5 =
      M_IS_SET_MASK(user_cfg_para_grp->para[6].t_mask, LOC_API_CFG_MASK_6_LOG_IPC_MASK_MODE5);
  pz_loc_cfg_param_grp->log_cfg.u_logCompressEnable =
    M_IS_SET_MASK(user_cfg_para_grp->para[6].t_mask, LOC_API_CFG_MASK_6_LOG_COMPRESS_ENABLE);
  if (pz_loc_cfg_param_grp->log_cfg.u_logTextLevelDebug)
  {
    log_SetLogLevel(LOG_LEVEL_D);
  }
  else if (pz_loc_cfg_param_grp->log_cfg.u_logTextLevelInfo)
  {
    log_SetLogLevel(LOG_LEVEL_I);
  }
  else if (pz_loc_cfg_param_grp->log_cfg.u_logTextLevelWarning)
  {
    log_SetLogLevel(LOG_LEVEL_W);
  }
  else if (pz_loc_cfg_param_grp->log_cfg.u_logTextLevelError)
  {
    log_SetLogLevel(LOG_LEVEL_E);
  }
  else
  {
    log_SetLogLevel(LOG_LEVEL_NONE);
  }

  if (pz_loc_cfg_param_grp->log_cfg.u_logTextOutputIpc)
  {
    log_SetLogOutputType(LOG_OUTPUT_IPC);
    pz_loc_cfg_param_grp->log_cfg.u_logTextOutputRaw = 0;
  }
  else if (pz_loc_cfg_param_grp->log_cfg.u_logTextOutputRaw)
  {
    log_SetLogOutputType(LOG_OUTPUT_RAW);
  } 
  pz_loc_cfg_param_grp->log_cfg.u_logTimeOption = user_cfg_para_grp->para[6].u_field[0];

  ipc_filter_t z_ipc_filter = { 0 };
  uint32_t q_filter_ipc_index = 0;

  /** SM configuration */
  pz_loc_cfg_param_grp->sm_cfg.b_enable =
    M_IS_SET_MASK(user_cfg_para_grp->para[0].t_mask, LOC_API_CFG_MASK_0_SM_ENABLE);
  pz_loc_cfg_param_grp->sm_cfg.b_enable_rcv_rtcm_dec =
    M_IS_SET_MASK(user_cfg_para_grp->para[0].t_mask, LOC_API_CFG_MASK_0_SM_RCV_RTCM_DEC_ENABLE);
  pz_loc_cfg_param_grp->sm_cfg.b_enable_ref_rtcm_dec =
    M_IS_SET_MASK(user_cfg_para_grp->para[0].t_mask, LOC_API_CFG_MASK_0_SM_REF_RTCM_DEC_ENABLE);
  pz_loc_cfg_param_grp->sm_cfg.b_enable_rcv_twin_rtcm_dec =
    M_IS_SET_MASK(user_cfg_para_grp->para[0].t_mask, LOC_API_CFG_MASK_0_SM_RCV_TWIN_RTCM_DEC_ENABLE);
  pz_loc_cfg_param_grp->sm_cfg.b_enable_ssr_service_qxwz =
    M_IS_SET_MASK(user_cfg_para_grp->para[0].t_mask, LOC_API_CFG_MASK_0_SM_QXWZ_SSR_ENABLE);
  pz_loc_cfg_param_grp->sm_cfg.b_enable_pos_and_meas_report =
    M_IS_SET_MASK(user_cfg_para_grp->para[0].t_mask, LOC_API_CFG_MASK_0_SM_POS_AND_MEAS_REPORT_ENABLE);
  pz_loc_cfg_param_grp->sm_cfg.b_disable_decode_rtcm_obs =
    M_IS_SET_MASK(user_cfg_para_grp->para[0].t_mask, LOC_API_CFG_MASK_0_SM_DECODE_RTCM_OBS_DISABLE);
  pz_loc_cfg_param_grp->sm_cfg.b_disable_decode_rtcm_nav =
    M_IS_SET_MASK(user_cfg_para_grp->para[0].t_mask, LOC_API_CFG_MASK_0_SM_DECODE_RTCM_NAV_DISABLE);

  pz_loc_cfg_param_grp->sm_cfg.f_meas_update_interval = loc_cfg_intervalMask2Interval(user_cfg_para_grp->para[0].t_mask);
  pz_loc_cfg_param_grp->sm_cfg.f_al = user_cfg_para_grp->para[0].f_field[0];
  if(pz_loc_cfg_param_grp->sm_cfg.f_al <= FABS_ZEROS)
  {
    pz_loc_cfg_param_grp->sm_cfg.f_al = LOC_API_DEFAULT_ALERT_LIMIT;   // default AL=3.0
  }
  /*configure the elevation mask*/
  pz_loc_cfg_param_grp->sm_cfg.f_ele_mask = LOC_API_DEFAULT_ELE_MASK_LIMIT;   // default elevation mask=10.0
  if (user_cfg_para_grp->para[0].f_field[1] >= FABS_ZEROS && user_cfg_para_grp->para[0].f_field[1] <= 90.0f)
  {
    pz_loc_cfg_param_grp->sm_cfg.f_ele_mask = user_cfg_para_grp->para[0].f_field[1];
  }

  if (pz_loc_cfg_param_grp->sm_cfg.b_enable)
  {
    z_ipc_filter.task_enable_list[TASK_INDEX_SM] = TRUE;

    if (pz_loc_cfg_param_grp->log_cfg.u_logIpcMaskMode1)
    {
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_SM_INIT;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_SM_START;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_SM_STOP;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_SM_RELEASE;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_SM_SSR_STREAM_QX;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_SM_SSR_STREAM_GEE;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_SM_RCV_OBS_RTCM;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_SM_REF_CORR_RTCM;
    }

    if (pz_loc_cfg_param_grp->log_cfg.u_logIpcMaskMode2)
    {
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_SM_REPORT_POS_FIX;
    }

    if (pz_loc_cfg_param_grp->log_cfg.u_logIpcMaskMode3)
    {    
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_SM_SSR_LOS_BLK;
    }
    if (pz_loc_cfg_param_grp->log_cfg.u_logIpcMaskMode4)
    {
        z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_SM_TWIN_RCV_OBS_RTCM;
    }
  }

  /** HPP configuration */
  pz_loc_cfg_param_grp->hpp_cfg.b_enable =
    M_IS_SET_MASK(user_cfg_para_grp->para[1].t_mask, LOC_API_CFG_MASK_1_HPP_ENABLE);
  pz_loc_cfg_param_grp->hpp_cfg.b_enable_pvt =
    M_IS_SET_MASK(user_cfg_para_grp->para[1].t_mask, LOC_API_CFG_MASK_1_HPP_PVT_ENABLE);
  pz_loc_cfg_param_grp->hpp_cfg.b_enable_rtk =
    M_IS_SET_MASK(user_cfg_para_grp->para[1].t_mask, LOC_API_CFG_MASK_1_HPP_RTK_ENABLE);
  pz_loc_cfg_param_grp->hpp_cfg.b_enable_orient =
    M_IS_SET_MASK(user_cfg_para_grp->para[1].t_mask, LOC_API_CFG_MASK_1_HPP_ORIENT_ENABLE);

  if (pz_loc_cfg_param_grp->hpp_cfg.b_enable)
  {
    z_ipc_filter.task_enable_list[TASK_INDEX_HPP] = TRUE;

    if (pz_loc_cfg_param_grp->log_cfg.u_logIpcMaskMode1)
    {
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_HPP_INIT;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_HPP_START;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_HPP_STOP;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_HPP_RELEASE;
    }

    if (pz_loc_cfg_param_grp->log_cfg.u_logIpcMaskMode2)
    {

    }

    if (pz_loc_cfg_param_grp->log_cfg.u_logIpcMaskMode3)
    {
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_HPP_RCV_MEAS_1HZ_PUT;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_HPP_REF_CORR_PUT;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_HPP_EPHEMERIS_PUT;
    }
    pz_loc_cfg_param_grp->hpp_cfg.f_max_age = user_cfg_para_grp->para[1].f_field[0];
    pz_loc_cfg_param_grp->hpp_cfg.f_max_prefix_age = user_cfg_para_grp->para[1].f_field[1];
  }

  /** DCP configuration */
  pz_loc_cfg_param_grp->dcp_cfg.b_enable =
    M_IS_SET_MASK(user_cfg_para_grp->para[2].t_mask, LOC_API_CFG_MASK_2_DCP_ENABLE);
  if (pz_loc_cfg_param_grp->dcp_cfg.b_enable)
  {
    z_ipc_filter.task_enable_list[TASK_INDEX_DCP] = TRUE;

    if (pz_loc_cfg_param_grp->log_cfg.u_logIpcMaskMode1)
    {
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_DCP_INIT;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_DCP_START;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_DCP_STOP;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_DCP_RELEASE;
    }

    if (pz_loc_cfg_param_grp->log_cfg.u_logIpcMaskMode2)
    {
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_DCP_RCV_MEAS_NHZ_PUT;
    }

    if (pz_loc_cfg_param_grp->log_cfg.u_logIpcMaskMode3)
    {
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_DCP_TDCP_MEAS_PUT;
    }
  }

  /** ORT configuration */
  pz_loc_cfg_param_grp->ort_cfg.b_enable =
    M_IS_SET_MASK(user_cfg_para_grp->para[2].t_mask, LOC_API_CFG_MASK_2_ORT_ENABLE);
  if (pz_loc_cfg_param_grp->ort_cfg.b_enable)
  {
    z_ipc_filter.task_enable_list[TASK_INDEX_ORT] = TRUE;
    pz_loc_cfg_param_grp->sm_cfg.b_enable_rcv_twin_rtcm_dec = TRUE;

    if (pz_loc_cfg_param_grp->log_cfg.u_logIpcMaskMode1)
    {
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_ORT_INIT;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_ORT_START;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_ORT_STOP;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_ORT_RELEASE;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_SM_REPORT_ORT_FIX;
    }

    if (pz_loc_cfg_param_grp->log_cfg.u_logIpcMaskMode2)
    {

    }

    if (pz_loc_cfg_param_grp->log_cfg.u_logIpcMaskMode3)
    {
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_ORT_RCV_MEAS_PUT;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_ORT_ORT_MEAS_PUT;
    }
  }

  /** PPP configuration */
  pz_loc_cfg_param_grp->ppp_cfg.b_enable =
    M_IS_SET_MASK(user_cfg_para_grp->para[3].t_mask, LOC_API_CFG_MASK_3_PPP_ENABLE);
  if (pz_loc_cfg_param_grp->ppp_cfg.b_enable)
  {
    z_ipc_filter.task_enable_list[TASK_INDEX_PPP] = TRUE;

    if (pz_loc_cfg_param_grp->log_cfg.u_logIpcMaskMode1)
    {
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_PPP_INIT;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_PPP_START;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_PPP_STOP;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_PPP_RELEASE;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_PPP_ALGO_START;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_PPP_SSR_LOS_BLK;
    }

    if (pz_loc_cfg_param_grp->log_cfg.u_logIpcMaskMode2)
    {
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_PPP_SSR_STREAM_AG_LOS;
    }

    if (pz_loc_cfg_param_grp->log_cfg.u_logIpcMaskMode3)
    {
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_PPP_SSR_STREAM_QX;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_PPP_TIGHT_SAT_SIGNAL_MEAS_PUT;
    }
  }

  /** SD configuration */
  pz_loc_cfg_param_grp->sd_cfg.b_enable =
    M_IS_SET_MASK(user_cfg_para_grp->para[4].t_mask, LOC_API_CFG_MASK_4_SD_ENABLE);
  if (pz_loc_cfg_param_grp->sd_cfg.b_enable)
  {
    z_ipc_filter.task_enable_list[TASK_INDEX_SD] = TRUE;

    if (pz_loc_cfg_param_grp->log_cfg.u_logIpcMaskMode1)
    {
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_SD_INIT;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_SD_START;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_SD_STOP;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_SD_RELEASE;
    }

    if (pz_loc_cfg_param_grp->log_cfg.u_logIpcMaskMode2)
    {

    }

    if (pz_loc_cfg_param_grp->log_cfg.u_logIpcMaskMode3)
    {
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_SD_EPHEMERIS_PUT;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_SD_SAT_PVT_POLY_REQUEST;
    }
  }

  /** VDR configuration */
  pz_loc_cfg_param_grp->vdr_cfg.b_enable =
    M_IS_SET_MASK(user_cfg_para_grp->para[5].t_mask, LOC_API_CFG_MASK_5_VDR_ENABLE);
  memcpy(pz_loc_cfg_param_grp->vdr_cfg.u_field, user_cfg_para_grp->para[5].u_field,
      sizeof(pz_loc_cfg_param_grp->vdr_cfg.u_field));
  memcpy(pz_loc_cfg_param_grp->vdr_cfg.f_field, user_cfg_para_grp->para[5].f_field,
      sizeof(pz_loc_cfg_param_grp->vdr_cfg.f_field));
  if (pz_loc_cfg_param_grp->vdr_cfg.b_enable)
  {
    z_ipc_filter.task_enable_list[TASK_INDEX_VDR] = TRUE;

    if (pz_loc_cfg_param_grp->log_cfg.u_logIpcMaskMode1)
    {
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_VDR_INIT;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_VDR_START;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_VDR_STOP;
      z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_VDR_RELEASE;
    }

    if (pz_loc_cfg_param_grp->log_cfg.u_logIpcMaskMode2)
    {

    }

    if (pz_loc_cfg_param_grp->log_cfg.u_logIpcMaskMode3)
    {

    }

    if (pz_loc_cfg_param_grp->log_cfg.u_logIpcMaskMode5)
    {
	  z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_VDR_WHEEL_DATA_PUT;
    z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_VDR_IMU_DATA_PUT;
	  z_ipc_filter.filter_need_saved_ipc[q_filter_ipc_index++] = C_M_VDR_GNSS_FIX_PUT;
    }

  }
  z_ipc_filter.filter_need_saved_ipc_count = q_filter_ipc_index;
  ipctask_ipc_filter_set(&z_ipc_filter);

  /** IPC configuration */
  pz_loc_cfg_param_grp->ipc_cfg.b_multiTask =
    M_IS_SET_MASK(user_cfg_para_grp->para[7].t_mask, LOC_API_CFG_MASK_7_IPC_MULTI);
  pz_loc_cfg_param_grp->ipc_cfg.b_singleTask =
    M_IS_SET_MASK(user_cfg_para_grp->para[7].t_mask, LOC_API_CFG_MASK_7_IPC_SINGLE);

  if (pz_loc_cfg_param_grp->ipc_cfg.b_multiTask)
  {
    pz_loc_cfg_param_grp->ipc_cfg.b_singleTask = 0;
  } 
  else if (pz_loc_cfg_param_grp->ipc_cfg.b_singleTask)
  {
    pz_loc_cfg_param_grp->ipc_cfg.b_singleTask = 1;
  }
}

/**
 * @brief Set All configuration parameter structure
 * @param[in]   pz_locConfigParamGroup
 * @return      None
 */
BOOL loc_cfg_SetConfigParamGroup(const loc_api_config_para_group_t* pz_api_config_para_grp)
{
  if (FALSE == gz_loc_internal_config.b_init)
  {
    return FALSE;
  }

  mutex_lock(&gz_loc_internal_config.mtx);

  memcpy(&gz_loc_internal_config.loc_api_config_para,
    pz_api_config_para_grp, sizeof(loc_api_config_para_group_t));
  loc_cfg_CvtUserParamToInternal(pz_api_config_para_grp, &(gz_loc_internal_config.config));

  mutex_unlock(&gz_loc_internal_config.mtx);

  return TRUE;
}

/**
 * @brief Get All configuration parameter structure
 * @return      pz_locConfigParamGroup
 */
loc_ConfigParamGroup_t* loc_cfg_getConfigParamGroup()
{
  if (FALSE == gz_loc_internal_config.b_init)
  {
    return NULL;
  }

  return &(gz_loc_internal_config.config);
}

/**
 * @brief Get configuration parameter structure 
 * @return      loc_api_config_para_group_t
 */
loc_api_config_para_group_t* loc_cfg_getLocApiConfigPara()
{
  if (FALSE == gz_loc_internal_config.b_init)
  {
    return NULL;
  }

  return &(gz_loc_internal_config.loc_api_config_para);
}

/**
 * @brief convert GNSS process inteval from mask
 * @param[in] mask 
 * @return float 
 */
float loc_cfg_intervalMask2Interval(uint64_t mask)
{
  float f_inteval = 1.0f;
  if (M_IS_SET_MASK(mask, LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_1))
  {
    f_inteval = 0.1f;
  }
  else if (M_IS_SET_MASK(mask, LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_2))
  {
    f_inteval = 0.2f;
  }
  else if (M_IS_SET_MASK(mask, LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_3))
  {
    f_inteval = 0.3f;
  }
  else if (M_IS_SET_MASK(mask, LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_4))
  {
    f_inteval = 0.4f;
  }
  else if (M_IS_SET_MASK(mask, LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_5))
  {
    f_inteval = 0.5f;
  }
  else if (M_IS_SET_MASK(mask, LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_6))
  {
    f_inteval = 0.6f;
  }
  else if (M_IS_SET_MASK(mask, LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_7))
  {
    f_inteval = 0.7f;
  }
  else if (M_IS_SET_MASK(mask, LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_8))
  {
    f_inteval = 0.8f;
  }
  else if (M_IS_SET_MASK(mask, LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_9))
  {
    f_inteval = 0.9f;
  }
  else if (M_IS_SET_MASK(mask, LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_10))
  {
    f_inteval = 1.0f;
  }
  return f_inteval;
}
