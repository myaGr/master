/**@file        loc_core_cfg.h
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

#ifndef __LOC_CORE_CFG_H__
#define __LOC_CORE_CFG_H__

#include "cmn_def.h"
#include "loc_core_api.h"

BEGIN_DECL

/** 0:SM Configuration mask */
#define LOC_API_CFG_MASK_0_SM_ENABLE                        ((uint64_t)0x1<< 0)
#define LOC_API_CFG_MASK_0_SM_RCV_RTCM_DEC_ENABLE           ((uint64_t)0x1<< 1)
#define LOC_API_CFG_MASK_0_SM_REF_RTCM_DEC_ENABLE           ((uint64_t)0x1<< 2)
#define LOC_API_CFG_MASK_0_SM_RCV_TWIN_RTCM_DEC_ENABLE      ((uint64_t)0x1<< 3)
#define LOC_API_CFG_MASK_0_SM_QXWZ_SSR_ENABLE               ((uint64_t)0x1<< 4)
#define LOC_API_CFG_MASK_0_SM_POS_AND_MEAS_REPORT_ENABLE    ((uint64_t)0x1<< 5)
#define LOC_API_CFG_MASK_0_SM_DECODE_RTCM_OBS_DISABLE       ((uint64_t)0x1<< 6)
#define LOC_API_CFG_MASK_0_SM_DECODE_RTCM_NAV_DISABLE       ((uint64_t)0x1<< 7)
#define LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_1          ((uint64_t)0x1<< 8)
#define LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_2          ((uint64_t)0x1<< 9)
#define LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_3          ((uint64_t)0x1<< 10)
#define LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_4          ((uint64_t)0x1<< 11)
#define LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_5          ((uint64_t)0x1<< 12)
#define LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_6          ((uint64_t)0x1<< 13)
#define LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_7          ((uint64_t)0x1<< 14)
#define LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_8          ((uint64_t)0x1<< 15)
#define LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_9          ((uint64_t)0x1<< 16)
#define LOC_API_CFG_MASK_0_SM_MEAS_UPDATE_SAMPLE_10         ((uint64_t)0x1<< 17)

/** 1:HPP Configuration mask */
#define LOC_API_CFG_MASK_1_HPP_ENABLE                       ((uint64_t)0x1<< 0)
#define LOC_API_CFG_MASK_1_HPP_PVT_ENABLE                   ((uint64_t)0x1<< 1)
#define LOC_API_CFG_MASK_1_HPP_RTK_ENABLE                   ((uint64_t)0x1<< 2)
#define LOC_API_CFG_MASK_1_HPP_ORIENT_ENABLE                ((uint64_t)0x1<< 3)
#define LOC_API_CFG_MASK_1_HPP_RCV_RTCM_DEC_ENABLE          ((uint64_t)0x1<< 4)
#define LOC_API_CFG_MASK_1_HPP_REF_RTCM_DEC_ENABLE          ((uint64_t)0x1<< 5)
#define LOC_API_CFG_MASK_1_HPP_RCV_TWIN_RTCM_DEC_ENABLE     ((uint64_t)0x1<< 6)


/** 2:DCP Configuration mask */
#define LOC_API_CFG_MASK_2_DCP_ENABLE                       ((uint64_t)0x1<< 0)
/** Orient Configuration mask */
#define LOC_API_CFG_MASK_2_ORT_ENABLE                       ((uint64_t)0x1<<32)

/** 3:PPP Configuration mask */
#define LOC_API_CFG_MASK_3_PPP_ENABLE                       ((uint64_t)0x1<< 0)

/** 4:SD Configuration mask */
#define LOC_API_CFG_MASK_4_SD_ENABLE                        ((uint64_t)0x1<< 0)

/** 5:VDR Configuration mask */
#define LOC_API_CFG_MASK_5_VDR_ENABLE                       ((uint64_t)0x1<< 0)

/** 6:LOG Configuration mask */
#define LOC_API_CFG_MASK_6_LOG_ENABLE                       ((uint64_t)0x1<< 0)
#define LOC_API_CFG_MASK_6_LOG_TEXT_LEVEL_DEBUG             ((uint64_t)0x1<< 1)
#define LOC_API_CFG_MASK_6_LOG_TEXT_LEVEL_INFO              ((uint64_t)0x1<< 2)
#define LOC_API_CFG_MASK_6_LOG_TEXT_LEVEL_WARNING           ((uint64_t)0x1<< 3)
#define LOC_API_CFG_MASK_6_LOG_TEXT_LEVEL_ERROR             ((uint64_t)0x1<< 4)
#define LOC_API_CFG_MASK_6_LOG_TEXT_OUTPUT_IPC              ((uint64_t)0x1<< 5)
#define LOC_API_CFG_MASK_6_LOG_TEXT_OUTPUT_RAW              ((uint64_t)0x1<< 6)
#define LOC_API_CFG_MASK_6_LOG_IPC_MASK_MODE1               ((uint64_t)0x1<< 7)
#define LOC_API_CFG_MASK_6_LOG_IPC_MASK_MODE2               ((uint64_t)0x1<< 8)
#define LOC_API_CFG_MASK_6_LOG_IPC_MASK_MODE3               ((uint64_t)0x1<< 9)
#define LOC_API_CFG_MASK_6_LOG_IPC_MASK_MODE4               ((uint64_t)0x1<<10)
#define LOC_API_CFG_MASK_6_LOG_IPC_MASK_MODE5               ((uint64_t)0x1<<11)
#define LOC_API_CFG_MASK_6_LOG_IPC_MASK_MODE6               ((uint64_t)0x1<<12)
#define LOC_API_CFG_MASK_6_LOG_IPC_MASK_MODE7               ((uint64_t)0x1<<13)
#define LOC_API_CFG_MASK_6_LOG_IPC_MASK_MODE8               ((uint64_t)0x1<<14)
#define LOC_API_CFG_MASK_6_LOG_COMPRESS_ENABLE              ((uint64_t)0x1<<15)

/** 7:IPC Configuration mask */
#define LOC_API_CFG_MASK_7_IPC_MULTI                        ((uint64_t)0x1<< 0)
#define LOC_API_CFG_MASK_7_IPC_SINGLE                       ((uint64_t)0x1<< 1)

/* Integrity alert limit(AL), */
#define LOC_API_DEFAULT_ALERT_LIMIT                         ((float)3.0f)

/* elevation mask limit*/
#define LOC_API_DEFAULT_ELE_MASK_LIMIT                         ((float)10.0f)

typedef struct {
  uint64_t b_enable                    : 1;
  uint64_t b_enable_rcv_rtcm_dec       : 1;
  uint64_t b_enable_ref_rtcm_dec       : 1;
  uint64_t b_enable_rcv_twin_rtcm_dec  : 1;
  uint64_t b_enable_ssr_service_qxwz   : 1;
  uint64_t b_enable_pos_and_meas_report: 1;
  uint64_t b_disable_decode_rtcm_obs   : 1;
  uint64_t b_disable_decode_rtcm_nav   : 1;
  float    f_meas_update_interval;
  float    f_al;
  float    f_ele_mask;                    /*uint in degree*/
} sm_ConfigParam_t;

typedef struct {
  uint64_t b_enable                    : 1;
  uint64_t b_enable_pvt                : 1;
  uint64_t b_enable_rtk                : 1;
  uint64_t b_enable_orient             : 1;
  uint64_t b_enable_rcv_rtcm_dec       : 1;
  uint64_t b_enable_ref_rtcm_dec       : 1;
  uint64_t b_enable_rcv_twin_rtcm_dec  : 1;
  float  f_max_age;
  float    f_max_prefix_age;
} hpp_ConfigParam_t;

typedef struct {
  uint8_t b_enable                    : 1;
} sd_ConfigParam_t;

typedef struct {
  uint8_t b_enable                    : 1;
} dcp_ConfigParam_t;

typedef struct {
  uint8_t b_enable                    : 1;
} ppp_ConfigParam_t;

typedef struct {
  uint8_t b_enable                    : 1;
  uint8_t  u_field[10];
  float    f_field[10];
} vdr_ConfigParam_t;

typedef struct
{
  uint8_t b_enable                    : 1;
} ort_ConfigParam_t;

typedef struct {
  uint8_t b_enable                    : 1;
  uint8_t u_logTextLevelDebug         : 1;
  uint8_t u_logTextLevelInfo          : 1;
  uint8_t u_logTextLevelWarning       : 1;
  uint8_t u_logTextLevelError         : 1;
  uint8_t u_logTextOutputIpc          : 1;
  uint8_t u_logTextOutputRaw          : 1;
  uint8_t u_logIpcMaskMode1           : 1;
  uint8_t u_logIpcMaskMode2           : 1;
  uint8_t u_logIpcMaskMode3           : 1;
  uint8_t u_logIpcMaskMode4           : 1;
  uint8_t u_logIpcMaskMode5           : 1;
  uint8_t u_logCompressEnable         : 1;
  uint8_t u_logTimeOption;
} log_ConfigParam_t;

typedef struct {
  uint8_t b_multiTask                 : 1;
  uint8_t b_singleTask                : 1;
} ipc_ConfigParam_t;

#define VERSION_LOC_CONFIG_PARAM  (0)
typedef struct {
  uint8_t         u_version;
  sm_ConfigParam_t   sm_cfg;
  hpp_ConfigParam_t hpp_cfg;
  dcp_ConfigParam_t dcp_cfg;
  ppp_ConfigParam_t ppp_cfg;
  sd_ConfigParam_t   sd_cfg;
  vdr_ConfigParam_t vdr_cfg;
  ort_ConfigParam_t ort_cfg;
  log_ConfigParam_t log_cfg;
  ipc_ConfigParam_t ipc_cfg;
} loc_ConfigParamGroup_t;

typedef struct
{
  uint8_t u_bInit;
} loc_core_api_ctrl_t;

/**
 * @brief location core configuration initialze
 * @return      None
 */
void loc_cfg_initialize();

/**
 * @brief Convert user set config mask to location core internal config parameter
 * @param[in]   user_cfg_mask
 * @param[in]   loc_cfg_param
 * @return      None
 */
void loc_cfg_CvtUserParamToInternal(const loc_api_config_para_group_t* user_cfg_para_grp,
  loc_ConfigParamGroup_t* loc_cfg_param);

/**
 * @brief Set All configuration parameter structure
 * @param[in]   pz_loc_all_cfg_param
 * @return      None
 */
BOOL loc_cfg_SetConfigParamGroup(const loc_api_config_para_group_t* pz_api_config_para_grp);

/**
 * @brief Get All configuration parameter structure
 * @return      pz_locConfigParamGroup
 */
loc_ConfigParamGroup_t* loc_cfg_getConfigParamGroup();

/**
 * @brief Get configuration parameter structure
 * @return      loc_api_config_para_group_t
 */
loc_api_config_para_group_t* loc_cfg_getLocApiConfigPara();

/**
 * @brief convert GNSS process inteval from mask
 * @param[in] mask 
 * @return float 
 */
float loc_cfg_intervalMask2Interval(uint64_t mask);

END_DECL

#endif
