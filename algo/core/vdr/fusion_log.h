/**@file        fusion_log.h
 * @brief		fusion log file
 * @version     V1.0
 * @copyright   Copyright (c) 2022-2022  Guangzhou Asensing Techology Co.,Ltd.
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/01/21  <td>0.1      <td>shaobing   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __FUSION_LOG_H__
#define __FUSION_LOG_H__

#include <stdint.h>
#include "fusion_api.h"
#include "fusion_err_model.h"
#include "mw_log.h"
#include "fusion_data_encode.h"

#define LOG_OUTPUT_MAX_MSG_LEN 1024
#define LOG_MSG_DISABLED             (0x0000U)
#define LOG_MSG_INFO                 (0x0001U)
#define LOG_MSG_WARNING              (0x0002U)
#define LOG_MSG_ERROR                (0x0004U)
#define LOG_MSG_NMEA                 (0x0008U)
#define LOG_MSG_NAV                  (0x0010U)
#define LOG_MSG_INS                  (0x0020U)
#define LOG_MSG_INS_DEBUG            (0x0040U)
#define LOG_MSG_ALL (LOG_MSG_INFO|LOG_MSG_WARNING|LOG_MSG_ERROR|LOG_MSG_NMEA|LOG_MSG_INS|LOG_MSG_INS_DEBUG|LOG_MSG_NAV)

typedef void (*fusion_log_cb)(uint8_t* buf, uint32_t len);
void fusion_log(uint32_t msg_filter, const uint8_t* format, ...);
void fusion_log_callback_register(fusion_log_cb z_fusion_cb);
void fusion_log_filter_set(uint32_t q_output_mask);
void fusion_log_initial(void* pz_log_cb);
void vdr_save_imu_log(AsensingIMU_t* pz_AsensingIMU);
void vdr_save_wheel_log(AsensingWhpulse_t* pz_AsensingWheel);
void vdr_save_gnss_log(AsensingGNSSPara_t* pz_AsensingGNSS);
void vdr_save_cp_meas_log(AsensingGNSSCpMeas_t* pz_gnss_cp_meas);
void vdr_save_pr_meas_log(AsensingGNSSPsrMeas_t* pz_gnss_psr_meas);
void vdr_save_ins_log(INSResults_t* pz_InsPositionFix);
void vdr_save_inspl_log(INSIntegrity_t* pz_integrity);

#endif // !__FUSION_LOG_H__
