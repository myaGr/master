/**@file        cmn_highPrecision.h
 * @brief       load and march high precision reference position
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/10/26  <td>0.1      <td>zhangxuecheng <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#ifndef __CMN_HIGH_PRECISION_H__
#define __CMN_HIGH_PRECISION_H__

//#ifdef HPREF

#include "gnss_type.h"
#include "cmn_def.h"

BEGIN_DECL

typedef struct {
  double d_tod;
  double pd_pos[3];
} cmn_highPrecision_oneEpoch;

typedef struct cmn_highPrecision_oneNode {
  uint8_t u_valid;
  uint8_t u_epochNum;
  double d_beginTod;
  double d_endTod;
  cmn_highPrecision_oneEpoch* pz_data;
  struct cmn_highPrecision_oneNode* pz_next;
} cmn_highPrecision_oneNode;

/**
  * @brief init the high precision structure
  * @return TRUE represent success and FALSE for fail
  */
uint8_t cmn_highPrecision_init();

/**
  * @brief deinit the high precision structure
  * @return TRUE represent success and FALSE for fail
  */
uint8_t cmn_highPrecision_deinit();

/**
  * @brief  inject the high precision data
  * @param[in]      u_num the inject high precision epoch num
  * @param[in]      pz_highPrecision_array the injected data
  * @return TRUE represent success and FALSE for fail
  */
uint8_t cmn_highPrecision_inject(uint8_t u_num, cmn_highPrecision_oneEpoch* pz_highPrecision_array);

/**
  * @brief  search the high precision data
  * @param[in]      z_tor the target time
  * @param[out]     pd_pos the high precision reference
  * @return TRUE represent success and FALSE for fail
  */
uint8_t cmn_highPrecision_search(GpsTime_t z_tor, double* pd_pos);

//#endif

END_DECL

#endif