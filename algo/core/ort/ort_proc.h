/**@file        ort_proc.h
 * @brief       Orientation Main Process module
 * @details
 * @author      chenjinhe
 * @date        2023/08/13
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/08/13  <td>0.1      <td>chenjinhe   <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#ifndef __ORT_PROC_H__
#define __ORT_PROC_H__
#include "cmn_def.h"
#include "rtk_type.h"
#include "gnss_type.h"
BEGIN_DECL

typedef enum
{
  ORT_SUCC = 0,
  ORT_MEM_FAIL,
  ORT_TIME_REPEAT,
  ORT_GET_MAIN_ANT_PVT_FAIL,
  ORT_GET_MAIN_PVT_FAIL,
  ORT_GET_MAIN_ANT_OBS_FAIL,
  ORT_GET_AUXI_ANT_OBS_FAIL,
  ORT_GET_AUXI_ANT_PVT_FAIL,
  ORT_FILTER_FAIL
}ort_solStatus_t;
typedef uint8_t ort_solStatus;

/**
 * @brief Initialze Orientation module
 * @param[in]  pz_opt represent the algorithm option for the Orientation
 * @return     status -- 0: initilize success, other: fail
 */
uint8_t ort_init(const rtk_alg_opt_t* pz_opt);
/**
 * @brief deinitilize the Orientation algorithm
 * @param[in]  void
 * @param[out] void
 * @return     void
 */
void ort_deInit();

/**
 *@brief load the default option for the Orientation algorithm
 * @param[out]  pz_opt represent the algorithm optimization for the Orientation
 * @return     void
 */
void ort_loadDefaultOption(rtk_alg_opt_t* pz_opt);
/**
 * @brief push the main antenna measure block
 * @param[in]  pz_mainMeasBlock is the measure of main antenna
 * @param[out] void
 * @return     void
 */
void ort_pushMainAntMeas(const GnssMeasBlock_t* pz_mainMeasBlock);
/**
 * @brief push the main antenna position
 * @param[in]  pz_pvtInfo is the position result of main antenna
 * @param[out] void
 * @return     void
 */
void ort_pushMainAntPosition(const gnss_PVTResult_t* pz_pvtInfo);
/**
 * @brief push the auxiliary antenna measure block
 * @param[in]  pz_auxiMeasBlock is the measure of auxiliary antenna
 * @param[out] void
 * @return     void
 */
void ort_pushAuxiAntMeas(const GnssMeasBlock_t* pz_auxiMeasBlock);
/**
 * @brief process the main antenna and auxiliary antenna measure block to get heading .etc
 * @param[in]  z_obsTime is observation time
 * @param[out] pz_ortResult is the result of heading .etc
 * @param[out] pz_mainAntTdcpMeasBlock the main measure block expressed by TDCP struct
 * @param[out] pz_auxiAntTdcpMeasBlock the auxiliary measure block expressed by TDCP struct
 * @return     0 represent success and other failure
 */
ort_solStatus ort_startProcess(GpsTime_t z_obsTime, gnss_OrientFix_t* pz_ortResult, gnss_TdcpMeasBlock_t* pz_mainAntTdcpMeasBlock, gnss_TdcpMeasBlock_t* pz_auxiAntTdcpMeasBlock);
END_DECL
#endif