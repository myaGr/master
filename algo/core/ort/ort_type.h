/**@file        ort_type.h
 * @brief       The type used by orientation
 * @details
 * @author      chenjinhe
 * @date        2023/08/17
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/08/17  <td>0.1      <td>chenjinhe   <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#ifndef __ORT_TYPE_H__
#define __ORT_TYPE_H__
#include "cmn_def.h"
#include "gnss_type.h"
#include "cmn_turboEdit.h"
#include "cmn_dopplerDetectCS.h"
typedef struct
{
  cmn_turboEditFilterSet z_ortGfDetectFilter;                     /*the GF detector*/
  cmn_turboEditFilterSet z_ortMwDetectFilter;                     /*the MW detector*/
  gnss_dopplerDetectPairingBlock_t z_dopplerDetectPairingBlockOrt;/*the doppler detector*/
  gnss_TdcpMeasBlock_t* pz_preTdcpMeasOrt;                        /*the previous epoch observations used by TDCP detector*/
  uint8_t u_tdcpMethodValid;                                      /*the mask of TDCP method valid*/
  double d_deltaTimeDopplerDetect;                                /*the delta time between current epoch and previous epoch for doppler detector*/
}ort_cycleSlipDetectObj;//the object used by cycle slip detect in orientation mode

/**
 * @brief Initialze cycle slip detector object used by orientation
 * @param[in]  pz_detectObj represent the object used by cycle slip detect in orientation mode
 * @return     void
 */
void ort_initCycleSlipDetectObject(ort_cycleSlipDetectObj* pz_detectObj);

/**
 * @brief de-initialze cycle slip detector object used by orientation
 * @param[in]  pz_detectObj represent the object used by cycle slip detect in orientation mode
 * @return     void
 */
void ort_deInitCycleSlipDetectObject(ort_cycleSlipDetectObj* pz_detectObj);
#endif