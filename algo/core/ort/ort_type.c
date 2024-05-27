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
#include "ort_type.h"
#include "mw_alloc.h"
 /**
  * @brief Initialze cycle slip detector object used by orientation
  * @param[in]  pz_detectObj represent the object used by cycle slip detect in orientation mode
  * @return     void
  */
void ort_initCycleSlipDetectObject(ort_cycleSlipDetectObj* pz_detectObj)
{
  if (NULL != pz_detectObj)
  {
    cmn_initTEfilterSet(&(pz_detectObj->z_ortGfDetectFilter));
    cmn_initTEfilterSet(&(pz_detectObj->z_ortMwDetectFilter));
    pz_detectObj->z_dopplerDetectPairingBlockOrt.pz_curBlock = NULL;
    pz_detectObj->z_dopplerDetectPairingBlockOrt.pz_preBlock = NULL;
    cmn_initDopplerDetectCyleSlip(&(pz_detectObj->z_dopplerDetectPairingBlockOrt));
    pz_detectObj->pz_preTdcpMeasOrt = (gnss_TdcpMeasBlock_t*)OS_MALLOC(sizeof(gnss_TdcpMeasBlock_t));
    pz_detectObj->u_tdcpMethodValid = 0;
    pz_detectObj->d_deltaTimeDopplerDetect = 0.0;
  }
  return;
}

/**
 * @brief de-initialze cycle slip detector object used by orientation
 * @param[in]  pz_detectObj represent the object used by cycle slip detect in orientation mode
 * @return     void
 */
void ort_deInitCycleSlipDetectObject(ort_cycleSlipDetectObj* pz_detectObj)
{
  if (NULL != pz_detectObj)
  {
    cmn_deinitTEfilterSet(&(pz_detectObj->z_ortGfDetectFilter));
    cmn_deinitTEfilterSet(&(pz_detectObj->z_ortMwDetectFilter));
    cmn_deinitDopplerDetectCyleSlip(&(pz_detectObj->z_dopplerDetectPairingBlockOrt));
    OS_FREE(pz_detectObj->pz_preTdcpMeasOrt);
  }
  return;
}