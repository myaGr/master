/**@file        dcp_ort_sol.c
 * @brief       solute the real-time orient using the carrier-phase, doppler or pseudo-range observations
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/11/08  <td>0.1      <td>chenjinhe   <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#include "dcp_ort_sol.h"
#include "dcp_solution.h"
#include "mw_alloc.h"
#include "gnss_common.h"
#include "cmn_utils.h"
#include <math.h>


#define DCP_AUXI_ANT_MAX_BLOCK             (2)                                              /*the maximum number of auxiliary antenna block, default > 1*/
#define DCP_MAIN_ANT_MAX_BLOCK             (2)                                              /*the maximum number of main antenna block, default > 1*/

BOOL gz_isOrtDcpMemoryAllocSucc = TRUE;
dcp_solInfo_t gz_dcpOrtMainSolInfo = { 0 };
dcp_solInfo_t gz_dcpOrtAuxiSolInfo = { 0 };
gnss_OrientFix_t gz_ortResult = { 0 };

gnss_TdcpMeasBlock_t* gpz_auxiAntDcpMeasBlock[DCP_AUXI_ANT_MAX_BLOCK] = { NULL };         /*[0] saves latest valid auxi antenna measure block,
                                                                                          [1] & [1+] save backward auxi antenna measure block*/

gnss_TdcpMeasBlock_t* gpz_mainAntDcpMeasBlock[DCP_MAIN_ANT_MAX_BLOCK] = { NULL };         /*[0] saves latest valid main antenna measure block,
                                                                                          [1] & [1+] save backward main antenna measure block*/

GpsTime_t gz_lastCalTime = { 0 };
/**
 * @brief initialize DCP orient model
 * @param[in]  pz_dcpConfig the configuration for the DCP model
 * @return void
 */
void DCP_ortInit(const gnss_DcpConfigOpt_t* pz_dcpConfig)
{
  uint16_t w_i = 0;
  memset(&gz_lastCalTime, 0, sizeof(GpsTime_t));
  memset(&gz_ortResult, 0, sizeof(gnss_OrientFix_t));
  DCP_init(pz_dcpConfig, &gz_dcpOrtMainSolInfo);
  DCP_init(pz_dcpConfig, &gz_dcpOrtAuxiSolInfo);
  for (w_i = 0; w_i < DCP_AUXI_ANT_MAX_BLOCK; ++w_i)
  {
    gpz_auxiAntDcpMeasBlock[w_i] = (gnss_TdcpMeasBlock_t*)OS_MALLOC(sizeof(gnss_TdcpMeasBlock_t));
    if (NULL == gpz_auxiAntDcpMeasBlock[w_i])
    {
      gz_isOrtDcpMemoryAllocSucc = FALSE;
      break;
    }
  }

  for (w_i = 0; w_i < DCP_MAIN_ANT_MAX_BLOCK; ++w_i)
  {
    gpz_mainAntDcpMeasBlock[w_i] = (gnss_TdcpMeasBlock_t*)OS_MALLOC(sizeof(gnss_TdcpMeasBlock_t));
    if (NULL == gpz_mainAntDcpMeasBlock[w_i])
    {
      gz_isOrtDcpMemoryAllocSucc = FALSE;
      break;
    }
  }
  return;
}

/**
*@brief deinitilize the DCP orient algorithm
* @param[in/out]  void
* @return         void
*/
void DCP_ortDeinit()
{
  uint16_t w_i = 0;
  DCP_deinit(&gz_dcpOrtMainSolInfo);
  DCP_deinit(&gz_dcpOrtAuxiSolInfo);
  for (w_i = 0; w_i < DCP_AUXI_ANT_MAX_BLOCK; ++w_i)
  {
    OS_FREE(gpz_auxiAntDcpMeasBlock[w_i]);
  }

  for (w_i = 0; w_i < DCP_MAIN_ANT_MAX_BLOCK; ++w_i)
  {
    OS_FREE(gpz_mainAntDcpMeasBlock[w_i]);
  }
  return;
}

/**
*@brief update the back ground result for main antenna
* @param[in]  pz_tdcpMeasBlock is TDCP measuremt for main antenna
* @return         void
*/
void DCP_updateMainAntBackGroundResult(const gnss_TdcpMeasBlock_t* pz_tdcpMeasBlock)
{
  dcp_updateBaseMeasBlock(pz_tdcpMeasBlock, &gz_dcpOrtMainSolInfo);
  return;
}

/**
*@brief update the back ground result for auxi antenna
* @param[in]  pz_tdcpMeasBlock is TDCP measuremt for main antenna
* @return         void
*/
void DCP_updateAuxiAntBackGroundResult(const gnss_TdcpMeasBlock_t* pz_tdcpMeasBlock)
{
  dcp_updateBaseMeasBlock(pz_tdcpMeasBlock, &gz_dcpOrtAuxiSolInfo);
  return;
}

/**
*@brief update the back ground result of orient
* @param[in]  pz_tdcpMeasBlock is TDCP measuremt for main antenna
* @return         void
*/
void DCP_updatOrtBackGroundResult(const gnss_OrientFix_t* pz_ortResult)
{
  gz_ortResult = *pz_ortResult;
  return;
}

/**
*@brief push auxi antenna measure
* @param[in]  pz_auxiCurentTdcpMeas is TDCP measuremt for auxi antenna of current epoch
* @return         void
*/
void DCP_PushRcvAuxiMeasBlk_NHz(const gnss_TdcpMeasBlock_t* pz_auxiCurentTdcpMeas)
{
  uint8_t u_i = 0;
  if (TRUE == gz_isOrtDcpMemoryAllocSucc)
  {
    for (u_i = DCP_AUXI_ANT_MAX_BLOCK - 1; u_i >= 1; --u_i)
    {
      memcpy(gpz_auxiAntDcpMeasBlock[u_i], gpz_auxiAntDcpMeasBlock[u_i - 1], sizeof(gnss_TdcpMeasBlock_t));
    }
    *gpz_auxiAntDcpMeasBlock[0] = *pz_auxiCurentTdcpMeas;
  }
  return;
}

/**
*@brief push main antenna measure
* @param[in]  pz_mainCurentTdcpMeas is TDCP measuremt for main antenna of current epoch
* @return         void
*/
void DCP_PushRcvMainMeasBlk_NHz(const gnss_TdcpMeasBlock_t* pz_mainCurentTdcpMeas)
{
  uint8_t u_i = 0;
  if (TRUE == gz_isOrtDcpMemoryAllocSucc)
  {
    for (u_i = DCP_AUXI_ANT_MAX_BLOCK - 1; u_i >= 1; --u_i)
    {
      memcpy(gpz_mainAntDcpMeasBlock[u_i], gpz_mainAntDcpMeasBlock[u_i - 1], sizeof(gnss_TdcpMeasBlock_t));
    }
    *gpz_mainAntDcpMeasBlock[0] = *pz_mainCurentTdcpMeas;
  }
  return;
}
/**
 * @brief get the index of auxiliary antenna measure block according to measure time
 * @param[in]  pz_obsTime is observation time
 * @return     the index of auxiliary antenna measure block, -1 represent matched failure
 */
int16_t DCP_getAuxiliaryAntMeasIndexAccordTime(const GpsTime_t* pz_obsTime)
{
  int16_t w_index = -1;
  int16_t w_i = 0;
  double d_timeDiff = -1.0;
  double d_minTimeDiff = 0.5;
  for (w_i = 0; w_i < DCP_AUXI_ANT_MAX_BLOCK; ++w_i)
  {
    d_timeDiff = tm_GpsTimeDiff(pz_obsTime, &(gpz_auxiAntDcpMeasBlock[w_i]->z_obsTime));
    if (fabs(d_timeDiff) < d_minTimeDiff)
    {
      w_index = w_i;
      d_minTimeDiff = fabs(d_timeDiff);
    }
  }
  if (d_minTimeDiff > ORT_MAIN_AUXI_TIME_DIFF_THRES)
  {
    w_index = -1;
  }
  return w_index;
}

/**
 * @brief get the index of main antenna measure block according to measure time
 * @param[in]  pz_obsTime is observation time
 * @return     the index of main antenna measure block, -1 represent matched failure
 */
int16_t DCP_getMainAntMeasIndexAccordTime(const GpsTime_t* pz_obsTime)
{
  int16_t w_index = -1;
  int16_t w_i = 0;
  double d_timeDiff = -1.0;
  double d_minTimeDiff = 0.5;
  for (w_i = 0; w_i < DCP_MAIN_ANT_MAX_BLOCK; ++w_i)
  {
    d_timeDiff = tm_GpsTimeDiff(pz_obsTime, &(gpz_mainAntDcpMeasBlock[w_i]->z_obsTime));
    if (fabs(d_timeDiff) < d_minTimeDiff)
    {
      w_index = w_i;
      d_minTimeDiff = fabs(d_timeDiff);
    }
  }
  if (d_minTimeDiff > ORT_MAIN_AUXI_TIME_DIFF_THRES)
  {
    w_index = -1;
  }
  return w_index;
}

/**
 * @brief get the position variance
 * @param[in]  pz_curMainDcpMeas main antenna TDCP measure block of current epoch
 * @param[in]  pz_curAuxiDcpMeas auxi antenna TDCP measure block of current epoch
 * @param[out] pd_posSigma is the sigma of position x,y,z
 * @return     void
 */
void DCPgetPosSigma(const gnss_TdcpMeasBlock_t* pz_curMainDcpMeas, const gnss_TdcpMeasBlock_t* pz_curAuxiDcpMeas, double pd_posSigma[3])
{
  uint8_t u_i = 0;
  double pd_mainPosXyzUnc[3] = { 0.0 };
  double pd_auxiPosXyzUnc[3] = { 0.0 };
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pd_mainPosXyzUnc[u_i] = (double)(pz_curMainDcpMeas->z_posSol.f_posXyzUnc[u_i]);
    pd_auxiPosXyzUnc[u_i] = (double)(pz_curAuxiDcpMeas->z_posSol.f_posXyzUnc[u_i]);
  }
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pd_posSigma[u_i] = sqrt(pd_mainPosXyzUnc[u_i] * pd_mainPosXyzUnc[u_i] + pd_auxiPosXyzUnc[u_i] * pd_auxiPosXyzUnc[u_i]);
  }
  return;
}
/**
 * @brief fill up the field of u_DcpPosType and u_fixFlag
 * @param[out] pz_curMainDcpMeas TDCP measure block of current epoch
 * @param[out] pz_ortResult is the result of heading .etc
 * @return void
 */
void DCP_fillUpDcpTypeAndFixType(const gnss_TdcpMeasBlock_t* pz_curMainDcpMeas, const gnss_TdcpMeasBlock_t* pz_curAuxiDcpMeas, gnss_OrientFix_t* pz_ortResult)
{
  uint8_t u_mainFixFlag = pz_curMainDcpMeas->z_posSol.u_fixFlag;
  uint8_t u_auxiFixFlag = pz_curAuxiDcpMeas->z_posSol.u_fixFlag;
  uint8_t u_fixFlag = 0;
  if (u_mainFixFlag == u_auxiFixFlag)
  {
    u_fixFlag = u_mainFixFlag;
  }
  else if (GNSS_FIX_FLAG_FIXED == u_mainFixFlag)
  {
    u_fixFlag = u_auxiFixFlag;
  }
  else if(GNSS_FIX_FLAG_FIXED == u_auxiFixFlag)
  {
    u_fixFlag = u_mainFixFlag;
  }
  else
  {
    if (u_mainFixFlag < u_auxiFixFlag)
    {
      u_fixFlag = u_mainFixFlag;
    }
    else
    {
      u_fixFlag = u_auxiFixFlag;
    }
  }
  pz_ortResult->u_DcpPosType = pz_curMainDcpMeas->z_posSol.u_DcpPosType;
  if ((pz_curMainDcpMeas->z_posSol.u_DcpPosType) > 0 && (pz_curAuxiDcpMeas->z_posSol.u_DcpPosType) > 0)
  {
    if ((pz_curAuxiDcpMeas->z_posSol.u_DcpPosType) > (pz_ortResult->u_DcpPosType))
    {
      pz_ortResult->u_DcpPosType = (pz_curAuxiDcpMeas->z_posSol.u_DcpPosType);
    }
  }
  else
  {
    pz_ortResult->u_DcpPosType = GNSS_DCP_POS_INVALID;
    u_fixFlag = GNSS_FIX_FLAG_INVALID;
  }
  pz_ortResult->u_ortFixFlag = u_fixFlag;
  return;
}
/**
 * @brief calculate the current orient using the previous epoch and current observations by TDCP or TDPR
 * @param[out] z_obsTime time of current epoch
 * @param[out] pz_ortResult is the result of heading .etc
 * @return 0 represent success and other failure
 */
ort_dcpSolStatus DCP_SolveRealtimeOrt(GpsTime_t z_obsTime, gnss_OrientFix_t* pz_ortResult)
{
  int16_t w_auxiAntIndex = -1;
  int16_t w_mainAntIndex = -1;
  uint8_t u_mainAntStatus = 0;
  uint8_t u_auxiAntStatus = 0;
  uint8_t u_i = 0;
  double pd_posSigma[3] = { 0.0 };
  gnss_TdcpMeasBlock_t* pz_currMainAntDcpMeas = NULL;
  memset(pz_ortResult, 0, sizeof(gnss_OrientFix_t));
  pz_ortResult->u_version = VERSION_GNSS_ORT_FIX;
  pz_ortResult->w_size = sizeof(gnss_OrientFix_t);
  pz_ortResult->z_gpsTime = z_obsTime;
  tm_cvt_GpstToEpoch(&z_obsTime, &(pz_ortResult->z_epoch));
  pz_ortResult->u_leapsec = gz_ortResult.u_leapsec;
  pz_ortResult->u_fixSource = FIX_SOURCE_DCP;
  if (FALSE == gz_isOrtDcpMemoryAllocSucc)
  {
    return ORT_DCP_MEM_FAIL;
  }
  if (fabs(tm_GpsTimeDiff(&z_obsTime, &gz_lastCalTime)) < 1.0e-3)
  {
    return ORT_DCP_TIME_REPEAT;
  }
  w_mainAntIndex = DCP_getMainAntMeasIndexAccordTime(&z_obsTime);
  if (w_mainAntIndex < 0)
  {
    return ORT_DCP_GET_MAIN_ANT_OBS_FAIL;
  }
  w_auxiAntIndex = DCP_getAuxiliaryAntMeasIndexAccordTime(&z_obsTime);
  if (w_auxiAntIndex < 0)
  {
    return ORT_DCP_GET_AUXI_ANT_OBS_FAIL;
  }
  pz_currMainAntDcpMeas = (gnss_TdcpMeasBlock_t*)OS_MALLOC(sizeof(gnss_TdcpMeasBlock_t));
  memcpy(pz_currMainAntDcpMeas, gpz_mainAntDcpMeasBlock[w_mainAntIndex], sizeof(gnss_TdcpMeasBlock_t));
  pz_ortResult->f_age = (float)(tm_GpsTimeDiff(&(pz_currMainAntDcpMeas->z_obsTime), &(gpz_auxiAntDcpMeasBlock[w_auxiAntIndex]->z_obsTime)));
  u_mainAntStatus = dcp_SolveRealtimePos(pz_currMainAntDcpMeas, &gz_dcpOrtMainSolInfo);
  u_auxiAntStatus = dcp_SolveRealtimePos(gpz_auxiAntDcpMeasBlock[w_auxiAntIndex], &gz_dcpOrtAuxiSolInfo);
  if (1 == u_mainAntStatus && 1 == u_auxiAntStatus)
  {
    pz_ortResult->u_SvTrackCount = gz_ortResult.u_SvTrackCount;
    pz_ortResult->u_SvTrackCountEleThres = gz_ortResult.u_SvTrackCountEleThres;
    pz_ortResult->u_SvInUseCount = gz_ortResult.u_SvInUseCount;
    pz_ortResult->u_SvInUseCountMuliFreq = gz_ortResult.u_SvInUseCountMuliFreq;
    pz_ortResult->u_ortSolStatus = gz_ortResult.u_ortSolStatus;
    pz_ortResult->u_ortPosVelType = gz_ortResult.u_ortPosVelType;
    pz_ortResult->u_GPSsignalUsedMask = gz_ortResult.u_GPSsignalUsedMask;
    pz_ortResult->u_GLOsignalUsedMask = gz_ortResult.u_GLOsignalUsedMask;
    pz_ortResult->u_GalSignalUsedMask = gz_ortResult.u_GalSignalUsedMask;
    pz_ortResult->u_BDSsignalUsedMask = gz_ortResult.u_BDSsignalUsedMask;
    DCPgetPosSigma(pz_currMainAntDcpMeas, gpz_auxiAntDcpMeasBlock[w_auxiAntIndex], pd_posSigma);
    gnss_getOrtHeadingPitch(pz_currMainAntDcpMeas->z_posSol.d_xyz, gpz_auxiAntDcpMeasBlock[w_auxiAntIndex]->z_posSol.d_xyz, pd_posSigma, &(pz_ortResult->z_ortResult));
    DCP_fillUpDcpTypeAndFixType(pz_currMainAntDcpMeas, gpz_auxiAntDcpMeasBlock[w_auxiAntIndex], pz_ortResult);
    if (GNSS_FIX_FLAG_INVALID != (pz_ortResult->u_ortFixFlag))
    {
      pz_ortResult->u_ortSolStatus = C_GNSS_ORT_SOL_STATUS_SOL_COMPUTED;
    }
    pz_ortResult->u_ortPosVelType = gnss_cvt_ortFixFlag2PosVelType(pz_ortResult->u_ortFixFlag);

    pz_ortResult->z_mainAntInfo.u_SvInUseCount = gpz_mainAntDcpMeasBlock[w_mainAntIndex]->z_posSol.z_SvStatus.u_SvInUseCount;
    pz_ortResult->z_mainAntInfo.u_posFixFlag = gpz_mainAntDcpMeasBlock[w_mainAntIndex]->z_posSol.u_fixFlag;
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pz_ortResult->z_mainAntInfo.d_xyz[u_i] = gpz_mainAntDcpMeasBlock[w_mainAntIndex]->z_posSol.d_xyz[u_i];
      pz_ortResult->z_mainAntInfo.d_lla[u_i] = gpz_mainAntDcpMeasBlock[w_mainAntIndex]->z_posSol.d_lla[u_i];
      pz_ortResult->z_mainAntInfo.f_velXyz[u_i] = gpz_mainAntDcpMeasBlock[w_mainAntIndex]->z_posSol.f_velXyz[u_i];
      pz_ortResult->z_mainAntInfo.f_velEnu[u_i] = gpz_mainAntDcpMeasBlock[w_mainAntIndex]->z_posSol.f_velEnu[u_i];
      pz_ortResult->z_ortResult.f_velXyz[u_i] = gpz_auxiAntDcpMeasBlock[w_auxiAntIndex]->z_posSol.f_velXyz[u_i];
      pz_ortResult->z_ortResult.f_velEnu[u_i] = gpz_auxiAntDcpMeasBlock[w_auxiAntIndex]->z_posSol.f_velEnu[u_i];
      pz_ortResult->z_mainAntInfo.pd_refStationCoordinate[u_i] = gpz_mainAntDcpMeasBlock[w_mainAntIndex]->z_posSol.z_rtk.pd_StationCoordinate[u_i];
    }
    if (GNSS_FIX_FLAG_FIXED == (pz_ortResult->u_ortFixFlag))
    {
      if (pz_ortResult->z_mainAntInfo.u_SvInUseCount > 20 && pz_ortResult->u_SvInUseCount > 20
        && (pz_ortResult->z_ortResult.f_headingStd) < 3.0f && (pz_ortResult->z_ortResult.f_pitchStd) < 3.0f)
      {
        pz_ortResult->z_ortResult.f_headingStd = M_MIN(0.9f, pz_ortResult->z_ortResult.f_headingStd);
        pz_ortResult->z_ortResult.f_pitchStd = M_MIN(0.9f, pz_ortResult->z_ortResult.f_pitchStd);
      }
    }
  }
  gz_lastCalTime = z_obsTime;
  OS_FREE(pz_currMainAntDcpMeas);
  return ORT_DCP_SUCC;
}