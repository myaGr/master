/**********************************************************************************
* @note
* @par History :
*<table>
* <tr><th>Date        <th>Version  <th>Author      <th>Description
* <tr><td>2022/10/08  <td> 0.1     <td>chenjinhe   <td>Init version
* < / table>
*
**********************************************************************************
*/
#ifndef __CMN_TURBO_EDIT_H__
#define __CMN_TURBO_EDIT_H__
#include "cmn_def.h"
#include "gnss_type.h"
BEGIN_DECL

/** Enum for all the supported GNSS single frequency measurement types */
typedef enum
{
  C_TE_L2L5 =0,
  C_TE_L1L2,
  C_TE_L1L5,
  C_TE_COMBINE_MAX
}turboEdit_combineTypeEnumVal;
/*the filter information used by TurboEdit method, the combination observations include GF and MW*/
typedef struct
{
  uint8_t   u_isFirstEpoch;/*the mask of whether is first epoch*/
  uint8_t   u_predictFlag;/*the flag of whether doing kalman predict step*/
  float     f_eleA;/*the elevation of satellite*/
  double    d_value;/*the value of GF or MW combination observations*/
  double    d_velocity;/*the velocity of GF or MW combination observations*/
  double    d_valueD;/*the covariance of GF or MW combination observations*/
  double    d_valueQ;/*the input of GF or MW combination observations used to decide the threshold of cycle slip*/
  GpsTime_t z_beginTime;/*the begin time of GF or MW combination observations*/
  GpsTime_t z_endTime;/*the end time of GF or MW combination observations*/
}cmn_oneObsTurboEditFilterInfo;

typedef struct
{
  cmn_oneObsTurboEditFilterInfo* pz_turboEditSet[C_TE_COMBINE_MAX * ALL_GNSS_SYS_SV_NUMBER];/*the filter information used by TurboEdit method, the combination observations include GF and MW*/
}cmn_turboEditFilterSet;

/**
 * @brief the inteface of initilizing the filter information used by GF or MW method
 * @param[out]  pz_TEfilterSet is the filter information used by GF or MW method
 * @return     void
 */
void cmn_initTEfilterSet(cmn_turboEditFilterSet* pz_TEfilterSet);

/**
 * @brief the inteface of deinitilizing the filter information used by GF or MW method
 * @param[out]  pz_TEfilterSet is the filter information used by GF or MW method
 * @return     void
 */
void cmn_deinitTEfilterSet(cmn_turboEditFilterSet* pz_TEfilterSet);

/**
 * @brief initilize the turbo edit filter
 * @param[out]  pz_turboEditFilterInfo is the filter information used by turbo edit method
 * @return      void
 */
void cmn_initTurboEditFilterInfo(cmn_oneObsTurboEditFilterInfo* pz_turboEditFilterInfo);

/**
 * @brief get the index of turbo edit filter
 * @param[in]  w_iSat is satellite id
 * @param[in]  z_freqType is the first frequency type
 * @param[in]  z_twinFreqType is the second frequency type
 * @return     the index of turbo edit filter
 */
uint16_t cmn_getKeyValue(uint16_t w_iSat, gnss_FreqType z_freqType, gnss_FreqType z_twinFreqType);

/**
 * @brief filter the GF value and detect the cycle slip
 * @param[in]   pz_tor is observation time
 * @param[in]   d_gfValue is GF combination value
 * @param[in]   f_ele is satellite elevation
 * @param[out]  pz_gfFilterInfo is the filter information used by GF method
 * @return      0 represent non-detect, 1 represent non cycle slip, 2 represent cycle slip has happened
 */
uint8_t cmn_gfFilterDetect(const GpsTime_t* pz_tor, double d_gfValue, float f_ele, 
  cmn_oneObsTurboEditFilterInfo* pz_gfFilterInfo);

/**
 * @brief the inteface of using GF method to detect cycle slip
 * @param[out]  pz_satSignalCur is the observation information
 * @param[out]  pz_gfFilterSet is the filter information used by GF method
 * @return     1 represent success and 0 failure
 */
uint8_t cmn_gfDetectCycleSlip(gnss_SatSigMeasCollect_t* pz_satSignalCur, cmn_turboEditFilterSet* pz_gfFilterSet);

/**
 * @brief the inteface of using MW method to detect cycle slip
 * @param[out]  pz_satSignalCur is the observation information
 * @param[out]  pz_mwFilterSet is the filter information used by MW method
 * @return     1 represent success and 0 failure
 */
uint8_t cmn_mwDetectCycleSlip(gnss_SatSigMeasCollect_t* pz_satSignalCur, cmn_turboEditFilterSet* pz_mwFilterSet);
END_DECL
#endif