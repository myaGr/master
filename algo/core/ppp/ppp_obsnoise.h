/**@file        ppp_quality.h
 * @brief       quality of observation checking
 * @details
 * @author      liuguo
 * @date        2023/03/21
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/03/21  <td>0.1      <td>liuguo      <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __PPP_QUALITY_H__
#define __PPP_QUALITY_H__

#include "ppp_type.h"

BEGIN_DECL

typedef struct
{
  GpsTime_t z_time;
  float     f_cmc;
  float     f_meancmc;
}ppp_freqCmc_t;
typedef struct
{
  ppp_freqCmc_t* pz_freqCmc[MAX_GNSS_SIGNAL_FREQ];
}ppp_satCmc_t;

typedef struct
{
  ppp_satCmc_t* pz_satCmcBlock[ALL_GNSS_SYS_SV_NUMBER];
}ppp_obsCmcBlock_t;

/**
 * @brief computation of pseudorange sigma
 * @param[in] pz_satSigMeasCollect is the gnss observation
 * @param[in] pz_obsCmcBlock is the previous cmc
 * @param[in] pz_PPPfilterInfo the filter for the PPP-RTK algorithm
 * @param[out] pz_FilterObs is the sigma of observation
 * @return 0 represent successful and other represent failure
 */
uint8_t ppp_pseudoSigmaCompute(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, const ppp_filterInfo_t* pz_PPPfilterInfo, ppp_obsCmcBlock_t* pz_obsCmcBlock, ppp_EpochFilterObs_t* pz_FilterObs);

END_DECL

#endif
