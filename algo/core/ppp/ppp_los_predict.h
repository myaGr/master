/**@file        ppp_lospredict.h
 * @brief       LOS prediction  when ssr is interrupted
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/11/08  <td>0.1      <td>yuezhu      <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#ifndef _PPP_LOS_PREDICT_H_
#define _PPP_LOS_PREDICT_H_
#include "cmn_def.h"
#include "cmn_utils.h"
#include "gnss_type.h"
#include "ppp_type.h"

BEGIN_DECL

#define SSR_PREDICT_SAT_NUM_LIMIT   (35)
#define DATA_MAX_HIS_TIME           (120)   /*unit:s*/
#define SOL_INTERVAL_SEC            (5)
#define DATA_MAX_HIS_COUNT          (DATA_MAX_HIS_TIME/((uint8_t)SOL_INTERVAL_SEC)) 
 
#define GNSS_LOS_PREDICT_FLAG_FAIL  (0x0)
#define GNSS_LOS_PREDICT_FLAG_GM11  (0x1)
#define GNSS_LOS_PREDICT_FLAG_POLY  (0x2)
typedef uint8_t gnss_losPredictFlag;

typedef struct
{
  GpsTime_t              z_orbtime;                           /* orbclk time */
  int32_t                q_iode;                              /* ephemeris iode */
  int32_t                q_los;                               /* los correction of history */
  int32_t                q_diffthreshold;
}ppp_satHisSSR_t;
typedef struct
{
  gnss_losPredictFlag     u_losPredictFlag;
  int32_t                 q_iode;                              /* ephemeris iode */
  int32_t                 q_los;                               /* los correction of predicted */ 
}ppp_satPredictSSR_t;
typedef struct
{
  GpsTime_t               z_tor;                                /* time of observation */
  ppp_satHisSSR_t*        pz_satHisSSR[SSR_PREDICT_SAT_NUM_LIMIT]; /* correction of satellites */
}ppp_epochHisSSR_t;
typedef struct
{
  ppp_satPredictSSR_t*    pz_satPredictSSR[SSR_PREDICT_SAT_NUM_LIMIT];  /* correction of satellites */
}ppp_epochPredictSSR_t;

typedef struct
{
  int32_t                q_basecorr[C_GNSS_MAX];                   /* correction of the base adjustment*/
  uint16_t               w_nhis;                                   /* the number epoch of history ssr correction */
  uint8_t                u_satidxlist[SSR_PREDICT_SAT_NUM_LIMIT];     /* satellite index of the system */
  ppp_epochHisSSR_t      pz_epochHisSSR[DATA_MAX_HIS_COUNT];       /* some epoch history ssr correction */
  GpsTime_t              z_predictTime;                            /* time of Predict*/
  uint8_t                u_isNeedPredict[SSR_PREDICT_SAT_NUM_LIMIT / 8 + 1];/* flag of do predict or not, 0: do not, 1:do predict */
}ppp_LosPredictBlock_t;

/**
 * @brief the interface of LOS predict algorithm of SSR
 * @param[in]      pz_satSigMeasCollect is the observation information
 * @param[in/out]      pz_SsrLocBlk is the SSR product
 * @param[out]     pz_LosPredictInfo is LOS Predict infomation of SSR
 * @return         0 represent successs and other failed
 */
int32_t ppp_LosPredictSSR(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, gnss_ssrLosBlock_t* pz_SsrLocBlk, 
  const GpsTime_t* z_tor , ppp_LosPredictBlock_t* pz_LosPredictBlk);

END_DECL
#endif