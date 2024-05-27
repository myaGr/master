/************************************************************
* Copyrights(C)
* All rights Reserved
* 文件名称：gnss_qos.c
* 文件描述：观测量的质量监控以及权重的管理
* 版本号：1.0.0
* 作者：
* 日期：09/10
************************************************************/
#include "gnss.h"
#include "gnss_qos.h"
#include "gnss_sys_api.h"
#include "gnss_sd.h"
#include "gnss_sd_dec.h"
#include "gnss_config.h"
#include "gnss_pe.h"
#include "gnss_mode.h"
#include "gnss_math.h"
#include "gnss_pe.h"
#include "gnss_mode.h"

#undef MODULE_TAG
#define MODULE_TAG OBJ_QOS

extern Gnss_Cfg_t g_pe_cfg;
extern Sd_t Sd_data;
extern uint8_t firstFix;
extern PE_MODES peMode;
extern PeStateMonitor_t peState;
//extern AGFusionResults     DRFusionRslt;
#if 0
extern Rtcm_data_t g_rtcm_data;
#endif

void gnss_Qos_avgCno(meas_blk_t* pMeas)
{
  uint8_t            cnt = 0;
  uint32_t           i;
  gnss_meas_t* pSvMeas;

  pMeas->avgCno = 0;

  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &(pMeas->meas[i]);

    if (pSvMeas->prn == 0) continue;

    pMeas->avgCno += pSvMeas->cno;
    cnt++;
  }
  /* calculate average cno */
  if (cnt > 0)
  {
    pMeas->avgCno /= cnt;
  }
}
void gnss_LowAvgCno_Detection(meas_blk_t* pMeas, PE_MODES* pMode)
{
  USER_SCENARIO* p;
  p = &(pMode->userSceData);
  /*Avg20Cnt avgcno < 20 dB-Hz cnt*/
  if (pMeas->avgCno < 20)
  {
    if (pMeas->avgCno <= 15)
    {
      p->LowAvgCnocnt += 2;
    }
    else
    {
      p->LowAvgCnocnt += 1;
    }
    if (p->LowAvgCnocnt > 5)
    {
      p->LowAvgCnocnt = 6;
    }
  }
  else if (pMeas->avgCno >= 30)
  {
    p->LowAvgCnocnt = 0;
  }
  else if (p->LowAvgCnocnt > 0)
  {
    p->LowAvgCnocnt -= 1;
  }
  else
  {
    p->LowAvgCnocnt = 0;
  }
}
/***********************************************************************
* 函数介绍: gnss_Qos_Cno_Mask
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：09/22/
***********************************************************************/
void gnss_Qos_Cno_Mask(meas_blk_t* pMeas)
{
  uint8_t            cno35Cnt = 0, cno25Cnt = 0, cno18Cnt = 0, cnt = 0;
  uint16_t           cno_mask, max_cno_mask;
  uint32_t           i;
  gnss_meas_t* pSvMeas;

  pMeas->maxCno = 0;
  pMeas->avgCno = 0;
  pMeas->maxCno_L5 = 0;
  pMeas->avgCno_L5 = 0;
  pMeas->cpNoLLI = 0;
  max_cno_mask = 30;

  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &(pMeas->meas[i]);

    if (pSvMeas->prn == 0) continue;

    if (pSvMeas->cno >= 25)
    {
      cno25Cnt++;
    }
    if (pSvMeas->cno <= 18)
    {
      cno18Cnt++;
    }

    if (pSvMeas->cycleSlipCount == 0)
    {
      pMeas->cpNoLLI++;
    }

    if (pSvMeas->freq_index > 0) continue;

    /* calculate the maximum cno and average cno */
    if (pSvMeas->cno > pMeas->maxCno)
    {
      pMeas->maxCno = pSvMeas->cno;
    }
    pMeas->avgCno += pSvMeas->cno;
    cnt++;
  }
  /* calculate average cno */
  if (cnt > 0)
  {
    pMeas->avgCno /= cnt;
  }

  cno_mask = g_pe_cfg.cno_mask;

  /*Caculate L5 info*/
  for (cnt = 0, i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &(pMeas->meas[i]);

    if (pSvMeas->prn == 0) continue;
    if (pSvMeas->freq_index <= 0) continue;

    /* calculate the maximum cno and average cno */
    if (pSvMeas->cno > pMeas->maxCno_L5)
    {
      pMeas->maxCno_L5 = pSvMeas->cno;
    }
    pMeas->avgCno_L5 += pSvMeas->cno;
    cnt++;
  }
  /* calculate average cno */
  if (cnt > 0)
  {
    pMeas->avgCno_L5 /= cnt;
  }
  GLOGI("Avg Cn0: %02d %02d", pMeas->avgCno, pMeas->avgCno_L5);
  /*
  1. If cno20 >=8, use 18dB-Hz as cno mask
  2. If average cno < 18, use 10dB-Hz as cno mask
  3. If autoMobile mode:
  (1) average cno >= 35 and more than 10 measures, 30dB-Hz as threshold
  (2) else use 18dB-Hz as threshold
  (3) average cno < 35, use 15dB-Hz as threshold
  */
  for (i = 0; i < GNSS_MAX_MODE; i++)
  {
    cno35Cnt += pMeas->Cno35Cnt[i];
  }

  if (pMeas->Cno20Cnt >= 8 && pMeas->measCnt >= 10 && pMeas->avgCno >= 25)
  {
    cno_mask = 18;
  }
  if (pMeas->avgCno < 18)
  {
    cno_mask = 10;
  }
  if (g_pe_cfg.automobile)
  {
    if (pMeas->avgCno >= 35)
    {
      if (pMeas->measCnt >= 10)
      {
        if (cno35Cnt >= 8 && !firstFix)
        {
          cno_mask = 30;
        }
        else
        {
          if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100)
          {
            cno_mask = (uint16_t)(pMeas->avgCno * 0.8);
            if (cno_mask > max_cno_mask)
            {
              cno_mask = max_cno_mask;
            }
          }
          else
          {
            cno_mask = 20;
          }
        }
      }
      else
      {
        cno_mask = 18;
      }
    }
    else if (pMeas->avgCno >= 30)
    {
      cno_mask = 15;
    }
    else
    {
      cno_mask = 10;
      if (pMeas->measCnt > 22 && pMeas->avgCno < 24 && cno25Cnt > 6 && cno18Cnt > 6
        && (peMode.staticData.historyStatic & 0xF) == 0x0)
      {
        cno_mask = 21;
      }
    }
  }
  for (i = 0; i < pMeas->measCnt; i++)
  {
    // Normal case cno mask
    if (pMeas->meas[i].cno < cno_mask)
    {
      pMeas->meas[i].status &= 0xF8;
    }

    // Indoor Mode cno mask
  }
  /*LowAvgCno Detection*/
  gnss_LowAvgCno_Detection(pMeas, &peMode);
}

/***********************************************************************
* 函数介绍: gnss_Qos_Pr_Chck
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：09/22/
***********************************************************************/
#if 0
void gnss_Qos_Pr_Chck(meas_blk_t* pMeas)
{
  uint32_t            i, cnt = 0, mode;
  uint32_t            cnt_a, cnt_b, bad_num;
  double            prMax = 0, prMin = 1.0e20;
  uint32_t            list_a[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], list_b[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM];
  uint32_t* bad_list;

  for (mode = 0; mode < GNSS_MAX_MODE; mode++)
  {
    prMax = 0;
    prMin = 1.0e20;
    cnt = 0;
    cnt_a = 0;
    cnt_b = 0;

    // Find the max and min
    for (i = 0; i < pMeas->measCnt; i++)
    {
      if (pMeas->meas[i].prn == 0) continue;
      if ((pMeas->meas[i].status & 0x1) == 0) continue;

      if (pMeas->meas[i].gnssMode == mode &&
        pMeas->meas[i].sat_type == MEO_SAT)
      {
        if (pMeas->meas[i].pseudoRange > prMax)
        {
          prMax = pMeas->meas[i].pseudoRange;
        }
        if (pMeas->meas[i].pseudoRange < prMin)
        {
          prMin = pMeas->meas[i].pseudoRange;
        }
        cnt++;
      }
    }
    if (cnt < 3)
    {
      continue;
    }
    /* Maximum time transit difference between all satellites are smaller than 30ms */
    if (fabs(prMax - prMin) < 30 * LIGHT_MSEC)
    {
      continue;
    }
    for (i = 0; i < pMeas->measCnt; i++)
    {
      if ((pMeas->meas[i].status & 0x1) == 0)
      {
        continue;
      }
      if (pMeas->meas[i].gnssMode == mode &&
        pMeas->meas[i].sat_type == MEO_SAT)
      {
        if ((prMax - pMeas->meas[i].pseudoRange) < (prMax - prMin) / 2)
        {
          list_a[cnt_a++] = i;
        }
        else
        {
          list_b[cnt_b++] = i;
        }
      }
    }
    // bad sv list and sv number 
    if (cnt_a > cnt_b)
    {
      bad_list = list_b;
      bad_num = cnt_b;
    }
    else
    {
      bad_list = list_a;
      bad_num = cnt_a;
    }
    // reject PR and DR
    for (i = 0; i < bad_num; i++)
    {
      pMeas->meas[bad_list[i]].status &= 0xF8;
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "Rej gnssMode(%d),sv(%d) (%s)", pMeas->meas[bad_list[i]].gnssMode, pMeas->meas[bad_list[i]].prn, __FUNCTION__);
    }
  }
}
#endif
/***********************************************************************
* 函数介绍: gnss_Qos_Dynamic_Elevation
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：10/01/
***********************************************************************/
void gnss_Qos_Dynamic_Elevation(meas_blk_t* pMeas)
{
  //uint8_t             status_save;
  uint8_t             elevationCnt = 0;
  uint32_t            i;
  double            dynamicElevationThreshold = 7;
  double            ave_elevation = 0.0;
  double            elevationSum = 0.0;
  sat_data_t* sp;
  //DOP_TYPE       dop1, dop2;


  if (peMode.userSceData.isUnderEleRoad)
  {
    return;
  }

  if (peMode.userSceData.isPDRMode)
  {
    return;
  }

  if (peState.posHasBias)
  {
    return;
  }

  for (i = 0; i < pMeas->measCnt; i++)
  {
    if (pMeas->meas[i].prn == 0) continue;
    // by-pass rejected sv
    if ((pMeas->meas[i].status & 0x3) == 0) continue;


    // by-pass no elevation sv
    sp = gnss_sd_get_sv_data(pMeas->meas[i].gnssMode, pMeas->meas[i].prn, pMeas->meas[i].freq_index);
    if (sp == NULL || sp->d_cos.elev <= 0)
    {
      continue;
    }

    // elevation sum
    elevationSum += sp->d_cos.elev * RAD2DEG;
    elevationCnt++;
  }

  // there is no satellites with positive elevation angle
  if (elevationCnt < 6)
  {
    return;
  }

  ave_elevation = elevationSum / (double)elevationCnt;

  if (ave_elevation < 30.0)
  {
    dynamicElevationThreshold = dynamicElevationThreshold + 1;
  }
  else if (ave_elevation >= 30.0 && ave_elevation < 40.0)
  {
    dynamicElevationThreshold = dynamicElevationThreshold + 3;
  }
  else if (ave_elevation >= 40.0 && ave_elevation < 50.0)
  {
    dynamicElevationThreshold = dynamicElevationThreshold + 5;
  }
  else/* if (ave_elevation >= 50.0)*/
  {
    dynamicElevationThreshold = dynamicElevationThreshold + 6;
  }

  // Elevation check
  for (i = 0; i < pMeas->measCnt; i++)
  {
    if (pMeas->meas[i].prn == 0) continue;
    if ((pMeas->meas[i].status & 0x3) == 0) continue;
    sp = gnss_sd_get_sv_data(pMeas->meas[i].gnssMode, pMeas->meas[i].prn, pMeas->meas[i].freq_index);
    // by-pass those satellites with positive elevation
    if (sp == NULL || sp->d_cos.elev <= 0)
    {
      continue;
    }

    // check DOP before and after reject one satellite
    /* waste calculation resource
    gnss_Pe_Dop(pMeas,&dop1,1);
    status_save = pMeas->meas[i].status;
    pMeas->meas[i].status &= 0xFC;
    gnss_Pe_Dop(pMeas,&dop2,1);
    pMeas->meas[i].status = status_save;
    if (dop1.pDOP == 0.0 || dop2.pDOP == 0.0) continue;
    if ((dop2.hDOP / dop1.hDOP) >= 2.0) continue;
    */

    // If lower than elevation threshold and cno is lower than 35, mark this satellite
    if (sp->d_cos.elev <= dynamicElevationThreshold * DEG2RAD)
    {
      if (pMeas->meas[i].cno < 35)
      {
        pMeas->meas[i].prWeightChck |= ELEV_CHECK;
        if (fabs(pMeas->meas[i].dr_diff) > 2.0)
        {
          pMeas->meas[i].drWeightChck |= ELEV_CHECK;
        }
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "%02d %03d de-weight because low ele", pMeas->meas[i].gnssMode,
          pMeas->meas[i].prn);
      }
    }
  }
}

/***********************************************************************
* 函数介绍: gnss_Qos_PR_Jitter
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：09/22/
***********************************************************************/
#ifdef USED_IN_MC262M
void gnss_Qos_CodeStd_Cal(meas_blk_t* pMeas, double* codeThres, double* carrThres)
{
  uint32_t              i, prCnt = 0, drCnt = 0;
  double              PRAvg = 0.0, PRstd = 0.0;
  double              DRAvg = 0.0, DRstd = 0.0;

  double codeStd[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 };
  double carrStd[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 };

  gnss_meas_t* pSvMeas;

  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &(pMeas->meas[i]);
    if (pSvMeas->prn == 0) {
      continue;
    }
    if (pSvMeas->status & 0x1 && pSvMeas->codeDetStd < 1000)
    {
      codeStd[prCnt++] = pSvMeas->codeDetStd;
      PRAvg += pSvMeas->codeDetStd;
    }
    if (pSvMeas->status & 0x2)
    {
      carrStd[drCnt++] = pSvMeas->dopplerStd;
      DRAvg += pSvMeas->dopplerStd;
    }
  }

  if (prCnt > 0)
  {
    PRAvg /= prCnt;
    for (i = 0; i < prCnt; i++)
    {
      PRstd += (codeStd[i] - PRAvg) * (codeStd[i] - PRAvg);
    }

    if (PRstd > 0.0)
    {
      PRstd = sqrt(PRstd / prCnt);
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s PR:%d,%10.4f,%10.4f", __FUNCTION__,
        peMode.userSceData.isUnderEleRoad, PRAvg, PRstd);
    }

  }
  if (drCnt > 0)
  {
    DRAvg /= drCnt;
    for (i = 0; i < drCnt; i++)
    {
      DRstd += (carrStd[i] - DRAvg) * (carrStd[i] - DRAvg);
    }

    if (DRstd > 0.0)
    {
      DRstd = sqrt(DRstd / drCnt);
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s DR:%d,%10.4f,%10.4f", __FUNCTION__,
        peMode.userSceData.isUnderEleRoad, DRAvg, DRstd);
    }

  }
}
#endif
/***********************************************************************
* 函数介绍: gnss_Qos_MP
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：09/22/
***********************************************************************/
#ifdef USED_IN_MC262M
void gnss_Qos_MP(meas_blk_t* pMeas)
{
  uint32_t             i;
  gnss_meas_t* pSvMeas;
  double             codeStdThres;
  double             carrStdThres;

  //this function only working for Phone Device
  //phone(0) and mobile(1).
  if (g_pe_cfg.automobile)
  {
    codeStdThres = g_pe_cfg.codeStdAuto;
    carrStdThres = g_pe_cfg.carrStdAuto;
    if (firstFix == FALSE)
    {
      codeStdThres = 80.0;
    }
  }
  else
  {
    codeStdThres = g_pe_cfg.codeStdPhone;
    carrStdThres = g_pe_cfg.carrStdPhone;

    if (pMeas->avgCno < 15)
    {
      codeStdThres *= 4.0;
      carrStdThres *= 4.0;
    }
    else if (pMeas->avgCno < 20)
    {
      codeStdThres *= 2.0;
      carrStdThres *= 2.0;
    }
  }

  gnss_Qos_CodeStd_Cal(pMeas, &codeStdThres, &carrStdThres);

  //MP check
  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &pMeas->meas[i];
    if (pSvMeas->prn == 0) continue;
    // clear flags
    pSvMeas->prWeightChck = 0;
    pSvMeas->drWeightChck = 0;
    if (g_pe_cfg.chipType == SPRD)
    {
      switch (pSvMeas->mpIndicator)
      {
      case AGGNSS_MULTIPATH_INDICATOR_PRESENT:
      {
        pSvMeas->prWeightChck |= MP_CHECK;
        break;
      }
      default:break;
      }
    }
    else
    {

      if ((pSvMeas->codeDetStd >= codeStdThres) && (pSvMeas->status & 0x1))
      {
        pSvMeas->status &= 0xFE;
        pMeas->PRMPNum++;
#if defined(PLAYBACK_MODE)
        SYS_LOGGING(OBJ_PE, LOG_INFO, "MP Reject PR: %02d %03d", pSvMeas->gnssMode, pSvMeas->prn);
#endif
        /* For PDR mode, reject DR if PR was rejected */
        if (peMode.userSceData.isPDRMode)
        {
          pSvMeas->status &= 0xFD;
          pMeas->DRMPNum++;
#if defined(PLAYBACK_MODE)
          SYS_LOGGING(OBJ_PE, LOG_INFO, "MP Reject DR: %02d %03d", pSvMeas->gnssMode, pSvMeas->prn);
#endif
        }
      }
      if ((pSvMeas->dopplerStd >= carrStdThres) && (pSvMeas->status & 0x2))
      {
        pSvMeas->status &= 0xFD;
        pMeas->DRMPNum++;
#if defined(PLAYBACK_MODE)
        SYS_LOGGING(OBJ_PE, LOG_INFO, "MP Reject DR: %02d %03d", pSvMeas->gnssMode, pSvMeas->prn);
#endif
      }
    }
  }

  /* If In MP environment, then don't use RTD */
#if 0
  if (((double)pMeas->PRMPNum / (double)pMeas->measCnt) > 0.3 && !(g_rtcm_data.rtd.RtdDataFull))
  {
    pMeas->rtdUsed = FALSE;
    g_rtcm_data.rtd.RtdUseFlag = FALSE;
    g_rtcm_data.rtd.RtdEnterCnt = 0;
    SYS_LOGGING(OBJ_QOS, LOG_INFO, "Disable RTD in MP environment:%02d,%02d", pMeas->PRMPNum, pMeas->measCnt);
  }
#endif
}
#endif
/***********************************************************************
* 函数介绍: gnss_Qos_Chck_Count
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：10/08/
***********************************************************************/
void gnss_Qos_Chck_Count(meas_blk_t* pMeas)
{
  uint16_t             prWeightChck, drWeightChck;
  uint32_t             i, prNum = 0, drNum = 0, prDewNum = 0, drDewNum = 0;
  gnss_meas_t* pSvMeas;

  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &pMeas->meas[i];
    if (pSvMeas->prn == 0) continue;
    if ((pSvMeas->status & 0x3) == 0) continue;
    if (pSvMeas->status & 0x1) prNum++;
    if (pSvMeas->status & 0x2) drNum++;
    // calculate prChckCnt
    pSvMeas->prChckCnt = 0;
    prWeightChck = pSvMeas->prWeightChck;
    if (prWeightChck) prDewNum++;
    while (prWeightChck)
    {
      if (prWeightChck & 0x1)
      {
        pSvMeas->prChckCnt++;
      }
      prWeightChck >>= 1;
    }
    // reject PR
    if (pSvMeas->prChckCnt > CHECK_REJ_CNT)
    {
      pSvMeas->status &= 0xFE;
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s Rej PR:%02d,%03d", __FUNCTION__, pSvMeas->gnssMode, pSvMeas->prn);
    }

    // calculate drChckCnt
    pSvMeas->drChckCnt = 0;
    drWeightChck = pSvMeas->drWeightChck;
    if (drWeightChck) drDewNum++;
    while (drWeightChck)
    {
      if (drWeightChck & 0x1)
      {
        pSvMeas->drChckCnt++;
      }
      drWeightChck >>= 1;
    }
    // reject DR
    if (pSvMeas->drChckCnt > CHECK_REJ_CNT)
    {
      pSvMeas->status &= 0xFD;
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s Rej DR:%02d,%03d", __FUNCTION__, pSvMeas->gnssMode, pSvMeas->prn);
    }
  }
  SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s,%14.6f,%02d,%02d,%02d,%02d", __FUNCTION__, pMeas->tor, prNum, prDewNum, drNum, drDewNum);
}


/***********************************************************************
* 函数介绍: gnss_Qos_Outlier_Prepare_QCOM
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：3/13/
***********************************************************************/
void gnss_Qos_Outlier_Prepare_QCOM(meas_blk_t* pMeas, int8_t* idx0, int8_t* idx1)
{
  uint8_t                 i, j, goodQuality = FALSE;
  int8_t                 index[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], qualityFlag[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { FALSE }, candidateIndex = -1, firstCandidate = -1, firstIndex = -1;
  double                avgDiff[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, angleThres, firstAvgDiff = 0.0;
  gnss_meas_t* pSv0, * pSv1;
  sat_data_t* sp0, * sp1;
  double                delta;
  double                ratio1 = 1.0, ratio2 = 1.0;
  (*idx0) = -1;
  (*idx1) = -1;
  memset(index, -1, sizeof(int8_t) * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM);

  /* If position is biased, then user larger DIFF difference and smaller angle DIFF */
  if (peState.posHasBias)
  {
    ratio1 = 1.0;
    ratio2 = 0.5;
  }

  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSv0 = &(pMeas->meas[i]);

    // 1. bypass invalid PRN and invalid PR diff
    if (pSv0->prn == 0 || pSv0->prDiffValid == 0) continue;
    // 2. bypass invalid PR
    if ((pSv0->status & 0x1) == 0) continue;
    // 3. system mode check
    if (pMeas->rtdUsed && pSv0->gnssMode == GLN_MODE) continue;

    sp0 = gnss_sd_get_sv_data(pSv0->gnssMode, pSv0->prn, pSv0->freq_index);
    if (sp0 == NULL) continue;

    firstIndex = -1;
    firstAvgDiff = 0.0;
    qualityFlag[i] = FALSE;

    for (j = i + 1; j < pMeas->measCnt; j++)
    {
      pSv1 = &(pMeas->meas[j]);

      // 1. bypass invalid PRN and invalid PR diff
      if (pSv1->prn == 0 || pSv1->prDiffValid == 0) continue;
      // 2. bypass invalid PR
      if ((pSv1->status & 0x1) == 0) continue;
      // 3. system mode check
      if (pMeas->rtdUsed && pSv1->gnssMode == GLN_MODE) continue;

      sp1 = gnss_sd_get_sv_data(pSv1->gnssMode, pSv1->prn, pSv1->freq_index);
      if (sp1 == NULL) continue;

      delta = sp0->d_cos.az * RAD2DEG - sp1->d_cos.az * RAD2DEG;
      if (delta > 180.0)
      {
        delta -= 360;
      }
      else if (delta < -180.0)
      {
        delta += 360;
      }
      delta = fabs(delta);

      if ((pSv0->quality & PR_GOOD) && (pSv1->quality & PR_GOOD))
      {
        angleThres = 60;
        ratio2 = 1.0;
      }
      else
      {
        angleThres = 100;
      }
      if (delta >= angleThres * ratio2 && fabs((double)pSv0->pr_diff - pSv1->pr_diff) < 30 * ratio1)
      {
        if (!(pSv0->quality & PR_GOOD))
        {
          index[i] = j;
          avgDiff[i] = (pSv0->pr_diff + pSv1->pr_diff) / 2;
          break;
        }
        else if (pSv0->quality & PR_GOOD)
        {
          if (firstIndex == -1)
          {
            firstIndex = j;
            firstAvgDiff = (pSv0->pr_diff + pSv1->pr_diff) / 2;
          }
          if (pSv1->quality & PR_GOOD)
          {
            index[i] = j;
            avgDiff[i] = (pSv0->pr_diff + pSv1->pr_diff) / 2;
            qualityFlag[i] = TRUE;
            break;
          }
        }
      }
    }
    if (index[i] == -1 && firstIndex != -1)
    {
      index[i] = firstIndex;
      avgDiff[i] = firstAvgDiff;
    }
  }
  for (i = 0; i < pMeas->measCnt; i++)
  {
    if (index[i] < 0) continue;
    if (pMeas->meas[index[i]].cno >= 25 && fabs(avgDiff[i]) < 50.0)
    {
      if (firstCandidate == -1)
      {
        firstCandidate = i;
      }

      goodQuality = qualityFlag[i];
      if (goodQuality)
      {
        candidateIndex = i;
        //minAvgDiff = fabs(avgDiff[i]);
        break;
      }

      //minAvgDiff = fabs(avgDiff[i]);
    }
  }
  if (!goodQuality)
  {
    candidateIndex = firstCandidate;
  }

  if (candidateIndex >= 0)
  {
    (*idx0) = candidateIndex;
    (*idx1) = index[candidateIndex];
    pSv0 = &(pMeas->meas[candidateIndex]);
    pSv1 = &(pMeas->meas[index[candidateIndex]]);
    SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s,%02d,%02d,%02d,%02d,%x,%x,%10.4f,%10.4f", __FUNCTION__, pSv0->gnssMode, pSv0->prn, pSv1->gnssMode, pSv1->prn, pSv0->quality, pSv1->quality, pSv0->pr_diff, pSv1->pr_diff);
  }
}

/***********************************************************************
* 函数介绍: gnss_Qos_Outlier_Prepare_SPRD
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：3/13/
***********************************************************************/
void gnss_Qos_Outlier_Prepare_SPRD(meas_blk_t* pMeas, int8_t* idx0, int8_t* idx1)
{
  uint8_t                 i, j;
  int8_t                 index[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], candidateIndex = -1;
  double                avgDiff[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 };
  gnss_meas_t* pSv0, * pSv1;
  sat_data_t* sp0, * sp1;
  double                delta;

  (*idx0) = -1;
  (*idx1) = -1;
  memset(index, -1, sizeof(int8_t) * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM);
  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSv0 = &(pMeas->meas[i]);

    // 1. bypass invalid PRN and invalid PR diff
    if (pSv0->prn == 0 || pSv0->prDiffValid == 0) continue;
    // 2. bypass invalid PR
    if ((pSv0->status & 0x1) == 0) continue;
    if (pMeas->rtdUsed && pSv0->gnssMode == GLN_MODE) continue;

    sp0 = gnss_sd_get_sv_data(pSv0->gnssMode, pSv0->prn, pSv0->freq_index);
    if (sp0 == NULL) continue;

    for (j = i + 1; j < pMeas->measCnt; j++)
    {
      pSv1 = &(pMeas->meas[j]);

      // 1. bypass invalid PRN and invalid PR diff
      if (pSv1->prn == 0 || pSv1->prDiffValid == 0) continue;
      // 2. bypass invalid PR
      if ((pSv1->status & 0x1) == 0) continue;
      // 3. system mode check
      if (pMeas->rtdUsed && pSv1->gnssMode == GLN_MODE) continue;

      sp1 = gnss_sd_get_sv_data(pSv1->gnssMode, pSv1->prn, pSv1->freq_index);
      if (sp1 == NULL) continue;

      // case1: both satellite are good
      if ((pSv0->quality & PR_GOOD) && (pSv1->quality & PR_GOOD))
      {
        index[i] = j;
        avgDiff[i] = (pSv0->pr_diff + pSv1->pr_diff) / 2;
        break;
      }

      // case2: separate for 100 degree and PR DIFF smaller than 30
      delta = sp0->d_cos.az * RAD2DEG - sp1->d_cos.az * RAD2DEG;
      if (delta > 180.0)
      {
        delta -= 360;
      }
      else if (delta < -180.0)
      {
        delta += 360;
      }
      delta = fabs(delta);
      if (delta >= 100.0 && fabs((double)pSv0->pr_diff - pSv1->pr_diff) < 30.0)
      {
        index[i] = j;
        avgDiff[i] = (pSv0->pr_diff + pSv1->pr_diff) / 2;
        break;
      }
    }
  }
  for (i = 0; i < pMeas->measCnt; i++)
  {
    if (index[i] < 0) continue;
    if (pMeas->meas[index[i]].cno >= 25 && fabs(avgDiff[i]) < 50.0)
    {
      candidateIndex = i;
      //minAvgDiff = fabs(avgDiff[i]);
      break;
    }
  }

  if (candidateIndex >= 0)
  {
    (*idx0) = candidateIndex;
    (*idx1) = index[candidateIndex];
#if defined(PLAYBACK_MODE)
    pSv0 = &(pMeas->meas[candidateIndex]);
    pSv1 = &(pMeas->meas[index[candidateIndex]]);
    SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s,%02d,%02d,%02d,%02d,%x,%x,%10.4f,%10.4f", __FUNCTION__, pSv0->gnssMode, pSv0->prn, pSv1->gnssMode, pSv1->prn, pSv0->quality, pSv1->quality, pSv0->pr_diff, pSv1->pr_diff);
#endif
  }
}

/***********************************************************************
* 函数介绍: gnss_Qos_Outlier_QCOM
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：10/12/
***********************************************************************/
void gnss_Qos_Outlier_QCOM(meas_blk_t* pMeas)
{
  uint8_t            DRRejFlag = FALSE;
  int8_t            idx0 = -1, idx1 = -1;
  uint8_t            criteria = 0, i, j, prRejCnt = 0, prRejLimit = 0;
  uint8_t            index[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 }, indexTmp;
  uint8_t            diffNum = 0;
  uint8_t            smallDiffNum1 = 0;
  gnss_meas_t* pSvMeas0 = NULL, * pSvMeas1 = NULL, * pSvMeas;
  float           avg_diff, threshold, minDiffThres = 30.0, thres1, thres2;
  double           ratio = 1.0, threshold_ratio = 10.0;
  //DOP_TYPE      dop1,dop2;
  float           prDiff[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, prDiffTmp;
  USER_MOTION* pUsrContext;
  Kf_t* p_Kf;


  p_Kf = gnss_Pe_Get_Kf();

  pUsrContext = &peMode.userContext;
  if (pMeas->measCnt <= 5 || pMeas->hasDiff == FALSE)
  {
    return;
  }

  if (peState.posHasBias)
  {
    ratio = 1.0;
  }

  gnss_Pe_PRDR_Num(pMeas);
  gnss_Qos_Outlier_Prepare_QCOM(pMeas, &idx0, &idx1);
  /* determine the PR rejected limit */
  if (pMeas->validPrNum <= 6)
  {
    prRejLimit = 0;
  }
  else if (pMeas->validPrNum >= 18)
  {
    threshold_ratio = 2.0;
    prRejLimit = 8;
    minDiffThres = 10.0;
  }
  else if (pMeas->validPrNum >= 15)
  {
    prRejLimit = 6;
    threshold_ratio = 4.0;
    minDiffThres = 20.0;
  }
  else if (pMeas->validPrNum >= 12)
  {
    prRejLimit = 4;
    threshold_ratio = 6.0;
  }
  else if (pMeas->validPrNum >= 10)
  {
    prRejLimit = 2;
    threshold_ratio = 8.0;
  }
  else
  {
    prRejLimit = 1;
  }
  if (p_Kf->kfCnt <= 10)
  {
    threshold_ratio = 10.0;
  }
#if 0
  /* Suppose maximum two cno has least PR error */
  for (i = 0; i < pMeas->measCnt; i++)
  {
    if (pMeas->meas[i].prn == 0 || pMeas->meas[i].prDiffValid == 0) continue;

    if ((pMeas->meas[i].status & 0x1) && (pSvMeas0 == NULL))
    {
      pSvMeas0 = &pMeas->meas[i];
      continue;
    }
    if (pSvMeas0)
    {
      if ((pMeas->meas[i].status & 0x1) && (pSvMeas1 == NULL))
      {
        pSvMeas1 = &pMeas->meas[i];
        break;
      }
    }
  }

  if (pSvMeas0 == NULL || pSvMeas1 == NULL) return;

  sp0 = gnss_sd_get_sv_data(pSvMeas0->gnssMode, pSvMeas0->prn);
  sp1 = gnss_sd_get_sv_data(pSvMeas1->gnssMode, pSvMeas1->prn);

  /* Check satellite data */
  if (sp0 == NULL || sp1 == NULL)
  {
    return;
  }

  delta = sp0->d_cos.az * RAD2DEG - sp1->d_cos.az * RAD2DEG;
  if (delta > 180.0)
  {
    delta -= 360;
  }
  else if (delta < -180.0)
  {
    delta += 360;
  }

  /* If az diff is smaller than 60, then return */
  if (fabs(delta) < 60)
  {
    for (i = 0; i < pMeas->measCnt; i++)
    {
      pSvMeas1 = &pMeas->meas[i];
      if (pSvMeas1->prn == 0 || pSvMeas1->prDiffValid == 0) continue;
      if ((pSvMeas1->status & 0x1) == 0) continue;
      if (pSvMeas1->prn == pSvMeas0->prn && pSvMeas1->gnssMode == pSvMeas0->gnssMode) continue;
      sp1 = gnss_sd_get_sv_data(pSvMeas1->gnssMode, pSvMeas1->prn);
      delta = sp0->d_cos.az * RAD2DEG - sp1->d_cos.az * RAD2DEG;
      if (delta > 180.0)
      {
        delta -= 360;
      }
      else if (delta < -180.0)
      {
        delta += 360;
      }
      if (fabs(delta) >= 60 && pSvMeas1->cno >= 25)
      {
        goto OUTLIER_PROC;
      }
    }
    return;
  }
#endif
  if (idx0 >= 0 && idx0 < MAX_MEAS_NUM)
  {
    pSvMeas0 = &(pMeas->meas[idx0]);
  }
  else
  {
    pSvMeas0 = NULL;
  }
  if (idx1 >= 0 && idx1 < MAX_MEAS_NUM)
  {
    pSvMeas1 = &(pMeas->meas[idx1]);
  }
  else
  {
    pSvMeas1 = NULL;
  }
  if (pSvMeas0 == NULL || pSvMeas1 == NULL)
  {
    return;
  }
  //OUTLIER_PROC:
  /*
  1. If diff of maximum two cno is smaller than 20, then do it
  2. If diff is larger than 30 and it's LS fix, then do it
  */
  if (fabs((double)pSvMeas0->pr_diff - pSvMeas1->pr_diff) < 30 * ratio)
  {
    criteria = 1;
  }
  else if (fabs((double)pSvMeas0->pr_diff - pSvMeas1->pr_diff) < 40 * ratio && firstFix == FALSE)
  {
    criteria = 2;
  }

  if (g_pe_cfg.automobile && peMode.userSceData.isUnderEleRoad)
  {
    thres1 = 60.0;
    thres2 = 15.0;
  }
  else
  {
    thres1 = 50.0;
    thres2 = 10.0;
  }

  if (criteria > 0)
  {
    /* calculate average PR diff */
    avg_diff = (pSvMeas0->pr_diff + pSvMeas1->pr_diff) / 2;

    if (criteria == 1)
    {
      threshold = (float)(threshold_ratio * fabs((double)pSvMeas0->pr_diff - pSvMeas1->pr_diff));
      /* set threshold */
      if (threshold > thres1)
      {
        threshold = thres1;
      }
      else if (threshold < thres2)
      {
        threshold = thres2;
      }
      /*
      1. low speed mode, use smaller threshold
      */
      if (pUsrContext->inital)
      {
        if (pUsrContext->motion == USER_MOTION_STATIC || pUsrContext->motion == USER_MOTION_WALK)
        {
          threshold = thres2;
        }
      }
    }
    else /*if (criteria == 2)*/
    {
      threshold = (float)(5 * fabs((double)pSvMeas0->pr_diff - pSvMeas1->pr_diff));
      if (threshold < thres1)
      {
        threshold = thres1;
      }
    }
  }
  else
  {
    return;
  }

  /* save diff*/
  for (i = 0; i < pMeas->measCnt; i++)
  {
    if (pMeas->meas[i].prn == 0 || pMeas->meas[i].prDiffValid == 0) continue;
    if ((pMeas->meas[i].status & 0x1) == 0) continue;
    if (fabs((double)pMeas->meas[i].pr_diff - avg_diff) < 10)
    {
      smallDiffNum1++;
    }
    prDiff[diffNum] = pMeas->meas[i].pr_diff;
    index[diffNum] = i;
    diffNum++;
  }


  /*
  1. adjust threshold according to small diff number;
  2. If more than 12 valid PR and 80 percent has small diff, then use 20 as threshold
  */
  pMeas->isSmallDiff = FALSE;
  if (diffNum != 0)
  {
    if ((double)smallDiffNum1 / (double)diffNum > 0.80)
    {
      if ((diffNum >= 20) && (p_Kf->kfCnt > 10))
      {
        threshold = 7.0;
      }
      else if (diffNum >= 15)
      {
        threshold = 12.0;
      }
      else if (diffNum >= 12)
      {
        threshold = 15.0;
      }
      else
      {
        threshold = 25.0;
      }
      pMeas->isSmallDiff = TRUE;
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "Use Smaller threshold in %s : %10.4f", __FUNCTION__, threshold);
    }
    else if (peMode.userSceData.isPDRMode)
    {
      if (smallDiffNum1 >= 8)
      {
        if (smallDiffNum1 >= 12)
        {
          threshold = 10.0;
        }
        else
        {
          threshold = 20.0;
        }
        pMeas->isSmallDiff = TRUE;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "Use Smaller threshold under Walking Mode: %10.4f", threshold);
      }
      else if (smallDiffNum1 >= 6 && p_Kf->posResStd < 10.0)
      {
        threshold = 30.0;
        pMeas->isSmallDiff = TRUE;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "Use Smaller threshold under Walking Mode: %10.4f", threshold);
      }
    }

    /* Sorting according to Diff */
    for (i = diffNum - 1; i > 0; i--)
    {
      for (j = 0; j < i; j++)
      {
        if (fabs(prDiff[j]) < fabs(prDiff[j + 1]))
        {
          prDiffTmp = prDiff[j];
          indexTmp = index[j];
          prDiff[j] = prDiff[j + 1];
          index[j] = index[j + 1];
          prDiff[j + 1] = prDiffTmp;
          index[j + 1] = indexTmp;
        }
      }
    }
  }

  SYS_LOGGING(OBJ_QOS, LOG_INFO, "OBS DIFF:%14.6f,%14.6f,%f", pSvMeas0->pr_diff, pSvMeas1->pr_diff, threshold);
  /* Check the diff with threshold
  (1) If larger than 1.5*threshold, then reject this satellite
  (2) If larger than threshold, then de-weight this satellite
  */
#if 0
  if (peMode.staticData.staticFlag)
  {
    if (threshold > 50.0)
    {
      threshold = 50.0;
    }
  }
#endif
  for (i = 0; i < diffNum; i++)
  {
    indexTmp = index[i];
    pSvMeas = &pMeas->meas[indexTmp];
    if (pSvMeas->prn == 0) continue;
    if ((pSvMeas->gnssMode == pSvMeas0->gnssMode) && (pSvMeas->prn == pSvMeas0->prn))
    {
      continue;
    }
    if ((pSvMeas->gnssMode == pSvMeas1->gnssMode) && (pSvMeas->prn == pSvMeas1->prn))
    {
      continue;
    }
    DRRejFlag = FALSE;
    if ((pSvMeas->status & 0x3) == 0) continue;
    if (pSvMeas->quality & PR_GOOD) continue;
    // PR check
    if (fabs((double)pSvMeas->pr_diff - avg_diff) > 1.5 * threshold && prRejCnt < prRejLimit)
    {
      /* If PDOP enlarge two times after reject this satellite, then don't reject this satellite */
      if (gnss_Pe_Dop_Check(pMeas, indexTmp, 1) == FALSE && fabs(pSvMeas->pr_diff) < 100) continue;

      prRejCnt++;
      pSvMeas->status &= 0xFE;
#if defined(PLAYBACK_MODE)
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was rejected in %s %10.4f",
        pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, pSvMeas->pr_diff);
#endif
      /* For pure MP, reject DR too */
      if (pMeas->isSmallDiff == FALSE)
      {
        if (fabs((double)pSvMeas->pr_diff - avg_diff) > 2.5 * threshold && pSvMeas->cno >= 35 &&
          ((peMode.staticData.staticFlag == FALSE && peMode.staticData.historyStatic) || (pSvMeas->drWeightChck > 0)))
        {
          DRRejFlag = TRUE;
          goto OUTLIER_DR_PROC;
        }
      }

      /* walking DR refine */
      if (peMode.userSceData.isPDRMode)
      {
        DRRejFlag = TRUE;
        goto OUTLIER_DR_PROC;
      }

      /* If position is biased, then reject DR if PR Error is very large */
      if (peState.posHasBias && fabs((double)pSvMeas->pr_diff - avg_diff) > 2.5 * threshold)
      {
        DRRejFlag = TRUE;
        goto OUTLIER_DR_PROC;
      }
    OUTLIER_DR_PROC:
      if (DRRejFlag)
      {
        pSvMeas->status &= 0xFD;
#if defined(PLAYBACK_MODE)
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "DR was reject in Outlier:%02d,%02d,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->dr_diff);
#endif
        continue;
      }

    }
    else if (fabs((double)pSvMeas->pr_diff - avg_diff) > threshold && fabs(pSvMeas->pr_diff) > minDiffThres)
    {
      pSvMeas->prWeightChck |= DIFF_CHECK;
#if defined(PLAYBACK_MODE)
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was de-weight in %s %10.4f",
        pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, pSvMeas->pr_diff);
#endif
    }

    // DR check
    if (fabs((double)pSvMeas->pr_diff - avg_diff) > 300)
    {
      pSvMeas->drWeightChck |= DIFF_CHECK;
    }
  }
}

/***********************************************************************
* 函数介绍: gnss_Qos_Outlier_UBLOX
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：10/12/
***********************************************************************/
void gnss_Qos_Outlier_UBLOX(meas_blk_t* pMeas)
{
  uint8_t            DRRejFlag = FALSE;
  int8_t            idx0 = -1, idx1 = -1;
  uint8_t            criteria = 0, i, j, prRejCnt = 0, prRejLimit = 0;
  uint8_t            index[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 }, indexTmp;
  uint8_t            diffNum = 0;
  uint8_t            smallDiffNum1 = 0;
  gnss_meas_t* pSvMeas0 = NULL;
  gnss_meas_t* pSvMeas1 = NULL;
  gnss_meas_t* pSvMeas = NULL;
  float           avg_diff, threshold, minDiffThres = 30.0, thres1, thres2;
  double           ratio = 1.0, threshold_ratio = 10.0;
  //DOP_TYPE      dop1,dop2;
  float           prDiff[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, prDiffTmp;
  USER_MOTION* pUsrContext;
  Kf_t* p_Kf;


  p_Kf = gnss_Pe_Get_Kf();

  pUsrContext = &peMode.userContext;
  if (pMeas->measCnt <= 5 || pMeas->hasDiff == FALSE)
  {
    return;
  }

  if (peState.posHasBias)
  {
    ratio = 1.0;
  }

  gnss_Pe_PRDR_Num(pMeas);
  gnss_Qos_Outlier_Prepare_QCOM(pMeas, &idx0, &idx1);
  /* determine the PR rejected limit */
  if (pMeas->validPrNum <= 6)
  {
    prRejLimit = 0;
  }
  else if (pMeas->validPrNum >= 18)
  {
    prRejLimit = 8;
    threshold_ratio = 2.0;
    minDiffThres = 10.0;
  }
  else if (pMeas->validPrNum >= 15)
  {
    prRejLimit = 6;
    threshold_ratio = 4.0;
    minDiffThres = 20.0;
  }
  else if (pMeas->validPrNum >= 12)
  {
    prRejLimit = 4;
    threshold_ratio = 6.0;
  }
  else if (pMeas->validPrNum >= 10)
  {
    prRejLimit = 2;
    threshold_ratio = 8.0;
  }
  else
  {
    prRejLimit = 1;
    threshold_ratio = 10.0;
  }

  if (p_Kf->kfCnt <= 20)
  {
    if (pMeas->validPrNum >= 18)
    {
      threshold_ratio = 3.0;
    }
    else if (pMeas->validPrNum >= 14)
    {
      threshold_ratio = 5.0;
    }
    else
    {
      threshold_ratio = 10.0;
    }
  }

  if (idx0 >= 0 && idx0 < MAX_MEAS_NUM)
  {
    pSvMeas0 = &(pMeas->meas[idx0]);
  }
  else
  {
    pSvMeas0 = NULL;
  }
  if (idx1 >= 0 && idx1 < MAX_MEAS_NUM)
  {
    pSvMeas1 = &(pMeas->meas[idx1]);
  }
  else
  {
    pSvMeas1 = NULL;
  }
  if (pSvMeas0 == NULL || pSvMeas1 == NULL)
  {
    return;
  }
  //OUTLIER_PROC:
  /*
  1. If diff of maximum two cno is smaller than 20, then do it
  2. If diff is larger than 30 and it's LS fix, then do it
  */
  if (fabs((double)pSvMeas0->pr_diff - pSvMeas1->pr_diff) < 30 * ratio)
  {
    criteria = 1;
  }
  else if (fabs((double)pSvMeas0->pr_diff - pSvMeas1->pr_diff) < 40 * ratio && firstFix == FALSE)
  {
    criteria = 2;
  }

  if (g_pe_cfg.automobile && peMode.userSceData.isUnderEleRoad && g_pe_cfg.chipType != UBLOX)
  {
    thres1 = 100.0;
    thres2 = 75.0;
  }
  else
  {
    thres1 = 50.0;
    thres2 = 10.0;
  }

  if (criteria > 0)
  {
    /* calculate average PR diff */
    avg_diff = (pSvMeas0->pr_diff + pSvMeas1->pr_diff) / 2;

    if (criteria == 1)
    {
      threshold = (float)(threshold_ratio * fabs((double)pSvMeas0->pr_diff - pSvMeas1->pr_diff));
      /* set threshold */
      if (threshold > thres1)
      {
        threshold = thres1;
      }
      else if (threshold < thres2)
      {
        threshold = thres2;
      }
      /*
      1. low speed mode, use smaller threshold
      */
      if (pUsrContext->inital)
      {
        if (pUsrContext->motion == USER_MOTION_STATIC || pUsrContext->motion == USER_MOTION_WALK)
        {
          threshold = thres2;
        }
      }
    }
    else /*if (criteria == 2)*/
    {
      threshold = (float)(5 * fabs((double)pSvMeas0->pr_diff - pSvMeas1->pr_diff));
      if (threshold < thres1)
      {
        threshold = thres1;
      }
    }
  }
  else
  {
    return;
  }

  /* save diff*/
  for (i = 0; i < pMeas->measCnt; i++)
  {
    if (pMeas->meas[i].prn == 0 || pMeas->meas[i].prDiffValid == 0) continue;
    if ((pMeas->meas[i].status & 0x1) == 0) continue;
    if (fabs((double)pMeas->meas[i].pr_diff - avg_diff) < 10)
    {
      smallDiffNum1++;
    }
    prDiff[diffNum] = pMeas->meas[i].pr_diff;
    index[diffNum] = i;
    diffNum++;
  }

  if (diffNum == 0) {
    return;
  }

  /*
  1. adjust threshold according to small diff number;
  2. If more than 12 valid PR and 80 percent has small diff, then use 20 as threshold
  */
  pMeas->isSmallDiff = FALSE;
  if ((double)smallDiffNum1 / (double)diffNum > 0.80)
  {
    if ((diffNum >= 20) && (p_Kf->kfCnt > 20))
    {
      threshold = 7.0;
    }
    else if (diffNum >= 15)
    {
      threshold = 15.0;
    }
    else if (diffNum >= 12)
    {
      threshold = 20.0;
    }
    else
    {
      threshold = 25.0;
    }
    pMeas->isSmallDiff = TRUE;
    SYS_LOGGING(OBJ_QOS, LOG_INFO, "Use Smaller threshold in %s : %10.4f", __FUNCTION__, threshold);
  }
  else if (peMode.userSceData.isPDRMode)
  {
    if (smallDiffNum1 >= 8)
    {
      if (smallDiffNum1 >= 12)
      {
        threshold = 10.0;
      }
      else
      {
        threshold = 20.0;
      }
      pMeas->isSmallDiff = TRUE;
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "Use Smaller threshold under Walking Mode: %10.4f", threshold);
    }
    else if (smallDiffNum1 >= 6 && p_Kf->posResStd < 10.0)
    {
      threshold = 30.0;
      pMeas->isSmallDiff = TRUE;
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "Use Smaller threshold under Walking Mode: %10.4f", threshold);
    }
  }
  /* Sorting according to Diff */
  for (i = diffNum - 1; i > 0; i--)
  {
    for (j = 0; j < i; j++)
    {
      if (fabs(prDiff[j]) < fabs(prDiff[j + 1]))
      {
        prDiffTmp = prDiff[j];
        indexTmp = index[j];
        prDiff[j] = prDiff[j + 1];
        index[j] = index[j + 1];
        prDiff[j + 1] = prDiffTmp;
        index[j + 1] = indexTmp;
      }
    }
  }

  SYS_LOGGING(OBJ_QOS, LOG_INFO, "OBS DIFF:%14.6f,%14.6f,%f", pSvMeas0->pr_diff, pSvMeas1->pr_diff, threshold);
  /* Check the diff with threshold
  (1) If larger than 1.5*threshold, then reject this satellite
  (2) If larger than threshold, then de-weight this satellite
  */
#if 0
  if (peMode.staticData.staticFlag)
  {
    if (threshold > 50.0)
    {
      threshold = 50.0;
    }
  }
#endif
  for (i = 0; i < diffNum; i++)
  {
    indexTmp = index[i];
    pSvMeas = &pMeas->meas[indexTmp];
    if (pSvMeas->prn == 0) continue;
    if ((pSvMeas->gnssMode == pSvMeas0->gnssMode) && (pSvMeas->prn == pSvMeas0->prn) && (pSvMeas->freq_index == pSvMeas0->freq_index))
    {
      continue;
    }
    if ((pSvMeas->gnssMode == pSvMeas1->gnssMode) && (pSvMeas->prn == pSvMeas1->prn) && (pSvMeas->freq_index == pSvMeas1->freq_index))
    {
      continue;
    }
    DRRejFlag = FALSE;
    if ((pSvMeas->status & 0x3) == 0) continue;
    if (pSvMeas->quality & PR_GOOD) continue;
    // PR check
    if (fabs((double)pSvMeas->pr_diff - avg_diff) > 1.5 * threshold && prRejCnt < prRejLimit)
    {
      /* If PDOP enlarge two times after reject this satellite, then don't reject this satellite */
      if (gnss_Pe_Dop_Check(pMeas, indexTmp, 1) == FALSE && fabs(pSvMeas->pr_diff) < 100) continue;

      prRejCnt++;
      pSvMeas->status &= 0xFE;
#if defined(PLAYBACK_MODE)
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was rejected in %s %10.4f",
        pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, pSvMeas->pr_diff);
#endif
      /* For pure MP, reject DR too */
      if (pMeas->isSmallDiff == FALSE)
      {
        if (fabs((double)pSvMeas->pr_diff - avg_diff) > 2.5 * threshold && pSvMeas->cno >= 35 &&
          ((peMode.staticData.staticFlag == FALSE && peMode.staticData.historyStatic) || (pSvMeas->drWeightChck > 0)))
        {
          DRRejFlag = TRUE;
          goto OUTLIER_DR_PROC;
        }
      }

      /* walking DR refine */
      if (peMode.userSceData.isPDRMode || fabs(pSvMeas->pr_diff) > 1000.0)
      {
        DRRejFlag = TRUE;
        goto OUTLIER_DR_PROC;
      }

      /* If position is biased, then reject DR if PR Error is very large */
      if (peState.posHasBias && fabs((double)pSvMeas->pr_diff - avg_diff) > 2.5 * threshold)
      {
        DRRejFlag = TRUE;
        goto OUTLIER_DR_PROC;
      }
    OUTLIER_DR_PROC:
      if (DRRejFlag)
      {
        pSvMeas->status &= 0xFD;
#if defined(PLAYBACK_MODE)
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "DR was reject in Outlier:%02d,%02d,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->dr_diff);
#endif
        continue;
      }

    }
    else if (fabs((double)pSvMeas->pr_diff - avg_diff) > threshold && fabs(pSvMeas->pr_diff) > minDiffThres)
    {
      pSvMeas->prWeightChck |= DIFF_CHECK;
#if defined(PLAYBACK_MODE)
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was de-weight in %s %10.4f",
        pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, pSvMeas->pr_diff);
#endif
    }

    // DR check
    if (fabs((double)pSvMeas->pr_diff - avg_diff) > 300)
    {
      pSvMeas->drWeightChck |= DIFF_CHECK;
    }
  }
}
/***********************************************************************
* 函数介绍: gnss_Qos_Outlier_SPRD
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：10/12/
***********************************************************************/
void gnss_Qos_Outlier_SPRD(meas_blk_t* pMeas)
{
#ifdef USED_IN_MC262M
  int8_t            idx0 = -1, idx1 = -1;
  uint8_t            criteria = 0, i, j, prRejCnt = 0, prRejLimit = 0;
  uint8_t            index[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 }, indexTmp;
  uint8_t            diffNum = 0;
  uint8_t            smallDiffNum1 = 0;
  gnss_meas_t* pSvMeas0 = NULL, * pSvMeas1 = NULL, * pSvMeas;
  float           avg_diff, threshold, minDiffThres = 30.0, thres1, thres2;
  //DOP_TYPE      dop1,dop2;
  float           prDiff[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, prDiffTmp;
  USER_MOTION* pUsrContext;


  pUsrContext = &peMode.userContext;
  if (pMeas->measCnt <= 5 || pMeas->hasDiff == FALSE)
  {
    return;
  }

  gnss_Pe_PRDR_Num(pMeas);
  gnss_Qos_Outlier_Prepare_SPRD(pMeas, &idx0, &idx1);
  /* determine the PR rejected limit */
  if (pMeas->validPrNum <= 5)
  {
    prRejLimit = 0;
  }
  else if (pMeas->validPrNum > 12)
  {
    prRejLimit = 4;
  }
  else if (pMeas->validPrNum > 10)
  {
    prRejLimit = 2;
  }
  else
  {
    prRejLimit = pMeas->validPrNum - 6;
  }
#if 0
  /* Suppose maximum two cno has least PR error */
  for (i = 0; i < pMeas->measCnt; i++)
  {
    if (pMeas->meas[i].prn == 0 || pMeas->meas[i].prDiffValid == 0) continue;

    if ((pMeas->meas[i].status & 0x1) && (pSvMeas0 == NULL))
    {
      pSvMeas0 = &pMeas->meas[i];
      continue;
    }
    if (pSvMeas0)
    {
      if ((pMeas->meas[i].status & 0x1) && (pSvMeas1 == NULL))
      {
        pSvMeas1 = &pMeas->meas[i];
        break;
      }
    }
  }

  if (pSvMeas0 == NULL || pSvMeas1 == NULL) return;

  sp0 = gnss_sd_get_sv_data(pSvMeas0->gnssMode, pSvMeas0->prn);
  sp1 = gnss_sd_get_sv_data(pSvMeas1->gnssMode, pSvMeas1->prn);

  /* Check satellite data */
  if (sp0 == NULL || sp1 == NULL)
  {
    return;
  }

  delta = sp0->d_cos.az * RAD2DEG - sp1->d_cos.az * RAD2DEG;
  if (delta > 180.0)
  {
    delta -= 360;
  }
  else if (delta < -180.0)
  {
    delta += 360;
  }

  /* If az diff is smaller than 60, then return */
  if (fabs(delta) < 60)
  {
    for (i = 0; i < pMeas->measCnt; i++)
    {
      pSvMeas1 = &pMeas->meas[i];
      if (pSvMeas1->prn == 0 || pSvMeas1->prDiffValid == 0) continue;
      if ((pSvMeas1->status & 0x1) == 0) continue;
      if (pSvMeas1->prn == pSvMeas0->prn && pSvMeas1->gnssMode == pSvMeas0->gnssMode) continue;
      sp1 = gnss_sd_get_sv_data(pSvMeas1->gnssMode, pSvMeas1->prn);
      delta = sp0->d_cos.az * RAD2DEG - sp1->d_cos.az * RAD2DEG;
      if (delta > 180.0)
      {
        delta -= 360;
      }
      else if (delta < -180.0)
      {
        delta += 360;
      }
      if (fabs(delta) >= 60 && pSvMeas1->cno >= 25)
      {
        goto OUTLIER_PROC;
      }
    }
    return;
  }
#endif
  if (idx0 >= 0 && idx0 < MAX_MEAS_NUM)
  {
    pSvMeas0 = &(pMeas->meas[idx0]);
  }
  else
  {
    pSvMeas0 = NULL;
  }
  if (idx1 >= 0 && idx1 < MAX_MEAS_NUM)
  {
    pSvMeas1 = &(pMeas->meas[idx1]);
  }
  else
  {
    pSvMeas1 = NULL;
  }
  if (pSvMeas0 == NULL || pSvMeas1 == NULL)
  {
    return;
  }
  //OUTLIER_PROC:
  /*
  1. If diff of maximum two cno is smaller than 20, then do it
  2. If diff is larger than 30 and it's LS fix, then do it
  */
  if (fabs(pSvMeas0->pr_diff - pSvMeas1->pr_diff) < 30)
  {
    criteria = 1;
  }
  else if (fabs(pSvMeas0->pr_diff - pSvMeas1->pr_diff) < 40 && firstFix == FALSE)
  {
    criteria = 2;
  }

  if (g_pe_cfg.automobile && peMode.userSceData.isUnderEleRoad)
  {
    thres1 = 100.0;
    thres2 = 75.0;
  }
  else
  {
    thres1 = 80.0;
    thres2 = 50.0;
  }

  if (criteria > 0)
  {
    /* calculate average PR diff */
    avg_diff = (pSvMeas0->pr_diff + pSvMeas1->pr_diff) / 2;

    if (criteria == 1)
    {
      threshold = (float)(10 * fabs(pSvMeas0->pr_diff - pSvMeas1->pr_diff));
      /* set threshold */
      if (threshold > thres1)
      {
        threshold = thres1;
      }
      else if (threshold < thres2)
      {
        threshold = thres2;
      }
      /*
      1. low speed mode, use smaller threshold
      */
      if (pUsrContext->inital)
      {
        if (pUsrContext->motion == USER_MOTION_STATIC || pUsrContext->motion == USER_MOTION_WALK)
        {
          threshold = thres2;
        }
      }
    }
    else if (criteria == 2)
    {
      threshold = (float)(5 * fabs(pSvMeas0->pr_diff - pSvMeas1->pr_diff));
      if (threshold < thres1)
      {
        threshold = thres1;
      }
    }
  }
  else
  {
    return;
  }

  /* save diff*/
  for (i = 0; i < pMeas->measCnt; i++)
  {
    if (pMeas->meas[i].prn == 0 || pMeas->meas[i].prDiffValid == 0) continue;
    if ((pMeas->meas[i].status & 0x1) == 0) continue;
    if (fabs(pMeas->meas[i].pr_diff - avg_diff) < 10)
    {
      smallDiffNum1++;
    }
    prDiff[diffNum] = pMeas->meas[i].pr_diff;
    index[diffNum] = i;
    diffNum++;
  }


  /*
  1. adjust threshold according to small diff number;
  2. If more than 12 valid PR and 80 percent has small diff, then use 20 as threshold
  */
  pMeas->isSmallDiff = FALSE;
  if (diffNum != 0)
  {
    if ((double)smallDiffNum1 / (double)diffNum > 0.80)
    {
      if (diffNum >= 15)
      {
        threshold = 15.0;
      }
      else if (diffNum >= 12)
      {
        threshold = 20.0;
      }
      else
      {
        threshold = 25.0;
      }
      pMeas->isSmallDiff = TRUE;
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "Use Smaller threshold in %s : %10.4f", __FUNCTION__, threshold);
    }
    /* Sorting according to Diff */
    for (i = diffNum - 1; i > 0; i--)
    {
      for (j = 0; j < i; j++)
      {
        if (fabs(prDiff[j]) < fabs(prDiff[j + 1]))
        {
          prDiffTmp = prDiff[j];
          indexTmp = index[j];
          prDiff[j] = prDiff[j + 1];
          index[j] = index[j + 1];
          prDiff[j + 1] = prDiffTmp;
          index[j + 1] = indexTmp;
        }
      }
    }
  }

  SYS_LOGGING(OBJ_QOS, LOG_INFO, "OBS DIFF:%14.6f,%14.6f,%f", pSvMeas0->pr_diff, pSvMeas1->pr_diff, threshold);
  /* Check the diff with threshold
  (1) If larger than 1.5*threshold, then reject this satellite
  (2) If larger than threshold, then de-weight this satellite
  */
  for (i = 0; i < diffNum; i++)
  {
    indexTmp = index[i];
    pSvMeas = &pMeas->meas[indexTmp];
    if (pSvMeas->prn == 0) continue;
    if ((pSvMeas->gnssMode == pSvMeas0->gnssMode) && (pSvMeas->prn == pSvMeas0->prn))
    {
      continue;
    }
    if ((pSvMeas->gnssMode == pSvMeas1->gnssMode) && (pSvMeas->prn == pSvMeas1->prn))
    {
      continue;
    }
    if ((pSvMeas->status & 0x3) == 0) continue;
    if (pSvMeas->quality & PR_GOOD) continue;
    // PR check
    if (fabs(pSvMeas->pr_diff - avg_diff) > 1.5 * threshold && prRejCnt < prRejLimit)
    {
      /* If PDOP enlarge two times after reject this satellite, then don't reject this satellite */
      if (gnss_Pe_Dop_Check(pMeas, indexTmp, 1) == FALSE) continue;

      prRejCnt++;
      pSvMeas->status &= 0xFE;
      if ((pSvMeas0->quality & 0x1) && (pSvMeas1->quality & 0x1))
      {
        if (fabs(pSvMeas->pr_diff - avg_diff) > 2.5 * threshold && pMeas->isSmallDiff == FALSE)
        {
          pSvMeas->status &= 0xFD;
        }
      }
#if defined(PLAYBACK_MODE)
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was rejected in %s %10.4f,%10.4f",
        pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, pSvMeas->pr_diff, pSvMeas->dr_err);
#endif
    }
    else if (fabs(pSvMeas->pr_diff - avg_diff) > threshold && fabs(pSvMeas->pr_diff) > minDiffThres)
    {
      pSvMeas->prWeightChck |= DIFF_CHECK;
#if defined(PLAYBACK_MODE)
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was deWeight in %s %10.4f,%10.4f",
        pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, pSvMeas->pr_diff, pSvMeas->dr_err);
#endif
    }

    // DR check
    if (fabs(pSvMeas->pr_diff - avg_diff) > 300)
    {
      pSvMeas->drWeightChck |= DIFF_CHECK;
    }
  }
#endif
}
/***********************************************************************
* 函数介绍: gnss_Qos_BadMeasFlag_Clear
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：9/19/
***********************************************************************/
void gnss_Qos_BadMeasFlag_Clear(sat_data_t* sp, uint8_t meas_type)
{
  if (sp == NULL) return;
  //if(g_pe_cfg.chipType != SPRD) return;

  if (meas_type == MEAS_TYPE_PR)
  {
    sp->badPrCnt = 0;
    sp->torBadPr = 0;
    sp->isBadPr = FALSE;
  }
  else if (meas_type == MEAS_TYPE_DR)
  {


  }
}
/***********************************************************************
* 函数介绍: gnss_Qos_BadMeas_Set
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：9/19/
***********************************************************************/
void gnss_Qos_BadMeasFlag_Set(double tor, sat_data_t* sp, uint8_t meas_type)
{
  if (sp == NULL) return;
  //if(g_pe_cfg.chipType != SPRD) return;

  if (meas_type == MEAS_TYPE_PR)
  {
    if (sp->isBadPr && fabs(tor - sp->torBadPr) > 0.5)
    {
      sp->torBadPr = tor;
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "PR Bad Meas detect: %02d,%03d,%10.4f", sp->gnssMode, sp->prn, tor);
    }
    else
    {
      //last epoch PR meas is reject/deweight in MAD/cluster
      if (sp->badPrCnt > 0 && (tor - sp->torBadPr) > 0.5 && (tor - sp->torBadPr) < 1.5)
      {
        sp->badPrCnt++;
        sp->torBadPr = tor;
        if (sp->badPrCnt >= BAD_MEAS_CNT)
        {
          sp->isBadPr = TRUE;
          SYS_LOGGING(OBJ_QOS, LOG_INFO, "PR Bad Meas detect: %02d,%03d,%10.4f", sp->gnssMode, sp->prn, tor);
        }
      }
      else
      {
        gnss_Qos_BadMeasFlag_Clear(sp, MEAS_TYPE_PR);
        sp->badPrCnt++;
        sp->torBadPr = tor;
      }
    }
  }
  else if (meas_type == MEAS_TYPE_DR)
  {


  }
}

/***********************************************************************
* 函数介绍: gnss_Qos_MAD
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：11/14/
***********************************************************************/
void gnss_Qos_MAD(meas_blk_t* pMeas)
{
  uint8_t            flag, prRejNum = 0, drRejNum = 0;
  uint32_t           i/*, j*/;
  uint8_t            index[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 };
  uint8_t            diffNum = 0;
  //uint8_t            status;
  //float           prDiffThres;
  float           diff[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, diffAbs[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 };
  float           median = 0.0, rejectK1 = 6.0, rejectK2 = 5.0, deweightK1 = 4.0, deweightK2 = 3.0;
  //sat_data_t* sp;
  //DOP_TYPE      dop1,dop2;
  Kf_t* p_Kf;
  float          diffAbs_std = 0.0, diffAbs_avg = 0.0;

  if (pMeas->measCnt <= 5 || pMeas->hasDiff == FALSE)
  {
    return;
  }

  gnss_Pe_PRDR_Num(pMeas);

  p_Kf = gnss_Pe_Get_Kf();

  if (p_Kf->kfCnt > 20)
  {
    if (pMeas->validPrNum > 25)
    {
      rejectK1 = 2.0, deweightK1 = 1.0;
    }

    if (pMeas->validDrNum > 25)
    {
      rejectK2 = 3.0, deweightK2 = 1.5;
    }
  }
  else
  {
    if (pMeas->validPrNum > 25)
    {
      rejectK1 = 4.0, deweightK1 = 2.0;
    }
  }

  // PR MAD outliers detection 
  for (i = 0; i < pMeas->measCnt; i++)
  {
    if (pMeas->meas[i].prn == 0 || pMeas->meas[i].prDiffValid == 0) continue;
    if ((pMeas->meas[i].status & 0x1) == 0) continue;
    diff[diffNum] = pMeas->meas[i].pr_diff;
    index[diffNum] = i;
    diffNum++;
  }

  flag = gnss_MAD(diff, diffAbs, diffNum, &median);
  gnss_math_fstd(diffAbs, diffNum, &diffAbs_avg, &diffAbs_std);
  SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s  %02d  avg %f std %f %f", __FUNCTION__, diffNum, diffAbs_avg, diffAbs_std, median);
  if (flag == FALSE) return;

  if (fabs(diffAbs_avg) > 10 && diffAbs_std > 10)
  {
    if (pMeas->validPrNum <= 25)
    {
      rejectK1 = 4.0, deweightK1 = 2.0;
    }
  }
  // PR Reject Logic
  for (i = 0; i < diffNum; i++)
  {
    if (diffAbs[i] > rejectK1 * (median / 0.6745))
    {
      /* If PDOP enlarge two times after reject this satellite, then don't reject this satellite */
      if (gnss_Pe_Dop_Check(pMeas, index[i], 1) == FALSE) continue;

      pMeas->meas[index[i]].status &= 0xFE;
      prRejNum++;
      //sp = gnss_sd_get_sv_data(pMeas->meas[index[i]].gnssMode, pMeas->meas[index[i]].prn, pMeas->meas[index[i]].freq_index);
      /*if(sp)
      {
      gnss_Qos_BadMeasFlag_Set(pMeas->tor,sp,MEAS_TYPE_PR);
      }*/

#if defined(PLAYBACK_MODE)
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s reject PR %d %02d %03d %8.3lf", __FUNCTION__, pMeas->meas[index[i]].freq_index, pMeas->meas[index[i]].gnssMode, pMeas->meas[index[i]].prn, diffAbs[i]);
#endif
      if (peMode.userSceData.isPDRMode || diffAbs[i] > 1000.0)
      {
        pMeas->meas[index[i]].status &= 0xFD;
#if defined(PLAYBACK_MODE)
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s reject DR %d %02d %03d", __FUNCTION__, pMeas->meas[index[i]].freq_index, pMeas->meas[index[i]].gnssMode, pMeas->meas[index[i]].prn);
#endif
      }
    }
  }
  // PR de-weight Logic
  if (prRejNum < 3)
  {
    for (i = 0; i < diffNum; i++)
    {
      if ((pMeas->meas[index[i]].status & 0x1) == 0) continue;
      if (diffAbs[i] > deweightK1 * (median / 0.6745))
      {
        pMeas->meas[index[i]].prWeightChck |= MAD_CHECK;
        //sp = gnss_sd_get_sv_data(pMeas->meas[index[i]].gnssMode, pMeas->meas[index[i]].prn, pMeas->meas[index[i]].freq_index);
        /*if(sp)
        {
        gnss_Qos_BadMeasFlag_Set(pMeas->tor,sp,MEAS_TYPE_PR);
        }*/
#if defined(PLAYBACK_MODE)
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s de-weight PR %d %02d %03d", __FUNCTION__, pMeas->meas[index[i]].freq_index, pMeas->meas[index[i]].gnssMode, pMeas->meas[index[i]].prn);
#endif
      }
    }
  }

  //calculate the std of pr diff
  //diffNum = 0;
  //prDiffThres = 800;

  // DR MAD outliers detection
  diffNum = 0;
  for (i = 0; i < pMeas->measCnt; i++)
  {
    if (pMeas->meas[i].prn == 0) continue;
    if ((pMeas->meas[i].status & 0x2) == 0) continue;
    diff[diffNum] = pMeas->meas[i].dr_diff;
    index[diffNum] = i;
    diffNum++;
  }

  flag = gnss_MAD(diff, diffAbs, diffNum, &median);
  if (flag == FALSE) return;

  // DR MAD outliers reject and de-weight
  for (i = 0; i < diffNum; i++)
  {
    if (diffAbs[i] > rejectK2 * (median / 0.6745))
    {
      pMeas->meas[index[i]].status &= 0xFD;
      drRejNum++;
#if defined(PLAYBACK_MODE)
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s reject DR %d %02d,%03d,%10.4f", __FUNCTION__, pMeas->meas[index[i]].freq_index, pMeas->meas[index[i]].gnssMode, pMeas->meas[index[i]].prn, pMeas->meas[index[i]].dr_diff);
#endif
    }
    else if (diffAbs[i] > deweightK2 * (median / 0.6745))
    {
      pMeas->meas[index[i]].drWeightChck |= MAD_CHECK;
#if defined(PLAYBACK_MODE)
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s de-weight DR %d %02d,%03d,%10.4f", __FUNCTION__, pMeas->meas[index[i]].freq_index, pMeas->meas[index[i]].gnssMode, pMeas->meas[index[i]].prn, pMeas->meas[index[i]].dr_diff);
#endif
    }
  }
}

/***********************************************************************
* 函数介绍: gnss_Qos_DRSmoothPR
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：05/09/
***********************************************************************/
void gnss_Qos_DRSmoothPR(gnss_meas_t* pSvMeas)
{
  uint32_t              j, cnt;
  sat_data_t* sp;
  double              dt, diff, avgDr, avgPr;


  if (pSvMeas->prn == 0) return;
  if ((pSvMeas->status & 0x1) == 0) return;
  sp = gnss_sd_get_sv_data(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);
  if (sp == NULL) {
    return;
  }
  
  if (sp->measBackCnt == 0) {
    return;
  }

  avgPr = sp->measBack[0].pr;
  cnt = 1;
  for (j = 0; j < MEAS_SAVE_NUM - 1; j++)
  {
    dt = sp->measBack[j].tot - sp->measBack[j + 1].tot;
    if (fabs(dt) < 1.5 && sp->measBack[j].pr > 0.0 && sp->measBack[j + 1].pr > 0.0)
    {
      avgDr = 0.5 * (sp->measBack[j].dr + sp->measBack[j + 1].dr + sp->measBack[j].driftAdj);
      diff = sp->measBack[j].pr - (sp->measBack[j + 1].pr + avgDr * dt);
      if (fabs(diff) < 2000.0)
      {
        avgPr += (sp->measBack[j + 1].pr + avgDr * (sp->measBack[0].tor - sp->measBack[j + 1].tor));
        cnt++;
      }
      else
      {
        break;
      }
    }
  }
#ifdef USED_IN_MC262M
  pSvMeas->smoothedPR = avgPr / cnt;
#endif
  //SYS_LOGGING(OBJ_QOS,LOG_INFO,"%s,%02d,%02d,%14.4f,%14.4f,%14.4f",__FUNCTION__,pSvMeas->gnssMode,pSvMeas->prn,pSvMeas->pseudoRange,pSvMeas->smoothedPR,pSvMeas->pseudoRange - pSvMeas->smoothedPR);

}

/***********************************************************************
* 函数介绍: gnss_Qos_ReAcq_Proc
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：08/17/
***********************************************************************/
void gnss_Qos_PrStatus_Check(meas_blk_t* pMeas)
{
  uint32_t                  i;
  double                  sumDiffThres;
  gnss_meas_t* pSvMeas;
  sat_data_t* sp;

  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &(pMeas->meas[i]);
    if (pSvMeas->prn == 0 || pSvMeas->status == 0) continue;
    if ((pSvMeas->status & 0x1) == 0) continue;

    sp = gnss_sd_get_sv_data(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);
    if (sp == NULL) continue;
    if (sp->isReAcq)
    {
      // PR 
      if ((pMeas->tor - sp->torReAcq) <= 3.0)
      {
        pSvMeas->status &= 0xFE;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "PR ReAcq Rej: %14.6f,%02d,%02d", pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn);
        continue;
      }
      else
      {
        pSvMeas->prWeightChck |= REACQ_CHECK;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "PR ReAcq Deweight: %14.6f,%02d,%02d", pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn);
      }

      // DR
      if ((pMeas->tor - sp->torReAcq) <= 3.0 && pSvMeas->cno < 30)
      {
        pSvMeas->status &= 0xFD;
        continue;
      }
    }
    if (pSvMeas->cno >= 35)
    {
      sumDiffThres = 30;
    }
    else if (pSvMeas->cno >= 30)
    {
      sumDiffThres = 40;
    }
    else
    {
      sumDiffThres = 50;
    }
    if (sp->prWithErr)
    {
      if (sp->prDiffSum >= sumDiffThres)
      {
        pSvMeas->status &= 0xFE;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "PR Error Rej: %14.6f,%02d,%02d", pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn);
        continue;
      }
      else
      {
        pSvMeas->prWeightChck |= PRERR_CHECK;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "PR Error Deweight: %14.6f,%02d,%02d", pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn);
      }
    }
#if 0
    if (sp->isBadPr && !(pSvMeas->quality & PR_GOOD))
    {
      pSvMeas->prWeightChck |= BADMEAS_CHECK;
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "PR Bad Meas Deweight: %14.6f,%02d,%02d", pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn);
    }
#endif
  }
}
void gnss_Qos_PrDr_DrCheck(meas_blk_t* pMeas)
{
  uint32_t                  i;
  gnss_meas_t* pSvMeas;
  //sat_data_t* sp;

  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &(pMeas->meas[i]);
    if (pSvMeas->prn == 0 || pSvMeas->status == 0) continue;
    if ((pSvMeas->status & 0x3) != 2) continue;

    if (pMeas->avgCno < 25 && pSvMeas->cno <= 25 && pSvMeas->sv_info.fltElev < 25.0
      && fabs(pSvMeas->pr_diff) > 15.0 && pSvMeas->drDiffStd > 1.0)
    {
      pSvMeas->status &= 0xFD;
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "Dr rej because Pr: %14.6f,%02d,%02d", pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn);
    }
  }
}

/***********************************************************************
* 函数介绍: gnss_Qos_PrError_Detect
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：09/01/
***********************************************************************/
void gnss_Qos_PrError_Detect(gnss_meas_t* pSvMeas, double* prDiff, double* drDiff, uint8_t cnt, double tor, sat_data_t* sp)
{
#ifdef USED_IN_MC262M
  uint8_t             k, flag = FALSE;
  uint8_t             cntUse1 = 0, cntUse2 = 0;
  double            sumDiff = 0, sumDiff1 = 0, sumDiff2 = 0;
  double            thres1, thres2, thres3;

  if (pSvMeas->cno >= 35)
  {
    thres1 = 20;
    thres2 = 15;
    thres3 = 10;
  }
  else if (pSvMeas->cno >= 30)
  {
    thres1 = 25;
    thres2 = 20;
    thres3 = 15;
  }
  else
  {
    thres1 = 30;
    thres2 = 25;
    thres3 = 20;
  }

  for (k = 0; k < cnt; k++)
  {
    if (prDiff[k] > 0 && fabs(drDiff[k]) < 3.5)
    {
      cntUse1++;
      sumDiff1 += prDiff[k];
    }
    else
    {
      break;
    }
  }

  for (k = 0; k < cnt; k++)
  {
    if ((prDiff[k] > 0 || fabs(prDiff[k]) < 2.0) && fabs(drDiff[k]) < 3.0)
    {
      cntUse2++;
      sumDiff2 += prDiff[k];
    }
    else
    {
      break;
    }
  }

  if (cntUse2 == cnt && cnt >= 4 && sumDiff2 > thres2 && (tor - sp->torReAcq) > 12)
  {
    flag = TRUE;
    sumDiff = sumDiff2;
  }
  else if (cntUse1 >= 3 && sumDiff1 > thres1 && (tor - sp->torReAcq) > 12)
  {
    flag = TRUE;
    sumDiff = sumDiff1;
  }

  if (flag)
  {
    sp->prWithErr = TRUE;
    sp->torPrErr = tor;
    sp->prDiffSum = sumDiff;
    if (peMode.staticData.staticFlag == FALSE)
    {
#if defined(PLAYBACK_MODE)
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s : Detect PR Error: %14.6f,%02d,%02d,%14.4f,%14.4f,%14.4f,%14.4f,%14.4f,%d", __FUNCTION__, tor, pSvMeas->gnssMode, pSvMeas->prn, sumDiff, pSvMeas->pr_diff, pSvMeas->pr_err, pSvMeas->dr_err, pSvMeas->dr_diff, peMode.staticData.staticFlag);
#endif
    }
  }

  if (flag && !sp->updateSumDiff)
  {
    sp->updateSumDiff = TRUE;
    sp->prErrSum = sumDiff;
    SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s : Update PR Error Sum: %14.6f,%02d,%02d,%14.4f,%14.4f", __FUNCTION__, tor, pSvMeas->gnssMode, pSvMeas->prn, sp->prErrSum, pSvMeas->pr_diff);
  }
  else if (sp->updateSumDiff)
  {
    sp->prErrSum += prDiff[0];
    SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s : Update PR Error Sum: %14.6f,%02d,%02d,%14.4f,%14.4f", __FUNCTION__, tor, pSvMeas->gnssMode, pSvMeas->prn, sp->prErrSum, pSvMeas->pr_diff);
  }

  if (!flag && sp->updateSumDiff && (sp->prErrSum > 0 && sp->prErrSum < 15) || sp->prErrSum < 0)
  {
    sp->updateSumDiff = FALSE;
    sp->prErrSum = 0;
  }
#endif
}
/***********************************************************************
* 函数介绍: gnss_Qos_PRDR_Check_SPRD
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：11/16/
***********************************************************************/
void gnss_Qos_PRDR_Check_SPRD(meas_blk_t* pMeas)
{
#ifdef USED_IN_MC262M
  uint8_t             isReAcq;
  int8_t             sign[MEAS_SAVE_NUM] = { 0 };
  uint8_t             stdDiffNum = 0;
  uint32_t            i, j, cnt = 0, cntUse = 0, cntSign = 0;
  gnss_meas_t* pSvMeas;
  sat_data_t* sp;
  double            avgDr, dt;
  double            prDiff[MEAS_SAVE_NUM] = { 0.0 }, sumDiff, stdDiff = 0.0, avgDiff = 0.0;
  double            stdDiffBack[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 };
  double            sumThres, ratio = 3.0, stdThres;
  double            drDiff[MEAS_SAVE_NUM], drDiffStdThres, prDiffThres;

  sumThres = 40.0;
  ratio = 2.0;
  drDiffStdThres = 1.50;
  prDiffThres = 20;


  /*1. Calculate the PRDR diff */
  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &pMeas->meas[i];
    if (pSvMeas->prn == 0) continue;
    if ((pSvMeas->status & 0x1) == 0) continue;
    sp = gnss_sd_get_sv_data(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);
    if (sp == NULL) continue;
    isReAcq = FALSE;
    /*
    Re-ACQ status check
    */
    if (sp->isReAcq)
    {
      if ((pMeas->tor - sp->torReAcq) >= 8)
      {
        sp->isReAcq = FALSE;
      }
    }

    //PR Error status check
    if (sp->prWithErr)
    {
      if ((pMeas->tor - sp->torPrErr) >= 4)
      {
        sp->prWithErr = FALSE;
      }
    }
    if (sp->isBadPr)
    {
      if ((pMeas->tor - sp->torBadPr) >= 3.5)
      {
        gnss_Qos_BadMeasFlag_Clear(sp, MEAS_TYPE_PR);
      }
    }

    cnt = 0;
    sumDiff = 0.0;
    memset(prDiff, 0, sizeof(double) * MEAS_SAVE_NUM);
    memset(sign, 0, sizeof(int8_t) * MEAS_SAVE_NUM);
    memset(drDiff, 0, sizeof(double) * MEAS_SAVE_NUM);

    for (j = 0; j < MEAS_SAVE_NUM - 1; j++)
    {
      dt = sp->measBack[j].tot - sp->measBack[j + 1].tot;
      if ((pSvMeas->gnssMode == GPS_MODE || pSvMeas->gnssMode == BDS_MODE || pSvMeas->gnssMode == GAL_MODE) && dt < (-SECS_IN_WEEK / 2.0))//cross week
      {
        dt += SECS_IN_WEEK;
      }
      else if (pSvMeas->gnssMode == GLN_MODE && dt < (-SECS_IN_DAY / 2.0))
      {
        dt += SECS_IN_DAY;
      }
      if (fabs(dt) < 1.5 && sp->measBack[j].pr > 0.0 && sp->measBack[j + 1].pr > 0.0)
      {
        avgDr = 0.5 * (sp->measBack[j].dr + sp->measBack[j + 1].dr);
        prDiff[cnt] = sp->measBack[j].pr - (sp->measBack[j + 1].pr + avgDr * dt);
        drDiff[cnt] = sp->measBack[j + 1].dr - sp->measBack[j].dr;
        if (fabs(prDiff[cnt]) < 2000.0)
        {
          sign[cnt] = prDiff[cnt] > 0 ? 1 : -1;
          sumDiff += prDiff[cnt];
#if defined(PLAYBACK_MODE)
          SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s,%02d,%03d,%14.6f,%10.4f,%10.4f", __FUNCTION__, pSvMeas->gnssMode, pSvMeas->prn, sp->measBack[j].tor, prDiff[cnt], sp->measBack[j].dr);
#endif
          cnt++;
        }
        else
        {
          break;
        }
        if (j == 0 && sp->measBack[j].tot == sp->measBack[j + 1].tot)
        {
          pSvMeas->status &= 0xF8;
#if defined(PLAYBACK_MODE)
          SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s, Rej because tot unchange: %d %d", __FUNCTION__, pSvMeas->gnssMode, pSvMeas->prn);
#endif
          break;
        }
      }
      else
      {
        // if measurement break, reset reacq/pr error flag
        if (j == 0)
        {
          sp->isReAcq = FALSE;
          sp->prWithErr = FALSE;
          sp->updateSumDiff = FALSE;
          sp->prErrSum = 0;
          sp->goodPrCnt = 0;
          gnss_Qos_BadMeasFlag_Clear(sp, MEAS_TYPE_PR);
        }
        break;
      }
    }

    /* If no fix and only one epoch measurements, then don't use it */
    if (!firstFix && cnt == 0)
    {
      pSvMeas->status &= 0xF8;
      continue;
    }

    if (cnt > 0)
    {
      gnss_math_dstd(prDiff, cnt, &avgDiff, &stdDiff);
      if (stdDiff < 200 && stdDiff > 0.0)
      {
        stdDiffBack[stdDiffNum++] = stdDiff;
      }

      /*ReACQ detect for SPRD*/
      if (fabs(prDiff[0]) < 2000.0)
      {
        if (pSvMeas->cno > 35 && fabs(prDiff[0]) > 20.0)
        {
          isReAcq = TRUE;
        }
        else if (pSvMeas->cno > 30 && fabs(prDiff[0]) > 40.0)
        {
          isReAcq = TRUE;
        }
        else if (fabs(prDiff[0]) > 50.0)
        {
          isReAcq = TRUE;
        }
        if (isReAcq)
        {
          sp->isReAcq = TRUE;
          sp->torReAcq = pMeas->tor;
          sp->prWithErr = FALSE;
          sp->updateSumDiff = FALSE;
          sp->prErrSum = 0;
          sp->goodPrCnt = 0;
          gnss_Qos_BadMeasFlag_Clear(sp, MEAS_TYPE_PR);
          memset(&(sp->measBack[1]), 0, sizeof(MEAS_SAVE) * (MEAS_SAVE_NUM - 1));
          sp->measBackCnt = 1;
        }
      }
      /*
      *   Step1:
      *   Check PR-DR continuous for one epoch
      *   delta = [PR(k)-PR(k-1)] - DR(k)
      */

      if (fabs(prDiff[0]) > prDiffThres && fabs(prDiff[0]) < 2000.0)
      {
        if (fabs(prDiff[0]) <= 2.5 * prDiffThres)
        {
          pSvMeas->prWeightChck |= PRDR_CHECK;
          SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s, deweight : %14.6f,%02d,%02d,%10.4f", __FUNCTION__, pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, prDiff[0]);
        }
        else
        {
          pSvMeas->status &= 0xFE;
          SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s, Rej : %14.6f,%02d,%02d,%10.4f", __FUNCTION__, pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, prDiff[0]);
          continue;
        }
        gnss_Qos_BadMeasFlag_Clear(sp, MEAS_TYPE_PR);
      }
      /*
      *   Step2:
      *   Check PR-DR continuous sum
      *   delta(k) = [PR(k)-PR(k-1)] - DR(k)
      *   sum = sum(delta(k))
      */
      if (cnt >= 4)
      {
        gnss_math_dstd(prDiff, 4, &avgDiff, &stdDiff);
        pSvMeas->stdPRDRDiff = stdDiff;
        pSvMeas->sumPRDRDiff = sumDiff;

        gnss_math_dstd(drDiff, 4, &avgDiff, &stdDiff);
        pSvMeas->drDiffStd = stdDiff;
        if (firstFix)
        {
          if (pSvMeas->drDiffStd >= drDiffStdThres)
          {
            pSvMeas->status &= 0xFD;
          }
        }

#if defined(PLAYBACK_MODE)
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "PRDRSTD:%10.4f,%02d,%02d,%02d,%10.4f,%10.4f,%10.4f", pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->cno, pSvMeas->stdPRDRDiff, sumDiff, stdDiff);
#endif
        if (firstFix == FALSE)
        {
          if (pSvMeas->stdPRDRDiff > 10.0)
          {
            pSvMeas->status &= 0xFE;
            continue;
          }
        }
        else
        {
          if (peMode.staticData.staticFlag)
          {
            stdThres = 15.0;
          }
          else
          {
            stdThres = 20.0;
          }
          if (pSvMeas->stdPRDRDiff > stdThres)
          {
            pSvMeas->status &= 0xFE;
            continue;
          }
        }

        // check delta = PR5 - (PR0 + DR0 * 5)
        /*
        *  1, sumDiff > thres && sumDiff <= ratio*thres, deweight
        *  2, sumDiff > ratio*thres, reject
        *  3, sumDiff < thres && sumDiff > 20, all diff > 0, deweight
        */
        if (fabs(sumDiff) > sumThres && fabs(sumDiff) < 2000.0)
        {
          if (fabs(sumDiff) <= ratio * sumThres)
          {
            pSvMeas->prWeightChck |= PRDR_CHECK;
            SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s, deweight : %14.6f,%02d,%02d,%10.4f", __FUNCTION__, pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, sumDiff);
          }
          else
          {
            pSvMeas->status &= 0xFC;
            SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s, Rej : %14.6f,%02d,%02d,%10.4f", __FUNCTION__, pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, sumDiff);
          }

          if (pSvMeas->stdPRDRDiff < 5.0 && (pSvMeas->status & 0x2))
          {
#if defined(PLAYBACK_MODE)
            SYS_LOGGING(OBJ_QOS, LOG_INFO, "WYS:%14.6f,%02d,%02d,%10.4f,%10.4f,%10.4f,%10.4f,%d", pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, sumDiff, pSvMeas->pr_err, pSvMeas->dr_err, pSvMeas->dr_diff, peMode.staticData.staticFlag);
#endif
          }
        }
      }


      if (cnt >= 3)
      {
        gnss_Qos_PrError_Detect(pSvMeas, prDiff, drDiff, cnt, pMeas->tor, sp);
      }
    }
  }

  if (stdDiffNum > 0)
  {
    pMeas->prdrDiffStd_avg = gnssClcAvg_DBL(stdDiffBack, stdDiffNum);
  }
  else
  {
    pMeas->prdrDiffStd_avg = -1.0;
  }
#endif
}
/***********************************************************************
* 函数介绍: gnss_Qos_DR_Status_Check
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：11/16/
***********************************************************************/
void gnss_Qos_DR_Status_Check(double tor, uint8_t cnt, double* dr_diff, gnss_meas_t* pSvMeas)
{
  uint8_t             flag = FALSE;
  uint8_t             i;
  uint8_t             noUpdateCnt = 0;

  for (i = 0; i < cnt; i++)
  {
    if (fabs(dr_diff[i]) < 1e-5)
    {
      noUpdateCnt++;
    }
    else
    {
      break;
    }
  }
#ifdef USED_IN_MC262M
  pSvMeas->drNoUdCnt = noUpdateCnt;
#endif

  if (peMode.userSceData.isPDRMode)
  {
    if (noUpdateCnt >= 1)
    {
      flag = TRUE;
    }
  }
  else
  {
    if (noUpdateCnt >= 4 && fabs(pSvMeas->dr_diff) >= 1.5)
    {
      flag = TRUE;
    }
    else if (noUpdateCnt >= 3 && fabs(pSvMeas->dr_diff) >= 3.0)
    {
      flag = TRUE;
    }
    else if (cnt == noUpdateCnt)
    {
      flag = TRUE;
    }
  }

  if (flag && (pSvMeas->status & 0x2))
  {
    pSvMeas->status &= 0xFD;
    if (g_pe_cfg.chipType == UBLOX && noUpdateCnt == 5 && pSvMeas->cno < 25)
    {
      pSvMeas->status &= 0xFE;
    }
#if defined(PLAYBACK_MODE)
    SYS_LOGGING(OBJ_QOS, LOG_INFO, "DR not update: %10.4f,%02d,%03d,%10.4f", 
        tor, pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->dr_diff);
#endif
  }
}
/***********************************************************************
* 函数介绍: gnss_Qos_PRDR_Check_QCOM
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：11/16/
***********************************************************************/
void gnss_Qos_PRDR_Check_QCOM(meas_blk_t* pMeas)
{
  int8_t             sign[MEAS_SAVE_NUM] = { 0 };
  uint8_t             stdDiffNum = 0;
  uint32_t            i, j, cnt = 0/*, cntUse = 0, cntSign = 0*/;
  gnss_meas_t* pSvMeas;
  sat_data_t* sp;
  double            avgDr, dt;
  double            diff[MEAS_SAVE_NUM] = { 0.0 }, sumDiff, stdDiff = 0.0, avgDiff = 0.0;
  double            stdDiffBack[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 };
  double            sumThres, ratio = 3.0;
  double            drDiff[MEAS_SAVE_NUM], prDiffThres;

  sumThres = 40.0;
  ratio = 1.5;
  prDiffThres = 8.0;

  if (peState.posHasBias)
  {
    prDiffThres = 40.0;
    sumThres = 150;
  }

  /*1. Calculate the PRDR diff */
  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &pMeas->meas[i];
    if (pSvMeas->prn == 0) continue;
    if ((pSvMeas->status & 0x1) == 0) continue;
    sp = gnss_sd_get_sv_data(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);
    if (sp == NULL) continue;

    cnt = 0;
    sumDiff = 0.0;
    memset(diff, 0, sizeof(double) * MEAS_SAVE_NUM);
    memset(sign, 0, sizeof(int8_t) * MEAS_SAVE_NUM);
    memset(drDiff, 0, sizeof(double) * MEAS_SAVE_NUM);

    if (sp->isBadPr)
    {
      if ((pMeas->tor - sp->torBadPr) >= 3.5)
      {
        gnss_Qos_BadMeasFlag_Clear(sp, MEAS_TYPE_PR);
      }
    }

    for (j = 0; j < MEAS_SAVE_NUM - 1; j++)
    {
      dt = sp->measBack[j].tot - sp->measBack[j + 1].tot;
      if ((pSvMeas->gnssMode == GPS_MODE || pSvMeas->gnssMode == BDS_MODE || pSvMeas->gnssMode == GAL_MODE) && dt < (-SECS_IN_WEEK / 2.0))//cross week
      {
        dt += (double)SECS_IN_WEEK;
      }
      else if (pSvMeas->gnssMode == GLN_MODE && dt < (-SECS_IN_DAY / 2.0))
      {
        dt += (double)SECS_IN_DAY;
      }
      if (fabs(dt) < 1.5 && sp->measBack[j].pr > 0.0 && sp->measBack[j + 1].pr > 0.0)
      {
        avgDr = 0.5 * (sp->measBack[j].dr + sp->measBack[j + 1].dr);
        diff[cnt] = sp->measBack[j].pr - (sp->measBack[j + 1].pr + avgDr * dt);
        drDiff[cnt] = sp->measBack[j + 1].dr - sp->measBack[j].dr;

        if (fabs(diff[cnt]) < 2000.0)
        {
          sign[cnt] = diff[cnt] > 0 ? 1 : -1;
          sumDiff += diff[cnt];
#if defined(PLAYBACK_MODE)
          SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s,%02d,%03d,%14.6f,%10.4f,%10.4f,%10.4f", __FUNCTION__, pSvMeas->gnssMode, pSvMeas->prn, sp->measBack[j].tor, diff[cnt], sp->measBack[j].dr, drDiff[cnt]);
#endif
          cnt++;
        }
        else
        {
          break;
        }
        if (j == 0 && sp->measBack[j].tot == sp->measBack[j + 1].tot)
        {
          pSvMeas->status &= 0xF8;
#if defined(PLAYBACK_MODE)
          SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s, Rej because tot unchange: %d %d", __FUNCTION__, pSvMeas->gnssMode, pSvMeas->prn);
#endif
          break;
        }
      }
      else
      {
        if (j == 0)
        {
          gnss_Qos_BadMeasFlag_Clear(sp, MEAS_TYPE_PR);
        }
        break;
      }
    }

    /* If no fix and only one epoch measurements, then don't use it */
    if (!firstFix && sp->measBackCnt < 2)
    {
      pSvMeas->status &= 0xF8;
      continue;
    }

    if (cnt > 0)
    {
      gnss_math_dstd(diff, cnt, &avgDiff, &stdDiff);
      if (stdDiff < 200 && stdDiff > 0.0)
      {
        stdDiffBack[stdDiffNum++] = stdDiff;
      }
      if (cnt >= 4)
      {
        pSvMeas->stdPRDRDiff = stdDiff;
        pSvMeas->sumPRDRDiff = sumDiff;
      }

      /*
      *   Step1:
      *   Check PR-DR continuous for one epoch
      *   delta = [PR(k)-PR(k-1)] - DR(k)
      */
#if 0
      if (pSvMeas->cno < 18)
      {
        prDiffThres = 40.0;
      }
      else if (pSvMeas->cno < 25)
      {
        prDiffThres = 35.0;
      }
      else if (pSvMeas->cno < 30)
      {
        prDiffThres = 25.0;
      }
      else
      {
        prDiffThres = 20.0;
      }
#endif
      if (fabs(diff[0]) > prDiffThres && fabs(diff[0]) < 2000.0)
      {
        if (fabs(diff[0]) <= 1.5 * prDiffThres)
        {
          pSvMeas->prWeightChck |= PRDR_CHECK;
          SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s, deweight : %14.6f,%02d,%02d,%10.4f", __FUNCTION__, pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, diff[0]);
        }
        else
        {
          if (peMode.userSceData.isPDRMode && fabs(pSvMeas->pr_diff) > 10.0)
          {
            pSvMeas->status &= 0xFC;
            SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s, Rej : %14.6f,%02d,%02d,%10.4f", __FUNCTION__, pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, diff[0]);
            continue;
          }
          else
          {
            pSvMeas->status &= 0xFE;
            if (pSvMeas->stdPRDRDiff > 15.0 || pSvMeas->sumPRDRDiff > 80.0)
            {
              pSvMeas->status &= 0xFD;
              SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s, DRRej : %14.6f,%02d,%02d,%10.4f", __FUNCTION__, pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, drDiff[0]);
            }
            SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s, Rej : %14.6f,%02d,%02d,%10.4f", __FUNCTION__, pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, diff[0]);
            continue;
          }
        }
        gnss_Qos_BadMeasFlag_Clear(sp, MEAS_TYPE_PR);
      }
      /*
      *   Step2:
      *   Check PR-DR continuous sum
      *   delta(k) = [PR(k)-PR(k-1)] - DR(k)
      *   sum = sum(delta(k))
      */
      if (cnt >= 4)
      {
        gnss_math_dstd(diff, 4, &avgDiff, &stdDiff);
        if (sp->prRCnt < 3)
        {
          sp->prRCnt++;
        }
        sp->prR = (sp->prRCnt - 1) * sp->prR / sp->prRCnt + (stdDiff * stdDiff) / sp->prRCnt;

        gnss_math_dstd(drDiff, 4, &avgDiff, &stdDiff);
        pSvMeas->drDiffStd = stdDiff;

#if defined(PLAYBACK_MODE)
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "PRDRSTD:%10.4f,%02d,%02d,%02d,%10.4f,%10.4f,%10.4f", pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->cno, pSvMeas->stdPRDRDiff, sumDiff, stdDiff);
#endif
        if (firstFix == FALSE && peMode.userSceData.isUnderEleRoad == FALSE)
        {
          if (pSvMeas->stdPRDRDiff > 15.0)
          {
            pSvMeas->status &= 0xFE;
            SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s, Rej : %14.6f,%02d,%02d by stdPRDRDiff", __FUNCTION__, pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn);
            continue;
          }
        }
        else if (peState.posHasBias && peMode.userSceData.isUnderEleRoad == FALSE)
        {
          if (pSvMeas->stdPRDRDiff > 50.0)
          {
            pSvMeas->status &= 0xFE;
            continue;
          }
        }

        // check delta = PR5 - (PR0 + DR0 * 5)
        /*
        *  1, sumDiff > thres && sumDiff <= ratio*thres, deweight
        *  2, sumDiff > ratio*thres, reject
        *  3, sumDiff < thres && sumDiff > 20, all diff > 0, deweight
        */
        if (fabs(sumDiff) > sumThres && fabs(sumDiff) < 2000.0)
        {
          if (fabs(sumDiff) <= ratio * sumThres)
          {
            pSvMeas->prWeightChck |= PRDR_CHECK;
            SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s, deweight : %14.6f,%02d,%02d,%10.4f", __FUNCTION__, pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, sumDiff);
          }
          else
          {
            if (peMode.userSceData.isPDRMode)
            {
              pSvMeas->status &= 0xFC;
            }
            else
            {
              pSvMeas->status &= 0xFE;
            }
          }
        }
      }

      if (cnt >= 3)
      {
        gnss_Qos_DR_Status_Check(pMeas->tor, cnt, drDiff, pSvMeas);
      }
    }
  }

  if (stdDiffNum > 0)
  {
    pMeas->prdrDiffStd_avg = gnssClcAvg_DBL(stdDiffBack, stdDiffNum);
  }
  else
  {
    pMeas->prdrDiffStd_avg = -1.0;
  }
}
/***********************************************************************
* 函数介绍: gnss_Qos_PRDR_Check_UBLOX
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：11/16/
***********************************************************************/
void gnss_Qos_PRDR_Check_UBLOX(meas_blk_t* pMeas)
{
  int8_t             sign[MEAS_SAVE_NUM] = { 0 };
  uint8_t             stdDiffNum = 0;
  uint32_t            i, j, cnt = 0/*, cntUse = 0, cntSign = 0*/, cnt_list[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 }, cnt12 = 0, firstCnt = 0, cnoHighThres1 = 0, cnoHighThres2 = 0;
  gnss_meas_t* pSvMeas;
  sat_data_t* sp;
  double            avgDr, dt;
  double            diff[MEAS_SAVE_NUM] = { 0.0 }, sumDiff, stdDiff = 0.0, avgDiff = 0.0, first_stdDiff = 0.0, first_avgDiff = 0.0;
  double            stdDiffBack[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, firstDiff[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 };
  double            sumThres, ratio = 3.0;
  double            drDiff[MEAS_SAVE_NUM], prDiffThres, drDiffThres, drDiffStdThres;
  double            sum_cnt4 = 0.0, sum_all = 0.0;


  sumThres = 40.0;
  ratio = 2.0;
  prDiffThres = 20.0;
  drDiffThres = 8.0;
  drDiffStdThres = 5.0;
  cnoHighThres1 = 45;
  cnoHighThres2 = 45;
  if (peState.posHasBias)
  {
    prDiffThres = 40.0;
    sumThres = 150;
  }

  /*1. Calculate the PRDR diff */
  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &pMeas->meas[i];
    if (pSvMeas->prn == 0) continue;
    if ((pSvMeas->status & 0x1) == 0) continue;
    sp = gnss_sd_get_sv_data(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);
    if (sp == NULL) continue;

    cnt = 0;
    sumDiff = 0.0;
    memset(diff, 0, sizeof(double) * MEAS_SAVE_NUM);
    memset(sign, 0, sizeof(int8_t) * MEAS_SAVE_NUM);
    memset(drDiff, 0, sizeof(double) * MEAS_SAVE_NUM);

    if (sp->isBadPr)
    {
      if ((pMeas->tor - sp->torBadPr) >= 3.5)
      {
        gnss_Qos_BadMeasFlag_Clear(sp, MEAS_TYPE_PR);
      }
    }

    for (j = 0; j < MEAS_SAVE_NUM - 1; j++)
    {
      dt = sp->measBack[j].tot - sp->measBack[j + 1].tot;
      if (!IS_PPK_REVERSE)
      {
        if ((pSvMeas->gnssMode == GPS_MODE || pSvMeas->gnssMode == BDS_MODE || pSvMeas->gnssMode == GAL_MODE) && dt < (-SECS_IN_WEEK / 2.0))//cross week
        {
          dt += (double)SECS_IN_WEEK;
        }
        else if (pSvMeas->gnssMode == GLN_MODE && dt < (-SECS_IN_DAY / 2.0))
        {
          dt += (double)SECS_IN_DAY;
        }
      }
      else
      {
        if ((pSvMeas->gnssMode == GPS_MODE || pSvMeas->gnssMode == BDS_MODE || pSvMeas->gnssMode == GAL_MODE) && dt > SECS_IN_WEEK / 2.0)//cross week
        {
          dt -= (double)SECS_IN_WEEK;
        }
        else if (pSvMeas->gnssMode == GLN_MODE && dt > SECS_IN_DAY / 2.0)
        {
          dt -= (double)SECS_IN_DAY;
        }
      }

      if (fabs(dt) < 1.5 && sp->measBack[j].pr > 0.0 && sp->measBack[j + 1].pr > 0.0 &&
        fabs(sp->measBack[j].dr) > 1e-3 && fabs(sp->measBack[j + 1].dr) > 1e-3)
      {
        avgDr = 0.5 * (sp->measBack[j].dr + sp->measBack[j + 1].dr + sp->measBack[j].driftAdj);
        if (fabs(avgDr) < 1e-6) break;
        diff[cnt] = sp->measBack[j].pr - sp->measBack[j].biasAdjMs * LIGHT_MSEC - (sp->measBack[j + 1].pr + avgDr * dt);
        drDiff[cnt] = sp->measBack[j + 1].dr - (sp->measBack[j].dr - sp->measBack[j].driftAdj);

        if (fabs(diff[cnt]) < 2000.0)
        {
          sign[cnt] = diff[cnt] > 0 ? 1 : -1;
          sumDiff += diff[cnt];
#if defined(PLAYBACK_MODE) && defined(PE_DEBUG_LOG)
          //SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s,%02d,%03d,%14.6f,%10.4f,%10.4f,%10.4f", __FUNCTION__, pSvMeas->gnssMode, pSvMeas->prn, sp->measBack[j].tor, diff[cnt], sp->measBack[j].dr, drDiff[cnt]);
#endif
          cnt++;
        }
        else
        {
          break;
        }
        if (j == 0 && sp->measBack[j].tot == sp->measBack[j + 1].tot)
        {
          pSvMeas->status &= 0xF8;
#if defined(PLAYBACK_MODE)
          //SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s, Rej because tot unchange: %d %d", __FUNCTION__, pSvMeas->gnssMode, pSvMeas->prn);
#endif
          break;
        }
      }
      else
      {
        if (j == 0)
        {
          gnss_Qos_BadMeasFlag_Clear(sp, MEAS_TYPE_PR);
        }
        break;
      }
    }

    /* If no fix and only one epoch measurements, then don't use it */
    if (!firstFix && sp->measBackCnt < 2)
    {
      pSvMeas->status &= 0xF8;
      continue;
    }
    else if (firstFix && cnt == 0 && sp->measBackCnt >= 2 && peMode.userSceData.isUnderEleRoad)
    {
      if (fabs(pSvMeas->dr_diff) > 8.0)
      {
        pSvMeas->status &= 0xFD;
      }
    }

    if (cnt >= 4) sum_cnt4++;
    sum_all++;
    cnt_list[i] = cnt;

    if (cnt > 0)
    {
      if (!firstFix && cnt == 1 && DBL_IS_EQUAL(pSvMeas->pr_diff, 0.0) && pSvMeas->cno >= cnoHighThres1)
      {
        firstDiff[firstCnt] = diff[cnt - 1];
        pSvMeas->first_diff = (float)firstDiff[firstCnt];
        firstCnt++;
      }
      gnss_math_dstd(diff, cnt, &avgDiff, &stdDiff);
      if (stdDiff < 200 && stdDiff > 0.0)
      {
        stdDiffBack[stdDiffNum++] = stdDiff;
      }
      pSvMeas->sumPRDRDiff = sumDiff;
      if (cnt >= 4)
      {
        pSvMeas->stdPRDRDiff = stdDiff;

      }

      /*
      *   Step1:
      *   Check PR-DR continuous for one epoch
      *   delta = [PR(k)-PR(k-1)] - DR(k)
      */
#if 0
      if (pSvMeas->cno < 18)
      {
        prDiffThres = 40.0;
      }
      else if (pSvMeas->cno < 25)
      {
        prDiffThres = 35.0;
      }
      else if (pSvMeas->cno < 30)
      {
        prDiffThres = 25.0;
      }
      else
      {
        prDiffThres = 20.0;
      }
#endif
      if (fabs(diff[0]) > prDiffThres && fabs(diff[0]) < 2000.0)
      {
        if (fabs(diff[0]) <= 2.5 * prDiffThres)
        {
          pSvMeas->prWeightChck |= PRDR_CHECK;
          //SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s, deweight : %14.6f,%02d,%02d,%10.4f", __FUNCTION__, pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, diff[0]);
        }
        else
        {
          if (peMode.userSceData.isPDRMode && fabs(pSvMeas->pr_diff) > 10.0)
          {
            pSvMeas->status &= 0xFC;
            //SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s, Rej : %14.6f,%02d,%02d,%10.4f", __FUNCTION__, pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, diff[0]);
            continue;
          }
          else
          {
            pSvMeas->status &= 0xFE;
            //SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s, Rej : %14.6f,%02d,%02d,%10.4f", __FUNCTION__, pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, diff[0]);
            //continue;
          }
        }
        gnss_Qos_BadMeasFlag_Clear(sp, MEAS_TYPE_PR);
      }

      if (fabs(drDiff[0]) > drDiffThres)
      {
        pSvMeas->status &= 0xFD;
        //SYS_LOGGING(OBJ_QOS, LOG_INFO, "Reject DR:%10.4f,%02d,%02d,%10.4f", pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->dr_diff);

      }

#if 0
      if (peMode.userSceData.isUnderEleRoad && fabs(drDiff[0]) > drDiffThres)
      {
        pSvMeas->status &= 0xFD;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s, Rej DR: %14.6f,%02d,%02d,%10.4f", __FUNCTION__, pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, drDiff[0]);
      }
#endif
      /*
      *   Step2:
      *   Check PR-DR continuous sum
      *   delta(k) = [PR(k)-PR(k-1)] - DR(k)
      *   sum = sum(delta(k))
      */
      if (cnt >= 4)
      {
        gnss_math_dstd(diff, 4, &avgDiff, &stdDiff);
        if (sp->prRCnt < 3)
        {
          sp->prRCnt++;
        }
        sp->prR = (sp->prRCnt - 1) * sp->prR / sp->prRCnt + (stdDiff * stdDiff) / sp->prRCnt;

        gnss_math_dstd(drDiff, 4, &avgDiff, &stdDiff);
        pSvMeas->drDiffStd = stdDiff;

        if (firstFix && (pSvMeas->status & 0x2))
        {
          if (pSvMeas->drDiffStd >= 2 * drDiffStdThres && ((pSvMeas->quality & DR_GOOD) == FALSE) && fabs(pSvMeas->dr_diff) > 1.0)
          {
            pSvMeas->status &= 0xFD;
            //SYS_LOGGING(OBJ_QOS, LOG_INFO, "Reject DR:%10.4f,%02d,%02d,%10.4f", pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->drDiffStd);
          }
          else if (fabs(pSvMeas->sumPRDRDiff / cnt) > 3.0 && fabs(diff[0]) > 4.0 && fabs(drDiff[0]) > 3.0)
          {
            pSvMeas->status &= 0xFD;
            //SYS_LOGGING(OBJ_QOS, LOG_INFO, "Reject DR:%10.4f,%02d,%02d,%10.4f", pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->drDiffStd);
          }
          else if (pSvMeas->drDiffStd >= drDiffStdThres && (pSvMeas->quality & DR_GOOD) == FALSE)
          {
            pSvMeas->drWeightChck |= PRDR_CHECK;
            // SYS_LOGGING(OBJ_QOS, LOG_INFO, "Deweight DR:%10.4f,%02d,%02d,%10.4f", pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->drDiffStd);
          }
        }

#if defined(PLAYBACK_MODE) && defined(PE_DEBUG_LOG)
        //SYS_LOGGING(OBJ_QOS, LOG_INFO, "PRDRSTD:%10.4f,%02d,%02d,%02d,%10.4f,%10.4f,%10.4f", pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->cno, pSvMeas->stdPRDRDiff, sumDiff, stdDiff);
#endif
        if (firstFix == FALSE && peMode.userSceData.isUnderEleRoad == FALSE)
        {
          if (pSvMeas->stdPRDRDiff > 30.0)
          {
            pSvMeas->status &= 0xFE;
            continue;
          }
        }
        else if (peState.posHasBias && peMode.userSceData.isUnderEleRoad == FALSE)
        {
          if (pSvMeas->stdPRDRDiff > 50.0)
          {
            pSvMeas->status &= 0xFE;
            continue;
          }
        }

        // check delta = PR5 - (PR0 + DR0 * 5)
        /*
        *  1, sumDiff > thres && sumDiff <= ratio*thres, deweight
        *  2, sumDiff > ratio*thres, reject
        *  3, sumDiff < thres && sumDiff > 20, all diff > 0, deweight
        */
        if (fabs(sumDiff) > sumThres && fabs(sumDiff) < 2000.0 && fabs(pSvMeas->pr_diff) > 20.0)
        {
          if (fabs(sumDiff) <= ratio * sumThres)
          {
            pSvMeas->prWeightChck |= PRDR_CHECK;
            SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s, deweight : %14.6f,%02d,%02d,%10.4f", __FUNCTION__, pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, sumDiff);
          }
          else
          {
            if (peMode.userSceData.isPDRMode)
            {
              pSvMeas->status &= 0xFC;
            }
            else
            {
              pSvMeas->status &= 0xFE;
            }
#if defined(PLAYBACK_MODE)
            //SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s, Rej : %14.6f,%02d,%02d,%10.4f,%6.2f", __FUNCTION__, pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, sumDiff, pSvMeas->dr_err);
#endif
          }
        }
        /*add pr/dr rejected  strategies in low cno(avgCno < 20) & static & UnderEleRoad scene */
        if (!peMode.staticData.staticFlag && peMode.userSceData.isUnderEleRoad)
        {
          if (pSvMeas->stdPRDRDiff > 10.0 && pSvMeas->stdPRDRDiff < 50 && fabs(pSvMeas->pr_diff) > 80.0 && pMeas->avgCno >= 25)
          {
            pSvMeas->status &= 0xFC;
#if defined(PLAYBACK_MODE)
            //SYS_LOGGING(OBJ_QOS, LOG_INFO, "Reject PRDR large diff:%10.4f,%02d,%02d,%10.4f,%d,%10.4f,%10.4f,%10.4f", pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->stdPRDRDiff, pSvMeas->cno, pSvMeas->pr_diff, pSvMeas->dr_diff, pSvMeas->drDiffStd);
#endif
          }
          if (pSvMeas->status & 0x2)//DR vaild
          {
            if (pSvMeas->stdPRDRDiff > 1.5 && fabs(pSvMeas->pr_diff) > 20.0 && pSvMeas->cno <= pMeas->avgCno && pMeas->avgCno < 20)
            {
              pSvMeas->status &= 0xFD;
#if defined(PLAYBACK_MODE)
              //SYS_LOGGING(OBJ_QOS, LOG_INFO, "Reject DR low cno:%10.4f,%02d,%02d,%10.4f,%d,%10.4f,%10.4f,%10.4f", pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->stdPRDRDiff, pSvMeas->cno, pSvMeas->pr_diff, pSvMeas->dr_diff, pSvMeas->drDiffStd);
#endif
            }
            else if (pSvMeas->stdPRDRDiff > 4.0 && fabs(pSvMeas->pr_diff) > 15.0 && pSvMeas->cno < 25 && pMeas->avgCno <= 20)
            {
              if ((pSvMeas->cno <= 20) || fabs(pSvMeas->pr_diff) > 25.0)
              {
                pSvMeas->status &= 0xFD;
#if defined(PLAYBACK_MODE)
                //SYS_LOGGING(OBJ_QOS, LOG_INFO, "Reject DR low cno:%10.4f,%02d,%02d,%10.4f,%d,%10.4f,%10.4f,%10.4f", pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->stdPRDRDiff, pSvMeas->cno, pSvMeas->pr_diff, pSvMeas->dr_diff, pSvMeas->drDiffStd);
#endif
              }
            }
          }
          if (pSvMeas->status & 0x1)//PR valid
          {
            if (pSvMeas->stdPRDRDiff > 3.0 && fabs(pSvMeas->pr_diff) > 40.0 && pSvMeas->cno < 25 && pSvMeas->cno > 15 && pMeas->avgCno <= 20 && pMeas->Cno20Cnt > 10)
            {
              pSvMeas->status &= 0xFE;
#if defined(PLAYBACK_MODE)
              // SYS_LOGGING(OBJ_QOS, LOG_INFO, "Reject PR low cno:%10.4f,%02d,%02d,%10.4f,%d,%10.4f,%10.4f,%10.4f,%d,%d", pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->stdPRDRDiff, pSvMeas->cno, pSvMeas->pr_diff, pSvMeas->dr_diff, pSvMeas->drDiffStd, pMeas->Cno20Cnt, pMeas->avgCno);
#endif
            }
            else if (pSvMeas->stdPRDRDiff > 3.0 && fabs(pSvMeas->pr_diff) > 40.0 && pSvMeas->cno <= pMeas->avgCno && pMeas->avgCno <= 15)
            {
              pSvMeas->status &= 0xFE;
#if defined(PLAYBACK_MODE)
              //SYS_LOGGING(OBJ_QOS, LOG_INFO, "Reject PR low cno:%10.4f,%02d,%02d,%10.4f,%d,%10.4f,%10.4f,%10.4f,%d,%d", pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->stdPRDRDiff, pSvMeas->cno, pSvMeas->pr_diff, pSvMeas->dr_diff, pSvMeas->drDiffStd, pMeas->Cno20Cnt, pMeas->avgCno);
#endif
            }
          }
        }
      }

      //if (cnt >= 1)
      {
        gnss_Qos_DR_Status_Check(pMeas->tor, cnt, drDiff, pSvMeas);
      }

      if (cnt == 5 && peMode.userSceData.isUnderEleRoad == 1)
      {
        cnt12 = 0;
        for (j = 0; j < MEAS_SAVE_NUM - 1; j++)
        {
          if (fabs(diff[j]) > 12) cnt12++;
        }
        if (cnt12 >= 2)
        {
          pSvMeas->status &= 0xFE;
#if defined(PLAYBACK_MODE)
          //SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s, Rej PR For Large diff : %14.6f,%02d,%02d,%10.4f,%10.4f", __FUNCTION__, pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->pr_diff, pSvMeas->pr_err);
#endif
        }
      }
    }
  }
  if (!firstFix && firstCnt > 5)
  {
    gnss_math_dstd(firstDiff, firstCnt, &first_avgDiff, &first_stdDiff);
    for (i = 0; i < pMeas->measCnt; i++)
    {
      pSvMeas = &pMeas->meas[i];
      if (pSvMeas->prn == 0) continue;
      if ((pSvMeas->status & 0x1) == 0) continue;
      if (FLT_IS_EQUAL(pSvMeas->first_diff, 0.0f)) continue;
      if (pSvMeas->cno < cnoHighThres1) continue;
      sp = gnss_sd_get_sv_data(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);
      if (sp == NULL) continue;
      if (((fabs(pSvMeas->first_diff) > (float)(fabs(first_avgDiff) + 3.0 * first_stdDiff) && fabs(pSvMeas->first_diff) > (float)(2.0 * fabs(first_avgDiff)) && pSvMeas->cno > cnoHighThres1) ||
        (fabs(pSvMeas->first_diff) > (float)(3.0 * first_stdDiff) && fabs(pSvMeas->first_diff) > (float)(2.0 * fabs(first_avgDiff)) && pSvMeas->cno > cnoHighThres2)) && pSvMeas->cycleSlipCount == 1)
      {
        pSvMeas->status &= 0xFE;
#if defined(PLAYBACK_MODE)
        //SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s, Rej PR For Large Diff: %14.6f,%02d,%02d,%10.4f,%10.4f", __FUNCTION__, pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->pr_diff, pSvMeas->pr_err);
#endif
      }
      else if (fabs(pSvMeas->first_diff) > (float)(1.5 * first_stdDiff) && fabs(pSvMeas->first_diff) > (float)(1.5 * fabs(first_avgDiff)) &&
        pSvMeas->cycleSlipCount == 1 && pSvMeas->cno > cnoHighThres2)
      {
        pSvMeas->prWeightChck |= PRDR_CHECK;
#if defined(PLAYBACK_MODE)
        //SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s, Deweight PR : %14.6f,%02d,%02d,%10.4f,%10.4f", __FUNCTION__, pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->pr_diff, pSvMeas->pr_err);
#endif
      }
    }
  }
  if (sum_all > 8)
  {
    if (sum_cnt4 / sum_all > 0.8)
    {
      for (i = 0; i < pMeas->measCnt; i++)
      {
        pSvMeas = &pMeas->meas[i];
        if (pSvMeas->prn == 0) continue;
        if ((pSvMeas->status & 0x1) == 0) continue;
        sp = gnss_sd_get_sv_data(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);
        if (sp == NULL) continue;

        if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100)
        {
          if (fabs(pSvMeas->cno) > 40 && cnt_list[i] > 2) continue;
        }

        if (cnt_list[i] < 4)
        {
          pSvMeas->status &= 0xF8;
#if defined(PLAYBACK_MODE)
          //SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s, Rej PR For Meas BreakUp : %14.6f,%02d,%02d,%10.4f,%10.4f", __FUNCTION__, pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->pr_diff, pSvMeas->pr_err);
#endif
        }
      }
    }
  }

  if (stdDiffNum > 0)
  {
    pMeas->prdrDiffStd_avg = gnssClcAvg_DBL(stdDiffBack, stdDiffNum);
  }
  else
  {
    pMeas->prdrDiffStd_avg = -1.0;
  }
}

/***********************************************************************
* 函数介绍: gnss_Qos_Dual_PRDR_Check_UBLOX
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：08/14/
***********************************************************************/
void gnss_Qos_Dual_PRDR_Check(meas_blk_t* pMeas)
{
  uint32_t 		   i, j, dual_freq_cnt = 0, dual_freq_diff10_cnt = 0;
 // gnss_meas_t* pSvMeas;
  int 		   dual_freq_flag;
  float 		   dual_pr_diff, first_threshold = 100.0, avg_dual_freq_diff = 0.0, first_diff_thres = 5.0, reject_prthres1 = 30.0;
  GNSS_TIME* pTime;
  uint32_t 			idx = 0;

  pTime = gnss_tm_get_time();
  //cal avg dual_pr_diff
  if (!firstFix && g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100)
  {
    for (i = 0; i < pMeas->measCnt; i++)
    {
      dual_freq_flag = 0;
      if ((pMeas->meas[i].status & 0x3) != 0x3) continue;
      for (j = 0; j < pMeas->measCnt && j < MAX_MEAS_NUM; j++)
      {
        if (i == j) continue;
        if ((pMeas->meas[i].prn == pMeas->meas[j].prn) && (pMeas->meas[i].gnssMode == pMeas->meas[j].gnssMode) && (pMeas->meas[j].status & 0x3) == 0x3)
        {
          dual_freq_flag = 1;
          break;
        }
      }
      if (dual_freq_flag <= 0) continue;
      if ((pMeas->meas[j].drWeightChck & DUALMEAS_CHECK) == DUALMEAS_CHECK || (pMeas->meas[j].prWeightChck & DUALMEAS_CHECK) == DUALMEAS_CHECK) continue;
      if (pMeas->meas[i].freq_index == 0)
      {
        dual_pr_diff = (float)(pMeas->meas[i].pseudoRange - pMeas->meas[j].pseudoRange - pTime->dcb[pMeas->meas[j].gnssMode + (pMeas->meas[j].freq_index - 1) * GNSS_MAX_MODE]);
      }
      else
      {
        dual_pr_diff = (float)(pMeas->meas[j].pseudoRange - pMeas->meas[i].pseudoRange - pTime->dcb[pMeas->meas[i].gnssMode + (pMeas->meas[i].freq_index - 1) * GNSS_MAX_MODE]);
      }
      dual_freq_cnt++;
      if (fabs(dual_pr_diff) > 10.0)
      {
        dual_freq_diff10_cnt++;
      }
      avg_dual_freq_diff += (float)fabs(dual_pr_diff);
    }
    if (dual_freq_cnt > 0)
    {
      avg_dual_freq_diff /= dual_freq_cnt;
    }
  }
  //calculate dual_PR_diff/dual_DR_diff to check the consistency
  for (i = 0; i < pMeas->measCnt; i++)
  {
    dual_freq_flag = 0;
    if ((pMeas->meas[i].status & 0x3) != 0x3) continue;
    if (firstFix)
    {
      if (pMeas->meas[i].prn == 0) continue;
      //if (pMeas->meas[i].freq_index > 0) continue;
      if ((pMeas->meas[i].drWeightChck & DUALMEAS_CHECK) == DUALMEAS_CHECK || (pMeas->meas[i].prWeightChck & DUALMEAS_CHECK) == DUALMEAS_CHECK) continue;
      if ((fabs(pMeas->meas[i].dr_diff) <= 0) || (fabs(pMeas->meas[i].pr_diff) <= 0)) continue;
    }
    for (j = 0; j < pMeas->measCnt && j < MAX_MEAS_NUM; j++)
    {
      if (i == j) continue;
      if ((pMeas->meas[i].prn == pMeas->meas[j].prn) && (pMeas->meas[i].gnssMode == pMeas->meas[j].gnssMode) && (pMeas->meas[j].status & 0x3) == 0x3)
      {
        dual_freq_flag = 1;
        break;
      }
    }
    if (dual_freq_flag <= 0) continue;
    if ((pMeas->meas[j].drWeightChck & DUALMEAS_CHECK) == DUALMEAS_CHECK || (pMeas->meas[j].prWeightChck & DUALMEAS_CHECK) == DUALMEAS_CHECK) continue;

    if (!firstFix && (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_UBLOX_F9 || g_pe_cfg.sub_chipType == SUB_CHIPTYPE_BRCM_47755 || g_pe_cfg.sub_chipType == SUB_CHIPTYPE_HD9310 || g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100))
    {
      if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_UBLOX_F9 || g_pe_cfg.sub_chipType == SUB_CHIPTYPE_HD9310) first_threshold = 20.0;
      if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_BRCM_47755 || g_pe_cfg.sub_chipType == SUB_CHIPTYPE_QCOM_855) first_threshold = 10.0;
      if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100)
      {
        if (avg_dual_freq_diff < 20 && (1.0 * dual_freq_diff10_cnt > 0.5 * dual_freq_cnt))
        {
          first_threshold = 10.0;
          first_diff_thres = 5.0;
        }
        else
        {
          first_threshold = 15.0;
          first_diff_thres = 7.5;
        }
      }
      if (pMeas->meas[i].freq_index == 0)
      {
        dual_pr_diff = (float)(pMeas->meas[i].pseudoRange - pMeas->meas[j].pseudoRange - pTime->dcb[pMeas->meas[j].gnssMode + (pMeas->meas[j].freq_index - 1) * GNSS_MAX_MODE]);
      }
      else
      {
        dual_pr_diff = (float)(pMeas->meas[j].pseudoRange - pMeas->meas[i].pseudoRange - pTime->dcb[pMeas->meas[i].gnssMode + (pMeas->meas[i].freq_index - 1) * GNSS_MAX_MODE]);
      }
      if (fabs(dual_pr_diff) > 60)
      {
        if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100)
        {
          pMeas->meas[i].status &= 0xFE;
          pMeas->meas[j].status &= 0xFE;
          SYS_LOGGING(OBJ_QOS, LOG_INFO, "FIRSTFIX reject PR %02d,%03d,%02d,%02d", pMeas->meas[i].gnssMode, pMeas->meas[i].prn, pMeas->meas[i].freq_index, pMeas->meas[i].cno);
          SYS_LOGGING(OBJ_QOS, LOG_INFO, "FIRSTFIX reject PR %02d,%03d,%02d,%02d", pMeas->meas[j].gnssMode, pMeas->meas[j].prn, pMeas->meas[j].freq_index, pMeas->meas[j].cno);
        }
        else
        {
          if (pMeas->meas[i].freq_index < 2)
          {
            pMeas->meas[i].status &= 0xF0;
            SYS_LOGGING(OBJ_QOS, LOG_INFO, "FIRSTFIX reject PR %02d,%03d,%02d,%02d", pMeas->meas[i].gnssMode, pMeas->meas[i].prn, pMeas->meas[i].freq_index, pMeas->meas[i].cno);
          }
          if (pMeas->meas[j].freq_index < 2)
          {
            pMeas->meas[j].status &= 0xF0;
            SYS_LOGGING(OBJ_QOS, LOG_INFO, "FIRSTFIX reject PR %02d,%03d,%02d,%02d", pMeas->meas[j].gnssMode, pMeas->meas[j].prn, pMeas->meas[j].freq_index, pMeas->meas[j].cno);
          }
        }
      }
      else if (fabs(dual_pr_diff) > first_threshold)
      {
        if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100)
        {
          if ((FLT_IS_EQUAL(pMeas->meas[i].pr_diff, 0.0f) && FLT_IS_EQUAL(pMeas->meas[i].pr_diff, 0.0f)) || fabs(dual_pr_diff) > 30)
          {
            pMeas->meas[i].status &= 0xFE;
            pMeas->meas[j].status &= 0xFE;
#if defined(PLAYBACK_MODE)
            SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s reject PR for FirstFix %02d,%03d,%02d,%10.4f", __FUNCTION__, pMeas->meas[i].gnssMode, pMeas->meas[i].prn, pMeas->meas[i].freq_index, pMeas->meas[i].pr_diff);
            SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s reject PR for FirstFix %02d,%03d,%02d,%10.4f", __FUNCTION__, pMeas->meas[j].gnssMode, pMeas->meas[j].prn, pMeas->meas[j].freq_index, pMeas->meas[j].pr_diff);
#endif
          }
          else
          {
            if (fabs(pMeas->meas[j].pr_diff) > first_diff_thres)
            {
              pMeas->meas[j].status &= 0xFE;
#if defined(PLAYBACK_MODE)
              SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s reject PR for FirstFix %02d,%03d,%02d,%10.4f", __FUNCTION__, pMeas->meas[j].gnssMode, pMeas->meas[j].prn, pMeas->meas[j].freq_index, pMeas->meas[j].pr_diff);
#endif
            }
            if (fabs(pMeas->meas[i].pr_diff) > first_diff_thres)
            {
              pMeas->meas[i].status &= 0xFE;
#if defined(PLAYBACK_MODE)
              SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s reject PR for FirstFix %02d,%03d,%02d,%10.4f", __FUNCTION__, pMeas->meas[i].gnssMode, pMeas->meas[i].prn, pMeas->meas[i].freq_index, pMeas->meas[i].pr_diff);
#endif
            }
          }
        }
        else
        {
          if ((pMeas->meas[i].cno - pMeas->meas[j].cno) * (pMeas->meas[i].cno - pMeas->meas[j].cno) > 4)
          {
            if (pMeas->meas[i].cno < pMeas->meas[j].cno)
            {
              idx = i;
            }
            else if (pMeas->meas[i].cno > pMeas->meas[j].cno)
            {
              idx = j;
            }
          }
          else
          {
            if (pMeas->meas[i].freq_index < 2)
            {
              idx = i;
            }
            else if (pMeas->meas[j].freq_index < 2)
            {
              idx = j;
            }
          }
          pMeas->meas[idx].status &= 0xF0;
          SYS_LOGGING(OBJ_QOS, LOG_INFO, "FIRSTFIX reject PR %02d,%03d,%02d,%02d", pMeas->meas[idx].gnssMode, pMeas->meas[idx].prn, pMeas->meas[idx].freq_index, pMeas->meas[idx].cno);
        }
      }
    }
    else
    {
      //DR_reject logic
      if (fabs((double)pMeas->meas[i].dr_diff - pMeas->meas[j].dr_diff) > 10)//DR valid
      {
        if (fabs(pMeas->meas[i].dr_diff) > fabs(pMeas->meas[j].dr_diff) && (pMeas->meas[i].cno < pMeas->meas[j].cno))//bigger diff or lower cn0
        {
          pMeas->meas[i].status &= 0xFD;
#if defined(PLAYBACK_MODE)
          SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s reject DR %02d,%03d,%02d,%10.4f", __FUNCTION__, pMeas->meas[i].gnssMode, pMeas->meas[i].prn, pMeas->meas[i].freq_index, pMeas->meas[i].dr_diff);
#endif
        }
        else if (fabs(pMeas->meas[i].dr_diff) < fabs(pMeas->meas[j].dr_diff) && (pMeas->meas[i].cno > pMeas->meas[j].cno))//bigger diff or lower cn0
        {
          pMeas->meas[j].status &= 0xFD;
#if defined(PLAYBACK_MODE)
          SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s reject DR %02d,%03d,%02d,%10.4f", __FUNCTION__, pMeas->meas[i].gnssMode, pMeas->meas[i].prn, pMeas->meas[i].freq_index, pMeas->meas[i].dr_diff);
#endif
        }
        else
        {
          pMeas->meas[i].drWeightChck |= DUALMEAS_CHECK;
          pMeas->meas[j].drWeightChck |= DUALMEAS_CHECK;
#if defined(PLAYBACK_MODE)
          SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s de-weight DR %02d,%03d,%02d,%10.4f", __FUNCTION__, pMeas->meas[i].gnssMode, pMeas->meas[i].prn, pMeas->meas[i].freq_index, pMeas->meas[i].dr_diff);
          SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s de-weight DR %02d,%03d,%02d,%10.4f", __FUNCTION__, pMeas->meas[j].gnssMode, pMeas->meas[j].prn, pMeas->meas[j].freq_index, pMeas->meas[j].dr_diff);
#endif
        }
      }
      else if (fabs((double)pMeas->meas[i].dr_diff - pMeas->meas[j].dr_diff) > 5)//DR_de_weight logic
      {
        if (fabs(pMeas->meas[i].dr_diff) > fabs(pMeas->meas[j].dr_diff) && (pMeas->meas[i].cno < pMeas->meas[j].cno))//bigger diff or lower cn0
        {
          pMeas->meas[i].drWeightChck |= DUALMEAS_CHECK;
#if defined(PLAYBACK_MODE)
          SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s de-weight DR %02d,%03d,%02d,%10.4f", __FUNCTION__, pMeas->meas[i].gnssMode, pMeas->meas[i].prn, pMeas->meas[i].freq_index, pMeas->meas[i].dr_diff);
#endif
        }
        else if (fabs(pMeas->meas[i].dr_diff) < fabs(pMeas->meas[j].dr_diff) && (pMeas->meas[i].cno > pMeas->meas[j].cno))
        {
          pMeas->meas[j].drWeightChck |= DUALMEAS_CHECK;
#if defined(PLAYBACK_MODE)
          SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s de-weight DR %02d,%03d,%02d,%10.4f", __FUNCTION__, pMeas->meas[j].gnssMode, pMeas->meas[j].prn, pMeas->meas[j].freq_index, pMeas->meas[j].dr_diff);
#endif
        }
      }

      if (pMeas->avgCno > 25 && pMeas->validPrNum >= 28 && pMeas->validDrNum >= 28)
      {
        reject_prthres1 = 25;
      }
      else
      {
        reject_prthres1 = 30;
      }

      if (pMeas->meas[i].prDiffValid == 0 || pMeas->meas[j].prDiffValid == 0)
      {
        if (fabs((double)pMeas->meas[i].pr_diff - pMeas->meas[j].pr_diff) > 50 && fabs(pMeas->meas[i].pr_diff) > fabs(pMeas->meas[j].pr_diff) && fabs(pMeas->meas[i].pr_diff) > 50)
        {
          pMeas->meas[i].status &= 0xFE;
          if (fabs(pMeas->meas[i].pr_diff) > 1000.0)
          {
              pMeas->meas[i].status &= 0xFD;
          }
#if defined(PLAYBACK_MODE)
          SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s reject PR %02d,%03d,%02d,%10.4f", __FUNCTION__, pMeas->meas[i].gnssMode, pMeas->meas[i].prn, pMeas->meas[i].freq_index, pMeas->meas[i].pr_diff);
#endif
        }
        else if (fabs((double)pMeas->meas[i].pr_diff - pMeas->meas[j].pr_diff) > 50 && fabs(pMeas->meas[i].pr_diff) < fabs(pMeas->meas[j].pr_diff) && fabs(pMeas->meas[j].pr_diff) > 50)
        {
          pMeas->meas[j].status &= 0xFE;
          if (fabs(pMeas->meas[j].pr_diff) > 1000.0)
          {
              pMeas->meas[j].status &= 0xFD;
          }
#if defined(PLAYBACK_MODE)
          SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s reject PR %02d,%03d,%02d,%10.4f", __FUNCTION__, pMeas->meas[j].gnssMode, pMeas->meas[j].prn, pMeas->meas[j].freq_index, pMeas->meas[j].pr_diff);
#endif
        }
        continue;
      }
      if (fabs((double)pMeas->meas[i].pr_diff - pMeas->meas[j].pr_diff) > reject_prthres1)//PR threshhold according to satelites
      {
        if (fabs(pMeas->meas[i].pr_diff) > fabs(pMeas->meas[j].pr_diff) && fabs(pMeas->meas[i].pr_diff) > 15 && (pMeas->meas[i].cno < pMeas->meas[j].cno))//bigger diff or lower cn0
        {
          pMeas->meas[i].status &= 0xFE;
          if (fabs(pMeas->meas[i].pr_diff) > 1000.0)
          {
              pMeas->meas[i].status &= 0xFD;
          }
#if defined(PLAYBACK_MODE)
          SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s reject PR %02d,%03d,%02d,%10.4f", __FUNCTION__, pMeas->meas[i].gnssMode, pMeas->meas[i].prn, pMeas->meas[i].freq_index, pMeas->meas[i].pr_diff);
#endif
        }
        else if (fabs(pMeas->meas[i].pr_diff) < fabs(pMeas->meas[j].pr_diff) && fabs(pMeas->meas[j].pr_diff) > 15 && (pMeas->meas[j].cno < pMeas->meas[i].cno))//bigger diff or lower cn0
        {
          pMeas->meas[j].status &= 0xFE;
          if (fabs(pMeas->meas[j].pr_diff) > 1000.0)
          {
              pMeas->meas[j].status &= 0xFD;
          }
#if defined(PLAYBACK_MODE)
          SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s reject PR %02d,%03d,%02d,%10.4f", __FUNCTION__, pMeas->meas[j].gnssMode, pMeas->meas[j].prn, pMeas->meas[j].freq_index, pMeas->meas[j].pr_diff);
#endif
        }
        else
        {
          if (fabs(pMeas->meas[i].pr_diff) > fabs(pMeas->meas[j].pr_diff) && fabs(pMeas->meas[i].pr_diff) > 15)
          {
            pMeas->meas[i].prWeightChck |= DUALMEAS_CHECK;
#if defined(PLAYBACK_MODE)
            SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s de-weight PR %02d,%03d,%02d,%10.4f", __FUNCTION__, pMeas->meas[i].gnssMode, pMeas->meas[i].prn, pMeas->meas[i].freq_index, pMeas->meas[i].pr_diff);
#endif
          }
          else if (fabs(pMeas->meas[i].pr_diff) < fabs(pMeas->meas[j].pr_diff) && fabs(pMeas->meas[j].pr_diff) > 15)
          {
            pMeas->meas[j].prWeightChck |= DUALMEAS_CHECK;
#if defined(PLAYBACK_MODE)
            SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s de-weight PR %02d,%03d,%02d,%10.4f", __FUNCTION__, pMeas->meas[j].gnssMode, pMeas->meas[j].prn, pMeas->meas[j].freq_index, pMeas->meas[j].pr_diff);
#endif
          }
        }
      }
      else if (fabs((double)pMeas->meas[i].pr_diff - pMeas->meas[j].pr_diff) > 15)//PR threshhold according to satelites
      {
        if (fabs(pMeas->meas[i].pr_diff) > fabs(pMeas->meas[j].pr_diff) && fabs(pMeas->meas[i].pr_diff) > 10.0 && (pMeas->meas[i].cno < pMeas->meas[j].cno))//bigger diff or lower cn0
        {
          pMeas->meas[i].prWeightChck |= DUALMEAS_CHECK;
#if defined(PLAYBACK_MODE)
          SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s de-weight PR %02d,%03d,%02d,%10.4f", __FUNCTION__, pMeas->meas[i].gnssMode, pMeas->meas[i].prn, pMeas->meas[i].freq_index, pMeas->meas[i].pr_diff);
#endif
        }
        else if (fabs(pMeas->meas[i].pr_diff) < fabs(pMeas->meas[j].pr_diff) && fabs(pMeas->meas[j].pr_diff) > 10.0 && (pMeas->meas[j].cno < pMeas->meas[i].cno))//bigger diff or lower cn0
        {
          pMeas->meas[j].prWeightChck |= DUALMEAS_CHECK;
#if defined(PLAYBACK_MODE)
          SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s de-weight PR %02d,%03d,%02d,%10.4f", __FUNCTION__, pMeas->meas[j].gnssMode, pMeas->meas[j].prn, pMeas->meas[j].freq_index, pMeas->meas[j].pr_diff);
#endif
        }
      }
    }
  }
}


/***********************************************************************
* 函数介绍: gnss_Qos_PR_Cluster_QCOM
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：12/16/
***********************************************************************/
void gnss_Qos_PR_Cluster_QCOM(meas_blk_t* pMeas, PeStateMonitor_t* p_state)
{
  uint8_t             indx[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 }, indx_a[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], indx_b[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], indx_c[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], min_indx[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM];
  uint8_t             prRejLimit, prRejNum, outlierFlag;
  uint8_t             Table_NumThres[4] = { 0 };
  uint32_t            i;
  uint32_t            totalCnt = 0, totalCntUse = 0;
  uint32_t            cnt_a, cnt_b, cnt_c;
  float            diff[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, stdDis_a = 0.0, stdDiff_a = 0.0, prDiffAvg;
  float            distance[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, avgDistance_a, avgDiff_a;
  float            distance_a[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], distance_b[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], distance_c[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM];
  float            diff_a[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], diff_b[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], diff_c[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM];
  float            thres_ac, thres_ab, min_thres, outlierThres = 30.0;
  float            gap_distance, gap_ratio;;
  gnss_meas_t* pSvMeas;
  //sat_data_t* sp;

  memset(indx_a, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(uint8_t));
  memset(indx_b, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(uint8_t));
  memset(indx_c, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(uint8_t));
  memset(distance_a, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(float));
  memset(distance_b, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(float));
  memset(distance_c, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(float));
  memset(diff_a, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(float));
  memset(diff_b, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(float));
  memset(diff_c, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(float));

  pMeas->avgPrDiffDis = -1.0;
  pMeas->prDiffStd_a = -1.0;

  if (pMeas->hasDiff == FALSE)
  {
    return;
  }

  Table_NumThres[0] = 6;
  Table_NumThres[1] = 9;
  Table_NumThres[2] = 12;
  Table_NumThres[3] = 15;

  /* determine the PR rejected limit */
  gnss_Pe_PRDR_Num(pMeas);
  if (pMeas->validPrNum <= 5)
  {
    prRejLimit = 0;
  }
  else if (pMeas->validPrNum <= 10)
  {
    prRejLimit = pMeas->validPrNum - 6;
  }
  else
  {
    prRejLimit = pMeas->validPrNum - 8;
  }

  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &pMeas->meas[i];
    if (pSvMeas->prn == 0 || pSvMeas->prDiffValid == 0) continue;
    if ((pSvMeas->status & 0x1) == 0) continue;

    diff[totalCnt] = (float)pSvMeas->pr_diff;
    indx[totalCnt++] = i;
  }

  // calcuclate pr diff std for position bias detection
  if (totalCnt >= 5)
  {
    gnss_math_fstd(diff, totalCnt, &prDiffAvg, &(pMeas->prDiffStd_2));
  }
  else
  {
    pMeas->prDiffStd_2 = -1.0;
  }

  if (totalCnt < 8) return;

  /* Calculate Distance */
  gnss_Calc_distance(diff, distance, totalCnt);

  /* Sorting according to distance */
  gnss_Sort_WithIndx_1(distance, diff, indx, totalCnt);

  //k-mean cluster
  //get the mean of each cluster using k-mean algorithm
  outlierFlag = FALSE;
  totalCntUse = totalCnt;

  if ((distance[totalCnt - 1] - distance[totalCnt - 2]) > outlierThres)
  {
    outlierFlag = TRUE;
    totalCntUse = totalCnt - 1;
  }

  if ((distance[totalCnt - 2] - distance[totalCnt - 3]) > outlierThres)
  {
    outlierFlag = TRUE;
    totalCntUse = totalCnt - 2;
  }
  if (!outlierFlag && ((((double)distance[totalCnt - 1] + distance[totalCnt - 2]) / 2.0 - distance[totalCnt - 3]) > outlierThres))
  {
    outlierFlag = TRUE;
    totalCntUse = totalCnt - 2;
  }

  if (!gnss_kMean(totalCntUse, &distance[0], &min_indx[0], FALSE))
  {
    return;
  }
  //divide the samples into three clusters
  cnt_a = 0;
  cnt_b = 0;
  cnt_c = 0;

  for (i = 0; i < totalCntUse; i++)
  {
    if (min_indx[i] == 0)
    {
      distance_a[cnt_a] = distance[i];
      indx_a[cnt_a] = indx[i];
      diff_a[cnt_a] = diff[i];
      cnt_a++;
    }
    else if (min_indx[i] == 1)
    {
      distance_b[cnt_b] = distance[i];
      indx_b[cnt_b] = indx[i];
      diff_b[cnt_b] = diff[i];
      cnt_b++;
    }
    else
    {
      distance_c[cnt_c] = distance[i];
      indx_c[cnt_c] = indx[i];
      diff_c[cnt_c] = diff[i];
      cnt_c++;
    }
  }

  if (outlierFlag)
  {
    for (i = totalCntUse; i < totalCnt; i++)
    {
      distance_c[cnt_c] = distance[i];
      indx_c[cnt_c] = indx[i];
      diff_c[cnt_c] = diff[i];
      cnt_c++;
    }
  }

  if (cnt_a == 0)
  {
    return;
  }

  //Calculate average and std of culser A's distance
  gnss_math_fstd(distance_a, cnt_a, &avgDistance_a, &stdDis_a);

  /* calculate cluster A's PR diff average and std*/
  gnss_math_fstd(diff_a, cnt_a, &avgDiff_a, &stdDiff_a);

  pMeas->avgPrDiffDis = avgDistance_a;
  pMeas->prDiffStd_a = stdDiff_a;

  //decide to reject or deweight the measurement according to the cluster result
  gap_distance = distance[totalCnt - 1] - distance[0];
  gap_ratio = distance[totalCnt - 1] / distance[0];
  if (gap_distance < 5.0 && gap_ratio < 2)
  {
    return;
  }

  prRejNum = 0;

  //get pr reject threshold
  if (cnt_a <= Table_NumThres[0])
  {
    thres_ac = 20 * stdDis_a;
    min_thres = 20;
  }
  else if (cnt_a <= Table_NumThres[1])
  {
    thres_ac = 15 * stdDis_a;
    min_thres = 15;
  }
  else if (cnt_a <= Table_NumThres[2])
  {
    thres_ac = 10 * stdDis_a;
    min_thres = 10;
  }
  else if (cnt_a <= Table_NumThres[3])
  {
    thres_ac = 7.5f * stdDis_a;
    min_thres = 7.5;
  }
  else
  {
    thres_ac = (float)(5 * stdDis_a);
    min_thres = 5;
  }

  if (cnt_c >= cnt_a)
  {
    thres_ac *= (2 + (cnt_c - cnt_a));
    min_thres *= (2 + (cnt_c - cnt_a));
  }

  if (thres_ac < min_thres)
  {
    thres_ac = min_thres;
  }
  else if (thres_ac > 100)
  {
    thres_ac = 100;
  }



  //get pr deweight threshold
  if (cnt_a <= Table_NumThres[0])
  {
    thres_ab = 15 * stdDis_a;
    min_thres = 15.0;
  }
  else if (cnt_a <= Table_NumThres[1])
  {
    thres_ab = 10 * stdDis_a;
    min_thres = 10.0;
  }
  else if (cnt_a <= Table_NumThres[2])
  {
    thres_ab = (float)(7.5 * stdDis_a);
    min_thres = 7.5;
  }
  else if (cnt_a <= Table_NumThres[3])
  {
    thres_ab = (float)(5 * stdDis_a);
    min_thres = 5;
  }
  else
  {
    thres_ab = (float)(3.0 * stdDis_a);
    min_thres = 3;
  }

  if (thres_ab < min_thres)
  {
    thres_ab = min_thres;
  }
  else if (thres_ab > 80)
  {
    thres_ab = 80;
  }

  if (p_state->posHasBias)
  {
    if (avgDistance_a > 150)
    {
      thres_ac *= 15;
      thres_ab *= 15;
    }
    else if (avgDistance_a > 100)
    {
      thres_ac *= 10;
      thres_ab *= 10;
    }
    else if (avgDistance_a > 50)
    {
      thres_ac *= 5;
      thres_ab *= 5;
    }
    else
    {
      thres_ac *= 2;
      thres_ab *= 2;
    }
  }

  SYS_LOGGING(OBJ_QOS, LOG_INFO, "PR cluster rej_thres:%10.4f ,deweight_thres:%10.4f ,diffStd of cluster a:%10.4f,", thres_ac, thres_ab, stdDiff_a);

  if (cnt_c > 0 && cnt_c < (cnt_a + cnt_b))
  {
    for (i = cnt_c; i > 0; i--)
    {
      pSvMeas = &pMeas->meas[indx_c[i - 1]];
      if (pSvMeas->quality & PR_GOOD) continue;
      if (((distance_c[i - 1] - avgDistance_a) > thres_ac) && prRejNum < prRejLimit &&
        ((fabs(diff_c[i - 1]) > 20.0) || (stdDiff_a < 5.0)))
      {
        /* If PDOP enlarge two times after reject this satellite, then don't reject this satellite */
        if ((gnss_Pe_Dop_Check(pMeas, indx_c[i - 1], 1) == FALSE) && (
          distance_c[i - 1] - avgDistance_a) < 2 * thres_ac)
        {
          /*pSvMeas->prWeightChck  |= CLUSTER_CHECK;
          SYS_LOGGING(OBJ_QOS,LOG_INFO,"gnssMode(%02d)prn(%03d) was deweight in %s cluster_c: %10.4f,%10.4f",
          pSvMeas->gnssMode,pSvMeas->prn,__FUNCTION__,distance_c[i-1],pSvMeas->pr_diff);*/
          continue;
        }
        pSvMeas->status &= 0xFE;
        prRejNum++;
        //sp = gnss_sd_get_sv_data(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);
        /*if(sp)
        {
        gnss_Qos_BadMeasFlag_Set(pMeas->tor,sp,MEAS_TYPE_PR);
        }*/
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was rejected in %s cluster_c: %10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_c[i - 1], pSvMeas->pr_diff);
      }
      else if (((distance_c[i - 1] - avgDistance_a) > thres_ab) &&
        ((fabs(diff_c[i - 1]) > 15.0) || (stdDiff_a < 5.0)) && cnt_c < cnt_a)
      {
        pSvMeas->prWeightChck |= CLUSTER_CHECK;
        //sp = gnss_sd_get_sv_data(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);
        /*if(sp)
        {
        gnss_Qos_BadMeasFlag_Set(pMeas->tor,sp,MEAS_TYPE_PR);
        }*/
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was deweight in %s cluster_c: %10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_c[i - 1], pSvMeas->pr_diff);
      }

    }
  }

  //thres_ab = 20.0;
  if (cnt_b > 0 && cnt_b < cnt_a)
  {
    for (i = cnt_b; i > 0; i--)
    {
      pSvMeas = &pMeas->meas[indx_b[i - 1]];
      if (pSvMeas->quality & PR_GOOD) continue;
      if ((distance_b[i - 1] - avgDistance_a) > thres_ac && prRejNum < prRejLimit &&
        ((fabs(diff_b[i - 1]) > 20.0) || (stdDiff_a < 5.0)))
      {
        /* If PDOP enlarge two times after reject this satellite, then don't reject this satellite */
        if (gnss_Pe_Dop_Check(pMeas, indx_b[i - 1], 1) == FALSE)
        {
          continue;
        }
        pSvMeas->status &= 0xFE;
        prRejNum++;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was rejected in %s cluster_b: %10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_b[i - 1], pSvMeas->pr_diff);
      }
      else if ((distance_b[i - 1] - avgDistance_a) > thres_ab &&
        ((fabs(diff_b[i - 1]) > 10.0) || (stdDiff_a < 5.0)))
      {
        pSvMeas->prWeightChck |= CLUSTER_CHECK;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was deweight in %s cluster_b: %10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_b[i - 1], pSvMeas->pr_diff);
      }
    }
  }

  if ((g_pe_cfg.automobile == 0) && (cnt_a + cnt_b >= 15) && (prRejNum <= 3))
  {
    for (i = cnt_c; i > 0; i--)
    {
      pSvMeas = &pMeas->meas[indx_c[i - 1]];
      if (prRejNum <= 3)
      {
        pSvMeas->status &= 0xFE;
        prRejNum++;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was rejected in %s cluster_c: %10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_c[i - 1], pSvMeas->pr_diff);
      }
    }
  }

}
/***********************************************************************
* 函数介绍: gnss_Qos_PR_Cluster_UBLOX
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：12/16/
***********************************************************************/
uint8_t gnss_Qos_PR_Cluster_UBLOX(meas_blk_t* pMeas, PeStateMonitor_t* p_state)
{
  uint8_t             indx[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 }, indx_a[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], indx_b[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], indx_c[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], min_indx[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM];
  uint8_t             prRejLimit, prRejNum, outlierFlag, flag = TRUE, smallDiffCnt = 0, goodDiffCnt = 0, measDiffCnt = 0, smallDiffCheckFlag = FALSE;
  uint8_t             Table_NumThres[4] = { 0 };
  uint32_t            i;
  uint32_t            totalCnt = 0, totalCntUse = 0;
  uint32_t            cnt_a, cnt_b, cnt_c;
  float            diff[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, stdDis_a = 0.0, stdDis_c = 0.0, stdDiff_a = 0.0, prDiffAvg;
  float            distance[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, avgDistance_a, avgDiff_a, avgDistance_c;
  float            distance_a[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], distance_b[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], distance_c[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM];
  float            diff_a[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], diff_b[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], diff_c[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM];
  float            thres_ac, thres_ab, min_thres, outlierThres = 30.0;
  float            gap_distance, ratio = 1.0;
  gnss_meas_t* pSvMeas;
  sat_data_t* sp;
  Kf_t* p_Kf;

  p_Kf = gnss_Pe_Get_Kf();

  memset(indx_a, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(uint8_t));
  memset(indx_b, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(uint8_t));
  memset(indx_c, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(uint8_t));
  memset(distance_a, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(float));
  memset(distance_b, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(float));
  memset(distance_c, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(float));
  memset(diff_a, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(float));
  memset(diff_b, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(float));
  memset(diff_c, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(float));

  pMeas->avgPrDiffDis = -1.0;
  pMeas->prDiffStd_a = -1.0;

  if (pMeas->hasDiff == FALSE)
  {
    return flag;
  }

  //Table_NumThres[0] = 8;
  //Table_NumThres[1] = 10;
  //Table_NumThres[2] = 15;

  Table_NumThres[0] = 6;
  Table_NumThres[1] = 8;
  Table_NumThres[2] = 12;
  Table_NumThres[3] = 16;

  /* determine the PR rejected limit */
  gnss_Pe_PRDR_Num(pMeas);
  if (pMeas->validPrNum <= 5)
  {
    prRejLimit = 0;
  }
  else
  {
    if (peMode.userSceData.isUnderEleRoad)
    {
      prRejLimit = pMeas->validPrNum - 5;
    }
    else
    {
      prRejLimit = pMeas->validPrNum - 6;
    }
  }

  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &pMeas->meas[i];
    if (pSvMeas->prn == 0 || pSvMeas->prDiffValid == 0) continue;
    if ((pSvMeas->status & 0x1) == 0) continue;

    diff[totalCnt] = (float)pSvMeas->pr_diff;
    indx[totalCnt++] = i;
  }

  // calcuclate pr diff std for position bias detection
  if (totalCnt >= 5)
  {
    gnss_math_fstd(diff, totalCnt, &prDiffAvg, &(pMeas->prDiffStd_2));
  }
  else
  {
    pMeas->prDiffStd_2 = -1.0;
  }

  if (totalCnt < 6) return flag;

  /* Calculate Distance */
  gnss_Calc_distance(diff, distance, totalCnt);

  /* Sorting according to distance */
  gnss_Sort_WithIndx_1(distance, diff, indx, totalCnt);

  //k-mean cluster
  //get the mean of each cluster using k-mean algorithm
  outlierFlag = FALSE;
  totalCntUse = totalCnt;

  if ((distance[totalCnt - 1] - distance[totalCnt - 2]) > outlierThres)
  {
    outlierFlag = TRUE;
    totalCntUse = totalCnt - 1;
  }

  if ((distance[totalCnt - 2] - distance[totalCnt - 3]) > outlierThres)
  {
    outlierFlag = TRUE;
    totalCntUse = totalCnt - 2;
  }
  if (!outlierFlag && ((((double)distance[totalCnt - 1] + distance[totalCnt - 2]) / 2.0 - distance[totalCnt - 3]) > outlierThres))
  {
    outlierFlag = TRUE;
    totalCntUse = totalCnt - 2;
  }

  if (!gnss_kMean(totalCntUse, &distance[0], &min_indx[0], FALSE))
  {
    return flag;
  }
  //divide the samples into three clusters
  cnt_a = 0;
  cnt_b = 0;
  cnt_c = 0;

  for (i = 0; i < totalCntUse; i++)
  {
    if (min_indx[i] == 0)
    {
      distance_a[cnt_a] = distance[i];
      indx_a[cnt_a] = indx[i];
      diff_a[cnt_a] = diff[i];
      cnt_a++;
    }
    else if (min_indx[i] == 1)
    {
      distance_b[cnt_b] = distance[i];
      indx_b[cnt_b] = indx[i];
      diff_b[cnt_b] = diff[i];
      cnt_b++;
    }
    else
    {
      distance_c[cnt_c] = distance[i];
      indx_c[cnt_c] = indx[i];
      diff_c[cnt_c] = diff[i];
      cnt_c++;
    }
  }

  if (outlierFlag)
  {
    for (i = totalCntUse; i < totalCnt; i++)
    {
      distance_c[cnt_c] = distance[i];
      indx_c[cnt_c] = indx[i];
      diff_c[cnt_c] = diff[i];
      cnt_c++;
    }
  }

  if (cnt_a == 0)
  {
    return flag;
  }

  //Calculate average and std of culser A's distance
  gnss_math_fstd(distance_a, cnt_a, &avgDistance_a, &stdDis_a);

  //Calculate average and std of culser C's distance
  gnss_math_fstd(distance_c, cnt_c, &avgDistance_c, &stdDis_c);

  /* calculate cluster A's PR diff average and std*/
  gnss_math_fstd(diff_a, cnt_a, &avgDiff_a, &stdDiff_a);

  pMeas->avgPrDiffDis = avgDistance_a;
  pMeas->prDiffStd_a = stdDiff_a;
  /*calculate DistanceAvg<10 cnt */
  for (i = 0; i < totalCntUse; i++)
  {
    if (fabs((double)distance[i] - avgDistance_a) < 10)
    {
      pMeas->prDiffDistanceAvg10Cnt++;
    }
  }

  for (i = 0; i < cnt_b; i++)
  {
    pSvMeas = &pMeas->meas[indx_b[i]];
    if (pSvMeas->quality & PR_GOOD)
    {
      flag = FALSE;
    }
  }

  for (i = 0; i < cnt_c; i++)
  {
    pSvMeas = &pMeas->meas[indx_c[i]];
    if (pSvMeas->quality & PR_GOOD)
    {
      flag = FALSE;
    }
  }

  //decide to reject or deweight the measurement according to the cluster result
  gap_distance = distance[totalCnt - 1] - distance[0];
  if (gap_distance < 5.0)
  {
    return flag;
  }

  prRejNum = 0;

  //get pr reject threshold
  if (cnt_a < Table_NumThres[0])
  {
    thres_ac = 20 * stdDis_a;
    min_thres = 20;
  }
  else if (cnt_a < Table_NumThres[1])
  {
    thres_ac = 15 * stdDis_a;
    min_thres = 15;
  }
  else if (cnt_a < Table_NumThres[2])
  {
    thres_ac = 10 * stdDis_a;
    min_thres = 10;
  }
  else if (cnt_a < Table_NumThres[3])
  {
    thres_ac = 7.5f * stdDis_a;
    min_thres = 5.0;
  }
  else
  {
    if (p_Kf->kfCnt > 20)
    {
      thres_ac = (float)(5.0 * stdDis_a);
      min_thres = 2.0;
    }
    else
    {
      thres_ac = 7.5f * stdDis_a;
      min_thres = 5.0;
    }
  }
  if (cnt_a < Table_NumThres[2] && ((double)avgDistance_c - avgDistance_a) > 8.0)
  {
    for (i = 0; i < pMeas->measCnt; i++)
    {
      pSvMeas = &(pMeas->meas[i]);
      if (pSvMeas->prn == 0) continue;
      if ((pSvMeas->status & 0x1) == 0) continue;
      sp = gnss_sd_get_sv_data(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);
      if (sp == NULL) continue;
      if (fabs(pSvMeas->pr_diff) < 15.0)
      {
        smallDiffCnt++;
      }
      if (fabs(pSvMeas->pr_diff) < 5.0)
      {
        goodDiffCnt++;
      }
      measDiffCnt++;
    }
    if (goodDiffCnt > 5 && (double)goodDiffCnt / (double)smallDiffCnt > 0.7 && p_Kf->posRes < 35 && avgDistance_c > 25)
    {
      if (fabs(avgDiff_a) <= 15.0)
      {
        smallDiffCheckFlag = TRUE;
      }
      else if (fabs(avgDiff_a) > 15.0 && avgDistance_a > 20.0 && avgDistance_c > 30.0 && stdDiff_a > 10.0)
      {
        smallDiffCheckFlag = TRUE;
      }
    }
  }
  if (smallDiffCheckFlag)
  {
    thres_ac = 7.5f * stdDis_a;
    min_thres = 7.5;
  }

  if (cnt_c >= cnt_a)
  {
    ratio = (float)(2 + (cnt_c - cnt_a));
    thres_ac *= ratio;
    min_thres *= ratio;
  }

  if (thres_ac < min_thres)
  {
    thres_ac = min_thres;
  }
  else if (thres_ac > 100)
  {
    thres_ac = 100;
  }

  //get pr deweight threshold
  if (cnt_a < Table_NumThres[0])
  {
    thres_ab = 15 * stdDis_a;
    min_thres = 15.0;
  }
  else if (cnt_a < Table_NumThres[1])
  {
    thres_ab = 10 * stdDis_a;
    min_thres = 10.0;
  }
  else if (cnt_a < Table_NumThres[2])
  {
    thres_ab = (float)(7.5 * stdDis_a);
    min_thres = 7.5;
  }
  else if (cnt_a < Table_NumThres[3])
  {
    thres_ab = (float)(5.0 * stdDis_a);
    min_thres = 3.0;
  }
  else
  {
    if (p_Kf->kfCnt > 20)
    {
      thres_ab = (float)(3.0 * stdDis_a);
      min_thres = 1.0;
    }
    else
    {
      thres_ab = (float)(5.0 * stdDis_a);
      min_thres = 3.0;
    }
  }

  if (smallDiffCheckFlag)
  {
    thres_ab = 7.5f * stdDis_a;
    min_thres = 5.0;
  }
  if (thres_ab < min_thres)
  {
    thres_ab = min_thres;
  }
  else if (thres_ab > 80)
  {
    thres_ab = 80;
  }

  if (p_state->posHasBias)
  {
    if (avgDistance_a > 150)
    {
      thres_ac *= 15;
      thres_ab *= 15;
    }
    else if (avgDistance_a > 100)
    {
      thres_ac *= 10;
      thres_ab *= 10;
    }
    else if (avgDistance_a > 50)
    {
      thres_ac *= 5;
      thres_ab *= 5;
    }
    else
    {
      thres_ac *= 2;
      thres_ab *= 2;
    }
  }

  SYS_LOGGING(OBJ_QOS, LOG_INFO, "PR cluster rej_thres:%10.4f ,deweight_thres:%10.4f ,diffStd of cluster a:%10.4f,", thres_ac, thres_ab, stdDiff_a);

  if (cnt_c > 0 && cnt_c < (cnt_a + cnt_b))
  {
    for (i = cnt_c; i > 0; i--)
    {
      pSvMeas = &pMeas->meas[indx_c[i - 1]];
      if (pSvMeas->quality & PR_GOOD)
      {
        if (!(((((double)distance_c[i - 1] - avgDistance_a) >= 2.0 * thres_ac && ((double)avgDistance_c - avgDistance_a) > 1.5 * avgDistance_a) ||
          (((double)avgDistance_c - avgDistance_a) > 2.5 * avgDistance_a)) && gap_distance > 20.0 && fabs(pSvMeas->pr_diff) > 20.0 && pSvMeas->cno > 40)
          && !(i == cnt_c && cnt_c >= 2 && distance_c[i - 1] > 40.0 && distance_c[i - 2] < 18.0))
        {
          continue;
        }
      }
      if (((distance_c[i - 1] - avgDistance_a) > thres_ac) && prRejNum < prRejLimit &&
        ((fabs(diff_c[i - 1]) > 20.0) || (stdDiff_a < 5.0)))
      {
        /* If PDOP enlarge two times after reject this satellite, then don't reject this satellite */
        if ((gnss_Pe_Dop_Check(pMeas, indx_c[i - 1], 1) == FALSE) && (
          distance_c[i - 1] - avgDistance_a) < 2 * thres_ac)
        {
          continue;
        }
        pSvMeas->status &= 0xFE;
        prRejNum++;
        //sp = gnss_sd_get_sv_data(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);

        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was rejected in %s cluster_c: %10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_c[i - 1], pSvMeas->pr_diff);
      }
      else if (((distance_c[i - 1] - avgDistance_a) > thres_ab) &&
        ((fabs(diff_c[i - 1]) > 15.0) || (stdDiff_a < 5.0)) && cnt_c < cnt_a)
      {
        pSvMeas->prWeightChck |= CLUSTER_CHECK;
        //sp = gnss_sd_get_sv_data(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);

        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was deweight in %s cluster_c: %10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_c[i - 1], pSvMeas->pr_diff);
      }

    }
  }

  //thres_ab = 20.0;
  if (cnt_b > 0 && cnt_b < cnt_a)
  {
    for (i = cnt_b; i > 0; i--)
    {
      pSvMeas = &pMeas->meas[indx_b[i - 1]];
      if (pSvMeas->quality & PR_GOOD && ((((double)distance_b[i - 1] - avgDistance_a) < 1.4 * thres_ac) || fabs(diff_b[i - 1]) < 30.0))
      {
        continue;
      }
      if ((distance_b[i - 1] - avgDistance_a) > thres_ac && prRejNum < prRejLimit &&
        ((fabs(diff_b[i - 1]) > 20.0) || (stdDiff_a < 5.0)))
      {
        /* If PDOP enlarge two times after reject this satellite, then don't reject this satellite */
        if (gnss_Pe_Dop_Check(pMeas, indx_b[i - 1], 1) == FALSE)
        {
          continue;
        }
        pSvMeas->status &= 0xFE;
        prRejNum++;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was rejected in %s cluster_b: %10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_b[i - 1], pSvMeas->pr_diff);
      }
      else if ((distance_b[i - 1] - avgDistance_a) > thres_ab &&
        ((fabs(diff_b[i - 1]) > 10.0) || (stdDiff_a < 5.0)))
      {
        pSvMeas->prWeightChck |= CLUSTER_CHECK;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was deweight in %s cluster_b: %10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_b[i - 1], pSvMeas->pr_diff);
      }
    }
  }

  return flag;
}

#if defined(PLAYBACK_MODE)
/***********************************************************************
* 函数介绍: gnss_Qos_PR_Cluster_Test
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：12/16/
***********************************************************************/
void gnss_Qos_PR_Cluster_Test(meas_blk_t* pMeas, PeStateMonitor_t* p_state)
{
  uint8_t             indx[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 }, indx_cluster[3][MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], min_indx[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM];
  uint8_t             prRejLimit, prRejNum, outlierFlag;
  uint8_t             Table_NumThres[3] = { 0 };
  uint32_t            i, j;
  uint32_t            totalCnt = 0, totalCntUse = 0;
  uint32_t            cnt_cluster[3], cnt[3], cluster_indx[3], cnt_good[3] = { 0 };
  float            diff[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, stdDis_a = 0.0, stdDiff_a = 0.0;
  float            distance[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, diff_avg[3] = { 0.0 }, dis_avg[3] = { 0.0 }, avgDistance_a, avgDiff_a;
  float            distance_cluster[3][MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM];
  float            diff_cluster[3][MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM];
  float            thres_ac, thres_ab, min_thres, outlierThres = 50.0;
  float            gap_distance;
  uint8_t* pIndx_1;
  uint8_t* pIndx_2;
  uint8_t* pIndx_3;
  float* pDiff_1;
  float* pDiff_2;
  float* pDiff_3;
  float* pDis_1;
  float* pDis_2;
  float* pDis_3;
  gnss_meas_t* pSvMeas;
  sat_data_t* sp;

  memset(cnt_cluster, 0, 3 * sizeof(uint32_t));
  memset(indx_cluster, 0, 3 * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(uint8_t));
  memset(distance_cluster, 0, 3 * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(float));
  memset(diff_cluster, 0, 3 * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(float));

  pMeas->avgPrDiffDis = -1.0;

  if (pMeas->hasDiff == FALSE)
  {
    return;
  }

  if (g_pe_cfg.chipType == UBLOX)
  {
    Table_NumThres[0] = 6;
    Table_NumThres[1] = 8;
    Table_NumThres[2] = 10;
  }
  else
  {
    Table_NumThres[0] = 8;
    Table_NumThres[1] = 10;
    Table_NumThres[2] = 12;
  }

  /* determine the PR rejected limit */
  gnss_Pe_PRDR_Num(pMeas);
  if (pMeas->validPrNum <= 5)
  {
    prRejLimit = 0;
  }
  else if (pMeas->validPrNum <= 10)
  {
    prRejLimit = pMeas->validPrNum - 6;
  }
  else
  {
    prRejLimit = pMeas->validPrNum - 8;
  }

  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &pMeas->meas[i];
    if (pSvMeas->prn == 0 || pSvMeas->prDiffValid == 0) continue;
    if ((pSvMeas->status & 0x1) == 0) continue;

    diff[totalCnt] = (float)pSvMeas->pr_diff;
    indx[totalCnt++] = i;
  }

  if (g_pe_cfg.chipType == SPRD)
  {
    if (totalCnt < 6) return;
  }
  else
  {
    if (totalCnt < 8) return;
  }

  /* Calculate Distance */
  gnss_Calc_distance(diff, distance, totalCnt);

  /* Sorting according to diff */
  gnss_Sort_WithIndx_1(diff, distance, indx, totalCnt);

  if (diff[0] < 0 && fabs(diff[0]) > 10.0)
  {
    return;
  }

  //k-mean cluster
  //get the mean of each cluster using k-mean algorithm
  outlierFlag = FALSE;
  totalCntUse = totalCnt;

  if (totalCnt >= 8)
  {
    if ((distance[totalCnt - 1] - distance[totalCnt - 2]) > outlierThres)
    {
      outlierFlag = TRUE;
      totalCntUse = totalCnt - 1;
    }

    if ((distance[totalCnt - 2] - distance[totalCnt - 3]) > outlierThres)
    {
      outlierFlag = TRUE;
      totalCntUse = totalCnt - 2;
    }
    /*if(!outlierFlag&&(((distance[totalCnt-1]+distance[totalCnt-2])/2.0-distance[totalCnt-3])>outlierThres))
    {
    outlierFlag = TRUE;
    totalCntUse = totalCnt-2;
    }*/
  }

  if (!gnss_kMean(totalCntUse, &diff[0], &min_indx[0], TRUE))
  {
    return;
  }
  //divide the samples into three clusters

  for (i = 0; i < totalCntUse; i++)
  {
    if (min_indx[i] < 3)
    {
      distance_cluster[min_indx[i]][cnt_cluster[min_indx[i]]] = distance[i];
      indx_cluster[min_indx[i]][cnt_cluster[min_indx[i]]] = indx[i];
      diff_cluster[min_indx[i]][cnt_cluster[min_indx[i]]] = diff[i];
      cnt_cluster[min_indx[i]]++;
    }
  }

  if (outlierFlag)
  {
    for (i = totalCntUse; i < totalCnt; i++)
    {
      distance_cluster[2][cnt_cluster[2]] = distance[i];
      indx_cluster[2][cnt_cluster[2]] = indx[i];
      diff_cluster[2][cnt_cluster[2]] = diff[i];
      cnt_cluster[2]++;
    }
  }

  for (i = 0; i < 3; i++)
  {
    if (cnt_cluster[i] > 0)
    {
      dis_avg[i] = (float)gnssClcAvg_FLT(&(distance_cluster[i][0]), cnt_cluster[i]);
      diff_avg[i] = (float)gnssClcAvg_FLT(&(diff_cluster[i][0]), cnt_cluster[i]);
    }
    else
    {
      dis_avg[i] = 0;
    }
    for (j = 0; j < cnt_cluster[i]; j++)
    {
      pSvMeas = &pMeas->meas[indx_cluster[i][j]];
      if (pSvMeas->quality & PR_GOOD)
      {
        cnt_good[i]++;
      }
    }
  }

  gnss_Cluster_Sort(&diff_avg[0], cnt_cluster, cluster_indx, cnt_good, 3);

  pIndx_1 = &(indx_cluster[cluster_indx[0]][0]);
  pDiff_1 = &(diff_cluster[cluster_indx[0]][0]);
  pDis_1 = &(distance_cluster[cluster_indx[0]][0]);
  cnt[0] = cnt_cluster[cluster_indx[0]];

  pIndx_2 = &(indx_cluster[cluster_indx[1]][0]);
  pDiff_2 = &(diff_cluster[cluster_indx[1]][0]);
  pDis_2 = &(distance_cluster[cluster_indx[1]][0]);
  cnt[1] = cnt_cluster[cluster_indx[1]];

  pIndx_3 = &(indx_cluster[cluster_indx[2]][0]);
  pDiff_3 = &(diff_cluster[cluster_indx[2]][0]);
  pDis_3 = &(distance_cluster[cluster_indx[2]][0]);
  cnt[2] = cnt_cluster[cluster_indx[2]];

  //Calculate average and std of culser A's distance
  gnss_math_fstd(pDis_1, cnt[0], &avgDistance_a, &stdDis_a);

  /* calculate cluster A's PR diff average and std*/
  gnss_math_fstd(pDiff_1, cnt[0], &avgDiff_a, &stdDiff_a);

  pMeas->avgPrDiffDis = avgDistance_a;

#if defined(PLAYBACK_MODE)
  for (i = 0; i < cnt[0]; i++)
  {
    SYS_LOGGING(OBJ_QOS, LOG_INFO, "PR clusterTest_a: %2d %2d %2d %10.4f %10.4f %2d %10.4f %10.4f %10.4f %10.4f", pMeas->meas[pIndx_1[i]].gnssMode, pMeas->meas[pIndx_1[i]].prn, pMeas->meas[pIndx_1[i]].cno, pDis_1[i], pDiff_1[i], cnt[0], avgDistance_a, stdDis_a, avgDiff_a, stdDiff_a);
  }

  for (i = 0; i < cnt[1]; i++)
  {
    SYS_LOGGING(OBJ_QOS, LOG_INFO, "PR clusterTest_b: %2d %2d %2d %10.4f %10.4f %2d", pMeas->meas[pIndx_2[i]].gnssMode, pMeas->meas[pIndx_2[i]].prn, pMeas->meas[pIndx_2[i]].cno, pDis_2[i], pDiff_2[i], cnt[1]);
  }

  for (i = 0; i < cnt[2]; i++)
  {
    SYS_LOGGING(OBJ_QOS, LOG_INFO, "PR clusterTest_c: %2d %2d %2d %10.4f %10.4f %2d", pMeas->meas[pIndx_3[i]].gnssMode, pMeas->meas[pIndx_3[i]].prn, pMeas->meas[pIndx_3[i]].cno, pDis_3[i], pDiff_3[i], cnt[2]);
  }
#endif
  //decide to reject or deweight the measurement according to the cluster result
  gap_distance = (float)fabs((double)diff[totalCnt - 1] - diff[0]);
  if (gap_distance < 20.0)
  {
    return;
  }

  prRejNum = 0;

  //get pr reject threshold
  if (cnt[0] <= Table_NumThres[0])
  {
    thres_ac = (float)(15.0 * stdDiff_a / 2.0);
    min_thres = 20;
  }
  else if (cnt[0] <= Table_NumThres[1])
  {
    thres_ac = (float)(12.5 * stdDiff_a / 2.0);
    min_thres = 15;
  }
  else if (cnt[0] <= Table_NumThres[2])
  {
    thres_ac = (float)(10.0 * stdDiff_a / 2.0);
    min_thres = 10;
  }
  else
  {
    thres_ac = (float)(7.5 * stdDiff_a / 2.0);
    min_thres = 7.5;
  }

  if (thres_ac < min_thres)
  {
    thres_ac = min_thres;
  }
  else if (thres_ac > 100)
  {
    thres_ac = 100;
  }

  //get pr deweight threshold
  if (cnt[0] <= Table_NumThres[0])
  {
    thres_ab = (float)(12.5 * stdDiff_a / 2.0);
    min_thres = 15.0;
  }
  else if (cnt[0] <= Table_NumThres[1])
  {
    thres_ab = (float)(10.0 * stdDiff_a / 2.0);
    min_thres = 10.0;
  }
  else if (cnt[0] <= Table_NumThres[2])
  {
    thres_ab = (float)(7.5 * stdDiff_a / 2.0);
    min_thres = 7.5;
  }
  else
  {
    thres_ab = (float)(5.0 * stdDiff_a / 2.0);
    min_thres = 5.0;
  }

  if (thres_ab < min_thres)
  {
    thres_ab = min_thres;
  }
  else if (thres_ab > 80)
  {
    thres_ab = 80;
  }

  if (p_state->posHasBias)
  {
    if (avgDistance_a > 150)
    {
      thres_ac *= 50;
      thres_ab *= 50;
    }
    else if (avgDistance_a > 100)
    {
      thres_ac *= 25;
      thres_ab *= 25;
    }
    else if (avgDistance_a > 50)
    {
      thres_ac *= 10;
      thres_ab *= 10;
    }
    else
    {
      thres_ac *= 5;
      thres_ab *= 5;
    }
  }

  SYS_LOGGING(OBJ_QOS, LOG_INFO, "PR cluster rej_thres:%10.4f ,deweight_thres:%10.4f ,diffStd of cluster a:%10.4f,", thres_ac, thres_ab, stdDiff_a);

  if (cnt[2] > 0 && cnt[2] <= (cnt[0] + cnt[1]))
  {
    for (i = cnt[2]; i > 0; i--)
    {
      pSvMeas = &pMeas->meas[pIndx_3[i - 1]];
      if ((pSvMeas->quality & PR_GOOD) && stdDiff_a > 2.0)
      {
        continue;
      }
      if ((fabs((double)pDiff_3[i - 1] - avgDiff_a) > thres_ac) && prRejNum < prRejLimit &&
        ((fabs(pDiff_3[i - 1]) > 20.0) || (stdDiff_a < 5.0)))
      {
        /* If PDOP enlarge two times after reject this satellite, then don't reject this satellite */
        if ((gnss_Pe_Dop_Check(pMeas, pIndx_3[i - 1], 1) == FALSE) &&
          fabs((double)pDiff_3[i - 1] - avgDiff_a) < 2.0 * thres_ac)
        {
          continue;
        }
        pSvMeas->status &= 0xFE;
        prRejNum++;

        sp = gnss_sd_get_sv_data(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);
        /*if(sp)
        {
        gnss_Qos_BadMeasFlag_Set(pMeas->tor,sp,MEAS_TYPE_PR);
        }*/
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was rejected in %s cluster_c: %10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, pDis_3[i - 1], pSvMeas->pr_diff);
      }
      else if ((fabs((double)pDiff_3[i - 1] - avgDiff_a) > thres_ab) &&
        ((fabs(pDiff_3[i - 1]) > 15.0) || (stdDiff_a < 5.0)))
      {
        if (pSvMeas->prWeightChck & CLUSTER_CHECK)
        {
          pSvMeas->status &= 0xFE;
          prRejNum++;
          SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was rejected in %s cluster_c: %10.4f,%10.4f",
            pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, pDis_3[i - 1], pSvMeas->pr_diff);
        }
        else
        {
          pSvMeas->prWeightChck |= CLUSTER_CHECK;
        }
        sp = gnss_sd_get_sv_data(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);
        /*if(sp)
        {
        gnss_Qos_BadMeasFlag_Set(pMeas->tor,sp,MEAS_TYPE_PR);
        }*/
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was deweight in %s cluster_c: %10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, pDis_3[i - 1], pSvMeas->pr_diff);
      }

    }
  }

  //thres_ab = 20.0;
  if (cnt[1] > 0 && cnt[1] <= (cnt[0] + cnt[2]))
  {
    for (i = cnt[1]; i > 0; i--)
    {
      pSvMeas = &pMeas->meas[pIndx_2[i - 1]];
      if (pSvMeas->quality & PR_GOOD)
      {
        continue;
      }
      if (fabs((double)pDiff_2[i - 1] - avgDiff_a) > thres_ac && prRejNum < prRejLimit &&
        ((fabs(pDiff_2[i - 1]) > 20.0) || (stdDiff_a < 5.0)))
      {
        /* If PDOP enlarge two times after reject this satellite, then don't reject this satellite */
        if (gnss_Pe_Dop_Check(pMeas, pIndx_2[i - 1], 1) == FALSE)
        {
          continue;
        }
        pSvMeas->status &= 0xFE;
        prRejNum++;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was rejected in %s cluster_b: %10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, pDis_2[i - 1], pSvMeas->pr_diff);
      }
      else if (fabs((double)pDiff_2[i - 1] - avgDiff_a) > thres_ab &&
        ((fabs(pDiff_2[i - 1]) > 10.0) || (stdDiff_a < 5.0)))
      {
        pSvMeas->prWeightChck |= CLUSTER_CHECK;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was deweight in %s cluster_b: %10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, pDis_2[i - 1], pSvMeas->pr_diff);
      }
    }
  }
}
#endif
/***********************************************************************
* 函数介绍: gnss_Qos_PR_Cluster_SPRD
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：12/16/
***********************************************************************/
void gnss_Qos_PR_Cluster_SPRD(meas_blk_t* pMeas, PeStateMonitor_t* p_state)
{
#ifdef USED_IN_MC262M
  uint8_t             indx[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 }, indx_a[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], indx_b[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], indx_c[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], min_indx[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM];
  uint8_t             prRejLimit, prRejNum, outlierFlag;
  uint8_t             Table_NumThres[3] = { 0 };
  uint32_t            i;
  uint32_t            totalCnt = 0, totalCntUse = 0;
  uint32_t            cnt_a, cnt_b, cnt_c;
  float            diff[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, stdDis_a = 0.0, stdDiff_a = 0.0;
  float            distance[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, avgDistance_a, avgDiff_a;
  float            distance_a[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], distance_b[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], distance_c[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM];
  float            diff_a[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], diff_b[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], diff_c[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM];
  float            thres_ac, thres_ab, min_thres, outlierThres = 30.0;
  float            gap_distance;
  gnss_meas_t* pSvMeas;
  sat_data_t* sp;

  memset(indx_a, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(uint8_t));
  memset(indx_b, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(uint8_t));
  memset(indx_c, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(uint8_t));
  memset(distance_a, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(float));
  memset(distance_b, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(float));
  memset(distance_c, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(float));
  memset(diff_a, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(float));
  memset(diff_b, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(float));
  memset(diff_c, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(float));

  pMeas->avgPrDiffDis = -1.0;
  pMeas->prDiffStd_a = -1.0;

  if (pMeas->hasDiff == FALSE)
  {
    return;
  }

  Table_NumThres[0] = 6;
  Table_NumThres[1] = 8;
  Table_NumThres[2] = 12;

  /* determine the PR rejected limit */
  gnss_Pe_PRDR_Num(pMeas);
  if (pMeas->validPrNum <= 5)
  {
    prRejLimit = 0;
  }
  else if (pMeas->validPrNum <= 10)
  {
    prRejLimit = pMeas->validPrNum - 6;
  }
  else
  {
    prRejLimit = pMeas->validPrNum - 8;
  }

  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &pMeas->meas[i];
    if (pSvMeas->prn == 0 || pSvMeas->prDiffValid == 0) continue;
    if ((pSvMeas->status & 0x1) == 0) continue;

    diff[totalCnt] = (float)pSvMeas->pr_diff;
    indx[totalCnt++] = i;
  }

  if (g_pe_cfg.chipType == SPRD)
  {
    if (totalCnt < 6) return;
  }
  else
  {
    if (totalCnt < 8) return;
  }

  /* Calculate Distance */
  gnss_Calc_distance(diff, distance, totalCnt);

  /* Sorting according to distance */
  gnss_Sort_WithIndx_1(distance, diff, indx, totalCnt);

  //k-mean cluster
  //get the mean of each cluster using k-mean algorithm
  outlierFlag = FALSE;
  totalCntUse = totalCnt;

  if (totalCnt >= 8)
  {
    if ((distance[totalCnt - 1] - distance[totalCnt - 2]) > outlierThres)
    {
      outlierFlag = TRUE;
      totalCntUse = totalCnt - 1;
    }

    if ((distance[totalCnt - 2] - distance[totalCnt - 3]) > outlierThres)
    {
      outlierFlag = TRUE;
      totalCntUse = totalCnt - 2;
    }
    if (!outlierFlag && (((distance[totalCnt - 1] + distance[totalCnt - 2]) / 2.0 - distance[totalCnt - 3]) > outlierThres))
    {
      outlierFlag = TRUE;
      totalCntUse = totalCnt - 2;
    }
  }

  if (!gnss_kMean(totalCntUse, &distance[0], &min_indx[0], FALSE))
  {
    return;
  }
  //divide the samples into three clusters
  cnt_a = 0;
  cnt_b = 0;
  cnt_c = 0;

  for (i = 0; i < totalCntUse; i++)
  {
    if (min_indx[i] == 0)
    {
      distance_a[cnt_a] = distance[i];
      indx_a[cnt_a] = indx[i];
      diff_a[cnt_a] = diff[i];
      cnt_a++;
    }
    else if (min_indx[i] == 1)
    {
      distance_b[cnt_b] = distance[i];
      indx_b[cnt_b] = indx[i];
      diff_b[cnt_b] = diff[i];
      cnt_b++;
    }
    else
    {
      distance_c[cnt_c] = distance[i];
      indx_c[cnt_c] = indx[i];
      diff_c[cnt_c] = diff[i];
      cnt_c++;
    }
  }

  if (outlierFlag)
  {
    for (i = totalCntUse; i < totalCnt; i++)
    {
      distance_c[cnt_c] = distance[i];
      indx_c[cnt_c] = indx[i];
      diff_c[cnt_c] = diff[i];
      cnt_c++;
    }
  }

  if (cnt_a == 0)
  {
    return;
  }

  //Calculate average and std of culser A's distance
  gnss_math_fstd(distance_a, cnt_a, &avgDistance_a, &stdDis_a);

  /* calculate cluster A's PR diff average and std*/
  gnss_math_fstd(diff_a, cnt_a, &avgDiff_a, &stdDiff_a);

  pMeas->avgPrDiffDis = avgDistance_a;
  pMeas->prDiffStd_a = stdDiff_a;

#if defined(PLAYBACK_MODE)
  for (i = 0; i < cnt_a; i++)
  {
    SYS_LOGGING(OBJ_QOS, LOG_INFO, "PR cluster_a: %2d %3d %2d %10.4f %10.4f %2d %10.4f %10.4f", pMeas->meas[indx_a[i]].gnssMode, pMeas->meas[indx_a[i]].prn, pMeas->meas[indx_a[i]].cno, distance_a[i], diff_a[i], cnt_a, avgDistance_a, stdDis_a);
  }

  for (i = 0; i < cnt_b; i++)
  {
    SYS_LOGGING(OBJ_QOS, LOG_INFO, "PR cluster_b: %2d %3d %2d %10.4f %10.4f %2d", pMeas->meas[indx_b[i]].gnssMode, pMeas->meas[indx_b[i]].prn, pMeas->meas[indx_b[i]].cno, distance_b[i], diff_b[i], cnt_b);
  }

  for (i = 0; i < cnt_c; i++)
  {
    SYS_LOGGING(OBJ_QOS, LOG_INFO, "PR cluster_c: %2d %3d %2d %10.4f %10.4f %2d", pMeas->meas[indx_c[i]].gnssMode, pMeas->meas[indx_c[i]].prn, pMeas->meas[indx_c[i]].cno, distance_c[i], diff_c[i], cnt_c);
  }
#endif
  //decide to reject or deweight the measurement according to the cluster result
  gap_distance = distance[totalCnt - 1] - distance[0];
  if (gap_distance < 5.0)
  {
    return;
  }

  prRejNum = 0;

  //get pr reject threshold
  if (cnt_a <= Table_NumThres[0])
  {
    thres_ac = 20 * stdDis_a;
    min_thres = 20;
  }
  else if (cnt_a <= Table_NumThres[1])
  {
    thres_ac = 15 * stdDis_a;
    min_thres = 15;
  }
  else if (cnt_a <= Table_NumThres[2])
  {
    thres_ac = 10 * stdDis_a;
    min_thres = 10;
  }
  else
  {
    thres_ac = (float)(7.5 * stdDis_a);
    min_thres = 7.5;
  }

  if (cnt_c >= cnt_a)
  {
    thres_ac *= (2 + (cnt_c - cnt_a));
    min_thres *= (2 + (cnt_c - cnt_a));
  }

  if (thres_ac < min_thres)
  {
    thres_ac = min_thres;
  }
  else if (thres_ac > 100)
  {
    thres_ac = 100;
  }



  //get pr deweight threshold
  if (cnt_a <= Table_NumThres[0])
  {
    thres_ab = 15 * stdDis_a;
    min_thres = 15.0;
  }
  else if (cnt_a <= Table_NumThres[1])
  {
    thres_ab = 10 * stdDis_a;
    min_thres = 10.0;
  }
  else if (cnt_a <= Table_NumThres[2])
  {
    thres_ab = (float)(7.5 * stdDis_a);
    min_thres = 7.5;
  }
  else
  {
    thres_ab = (float)(5.0 * stdDis_a);
    min_thres = 5.0;
  }

  if (thres_ab < min_thres)
  {
    thres_ab = min_thres;
  }
  else if (thres_ab > 80)
  {
    thres_ab = 80;
  }

  if (p_state->posHasBias)
  {
    if (avgDistance_a > 150)
    {
      thres_ac *= 50;
      thres_ab *= 50;
    }
    else if (avgDistance_a > 100)
    {
      thres_ac *= 25;
      thres_ab *= 25;
    }
    else if (avgDistance_a > 50)
    {
      thres_ac *= 10;
      thres_ab *= 10;
    }
    else
    {
      thres_ac *= 5;
      thres_ab *= 5;
    }
  }

  SYS_LOGGING(OBJ_QOS, LOG_INFO, "PR cluster rej_thres:%10.4f ,deweight_thres:%10.4f ,diffStd of cluster a:%10.4f,", thres_ac, thres_ab, stdDiff_a);

  if (cnt_c > 0 && cnt_c < (cnt_a + cnt_b))
  {
    for (i = cnt_c; i > 0; i--)
    {
      pSvMeas = &pMeas->meas[indx_c[i - 1]];
      if ((pSvMeas->quality & PR_GOOD) && stdDiff_a > 2.0)
      {
        continue;
      }
      if (((distance_c[i - 1] - avgDistance_a) > thres_ac) && prRejNum < prRejLimit &&
        ((fabs(diff_c[i - 1]) > 20.0) || (stdDiff_a < 5.0)))
      {
        /* If PDOP enlarge two times after reject this satellite, then don't reject this satellite */
        if ((gnss_Pe_Dop_Check(pMeas, indx_c[i - 1], 1) == FALSE) && (
          distance_c[i - 1] - avgDistance_a) < 2 * thres_ac)
        {
          continue;
        }
        pSvMeas->status &= 0xFE;
        prRejNum++;

        sp = gnss_sd_get_sv_data(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);
        /*if(sp)
        {
        gnss_Qos_BadMeasFlag_Set(pMeas->tor,sp,MEAS_TYPE_PR);
        }*/
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was rejected in %s cluster_c: %10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_c[i - 1], pSvMeas->pr_diff);
      }
      else if (((distance_c[i - 1] - avgDistance_a) > thres_ab) &&
        ((fabs(diff_c[i - 1]) > 15.0) || (stdDiff_a < 5.0)) && cnt_c < cnt_a)
      {
        pSvMeas->prWeightChck |= CLUSTER_CHECK;
        sp = gnss_sd_get_sv_data(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);
        /*if(sp)
        {
        gnss_Qos_BadMeasFlag_Set(pMeas->tor,sp,MEAS_TYPE_PR);
        }*/
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was deweight in %s cluster_c: %10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_c[i - 1], pSvMeas->pr_diff);
      }

    }
  }

  //thres_ab = 20.0;
  if (cnt_b > 0 && cnt_b < cnt_a)
  {
    for (i = cnt_b; i > 0; i--)
    {
      pSvMeas = &pMeas->meas[indx_b[i - 1]];
      if (pSvMeas->quality & PR_GOOD)
      {
        continue;
      }
      if ((distance_b[i - 1] - avgDistance_a) > thres_ac && prRejNum < prRejLimit &&
        ((fabs(diff_b[i - 1]) > 20.0) || (stdDiff_a < 5.0)))
      {
        /* If PDOP enlarge two times after reject this satellite, then don't reject this satellite */
        if (gnss_Pe_Dop_Check(pMeas, indx_b[i - 1], 1) == FALSE)
        {
          continue;
        }
        pSvMeas->status &= 0xFE;
        prRejNum++;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was rejected in %s cluster_b: %10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_b[i - 1], pSvMeas->pr_diff);
      }
      else if ((distance_b[i - 1] - avgDistance_a) > thres_ab &&
        ((fabs(diff_b[i - 1]) > 10.0) || (stdDiff_a < 5.0)))
      {
        pSvMeas->prWeightChck |= CLUSTER_CHECK;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was deweight in %s cluster_b: %10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_b[i - 1], pSvMeas->pr_diff);
      }
    }
  }
#endif
}
/***********************************************************************
* 函数介绍: gnss_Qos_DR_Cluster_QCOM
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：12/16/
***********************************************************************/
static void gnss_Qos_DR_Cluster_QCOM(meas_blk_t* pMeas, Kf_t* p_Kf, PeStateMonitor_t* p_state)
{
  uint8_t             drRejLimit, drRejNum, outlierFlag, goodDrFlag = FALSE, goodDrCnt = 6, deThresFlag;
  uint8_t             indx[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 }, indx_a[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 }, indx_b[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 }, indx_c[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 }, min_indx[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 };
  uint32_t            i;
  uint32_t            totalCnt = 0, totalCntUse = 0, cnt_a = 0, cnt_b = 0, cnt_c = 0;
  float            avgDis_a = 0, stdDis_a = 0, avgDiff_a = 0, stdDiff_a = 0;
  float            diff[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, diff_a[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, diff_b[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, diff_c[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 };
  float            distance[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, distance_a[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, distance_b[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, distance_c[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 };
  float            gap_distance, thres_ac, thres_ab, min_thres, outlierThres = 3.0;
  gnss_meas_t* pSvMeas;

  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &pMeas->meas[i];
    if (pSvMeas->prn == 0) continue;
    if ((pSvMeas->status & 0x2) == 0) continue;

    if (pMeas->hasDiff == FALSE || pSvMeas->sv_info.eph_status == FALSE)
    {
      continue;
    }

    diff[totalCnt] = pSvMeas->dr_diff;
    indx[totalCnt] = i;
    totalCnt++;
  }

  if (totalCnt < 8) return;

  gnss_Sort_WithIndx(diff, indx, totalCnt);

  /* determine the DR rejected limit */
  gnss_Pe_PRDR_Num(pMeas);
  if (pMeas->validDrNum <= 5)
  {
    drRejLimit = 0;
  }
  else if (pMeas->validDrNum <= 10)
  {
    drRejLimit = pMeas->validDrNum - 6;
  }
  else
  {
    drRejLimit = pMeas->validDrNum - 8;
  }

  /* Calculate Distance */
  gnss_Calc_distance(diff, distance, totalCnt);

  /* Sorting according to distance */
  gnss_Sort_WithIndx_1(distance, diff, indx, totalCnt);

  //k-mean cluster
  //get the mean of each cluster using k-mean algorithm
  outlierFlag = FALSE;
  totalCntUse = totalCnt;

  if ((distance[totalCnt - 1] - distance[totalCnt - 2]) > outlierThres)
  {
    outlierFlag = TRUE;
    totalCntUse = totalCnt - 1;
  }

  if ((distance[totalCnt - 2] - distance[totalCnt - 3]) > outlierThres)
  {
    outlierFlag = TRUE;
    totalCntUse = totalCnt - 2;
  }

  if (!outlierFlag && ((((double)distance[totalCnt - 1] + distance[totalCnt - 2]) / 2.0 - distance[totalCnt - 3]) > outlierThres))
  {
    outlierFlag = TRUE;
    totalCntUse = totalCnt - 2;
  }

  if (!gnss_kMean(totalCntUse, &distance[0], &min_indx[0], FALSE))
  {
    return;
  }
  //divide the samples into three clusters
  cnt_a = 0;
  cnt_b = 0;
  cnt_c = 0;
  for (i = 0; i < totalCntUse; i++)
  {
    if (min_indx[i] == 0)
    {
      diff_a[cnt_a] = diff[i];
      distance_a[cnt_a] = distance[i];
      indx_a[cnt_a] = indx[i];
      cnt_a++;
    }
    else if (min_indx[i] == 1)
    {
      diff_b[cnt_b] = diff[i];
      distance_b[cnt_b] = distance[i];
      indx_b[cnt_b] = indx[i];
      cnt_b++;
    }
    else
    {
      indx_c[cnt_c] = indx[i];
      diff_c[cnt_c] = diff[i];
      distance_c[cnt_c] = distance[i];
      cnt_c++;
    }
  }

  if (outlierFlag)
  {
    for (i = totalCntUse; i < totalCnt; i++)
    {
      distance_c[cnt_c] = distance[i];
      indx_c[cnt_c] = indx[i];
      diff_c[cnt_c] = diff[i];
      cnt_c++;
    }
  }

  if (cnt_a == 0)
  {
    return;
  }

  //Calculate average and std of culser A's distance
  gnss_math_fstd(distance_a, cnt_a, &avgDis_a, &stdDis_a);

  /* calculate cluster A's PR diff average and std*/
  gnss_math_fstd(diff_a, cnt_a, &avgDiff_a, &stdDiff_a);

  for (i = 0; i < cnt_a; i++)
  {
    SYS_LOGGING(OBJ_QOS, LOG_DEBUG, "DR cluster_a: %2d %3d %2d %10.4f %10.4f %2d %10.4f %10.4f",
      pMeas->meas[indx_a[i]].gnssMode, pMeas->meas[indx_a[i]].prn, pMeas->meas[indx_a[i]].cno,
      distance_a[i], diff_a[i], cnt_a, avgDis_a, stdDis_a);
  }

  for (i = 0; i < cnt_b; i++)
  {
    SYS_LOGGING(OBJ_QOS, LOG_DEBUG, "DR cluster_b: %2d %3d %2d %10.4f %10.4f %2d",
      pMeas->meas[indx_b[i]].gnssMode, pMeas->meas[indx_b[i]].prn, pMeas->meas[indx_b[i]].cno,
      distance_b[i], diff_b[i], cnt_b);
  }

  for (i = 0; i < cnt_c; i++)
  {
    SYS_LOGGING(OBJ_QOS, LOG_DEBUG, "DR cluster_c: %2d %3d %2d %10.4f %10.4f %2d",
      pMeas->meas[indx_c[i]].gnssMode, pMeas->meas[indx_c[i]].prn, pMeas->meas[indx_c[i]].cno,
      distance_c[i], diff_c[i], cnt_c);
  }

  //decide to reject or deweight the measurment according to the cluster result
  gap_distance = distance[totalCnt - 1] - distance[0];
  if (gap_distance < 0.5)
  {
    return;
  }

  //five DIFF of cluster A is small
  if (cnt_a >= goodDrCnt)
  {
    goodDrFlag = TRUE;
    for (i = 0; i < goodDrCnt; i++)
    {
      if (fabs(diff_a[i]) >= 1.0)
      {
        goodDrFlag = FALSE;
        break;
      }
    }
  }

  //if last kf vel res is small and five DIFF of cluster A is small
  // just decrease cluster threshold
  if (p_Kf->kf_Pvt_Info->kfFixStatus == FIX_STATUS_NEW && p_Kf->velRes < 0.75 &&
    p_Kf->velResStd>0 && p_Kf->velResStd < 0.75 && goodDrFlag)
  {
    deThresFlag = TRUE;
  }
  else
  {
    deThresFlag = FALSE;
  }
  //get dr reject threshold
  drRejNum = 0;
  if (cnt_a <= 8)
  {
    thres_ac = (float)(15.0 * stdDis_a);
    min_thres = 1.5;
  }
  else if (cnt_a <= 10)
  {
    thres_ac = (float)(12.5 * stdDis_a);
    min_thres = 1.0;
  }
  else if (cnt_a <= 15)
  {
    thres_ac = (float)(7.5 * stdDis_a);
    min_thres = 0.75;
  }
  else if (cnt_a <= 20)
  {
    thres_ac = (float)(5.0 * stdDis_a);
    min_thres = 0.5;
  }
  else
  {
    thres_ac = (float)(2.5 * stdDis_a);
    min_thres = 0.25;
  }

  if (deThresFlag && thres_ac > min_thres)
  {
    thres_ac -= (float)(2.5 * stdDis_a);
    SYS_LOGGING(OBJ_QOS, LOG_INFO, "DR cluster detect last good velocity,then decrease cluster thres: %8.3f", pMeas->tor);
  }

  if (cnt_c >= cnt_a)
  {
    thres_ac *= (2 + (cnt_c - cnt_a));
    min_thres *= (2 + (cnt_c - cnt_a));
  }

  if (thres_ac < min_thres)
  {
    thres_ac = min_thres;
  }
  else if (thres_ac > 10.0)
  {
    thres_ac = 10;
  }
  //get dr deweight threshold
  if (cnt_a <= 8)
  {
    thres_ab = (float)(10.0 * stdDis_a);
    min_thres = 1.0;
  }
  else if (cnt_a <= 10)
  {
    thres_ab = (float)(7.5 * stdDis_a);
    min_thres = 0.75;
  }
  else if (cnt_a <= 15)
  {
    thres_ab = (float)(5.0 * stdDis_a);
    min_thres = 0.5;
  }
  else if (cnt_a <= 20)
  {
    thres_ab = (float)(2.5 * stdDis_a);
    min_thres = 0.25;
  }
  else
  {
    thres_ab = (float)(1.5 * stdDis_a);
    min_thres = (float)0.15;
  }

  if (deThresFlag)
  {
    thres_ab -= (float)(1.5 * stdDis_a);
  }

  if (thres_ab < min_thres)
  {
    thres_ab = min_thres;
  }
  else if (thres_ab > 10.0)
  {
    thres_ab = 10;
  }

  SYS_LOGGING(OBJ_QOS, LOG_INFO, "DR cluster rej_thres : %10.4f deweight_thres : %10.4f, stdDiff_a: %10.4f", thres_ac, thres_ab, stdDiff_a);

  if (cnt_c > 0 && cnt_c < (cnt_a + cnt_b))
  {
    for (i = cnt_c; i > 0; i--)
    {
      pSvMeas = &pMeas->meas[indx_c[i - 1]];
      if (pSvMeas->quality & DR_GOOD) continue;

      if (((distance_c[i - 1] - avgDis_a) > thres_ac) && drRejNum < drRejLimit)
      {
        /* If PDOP enlarge two times after reject this satellite, then don't reject this satellite */
        if (gnss_Pe_Dop_Check(pMeas, indx_c[i - 1], 2) == FALSE &&
          (distance_c[i - 1] - avgDis_a) < 2 * thres_ac)
        {
          /*				pSvMeas->drWeightChck  |= CLUSTER_CHECK;
          SYS_LOGGING(OBJ_QOS,LOG_INFO,"gnssMode(%02d)prn(%03d) was deweight in %s DR cluster_c: %10.4f,%10.4f",
          pSvMeas->gnssMode,pSvMeas->prn,__FUNCTION__,distance_c[i-1],pSvMeas->dr_diff);*/
          continue;
        }
        pSvMeas->status &= 0xFD;
        drRejNum++;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was rejected in %s DR cluster_c: %10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_c[i - 1], pSvMeas->dr_diff);
      }
      else if (((distance_c[i - 1] - avgDis_a) > thres_ab) && cnt_c < cnt_a)
      {
        pSvMeas->drWeightChck |= CLUSTER_CHECK;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was deweight in %s DR cluster_c: %10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_c[i - 1], pSvMeas->dr_diff);
      }

    }
  }


  if (cnt_b > 0 && cnt_b < cnt_a)
  {
    for (i = cnt_b; i > 0; i--)
    {
      pSvMeas = &pMeas->meas[indx_b[i - 1]];
      if (pSvMeas->quality & DR_GOOD) continue;

      if ((distance_b[i - 1] - avgDis_a) > thres_ac && drRejNum < drRejLimit)
      {
        /* If PDOP enlarge two times after reject this satellite, then don't reject this satellite */
        if (gnss_Pe_Dop_Check(pMeas, indx_b[i - 1], 2) == FALSE)
        {
          continue;
        }
        pSvMeas->status &= 0xFD;
        drRejNum++;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was rejected in %s DR cluster_b: %10.4f,%10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_b[i - 1], pSvMeas->dr_diff, thres_ac);
      }
      else if ((distance_b[i - 1] - avgDis_a) > thres_ab)
      {
        pSvMeas->drWeightChck |= CLUSTER_CHECK;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was deweight in %s DR cluster_b: %10.4f,%10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_b[i - 1], pSvMeas->dr_diff, thres_ab);
      }
    }
  }
}
/***********************************************************************
* 函数介绍: gnss_Qos_DR_Cluster_QCOM
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：12/16/
***********************************************************************/
static void gnss_Qos_DR_Cluster_QCOM_WALK(meas_blk_t* pMeas, Kf_t* p_Kf, PeStateMonitor_t* p_state)
{
  uint8_t             drRejLimit, drRejNum, outlierFlag, goodDrFlag = FALSE, goodDrCnt = 6, deThresFlag;
  uint8_t             indx[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 }, /*indx_a[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 },*/ indx_b[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 }, indx_c[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 }, min_indx[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 };
  uint32_t            i;
  uint32_t            totalCnt = 0, totalCntUse = 0, cnt_a = 0, cnt_b = 0, cnt_c = 0;
  float            avgDis_a = 0, stdDis_a = 0, avgDiff_a = 0, stdDiff_a = 0;
  float            diff[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, diff_a[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }/*, diff_b[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, diff_c[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }*/;
  float            distance[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, distance_a[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, distance_b[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, distance_c[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 };
  float            gap_distance, thres_ac, thres_ab, min_thres, outlierThres = 3.0;
  gnss_meas_t* pSvMeas;

  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &pMeas->meas[i];
    if (pSvMeas->prn == 0) continue;
    if ((pSvMeas->status & 0x2) == 0) continue;

    if (pMeas->hasDiff == FALSE || pSvMeas->sv_info.eph_status == FALSE)
    {
      continue;
    }

    diff[totalCnt] = pSvMeas->dr_diff;
    indx[totalCnt] = i;
    totalCnt++;
  }

  if (totalCnt < 8) return;

  gnss_Sort_WithIndx(diff, indx, totalCnt);

  /* determine the DR rejected limit */
  gnss_Pe_PRDR_Num(pMeas);
  if (pMeas->validDrNum <= 5)
  {
    drRejLimit = 0;
  }
  else if (pMeas->validDrNum <= 10)
  {
    drRejLimit = pMeas->validDrNum - 6;
  }
  else
  {
    drRejLimit = pMeas->validDrNum - 8;
  }

  /* Calculate Distance */
  gnss_Calc_distance(diff, distance, totalCnt);

  /* Sorting according to distance */
  gnss_Sort_WithIndx_1(distance, diff, indx, totalCnt);

  //k-mean cluster
  //get the mean of each cluster using k-mean algorithm
  outlierFlag = FALSE;
  totalCntUse = totalCnt;

  if ((distance[totalCnt - 1] - distance[totalCnt - 2]) > outlierThres)
  {
    outlierFlag = TRUE;
    totalCntUse = totalCnt - 1;
  }

  if ((distance[totalCnt - 2] - distance[totalCnt - 3]) > outlierThres)
  {
    outlierFlag = TRUE;
    totalCntUse = totalCnt - 2;
  }

  if (!outlierFlag && ((((double)distance[totalCnt - 1] + distance[totalCnt - 2]) / 2.0 - distance[totalCnt - 3]) > outlierThres))
  {
    outlierFlag = TRUE;
    totalCntUse = totalCnt - 2;
  }

  if (!gnss_kMean(totalCntUse, &distance[0], &min_indx[0], FALSE))
  {
    return;
  }
  //divide the samples into three clusters
  cnt_a = 0;
  cnt_b = 0;
  cnt_c = 0;
  for (i = 0; i < totalCntUse; i++)
  {
    if (min_indx[i] == 0)
    {
      diff_a[cnt_a] = diff[i];
      distance_a[cnt_a] = distance[i];
      //indx_a[cnt_a] = indx[i];
      cnt_a++;
    }
    else if (min_indx[i] == 1)
    {
      //diff_b[cnt_b] = diff[i];
      distance_b[cnt_b] = distance[i];
      indx_b[cnt_b] = indx[i];
      cnt_b++;
    }
    else
    {
      indx_c[cnt_c] = indx[i];
      //diff_c[cnt_c] = diff[i];
      distance_c[cnt_c] = distance[i];
      cnt_c++;
    }
  }

  if (outlierFlag)
  {
    for (i = totalCntUse; i < totalCnt; i++)
    {
      distance_c[cnt_c] = distance[i];
      indx_c[cnt_c] = indx[i];
      //diff_c[cnt_c] = diff[i];
      cnt_c++;
    }
  }

  if (cnt_a == 0)
  {
    return;
  }

  //Calculate average and std of culser A's distance
  gnss_math_fstd(distance_a, cnt_a, &avgDis_a, &stdDis_a);

  /* calculate cluster A's PR diff average and std*/
  gnss_math_fstd(diff_a, cnt_a, &avgDiff_a, &stdDiff_a);

  //decide to reject or deweight the measurment according to the cluster result
  gap_distance = distance[totalCnt - 1] - distance[0];
  if (gap_distance < 0.1)
  {
    return;
  }

  //five DIFF of cluster A is small
  if (cnt_a >= goodDrCnt)
  {
    goodDrFlag = TRUE;
    for (i = 0; i < goodDrCnt; i++)
    {
      if (fabs(diff_a[i]) >= 1.0)
      {
        goodDrFlag = FALSE;
        break;
      }
    }
  }

  //if last kf vel res is small and five DIFF of cluster A is small
  // just decrease cluster threshold
  if (p_Kf->kf_Pvt_Info->kfFixStatus == FIX_STATUS_NEW && p_Kf->velRes < 0.75 &&
    p_Kf->velResStd>0 && p_Kf->velResStd < 0.75 && goodDrFlag)
  {
    deThresFlag = TRUE;
  }
  else
  {
    deThresFlag = FALSE;
  }
  //get dr reject threshold
  drRejNum = 0;
  if (cnt_a <= 8)
  {
    thres_ac = (float)(15.0 * stdDis_a);
    min_thres = 1.5;
  }
  else if (cnt_a <= 10)
  {
    thres_ac = (float)(12.5 * stdDis_a);
    min_thres = 1.0;
  }
  else if (cnt_a <= 15)
  {
    thres_ac = (float)(7.5 * stdDis_a);
    min_thres = 0.5;
  }
  else if (cnt_a <= 20)
  {
    thres_ac = (float)(5.0 * stdDis_a);
    min_thres = 0.25;
  }
  else
  {
    thres_ac = (float)(2.5 * stdDis_a);
    min_thres = 0.15f;
  }

  if (deThresFlag && thres_ac > min_thres)
  {
    thres_ac -= (float)(2.5 * stdDis_a);
    SYS_LOGGING(OBJ_QOS, LOG_INFO, "DR cluster detect last good velocity,then decrease cluster thres: %8.3f", pMeas->tor);
  }

  if (cnt_c >= cnt_a)
  {
    thres_ac *= (2 + (cnt_c - cnt_a));
    min_thres *= (2 + (cnt_c - cnt_a));
  }

  if (thres_ac < min_thres)
  {
    thres_ac = min_thres;
  }
  else if (thres_ac > 10.0)
  {
    thres_ac = 10;
  }
  //get dr deweight threshold
  if (cnt_a <= 8)
  {
    thres_ab = (float)(10.0 * stdDis_a);
    min_thres = 1.0;
  }
  else if (cnt_a <= 10)
  {
    thres_ab = (float)(7.5 * stdDis_a);
    min_thres = 0.75;
  }
  else if (cnt_a <= 15)
  {
    thres_ab = (float)(5.0 * stdDis_a);
    min_thres = 0.4f;
  }
  else if (cnt_a <= 20)
  {
    thres_ab = (float)(2.5 * stdDis_a);
    min_thres = 0.2f;
  }
  else
  {
    thres_ab = (float)(1.5 * stdDis_a);
    min_thres = (float)0.1;
  }

  if (deThresFlag)
  {
    thres_ab -= (float)(1.5 * stdDis_a);
  }

  if (thres_ab < min_thres)
  {
    thres_ab = min_thres;
  }
  else if (thres_ab > 10.0)
  {
    thres_ab = 10;
  }

  SYS_LOGGING(OBJ_QOS, LOG_INFO, "DR cluster rej_thres : %10.4f deweight_thres : %10.4f, stdDiff_a: %10.4f", thres_ac, thres_ab, stdDiff_a);

  if (cnt_c > 0 && cnt_c < (cnt_a + cnt_b))
  {
    for (i = cnt_c; i > 0; i--)
    {
      pSvMeas = &pMeas->meas[indx_c[i - 1]];
      if (pSvMeas->quality & DR_GOOD) continue;

      if (((distance_c[i - 1] - avgDis_a) > thres_ac) && drRejNum < drRejLimit)
      {
        /* If PDOP enlarge two times after reject this satellite, then don't reject this satellite */
        if (gnss_Pe_Dop_Check(pMeas, indx_c[i - 1], 2) == FALSE &&
          (distance_c[i - 1] - avgDis_a) < 2 * thres_ac)
        {
          /*				pSvMeas->drWeightChck  |= CLUSTER_CHECK;
          SYS_LOGGING(OBJ_QOS,LOG_INFO,"gnssMode(%02d)prn(%03d) was deweight in %s DR cluster_c: %10.4f,%10.4f",
          pSvMeas->gnssMode,pSvMeas->prn,__FUNCTION__,distance_c[i-1],pSvMeas->dr_diff);*/
          continue;
        }
        pSvMeas->status &= 0xFD;
        drRejNum++;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was rejected in %s DR cluster_c: %10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_c[i - 1], pSvMeas->dr_diff);
      }
      else if (((distance_c[i - 1] - avgDis_a) > thres_ab) && cnt_c < cnt_a)
      {
        pSvMeas->drWeightChck |= CLUSTER_CHECK;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was deweight in %s DR cluster_c: %10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_c[i - 1], pSvMeas->dr_diff);
      }

    }
  }


  if (cnt_b > 0 && cnt_b < cnt_a)
  {
    for (i = cnt_b; i > 0; i--)
    {
      pSvMeas = &pMeas->meas[indx_b[i - 1]];
      if (pSvMeas->quality & DR_GOOD) continue;

      if ((distance_b[i - 1] - avgDis_a) > thres_ac && drRejNum < drRejLimit)
      {
        /* If PDOP enlarge two times after reject this satellite, then don't reject this satellite */
        if (gnss_Pe_Dop_Check(pMeas, indx_b[i - 1], 2) == FALSE)
        {
          continue;
        }
        pSvMeas->status &= 0xFD;
        drRejNum++;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was rejected in %s DR cluster_b: %10.4f,%10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_b[i - 1], pSvMeas->dr_diff, thres_ac);
      }
      else if ((distance_b[i - 1] - avgDis_a) > thres_ab)
      {
        pSvMeas->drWeightChck |= CLUSTER_CHECK;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was deweight in %s DR cluster_b: %10.4f,%10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_b[i - 1], pSvMeas->dr_diff, thres_ab);
      }
    }
  }
}
/***********************************************************************
* 函数介绍: gnss_Qos_DR_Cluster_UBLOX
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：12/16/
***********************************************************************/
static void gnss_Qos_DR_Cluster_UBLOX(meas_blk_t* pMeas, Kf_t* p_Kf, PeStateMonitor_t* p_state)
{
  uint8_t             drRejLimit, drRejNum, outlierFlag, goodDrFlag = FALSE, goodDrCnt = 6, deThresFlag;
  uint8_t             indx[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 }, indx_a[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 }, indx_b[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 }, indx_c[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 }, min_indx[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 };
  uint8_t             Table_NumThres[4] = { 0 };
  uint32_t            i;
  uint32_t            totalCnt = 0, totalCntUse = 0, cnt_a = 0, cnt_b = 0, cnt_c = 0;
  float            avgDis_a = 0, stdDis_a = 0, avgDiff_a = 0, stdDiff_a = 0, avgDis_b = 0, stdDis_b = 0;
  float            diff[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, diff_a[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, diff_b[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, diff_c[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 };
  float            distance[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, distance_a[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, distance_b[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, distance_c[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 };
  float            gap_distance, thres_ac, thres_ab, min_thres, outlierThres = 3.0, ratio = 1.0, dr_thres = 0.5;
  gnss_meas_t* pSvMeas;

  Table_NumThres[0] = 8;
  Table_NumThres[1] = 10;
  Table_NumThres[2] = 15;
  Table_NumThres[3] = 20;

  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &pMeas->meas[i];
    if (pSvMeas->prn == 0) continue;
    if ((pSvMeas->status & 0x2) == 0) continue;

    if (pMeas->hasDiff == FALSE || pSvMeas->sv_info.eph_status == FALSE)
    {
      continue;
    }

    diff[totalCnt] = pSvMeas->dr_diff;
    indx[totalCnt] = i;
    totalCnt++;
  }

  if (totalCnt < 6) return;

  gnss_Sort_WithIndx(diff, indx, totalCnt);

  /* determine the DR rejected limit */
  gnss_Pe_PRDR_Num(pMeas);
  if (pMeas->validDrNum <= 5)
  {
    drRejLimit = 0;
  }
  else
  {
    drRejLimit = pMeas->validDrNum - 5;
  }
  /* Calculate Distance */
  gnss_Calc_distance(diff, distance, totalCnt);

  /* Sorting according to distance */
  gnss_Sort_WithIndx_1(distance, diff, indx, totalCnt);

  //k-mean cluster
  //get the mean of each cluster using k-mean algorithm
  outlierFlag = FALSE;
  totalCntUse = totalCnt;

  if ((distance[totalCnt - 1] - distance[totalCnt - 2]) > outlierThres)
  {
    outlierFlag = TRUE;
    totalCntUse = totalCnt - 1;
  }

  if ((distance[totalCnt - 2] - distance[totalCnt - 3]) > outlierThres)
  {
    outlierFlag = TRUE;
    totalCntUse = totalCnt - 2;
  }

  if (!outlierFlag && ((((double)distance[totalCnt - 1] + distance[totalCnt - 2]) / 2.0 - distance[totalCnt - 3]) > outlierThres))
  {
    outlierFlag = TRUE;
    totalCntUse = totalCnt - 2;
  }

  if (!gnss_kMean(totalCntUse, &distance[0], &min_indx[0], FALSE))
  {
    return;
  }
  //divide the samples into three clusters
  cnt_a = 0;
  cnt_b = 0;
  cnt_c = 0;
  for (i = 0; i < totalCntUse; i++)
  {
    if (min_indx[i] == 0)
    {
      diff_a[cnt_a] = diff[i];
      distance_a[cnt_a] = distance[i];
      indx_a[cnt_a] = indx[i];
      pMeas->meas[indx_a[cnt_a]].drClusterA = 0x1;
      cnt_a++;
    }
    else if (min_indx[i] == 1)
    {
      diff_b[cnt_b] = diff[i];
      distance_b[cnt_b] = distance[i];
      indx_b[cnt_b] = indx[i];
      cnt_b++;
    }
    else
    {
      indx_c[cnt_c] = indx[i];
      diff_c[cnt_c] = diff[i];
      distance_c[cnt_c] = distance[i];
      cnt_c++;
    }
  }

  if (outlierFlag)
  {
    for (i = totalCntUse; i < totalCnt; i++)
    {
      distance_c[cnt_c] = distance[i];
      indx_c[cnt_c] = indx[i];
      diff_c[cnt_c] = diff[i];
      cnt_c++;
    }
  }

  if (cnt_a == 0)
  {
    return;
  }

  //Calculate average and std of culser A's distance
  gnss_math_fstd(distance_a, cnt_a, &avgDis_a, &stdDis_a);

  //Calculate average and std of culser B's distance
  gnss_math_fstd(distance_b, cnt_b, &avgDis_b, &stdDis_b);
  /* calculate cluster A's PR diff average and std*/
  gnss_math_fstd(diff_a, cnt_a, &avgDiff_a, &stdDiff_a);

  for (i = 0; i < cnt_a; i++)
  {
    SYS_LOGGING(OBJ_QOS, LOG_DEBUG, "DR cluster_a: %2d %3d %2d %10.4f %10.4f %2d %10.4f %10.4f", 
      pMeas->meas[indx_a[i]].gnssMode, pMeas->meas[indx_a[i]].prn, pMeas->meas[indx_a[i]].cno, distance_a[i], diff_a[i], cnt_a, avgDis_a, stdDis_a);
  }

  for (i = 0; i < cnt_b; i++)
  {
    SYS_LOGGING(OBJ_QOS, LOG_DEBUG, "DR cluster_b: %2d %3d %2d %10.4f %10.4f %2d", 
      pMeas->meas[indx_b[i]].gnssMode, pMeas->meas[indx_b[i]].prn, pMeas->meas[indx_b[i]].cno, distance_b[i], diff_b[i], cnt_b);
  }

  for (i = 0; i < cnt_c; i++)
  {
    SYS_LOGGING(OBJ_QOS, LOG_DEBUG, "DR cluster_c: %2d %3d %2d %10.4f %10.4f %2d", 
      pMeas->meas[indx_c[i]].gnssMode, pMeas->meas[indx_c[i]].prn, pMeas->meas[indx_c[i]].cno, distance_c[i], diff_c[i], cnt_c);
  }

  //decide to reject or deweight the measurment according to the cluster result
  gap_distance = distance[totalCnt - 1] - distance[0];
  if (cnt_a >= 15 && gap_distance < 0.5 && avgDis_a < 0.5)
  {
    dr_thres = 0.4f;
  }
  else
  {
    dr_thres = 0.5;
  }

  if (gap_distance < dr_thres)
  {
    return;
  }

  //five DIFF of cluster A is small
  if (cnt_a >= goodDrCnt)
  {
    goodDrFlag = TRUE;
    for (i = 0; i < goodDrCnt; i++)
    {
      if (fabs(diff_a[i]) >= 1.0)
      {
        goodDrFlag = FALSE;
        break;
      }
    }
  }

  //if last kf vel res is small and five DIFF of cluster A is small
  // just decrease cluster threshold
  if (p_Kf->kf_Pvt_Info->kfFixStatus == FIX_STATUS_NEW && p_Kf->velRes < 0.75 &&
    p_Kf->velResStd>0 && p_Kf->velResStd < 0.75 && goodDrFlag)
  {
    deThresFlag = TRUE;
  }
  else
  {
    deThresFlag = FALSE;
  }
  //get dr reject threshold
  drRejNum = 0;
  if (cnt_a <= Table_NumThres[0])
  {
    thres_ac = (float)(15.0 * stdDis_a);
    min_thres = 1.5;
  }
  else if (cnt_a <= Table_NumThres[1])
  {
    thres_ac = (float)(12.5 * stdDis_a);
    min_thres = 1.0;
  }
  else if (cnt_a <= Table_NumThres[2])
  {
    thres_ac = (float)(7.5 * stdDis_a);
    min_thres = 0.75;
  }
  else if (cnt_a <= Table_NumThres[3])
  {
    thres_ac = (float)(5.0 * stdDis_a);
    min_thres = 0.3f;
  }
  else
  {
    if (p_Kf->kfCnt > 20)
    {
      thres_ac = (float)(3.0 * stdDis_a);
      min_thres = 0.3f;
    }
    else
    {
      thres_ac = (float)(5.0 * stdDis_a);
      min_thres = 0.3f;
    }
  }

  if (deThresFlag && thres_ac > min_thres)
  {
    thres_ac -= (float)(2.5 * stdDis_a);
    SYS_LOGGING(OBJ_QOS, LOG_INFO, "DR cluster detect last good velocity,then decrease cluster thres: %8.3f", pMeas->tor);
  }

  if (cnt_c >= cnt_a)
  {
    ratio = (float)(2 + (cnt_c - cnt_a));
    thres_ac *= ratio;
    min_thres *= ratio;
  }

  if (thres_ac < min_thres)
  {
    thres_ac = min_thres;
  }
  else if (thres_ac > 10.0)
  {
    thres_ac = 10;
  }
  //get dr deweight threshold
  if (cnt_a <= Table_NumThres[0])
  {
    thres_ab = (float)(10.0 * stdDis_a);
    min_thres = 1.0;
  }
  else if (cnt_a <= Table_NumThres[1])
  {
    thres_ab = (float)(7.5 * stdDis_a);
    min_thres = 0.75;
  }
  else if (cnt_a <= Table_NumThres[2])
  {
    thres_ab = (float)(5.0 * stdDis_a);
    min_thres = 0.5;
  }
  else if (cnt_a <= Table_NumThres[3])
  {
    thres_ab = (float)(3.0 * stdDis_a);
    min_thres = 0.3f;
  }
  else
  {
    if (p_Kf->kfCnt > 20)
    {
      thres_ab = (float)(2.0 * stdDis_a);
      min_thres = 0.2f;
    }
    else
    {
      thres_ab = (float)(3.0 * stdDis_a);
      min_thres = 0.3f;
    }
  }

  if (deThresFlag)
  {
    thres_ab -= (float)(1.5 * stdDis_a);
  }

  if (thres_ab < min_thres)
  {
    thres_ab = min_thres;
  }
  else if (thres_ab > 10.0)
  {
    thres_ab = 10;
  }

  SYS_LOGGING(OBJ_QOS, LOG_INFO, "DR cluster rej_thres : %10.4f deweight_thres : %10.4f, stdDiff_a: %10.4f", thres_ac, thres_ab, stdDiff_a);

  if (cnt_c > 0 && cnt_c <= (cnt_a + cnt_b))
  {
    for (i = cnt_c; i > 0; i--)
    {
      pSvMeas = &pMeas->meas[indx_c[i - 1]];
      if (pSvMeas->quality & DR_GOOD) continue;

      if (((distance_c[i - 1] - avgDis_a) > thres_ac) && drRejNum < drRejLimit)
      {
        /* If PDOP enlarge two times after reject this satellite, then don't reject this satellite */
        if (gnss_Pe_Dop_Check(pMeas, indx_c[i - 1], 2) == FALSE &&
          (distance_c[i - 1] - avgDis_a) < 2 * thres_ac)
        {
          continue;
        }
        if (deThresFlag && pSvMeas->cno >= (pMeas->avgCno + 5) && (distance_c[i - 1] - avgDis_a) < 4 * thres_ac)
        {
          continue;
        }
        pSvMeas->status &= 0xFD;
        drRejNum++;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was rejected in %s DR cluster_c: %10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_c[i - 1], pSvMeas->dr_diff);
      }
      else if (((distance_c[i - 1] - avgDis_a) > thres_ab) && cnt_c < cnt_a)
      {
        pSvMeas->drWeightChck |= CLUSTER_CHECK;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was deweight in %s DR cluster_c: %10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_c[i - 1], pSvMeas->dr_diff);
      }

    }
  }


  if (cnt_b > 0 && cnt_b < cnt_a)
  {
    for (i = cnt_b; i > 0; i--)
    {
      pSvMeas = &pMeas->meas[indx_b[i - 1]];
      if (pSvMeas->quality & DR_GOOD) continue;

      if ((distance_b[i - 1] - avgDis_a) > thres_ac && drRejNum < drRejLimit)
      {
        /* If PDOP enlarge two times after reject this satellite, then don't reject this satellite */
        if (gnss_Pe_Dop_Check(pMeas, indx_b[i - 1], 2) == FALSE)
        {
          continue;
        }
        if (deThresFlag && pSvMeas->cno >= (pMeas->avgCno + 5) && (avgDis_b - avgDis_a) < thres_ac)
        {
          continue;
        }
        pSvMeas->status &= 0xFD;
        drRejNum++;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was rejected in %s DR cluster_b: %10.4f,%10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_b[i - 1], pSvMeas->dr_diff, thres_ac);
      }
      else if ((distance_b[i - 1] - avgDis_a) > thres_ab)
      {
        pSvMeas->drWeightChck |= CLUSTER_CHECK;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was deweight in %s DR cluster_b: %10.4f,%10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_b[i - 1], pSvMeas->dr_diff, thres_ab);
      }
    }
  }
}
/***********************************************************************
* 函数介绍: gnss_Qos_DR_Cluster_SPRD
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：12/16/
***********************************************************************/
static void gnss_Qos_DR_Cluster_SPRD(meas_blk_t* pMeas, Kf_t* p_Kf, PeStateMonitor_t* p_state)
{
#ifdef USED_IN_MC262M
  uint8_t             drRejLimit, drRejNum, outlierFlag, goodDrFlag = FALSE, goodDrCnt = 6, deThresFlag;
  uint8_t             indx[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 }, indx_a[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 }, indx_b[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 }, indx_c[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 }, min_indx[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 };
  uint32_t            i;
  uint32_t            totalCnt = 0, totalCntUse = 0, cnt_a = 0, cnt_b = 0, cnt_c = 0;
  float            avgDis_a = 0, stdDis_a = 0, avgDiff_a = 0, stdDiff_a = 0;
  float            diff[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, diff_a[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, diff_b[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, diff_c[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 };
  float            distance[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, distance_a[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, distance_b[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, distance_c[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 };
  float            gap_distance, thres_ac, thres_ab, min_thres, outlierThres = 3.0;
  gnss_meas_t* pSvMeas;

  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &pMeas->meas[i];
    if (pSvMeas->prn == 0) continue;
    if ((pSvMeas->status & 0x2) == 0) continue;

    if (pMeas->hasDiff == FALSE || pSvMeas->sv_info.eph_status == FALSE)
    {
      continue;
    }

    diff[totalCnt] = pSvMeas->dr_diff;
    indx[totalCnt] = i;
    totalCnt++;
  }

  if (totalCnt < 6) return;

  gnss_Sort_WithIndx(diff, indx, totalCnt);

  /* determine the DR rejected limit */
  gnss_Pe_PRDR_Num(pMeas);
  if (pMeas->validDrNum <= 5)
  {
    drRejLimit = 0;
  }
  else if (pMeas->validDrNum <= 10)
  {
    drRejLimit = pMeas->validDrNum - 6;
  }
  else
  {
    drRejLimit = pMeas->validDrNum - 8;
  }

  /* Calculate Distance */
  gnss_Calc_distance(diff, distance, totalCnt);

  /* Sorting according to distance */
  gnss_Sort_WithIndx_1(distance, diff, indx, totalCnt);

  //k-mean cluster
  //get the mean of each cluster using k-mean algorithm
  outlierFlag = FALSE;
  totalCntUse = totalCnt;

  if ((distance[totalCnt - 1] - distance[totalCnt - 2]) > outlierThres)
  {
    outlierFlag = TRUE;
    totalCntUse = totalCnt - 1;
  }

  if ((distance[totalCnt - 2] - distance[totalCnt - 3]) > outlierThres)
  {
    outlierFlag = TRUE;
    totalCntUse = totalCnt - 2;
  }

  if (!outlierFlag && (((distance[totalCnt - 1] + distance[totalCnt - 2]) / 2.0 - distance[totalCnt - 3]) > outlierThres))
  {
    outlierFlag = TRUE;
    totalCntUse = totalCnt - 2;
  }

  if (!gnss_kMean(totalCntUse, &distance[0], &min_indx[0], FALSE))
  {
    return;
  }
  //divide the samples into three clusters
  cnt_a = 0;
  cnt_b = 0;
  cnt_c = 0;
  for (i = 0; i < totalCntUse; i++)
  {
    if (min_indx[i] == 0)
    {
      diff_a[cnt_a] = diff[i];
      distance_a[cnt_a] = distance[i];
      indx_a[cnt_a] = indx[i];
      cnt_a++;
    }
    else if (min_indx[i] == 1)
    {
      diff_b[cnt_b] = diff[i];
      distance_b[cnt_b] = distance[i];
      indx_b[cnt_b] = indx[i];
      cnt_b++;
    }
    else
    {
      indx_c[cnt_c] = indx[i];
      diff_c[cnt_c] = diff[i];
      distance_c[cnt_c] = distance[i];
      cnt_c++;
    }
  }

  if (outlierFlag)
  {
    for (i = totalCntUse; i < totalCnt; i++)
    {
      distance_c[cnt_c] = distance[i];
      indx_c[cnt_c] = indx[i];
      diff_c[cnt_c] = diff[i];
      cnt_c++;
    }
  }

  if (cnt_a == 0)
  {
    return;
  }

  //Calculate average and std of culser A's distance
  gnss_math_fstd(distance_a, cnt_a, &avgDis_a, &stdDis_a);

  /* calculate cluster A's PR diff average and std*/
  gnss_math_fstd(diff_a, cnt_a, &avgDiff_a, &stdDiff_a);

#if defined(PLAYBACK_MODE)		
  for (i = 0; i < cnt_a; i++)
  {
    SYS_LOGGING(OBJ_QOS, LOG_INFO, "DR cluster_a: %2d %3d %2d %10.4f %10.4f %2d %10.4f %10.4f", pMeas->meas[indx_a[i]].gnssMode, pMeas->meas[indx_a[i]].prn, pMeas->meas[indx_a[i]].cno, distance_a[i], diff_a[i], cnt_a, avgDis_a, stdDis_a);
  }

  for (i = 0; i < cnt_b; i++)
  {
    SYS_LOGGING(OBJ_QOS, LOG_INFO, "DR cluster_b: %2d %3d %2d %10.4f %10.4f %2d", pMeas->meas[indx_b[i]].gnssMode, pMeas->meas[indx_b[i]].prn, pMeas->meas[indx_b[i]].cno, distance_b[i], diff_b[i], cnt_b);
  }

  for (i = 0; i < cnt_c; i++)
  {
    SYS_LOGGING(OBJ_QOS, LOG_INFO, "DR cluster_c: %2d %3d %2d %10.4f %10.4f %2d", pMeas->meas[indx_c[i]].gnssMode, pMeas->meas[indx_c[i]].prn, pMeas->meas[indx_c[i]].cno, distance_c[i], diff_c[i], cnt_c);
  }
#endif
  //decide to reject or deweight the measurement according to the cluster result
  gap_distance = distance[totalCnt - 1] - distance[0];
  if (gap_distance < 0.5)
  {
    return;
  }

  //five DIFF of cluster A is small
  if (cnt_a >= goodDrCnt)
  {
    goodDrFlag = TRUE;
    for (i = 0; i < goodDrCnt; i++)
    {
      if (fabs(diff_a[i]) >= 1.0)
      {
        goodDrFlag = FALSE;
        break;
      }
    }
  }

  //if last kf vel res is small and five DIFF of cluster A is small
  // just decrease cluster threshold
  if (p_Kf->kf_Pvt_Info->kfFixStatus == FIX_STATUS_NEW && p_Kf->velRes < 0.75 &&
    p_Kf->velResStd>0 && p_Kf->velResStd < 0.75 && goodDrFlag)
  {
    deThresFlag = TRUE;
  }
  else
  {
    deThresFlag = FALSE;
  }
  //get dr reject threshold
  drRejNum = 0;
  if (cnt_a <= 8)
  {
    thres_ac = (float)(15.0 * stdDis_a);
    min_thres = 1.5;
  }
  else if (cnt_a <= 10)
  {
    thres_ac = (float)(12.5 * stdDis_a);
    min_thres = 1.0;
  }
  else if (cnt_a <= 13)
  {
    thres_ac = (float)(7.5 * stdDis_a);
    min_thres = 0.75;
  }
  else
  {
    thres_ac = (float)(5.0 * stdDis_a);
    min_thres = 0.5;
  }

  if (deThresFlag && thres_ac > min_thres)
  {
    thres_ac -= (float)(2.5 * stdDis_a);
    SYS_LOGGING(OBJ_QOS, LOG_INFO, "DR cluster detect last good velocity,then decrease cluster thres: %8.3f", pMeas->tor);
  }

  if (cnt_c >= cnt_a)
  {
    thres_ac *= (2 + (cnt_c - cnt_a));
    min_thres *= (2 + (cnt_c - cnt_a));
  }

  if (thres_ac < min_thres)
  {
    thres_ac = min_thres;
  }
  else if (thres_ac > 10.0)
  {
    thres_ac = 10;
  }
  //get dr deweight threshold
  if (cnt_a <= 8)
  {
    thres_ab = (float)(10.0 * stdDis_a);
    min_thres = 1.0;
  }
  else if (cnt_a <= 10)
  {
    thres_ab = (float)(7.5 * stdDis_a);
    min_thres = 0.75;
  }
  else if (cnt_a <= 13)
  {
    thres_ab = (float)(5.0 * stdDis_a);
    min_thres = 0.5;
  }
  else
  {
    thres_ab = (float)(4.0 * stdDis_a);
    min_thres = 0.3;
  }

  if (deThresFlag)
  {
    thres_ab -= (float)(1.5 * stdDis_a);
  }

  if (thres_ab < min_thres)
  {
    thres_ab = min_thres;
  }
  else if (thres_ab > 10.0)
  {
    thres_ab = 10;
  }

  SYS_LOGGING(OBJ_QOS, LOG_INFO, "DR cluster rej_thres : %10.4f deweight_thres : %10.4f, stdDiff_a: %10.4f", thres_ac, thres_ab, stdDiff_a);

  if (cnt_c > 0 && cnt_c < (cnt_a + cnt_b))
  {
    for (i = cnt_c; i > 0; i--)
    {
      pSvMeas = &pMeas->meas[indx_c[i - 1]];
      if (((distance_c[i - 1] - avgDis_a) > thres_ac) && drRejNum < drRejLimit)
      {
        /* If PDOP enlarge two times after reject this satellite, then don't reject this satellite */
        if (gnss_Pe_Dop_Check(pMeas, indx_c[i - 1], 2) == FALSE &&
          (distance_c[i - 1] - avgDis_a) < 2 * thres_ac)
        {
          continue;
        }
        pSvMeas->status &= 0xFD;
        drRejNum++;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was rejected in %s DR cluster_c: %10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_c[i - 1], pSvMeas->dr_diff);
      }
      else if (((distance_c[i - 1] - avgDis_a) > thres_ab) && cnt_c < cnt_a)
      {
        pSvMeas->drWeightChck |= CLUSTER_CHECK;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was deweight in %s DR cluster_c: %10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_c[i - 1], pSvMeas->dr_diff);
      }
      else if (pSvMeas->drDiffStd > 1.0 && fabs(pSvMeas->dr_diff) > 1.4)
      {
        pSvMeas->drWeightChck |= CLUSTER_CHECK;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s because of DRDIFFSTD at %14.6f,gnssMode(%02d)prn(%03d) was rejected in DR cluster_c: %10.4f,%10.4f,%10.4f",
          __FUNCTION__, pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, distance_c[i - 1], pSvMeas->dr_diff, pSvMeas->drDiffStd);
      }

    }
  }


  if (cnt_b > 0 && cnt_b < cnt_a)
  {
    for (i = cnt_b; i > 0; i--)
    {
      pSvMeas = &pMeas->meas[indx_b[i - 1]];
      if ((distance_b[i - 1] - avgDis_a) > thres_ac && drRejNum < drRejLimit)
      {
        /* If PDOP enlarge two times after reject this satellite, then don't reject this satellite */
        if (gnss_Pe_Dop_Check(pMeas, indx_b[i - 1], 2) == FALSE)
        {
          continue;
        }
        pSvMeas->status &= 0xFD;
        drRejNum++;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was rejected in %s DR cluster_b: %10.4f,%10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_b[i - 1], pSvMeas->dr_diff, thres_ac);
      }
      else if ((distance_b[i - 1] - avgDis_a) > thres_ab)
      {
        pSvMeas->drWeightChck |= CLUSTER_CHECK;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was deweight in %s DR cluster_b: %10.4f,%10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_b[i - 1], pSvMeas->dr_diff, thres_ab);
      }
      else if (pSvMeas->drDiffStd > 1.0 && fabs(pSvMeas->dr_diff) > 1.4)
      {
        pSvMeas->drWeightChck |= CLUSTER_CHECK;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s because of DRDIFFSTD at %14.6f,gnssMode(%02d)prn(%03d) was rejected in DR cluster_b: %10.4f,%10.4f,%10.4f",
          __FUNCTION__, pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, distance_b[i - 1], pSvMeas->dr_diff, pSvMeas->drDiffStd);
      }
    }
  }
#endif
}
/***********************************************************************
* 函数介绍: gnss_Qos_Cluster_ForOutlier
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：04/20/
***********************************************************************/
void gnss_Qos_Cluster_ForOutlier(float* diffInput, uint8_t  cntInput, uint8_t* indexInput, meas_blk_t* pMeas)
{
  uint8_t             index[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 }, indx_a[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], indx_b[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], indx_c[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], min_indx[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM];
  uint8_t             outlierFlag = 0, OutlierRej_ClusterCnt = 0, cno35Cnt = 0, rejectLimit_c = 3;
  uint32_t            i;
  uint32_t            cnt, cnt_a, cnt_b, cnt_c, cntUse;
  float            stdDis_a = 0.0;
  float            diff[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], distance[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, avgDistance_a;
  float            distance_a[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], distance_b[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], distance_c[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM];
  float            diff_a[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], diff_b[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], diff_c[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM];
  float            thres_ac, thres_ab, min_thres, outlierThres = 75;
  float            gap_distance, gap_distance_thres = 50.0;
  gnss_meas_t* pSvMeas;

  cnt = 0;
  for (i = 0; i < cntInput; i++)
  {
    pSvMeas = &pMeas->meas[indexInput[i]];
    if ((pSvMeas->status & 0x1) == 0) continue;
    diff[cnt] = diffInput[i];
    index[cnt] = indexInput[i];
    cnt++;
  }

  if (cnt < 8)
  {
    return;
  }

  memset(indx_a, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(uint8_t));
  memset(indx_b, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(uint8_t));
  memset(indx_c, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(uint8_t));
  memset(distance_a, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(float));
  memset(distance_b, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(float));
  memset(distance_c, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(float));
  memset(diff_a, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(float));
  memset(diff_b, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(float));
  memset(diff_c, 0, MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * sizeof(float));

  gnss_Calc_distance(diff, distance, cnt);

  /* Sorting according to distance */
  gnss_Sort_WithIndx_1(distance, diff, index, cnt);

  outlierFlag = FALSE;
  cntUse = cnt;

  if ((distance[cnt - 1] - distance[cnt - 2]) > outlierThres)
  {
    outlierFlag = TRUE;
    cntUse = cnt - 1;
  }

  if ((distance[cnt - 2] - distance[cnt - 3]) > outlierThres)
  {
    outlierFlag = TRUE;
    cntUse = cnt - 2;
  }
  if (!outlierFlag && ((((double)distance[cnt - 1] + distance[cnt - 2]) / 2.0 - distance[cnt - 3]) > outlierThres))
  {
    outlierFlag = TRUE;
    cntUse = cnt - 2;
  }

  if (!gnss_kMean(cntUse, &distance[0], &min_indx[0], FALSE))
  {
    return;
  }
  //divide the samples into three clusters
  cnt_a = 0;
  cnt_b = 0;
  cnt_c = 0;

  for (i = 0; i < GNSS_MAX_MODE; i++)
  {
    cno35Cnt += pMeas->Cno35Cnt[i];
  }
  for (i = 0; i < cntUse; i++)
  {
    if (min_indx[i] == 0)
    {
      distance_a[cnt_a] = distance[i];
      indx_a[cnt_a] = index[i];
      diff_a[cnt_a] = diff[i];
      cnt_a++;
    }
    else if (min_indx[i] == 1)
    {
      distance_b[cnt_b] = distance[i];
      indx_b[cnt_b] = index[i];
      diff_b[cnt_b] = diff[i];
      cnt_b++;
    }
    else
    {
      distance_c[cnt_c] = distance[i];
      indx_c[cnt_c] = index[i];
      diff_c[cnt_c] = diff[i];
      cnt_c++;
    }
  }

  if (outlierFlag)
  {
    for (i = cntUse; i < cnt; i++)
    {
      distance_c[cnt_c] = distance[i];
      indx_c[cnt_c] = index[i];
      diff_c[cnt_c] = diff[i];
      cnt_c++;
    }
  }

  if (cnt_a == 0)
  {
    return;
  }

  //Calculate average and std of culser A's distance
  gnss_math_fstd(distance_a, cnt_a, &avgDistance_a, &stdDis_a);

#if defined(PLAYBACK_MODE)
  for (i = 0; i < cnt_a; i++)
  {
    SYS_LOGGING(OBJ_QOS, LOG_INFO, "PR outlier cluster_a: %2d %2d %2d %10.4f %10.4f %2d %10.4f %10.4f", pMeas->meas[indx_a[i]].gnssMode, pMeas->meas[indx_a[i]].prn, pMeas->meas[indx_a[i]].cno, distance_a[i], diff_a[i], cnt_a, avgDistance_a, stdDis_a);
  }

  for (i = 0; i < cnt_b; i++)
  {
    SYS_LOGGING(OBJ_QOS, LOG_INFO, "PR outlier cluster_b: %2d %2d %2d %10.4f %10.4f %2d", pMeas->meas[indx_b[i]].gnssMode, pMeas->meas[indx_b[i]].prn, pMeas->meas[indx_b[i]].cno, distance_b[i], diff_b[i], cnt_b);
  }

  for (i = 0; i < cnt_c; i++)
  {
    SYS_LOGGING(OBJ_QOS, LOG_INFO, "PR outlier cluster_c: %2d %2d %2d %10.4f %10.4f %2d", pMeas->meas[indx_c[i]].gnssMode, pMeas->meas[indx_c[i]].prn, pMeas->meas[indx_c[i]].cno, distance_c[i], diff_c[i], cnt_c);
  }
#endif
  //decide to reject or deweight the measurement according to the cluster result
  gap_distance = distance[cnt - 1] - distance[0];
  if (pMeas->avgCno < 40 && cno35Cnt < 20 && pMeas->Cno45Cnt * 1.0 < 0.15 * cno35Cnt)
  {
    /*decrease thres when less measCnt*/
    if (pMeas->measCnt < 15)
    {
      gap_distance_thres = 25;
      rejectLimit_c = 2;
    }
    else
    {
      gap_distance_thres = 40;
      rejectLimit_c = 2;
    }
  }
  else
  {
    gap_distance_thres = 10;
    rejectLimit_c = 3;
  }
  if (gap_distance < gap_distance_thres)//50.0 accoding to different situations
  {
    return;
  }

  if (cnt_a < 8)
  {
    /*decrease thres when less measCnt and high avgcno scene*/
    if (pMeas->measCnt < 15)
    {
      thres_ac = 25 * stdDis_a;
      min_thres = 30;
    }
    else
    {
      thres_ac = 25 * stdDis_a;
      min_thres = 70;
    }
  }
  else if (cnt_a < 12)
  {
    thres_ac = 15 * stdDis_a;
    min_thres = 50;
  }
  else
  {
    thres_ac = 10 * stdDis_a;
    min_thres = 35;
  }

  if (thres_ac < min_thres)
  {
    thres_ac = min_thres;
  }

  if (cnt_a < 8)
  {
    /*decrease thres when less measCnt*/
    if (pMeas->measCnt < 15)
    {
      thres_ab = 15 * stdDis_a;
      min_thres = 20;
    }
    else
    {
      thres_ab = 15 * stdDis_a;
      min_thres = 50;
    }
  }
  else if (cnt_a < 12)
  {
    thres_ab = 10 * stdDis_a;
    min_thres = 35;
  }
  else
  {
    thres_ab = 5 * stdDis_a;
    min_thres = 20;
  }
  SYS_LOGGING(OBJ_QOS, LOG_DEBUG, "PR outlier cluster thres_ac:%10.4f %.3f %.3f", thres_ac, thres_ab, min_thres);

  if (cnt_c > 0 && cnt_c < cnt_a)
  {
    for (i = cnt_c; i > 0; i--)
    {
      pSvMeas = &pMeas->meas[indx_c[i - 1]];
      if (((distance_c[i - 1] - avgDistance_a) > thres_ac))
      {
        /* If PDOP enlarge two times after reject this satellite, then don't reject this satellite */
        if (gnss_Pe_Dop_Check(pMeas, indx_c[i - 1], 1) == FALSE &&
          (distance_c[i - 1] - avgDistance_a) < 2 * thres_ac)
        {
          continue;
        }
        pSvMeas->status &= 0xFE;
        OutlierRej_ClusterCnt++;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was rejected in %s cluster_c: %10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_c[i - 1], diff_c[i - 1]);
      }
      else if (cnt_a > 15 && ((double)distance_c[i - 1] - avgDistance_a) > 10.0 && fabs(pSvMeas->pr_diff) > 10.0)
      {
        if (gnss_Pe_Dop_Check(pMeas, indx_c[i - 1], 1) == FALSE &&
          (distance_c[i - 1] - avgDistance_a) < 2 * thres_ac)
        {
          continue;
        }
        pSvMeas->status &= 0xFE;
        OutlierRej_ClusterCnt++;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was rejected in %s cluster_c: %10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_c[i - 1], diff_c[i - 1]);
      }
    }
  }
  /*non reject sat situations in cluster_c*/
  if (cnt_a > 8 && OutlierRej_ClusterCnt < rejectLimit_c && cnt_c > 0 && cnt_c < cnt_a)
  {
    for (i = cnt_c; i > 0; i--)
    {
      pSvMeas = &pMeas->meas[indx_c[i - 1]];
      if ((pSvMeas->status & 0x1) == 0) continue;
      if (((double)distance_c[i - 1] - avgDistance_a) > 10.0 && fabs(pSvMeas->pr_diff) > 5.0)
      {
        if (gnss_Pe_Dop_Check(pMeas, indx_c[i - 1], 1) == FALSE &&
          (distance_c[i - 1] - avgDistance_a) < 2 * thres_ac)
        {
          continue;
        }
        pSvMeas->status &= 0xFE;
        OutlierRej_ClusterCnt++;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was rejected in %s cluster_c: %10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_c[i - 1], diff_c[i - 1]);
        if (OutlierRej_ClusterCnt >= rejectLimit_c)
        {
          break;
        }
      }
    }
  }
  else if (cnt_a <= 8 && OutlierRej_ClusterCnt < rejectLimit_c && cnt_c > 0 && cnt_c < cnt_a && pMeas->measCnt < 15)
  {
    /*reject large pr in cluster_c when less measCnt and high avgcno scene*/
    for (i = cnt_c; i > 0; i--)
    {
      pSvMeas = &pMeas->meas[indx_c[i - 1]];
      if ((pSvMeas->status & 0x1) == 0) continue;
      if ((distance_c[i - 1] - avgDistance_a) > thres_ac && fabs(pSvMeas->pr_diff) > 30.0)
      {
        pSvMeas->status &= 0xFE;
        OutlierRej_ClusterCnt++;
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "gnssMode(%02d)prn(%03d) was rejected in %s cluster_c: %10.4f,%10.4f",
          pSvMeas->gnssMode, pSvMeas->prn, __FUNCTION__, distance_c[i - 1], diff_c[i - 1]);
        if (OutlierRej_ClusterCnt >= rejectLimit_c)
        {
          break;
        }
      }
    }
  }
}
/***********************************************************************
* 函数介绍: gnss_Qos_Diff_Calc
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：04/20/
***********************************************************************/
uint8_t gnss_Qos_Diff_Calc(meas_blk_t* pMeas, Kf_t* pKf, float* diff, uint8_t* diffNum, uint8_t* index)
{
  uint8_t            i;
  uint8_t            diffCnt = 0, diffFlag = FALSE, satModeNum = 0;
  uint8_t            validNum[GNSS_MAX_MODE] = { 0 };
  double           dt;
  double           calRange;
  double           detPos[3];
  double           biasDiffUse[GNSS_MAX_MODE] = { 0.0 };
  double           biasDiffInit[GNSS_MAX_MODE] = { 0.0 };
  gnss_meas_t* pSvMeas;
  GNSS_TIME* pTime;
  USER_PVT      user;

  pTime = gnss_tm_get_time();
  if (pTime->init == FALSE)
  {
    return diffFlag;
  }

  gnss_Pe_Get_PosFix(&user);
  if (user.have_position == HAVE_POS_NONE)
  {
    return diffFlag;
  }

  //load  bias diff for use
  for (i = 0; i < GNSS_MAX_MODE; i++)
  {
    if (pMeas->rtdUsed)
    {
      biasDiffUse[i] = pKf->biasDiff[i];
      biasDiffInit[i] = pKf->biasDiffInit[i];
    }
    else
    {
      biasDiffUse[i] = pKf->biasDiffLocal[i];
      biasDiffInit[i] = pKf->biasDiffLocalInit[i];
    }
  }

  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &pMeas->meas[i];

    if (pSvMeas->prn == 0 || pSvMeas->pseudoRange < 0) continue;
    if ((pSvMeas->status & 0x1) == 0) continue;
    if (pSvMeas->sv_info.eph_status == FALSE)   continue;

    detPos[0] = pSvMeas->sv_info.p[0] - user.ecef.pos[0];
    detPos[1] = pSvMeas->sv_info.p[1] - user.ecef.pos[1];
    detPos[2] = pSvMeas->sv_info.p[2] - user.ecef.pos[2];
    calRange = gnssClcSqrtSum_DBL(detPos, 3);

    dt = calRange / LIGHT_SEC;
    detPos[0] = (pSvMeas->sv_info.p[0] + (WGS84_OMEGDOTE * pSvMeas->sv_info.p[1] * dt) - user.ecef.pos[0]);
    detPos[1] = (pSvMeas->sv_info.p[1] - (WGS84_OMEGDOTE * pSvMeas->sv_info.p[0] * dt) - user.ecef.pos[1]);
    detPos[2] = pSvMeas->sv_info.p[2] - user.ecef.pos[2];
    calRange = gnssClcSqrtSum_DBL(detPos, 3);
    /*
    (1) GPS + GLN + BDS, GPS + GLN or GPS + BDS
    (2) GLN + BDS
    */
    if (pTime->timeStatus[GPS_MODE] == TM_STATUS_ACCU)
    {
      if (fabs(biasDiffUse[pSvMeas->gnssMode]) > 0)
      {
        diff[diffCnt] = -(float)(calRange + pTime->bias[GPS_MODE] - biasDiffUse[pSvMeas->gnssMode] - pSvMeas->pseudoRange);
      }
      else if (pSvMeas->gnssMode == GPS_MODE)
      {
        diff[diffCnt] = -(float)(calRange + pTime->bias[pSvMeas->gnssMode] - pSvMeas->pseudoRange);
      }
      else
      {
        continue;
      }
    }
    else if (pTime->timeStatus[GLN_MODE] == TM_STATUS_ACCU && fabs(biasDiffUse[GLN_MODE]) > 0)
    {
      if (pSvMeas->gnssMode == GPS_MODE)
      {
        diff[diffCnt] = -(float)(calRange + pTime->bias[GLN_MODE] + biasDiffUse[GLN_MODE] - pSvMeas->pseudoRange);
      }
      else if (pSvMeas->gnssMode == BDS_MODE && fabs(biasDiffUse[pSvMeas->gnssMode]) > 0)
      {
        diff[diffCnt] = -(float)(calRange + pTime->bias[GLN_MODE] + biasDiffUse[GLN_MODE] - biasDiffUse[pSvMeas->gnssMode] - pSvMeas->pseudoRange);
      }
      else if (pSvMeas->gnssMode == GAL_MODE && fabs(biasDiffUse[pSvMeas->gnssMode]) > 0)
      {
        diff[diffCnt] = -(float)(calRange + pTime->bias[GLN_MODE] + biasDiffUse[GLN_MODE] - biasDiffUse[pSvMeas->gnssMode] - pSvMeas->pseudoRange);
      }
      else if (pSvMeas->gnssMode == GLN_MODE)
      {
        diff[diffCnt] = -(float)(calRange + pTime->bias[pSvMeas->gnssMode] - pSvMeas->pseudoRange);
      }
      else
      {
        continue;
      }
    }
    else if (pTime->timeStatus[BDS_MODE] == TM_STATUS_ACCU && fabs(biasDiffUse[BDS_MODE]) > 0)
    {
      if (pSvMeas->gnssMode == GPS_MODE)
      {
        diff[diffCnt] = -(float)(calRange + pTime->bias[BDS_MODE] + biasDiffUse[BDS_MODE] - pSvMeas->pseudoRange);
      }
      else if (pSvMeas->gnssMode == GLN_MODE && fabs(biasDiffUse[pSvMeas->gnssMode]) > 0)
      {
        diff[diffCnt] = -(float)(calRange + pTime->bias[BDS_MODE] + biasDiffUse[BDS_MODE] - biasDiffUse[pSvMeas->gnssMode] - pSvMeas->pseudoRange);
      }
      else if (pSvMeas->gnssMode == GAL_MODE && fabs(biasDiffUse[pSvMeas->gnssMode]) > 0)
      {
        diff[diffCnt] = -(float)(calRange + pTime->bias[BDS_MODE] + biasDiffUse[BDS_MODE] - biasDiffUse[pSvMeas->gnssMode] - pSvMeas->pseudoRange);
      }
      else if (pSvMeas->gnssMode == BDS_MODE)
      {
        diff[diffCnt] = -(float)(calRange + pTime->bias[pSvMeas->gnssMode] - pSvMeas->pseudoRange);
      }
      else
      {
        continue;
      }
    }
    else if (pTime->timeStatus[GAL_MODE] == TM_STATUS_ACCU && fabs(biasDiffUse[GAL_MODE]) > 0)
    {
      if (pSvMeas->gnssMode == GPS_MODE)
      {
        diff[diffCnt] = -(float)(calRange + pTime->bias[GAL_MODE] + biasDiffUse[GAL_MODE] - pSvMeas->pseudoRange);
      }
      else if (pSvMeas->gnssMode == GLN_MODE && fabs(biasDiffUse[pSvMeas->gnssMode]) > 0)
      {
        diff[diffCnt] = -(float)(calRange + pTime->bias[GAL_MODE] + biasDiffUse[GAL_MODE] - biasDiffUse[pSvMeas->gnssMode] - pSvMeas->pseudoRange);
      }
      else if (pSvMeas->gnssMode == BDS_MODE && fabs(biasDiffUse[pSvMeas->gnssMode]) > 0)
      {
        diff[diffCnt] = -(float)(calRange + pTime->bias[GAL_MODE] + biasDiffUse[GAL_MODE] - biasDiffUse[pSvMeas->gnssMode] - pSvMeas->pseudoRange);
      }
      else if (pSvMeas->gnssMode == GAL_MODE)
      {
        diff[diffCnt] = -(float)(calRange + pTime->bias[pSvMeas->gnssMode] - pSvMeas->pseudoRange);
      }
      else
      {
        continue;
      }
    }
    else if (pTime->timeStatus[GPS_MODE] == TM_STATUS_SET && pTime->timeStatus[GLN_MODE] == TM_STATUS_SET && pTime->timeStatus[BDS_MODE] == TM_STATUS_SET && pTime->timeStatus[GAL_MODE] == TM_STATUS_SET)
    {
      if (fabs(biasDiffUse[pSvMeas->gnssMode]) > 0)
      {
        diff[diffCnt] = -(float)(calRange + pTime->bias[GPS_MODE] - biasDiffUse[pSvMeas->gnssMode] - pSvMeas->pseudoRange);
      }
      else if (pSvMeas->gnssMode == GPS_MODE)
      {
        diff[diffCnt] = -(float)(calRange + pTime->bias[pSvMeas->gnssMode] - pSvMeas->pseudoRange);
      }
      else
      {
        continue;
      }
    }
    else
    {
      continue;
    }

    if (pSvMeas->freq_index == 1)
    {
      diff[diffCnt] += (float)(pTime->dcb[pSvMeas->gnssMode]);
    }
    else if (pSvMeas->freq_index == 2)
    {
      diff[diffCnt] += (float)(pTime->dcb[pSvMeas->gnssMode + GNSS_MAX_MODE]);
    }

    index[diffCnt++] = i;
    validNum[pSvMeas->gnssMode]++;
  }

  *diffNum = diffCnt;

  //check the calculate PR diff's quality
  for (i = 0; i < (GNSS_MAX_MODE - 1); i++)
  {
    if (validNum[i] > 0)
    {
      satModeNum++;
    }
  }
  if (satModeNum <= 1)
  {
    diffFlag = TRUE;
  }
  else
  {
    if ((validNum[GLN_MODE] > 0 && DBL_IS_EQUAL(biasDiffUse[GLN_MODE], biasDiffInit[GLN_MODE])) ||
      (validNum[BDS_MODE] > 0 && DBL_IS_EQUAL(biasDiffUse[BDS_MODE], biasDiffInit[BDS_MODE])) ||
      (validNum[GAL_MODE] > 0 && DBL_IS_EQUAL(biasDiffUse[GAL_MODE], biasDiffInit[GAL_MODE])))
    {
      diffFlag = FALSE;
    }
    else
    {
      diffFlag = TRUE;
    }
  }

  return diffFlag;
}
/***********************************************************************
* 函数介绍: gnss_Qos_MAD_ForOutlier
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：04/20
***********************************************************************/
void gnss_Qos_MAD_ForOutlier(meas_blk_t* pMeas, float* diff, uint8_t diffNum, uint8_t* index, uint8_t posFlag, uint8_t diffFlag)
{
  uint8_t            i;
  uint8_t            flag, diffAbs20Cnt = 0, diffAbs10Cnt = 0, diffAbsCnt = 0;
  float           median, rejectK1 = 10, rejectK2 = 4.0, reject3 = 2.0, reject4 = 1.5;
  float           diffAbs[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 };


  flag = gnss_MAD(diff, diffAbs, diffNum, &median);
  if (flag == FALSE) return;


  if (!firstFix && g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100)
  {
    for (i = 0; i < diffNum; i++)
    {
      if (index[i] >= MAX_MEAS_NUM) continue;
      diffAbsCnt++;
      if (diffAbs[i] > 10.0)
      {
        diffAbs10Cnt++;
      }
      if (diffAbs[i] > 20.0)
      {
        diffAbs20Cnt++;
      }
    }
  }
  for (i = 0; i < diffNum; i++)
  {
    if (index[i] >= MAX_MEAS_NUM) continue;
    if (diffAbs[i] > rejectK1 * (median / 0.6745) && diffAbs[i] > 10000)
    {
      pMeas->meas[index[i]].status &= 0xF8;
#if defined(PLAYBACK_MODE)
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s: reject PR/DR meas with large error:  %02d %03d", __FUNCTION__, pMeas->meas[index[i]].gnssMode, pMeas->meas[index[i]].prn);
#endif
    }
    else if (diffAbs[i] > rejectK2 * (median / 0.6745) && diffAbs[i] > 80 && posFlag && diffFlag)
    {
      pMeas->meas[index[i]].status &= 0xFE;
#if defined(PLAYBACK_MODE)
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s reject PR before first fix: %02d %03d", __FUNCTION__, pMeas->meas[index[i]].gnssMode, pMeas->meas[index[i]].prn);
#endif
    }
    else if (diffAbs[i] > (median / 0.6745) && diffAbs[i] > 40 && posFlag && diffFlag && pMeas->meas[index[i]].cno < 20 && pMeas->meas[index[i]].cno < pMeas->avgCno)
    {
      pMeas->meas[index[i]].status &= 0xFE;
#if defined(PLAYBACK_MODE)
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s reject PR before first fix: %02d %03d", __FUNCTION__, pMeas->meas[index[i]].gnssMode, pMeas->meas[index[i]].prn);
#endif
    }
    else if (diffAbs[i] > rejectK2 * (median / 0.6745) && diffAbs[i] > 30 && posFlag && diffFlag && pMeas->meas[index[i]].cno < 40)
    {
      pMeas->meas[index[i]].status &= 0xFE;
#if defined(PLAYBACK_MODE)
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s reject PR before first fix: %02d %03d", __FUNCTION__, pMeas->meas[index[i]].gnssMode, pMeas->meas[index[i]].prn);
#endif
    }
    else if (diffAbs[i] > reject3 * (median / 0.6745) && diffAbs[i] > 500 && diffFlag && !posFlag)
    {
      pMeas->meas[index[i]].status &= 0xFE;
#if defined(PLAYBACK_MODE)
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s reject PR before first fix: %02d %03d,%d,%d", __FUNCTION__, pMeas->meas[index[i]].gnssMode, pMeas->meas[index[i]].prn, pMeas->avgCno, pMeas->meas[index[i]].cno);
#endif
    }
    if (!firstFix && g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100)
    {
      if (1.0 * diffAbs20Cnt <= 0.4 * diffAbs10Cnt && median <= 5.0)
      {
        reject4 = 4.0;
      }
      else
      {
        reject4 = 1.5;
      }
      if (diffAbs[i] > reject4 * (median / 0.6745) && diffAbs[i] > 20.0 && !diffFlag && posFlag && median <= 10.0)
      {
        pMeas->meas[index[i]].status &= 0xFE;
#if defined(PLAYBACK_MODE)
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s reject large PR before first fix: %02d %03d,%d,%d", __FUNCTION__, pMeas->meas[index[i]].gnssMode, pMeas->meas[index[i]].prn, pMeas->avgCno, pMeas->meas[index[i]].cno);
#endif
      }
    }
    if (!g_pe_cfg.automobile && g_pe_cfg.sub_chipType == SUB_CHIPTYPE_QCOM_855 && peMode.userContext.context != USER_CONTEXT_VEHICLE)
    {
      if (!firstFix && diffAbs[i] > reject4 * (median / 0.6745) && diffAbs[i] > 15.0 && !diffFlag && posFlag && median <= 10.0)
      {
        pMeas->meas[index[i]].status &= 0xFE;
#if defined(PLAYBACK_MODE)
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s reject large PR1 before first fix: %02d %03d,%d,%d", __FUNCTION__, pMeas->meas[index[i]].gnssMode, pMeas->meas[index[i]].prn, pMeas->avgCno, pMeas->meas[index[i]].cno);
#endif
      }
      else if (!firstFix && diffAbs[i] > reject3 * (median / 0.6745) && diffAbs[i] > 50.0 && !diffFlag && posFlag && median > 10.0 && median < 20.0)
      {
        pMeas->meas[index[i]].status &= 0xFE;
#if defined(PLAYBACK_MODE)
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s reject large PR2 before first fix: %02d %03d,%d,%d", __FUNCTION__, pMeas->meas[index[i]].gnssMode, pMeas->meas[index[i]].prn, pMeas->avgCno, pMeas->meas[index[i]].cno);
#endif
      }
      else if (!firstFix && diffAbs[i] > rejectK2 * (median / 0.6745) && diffAbs[i] > 100.0 && !diffFlag && posFlag && median >= 20.0)
      {
        pMeas->meas[index[i]].status &= 0xFE;
#if defined(PLAYBACK_MODE)
        SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s reject large PR3 before first fix: %02d %03d,%d,%d", __FUNCTION__, pMeas->meas[index[i]].gnssMode, pMeas->meas[index[i]].prn, pMeas->avgCno, pMeas->meas[index[i]].cno);
#endif
      }
    }
  }
}
/***********************************************************************
* 函数介绍: gnss_Qos_InitOutlier_Detection
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：04/20
***********************************************************************/
void gnss_Qos_InitOutlier_Detection(meas_blk_t* pMeas, Kf_t* pKf)
{
  uint8_t            i;
  uint8_t            detectFlag = FALSE, goodPos = FALSE, diffQuality;
  uint8_t            index[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 };
  uint8_t            diffNum = 0;
  int32_t           delta_week;
  float           velocity = 0;
  float           diff[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 };
  double           dt, distance;
  GNSS_TIME* pTime;
  USER_PVT      user;

  if (!firstFix)
  {
    if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_HD9310 || g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100)
    {
      gnss_Qos_PRDR_MAD(pMeas);
    }
  }

  // not time or position, just return
  pTime = gnss_tm_get_time();
  if (pTime->init == FALSE)
  {
    return;
  }
  gnss_Pe_Get_PosFix(&user);
  if (user.have_position == HAVE_POS_NONE)
  {
    return;
  }

  dt = pTime->rcvr_time[GPS_MODE] - user.posfix_t;
  delta_week = (int32_t)(pTime->week[0] - user.posfix_wn);
  dt += (double)SECS_IN_WEEK * delta_week;

  if (fabs(dt) > POSITION_HOLD_TIME)
  {
    return;
  }

  /*detect whether need to do initial outlier detection:
  (1) still no position fix;
  (2) one sat system time/bias is not accurate.
  */
  gnss_Pe_PRDR_Num(pMeas);
  if (!firstFix)
  {
    detectFlag = TRUE;
  }
  for (i = 0; i < GNSS_MAX_MODE; i++)
  {
    if (pMeas->prNumEachSystem[i] > 0 && !gnss_tm_check_bias_status(i))
    {
      detectFlag = TRUE;
      break;
    }
  }
  if (!detectFlag)
  {
    return;
  }

  // Calculate initial meas diff for outliers detection
  diffQuality = gnss_Qos_Diff_Calc(pMeas, pKf, diff, &diffNum, index);

  if (diffNum < 4)
  {
    return;
  }
  if (user.ecef.have_position > POS_FRM_NULL)
  {
    goodPos = TRUE;
  }
  else
  {
    for (i = 0; i < 3; i++)
    {
      velocity += user.lla.vel[i] * user.lla.vel[i];
    }
    velocity = (float)sqrt(velocity);

    if (g_pe_cfg.automobile)
    {
      if (fabs(dt) < 10)
      {
        goodPos = TRUE;
      }
      else if (velocity < 0.3 && fabs(dt) < 600)
      {
        goodPos = TRUE;
      }
    }
    else
    {
      distance = velocity * fabs(dt);

      if (distance < 50)
      {
        goodPos = TRUE;
      }
    }
  }


  /* use MAD to do PR outlier detection  */
  gnss_Qos_MAD_ForOutlier(pMeas, diff, diffNum, index, goodPos, diffQuality);

  //has position fix or no last LS/KF position
  if (firstFix || !goodPos || !diffQuality)
  {
    return;
  }
  //use PR cluster for outlier detection
  gnss_Qos_Cluster_ForOutlier(diff, diffNum, index, pMeas);
}

static void gnss_Qos_Check_SVQuality_QCOM(meas_blk_t* pMeas)
{
  uint8_t               goodPR;
  uint32_t              i;
  uint32_t              cnoThres;
  gnss_meas_t* pSvMeas;
  sat_data_t* sp;

  if (g_pe_cfg.automobile)
  {
    cnoThres = 45;
  }
  else
  {
    cnoThres = 35;
  }

  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &(pMeas->meas[i]);
    if (pSvMeas->prn == 0) continue;
    if (pSvMeas->status == 0) continue;

    pSvMeas->quality = 0;
    pSvMeas->isPrChiSqTest = TRUE;
    pSvMeas->isDrChiSqTest = TRUE;

    if (fabs(pSvMeas->stdPRDRDiff) < 0.000001) continue;
    if (fabs(pSvMeas->drDiffStd) < 0.000001) continue;

    sp = gnss_sd_get_sv_data(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);
    goodPR = FALSE;
    if (pSvMeas->status & 0x1) // PR quality check
    {

      /* case 0:
      *  How to determine one measurement is good or not, we use following criteria
      *  (1) cno >= 35
      *  (2) PRDR std is smaller than 10.0
      *  (3) PR diff is smaller than 10.0
      *  case 1:
      *  (1) cno >= 35
      *  (2)
      */
      if (pSvMeas->cno >= cnoThres)
      {
        if (peMode.userSceData.isPDRMode)
        {
          if (pSvMeas->stdPRDRDiff > 1e-3 && pSvMeas->stdPRDRDiff < 5.0 && fabs(pSvMeas->pr_diff) < 10.0)
          {
            goodPR = TRUE;
          }
        }
        else if (g_pe_cfg.automobile == 0)
        {
          if (pSvMeas->stdPRDRDiff > 1e-3 && pSvMeas->stdPRDRDiff < 5.0 && fabs(pSvMeas->pr_diff) < 5.0)
          {
            goodPR = TRUE;
          }
        }
        else if (pSvMeas->stdPRDRDiff > 1e-3 && pSvMeas->stdPRDRDiff < 10.0 && fabs(pSvMeas->pr_diff) < 10.0)
        {
          goodPR = TRUE;
        }
        else if (pSvMeas->stdPRDRDiff > 1e-3 && pSvMeas->stdPRDRDiff < 5.0 && fabs(pSvMeas->pr_diff) < 20.0 &&
          peMode.userSceData.isUnderEleRoad)
        {
          goodPR = TRUE;
        }
      }
      else if (pSvMeas->cno >= 25)
      {
        if (pSvMeas->stdPRDRDiff > 1e-3 && pSvMeas->stdPRDRDiff < 5.0 && fabs(pSvMeas->pr_diff) < 10.0)
        {
          goodPR = TRUE;
        }
      }
#if 1
      if (peMode.userSceData.isDownTown && peState.posHasBias)
      {
        if (goodPR == FALSE)
        {
          // case0:
          if (pSvMeas->stdPRDRDiff > 1e-3 && pSvMeas->stdPRDRDiff < 15.0 && fabs(pSvMeas->sumPRDRDiff) < 20.0
            && pSvMeas->cno >= 38 && fabs(pSvMeas->pr_diff) < 100.0)
          {
            goodPR = TRUE;
            sp->goodPrCnt = 0;
            goto GOODPR_PROC;
          }
          //case1:
          if (pSvMeas->stdPRDRDiff > 1e-3 && pSvMeas->stdPRDRDiff < 10.0 && fabs(pSvMeas->sumPRDRDiff) < 20.0
            && fabs(pSvMeas->pr_diff) < 100.0 && pSvMeas->cno >= 30)
          {
            if (sp->goodPrCnt < 3) sp->goodPrCnt++;
            if (sp->goodPrCnt >= 3) goodPR = TRUE;
          }
          else
          {
            sp->goodPrCnt = 0;
            goodPR = FALSE;
          }
        }
        else
        {
          sp->goodPrCnt = 0;
        }
      }
#endif
    GOODPR_PROC:
      if (goodPR == TRUE)
      {
        pSvMeas->quality |= PR_GOOD;
        pSvMeas->isPrChiSqTest = FALSE;
      }
    }

    if (pSvMeas->status & 0x2) // DR quality check
    {
    }
  }
}

static void gnss_Qos_Check_SVQuality_SPRD(meas_blk_t* pMeas)
{
#ifdef USED_IN_MC262M
  uint8_t               goodPR;
  uint32_t              i;
  uint32_t              cnoThres;
  gnss_meas_t* pSvMeas;
  sat_data_t* sp;

  if (g_pe_cfg.automobile)
  {
    cnoThres = 45;
  }
  else
  {
    cnoThres = 35;
  }

  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &(pMeas->meas[i]);
    if (pSvMeas->prn == 0) continue;
    if (pSvMeas->status == 0) continue;
    pSvMeas->quality = 0;
    pSvMeas->isPrChiSqTest = TRUE;
    pSvMeas->isDrChiSqTest = TRUE;
    sp = gnss_sd_get_sv_data(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);
    goodPR = FALSE;
    if (pSvMeas->status & 0x1) // PR quality check
    {
      if (pSvMeas->cno >= cnoThres)
      {
        if (pSvMeas->stdPRDRDiff > 0.0 && pSvMeas->stdPRDRDiff < 10.0 && fabs(pSvMeas->pr_diff) < 10.0)
        {
          goodPR = TRUE;
        }
        else if (pSvMeas->stdPRDRDiff > 0.0 && pSvMeas->stdPRDRDiff < 3.0 && fabs(pSvMeas->sumPRDRDiff) < 5.0
          && fabs(pSvMeas->pr_diff) < 15.0)
        {
          goodPR = TRUE;
        }
      }
      else if (pSvMeas->cno >= 25)
      {
        if (pSvMeas->stdPRDRDiff > 0.0 && pSvMeas->stdPRDRDiff < 3.0 && fabs(pSvMeas->sumPRDRDiff) < 5.0
          && fabs(pSvMeas->pr_diff) < 15.0)
        {
          goodPR = TRUE;
        }
      }
#if 1
      if (pSvMeas->stdPRDRDiff > 0.0 && pSvMeas->stdPRDRDiff < 5.0 && fabs(pSvMeas->sumPRDRDiff) < 5.0)
      {
        if (sp->goodPrCnt < 5) sp->goodPrCnt++;
        if (sp->goodPrCnt >= 5) goodPR = TRUE;
      }
      else
      {
        sp->goodPrCnt = 0;
        goodPR = FALSE;
      }
#endif
      if (goodPR == TRUE)
      {
        pSvMeas->quality |= PR_GOOD;
        pSvMeas->isPrChiSqTest = FALSE;
      }
#if defined(PLAYBACK_MODE)
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s,%14.6f,%02d,%02d,%02d,%x,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f", __FUNCTION__, pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->cno, pSvMeas->quality, pSvMeas->stdPRDRDiff, pSvMeas->drDiffStd, pSvMeas->sumPRDRDiff, pSvMeas->dr_err, pSvMeas->pr_err, sp->prR);
#endif
    }

    if (pSvMeas->status & 0x2) // DR quality check
    {
      if (fabs(pSvMeas->dr_diff) > 0.0 && fabs(pSvMeas->dr_diff) < 0.5
        && pSvMeas->drDiffStd < 0.5 && pSvMeas->drDiffStd > 0.0)
      {
        pSvMeas->quality |= DR_GOOD;
        pSvMeas->isDrChiSqTest = FALSE;
  }
}
  }
#endif
}
void gnss_Qos_QualityCheck_Thres(meas_blk_t* pMeas, double posRes, double* diffThres1, double* diffThres2, double* diffStdThres, uint8_t* goodDiff5Cnt, uint8_t* smallDiff15Cnt, uint8_t* measAllCnt)
{
  uint8_t               measCnt = 0/*, smallDiffFlag = FALSE*/, goodDiffCnt = 0, smallDiffCnt = 0/*, posQuality = FALSE*/;
  uint32_t              i;
  gnss_meas_t* pSvMeas;


  // last position quality check
  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &(pMeas->meas[i]);
    if (pSvMeas->prn == 0) continue;

    if (pSvMeas->status & 0x1)
    {
      if (fabs(pSvMeas->pr_diff) < 15.0)
      {
        smallDiffCnt++;
      }
      if (fabs(pSvMeas->pr_diff) < 5.0)
      {
        goodDiffCnt++;
      }
      measCnt++;
    }
  }
  *goodDiff5Cnt = goodDiffCnt;
  *smallDiff15Cnt = smallDiffCnt;
  *measAllCnt = measCnt;

  if (peMode.userSceData.isUnderEleRoad == 0)
  {
    if ((smallDiffCnt >= 3 && posRes > 0 && posRes < 25.0) || (smallDiffCnt >= 4 && posRes > 0 && posRes < 35.0) ||
      (smallDiffCnt >= 6 && posRes > 0 && posRes < 40.0))
    {
      *diffThres1 = 50;
    }
    if ((smallDiffCnt >= 8 && posRes > 0 && posRes < 15.0) ||
      (measCnt != 0 && ((double)smallDiffCnt / (double)measCnt > 0.6) && smallDiffCnt >= 6 && ((double)goodDiffCnt / (double)smallDiffCnt > 0.5)))
    {
      *diffThres1 = 20;
    }
  }
  else
  {
    if (smallDiffCnt >= 15 && ((double)goodDiffCnt / (double)smallDiffCnt > 0.65) && posRes > 0 && posRes < 15.0)
    {
      *diffThres1 = 20;
    }
    else if (smallDiffCnt >= 15 && ((double)goodDiffCnt / (double)smallDiffCnt > 0.45) && posRes > 0 && posRes < 15.0)
    {
      *diffThres1 = 30;
    }
    else if (smallDiffCnt >= 10 && ((double)goodDiffCnt / (double)smallDiffCnt > 0.45) && posRes > 0 && posRes < 35.0)
    {
      *diffThres1 = 60;
    }
    else if (smallDiffCnt >= 3 && posRes > 0 && posRes < 35.0)
    {
      *diffThres1 = 100;
    }
  }

  if (measCnt > 0 && ((double)goodDiffCnt / (double)measCnt) > 0.8)
  {
    *diffThres1 = 5.0;
    *diffThres2 = 5.0;
  }

  if (peMode.userSceData.isUnderEleRoad)
  {
    *diffStdThres = 5.0;
  }
  else
  {
    *diffStdThres = 3.5;
  }
}
static void gnss_Qos_Check_SVQuality_UBLOX(meas_blk_t* pMeas, Kf_t* p_Kf)
{
  uint8_t               goodPR = FALSE, goodDR = FALSE, smallDiff15Cnt, goodDiff5Cnt, measAllCnt, PosBias = FALSE;
  uint32_t              i;
  uint32_t              cnoThres;
  double              dt, prDiffThres1, prDiffThres2, PRDRDiffStdThres, svDiffStdThres, svPrDiffThres;
  gnss_meas_t* pSvMeas;
  sat_data_t* sp;

  if (g_pe_cfg.automobile)
  {
    cnoThres = 38;
  }
  else
  {
    cnoThres = 30;
  }

  prDiffThres1 = 2000;
  prDiffThres2 = 10;

  //adjust quality check thres
  gnss_Qos_QualityCheck_Thres(pMeas, p_Kf->posRes, &prDiffThres1, &prDiffThres2, &PRDRDiffStdThres, &goodDiff5Cnt, &smallDiff15Cnt, &measAllCnt);

  if (pMeas->avgCno >= 40 && pMeas->prDiff5Cnt > 10 && (double)pMeas->prDiff5Cnt / pMeas->prDiffNum > 0.6 && pMeas->prDiffStd_1 < 12.0 && pMeas->prDiffStd_1 > 0)
  {
    PosBias = TRUE;
  }

  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &(pMeas->meas[i]);
    if (pSvMeas->prn == 0) continue;
    if (pSvMeas->status == 0) continue;

    pSvMeas->quality = 0;
    pSvMeas->isPrChiSqTest = TRUE;
    pSvMeas->isDrChiSqTest = TRUE;

    if (fabs(pSvMeas->stdPRDRDiff) < 0.000001) continue;
    if (fabs(pSvMeas->drDiffStd) < 0.000001) continue;

    sp = gnss_sd_get_sv_data(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);
    goodPR = FALSE;
    goodDR = FALSE;

    if (pSvMeas->status & 0x1) // PR quality check
    {
      if (pSvMeas->cno >= 38 && sp->d_cos.elev >= 20 * DEG2RAD)
      {
        svDiffStdThres = 2.0;
      }
      else
      {
        svDiffStdThres = PRDRDiffStdThres;
      }

      if (pSvMeas->cno > 30 && sp->d_cos.elev < 20 * DEG2RAD && prDiffThres1 < 50)
      {
        svPrDiffThres = prDiffThres1 * 2.0;
      }
      else
      {
        svPrDiffThres = prDiffThres1;
      }


      dt = sp->measBack[1].tot - sp->measBack[0].tot;
      if (dt < -SECS_IN_WEEK / 2.0)
      {
        dt += (double)SECS_IN_WEEK;
      }
      else if (dt > SECS_IN_WEEK / 2.0)
      {
        dt -= (double)SECS_IN_WEEK;
      }
      if (fabs(dt) < 1.5 && sp->measBack[0].pr > 0.0 && sp->measBack[1].pr > 0.0 && firstFix && !peMode.staticData.staticFlag
        && (pSvMeas->cno >= 35 || pSvMeas->sv_info.fltElev > 15.0 || peMode.userSceData.isUnderEleRoad == FALSE))
      {
        if ((sp->measBack[0].cycleSlipCnt == 0 || sp->measBack[0].cycleSlipCnt == 2) &&
          (sp->measBack[1].cycleSlipCnt == 0 || sp->measBack[1].cycleSlipCnt == 2) &&
          pSvMeas->stdPRDRDiff < svDiffStdThres && fabs(pSvMeas->sumPRDRDiff) < 20 && fabs(pSvMeas->pr_diff) < svPrDiffThres)
        {
          goodPR = TRUE;
          if (goodDiff5Cnt >= 8 && fabs(pSvMeas->pr_diff) > 20 && pSvMeas->cno <= cnoThres && (double)goodDiff5Cnt / (double)smallDiff15Cnt > 0.7)
          {
            goodPR = FALSE;
          }
        }
      }

      if (goodPR == FALSE && pSvMeas->cno >= cnoThres)
      {
        if (pSvMeas->stdPRDRDiff > 1e-3 && pSvMeas->stdPRDRDiff < 5.0 && fabs(pSvMeas->pr_diff) < prDiffThres2)
        {
          goodPR = TRUE;
        }
        else if ((pSvMeas->cno - pMeas->avgCno > 11) && (pSvMeas->stdPRDRDiff > 1e-3) && (pSvMeas->stdPRDRDiff < 5.0) && (fabs(pSvMeas->pr_diff) < 1.4 * prDiffThres2))
        {
          goodPR = TRUE;
        }
        else if (PosBias && pSvMeas->stdPRDRDiff > 1e-3 && pSvMeas->stdPRDRDiff < 1.0 &&
          fabs(pSvMeas->pr_diff) < 2.0 * prDiffThres2 && fabs(pSvMeas->sumPRDRDiff) < 2.0)
        {
          goodPR = TRUE;
        }
      }

#if 0
      if (goodPR == FALSE && peMode.userSceData.isUnderEleRoad && pSvMeas->cno > 30)
      {
        if ((sp->measBack[0].cycleSlipCnt == 0 || sp->measBack[0].cycleSlipCnt == 2) &&
          pSvMeas->stdPRDRDiff > 1e-3 &&
          ((pSvMeas->stdPRDRDiff < 5.0 && fabs(pSvMeas->pr_diff) < 20.0) || (pSvMeas->stdPRDRDiff < 3.0 && fabs(pSvMeas->pr_diff) < 25.0)))
        {
          goodPR = TRUE;
        }
      }
#endif

    GOODPR_PROC:
      if (goodPR == TRUE)
      {
        pSvMeas->quality |= PR_GOOD;
        pSvMeas->prWeightChck = 0;
        pSvMeas->isPrChiSqTest = FALSE;
        pMeas->goodPrCnt++;
      }
      pMeas->CheckPrCnt++;
    }

    if (pSvMeas->status & 0x2) // DR quality check
    {
      dt = sp->measBack[1].tot - sp->measBack[0].tot;
      if (dt < -SECS_IN_WEEK / 2.0)
      {
        dt += (double)SECS_IN_WEEK;
      }
      else if (dt > SECS_IN_WEEK / 2.0)
      {
        dt -= (double)SECS_IN_WEEK;
      }
      if (fabs(dt) < 1.5 && sp->measBack[0].pr > 0.0 && sp->measBack[1].pr > 0.0 && firstFix && !peMode.staticData.staticFlag)
      {
        if ((sp->measBack[0].cycleSlipCnt == 0) && (sp->measBack[1].cycleSlipCnt == 0) && pSvMeas->drDiffStd < 1.0)
        {
          goodDR = TRUE;
        }
      }

      if (goodDR == TRUE)
      {
        pSvMeas->quality |= DR_GOOD;
        pSvMeas->drWeightChck = 0;
        pSvMeas->isDrChiSqTest = FALSE;
      }
    }

    if ((pSvMeas->status & 0x1) || (pSvMeas->status & 0x2))
    {
      SYS_LOGGING(OBJ_QOS, LOG_DEBUG, "%s,%14.6f,%02d,%3d,%02d,%x,%x,%10.4f,%10.4f,%10.4f,%10.4f",
        __FUNCTION__, pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->cno,
        pSvMeas->quality, pSvMeas->cycleSlipCount, pSvMeas->stdPRDRDiff,
        pSvMeas->drDiffStd, pSvMeas->sumPRDRDiff, sp->prR);
    }
  }
}

void gnss_Qos_PrDrDiffSTD(meas_blk_t* pMeas)
{
  uint8_t              i, totalCnt = 0, rejectCnt = 0, smallDiffStdCnt = 0, medianDiffStdCnt = 0, largeDiffCnt = 0, rejectThresCnt = 5;
  uint8_t              indx[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 };
  float             diff[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, prdrDiffStd[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, avgdiffstd = 0.0, stddiffstd = 0.0;
  gnss_meas_t* pSvMeas;

  if (!firstFix)
  {
    return;
  }
  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &pMeas->meas[i];
    if (pSvMeas->prn == 0 || pSvMeas->prDiffValid == 0) continue;
    if ((pSvMeas->status & 0x2) != 0x2) continue;

    if (fabs(pSvMeas->stdPRDRDiff) < 1e-4) continue;

    diff[totalCnt] = (float)pSvMeas->dr_diff;
    prdrDiffStd[totalCnt] = (float)pSvMeas->stdPRDRDiff;
    if (pSvMeas->stdPRDRDiff > 5.0)
    {
      largeDiffCnt++;
    }
    else if (pSvMeas->stdPRDRDiff > 2.5)
    {
      medianDiffStdCnt++;
    }
    else
    {
      smallDiffStdCnt++;
    }
    indx[totalCnt++] = i;
  }

  if (totalCnt < 6) return;
  gnss_Sort_WithIndx_1(prdrDiffStd, diff, indx, totalCnt);

  //Calculate average and std of stddiff
  gnss_math_fstd(prdrDiffStd, totalCnt, &avgdiffstd, &stddiffstd);

  if ((avgdiffstd > 4.0 || stddiffstd > 3.0) && pMeas->avgCno < 20)
  {
    return;
  }
  if (medianDiffStdCnt > 5 && avgdiffstd < 2.0 && stddiffstd < 2.0)
  {
    rejectThresCnt = 3;
  }
  else
  {
    rejectThresCnt = 5;
  }
  for (i = totalCnt; i > 0; i--)
  {
    pSvMeas = &(pMeas->meas[indx[i - 1]]);
    if (pSvMeas->stdPRDRDiff < 2.5 || rejectCnt > rejectThresCnt)
    {
      break;
    }
    if (pSvMeas->stdPRDRDiff > 5.0 && fabs(pSvMeas->pr_diff) > 25 && fabs(pSvMeas->dr_diff) > 1.5 && rejectCnt < 1)//reject largest
    {
      pSvMeas->status &= 0xFD;
      rejectCnt++;
#if defined(PLAYBACK_MODE)
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "DR reject %d %02d,%3d,%10.4f,%10.4f,%10.4f,%d", pSvMeas->freq_index, pSvMeas->gnssMode, pSvMeas->prn, prdrDiffStd[i - 1], diff[i - 1], pSvMeas->pr_diff, pSvMeas->cno);
#endif
    }
    else if (pSvMeas->stdPRDRDiff > 2.5 && fabs(pSvMeas->pr_diff) > 15 && (pSvMeas->drClusterA & 0x1))
    {
      pSvMeas->status &= 0xFD;
      rejectCnt++;
#if defined(PLAYBACK_MODE)
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "DR reject %d %02d,%3d,%10.4f,%10.4f,%10.4f,%d", pSvMeas->freq_index, pSvMeas->gnssMode, pSvMeas->prn, prdrDiffStd[i - 1], diff[i - 1], pSvMeas->pr_diff, pSvMeas->cno);
#endif
    }
    else if (pSvMeas->stdPRDRDiff > 3.0 && fabs(pSvMeas->pr_diff) > 10.0 && (pSvMeas->drClusterA & 0x1))
    {
      pSvMeas->status &= 0xFD;
      rejectCnt++;
#if defined(PLAYBACK_MODE)
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "DR reject %d %02d,%3d,%10.4f,%10.4f,%10.4f,%d", pSvMeas->freq_index, pSvMeas->gnssMode, pSvMeas->prn, prdrDiffStd[i - 1], diff[i - 1], pSvMeas->pr_diff, pSvMeas->cno);
#endif
    }
  }
}
/***********************************************************************
* 函数介绍: gnss_Qos_Main
* 输入参数：
* 输出参数：
* 返回值：
* 作者：
* 日期：09/03/
***********************************************************************/
void gnss_Qos_Main(meas_blk_t* pMeas, Kf_t* p_Kf, PeStateMonitor_t* p_state)
{
  // 1. CNO Mask check
  //gnss_Qos_Cno_Mask(pMeas);

  // 2. PR check to detect very large PR Error
  //gnss_Qos_Pr_Chck(pMeas);

  // 3. Dynamic Elevation selection according to average elevation
  gnss_Qos_Dynamic_Elevation(pMeas);

  // 4. MP de-weight according to multipath indicator
  //gnss_Qos_MP(pMeas);

  // 4. PRDR consistence check
  if (g_pe_cfg.chipType == QCOM)
  {
    gnss_Qos_PRDR_Check_QCOM(pMeas);
  }
  else if (g_pe_cfg.chipType == SPRD)
  {
    gnss_Qos_PRDR_Check_SPRD(pMeas);
  }
  else if (g_pe_cfg.chipType == UBLOX)
  {
    gnss_Qos_PRDR_Check_UBLOX(pMeas);
  }

  gnss_Qos_Dual_PRDR_Check(pMeas);

  // 5. inital outlier detection of PR meas before fix */
  gnss_Qos_InitOutlier_Detection(pMeas, p_Kf);

  // 6. MAD
  gnss_Qos_MAD(pMeas);

  // 7. check satellite quality
  if (g_pe_cfg.chipType == QCOM)
  {
    gnss_Qos_Check_SVQuality_QCOM(pMeas);
  }
  else if (g_pe_cfg.chipType == SPRD)
  {
    gnss_Qos_Check_SVQuality_SPRD(pMeas);
  }
  else if (g_pe_cfg.chipType == UBLOX)
  {
    gnss_Qos_Check_SVQuality_UBLOX(pMeas, p_Kf);
  }


  //8. Outlier check
  if (g_pe_cfg.chipType == QCOM)
  {
    gnss_Qos_Outlier_QCOM(pMeas);
  }
  else if (g_pe_cfg.chipType == SPRD)
  {
    gnss_Qos_Outlier_SPRD(pMeas);
  }
  else if (g_pe_cfg.chipType == UBLOX)
  {
    gnss_Qos_Outlier_UBLOX(pMeas);
  }

  // 9. DR Cluster
  if (g_pe_cfg.chipType == QCOM)
  {
    if (peMode.userSceData.isWALKMODE || peMode.staticData.staticFlag)
    {
      gnss_Qos_DR_Cluster_QCOM_WALK(pMeas, p_Kf, p_state);
    }
    else
    {
      gnss_Qos_DR_Cluster_QCOM(pMeas, p_Kf, p_state);
    }
  }
  else if (g_pe_cfg.chipType == SPRD)
  {
    gnss_Qos_DR_Cluster_SPRD(pMeas, p_Kf, p_state);
  }
  else if (g_pe_cfg.chipType == UBLOX)
  {
    gnss_Qos_DR_Cluster_UBLOX(pMeas, p_Kf, p_state);
  }


  // 10. PR Cluster
  if (g_pe_cfg.chipType == QCOM)
  {
    gnss_Qos_PR_Cluster_QCOM(pMeas, p_state);
  }
  else if (g_pe_cfg.chipType == SPRD)
  {
    gnss_Qos_PR_Cluster_SPRD(pMeas, p_state);
  }
  else if (g_pe_cfg.chipType == UBLOX)
  {
    if (!gnss_Qos_PR_Cluster_UBLOX(pMeas, p_state))
    {
      //gnss_Qos_PR_Cluster_Test(pMeas,p_state);
    }
  }

  gnss_Qos_PrDr_DrCheck(pMeas);

#if 0
  // 11. ReACQ process
  gnss_Qos_PrStatus_Check(pMeas);
#endif
  gnss_Qos_PrDrDiffSTD(pMeas);


  // 12. Weight count
  gnss_Qos_Chck_Count(pMeas);
    }
void gnss_Qos_PRDR_MAD(meas_blk_t* pMeas)
{
  uint8_t            flag, prRejNum = 0;
  uint32_t           i;
  uint8_t            index[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 };
  uint8_t            diffNum = 0;
  //uint8_t            status;
  float           diff[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, diffAbs[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 };
  float           median = 0.0, rejectK1 = 9.0, diffAbs_avg, diffAbs_std;

  if (pMeas->measCnt <= 5)
  {
    return;
  }

  if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_HD9310)
  {
    rejectK1 = 8.0;
  }
  else if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100)
  {
    rejectK1 = 9.0;
  }
  gnss_Pe_PRDR_Num(pMeas);

  // PR MAD outliers detection 
  for (i = 0; i < pMeas->measCnt; i++)
  {
    if (pMeas->meas[i].prn == 0) continue;
    if ((pMeas->meas[i].status & 0x1) == 0) continue;
    if (DBL_IS_EQUAL(pMeas->meas[i].sumPRDRDiff, 0.0f)) continue;
    diff[diffNum] = (float)pMeas->meas[i].sumPRDRDiff;
    index[diffNum] = i;
    diffNum++;
  }

  flag = gnss_MAD(diff, diffAbs, diffNum, &median);
  gnss_math_fstd(diff, diffNum, &diffAbs_avg, &diffAbs_std);
  if (flag == FALSE) {
    return;
  }

  // PR Reject Logic
  for (i = 0; i < diffNum; i++)
  {
    if ((diffAbs[i] > rejectK1 * (median / 0.6745)) && (fabs(pMeas->meas[index[i]].sumPRDRDiff) > 2.5) && (fabs(diffAbs_std) > 0.8))
    {
      pMeas->meas[index[i]].status &= 0xFE;
      prRejNum++;

#if defined(PLAYBACK_MODE)
      SYS_LOGGING(OBJ_QOS, LOG_INFO, "%s reject PR %02d %03d %d %f", __FUNCTION__, pMeas->meas[index[i]].gnssMode, pMeas->meas[index[i]].prn,
        pMeas->meas[index[i]].freq_index, pMeas->meas[index[i]].sumPRDRDiff);
#endif
    }
  }
}
