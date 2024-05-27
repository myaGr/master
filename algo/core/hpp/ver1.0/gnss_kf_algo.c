/************************************************************
* Copyrights(C)
* All rights Reserved
* �ļ����ƣ�gnss_kf_algo.c
* �ļ�������Navigation Filter ��λ����
* �汾�ţ�1.0.0
* ���ߣ�
* ���ڣ�08/29
************************************************************/
#include <string.h>
#include "gnss_kf.h"
#include "gnss_kf_math.h"
#include "gnss_kf_algo.h"
#include "gnss_sys_api.h"
#include "gnss_math.h"
#include "gnss_mode.h"
#include "gnss_pe.h"
#include "gnss_sd.h"

#undef MODULE_TAG
#define MODULE_TAG OBJ_KF

extern PE_MODES    peMode;
extern KF_INFO     kfInfo;
extern const uint8_t    clockBiasIdx[BIAS_NUM][BIAS_NUM];
extern const uint16_t   clockBiasBitMask[BIAS_NUM];
extern Gnss_Cfg_t  g_pe_cfg;
extern PeStateMonitor_t peState;
extern double realPosErr;
extern VDR_Gnss_Feedback_t   vdrFeedbackInfo;

void gnss_Kf_ZUPT_Hold_Static(KF_INFO* kfInfo_temp, double* deltaX, Kf_t* p_Kf)
{
  uint32_t      i;
  double      H[N_STATE] = { 0.0 };
  double      innovation, variance;
  // position hold
  variance = p_Kf->kf_Pvt_Info->posVarZupt;
  if (peMode.staticData.staticFlag && peMode.userSceData.isUnderEleRoad)
  {
    variance = variance * 6;
  }
  for (i = 0; i < 3; i++)
  {
    memset(H, 0, sizeof(double) * N_STATE);
    H[i] = -1.0;
    innovation = kfInfo_temp->X[i] + deltaX[i] - p_Kf->kf_Pvt_Info->ecefPos[i];
    udKFUpdate(H, deltaX, variance, innovation, SBOUND_PR, COAST_UPDATE, 1, 0);
  }
  // velocity hold
  variance = p_Kf->kf_Pvt_Info->velVarZupt;
  for (i = 0; i < 3; i++)
  {
    memset(H, 0, sizeof(double) * N_STATE);
    H[3 + i] = -1.0;
    innovation = kfInfo_temp->X[3 + i] + deltaX[3 + i];
    udKFUpdate(H, deltaX, variance, innovation, SBOUND_DR, COAST_UPDATE, 1, 0);
  }
}
void gnss_Kf_ZUPT(KF_INFO* kfInfo_temp, double* deltaX, Kf_t* p_Kf)
{
  uint32_t      i;
  double      H[N_STATE] = { 0.0 };
  double      innovation, variance;
  //double      test;

  // position hold
  variance = p_Kf->kf_Pvt_Info->posVarZupt;
  if (peMode.staticData.staticFlag && peMode.userSceData.isUnderEleRoad && g_pe_cfg.chipType != UBLOX)
  {
    variance = variance * 40;
  }
  for (i = 0; i < 3; i++)
  {
    memset(H, 0, sizeof(double) * N_STATE);
    H[i] = -1.0;
    innovation = 0;
    udKFUpdate(H, deltaX, variance, innovation, SBOUND_PR, COAST_UPDATE, 1, 0);
  }
  // velocity hold
  variance = p_Kf->kf_Pvt_Info->velVarZupt;
  if (peMode.staticData.staticFlag && peMode.userSceData.isUnderEleRoad)
  {
    variance = variance * 40;
  }
  for (i = 0; i < 3; i++)
  {
    memset(H, 0, sizeof(double) * N_STATE);
    H[3 + i] = -1.0;
    innovation = kfInfo_temp->X[3 + i];
    udKFUpdate(H, deltaX, variance, innovation, SBOUND_DR, COAST_UPDATE, 1, 0);
  }
  GLOGI("%s %8.3f", __FUNCTION__, kfInfo_temp->tor);
}

#if 0
/**********************************************************************
* Function Name:    gnss_Kf_Static_Detect
*
* Description:
*    Detect static constrain and lowSpeedMode
*
* Input:
*     lsPvtInfo:     current Ls result
*     kfPvtInfo:     last KF result
*
*Output:
*     kfStaticPara.staticFlag:    0 -- static constrain isn't detected
*                                 1 -- enter static constrain
*
*     kfStaticPara.lowSpeedMode:  0 -- high speed mode
*                                 1 -- low speed mode
*
* Return:
*
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnss_Kf_Static_Detect(Kf_t* p_Kf)
{
  uint8_t                     exitCnt;
  float                    lsSpeed, deltaDoplVar;
  float                    speedTh_1, speedTh_2, speedTh_3, speedTh_4, varTh_1, varTh_2, varTh_3;
  float                    lsENUvel[3];
  KF_PVT_INFO* kfPvtInfo;
  LS_PVT_INFO* lsPvtInfo;
  meas_blk_t* pMeas;
  STATIC_DATA* kfStaticData;
  GNSS_TIME* pTime;
  //PE_MODES*            peMode;

  //peMode = p_Kf->peMode;
  lsPvtInfo = p_Kf->ls_Pvt_Info;
  kfPvtInfo = p_Kf->kf_Pvt_Info;
  pMeas = p_Kf->meas_blk;
  kfStaticData = &p_Kf->kf_Static_Data;

  pTime = gnss_tm_get_time();

  //ls speed
  gnssConvEcef2EnuVel(lsPvtInfo->ecefVel, lsENUvel, kfPvtInfo->kfLLApos);
  lsSpeed = gnssClcSqrtSum_FLT(lsENUvel, 2);

  //doppler variance in static case
  deltaDoplVar = gnss_Kf_StaticDoplVar(lsPvtInfo, pMeas);

  //lowSpeedMode detection
  if (lsSpeed > 0 && p_Kf->kfCnt > 10 && kfStaticData->staticFlag == FALSE)
  {
    if (kfPvtInfo->kfHeadingVel > 1.0)
    {
      if (kfStaticData->avgCnt < 15)
      {
        kfStaticData->avgCnt++;
        kfStaticData->avgVel = kfStaticData->avgVel * (((float)kfStaticData->avgCnt - (float)1.0) / (float)kfStaticData->avgCnt) + kfPvtInfo->kfHeadingVel * ((float)1.0 / (float)kfStaticData->avgCnt);
      }
      else
      {
        kfStaticData->avgVel = kfStaticData->avgVel * (float)(14.0 / 15.0) + kfPvtInfo->kfHeadingVel * (float)(1.0 / 15.0);
      }
    }

    if (kfStaticData->avgVel < 2.0)
    {
      kfStaticData->lowSpeedMode = TRUE;
    }
    else
    {
      kfStaticData->lowSpeedMode = FALSE;
    }

    SYS_LOGGING(OBJ_KF, LOG_INFO, "Low speed mode: %d", kfStaticData->lowSpeedMode);
  }

  //static constrain detection
  speedTh_1 = (float)0.3;
  speedTh_2 = (float)0.6;
  speedTh_3 = (float)0.2;
  speedTh_4 = (float)0.3;

  varTh_1 = (float)0.2;
  varTh_2 = (float)0.4;
  varTh_3 = (float)0.3;


  //if (!kfStaticPara.lowSpeedMode /*&& !walk_mode*/)
  /*
  If average speed is large and current speed is lower, then it's vehicle mode
  */
  //if (kfPvtInfo->kfHeadingVel < 1.5 && kfStaticData->avgVel > 5.0)
  /*if (peMode.userContext.context == USER_CONTEXT_VEHICLE)
  {
  speedTh_1 = (float)0.8;
  speedTh_2 = (float)1.2;
  speedTh_3 = (float)1.0;
  varTh_1 = (float)0.5;
  varTh_2 = (float)1.0;
  }*/

  /* Enter static detection */
  if (kfStaticData->staticFlag == FALSE)
  {
    if (lsSpeed > 0)
    {
      if ((deltaDoplVar > 0 && deltaDoplVar < varTh_1 && lsSpeed < speedTh_2) || (lsSpeed < speedTh_1))
      {
        kfStaticData->speedStaticCnt++;
      }
      else
      {
        kfStaticData->speedStaticCnt = 0;
      }
    }
    else if (deltaDoplVar < varTh_1 && deltaDoplVar > 0.0)
    {
      kfStaticData->speedStaticCnt++;
    }
    else
    {
      kfStaticData->speedStaticCnt = 0;
    }

    if (kfStaticData->speedStaticCnt > 5)
    {
      kfStaticData->speedStaticCnt = 5;
    }

    if (kfStaticData->speedStaticCnt >= 2)
    {
      kfStaticData->staticFlag = TRUE;
      kfStaticData->speedDynamicCnt = 0;
    }
  }
  else
  {
    /* Exit static detection */
    if (lsSpeed > 0)
    {
      if (deltaDoplVar > varTh_3 && lsSpeed > speedTh_4)
      {
        kfStaticData->speedDynamicCnt = kfStaticData->speedDynamicCnt + 2;
      }
      if (deltaDoplVar > varTh_1 && lsSpeed > speedTh_3)
      {
        kfStaticData->speedDynamicCnt++;
      }
      else
      {
        kfStaticData->speedDynamicCnt = 0;
      }
    }
    else if (deltaDoplVar > varTh_2)
    {
      kfStaticData->speedDynamicCnt++;
    }
    else
    {
      kfStaticData->speedDynamicCnt = 0;
    }

    if (kfStaticData->speedDynamicCnt > 5)
    {
      kfStaticData->speedDynamicCnt = 5;
    }

    if (peMode.indoorMode.mode && kfStaticData->avgVel < 2 && kfStaticData->avgVel != 0.0)
    {
      exitCnt = 3;
    }
    else
    {
      exitCnt = 2;
    }

    if (kfStaticData->speedDynamicCnt >= exitCnt)
    {
      kfStaticData->staticFlag = FALSE;
      kfStaticData->speedStaticCnt = 0;
    }
  }

  kfStaticData->lsVel = lsSpeed;
  kfStaticData->deltaDoplVar = deltaDoplVar;

  SYS_LOGGING(OBJ_KF, LOG_INFO, "static constrain detect: tor = %16.8f, kfCnt = %05d, lsSpeed = %10.6f, doplVar = %8.4f, staticCnt = %d, dynamicCnt = %d, staticFlag = %d  %d",
    pTime->rcvr_time[GPS_MODE], p_Kf->kfCnt, lsSpeed, deltaDoplVar, kfStaticData->speedStaticCnt, kfStaticData->speedDynamicCnt, kfStaticData->staticFlag, peMode.staticData.staticFlag);

  kfPvtInfo->staticFlag = kfStaticData->staticFlag;
  // save history static flag
  kfStaticData->historyStatic = (kfStaticData->historyStatic << 1) | (kfStaticData->staticFlag & 0x1);
  if ((kfStaticData->historyStatic & 0x3) == 0x1)
  {
    p_Kf->errBack = kfPvtInfo->kfErrEst;
  }
}
#endif

/**********************************************************************
* Function Name:    gnss_Kf_Static_Proc
*
* Description:
*    static constrain process
*
* Input:
*     kfPvtInfo:     last KF result
*     kfStaticPara:  static constrain related parameters
*
*Output:
*     kfInfo->X:     KF position and velocity under static constrain
*
*
* Return:
*
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnss_Kf_Static_Proc(Kf_t* p_Kf)
{
  uint8_t                    i;
  double                   dLat, dLon;
  double                   distance;
  double                   dis_threshold = 100.0;
  double                   llaPos[3];
  double                   ratio;
  KF_PVT_INFO* kfPvtInfo;
  STATIC_DATA* pStaticData;
  USER_MOTION* pUsrContext;

  kfPvtInfo = p_Kf->kf_Pvt_Info;
  pStaticData = &peMode.staticData;
  pUsrContext = &(peMode.userContext);

  if (p_Kf->kfCnt <= 10)
  {
    return;
  }

  if ((pStaticData->historyStatic & 0x1) == 1)
  {
    for (i = 0; i < 3; i++)
    {
      pStaticData->ecefPos_static[i] = kfPvtInfo->ecefPos[i];
      pStaticData->llaPos_static[i] = kfPvtInfo->kfLLApos[i];
    }
  }

  if (pStaticData->staticFlag && (pUsrContext->isVehicleMode || (g_pe_cfg.chipType == UBLOX && (pStaticData->historyStatic & 0x3) == 0x1)))
  {
    if (g_pe_cfg.chipType == SPRD)
    {
      if (!peMode.userContext.isVehicleMode)
      {
        dis_threshold = 2.0; //ori 1m, then 3m, now back to 1m
      }
      else
      {
        dis_threshold = 16.0;
      }
    }
    else
    {
      if (peMode.userContext.context == USER_CONTEXT_HAND)
      {
        dis_threshold = 2.0; //ori 1m, then 3m, now back to 1m
      }
      else if (peMode.userContext.context == USER_CONTEXT_VEHICLE)
      {
        dis_threshold = 8.0;
      }
    }
    // If indoor mode, use large threshold 
    if (peMode.indoorMode.mode)
    {
      dis_threshold = 32.0;
    }
    gnssConvEcef2Lla(kfInfo.X, llaPos);
    dLat = (double)((llaPos[0] - pStaticData->llaPos_static[0]) * (180 / PI) * 111319);
    dLon = (double)((llaPos[1] - pStaticData->llaPos_static[1]) * (180 / PI) * 111133);
    distance = (double)sqrt(dLat * dLat + dLon * dLon);
    if (distance > dis_threshold)
    {
      ratio = 0.0;
    }
    else
    {
      ratio = 1.0;
    }
    for (i = 0; i < 3; i++)
    {
      pStaticData->ecefPos_static[i] = ratio * pStaticData->ecefPos_static[i] + (1.0 - ratio) * kfInfo.X[i];
    }
    SYS_LOGGING(OBJ_KF, LOG_INFO, "static move position: dis = %f, threshold = %f", distance, dis_threshold);
    gnssConvEcef2Lla(pStaticData->ecefPos_static, pStaticData->llaPos_static);
    // use static position to set X and set velocity to zero
    for (i = 0; i < 3; i++)
    {
      kfInfo.X[i] = pStaticData->ecefPos_static[i];
      kfPvtInfo->ecefVel[i] = (float)kfInfo.X[i + 3];
      kfInfo.X[i + 3] = 0.0;
#ifndef PVMODE
      kfInfo.X[i + 11] = 0.0;
#endif
    }

    ////enlarge Q code.
    //p_Kf->enlargeQ = TRUE;
    //p_Kf->QFactor = 1.0;
    //p_Kf->enlargeCnt = 0;
  }//if staticFlag    
}

/***********************************************************************
* ��������: gnss_Kf_DR_ErrNum
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�12/08/
***********************************************************************/
void gnss_Kf_DR_ErrNum(uint8_t drNum, uint8_t* DrRejLmt)
{
  if (drNum >= 13)
  {
    *DrRejLmt = 5;
  }
  else if (drNum >= 10)
  {
    *DrRejLmt = 4;
  }
  else if (drNum >= 7)
  {
    *DrRejLmt = drNum - 6;
  }
  else
  {
    *DrRejLmt = 0;
  }
}

/***********************************************************************
* ��������: gnss_Kf_PosBias_Detect
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�10/24/
***********************************************************************/
void gnss_Kf_PosBias_Detect(Kf_t* p_Kf, KF_INFO* kfInfo_temp)
{
  static uint8_t          posGoodStatic = FALSE;
  static uint8_t          exitCnt = 0, enterCnt = 0;
  uint8_t                 cnt2 = 0;
  uint8_t                 a;
  uint8_t                 flag1 = FALSE, byPassFlag = FALSE;
  uint8_t                 testCnt;
  uint8_t                 i, j, updateFlag = TRUE, idx;
  gnss_meas_t* pSvMeas;
  meas_blk_t* pMeas;
  //SV_INFO* pSvInfo;
  sat_data_t* sp;
  float                prNoiseVar;
  double                deltaX[N_STATE] = { 0 }, PR, test, testSave[MEAS_SAVE_NUM];
  double                dt;
  double                testAvg, testStd;

  pMeas = p_Kf->meas_blk;

  SYS_LOGGING(OBJ_KF, LOG_INFO, "POSITION BIAS STATUS:%14.6f,%d", pMeas->tor, peState.posHasBias);
  /*
  state machine for position bias detection, 0 & 1, 0--> position is correct, 1 --> position is biased
  */
  if (peState.posHasBias)
  {
    if (pMeas->isSmallDiff || pMeas->prDiff10Cnt >= 8 || pMeas->prDiff20Cnt >= 12)
    {
      exitCnt++;
      goto POSBIAS_EXIT_CHECK;
    }
    /* Keep 100 second */
    if (p_Kf->torOfAdjP > 0.0 && fabs(pMeas->tor - p_Kf->torOfAdjP) < 100.0)
    {
      if (exitCnt > 0)
      {
        exitCnt--;
      }
      return;
    }
    /* If good measurements or KF RSLT is good, then don't detect */
    if (p_Kf->isKfRsltGood)
    {
      exitCnt++;
      goto POSBIAS_EXIT_CHECK;
    }

    /* There is no reject and De-weight in KF */
    a = p_Kf->kf_ctrl.prDewNum + p_Kf->kf_ctrl.prRejNum;
    if ((double)a / p_Kf->kf_ctrl.prNum < 0.2 && p_Kf->kf_ctrl.prNum >= 10)
    {
      exitCnt++;
      goto POSBIAS_EXIT_CHECK;
    }
    if (exitCnt > 0)
    {
      exitCnt--;
    }
  POSBIAS_EXIT_CHECK:
    if (exitCnt >= 20)
    {
      peState.posHasBias = FALSE;
      p_Kf->torOfAdjP = 0.0;
      exitCnt = 0;
      enterCnt = 0;
      SYS_LOGGING(OBJ_KF, LOG_INFO, "EXIT POSTIION BIAS(%14.6f)", pMeas->tor);
      return;
    }
  }
  else
  {
    /*
    (1) If small DIFF or KF result is good, don't check
    (2) Check KF results quality according to DIFF
    */
    if (pMeas->isSmallDiff || p_Kf->isKfRsltGood || pMeas->prDiff10Cnt >= 6 || pMeas->prDiff20Cnt >= 8)
    {
      byPassFlag = TRUE;
    }


    if (pMeas->prDiff10Cnt >= 4 && pMeas->prDiff20Cnt >= 6 && p_Kf->posResStd < 25.0)
    {
      byPassFlag = TRUE;
    }

    /*
    If KF is good at last epoch, then by pass
    */
    a = p_Kf->kf_ctrl.prDewNum + p_Kf->kf_ctrl.prRejNum;
    if (p_Kf->posResStd > 0.0 && p_Kf->posResStd < 25.0 && (p_Kf->kf_ctrl.prNum - a) >= 8)
    {
      byPassFlag = TRUE;
    }

    /* If static and more than 8 satellite DIFF smaller than 20, then position is good */
    if (peMode.staticData.staticFlag && pMeas->prDiff20Cnt >= 8)
    {
      byPassFlag = TRUE;
    }

    if (byPassFlag)
    {
      enterCnt = 0;
      exitCnt = 0;
      if (peMode.staticData.staticFlag)
      {
        posGoodStatic = TRUE;
      }
      else
      {
        posGoodStatic = FALSE;
      }
      return;
    }

    /*
    If can't determine position is good, but if static and posGoodStatic is true, then don't check
    */
    if (peMode.staticData.staticFlag && posGoodStatic)
    {
      return;
    }
    else
    {
      posGoodStatic = FALSE;
    }

    for (i = 0; i < pMeas->measCnt; i++)
    {
      pSvMeas = &pMeas->meas[i];
      //pSvInfo = &pSvMeas->sv_info;

      /* get the idx of clock bias by satMode and clockBiasFlag */
      idx = clockBiasIdx[pSvMeas->gnssMode][p_Kf->kf_ctrl.biasLinkFlag];

      //pr update
      memset(&deltaX, 0, N_STATE * sizeof(double));
      sp = gnss_sd_get_sv_data(pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->freq_index);
      if (sp == NULL) continue;
      if ((pSvMeas->status & 0x1) && fabs(pSvMeas->pr_diff) >= 15.0)// PR valid
      {
        prNoiseVar = gnss_Kf_PrNoiseVarCal(pSvMeas, pMeas->avgCno, 0);
        PR = pSvMeas->pseudoRange;
        gnss_Kf_PR_Adj(p_Kf, pSvMeas->gnssMode, &PR);
        //gnss_Kf_PrUpdate(pSvInfo->dcos, deltaX, pSvMeas->range, PR, &prNoiseVar, pSvMeas->prChckCnt, &updateFlag, 0, idx);
        gnss_Kf_PrUpdate(pSvMeas, deltaX, PR, &prNoiseVar, &updateFlag, &test, 0, 0, idx);
        sp->measBack[0].PRKfTest = test;

        SYS_LOGGING(OBJ_KF, LOG_INFO, "%s,%10.4f,%02d,%02d,%03d,%10.4f,%10.4f", __FUNCTION__, pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->cno, pSvMeas->pr_diff, test);

        /* calculate PR Test variance */
        testSave[0] = test;
        testCnt = 1;
        for (j = 0; j < MEAS_SAVE_NUM - 1; j++)
        {
          dt = sp->measBack[j].tot - sp->measBack[j + 1].tot;
          if (fabs(dt) > 1.5 || fabs(sp->measBack[j + 1].PRKfTest) < 1e-3)
          {
            break;
          }
          testSave[j + 1] = sp->measBack[j + 1].PRKfTest;
          testCnt++;
        }
        flag1 = FALSE;
        if (testCnt > 2)
        {
          gnss_math_dstd(testSave, testCnt, &testAvg, &testStd);
          if (fabs(testAvg) >= 4.0 && testStd < 1.5)
          {
            flag1 = TRUE;
          }
          else if (fabs(testAvg) >= 6.0 && testStd < 2.0)
          {
            flag1 = TRUE;
          }
          if (flag1 == TRUE)
          {
            cnt2++;
            SYS_LOGGING(OBJ_KF, LOG_INFO, "KF TestAvg:%10.4f,%02d,%02d,%03d,%6.2f,%6.2f,%02d,%02d,%10.4f,%10.4f", pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, pSvMeas->cno, testAvg, testStd, testCnt, peMode.staticData.staticFlag, pSvMeas->pr_diff, p_Kf->posResStd);
          }
        }
      }
      else
      {
        sp->measBack[0].PRKfTest = 0.0;
      }
    }
    if (cnt2 >= 2)
    {
      enterCnt++;
    }
    if (enterCnt >= 2)
    {
      SYS_LOGGING(OBJ_KF, LOG_INFO, "POSITION BIAS DETECTED(%14.6f,%6.2f)", pMeas->tor, realPosErr);
      peState.posHasBias = TRUE;
    }
    else
    {
      peState.posHasBias = FALSE;
    }

    if (peState.posHasBias)
    {
      enterCnt = 0;
      exitCnt = 0;
      p_Kf->torOfAdjP = pMeas->tor;
      /* Adjust position post-variance */
      for (i = 1; i < 4; i++)
      {
        kfInfo_temp->D_minus[i] *= 2.0;
      }
    }
  }
}

/***********************************************************************
* ��������: gnss_Kf_Adjust_PMinus
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�12/10/
***********************************************************************/
void gnss_Kf_Adjust_PMinus(KF_INFO* kfInfo_temp, Kf_t* p_Kf)
{
  uint32_t          k;
  double          prRatio = 1.0, drRatio = 1.0;
  meas_blk_t* pMeas;

  pMeas = p_Kf->meas_blk;

  /* PR Ratio */
  if (g_pe_cfg.automobile == TRUE)
  {
    if (!peMode.userSceData.isUnderEleRoad)
    {
      prRatio = 20.0;
    }
    else
    {
      prRatio = 10.0;
    }
  }
  else
  {
    if (!peMode.userSceData.isUnderEleRoad)
    {
      if (peMode.userSceData.isPDRMode)
      {
        prRatio = 1.0;
      }
      else
      {
        prRatio = 20.0;
      }
    }
    else
    {
      prRatio = 1.0;
    }
  }


  /* DR Ratio */
  if (pMeas->avgCno >= 20)
  {
    drRatio = 20.0;
  }
  else if (pMeas->avgCno >= 18)
  {
    drRatio = 10.0;
  }

  for (k = 0; k < N_MAT; k++)
  {
    kfInfo_temp->U_plus[k] *= 1.0;
  }

  /* Adjust position post-variance */
  for (k = 1; k < 4; k++)
  {
    kfInfo_temp->D_plus[k] *= prRatio;
  }

  /* Adjust velocity post-variance */
  for (k = 4; k < 7; k++)
  {
    kfInfo_temp->D_plus[k] *= drRatio;
  }

  /* Adjust Q */
  for (k = 0; k < 3; k++)
  {
    kfInfo_temp->Q[k][k] *= 1.0;
  }
}


/***********************************************************************
* ��������: gnss_Kf_Bias_Adjust
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�12/28/
***********************************************************************/
void gnss_Kf_Bias_Adjust(Kf_t* p_Kf)
{
  uint8_t                     flag;
  int32_t                    index[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM];
  uint32_t                    i, idx, cnt = 0, cntUse = 0;
  double                    prKf;
  meas_blk_t* pMeas;
  gnss_meas_t* pSvMeas;
  double                    inno[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM], innoAbs[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM];
  double                    avg, std, median, rejectK = 10;
  double                    biasDiffUse[GNSS_MAX_MODE];

  pMeas = p_Kf->meas_blk;

  if (pMeas->rtdUsed)
  {
    memcpy(biasDiffUse, p_Kf->biasDiff, GNSS_MAX_MODE * sizeof(double));
  }
  else
  {
    memcpy(biasDiffUse, p_Kf->biasDiffLocal, GNSS_MAX_MODE * sizeof(double));
  }

  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &(pMeas->meas[i]);
    if (pSvMeas->prn == 0) continue;
    if ((pSvMeas->status & 0x1) == 0) continue;
    if (pSvMeas->freq_index > 0) continue;
    idx = clockBiasIdx[pSvMeas->gnssMode][p_Kf->kf_ctrl.biasLinkFlag];

    prKf = pSvMeas->range + kfInfo.X[idx];
    if (p_Kf->kf_ctrl.biasLinkFlag == 1)
    {
      if (pSvMeas->gnssMode == GLN_MODE)
      {
        prKf -= biasDiffUse[GLN_MODE];
      }
    }
    else if (p_Kf->kf_ctrl.biasLinkFlag == 2)
    {
      if (pSvMeas->gnssMode == GLN_MODE)
      {
        prKf -= biasDiffUse[GLN_MODE];
      }
      else if (pSvMeas->gnssMode == BDS_MODE)
      {
        prKf -= biasDiffUse[BDS_MODE];
      }
    }
    else if (p_Kf->kf_ctrl.biasLinkFlag == 3)
    {
      if (pSvMeas->gnssMode == GLN_MODE)
      {
        prKf -= biasDiffUse[GLN_MODE];
      }
      else if (pSvMeas->gnssMode == BDS_MODE)
      {
        prKf -= biasDiffUse[BDS_MODE];
      }
      else if (pSvMeas->gnssMode == GAL_MODE)
      {
        prKf -= biasDiffUse[GAL_MODE];
      }
    }
    inno[cnt] = (pSvMeas->pseudoRange - prKf);
    index[cnt] = cnt;
    cnt++;
  }

  if (cnt < 2) return;

  cntUse = cnt;

  if (cnt >= 6)
  {
    flag = gnss_MAD_DBL(inno, innoAbs, cntUse, &median);
    if (flag == FALSE) return;

    /* reject out-lies according to MAD */
    for (i = 0; i < cnt; i++)
    {
      if (innoAbs[i] > rejectK * (median / 0.6745))
      {
        index[i] = -1;
        cntUse--;
      }
    }
  }

  avg = 0.0;
  for (i = 0; i < cnt; i++)
  {
    if (index[i] == -1)
    {
      continue;
    }
    avg += inno[i];
  }
  avg /= cntUse;

  std = 0.0;
  for (i = 0; i < cnt; i++)
  {
    if (index[i] == -1)
    {
      continue;
    }
    std += (inno[i] - avg) * (inno[i] - avg);
  }
  std = sqrt(std / cntUse);

  p_Kf->avgInno = avg;
  p_Kf->stdInno = std;

  if (std < 1000.0 && avg < 5000)
  {
    SYS_LOGGING(OBJ_KF, LOG_INFO, "%s,%10.3f,%10.3f", __FUNCTION__, avg, std);
  }

  if ((p_Kf->lastKfStatus & KF_RESTART) && (p_Kf->pCfg->chipType == SPRD || p_Kf->pCfg->chipType == QCOM))
  {
    if (std < 40.0 && fabs(avg) >= 100.0)
    {
      SYS_LOGGING(OBJ_KF, LOG_INFO, "KFBIASADJUST:avg:%10.3f,%10.3f", avg, std);
      goto ADJUSTBIAS;
    }
    else
    {
      return;
    }
  }
  else
  {
    // case1
    if (std < 3.0 && fabs(avg) >= 10 && pMeas->avgCno >= 25)
    {
      SYS_LOGGING(OBJ_KF, LOG_INFO, "KFBIASADJUST:avg:%10.3f,%10.3f", avg, std);
      goto ADJUSTBIAS;
    }

    if (std < 20.0 && fabs(avg) >= 50)
    {
      SYS_LOGGING(OBJ_KF, LOG_INFO, "KFBIASADJUST:avg:%10.3f,%10.3f", avg, std);
      goto ADJUSTBIAS;
    }
    // case2
    if (std < 25.0 && fabs(avg) >= 70.0)
    {
      SYS_LOGGING(OBJ_KF, LOG_INFO, "KFBIASADJUST:avg:%10.3f,%10.3f", avg, std);
      goto ADJUSTBIAS;
    }
    // case3
    if (std < 30.0 && fabs(avg) >= 120.0 && pMeas->avgCno >= 25)
    {
      SYS_LOGGING(OBJ_KF, LOG_INFO, "KFBIASADJUST:avg:%10.3f,%10.3f", avg, std);
      goto ADJUSTBIAS;
    }
    // case4
    if (std < 50.0 && fabs(avg) >= 100.0 && pMeas->avgCno < 25)
    {
      SYS_LOGGING(OBJ_KF, LOG_INFO, "KFBIASADJUST:avg:%10.3f,%10.3f", avg, std);
      goto ADJUSTBIAS;
    }
    // case5
    if (std < 50.0 && fabs(avg) >= 200.0)
    {
      SYS_LOGGING(OBJ_KF, LOG_INFO, "KFBIASADJUST:avg:%10.3f,%10.3f", avg, std);
      goto ADJUSTBIAS;
    }
    // case6
    if (std < 100.0 && fabs(avg) >= 2000.0)
    {
      SYS_LOGGING(OBJ_KF, LOG_INFO, "KFBIASADJUST:avg:%10.3f,%10.3f", avg, std);
      goto ADJUSTBIAS;
    }
    return;
  }

ADJUSTBIAS:
  kfInfo.X[6] += avg;
  kfInfo.X[7] += avg;
  kfInfo.X[8] += avg;
  kfInfo.X[9] += avg;

}
/***********************************************************************
* ��������: gnss_Kf_Drif_Adjust
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�12/28/
***********************************************************************/
void gnss_Kf_Drift_Adjust(Kf_t* p_Kf)
{
  uint32_t                    i, j, cnt = 0;
  double                    drKf;
  meas_blk_t* pMeas;
  gnss_meas_t* pSvMeas;
  double                    inno[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM];
  double                    avg, std;
  double* dcos;
  float* svVel;

  pMeas = p_Kf->meas_blk;

#if defined(PLAYBACK_MODE)
  SYS_LOGGING(OBJ_KF, LOG_INFO, "DRIFT COM:%10.4f,%6.2f,%6.2f", pMeas->tor, kfInfo.X[10], pMeas->clock.drift_nsps * LIGHT_NSEC);
#endif

  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &(pMeas->meas[i]);
    if (pSvMeas->prn == 0) continue;
    if ((pSvMeas->status & 0x2) == 0) continue;
    if (pSvMeas->freq_index > 0) continue;

    drKf = 0;

    dcos = &(pSvMeas->sv_info.dcos[0]);
    svVel = &(pSvMeas->sv_info.v[0]);

    for (j = 0; j < 3; j++)
    {
      drKf += dcos[j] * (kfInfo.X[j + 3] - (double)svVel[j]);
    }

    drKf += kfInfo.X[10];

    inno[cnt++] = pSvMeas->pseudoRangeRate - drKf;
  }

  if (cnt < 5) return;

  avg = 0.0;
  for (i = 0; i < cnt; i++)
  {
    avg += inno[i];
  }
  avg /= cnt;

  std = 0.0;
  for (i = 0; i < cnt; i++)
  {
    std += (inno[i] - avg) * (inno[i] - avg);
  }
  std = sqrt(std / cnt);

  if (p_Kf->pCfg->chipType == SPRD || p_Kf->pCfg->chipType == QCOM || p_Kf->pCfg->sub_chipType == SUB_CHIPTYPE_ST_8100)
  {
    if (p_Kf->lastKfStatus & KF_RESTART)
    {
      if (std < 2.5 && fabs(avg) >= 5.0)
      {
        SYS_LOGGING(OBJ_KF, LOG_INFO, "KFDRIFTADJUST:avg:%10.3f,%10.3f", avg, std);
        goto ADJUSTBIAS;
      }
      else
      {
        return;
      }
    }
    else
    {
      if (std < 0.2 && fabs(avg) >= 2.0)
      {
        SYS_LOGGING(OBJ_KF, LOG_INFO, "KFDRIFTADJUST:avg:%10.3f,%10.3f", avg, std);
        goto ADJUSTBIAS;
      }
      else if (std < 1.0 && fabs(avg) >= 3.0)
      {
        SYS_LOGGING(OBJ_KF, LOG_INFO, "KFDRIFTADJUST:avg:%10.3f,%10.3f", avg, std);
        goto ADJUSTBIAS;
      }
      else if (std < 1.5 && fabs(avg) >= 5.0)
      {
        SYS_LOGGING(OBJ_KF, LOG_INFO, "KFDRIFTADJUST:avg:%10.3f,%10.3f", avg, std);
        goto ADJUSTBIAS;
      }
      else if (std < 2.0 && fabs(avg) >= 8.0)
      {
        SYS_LOGGING(OBJ_KF, LOG_INFO, "KFDRIFTADJUST:avg:%10.3f,%10.3f", avg, std);
        goto ADJUSTBIAS;
      }
      else
      {
        SYS_LOGGING(OBJ_KF, LOG_INFO, "KFDRIFTNOTADJUST:avg:%10.3f,%10.3f", avg, std);
        return;
      }
    }
  }
  else
  {
    return;
  }

ADJUSTBIAS:
  kfInfo.X[10] += avg;
}

/***********************************************************************
* ��������: gnss_Kf_Lla_HeadingCheck
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/19/
***********************************************************************/
void gnss_Kf_Lla_HeadingCheck(uint8_t n, uint8_t* indx, float* deltaHead, meas_blk_t* pMeas)
{
  uint8_t          i, flag;
  uint8_t          rejLimit = 2, rejNum = 0;
  float         median, rejectK1 = 8.0;
  float         deltaHeadAbs[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM];

  gnss_Pe_PRDR_Num(pMeas);

  if (pMeas->validPrNum < 9)
  {
    return;
  }

  flag = gnss_median(deltaHead, n, &median);
  if (flag == FALSE) return;

  for (i = 0; i < n; i++)
  {
    if (((double)deltaHead[i] - median) > PI)
    {
      deltaHead[i] -= (float)(2 * PI);
    }
    else if (((double)deltaHead[i] - median) < -PI)
    {
      deltaHead[i] += (float)(2 * PI);
    }
  }

  flag = gnss_MAD(deltaHead, deltaHeadAbs, n, &median);
  if (flag == FALSE) return;

  gnss_Sort_WithIndx(deltaHeadAbs, indx, n);

  for (i = n; i > 0; i--)
  {
    if (deltaHeadAbs[i - 1] > rejectK1 * (median / 0.6745) && deltaHeadAbs[i - 1] > 0.1 &&
      (pMeas->meas[indx[i - 1]].quality & PR_GOOD) == FALSE && rejNum < rejLimit)
    {
      if (peMode.userSceData.isPDRMode)
      {
        pMeas->meas[indx[i - 1]].status &= 0xFE;
        rejNum++;

        SYS_LOGGING(OBJ_KF, LOG_INFO, "%s,%02d,%02d,%10.4f,%10.4f", __FUNCTION__, pMeas->meas[indx[i - 1]].gnssMode,
          pMeas->meas[indx[i - 1]].prn, deltaHead[i - 1], deltaHeadAbs[i - 1]);
      }
    }
  }
}
#if 0



void gnss_Kf_Lla_HeadingCheckFlp(uint8_t n, uint8_t* indx, float* deltaHead, meas_blk_t* pMeas)
{
  char str[2048] = { '\0' }, * p = &str[0], strprn[2048] = { '\0' }, * pPrn = &strprn[0], strRaw[2048] = { '\0' }, * pRaw = &strRaw[0];
  float llaHeadingDiff[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 }, llaHeadingDiffAbs[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 }, llaHeadingDiffAbsRaw[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 };
  float fHeadingDiffMean = 0, fHeadingDiffStd = 0, fHeadingDiffMeanPart = 0, fHeadingDiffStdPart = 0;
  uint8_t uHeadingValid = 0;
  uint8_t uprn[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 };
  S16         i = 0;
  uint8_t          flag = 0;
  uint8_t          rejLimit = 2, rejNum = 0;
  float         median, rejectK1 = 10.0;

  gnss_Pe_PRDR_Num(pMeas);
  if (pMeas->validPrNum < 9)
  {
    return;
  }
  uHeadingValid = gnss_Pe_Ins_Vel_Check(&vdrFeedbackInfo, FLP_FEEDBACK_HEADING);
  if (!uHeadingValid)
  {
    return;
  }
  pRaw += sprintf(pRaw, "LLAHeadingFLPCheckRaw: %8.3f     ", pMeas->tor);
  p += sprintf(p, "LLAHeadingFLPCheck: %8.3f     ", pMeas->tor);
  pPrn += sprintf(pPrn, "LLAHeadingFLP_PrnCheck: %8.3f", pMeas->tor);

  for (i = 0; i < n; i++)
  {
    uprn[i] = pMeas->meas[indx[i]].prn;
    llaHeadingDiff[i] = deltaHead[i] * RAD2DEG - vdrFeedbackInfo.heading;
    if (llaHeadingDiff[i] > 180)
    {
      llaHeadingDiff[i] -= 360;
    }
    else if (llaHeadingDiff[i] < -180)
    {
      llaHeadingDiff[i] += 360;
    }
    llaHeadingDiffAbsRaw[i] = fabs(llaHeadingDiff[i]);
  }

  flag = gnss_MAD(llaHeadingDiff, llaHeadingDiffAbs, n, &median);
  if (flag == FALSE) return;

  gnss_Sort_WithIndx_1(llaHeadingDiffAbs, llaHeadingDiffAbsRaw, indx, n);



  rejectK1 = 20;
  for (i = n; i > 0; i--)
  {
    if ((llaHeadingDiffAbs[i - 1] > 50 * (median / 0.6745) && llaHeadingDiffAbs[i - 1] > 10)
      /*&& (pMeas->meas[indx[i - 1]].quality & PR_GOOD) == FALSE*/ && rejNum < rejLimit
      )
    {
      pMeas->meas[indx[i - 1]].status &= 0xFD;
      rejNum++;

      SYS_LOGGING(OBJ_KF, LOG_INFO, "MAD%s,%8.3f,%02d,%02d,%10.4f,%10.4f,%10.4f", __FUNCTION__, pMeas->tor, pMeas->meas[indx[i - 1]].gnssMode,
        pMeas->meas[indx[i - 1]].prn, llaHeadingDiff[i - 1], llaHeadingDiffAbs[i - 1],
        pMeas->meas[indx[i - 1]].pr_err);
    }
  }

  gnss_Sort_WithIndx_1(llaHeadingDiffAbsRaw, llaHeadingDiffAbs, uprn, n);
  gnss_math_fstd(llaHeadingDiffAbsRaw, n, &fHeadingDiffMean, &fHeadingDiffStd);

  gnss_Pe_PRDR_Num(pMeas);
  if (pMeas->validPrNum < 9)
  {
    return;
  }
  gnss_math_fstd(llaHeadingDiffAbsRaw, n - 2, &fHeadingDiffMeanPart, &fHeadingDiffStdPart);

  if (rejNum == rejLimit)
  {
    rejLimit = 3;
  }
  for (i = n; i > 0; i--)
  {
    //if (((llaHeadingDiffAbsRaw[i - 1] > THRES * 1.5 && fHeadingDiffStdPart * 1.5 < fHeadingDiffStd) || (llaHeadingDiffAbsRaw[i - 1] > THRES * 2.5 && fHeadingDiffStdPart * 1.2 < fHeadingDiffStd))
    //	&& (pMeas->meas[indx[i - 1]].quality & DR_GOOD) == FALSE && rejNum < rejLimit
    //	)
    if ((llaHeadingDiffAbsRaw[i - 1] > 170) && (rejNum < rejLimit && fHeadingDiffStdPart < fHeadingDiffStd))
    {
      pMeas->meas[indx[i - 1]].status &= 0xFD;
      rejNum++;

      SYS_LOGGING(OBJ_KF, LOG_INFO, "Sort%s,%8.3f,%02d,%02d,%10.4f,%10.4f,%10.4f%10.4f", __FUNCTION__, pMeas->tor, pMeas->meas[indx[i - 1]].gnssMode,
        pMeas->meas[indx[i - 1]].prn, fHeadingDiffStdPart, fHeadingDiffStd, llaHeadingDiffAbsRaw[i - 1],
        pMeas->meas[indx[i - 1]].pr_err);

    }
  }


  pRaw += sprintf(pRaw, " %8.6f  %8.6f  %8.6f  %8.6f ", fHeadingDiffMean, fHeadingDiffStd, fHeadingDiffMeanPart, fHeadingDiffStdPart);
  p += sprintf(p, " %8.6f ", median);
  for (i = n - 1; i >= 0; i--)
  {
    pRaw += sprintf(pRaw, " %8.6f ", llaHeadingDiffAbsRaw[i]);
    pPrn += sprintf(pPrn, " %02d ", uprn[i]);
    p += sprintf(p, " %8.6f ", llaHeadingDiffAbs[i]);
  }
  SYS_LOGGING(OBJ_KF, LOG_INFO, "%s", strRaw);
  SYS_LOGGING(OBJ_KF, LOG_INFO, "%s", strprn);
  SYS_LOGGING(OBJ_KF, LOG_INFO, "%s", str);
  GLOGI("LLAFlpHeadingMax: %8.3f  %d %d %d %8.3f %8.3f %8.3f %8.3f %8.6f  %8.6f  %8.6f  %8.6f %8.6f %d", pMeas->tor, pMeas->meas[indx[n - 1]].gnssMode,
    pMeas->meas[indx[n - 1]].prn, pMeas->meas[indx[n - 1]].quality & PR_GOOD,
    (llaHeadingDiffAbs[n - 1] / (median / 0.6745)), llaHeadingDiffAbs[n - 1], median,
    llaHeadingDiffAbsRaw[n - 1],
    fHeadingDiffMean, fHeadingDiffStd, fHeadingDiffMeanPart, fHeadingDiffStdPart, fabs(pMeas->meas[indx[n - 1]].pr_err),
    pMeas->meas[indx[n - 1]].cno);

}

void gnss_Kf_Vel_HeadingCheckFlp(uint8_t n, uint8_t* indx, float* deltaHead, meas_blk_t* pMeas, float THRES)
{
  char str[2048] = { '\0' }, * p = &str[0], strprn[2048] = { '\0' }, * pPrn = &strprn[0], strRaw[2048] = { '\0' }, * pRaw = &strRaw[0];
  float llaHeadingDiff[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 }, llaHeadingDiffAbs[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 }, llaHeadingDiffAbsRaw[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 };
  float fHeadingDiffMean = 0, fHeadingDiffStd = 0, fHeadingDiffMeanPart = 0, fHeadingDiffStdPart = 0;
  uint8_t uHeadingValid = 0;
  uint8_t uprn[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 };
  S16         i = 0;
  uint8_t          flag = 0;
  uint8_t          rejLimit = 2, rejNum = 0;
  float         median, rejectK1 = 10.0;

  gnss_Pe_PRDR_Num(pMeas);
  if (pMeas->validDrNum < 7)
  {
    return;
  }
  uHeadingValid = gnss_Pe_Ins_Vel_Check(&vdrFeedbackInfo, FLP_FEEDBACK_HEADING);
  if (!uHeadingValid)
  {
    return;
  }
  pRaw += sprintf(pRaw, "VelHeadingFLPCheckRaw: %8.3f     ", pMeas->tor);
  p += sprintf(p, "VelHeadingFLPCheck: %8.3f     ", pMeas->tor);
  pPrn += sprintf(pPrn, "VelHeadingFLP_PrnCheck: %8.3f", pMeas->tor);

  for (i = 0; i < n; i++)
  {
    uprn[i] = pMeas->meas[indx[i]].prn;
    llaHeadingDiff[i] = deltaHead[i] * RAD2DEG - vdrFeedbackInfo.heading;
    if (llaHeadingDiff[i] > 180)
    {
      llaHeadingDiff[i] -= 360;
    }
    else if (llaHeadingDiff[i] < -180)
    {
      llaHeadingDiff[i] += 360;
    }
    llaHeadingDiffAbsRaw[i] = fabs(llaHeadingDiff[i]);
  }

  flag = gnss_MAD(llaHeadingDiff, llaHeadingDiffAbs, n, &median);
  if (flag == FALSE) return;

  gnss_Sort_WithIndx_1(llaHeadingDiffAbs, llaHeadingDiffAbsRaw, indx, n);



  rejectK1 = 20;
  for (i = n; i > 0; i--)
  {
    if (((llaHeadingDiffAbs[i - 1] > rejectK1 * (median / 0.6745))
      || (llaHeadingDiffAbs[i - 1] > rejectK1))
      && (pMeas->meas[indx[i - 1]].quality & DR_GOOD) == FALSE && rejNum < rejLimit
      )
    {
      pMeas->meas[indx[i - 1]].status &= 0xFD;
      rejNum++;

      SYS_LOGGING(OBJ_KF, LOG_INFO, "MAD%s,%8.3f,%02d,%02d,%10.4f,%10.4f,%10.4f", __FUNCTION__, pMeas->tor, pMeas->meas[indx[i - 1]].gnssMode,
        pMeas->meas[indx[i - 1]].prn, llaHeadingDiff[i - 1], llaHeadingDiffAbs[i - 1],
        pMeas->meas[indx[i - 1]].dr_err);
    }
  }

  gnss_Sort_WithIndx_1(llaHeadingDiffAbsRaw, llaHeadingDiffAbs, uprn, n);
  gnss_math_fstd(llaHeadingDiffAbsRaw, n, &fHeadingDiffMean, &fHeadingDiffStd);

  gnss_Pe_PRDR_Num(pMeas);
  if (pMeas->validDrNum < 7)
  {
    return;
  }
  gnss_math_fstd(llaHeadingDiffAbsRaw, n - 2, &fHeadingDiffMeanPart, &fHeadingDiffStdPart);

  if (rejNum == rejLimit)
  {
    rejLimit = 3;
  }
  for (i = n; i > 0; i--)
  {
    //if (((llaHeadingDiffAbsRaw[i - 1] > THRES * 1.5 && fHeadingDiffStdPart * 1.5 < fHeadingDiffStd) || (llaHeadingDiffAbsRaw[i - 1] > THRES * 2.5 && fHeadingDiffStdPart * 1.2 < fHeadingDiffStd))
    //	&& (pMeas->meas[indx[i - 1]].quality & DR_GOOD) == FALSE && rejNum < rejLimit
    //	)
    if ((llaHeadingDiffAbsRaw[i - 1] > 20) && ((pMeas->meas[indx[i - 1]].quality & DR_GOOD) == FALSE) && rejNum < rejLimit && fHeadingDiffStdPart < fHeadingDiffStd)
    {
      pMeas->meas[indx[i - 1]].status &= 0xFD;
      rejNum++;

      SYS_LOGGING(OBJ_KF, LOG_INFO, "Sort%s,%8.3f,%02d,%02d,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f", __FUNCTION__, pMeas->tor, pMeas->meas[indx[i - 1]].gnssMode,
        pMeas->meas[indx[i - 1]].prn, fHeadingDiffStdPart, fHeadingDiffStd, llaHeadingDiffAbsRaw[i - 1], THRES,
        pMeas->meas[indx[i - 1]].dr_err);

    }
  }


  pRaw += sprintf(pRaw, " %8.6f  %8.6f  %8.6f  %8.6f ", fHeadingDiffMean, fHeadingDiffStd, fHeadingDiffMeanPart, fHeadingDiffStdPart);
  p += sprintf(p, " %8.6f ", median);
  for (i = n - 1; i >= 0; i--)
  {
    pRaw += sprintf(pRaw, " %8.6f ", llaHeadingDiffAbsRaw[i]);
    pPrn += sprintf(pPrn, " %02d ", uprn[i]);
    p += sprintf(p, " %8.6f ", llaHeadingDiffAbs[i]);
  }
  SYS_LOGGING(OBJ_KF, LOG_INFO, "%s", strRaw);
  SYS_LOGGING(OBJ_KF, LOG_INFO, "%s", strprn);
  SYS_LOGGING(OBJ_KF, LOG_INFO, "%s", str);
  GLOGI("VelFlpHeadingMax: %8.3f  %d %d %d %8.3f %8.3f %8.3f %8.3f %8.6f  %8.6f  %8.6f  %8.6f %8.6f %d", pMeas->tor, pMeas->meas[indx[n - 1]].gnssMode,
    pMeas->meas[indx[n - 1]].prn, pMeas->meas[indx[n - 1]].quality & DR_GOOD,
    (llaHeadingDiffAbs[n - 1] / (median / 0.6745)), llaHeadingDiffAbs[n - 1], median,
    llaHeadingDiffAbsRaw[n - 1],
    fHeadingDiffMean, fHeadingDiffStd, fHeadingDiffMeanPart, fHeadingDiffStdPart, fabs(pMeas->meas[indx[n - 1]].dr_err),
    pMeas->meas[indx[n - 1]].cno);
}
#endif // 0
/***********************************************************************
* ��������: gnss_Kf_Alt_Check
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�1/13/
***********************************************************************/
void gnss_Kf_Alt_Check(float* deltaAlt, uint8_t* index, uint8_t prNum, Kf_t* p_Kf)
{
  uint8_t               i;
  float              avg = 0.0;
#if defined(PLAYBACK_MODE)
  float              ratio1 = 0.0, std = 0.0;
#endif
  float              ratio2 = 0.0;
  float              maxVal, secMaxVal;

  meas_blk_t* pMeas;

  pMeas = p_Kf->meas_blk;

  gnss_Pe_BiasNum(pMeas);

  if (prNum < (pMeas->biasNum + 4))
  {
    return;
  }

  gnss_Sort_WithIndx(deltaAlt, index, prNum);
  for (i = 0; i < prNum - 2; i++)
  {
    avg += deltaAlt[i];
  }
  avg /= (prNum - 2);

#if defined(PLAYBACK_MODE)
  for (i = 0; i < prNum - 2; i++)
  {
    std += (deltaAlt[i] - avg) * (deltaAlt[i] - avg);
  }
  std /= (prNum - 2);
  std = (float)sqrt(std);
#endif

  maxVal = deltaAlt[prNum - 1];
  secMaxVal = deltaAlt[prNum - 2];

  if (maxVal > avg && secMaxVal > avg)
  {
    ratio2 = (maxVal - avg) / (secMaxVal - avg);
  }
  else
  {
    ratio2 = maxVal / secMaxVal;
  }
#if defined(PLAYBACK_MODE)
  ratio1 = (maxVal - avg) / std;
  SYS_LOGGING(OBJ_KF, LOG_INFO, "%s,%10.4f,%10.4f,%10.4f,%10.4f", __FUNCTION__, maxVal, secMaxVal, ratio1, ratio2);
#endif
  /*
  case1: suppose maximum deltaAlt is more larger than other values
  */
  if (prNum >= 7)
  {
    if (ratio2 >= 2.5 && maxVal >= 1.5 && (pMeas->meas[index[prNum - 1]].quality & PR_GOOD) == FALSE &&
      fabs(pMeas->meas[index[prNum - 1]].pr_diff) > 5.0)
    {
      pMeas->meas[index[prNum - 1]].status &= 0xFE;
#if defined(PLAYBACK_MODE)
      SYS_LOGGING(OBJ_KF, LOG_INFO, "%s reject PR:%02d,%03d,%02d,%10.4f", __FUNCTION__, pMeas->meas[index[prNum - 1]].gnssMode,
        pMeas->meas[index[prNum - 1]].prn, pMeas->meas[index[prNum - 1]].cno, pMeas->meas[index[prNum - 1]].pr_diff);
#endif
      return;
    }
  }
}


/***********************************************************************
* ��������: gnss_Kf_Get_BiasNum
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�1/13/
***********************************************************************/
uint8_t gnss_Kf_Get_BiasNum(uint8_t svModeValid, uint8_t biasLinkFlag)
{
  uint8_t        i;
  uint8_t        biasNum = 0;
  uint16_t       bitMask;
  uint8_t        biasUnLink;
  uint8_t        svModeValidTmp;

  svModeValidTmp = svModeValid;

  /* svMode re-arc */
  if (biasLinkFlag == 1)
  {
    if ((svModeValid & 0x3) == 0x2)
    {
      svModeValid |= 0x1;
    }
  }
  else if (biasLinkFlag == 2)
  {
    svModeValid |= ((svModeValidTmp >> 1) & 0x1);
    svModeValid |= ((svModeValidTmp >> 2) & 0x1);
  }
  else if (biasLinkFlag == 3)
  {
    svModeValid |= ((svModeValidTmp >> 1) & 0x1);
    svModeValid |= ((svModeValidTmp >> 2) & 0x1);
    svModeValid |= ((svModeValidTmp >> 3) & 0x1);
  }

  bitMask = clockBiasBitMask[biasLinkFlag];
  biasUnLink = svModeValid & bitMask;
  biasNum = 0;

  for (i = 0; i < BIAS_NUM; i++)
  {
    if (biasUnLink & 1)
    {
      biasNum++;
    }

    biasUnLink = biasUnLink >> 1;
  }

  return biasNum;
}

/***********************************************************************
* ��������: gnss_Kf_Check_BiasErr
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�03/02/
***********************************************************************/
void gnss_Kf_Check_BiasErr(Kf_t* p_Kf)
{
  // This function is used to check if need bias link for different satellite systems
  uint8_t                  i, flag = FALSE;
  double      biasError;
  double                 biasDiffUse[GNSS_MAX_MODE] = { 0.0 };
  //GNSS_TIME* pTime;
  meas_blk_t* pMeas;

  p_Kf->biasErrFlag[GPS_MODE] = FALSE;
  p_Kf->biasErrFlag[GLN_MODE] = FALSE;
  p_Kf->biasErrFlag[BDS_MODE] = FALSE;
  p_Kf->biasErrFlag[GAL_MODE] = FALSE;

  if ((kfInfo.status & KF_RUN) == 0)
  {
    return;
  }

  if (gnss_tm_check_bias_status(GPS_MODE) == FALSE)
  {
    return;
  }

  if (p_Kf->kf_ctrl.biasLinkFlag != 0)
  {
    return;
  }

  //pTime = gnss_tm_get_time();
  pMeas = p_Kf->meas_blk;

  //load  bias diff for use
  for (i = 0; i < GNSS_MAX_MODE; i++)
  {
    if (pMeas->rtdUsed && p_Kf->biasDiffUpCnt[i] >= BIASDIFF_UPCNT_THRES)
    {
      biasDiffUse[i] = p_Kf->biasDiff[i];
    }
    else if (!pMeas->rtdUsed && p_Kf->biasDiffLocalUpCnt[i] >= BIASDIFF_UPCNT_THRES)
    {
      biasDiffUse[i] = p_Kf->biasDiffLocal[i];
    }
    else
    {
      biasDiffUse[i] = 0.0;
    }
  }

  //check GLN bias with valid bias diff
  if (fabs(biasDiffUse[GLN_MODE]) > 1e-3)
  {
    biasError = kfInfo.X[6] - kfInfo.X[7] - biasDiffUse[GLN_MODE];
    if (fabs(biasError) > BIAS_ERROR_THRES && gnss_tm_check_bias_status(GLN_MODE) == TRUE)
    {
      p_Kf->biasErrFlag[GLN_MODE] = TRUE;
      SYS_LOGGING(OBJ_PE, LOG_INFO, "%s,GPS-GLN-DIFF:%10.4f", __FUNCTION__, biasError);
    }
  }

  //check BDS bias valid bias diff
  if (fabs(biasDiffUse[BDS_MODE]) > 1e-3)
  {
    biasError = kfInfo.X[6] - kfInfo.X[8] - biasDiffUse[BDS_MODE];
    if (fabs(biasError) > BIAS_ERROR_THRES && gnss_tm_check_bias_status(BDS_MODE) == TRUE)
    {
      p_Kf->biasErrFlag[BDS_MODE] = TRUE;
      SYS_LOGGING(OBJ_PE, LOG_INFO, "%s,GPS-BDS-DIFF:%10.4f", __FUNCTION__, biasError);
    }
  }

  //check GAL bias valid bias diff
  if (fabs(biasDiffUse[GAL_MODE]) > 1e-3)
  {
    biasError = kfInfo.X[6] - kfInfo.X[9] - biasDiffUse[GAL_MODE];
    if (fabs(biasError) > BIAS_ERROR_THRES && gnss_tm_check_bias_status(GAL_MODE) == TRUE)
    {
      p_Kf->biasErrFlag[GAL_MODE] = TRUE;
      SYS_LOGGING(OBJ_PE, LOG_INFO, "%s,GPS-GAL-DIFF:%10.4f", __FUNCTION__, biasError);
    }
  }
}

/***********************************************************************
* ��������: gnss_Kf_UpVel_Hold
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�04/10/
***********************************************************************/
void gnss_Kf_Alt_AlphaBeta(Kf_t* p_Kf)
{
  double               dt, residual;
  FiltRefAlt* pRefAlt;
  KF_PVT_INFO* kfPvtInfo = p_Kf->kf_Pvt_Info;

  pRefAlt = &(p_Kf->filt_Alt_Data);

  if (kfPvtInfo->kfFixStatus == FIX_STATUS_NEW && kfPvtInfo->kfFixMode == FULL_FIX)
  {
    if (pRefAlt->flag == FALSE)
    {
      pRefAlt->altitude = (float)kfPvtInfo->kfLLApos[2];
      pRefAlt->velUp = kfPvtInfo->kfENUvel[2];
      pRefAlt->alpha = (float)0.05;
      pRefAlt->beta = 0.5;
      pRefAlt->flag = TRUE;
    }
    else
    {
      dt = kfPvtInfo->tor - pRefAlt->tor;
      /* If time gap is larger than 10s, reset the filter*/
      if (dt > 10)
      {
        pRefAlt->flag = FALSE;
      }
      else
      {
        pRefAlt->altitude += (float)dt * pRefAlt->velUp;
        residual = kfPvtInfo->kfLLApos[2] - pRefAlt->altitude;
        pRefAlt->altitude += (float)(pRefAlt->alpha * residual);
        pRefAlt->velUp = kfPvtInfo->kfENUvel[2];
      }
    }
    pRefAlt->tor = kfPvtInfo->tor;
    SYS_LOGGING(OBJ_KF, LOG_INFO, "%s,%14.4f,%14.4f,%14.4f,%14.4f", __FUNCTION__, pRefAlt->tor, pRefAlt->altitude, kfPvtInfo->kfLLApos[2], pRefAlt->velUp);
  }
}

/***********************************************************************
* ��������: gnss_Kf_UpVel_Hold
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/27/
***********************************************************************/
void gnss_Kf_Bias_Hold(KF_INFO* kfInfo_temp, double* deltaX)
{
  uint8_t             i;
  double            innovation, variance, r;
  double            H[N_STATE] = { 0.0 };

  for (i = 0; i < 4; i++)
  {
    if ((g_pe_cfg.gnss_usage_flag & (1 << i)) == 0) continue;
    r = deltaX[6 + i];
    if (fabs(r) > 5)
    {
      variance = 100;
    }
    else
    {
      continue;
    }
    innovation = -r;
    H[6 + i] = 1;
    udKFUpdate(H, deltaX, variance, innovation, SBOUND_PR, COAST_UPDATE, 1, 0);
  }
}

/***********************************************************************
* ��������: gnss_Kf_UpVel_Hold
* ���������
* ���������
* ����ֵ��
* ���ߣ�
***********************************************************************/
void gnss_Kf_UpVel_Hold(KF_INFO* kfInfo_temp, double* deltaX, KF_PVT_INFO* kfPvtInfo, double varIn)
{
  uint8_t             i;
  double            innovation, variance;
  double            H[N_STATE] = { 0.0 };
  double            cosLat, cosLon, sinLat, sinLon;
  float            X[3], enu[3];

  cosLat = cos(kfPvtInfo->kfLLApos[0]);
  sinLat = sin(kfPvtInfo->kfLLApos[0]);
  cosLon = cos(kfPvtInfo->kfLLApos[1]);
  sinLon = sin(kfPvtInfo->kfLLApos[1]);

  H[3] = cosLat * cosLon;    //U
  H[4] = cosLat * sinLon;
  H[5] = sinLat;
  innovation = 0.0;
  for (i = 3; i < 6; i++)
  {
    innovation += H[i] * (kfInfo_temp->X[i] + deltaX[i]);
  }
  innovation = -innovation;
  variance = varIn;

  udKFUpdate(H, deltaX, variance, innovation, SBOUND_DR, COAST_UPDATE, 1, 0);

  // check
  for (i = 0; i < 3; i++)
  {
    X[i] = (float)(kfInfo_temp->X[3 + i] + deltaX[3 + i]);
  }
  gnssConvEcef2EnuVel(X, enu, kfPvtInfo->kfLLApos);
}

/***********************************************************************
* ��������: gnss_Kf_PDR_UpVel_Hold
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�11/16/
***********************************************************************/
void gnss_Kf_PDR_UpVel_Hold(KF_INFO* kfInfo_temp, double* deltaX, KF_PVT_INFO* kfPvtInfo, double varIn)
{
  uint8_t            i;
  uint8_t            flag = FALSE;
  float           ecefVel[3], enuVel[3], horizonVel;

  if ((kfInfo_temp->status & KF_RUN) == 0 || fabs(kfPvtInfo->kfLLApos[2]) < 1e-4)
  {
    return;
  }

  if (peMode.staticData.staticFlag)
  {
    return;
  }

  for (i = 0; i < 3; i++)
  {
    ecefVel[i] = (float)(kfInfo_temp->X[3 + i] + deltaX[3 + i]);
  }

  gnssConvEcef2EnuVel(ecefVel, enuVel, kfPvtInfo->kfLLApos);

  horizonVel = sqrtf(enuVel[0] * enuVel[0] + enuVel[1] * enuVel[1]);
  if (horizonVel < fabs(enuVel[2]) && fabs(enuVel[2]) > 0.5)
  {
    flag = TRUE;
  }

  if (flag)
  {
    gnss_Kf_UpVel_Hold(kfInfo_temp, deltaX, kfPvtInfo, varIn);
  }
}

/***********************************************************************
* ��������: gnss_Kf_Post_Res
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�03/20/
***********************************************************************/
void gnss_Kf_Post_Res(Kf_t* p_Kf, KF_INFO* pkfInfo)
{
  uint8_t               i, j, prCnt = 0, prCntUse = 0, drCnt = 0, drCntUse = 0, flag;
  uint8_t               indx[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0 };
  float              res_avg = 0.0, res_std = 0.0;
  double              toa;
  double              pos_res = 0.0, vel_res = 0.0, res, var_median;
  double              prNoiseVar[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, drNoiseVar[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 };
  float              pos_res_sat[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, pos_res_use[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, vel_res_sat[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 }, vel_res_use[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM] = { 0.0 };
  double              r = 0.0;
  double              satPosCorr[3];
  meas_blk_t* pMeas;
  gnss_meas_t* pSvMeas;
  uint8_t				 fre1_satcnt[GNSS_MAX_MODE] = { 0 };
  uint8_t				 fre2_satcnt[GNSS_MAX_MODE * 2] = { 0 };

  pMeas = p_Kf->meas_blk;

  prCnt = 0;
  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &(pMeas->meas[i]);
    pSvMeas->post_prdiff = 0.0;
    if (pSvMeas->prn == 0) continue;
    if (pSvMeas->status & 0x1)
    {
      //r = 0.0;
      r = gnssClcSqrtAminusB_DBL(pkfInfo->X, pSvMeas->sv_info.p, 3);
      //earth rotation correction
      toa = r / LIGHT_SEC;
      gnssEarthRotateCorr(pSvMeas->sv_info.p, satPosCorr, toa);
      r = gnssClcSqrtAminusB_DBL(pkfInfo->X, satPosCorr, 3);

      r += pkfInfo->X[6 + pSvMeas->gnssMode];
      // position post residual
      res = r - pSvMeas->pseudoRange;

      if (pSvMeas->freq_index == 0)
      {
        fre1_satcnt[pSvMeas->gnssMode]++;
      }

      if (pSvMeas->freq_index == 1)
      {
        res -= pkfInfo->X[11 + pSvMeas->gnssMode];
        fre2_satcnt[pSvMeas->gnssMode]++;
      }
      else if (pSvMeas->freq_index == 2)
      {
        res -= pkfInfo->X[11 + GNSS_MAX_MODE + pSvMeas->gnssMode];
        fre2_satcnt[GNSS_MAX_MODE + pSvMeas->gnssMode]++;
      }

      pos_res_sat[prCnt] = (float)res;
      pos_res_use[prCnt] = (float)res;
      prNoiseVar[prCnt] = pSvMeas->prNoise;
      pos_res += res * res;
      pSvMeas->post_prdiff = (float)(-res);//save post kf res
      prCnt++;
      SYS_LOGGING(OBJ_KF, LOG_DEBUG, "KF POST PR RES: %10.4f,%2d,%3d,%10.4f,%10.4f", pMeas->tor, pSvMeas->gnssMode, pSvMeas->prn, -res, pSvMeas->pr_diff);
    }
  }


  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &(pMeas->meas[i]);
    pSvMeas->post_drdiff = 0.0;
    if (pSvMeas->prn == 0) continue;
    if (pSvMeas->status & 0x2)
    {
      r = 0.0;
      for (j = 0; j < 3; j++)
      {
        r += pSvMeas->sv_info.dcos[j] * (pkfInfo->X[j + 3] - (double)pSvMeas->sv_info.v[j]);
      }
      r += pkfInfo->X[10];
      // velocity post residual 
      res = r - pSvMeas->pseudoRangeRate;
      vel_res_sat[drCnt] = (float)res;
      vel_res_use[drCnt] = (float)res;
      drNoiseVar[prCnt] = pSvMeas->drNoise;
      vel_res += (double)res * res;
      indx[drCnt] = i;
      pSvMeas->post_drdiff = (float)res;
      drCnt++;
      //SYS_LOGGING(OBJ_KF,LOG_INFO,"KF dr res: %2d,%2d,%10.4f",pSvMeas->gnssMode,pSvMeas->prn,res);
    }
  }

  //remove too large res
  prCntUse = 0;
  if (prCnt > 1)
  {
    flag = gnss_median_dbl(prNoiseVar, prCnt, &var_median);
  }
  else
  {
    flag = FALSE;
  }
  if (flag)
  {
    pos_res = 0;
    for (i = 0; i < prCnt; i++)
    {
      if (prNoiseVar[i] <= 6 * var_median / 0.6745)
      {
        pos_res += (double)pos_res_sat[i] * pos_res_sat[i];
        pos_res_use[prCntUse] = pos_res_sat[i];
        prCntUse++;
      }
    }
    prCnt = prCntUse;
  }

  if (prCnt > 1)
  {
    pos_res = sqrt(pos_res / (prCnt - 1));
  }
  else
  {
    pos_res = sqrt(pos_res);
  }

  //remove too large res
  drCntUse = 0;
  if (drCnt > 1)
  {
    flag = gnss_median_dbl(drNoiseVar, drCnt, &var_median);
  }
  else
  {
    flag = FALSE;
  }
  if (flag)
  {
    vel_res = 0;
    for (i = 0; i < drCnt; i++)
    {
      if (drNoiseVar[i] <= 20 * var_median / 0.6745)
      {
        vel_res += (double)vel_res_sat[i] * vel_res_sat[i];
        vel_res_use[drCntUse] = vel_res_sat[i];
        drCntUse++;
      }
    }
    drCnt = drCntUse;
  }

  if (drCnt > 1)
  {
    vel_res = sqrt(vel_res / (drCnt - 1));
  }
  else
  {
    vel_res = sqrt(vel_res);
  }
  p_Kf->posRes = pos_res;
  p_Kf->velRes = vel_res;

  //calculate pos/vel res std
  if (prCnt >= 6)
  {
    gnss_math_fstd(pos_res_use, prCnt, &res_avg, &res_std);
  }
  else
  {
    res_std = -1.0;
  }
  p_Kf->posResStd = res_std;

  if (drCnt >= 6)
  {
    gnss_math_fstd(vel_res_sat, drCnt, &res_avg, &res_std);
  }
  else
  {
    res_std = -1.0;
  }
  p_Kf->velResStd = res_std;

  for (i = 0; i < GNSS_MAX_MODE; i++)
  {
    p_Kf->fre1_satcnt[i] = fre1_satcnt[i];
  }

  for (i = 0; i < GNSS_MAX_MODE * 2; i++)
  {
    p_Kf->fre2_satcnt[i] = fre2_satcnt[i];
  }

  SYS_LOGGING(OBJ_KF, LOG_INFO, "%s,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f", __FUNCTION__, pMeas->tor, pos_res, p_Kf->posResStd, vel_res, p_Kf->velResStd);
}

/***********************************************************************
* ��������: gnss_Kf_Set_ChiSqStatus
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�05/12/
***********************************************************************/
void gnss_Kf_Set_ChiSqStatus(Kf_t* p_Kf)
{
  p_Kf->kf_ctrl.isPrChiSqTest = TRUE;
  p_Kf->kf_ctrl.isDrChiSqTest = TRUE;
}
/***********************************************************************
* ��������: gnss_Kf_Save_Filter
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�04/27/
***********************************************************************/
void gnss_Kf_Save_Filter(Kf_t* p_Kf, KF_INFO* pkfInfo)
{
  if (p_Kf == NULL || pkfInfo == NULL)
  {
    return;
  }


  if (peMode.staticData.historyStatic == 1)
  {
    memcpy(p_Kf->kfInfoBack.D_minus, pkfInfo->D_minus, (N_STATE + 1) * sizeof(double));
    memcpy(p_Kf->kfInfoBack.D_plus, pkfInfo->D_plus, (N_STATE + 1) * sizeof(double));
    memcpy(p_Kf->kfInfoBack.U_minus, pkfInfo->U_minus, N_MAT * sizeof(double));
    memcpy(p_Kf->kfInfoBack.U_plus, pkfInfo->U_plus, N_MAT * sizeof(double));
    memcpy(p_Kf->kfInfoBack.W1, pkfInfo->W1, N_MAT * sizeof(double));
    memcpy(p_Kf->kfInfoBack.W2, pkfInfo->W2, N_MAT * sizeof(double));
  }
}

/***********************************************************************
* ��������: gnss_Kf_Restore_Filter
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�04/27/
***********************************************************************/
void gnss_Kf_Restore_Filter(Kf_t* p_Kf, KF_INFO* kfInfo_temp)
{
  if (p_Kf == NULL || kfInfo_temp == NULL)
  {
    return;
  }

  memcpy(kfInfo_temp->D_plus, p_Kf->kfInfoBack.D_plus, (N_STATE + 1) * sizeof(double));
  memcpy(kfInfo_temp->U_plus, p_Kf->kfInfoBack.U_plus, N_MAT * sizeof(double));
  memcpy(kfInfo_temp->D_minus, p_Kf->kfInfoBack.D_minus, (N_STATE + 1) * sizeof(double));
  memcpy(kfInfo_temp->U_minus, p_Kf->kfInfoBack.U_minus, N_MAT * sizeof(double));
  memcpy(kfInfo_temp->W1, p_Kf->kfInfoBack.W1, N_MAT * sizeof(double));
  memcpy(kfInfo_temp->W2, p_Kf->kfInfoBack.W2, N_MAT * sizeof(double));
}

/***********************************************************************
* ��������: gnss_Kf_RefHeading_Cal
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�05/09/
***********************************************************************/
void gnss_Kf_RefHeading_Cal(Kf_t* p_Kf)
{

  uint32_t            i, j, k, cnt = 0, grpNum[3], step, idx0, idx1;
  float            heading[BACK_LLA_NUM / 2] = { 0.0 };
  float            avg = 0.0, std = 0.0/*, distance*/;
  KF_LLA_BACK* p1, * p2;

  grpNum[0] = 2;
  grpNum[1] = 3;
  grpNum[2] = 4;

  for (j = 0; j < 3; j++)
  {
    step = BACK_LLA_NUM / grpNum[j];
    for (k = 0; k < grpNum[j] - 1; k++)
    {
      avg = 0.0;
      std = 0.0;
      cnt = 0;
      memset(heading, 0, sizeof(float) * BACK_LLA_NUM / 2);
      for (i = 0; i < step; i++)
      {
        idx0 = i + k * step;
        idx1 = idx0 + step;
        p1 = &(p_Kf->kfLLABack[idx0]);
        p2 = &(p_Kf->kfLLABack[idx1]);
        if (p1->t > 0.0 && p2->t > 0.0)
        {
          heading[i] = gnss_lla2_heading(p2->LLApos, p1->LLApos);
          //distance = gnssCalPosDis(p1->LLApos, p2->LLApos, 1);
          if (heading[i] > PI) heading[i] -= (float)(2 * PI);
          else if (heading[i] <= -PI) heading[i] += (float)(2 * PI);
          avg += heading[i];
          cnt++;
        }
      }
      if (cnt == 0) continue;
      avg /= cnt;
      for (i = 0; i < cnt; i++)
      {
        std += (heading[i] - avg) * (heading[i] - avg);
      }
      std = (float)sqrt(std / cnt);
    }
  }
  GLOGD("ref head std:%f", std);
}

/***********************************************************************
* ��������: gnss_Kf_Rslt_Smooth
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�11/09/
***********************************************************************/
void gnss_Kf_Rslt_Smooth(Kf_t* p_Kf)
{
  uint8_t                i, a;
  static double        ecefPos[3], ecefVel[3], t0, factor = 4.0;
  double               dt = 0.0;
  KF_PVT_INFO* kfRslt;

  kfRslt = p_Kf->kf_Pvt_Info;

  if (kfRslt->kf_had_firstFix == FALSE)
  {
    factor = 4.0;
    memset(ecefPos, 0, 3 * sizeof(double));
    return;
  }

  a = p_Kf->kf_ctrl.prDewNum + p_Kf->kf_ctrl.prRejNum;
  if (p_Kf->posResStd < 5.0 && p_Kf->posResStd > 0.0 && (p_Kf->kf_ctrl.prNum - a) >= 12)
  {
    factor = 2.0;
  }
  else if (peState.posHasBias)
  {
    factor = 2.0;
  }
  else
  {
    factor = 4.0;
  }

  if (fabs(ecefPos[0]) < 1e-3)
  {
    for (i = 0; i < 3; i++)
    {
      ecefPos[i] = kfRslt->ecefPos[i];
      ecefVel[i] = kfRslt->ecefVel[i];
    }
    t0 = kfRslt->tor;
  }
  else
  {
    dt = kfRslt->tor - t0;
    t0 = kfRslt->tor;

    if (dt < -SECS_IN_WEEK / 2.0)
    {
      dt += (double)SECS_IN_WEEK;
    }
    else if (dt > SECS_IN_WEEK / 2.0)
    {
      dt -= (double)SECS_IN_WEEK;
    }

    if (fabs(dt) > 1.0)
    {
      memset(ecefPos, 0, 3 * sizeof(double));
    }
    else
    {
      for (i = 0; i < 3; i++)
      {
        ecefVel[i] = (ecefVel[i] + kfRslt->ecefVel[i]) / 2;
      }

      /* smooth position */
      for (i = 0; i < 3; i++)
      {
        ecefPos[i] = (factor - 1) * (ecefPos[i] + ecefVel[i] * dt) / factor + kfRslt->ecefPos[i] / factor;
      }

      /* Fill the position output */
      for (i = 0; i < 3; i++)
      {
        kfRslt->ecefPos[i] = ecefPos[i];
      }
      gnssConvEcef2Lla(kfRslt->ecefPos, kfRslt->kfLLApos);
    }
  }
}
/***********************************************************************
* ��������: gnss_Kf_PosHeading_Hold
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�05/12/
***********************************************************************/
void gnss_Kf_PosHeading_Hold(Kf_t* p_Kf, double* deltaX)
{
  uint8_t               i;
  double              cosLat, cosLon, sinLat, sinLon, sinHeading, cosHeading;
  double              H[N_STATE] = { 0.0 };
  KF_PVT_INFO* kfPvtInfo;
  double              innovation = 0, variance;
  double              test = 0.0;
  double              theta = 5 * DEG2RAD;
  float              enuDelta[3], ecefDelta[3], range, enuDelta1[3];

  kfPvtInfo = p_Kf->kf_Pvt_Info;
  /***********
  position heading hold is aimed at making sure the position update along the real
  moving direction, avoid along the cross direction.
  [deltaE,deltaN,deltaU] = Ceg * [deltaX,deltaY,deltaZ] where deltaX,deltaY,deltaZ
  are the update value of position. So this vector indicated the vector from last
  position to current assumed position.
  convert vector in ECEF coordinate to ENU coordinate.
  deltaE * cos(refHeading) - deltaN * sin(refHeading) = 0
  deltaE * cos(refHeading+n) - deltaN * sin(refHeading+n) = 0 where n is the uncertainty
  of reference heading.
  ***********/
  cosLat = cos(kfPvtInfo->kfLLApos[0]);
  sinLat = sin(kfPvtInfo->kfLLApos[0]);
  cosLon = cos(kfPvtInfo->kfLLApos[1]);
  sinLon = sin(kfPvtInfo->kfLLApos[1]);

  sinHeading = sin(kfPvtInfo->headingRef * DEG2RAD);
  cosHeading = cos(kfPvtInfo->headingRef * DEG2RAD);

  ecefDelta[0] = (float)deltaX[0];
  ecefDelta[1] = (float)deltaX[1];
  ecefDelta[2] = (float)deltaX[2];
  gnssConvEcef2EnuVel(ecefDelta, enuDelta, kfPvtInfo->kfLLApos);

  range = sqrtf(enuDelta[0] * enuDelta[0] + enuDelta[1] * enuDelta[1]);
  enuDelta[1] = (float)(range * cos(kfPvtInfo->headingRef * DEG2RAD));
  enuDelta[0] = (float)(range * sin(kfPvtInfo->headingRef * DEG2RAD));

  H[0] = sinLat * cosLon * sinHeading - sinLon * cosHeading;
  H[1] = sinLat * sinLon * sinHeading + cosHeading * cosLon;
  H[2] = -sinHeading * cosLat;

  innovation = 0.0;
  for (i = 0; i < 3; i++)
  {
    innovation += H[i] * deltaX[i];
  }
  innovation = -innovation;
  variance = (range * 1.0) * (range * 1.0);
  if (variance < 2.0)
  {
    variance = 2.0;
  }
  //test = udKFUpdate(H, deltaX, variance, innovation, SBOUND_PR, COAST_UPDATE, 1, 0);
  udKFUpdate(H, deltaX, variance, innovation, SBOUND_PR, COAST_UPDATE, 1, 0);
#if 0
  // E update
  H[0] = -sinLon;
  H[1] = cosLon;
  H[2] = 0;

  innovation = 0.0;
  for (i = 0; i < 3; i++)
  {
    innovation += H[i] * deltaX[i];
  }
  innovation = enuDelta[0] - innovation;
  variance = 1.0;
  test = udKFUpdate(H, deltaX, variance, innovation, SBOUND_PR, COAST_UPDATE, 1, 0);

  // N update
  H[0] = -sinLat * cosLon;
  H[1] = -sinLat * sinLon;
  H[2] = cosLat;
  innovation = 0.0;
  for (i = 0; i < 3; i++)
  {
    innovation += H[i] * deltaX[i];
  }
  innovation = enuDelta[1] - innovation;
  variance = 1.0;
  test = udKFUpdate(H, deltaX, variance, innovation, SBOUND_PR, COAST_UPDATE, 1, 0);
#endif

  ecefDelta[0] = (float)deltaX[0];
  ecefDelta[1] = (float)deltaX[1];
  ecefDelta[2] = (float)deltaX[2];
  gnssConvEcef2EnuVel(ecefDelta, enuDelta1, kfPvtInfo->kfLLApos);
}
