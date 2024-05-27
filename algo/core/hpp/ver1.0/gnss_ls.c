/************************************************************
* Copyrights(C)
* All rights Reserved
* �ļ����ƣ�gnss_ls.c
* �ļ���������С���˶�λ����
* �汾�ţ�1.0.0
* ���ߣ�
* ���ڣ�08/08
************************************************************/
#include "gnss.h"
#include "gnss_ls.h"
#include "gnss_math.h"
#include "gnss_sys_api.h"
#include "gnss_kf_math.h"
#include "gnss_mode.h"
#include "gnss_pe.h"
#include "cmn_utils.h"
//#include "gnss_core_api.h"

#undef MODULE_TAG
#define MODULE_TAG OBJ_LS

extern KF_INFO kfInfo;
extern PE_MODES peMode;
extern USER_PVT    user_pvt_backup;
extern Gnss_Cfg_t g_pe_cfg;
extern history_Info    HisInfo;
extern uint8_t firstFix;
extern PeStateMonitor_t peState;
extern Kf_t* gpz_kf_data;  //import from PE module
/***********************************************************************
* ��������: gnss_Ls_Init
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�08/08/
***********************************************************************/
void gnss_Ls_Init(Ls_t* pLs)
{
  memset(pLs, 0, sizeof(Ls_t));
}

/***********************************************************************
* ��������: gnss_Ls_Check_BiasNum
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�08/08/
***********************************************************************/
void gnss_Ls_Check_BiasNum(Ls_t* pLs)
{
  uint32_t           i, indx;
  gnss_meas_t* pSvMeas;
  meas_blk_t* pMeas;

  pMeas = pLs->meas;
  pLs->lsCtrl.satModeCnt = 0;

  for (i = 0; i < GNSS_MAX_MODE * 2; i++)
  {
    pMeas->fre2_satcnt[i] = 0;
    pLs->lsCtrl.dcbIndx[i] = 0;
    pLs->lsCtrl.lsDcbCalFlag[i] = FALSE;
  }

  /* Check how many system mode for PR */
  for (i = 0; i < pLs->lsCtrl.prNum; i++)
  {
    indx = pLs->lsCtrl.prUsedSV[i];
    pSvMeas = &(pMeas->meas[indx]);
    if (pSvMeas->prn == 0) continue;

    switch (pSvMeas->gnssMode)
    {
    case GPS_MODE:
      pLs->lsCtrl.satModeCnt |= 0x1;
      if (pSvMeas->freq_index == 0)
      {
        pLs->lsCtrl.lsDcbCalFlag[GPS_MODE] = TRUE;
        pLs->lsCtrl.lsDcbCalFlag[GPS_MODE + GNSS_MAX_MODE] = TRUE;
      }
      else if (pSvMeas->freq_index == 1)
      {
        pLs->lsCtrl.satModeCnt |= 0x10;
        pMeas->fre2_satcnt[GPS_MODE]++;
      }
      else if (pSvMeas->freq_index == 2)
      {
        pLs->lsCtrl.satModeCnt |= 0x100;
        pMeas->fre2_satcnt[GPS_MODE + GNSS_MAX_MODE]++;
      }
      break;
    case GLN_MODE:
      if (pLs->lsCtrl.isBiasLink)
      {
        pLs->lsCtrl.satModeCnt |= 0x1;
      }
      else
      {
        pLs->lsCtrl.satModeCnt |= 0x2;
        if (pSvMeas->freq_index == 0)
        {
          pLs->lsCtrl.lsDcbCalFlag[GLN_MODE] = TRUE;
          pLs->lsCtrl.lsDcbCalFlag[GLN_MODE + GNSS_MAX_MODE] = TRUE;
        }
        else if (pSvMeas->freq_index == 1)
        {
          pLs->lsCtrl.satModeCnt |= 0x20;
          pMeas->fre2_satcnt[GLN_MODE]++;
        }
        else if (pSvMeas->freq_index == 2)
        {
          pLs->lsCtrl.satModeCnt |= 0x200;
          pMeas->fre2_satcnt[GLN_MODE + GNSS_MAX_MODE]++;
        }
      }
      break;
    case BDS_MODE:
      if (pLs->lsCtrl.isBiasLink)
      {
        pLs->lsCtrl.satModeCnt |= 0x1;
      }
      else
      {
        pLs->lsCtrl.satModeCnt |= 0x4;
        if (pSvMeas->freq_index == 0)
        {
          pLs->lsCtrl.lsDcbCalFlag[BDS_MODE] = TRUE;
          pLs->lsCtrl.lsDcbCalFlag[BDS_MODE + GNSS_MAX_MODE] = TRUE;
        }
        else if (pSvMeas->freq_index == 1)
        {
          pLs->lsCtrl.satModeCnt |= 0x40;
          pMeas->fre2_satcnt[BDS_MODE]++;
        }
        else if (pSvMeas->freq_index == 2)
        {
          pLs->lsCtrl.satModeCnt |= 0x400;
          pMeas->fre2_satcnt[BDS_MODE + GNSS_MAX_MODE]++;
        }
      }
      break;
    case GAL_MODE:
      if (pLs->lsCtrl.isBiasLink)
      {
        pLs->lsCtrl.satModeCnt |= 0x1;
      }
      else
      {
        pLs->lsCtrl.satModeCnt |= 0x8;
        if (pSvMeas->freq_index == 0)
        {
          pLs->lsCtrl.lsDcbCalFlag[GAL_MODE] = TRUE;
          pLs->lsCtrl.lsDcbCalFlag[GAL_MODE + GNSS_MAX_MODE] = TRUE;
        }
        else if (pSvMeas->freq_index == 1)
        {
          pLs->lsCtrl.satModeCnt |= 0x80;
          pMeas->fre2_satcnt[GAL_MODE]++;
        }
        else if (pSvMeas->freq_index == 2)
        {
          pLs->lsCtrl.satModeCnt |= 0x800;
          pMeas->fre2_satcnt[GAL_MODE + GNSS_MAX_MODE]++;
        }
      }
      break;
    default: {break; }
    }
  }

  /* calculate the bias number for LS
  1. current, for each system, there is a bias
  2. In future, if needed, we consider bias link feature
  */
  pLs->lsCtrl.lsBiasNum = 0;
  pLs->lsCtrl.lsDcbNum[0] = 0;
  pLs->lsCtrl.lsDcbNum[1] = 0;
  for (i = 0; i < 4; i++)
  {
    pLs->lsCtrl.lsBiasNum += (pLs->lsCtrl.satModeCnt >> i) & 0x1;
    if (!pLs->lsCtrl.isBiasLink)
    {
      pLs->lsCtrl.biasIndx[i] = (pLs->lsCtrl.lsBiasNum) * ((pLs->lsCtrl.satModeCnt >> i) & 0x1);
      if ((pMeas->fre2_satcnt[i] > DCB_SAT_LIMIT) && pLs->lsCtrl.lsDcbCalFlag[i])
      {
        pLs->lsCtrl.lsDcbNum[0] += (pLs->lsCtrl.satModeCnt >> (i + 4)) & 0x1;
        pLs->lsCtrl.dcbIndx[i] = (pLs->lsCtrl.lsDcbNum[0]) * ((pLs->lsCtrl.satModeCnt >> (i + 4)) & 0x1);
      }
      else
      {
        pLs->lsCtrl.lsDcbCalFlag[i] = FALSE;
      }

      if ((pMeas->fre2_satcnt[i + GNSS_MAX_MODE] > DCB_SAT_LIMIT) && pLs->lsCtrl.lsDcbCalFlag[i + GNSS_MAX_MODE])
      {
        pLs->lsCtrl.lsDcbNum[1] += (pLs->lsCtrl.satModeCnt >> (i + 8)) & 0x1;
        pLs->lsCtrl.dcbIndx[i + 4] = (pLs->lsCtrl.lsDcbNum[1]) * ((pLs->lsCtrl.satModeCnt >> (i + 8)) & 0x1);
      }
      else
      {
        pLs->lsCtrl.lsDcbCalFlag[i + GNSS_MAX_MODE] = FALSE;
      }
    }
    else
    {
      pLs->lsCtrl.biasIndx[i] = (pLs->lsCtrl.lsBiasNum) * ((pLs->lsCtrl.satModeCnt) & 0x1);
    }
  }

  pLs->lsCtrl.lsBiasNum = pLs->lsCtrl.lsBiasNum + pLs->lsCtrl.lsDcbNum[0] + pLs->lsCtrl.lsDcbNum[1];
}
/***********************************************************************
* ��������: gnss_LS_BiasLink_Check
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/03/
***********************************************************************/
void gnss_LS_BiasLink_Check(Ls_t* pLs)
{
  uint8_t          biasDiffUpdateCnt[GNSS_MAX_MODE];
  double         biasDiffUse[GNSS_MAX_MODE];
  meas_blk_t* pMeas;

  pMeas = pLs->meas;
  if (pMeas != NULL && pMeas->rtdUsed)
  {
    memcpy(biasDiffUse, pLs->biasDiff, GNSS_MAX_MODE * sizeof(double));
    memcpy(biasDiffUpdateCnt, pLs->biasDiffUpCnt, GNSS_MAX_MODE * sizeof(uint8_t));
  }
  else
  {
    memcpy(biasDiffUse, pLs->biasDiffLocal, GNSS_MAX_MODE * sizeof(double));
    memcpy(biasDiffUpdateCnt, pLs->biasDiffLocalUpCnt, GNSS_MAX_MODE * sizeof(uint8_t));
  }

  if (pLs->lsCtrl.validSvNum[GPS_MODE] > 0 &&
    (pLs->lsCtrl.validSvNum[GLN_MODE] > 0 || pLs->lsCtrl.validSvNum[BDS_MODE] > 0 || pLs->lsCtrl.validSvNum[GAL_MODE] > 0))
  {
    pLs->lsCtrl.isBiasLink = TRUE;
  }
  else
  {
    pLs->lsCtrl.isBiasLink = FALSE;
  }

  if ((pLs->lsCtrl.validSvNum[GLN_MODE] > 0 && (fabs(biasDiffUse[GLN_MODE]) == 0 || biasDiffUpdateCnt[GLN_MODE] < 20)) ||
    (pLs->lsCtrl.validSvNum[BDS_MODE] > 0 && (fabs(biasDiffUse[BDS_MODE]) == 0 || biasDiffUpdateCnt[BDS_MODE] < 20)) ||
    (pLs->lsCtrl.validSvNum[GAL_MODE] > 0 && (fabs(biasDiffUse[GAL_MODE]) == 0 || biasDiffUpdateCnt[GAL_MODE] < 20)))
  {
    pLs->lsCtrl.isBiasLink = FALSE;
  }
}
/***********************************************************************
* ��������: gnss_Ls_Prepare
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�08/08/
***********************************************************************/
void gnss_Ls_Prepare(meas_blk_t* pMeas, Ls_t* pLs)
{
  uint8_t            pr_cnt = 0, dr_cnt = 0;
  uint32_t           i, j;
  gnss_meas_t* pSvMeas;
  double           dt;
  //GNSS_TIME* pTime;
  USER_PVT      userPos;

  pLs->meas = pMeas;
 // pTime = gnss_tm_get_time();

  /* bias propagate if there is clock drift */
  pLs->lsCtrl.lastTor = pLs->lsCtrl.tor;
  dt = pMeas->tor - pLs->lsCtrl.lastTor;
  if (dt < -SECS_IN_WEEK / 2.0)
  {
    dt += (double)SECS_IN_WEEK;
  }
  else if (dt > SECS_IN_WEEK / 2.0)
  {
    dt -= (double)SECS_IN_WEEK;
  }
  if (fabs(pLs->drift) > 0.0001 && pLs->ls_Pvt_Info->valid)
  {
    for (i = 0; i < BIAS_NUM; i++)
    {
      pLs->bias[i] += (pLs->drift * dt);
      pLs->bias_unc[i] += pLs->drift_unc * fabs(dt);
    }
    SYS_LOGGING(OBJ_LS, LOG_INFO, "LS Bias:%14.4f %14.4f %14.4f %14.4f",
      pLs->bias[0], pLs->bias[1], pLs->bias[2], pLs->bias[3]);
    // Clock Bias/Clock uncertainty propagation 
    pLs->drift_unc += (float)(G_DIV * fabs(dt));
  }

  // set current tor
  pLs->lsCtrl.tor = pMeas->tor;

  // save last fix status and clear current status 
  pLs->lsCtrl.lastStatus = pLs->lsCtrl.status;
  pLs->lsCtrl.status = 0;

  /* Initial prUsedSV and drUsedSV */
  memset(pLs->lsCtrl.prUsedSV, 0, sizeof(uint8_t) * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM);
  memset(pLs->lsCtrl.drUsedSV, 0, sizeof(uint8_t) * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM);
  memset(pLs->lsCtrl.validSvNum, 0, sizeof(uint8_t) * GNSS_MAX_MODE);
  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &(pMeas->meas[i]);
    if (pSvMeas->prn == 0) continue;
    if (pSvMeas->status & 0x1)
    {
      pLs->lsCtrl.validSvNum[pSvMeas->gnssMode]++;
    }
    if (pSvMeas->status & 0x2)
    {
      pLs->lsCtrl.drUsedSV[dr_cnt++] = i;
    }
  }

  if (!firstFix && peMode.tunnelMode.exitTunnelFlag == FALSE)
  {
    gnss_LS_BiasLink_Check(pLs);
  }
  /*When existing tunnel, use bias link */
  else if ((peMode.tunnelMode.mode == FALSE && peMode.tunnelMode.exitTunnelFlag == TRUE) || peState.posHasBias)
  {
    gnss_LS_BiasLink_Check(pLs);
  }
  else
  {
    pLs->lsCtrl.isBiasLink = FALSE;
  }

  /* In order to estimate bias for one satellite system, at least 3 satellites are needed
  but this condition is valid only when bias is unknown */
  if (pLs->lsCtrl.isBiasLink == FALSE && firstFix == FALSE)
  {
    for (i = 0; i < GNSS_MAX_MODE; i++)
    {
      if (gnss_tm_check_bias_status(i))continue;
      if (pLs->lsCtrl.validSvNum[i] < 3 && pLs->lsCtrl.validSvNum[i] > 0)
      {
        for (j = 0; j < pMeas->measCnt; j++)
        {
          pSvMeas = &(pMeas->meas[j]);
          if (pSvMeas->prn == 0 || pSvMeas->gnssMode != i) continue;
          pSvMeas->status &= 0xFE;
        }
        pLs->lsCtrl.validSvNum[i] = 0;
      }
    }
  }
  else if (pLs->lsCtrl.isBiasLink == TRUE && firstFix == FALSE)
  {

  }

  for (i = 0; i < pMeas->measCnt; i++)
  {
    pSvMeas = &(pMeas->meas[i]);
    if (pSvMeas->prn == 0) continue;
    if (pSvMeas->status & 0x1)
    {
      pLs->lsCtrl.prUsedSV[pr_cnt++] = i;
    }
  }
  pLs->lsCtrl.prNum = pr_cnt;
  pLs->lsCtrl.drNum = dr_cnt;
  pLs->lsCtrl.is2DPos = FALSE;
  pLs->lsCtrl.isForce2D = FALSE;
  pLs->lsCtrl.is2DVel = FALSE;
  pLs->raimCtrl.drRaimFlag = 0;
  pLs->raimCtrl.prRaimFlag = 0;

  gnss_Pe_Get_PosFix(&userPos);
  /* propagate LS position only when there is 2D or 3D velocity
  1. If LS is converged last time, propagate LS position
  2. If LS is un-converged last time, use user position
  3. If can't find a proper position, clear LS ECEF position
  */
  if (pLs->lsCtrl.lastStatus & LS_STATUS_HAS_2D_VEL ||
    pLs->lsCtrl.lastStatus & LS_STATUS_HAS_3D_VEL)
  {
    for (i = 0; i < 3; i++)
    {
      pLs->lsEcef.p[i] += pLs->lsEcef.v[i] * dt;
    }
  }
  else if (userPos.have_position >= HAVE_POS_APPX && userPos.have_position < HAVE_POS_OLD)
  {
    for (i = 0; i < 3; i++)
    {
      pLs->lsEcef.p[i] = userPos.ecef.pos[i];
      pLs->lsEcef.v[i] = userPos.ecef.vel[i];
    }
  }
  else
  {
    for (i = 0; i < 3; i++)
    {
      pLs->lsEcef.p[i] = 0;
      pLs->lsEcef.v[i] = 0;
    }
  }

}
/***********************************************************************
* ��������: gnss_Ls_Qr
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�08/28/
***********************************************************************/
static uint8_t gnss_Ls_Qr(float A[][15], int32_t M, int32_t N, float* diff, float* solution)
{

  int32_t      i, j, k;
  float      s, r=0.0f;
  float      up, down;
  float      sum = 0;
  float* U;
  float      CoeMatrixA[MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM][LS_PARA_NUM] = { 0 };

  if ((U = (float*)Sys_Malloc(M * sizeof(float))) == NULL)
  {
    return FALSE;
  }

  memset(U, 0, M * sizeof(float));
  memset(CoeMatrixA, 0, sizeof(float) * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * LS_PARA_NUM);
  memcpy(CoeMatrixA, A, sizeof(float) * M * LS_PARA_NUM);

  if (M < N)
  {
    Sys_Free(U);
    return FALSE;
  }

  if ((!diff) || (!solution))
  {
    Sys_Free(U);
    return FALSE;
  }

  //N is the number of column

  for (i = 0; i < N; i++)
  {
    s = 0;

    for (k = i; k < M; k++)
    {
#if 0
      {
        uint32_t tmp = 0;
        memcpy(&tmp, &(CoeMatrixA[k][i]), sizeof(float));
        SYS_LOGGING(OBJ_LS, LOG_INFO, "[DEBUG1] CoeMatrixA[%d][%d](0x%x 0x%x 0x%x 0x%x)",
          k, i, tmp >> 24, (tmp >> 16) & 0xFF, (tmp >> 8) & 0xFF, tmp & 0xFF);
      }
#endif
      s += CoeMatrixA[k][i] * CoeMatrixA[k][i];
#if 0
      {
        uint32_t tmp = 0;
        memcpy(&tmp, &s, sizeof(float));
        SYS_LOGGING(OBJ_LS, LOG_INFO, "[DEBUG2] s(0x%x 0x%x 0x%x 0x%x)",
          tmp >> 24, (tmp >> 16) & 0xFF, (tmp >> 8) & 0xFF, tmp & 0xFF);
      }
#endif
    }

    //Make V(k)*S <=0
    if (CoeMatrixA[i][i] >= 0)
    {
      s = (float)-sqrt(s);
    }
    else
    {
      s = (float)sqrt(s);
    }

    // construct the U array
    for (j = 0; j < M; j++)
    {
      U[i] = CoeMatrixA[i][i] - s;

      if (j < i)
      {
        U[j] = 0;
      }
      else if (j > i)
      {
        U[j] = CoeMatrixA[j][i];
      }
    }

    //recompute the coefficient matrix
    // the entries above the kth row will not change by the multiplicaiton of Q
    for (k = i + 1; k < N; k++)
    {
      //compute r for QV = V - R*U; now we have V and U
      // r = 2*U'*V/(U'*U)
      up = 0;
      down = 0;

      for (j = i; j < M; j++)
      {
        up += U[j] * CoeMatrixA[j][k];
        down += U[j] * U[j];
      }

      if (down != 0.0)
      {
        r = 2 * up / down;
      }

      // compute the entries of the column
      for (j = i; j < M; j++)
      {
        CoeMatrixA[j][k] = CoeMatrixA[j][k] - r * U[j];
      }
    }

    //compute the i th column QV= V- U
    CoeMatrixA[i][i] = s;

    // if any one number in the diagonal is zero then this matrix is singular
    if (fabs(CoeMatrixA[i][i]) < 1e-6)
    {
      Sys_Free(U);
      return FALSE;
    }

    for (j = i + 1; j < M; j++)
    {
      CoeMatrixA[j][i] = 0;
    }

    //compute the Q*B
    up = 0;
    down = 0;

    for (j = i; j < M; j++)
    {
      up += U[j] * diff[j];
      down += U[j] * U[j];
    }

    if (down != 0.0)
    {
      r = 2 * up / down;
    }

    for (j = i; j < M; j++)
    {
      diff[j] -= r * U[j];
    }
  }

  /* get the least square solution */
  for (j = N - 1; j >= 0; j--)
  {
    sum = 0;

    for (k = j + 1; k < N; k++)
    {
      sum += CoeMatrixA[j][k] * solution[k];
    }

    solution[j] = (diff[j] - sum) / CoeMatrixA[j][j];
  }
  Sys_Free(U);
  return TRUE;
}

/***********************************************************************
* ��������: gnss_Ls_PosErrEstimate
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/20/
***********************************************************************/
static void gnss_Ls_PosErrEstimate(Ls_t* pLs)
{
  uint8_t              i;
  uint16_t             k;
  float             errorSum;
  meas_blk_t* pMeas;
  gnss_meas_t* pSvMeas;

  errorSum = 0;
  /*load sv info*/
  pMeas = pLs->meas;
  for (i = 0; i < pLs->lsCtrl.prNum; i++)
  {
    pSvMeas = &pMeas->meas[pLs->lsCtrl.prUsedSV[i]];
    k = (uint16_t)(26.23 * exp(-0.03857 * pSvMeas->cno) * 1000);
    k *= k;
    errorSum += k;
  }

  errorSum = (float)sqrt(errorSum / pLs->lsCtrl.prNum);

  if (errorSum > 40)
  {
    errorSum = 40;
  }
  pLs->err_est.pErr = pLs->DoP.pDOP * errorSum;
  pLs->err_est.hErr = pLs->DoP.hDOP * errorSum;
  pLs->err_est.vErr = pLs->DoP.vDOP * errorSum;
  pLs->err_est.tErr = pLs->DoP.tDOP * errorSum;
}


/***********************************************************************
* ��������: gnss_Ls_Weight_Cal
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�08/29/
***********************************************************************/
static void gnss_Ls_Weight_Cal(Ls_t* pLs, uint8_t lsType)
{
  uint8_t             indx;
  uint32_t            i, num;
  uint32_t            cno;
  float            weight;
  gnss_meas_t* pSvMeas;
  meas_blk_t* pMeas;

  pMeas = pLs->meas;

  if (lsType == LS_POS_ID)   // PR weight
  {
    num = pLs->lsCtrl.prNum;
    for (i = 0; i < num; i++)
    {
      indx = pLs->lsCtrl.prUsedSV[i];
      pSvMeas = &(pMeas->meas[indx]);
      if (pSvMeas->prn == 0) continue;

      cno = pSvMeas->cno;

      /* CNO limitation */
      if (cno < 20)
      {
        cno = 20;
      }
      else if (cno > 41)
      {
        cno = 41;
      }
      weight = (float)sqrt(293.3 * 293.3 / (2 * 0.02 * pow(10.0, cno / 10.0) * pow(10.0, cno / 10.0))
        + 293.3 * 293.3 / (4 * 0.02 * pow(10.0, cno / 10.0)));
      /* weight need future work */
      weight *= (float)pow(PR_WEIGHT_FACTOR, pSvMeas->prChckCnt);

      weight = (float)(10.0 / weight);

      pLs->prWeight[i] = weight;
    }
    if (num > 0)
    {
      for (i = 1; i < num; i++)
      {
        pLs->prWeight[i] /= pLs->prWeight[0];
      }
      pLs->prWeight[0] = 1.0;
    }

    /* Added height weight */
    if (pLs->lsCtrl.is2DPos)
    {
      pLs->prWeight[num] = 1.0;
    }
  }
  else             // DR weight
  {
    num = pLs->lsCtrl.drNum;
    for (i = 0; i < num; i++)
    {
      /*indx = pLs->lsCtrl.drUsedSV[i];
      pSvMeas = &(pMeas->meas[indx]);
      */

      pLs->drWeight[i] = 1.0;          // current use 1 as DR weight
    }

    /* Added up velocity weight */
    if (pLs->lsCtrl.is2DVel)
    {
      pLs->drWeight[num] = 1.0;
    }
  }
}


/***********************************************************************
* ��������: gnss_Ls_Post_Res
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�08/29/
***********************************************************************/
void gnss_Ls_Post_Res(Ls_t* pLs, uint8_t lsType, float HMatrix[][15], float solution[15])
{
  uint32_t        i, j;
  float        res;
  int32_t        freeDim;

  /* Calculate post residual
  (1) "post = pre - H * solution"
  (2) "post = pre" when LS converge because solution is very small and
  (H * solution) can be ignored
  */
  if (lsType == LS_POS_ID)
  {
    memset(pLs->postPrRes, 0, sizeof(float) * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM);
    pLs->pos_res = 0;
    for (i = 0; i < pLs->lsCtrl.prNum; i++)
    {
      res = 0.0;
      for (j = 0; j < (uint32_t)(pLs->lsCtrl.lsBiasNum + 3); j++)
      {
        res += HMatrix[i][j] * solution[j];
      }
      pLs->postPrRes[i] = pLs->prePrRes[i] - res;

      pLs->pos_res += SQR(pLs->postPrRes[i]);
    }
    /* Calculate freedom number */
    if (pLs->lsCtrl.is2DPos)
    {
      freeDim = (int32_t)pLs->lsCtrl.prNum - (int32_t)(pLs->lsCtrl.lsBiasNum + 2);
    }
    else
    {
      freeDim = (int32_t)pLs->lsCtrl.prNum - (int32_t)(pLs->lsCtrl.lsBiasNum + 3);
    }
    if (freeDim <= 0)
    {
      pLs->pos_res = (float)sqrt(pLs->pos_res);
    }
    else
    {
      pLs->pos_res = (float)sqrt(pLs->pos_res / freeDim);
    }

  }
  else
  {
    memset(pLs->postDrRes, 0, sizeof(float) * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM);
    pLs->vel_res = 0;
    for (i = 0; i < pLs->lsCtrl.drNum; i++)
    {
      res = 0.0;
      for (j = 0; j < 4; j++)
      {
        res += HMatrix[i][j] * solution[j];
      }
      pLs->postDrRes[i] = pLs->preDrRes[i] - res;
      pLs->vel_res += SQR(pLs->postDrRes[i]);
    }

    /* Calculate freedom number */
    if (pLs->lsCtrl.is2DVel)
    {
      freeDim = (int32_t)pLs->lsCtrl.drNum - (int32_t)(3);
    }
    else
    {
      freeDim = (int32_t)pLs->lsCtrl.drNum - (int32_t)(4);
    }
    if (freeDim <= 0)
    {
      pLs->vel_res = (float)sqrt(pLs->vel_res);
    }
    else
    {
      pLs->vel_res = (float)sqrt(pLs->vel_res / freeDim);
    }
  }
}
/**********************************************************************
* Function Name:    gnss_Ls_BiasCorrection
*
* Description:
*       1. LS bias correction for tm module
*
* Input:    valid SV info and user position
*
* Output:   user PVT
*
* Dependency
*      None
*
* Author: lupei
**********************************************************************/
void gnss_Ls_BiasCorrection(uint8_t satMode, int32_t biasCorrection, Ls_t* pLs)
{
  pLs->bias[satMode] -= biasCorrection * LIGHT_MSEC;
}

/***********************************************************************
* ��������: gnss_Ls_BiasJump_Detection
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/21/
***********************************************************************/
void gnss_Ls_BiasJump_Detection(Ls_t* pLs, uint8_t lsType)
{
  uint32_t              k, i, num;
  double              preDiffSum = 0, biasChange = 0;
  meas_blk_t* pMeas;
  gnss_meas_t* pSvMeas;

  /* Clock bias Jump detection
  1) Use PR prefix residual
  2) Only when clock status has bias, then do bias jump detection and adjust
  3) GPS, GLN and BDS may have clock jump at different time epoch
  */
  pMeas = pLs->meas;

  if (lsType == LS_POS_ID && (pLs->lsCtrl.clockStatus & LS_CLOCK_GPS_BIAS_KNOWN))
  {
    for (i = 0; i < BIAS_NUM; i++)
    {
      preDiffSum = 0;
      num = 0;
      for (k = 0; k < pLs->lsCtrl.prNum; k++)
      {
        pSvMeas = &(pMeas->meas[pLs->lsCtrl.prUsedSV[k]]);
        if (i != pSvMeas->gnssMode)
        {
          continue;
        }
        preDiffSum += pLs->prePrRes[k];
        num++;
      }

      if (num != 0)
      {
        preDiffSum /= num;
        if (preDiffSum > (double)(0.75 * LIGHT_MSEC))                // 1ms adjust
        {
          biasChange = LIGHT_MSEC;
          SYS_LOGGING(OBJ_LS, LOG_INFO, "LS detected 1ms bias adjust for GNSS(%d)", i);
        }
        else if (preDiffSum < -(double)(0.75 * LIGHT_MSEC))         // -1ms adjust
        {
          biasChange = -LIGHT_MSEC;
          SYS_LOGGING(OBJ_LS, LOG_INFO, "LS detected -1ms bias adjust for GNSS(%d)", i);
        }
        else if (preDiffSum > (double)(0.25 * LIGHT_MSEC))          // 0.5ms adjust
        {
          biasChange = 0.5 * LIGHT_MSEC;
          SYS_LOGGING(OBJ_LS, LOG_INFO, "LS detected 0.5ms bias adjust for GNSS(%d)", i);
        }
        else if (preDiffSum < -(double)(0.25 * LIGHT_MSEC))         //-0.5ms adjust
        {
          biasChange = -0.5 * LIGHT_MSEC;
          SYS_LOGGING(OBJ_LS, LOG_INFO, "LS detected -0.5ms bias adjust for GNSS(%d)", i);
        }
        else
        {
          biasChange = 0.0;
          //SYS_LOGGING(OBJ_LS,LOG_ERROR,"LS detected unexpected bias adjust");
        }
      }

      // LS Bias adjust 
      if (pLs->lsCtrl.biasIndx[i])
      {
        pLs->bias[i] -= biasChange;
      }

      // If biasChange is not zero, then reset KF bias with LS bias
      if (fabs(biasChange) > 1e-6)
      {
        kfInfo.X[6 + i] = pLs->bias[i] - kfInfo.X[10] * (pLs->lsCtrl.tor - pLs->lsCtrl.lastTor);
      }

      // Pre-fix residual adjust 
      for (k = 0; k < pLs->lsCtrl.prNum; k++)
      {
        pSvMeas = &(pMeas->meas[pLs->lsCtrl.prUsedSV[k]]);
        if (i == pSvMeas->gnssMode)
        {
          pLs->prePrRes[k] -= (float)biasChange;
        }
      }
    }
  }
}

/***********************************************************************
* ��������: gnss_Ls_Diff_Cal
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�08/29/
***********************************************************************/
void gnss_Ls_Diff_Cal(Ls_t* pLs, uint8_t lsType)
{
  uint8_t             indx;
  uint32_t            k, num;
  gnss_meas_t* pSvMeas;
  meas_blk_t* pMeas;
  double            lla_pos[3];
  double            detPos[3], detVel[3];
  double            biasUse[BIAS_NUM] = { 0.0 }, biasDiffUse[GNSS_MAX_MODE] = { 0.0 };
  double            calRange, calDoppler;
  double            dt;
  double            pseudoRangeUse;
  GNSS_TIME* pTime;
  pMeas = pLs->meas;

  pTime = gnss_tm_get_time();
  /* measurement count assign */
  if (lsType == LS_POS_ID)
  {
    num = pLs->lsCtrl.prNum;
    memset(pLs->prePrRes, 0, sizeof(float) * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM);
  }
  else
  {
    num = pLs->lsCtrl.drNum;
    memset(pLs->preDrRes, 0, sizeof(float) * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM);
  }

  if (pLs->lsCtrl.isLocalBias == FALSE)
  {
    memcpy(biasUse, pLs->bias, BIAS_NUM * sizeof(double));
  }
  else
  {
    memcpy(biasUse, pLs->biasLocal, BIAS_NUM * sizeof(double));
  }

  if (pMeas->rtdUsed)
  {
    memcpy(biasDiffUse, pLs->biasDiff, GNSS_MAX_MODE * sizeof(double));
  }
  else
  {
    memcpy(biasDiffUse, pLs->biasDiffLocal, GNSS_MAX_MODE * sizeof(double));
  }

  /* Calculate pre-position residual */
  for (k = 0; k < num; k++)
  {
    if (lsType == LS_POS_ID)
    {
      indx = pLs->lsCtrl.prUsedSV[k];
    }
    else
    {
      indx = pLs->lsCtrl.drUsedSV[k];
    }
    pSvMeas = &(pMeas->meas[indx]);
    detPos[0] = pSvMeas->sv_info.p[0] - pLs->lsEcef.p[0];
    detPos[1] = pSvMeas->sv_info.p[1] - pLs->lsEcef.p[1];
    detPos[2] = pSvMeas->sv_info.p[2] - pLs->lsEcef.p[2];
    calRange = gnssClcSqrtSum_DBL(detPos, 3);

    pseudoRangeUse = pSvMeas->pseudoRange;

    if (pSvMeas->freq_index == 1)
    {
      if (pLs->lsCtrl.lsDcbCalFlag[pSvMeas->gnssMode] ||
        (pLs->lsCtrl.isBiasLink && DBL_ISNT_EQUAL(pLs->dcb[pSvMeas->gnssMode], 0.0)))
      {
        pseudoRangeUse += pLs->dcb[pSvMeas->gnssMode];
      }
      else
      {
        pseudoRangeUse += pTime->dcb[pSvMeas->gnssMode];
      }
    }
    else if (pSvMeas->freq_index == 2)
    {
      if (pLs->lsCtrl.lsDcbCalFlag[pSvMeas->gnssMode + GNSS_MAX_MODE] ||
        (pLs->lsCtrl.isBiasLink && DBL_ISNT_EQUAL(pLs->dcb[pSvMeas->gnssMode + GNSS_MAX_MODE], 0.0)))
      {
        pseudoRangeUse += pLs->dcb[pSvMeas->gnssMode + GNSS_MAX_MODE];
      }
      else
      {
        pseudoRangeUse += pTime->dcb[pSvMeas->gnssMode + GNSS_MAX_MODE];
      }
    }

    if (lsType == LS_POS_ID)
    {
      dt = calRange / LIGHT_SEC;
      detPos[0] = (pSvMeas->sv_info.p[0] + (WGS84_OMEGDOTE * pSvMeas->sv_info.p[1] * dt) - pLs->lsEcef.p[0]);
      detPos[1] = (pSvMeas->sv_info.p[1] - (WGS84_OMEGDOTE * pSvMeas->sv_info.p[0] * dt) - pLs->lsEcef.p[1]);
      detPos[2] = pSvMeas->sv_info.p[2] - pLs->lsEcef.p[2];
      calRange = gnssClcSqrtSum_DBL(detPos, 3);

      if (pSvMeas->gnssMode == GLN_MODE && pLs->lsCtrl.isBiasLink)
      {
        pLs->prePrRes[k] = (float)(calRange + biasUse[GPS_MODE] - biasDiffUse[GLN_MODE] - pseudoRangeUse);
      }
      else if (pSvMeas->gnssMode == BDS_MODE && pLs->lsCtrl.isBiasLink)
      {
        pLs->prePrRes[k] = (float)(calRange + biasUse[GPS_MODE] - biasDiffUse[BDS_MODE] - pseudoRangeUse);
      }
      else if (pSvMeas->gnssMode == GAL_MODE && pLs->lsCtrl.isBiasLink)
      {
        pLs->prePrRes[k] = (float)(calRange + biasUse[GPS_MODE] - biasDiffUse[GAL_MODE] - pseudoRangeUse);
      }
      else
      {
        pLs->prePrRes[k] = (float)(calRange + biasUse[pSvMeas->gnssMode] - pseudoRangeUse);
      }

    }
    else
    {
      detVel[0] = (double)pSvMeas->sv_info.v[0] - pLs->lsEcef.v[0];
      detVel[1] = (double)pSvMeas->sv_info.v[1] - pLs->lsEcef.v[1];
      detVel[2] = (double)pSvMeas->sv_info.v[2] - pLs->lsEcef.v[2];

      calDoppler = (detVel[0] * detPos[0] + detVel[1] * detPos[1] + detVel[2] * detPos[2]) / calRange;

      pLs->preDrRes[k] = (float)(calDoppler + pLs->drift - pSvMeas->pseudoRangeRate);
    }
  }

  /* 2D position and velocity added */
  if (lsType == LS_POS_ID && pLs->lsCtrl.is2DPos)
  {
    gnssConvEcef2Lla(pLs->lsEcef.p, lla_pos);
    pLs->prePrRes[num] = (float)(lla_pos[2] - pLs->lsCtrl.aidHeight);
  }
  else if (lsType == LS_VEL_ID && pLs->lsCtrl.is2DVel)
  {
    calRange = gnssClcSqrtSum_DBL(pLs->lsEcef.p, 3);

    if (gnss_SignFLT((float)calRange) == 0)
    {
      calRange = (double)1.0;
      pLs->lsEcef.p[0] = (float)1.0;
    }

    pLs->preDrRes[num] = (float)((pLs->lsEcef.v[0] * pLs->lsEcef.p[0]
      + pLs->lsEcef.v[1] * pLs->lsEcef.p[1]
      + pLs->lsEcef.v[2] * pLs->lsEcef.p[2]) / calRange) - (float)pLs->lsCtrl.aidUpVel;
  }
}

/***********************************************************************
* ��������: gnss_Ls_DoP
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/08/
***********************************************************************/
void gnss_Ls_DoP(Ls_t* pLs, float HMatrix[][15], uint8_t row, uint8_t colume)
{
  /*
  DOP is calculated according to "H = inv(G'G)"
  G = QR     -->   H = inv(R'Q'QR)
  Q'Q = I    -->   H = inv(R'R) = inv(R) * inv(R') = inv(R) * (inv(R))'
  */
  uint32_t           i, j, k;
  float           TDOP = 0, PDOP = 0, DOPS[3];

  float           ceg[3][3];

  float* H = (float*)Sys_Malloc(sizeof(float) * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * LS_PARA_NUM);
  float* invR = (float*)Sys_Malloc(sizeof(float) * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * LS_PARA_NUM);
  float* DoPMatrix = (float*)Sys_Malloc(sizeof(float) * LS_PARA_NUM * LS_PARA_NUM);
  float* Ecef2lla = (float*)Sys_Malloc(sizeof(float) * LS_PARA_NUM * LS_PARA_NUM);

  if ((H == NULL) || (invR == NULL) || (DoPMatrix == NULL) || (Ecef2lla == NULL))
  {
    Sys_Free(H); Sys_Free(invR); Sys_Free(DoPMatrix); Sys_Free(Ecef2lla);
    return;
  }
  memset(H, 0, sizeof(float) * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * LS_PARA_NUM);
  memset(invR, 0, sizeof(float) * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * LS_PARA_NUM);
  memset(DoPMatrix, 0, sizeof(float) * LS_PARA_NUM * LS_PARA_NUM);
  memset(Ecef2lla, 0, sizeof(float) * LS_PARA_NUM * LS_PARA_NUM);

  memcpy(H, HMatrix, sizeof(float) * LS_PARA_NUM * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM);

  /* 1. QR Factorize */
  if (!gnss_QR_Factorize((float(*)[15])H, row, colume))
  {
    pLs->DoP.hDOP = 0;
    pLs->DoP.vDOP = 0;
    pLs->DoP.pDOP = 0;
    pLs->DoP.tDOP = 0;
    Sys_Free(H); Sys_Free(invR); Sys_Free(DoPMatrix); Sys_Free(Ecef2lla);
    return;
  }

  /* 2. Inverse matrix of R */
  memset(invR, 0, sizeof(float) * LS_PARA_NUM * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM);
  gnss_Inverse_UpMatrix((float(*)[15])H, (float(*)[15])invR, colume);

  /* 3. DOP Matrix calculation */
  memset(DoPMatrix, 0, sizeof(float) * LS_PARA_NUM * LS_PARA_NUM);
  gnss_UpMatrix_Square((float(*)[15])invR, (float(*)[15])DoPMatrix, colume);

  /* 4. Calculate DOPS */
  for (i = 0; i < pLs->lsCtrl.lsBiasNum; i++)
  {
    TDOP += DoPMatrix[(3 + i) * LS_PARA_NUM + 3 + i];
  }
  PDOP = DoPMatrix[0 * LS_PARA_NUM + 0] + DoPMatrix[1 * LS_PARA_NUM + 1] + DoPMatrix[2 * LS_PARA_NUM + 2];
  if (TDOP < 0.0 || PDOP < 0.0)
  {
    SYS_LOGGING(OBJ_LS, LOG_ERROR, "Wrong DOP calculation %s(%d)", __FUNCTION__, __LINE__);
    pLs->DoP.hDOP = 0;
    pLs->DoP.vDOP = 0;
    pLs->DoP.pDOP = 0;
    pLs->DoP.tDOP = 0;
    Sys_Free(H); Sys_Free(invR); Sys_Free(DoPMatrix); Sys_Free(Ecef2lla);
    return;
  }
  else
  {
    pLs->DoP.tDOP = (float)sqrt(TDOP);
    pLs->DoP.pDOP = (float)sqrt(PDOP);
  }
  /* 5. Get the transition matrix */
  gnssGetEcef2EnuMatrix(ceg, pLs->lsLla.lla);
  memset(Ecef2lla, 0, sizeof(float) * LS_PARA_NUM * LS_PARA_NUM);
  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      Ecef2lla[i * LS_PARA_NUM + j] = ceg[i][j];
    }
  }
  for (i = 0; i < pLs->lsCtrl.lsBiasNum; i++)
  {
    Ecef2lla[(3 + i) * LS_PARA_NUM + 3 + i] = 1;
  }
  for (i = 0; i < 3; i++)
  {
    DOPS[i] = 0;
    for (j = 0; j < 3; j++)
    {
      for (k = 0; k < 3; k++)
      {
        DOPS[i] += Ecef2lla[i * LS_PARA_NUM + j] * DoPMatrix[j * LS_PARA_NUM + k] * Ecef2lla[i * LS_PARA_NUM + k];
      }
    }
  }
  pLs->DoP.hDOP = sqrtf(DOPS[0] + DOPS[1]);
  pLs->DoP.vDOP = sqrtf(DOPS[2]);
  Sys_Free(H); Sys_Free(invR); Sys_Free(DoPMatrix); Sys_Free(Ecef2lla);
  return;
}

/***********************************************************************
* ��������: gnss_Ls_Cal_HMatrix
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�08/29/
***********************************************************************/
void gnss_Ls_Cal_HMatrix(Ls_t* pLs, float* HMatrix, uint8_t lsType)
{
  uint32_t           i, j, indx, num;
  double           detPos[3], range;
  gnss_meas_t* pSvMeas;
  meas_blk_t* pMeas = pLs->meas;
  uint8_t            L1_sat_num = 0;

  L1_sat_num = pLs->lsCtrl.lsBiasNum - pLs->lsCtrl.lsDcbNum[0] - pLs->lsCtrl.lsDcbNum[1];
  /* assign measurement number */
  if (lsType == LS_POS_ID)
  {
    num = pLs->lsCtrl.prNum;
  }
  else
  {
    num = pLs->lsCtrl.drNum;
  }


  for (i = 0; i < num; i++)
  {
    if (lsType == LS_POS_ID)
    {
      indx = pLs->lsCtrl.prUsedSV[i];
    }
    else
    {
      indx = pLs->lsCtrl.drUsedSV[i];
    }

    pSvMeas = &(pMeas->meas[indx]);

    /* calculate direction cosine */
    detPos[0] = pSvMeas->sv_info.p[0] - pLs->lsEcef.p[0];
    detPos[1] = pSvMeas->sv_info.p[1] - pLs->lsEcef.p[1];
    detPos[2] = pSvMeas->sv_info.p[2] - pLs->lsEcef.p[2];
    range = gnssClcSqrtSum_DBL(detPos, 3);

    for (j = 0; j < 3; j++)
    {
      HMatrix[i * LS_PARA_NUM + j] = (float)(detPos[j] / range);
    }

    if (lsType == LS_POS_ID)
    {
      indx = pLs->lsCtrl.biasIndx[pSvMeas->gnssMode];
      HMatrix[i * LS_PARA_NUM + indx + 2] = 1.0;
      if ((pLs->lsCtrl.dcbIndx[pSvMeas->gnssMode] > 0) && (pSvMeas->freq_index == 1))
      {
        indx = pLs->lsCtrl.dcbIndx[pSvMeas->gnssMode];
        HMatrix[i * LS_PARA_NUM + indx + L1_sat_num + 2] = -1.0;
      }
      if ((pLs->lsCtrl.dcbIndx[pSvMeas->gnssMode + GNSS_MAX_MODE] > 0) && (pSvMeas->freq_index == 2))
      {
        indx = pLs->lsCtrl.dcbIndx[pSvMeas->gnssMode + GNSS_MAX_MODE];
        HMatrix[i * LS_PARA_NUM + indx + L1_sat_num + pLs->lsCtrl.lsDcbNum[0] + 2] = -1.0;
      }
    }
    else
    {
      HMatrix[i * LS_PARA_NUM + 3] = 1.0;
    }
  }

  if ((lsType == LS_POS_ID && pLs->lsCtrl.is2DPos)
    || (lsType == LS_VEL_ID && pLs->lsCtrl.is2DVel))
  {
    range = gnssClcSqrtSum_DBL(pLs->lsEcef.p, 3);

    if (gnss_SignFLT((float)range) == 0)
    {
      range = (double)1.0;
      pLs->lsEcef.p[0] = (double)1.0;
    }

    HMatrix[LS_PARA_NUM * num + 0] = (float)(-pLs->lsEcef.p[0] / range);
    HMatrix[LS_PARA_NUM * num + 1] = (float)(-pLs->lsEcef.p[1] / range);
    HMatrix[LS_PARA_NUM * num + 2] = (float)(-pLs->lsEcef.p[2] / range);
    HMatrix[LS_PARA_NUM * num + 3] = 0;
  }
}

/***********************************************************************
* ��������: gnss_Ls_Update
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�08/29/
***********************************************************************/
void gnss_Ls_Update(Ls_t* pLs, uint8_t lsType, float solution[])
{
  uint32_t        k;
  uint8_t            L1_sat_num = 0;

  L1_sat_num = pLs->lsCtrl.lsBiasNum - pLs->lsCtrl.lsDcbNum[0] - pLs->lsCtrl.lsDcbNum[1];

  if (lsType == LS_POS_ID)
  {
    pLs->lsEcef.p[0] += solution[0];
    pLs->lsEcef.p[1] += solution[1];
    pLs->lsEcef.p[2] += solution[2];
    for (k = 0; k < 4; k++)
    {
      if (pLs->lsCtrl.biasIndx[k] != 0)
      {
        if (pLs->lsCtrl.isLocalBias == FALSE)
        {
          pLs->bias[k] -= solution[pLs->lsCtrl.biasIndx[k] + 2];
          if (pLs->lsCtrl.dcbIndx[k] > 0)
          {
            pLs->dcb[k] -= solution[pLs->lsCtrl.dcbIndx[k] + 2 + L1_sat_num];
          }
          if (pLs->lsCtrl.dcbIndx[k + 4] > 0)
          {
            pLs->dcb[k + 4] -= solution[pLs->lsCtrl.dcbIndx[k + 4] + 2 + L1_sat_num + pLs->lsCtrl.lsDcbNum[0]];
          }

        }
        else
        {
          pLs->biasLocal[k] -= solution[pLs->lsCtrl.biasIndx[k] + 2];
        }
      }
    }
  }
  else
  {
    pLs->lsEcef.v[0] += solution[0];
    pLs->lsEcef.v[1] += solution[1];
    pLs->lsEcef.v[2] += solution[2];
    pLs->drift -= solution[3];
  }
}
/***********************************************************************
* ��������: gnss_Ls_Check_BackupAlt
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�08/29/
***********************************************************************/
void gnss_Ls_Check_BackupAlt(Ls_t* pLs)
{
  double             distance, delta_alt;
  USER_PVT* pUser_pvt;
  history_Info* pHisInfo;

  pUser_pvt = &user_pvt_backup;
  pHisInfo = &HisInfo;
  if (firstFix)
  {
    return;
  }

  if (pLs->ls_Pvt_Info->valid && pUser_pvt->have_position > HAVE_POS_NONE)
  {
    distance = gnssCalPosDis(pUser_pvt->lla.pos, pLs->ls_Pvt_Info->LLApos, TRUE);
    delta_alt = pLs->ls_Pvt_Info->LLApos[2] - pUser_pvt->lla.pos[2];

    if (distance < 5e3 && fabs(delta_alt)> 50 && pHisInfo->kfFixStatus == FIX_SOURCE_3D &&
      pHisInfo->kfPosRes < 30 && pHisInfo->kfRunEpochCnt > 60)
    {
      pLs->lsCtrl.is2DPos = TRUE;
      pLs->lsCtrl.isForce2D = TRUE;
      pLs->lsCtrl.aidHeight = (float)pUser_pvt->lla.pos[2];
    }
  }
}
/***********************************************************************
* ��������: gnss_Ls_Pos
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�08/29/
***********************************************************************/
uint8_t gnss_Ls_Pos(Ls_t* pLs)
{
  uint8_t            status;
  uint32_t           i, j, k, row, colume;
  float           solution[LS_PARA_NUM] = { 0.0f };
  float           iterCnvg;
  GNSS_TIME* pTime;

  pTime = gnss_tm_get_time();

  /* store HMatrix HDOP HDOP in static region to reduce Maximum Stack Usage */
  float* HMatrix = (float*)Sys_Malloc(sizeof(float) * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * LS_PARA_NUM);
  float* HDOP = (float*)Sys_Malloc(sizeof(float) * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * LS_PARA_NUM);
  float* prRes = (float*)Sys_Malloc(sizeof(float) * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM);

  if ((HMatrix == NULL) || (HDOP == NULL) || (prRes == NULL))
  {
    Sys_Free(HMatrix);
    Sys_Free(HDOP);
    Sys_Free(prRes);
    return FALSE;
  }

  memset(HMatrix, 0, sizeof(float) * LS_PARA_NUM * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM);
  memset(HDOP, 0, sizeof(float) * LS_PARA_NUM * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM);
  memset(prRes, 0, sizeof(float) * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM);

  /* weight calculation */
  gnss_Ls_Weight_Cal(pLs, LS_POS_ID);

  /* check the bias number */
  gnss_Ls_Check_BiasNum(pLs);

  row = pLs->lsCtrl.prNum + (pLs->lsCtrl.is2DPos == 1);
  colume = 3 + pLs->lsCtrl.lsBiasNum;

  /* make sure full rank */
  if (row < colume)
  {
    pLs->lsCtrl.status &= ~(LS_STATUS_HAS_2D_POS | LS_STATUS_HAS_3D_POS);
    pLs->posCvg = FALSE;
    Sys_Free(HMatrix); Sys_Free(HDOP); Sys_Free(prRes);
    return FALSE;
  }

  /*loops for position calculation */
  for (i = 0; i < LS_POS_MAX_ITERA; i++)
  {
    /*1. Calculate PR residual */
    gnss_Ls_Diff_Cal(pLs, LS_POS_ID);

    /*2. Clear and calculate Matrix */
    memset(HMatrix, 0, sizeof(float) * LS_PARA_NUM * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM);
    gnss_Ls_Cal_HMatrix(pLs, HMatrix, LS_POS_ID);
    memcpy(HDOP, HMatrix, sizeof(float) * LS_PARA_NUM * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM);
    /*3. add weight to GEO matrix and diff */
    memcpy(prRes, pLs->prePrRes, sizeof(float) * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM);
    for (k = 0; k < pLs->lsCtrl.prNum; k++)
    {
      for (j = 0; j < (uint32_t)(3 + pLs->lsCtrl.lsBiasNum); j++)
      {
        HMatrix[k * LS_PARA_NUM + j] *= (float)pLs->prWeight[k];
      }

      prRes[k] *= pLs->prWeight[k];
    }
    /* Aided height weight */
    if (pLs->lsCtrl.is2DPos)
    {
      prRes[k] *= pLs->prWeight[k];
    }

    /*4. QR LS calculation */
    status = gnss_Ls_Qr((float(*)[15])HMatrix, (int32_t)(row), (int32_t)(colume), (float*)&(prRes[0]), &solution[0]);

    /*5. Check the LS converge status */
    if (status == FALSE)
    {
      if (pLs->lsCtrl.is2DPos)
      {
        pLs->lsCtrl.status &= ~(LS_STATUS_HAS_2D_POS);
      }
      else
      {
        pLs->lsCtrl.status &= ~(LS_STATUS_HAS_3D_POS);
      }
      pLs->posCvg = FALSE;
      memset(pLs->lsEcef.p, 0, sizeof(double) * 3);
      Sys_Free(HMatrix); Sys_Free(HDOP); Sys_Free(prRes);
      return FALSE;
    }

    /*6. Update the LS position */
    gnss_Ls_Update(pLs, LS_POS_ID, solution);

    /*7. Post-residual calculation */
    gnss_Ls_Post_Res(pLs, LS_POS_ID, (float(*)[15])HDOP, solution);

    /*8. Check if need step out*/
    iterCnvg = ((float)fabs(solution[0]) + (float)fabs(solution[1])
      + (float)fabs(solution[2]) + (float)fabs(solution[3]));

    // Calculating the DOP and Error
    //gnss_Ls_DoP(pLs,(float(*)[15])HDOP,row,colume);	

    if (iterCnvg <= (float)LS_POS_CONVERGE_THRES && status == TRUE)
    {
      if (pLs->pos_res > 300.0)
      {
        SYS_LOGGING(OBJ_LS, LOG_INFO, "Very Large POS RES in LS %10.3f", pLs->pos_res);
        goto LS_FAIL_TAG;
      }
      for (j = 0; j < BIAS_NUM * 2; j++)
      {
        if (pLs->lsCtrl.dcbIndx[j] > 0 && DBL_ISNT_EQUAL(pLs->dcb[j], 0.0) && fabs(pLs->dcb[j] - pTime->dcb[j]) > 50.0)
        {
          SYS_LOGGING(OBJ_LS, LOG_INFO, "Very Large DCB error in LS j %d, LS DCB %10.3f, pTime DCB %10.3f", j, pLs->dcb[j], pTime->dcb[j]);
          goto LS_FAIL_TAG;
        }
      }
      // Set the position converge status
      if (pLs->lsCtrl.is2DPos)
      {
        pLs->lsCtrl.status |= LS_STATUS_HAS_2D_POS;
      }
      else
      {
        pLs->lsCtrl.status |= LS_STATUS_HAS_3D_POS;
      }
      /* If LS converge, then clock bias is known, no matter previous clock status
      is known or unknown.
      */
      pLs->lsCtrl.clockStatus |= LS_CLOCK_GPS_BIAS_KNOWN;

      // Convert ECEF position to LLA
      gnssConvEcef2Lla(&(pLs->lsEcef.p[0]), &(pLs->lsLla.lla[0]));

      // Calculating the DOP and Error
      gnss_Ls_DoP(pLs, (float(*)[15])HDOP, row, colume);

      gnss_Ls_PosErrEstimate(pLs);

      pLs->posCvg = TRUE;
      //SYS_LOGGING(OBJ_LS,LOG_INFO,"LS position converge at time %14.6f %02d",pLs->meas->tor,pLs->meas->measCnt);
      Sys_Free(HMatrix); Sys_Free(HDOP); Sys_Free(prRes);
      return TRUE;
    }
    else if (((iterCnvg - (float)fabs(solution[3])) > (float)(500000.0) || (pLs->pos_res > 3000.0)) && i >= 5)
    {
    LS_FAIL_TAG:
      status = FALSE;
      memset(pLs->lsEcef.p, 0, sizeof(double) * 3);
      /* Clear the position converge status */
      memset(pLs->dcb, 0, sizeof(double) * BIAS_NUM * 2);
      if (pLs->lsCtrl.is2DPos)
      {
        pLs->lsCtrl.status &= ~(LS_STATUS_HAS_2D_POS);
      }
      else
      {
        pLs->lsCtrl.status &= ~(LS_STATUS_HAS_3D_POS);
      }
      SYS_LOGGING(OBJ_LS, LOG_INFO, "LS position dis-converge");

      // Calculating the DOP and Error
      gnss_Ls_DoP(pLs, (float(*)[15])HDOP, row, colume);

      pLs->posCvg = FALSE;
      Sys_Free(HMatrix); Sys_Free(HDOP); Sys_Free(prRes);
      return FALSE;
    }
  }
  // Calculating the DOP and Error
  gnss_Ls_DoP(pLs, (float(*)[15])HDOP, row, colume);

  pLs->posCvg = FALSE;
  Sys_Free(HMatrix); Sys_Free(HDOP); Sys_Free(prRes);
  return FALSE;
}

void gnss_AddingNormalEquation(uint32_t q_paraNum, const float* pf_oneObsH, float f_weight, float f_omc, float* pf_BtP, float* pf_BtPB, float* pf_BtPL)
{
  uint32_t q_iRow = 0;
  uint32_t q_iCol = 0;
  uint32_t q_rowIndex = 0;
  for (q_iRow = 0; q_iRow < q_paraNum; ++q_iRow)
  {
    pf_BtP[q_iRow] = pf_oneObsH[q_iRow] * f_weight;
  }
  /*calculate the BtPB*/
  for (q_iRow = 0; q_iRow < q_paraNum; ++q_iRow)
  {
    q_rowIndex = q_iRow * q_paraNum;
    for (q_iCol = q_iRow; q_iCol < q_paraNum; ++q_iCol)
    {
      pf_BtPB[q_rowIndex + q_iCol] += pf_BtP[q_iRow] * pf_oneObsH[q_iCol];
    }
  }
  for (q_iRow = 0; q_iRow < q_paraNum; ++q_iRow)
  {
    q_rowIndex = q_iRow * q_paraNum;
    for (q_iCol = 0; q_iCol < q_iRow; ++q_iCol)
    {
      pf_BtPB[q_rowIndex + q_iCol] = pf_BtPB[q_iCol * q_paraNum + q_iRow];
    }
  }
  /*calculate the BtPL*/
  for (q_iRow = 0; q_iRow < q_paraNum; ++q_iRow)
  {
    pf_BtPL[q_iRow] += pf_BtP[q_iRow] * f_omc;
  }
  return;
}

uint8_t gnss_LS_solution(uint32_t q_paraNum, const float* pf_BtPB, const float* pf_BtPL, float* pf_solution, float* pf_NEQinv)
{
  uint8_t u_status = FALSE;
  uint32_t q_iRow = 0;
  uint32_t q_iCol = 0;
  uint32_t q_rowIndex = 0;
  matrix_t* pz_BtPB = NULL;
  matrix_t* pz_NEQinv = NULL;
  pz_BtPB = matrix_new(q_paraNum, q_paraNum);
  pz_NEQinv = matrix_new(q_paraNum, q_paraNum);
  for (q_iRow = 0; q_iRow < q_paraNum; ++q_iRow)
  {
    q_rowIndex = q_iRow * q_paraNum;
    for (q_iCol = 0; q_iCol < q_paraNum; ++q_iCol)
    {
      pz_BtPB->data[q_rowIndex + q_iCol] = (double)pf_BtPB[q_rowIndex + q_iCol];
    }
  }
  if (TRUE == matrix_inverse(pz_BtPB, pz_NEQinv))
  {
    for (q_iRow = 0; q_iRow < q_paraNum; ++q_iRow)
    {
      pf_solution[q_iRow] = 0.0;
      q_rowIndex = q_iRow * q_paraNum;
      for (q_iCol = 0; q_iCol < q_paraNum; ++q_iCol)
      {
        pf_NEQinv[q_rowIndex + q_iCol] = (float)pz_NEQinv->data[q_rowIndex + q_iCol];
        pf_solution[q_iRow] += (float)(pz_NEQinv->data[q_rowIndex + q_iCol] * pf_BtPL[q_iCol]);
      }
    }
    u_status = TRUE;
  }
  matrix_free(&pz_BtPB);
  matrix_free(&pz_NEQinv);
  return u_status;
}

void gnss_LS_Doppler_DOP(const float* pf_NEQinv, Ls_t* pLs)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_k = 0;
  float   pf_ceg[3][3] = { {0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f} };
  float   pf_DOPS[3] = { 0.0f };
  float   f_pDOP = 0.0;
  float   f_tDOP = 0.0;
  float   f_hDOP = 0.0;
  /*pf_NEQinv[0]  is pf_NEQinv[0*4+0], pf_NEQinv[5] is pf_NEQinv[1*4+1]*/
  /*pf_NEQinv[10] is pf_NEQinv[2*4+2],pf_NEQinv[15] is pf_NEQinv[3*4+3]*/
  if (pf_NEQinv[0] < 0.0 || pf_NEQinv[5] < 0.0 || pf_NEQinv[10] < 0.0 || pf_NEQinv[15] < 0.0)
  {
    pLs->DoP.hDOP = 0.0f;
    pLs->DoP.vDOP = 0.0f;
    pLs->DoP.pDOP = 0.0f;
    pLs->DoP.tDOP = 0.0f;
  }
  f_pDOP = pf_NEQinv[0] + pf_NEQinv[5] + pf_NEQinv[10];
  if (f_pDOP >= 0.0)
  {
    pLs->DoP.pDOP = (float)sqrt(f_pDOP);
  }
  f_tDOP = pf_NEQinv[15];
  if (f_tDOP >= 0.0)
  {
    pLs->DoP.tDOP = (float)sqrt(f_tDOP);
  }
  /* Get the transition matrix */
  gnssGetEcef2EnuMatrix(pf_ceg, pLs->lsLla.lla);
  for (u_i = 0; u_i < 3; u_i++)
  {
    pf_DOPS[u_i] = 0;
    for (u_j = 0; u_j < 3; u_j++)
    {
      for (u_k = 0; u_k < 3; u_k++)
      {
        pf_DOPS[u_i] += pf_ceg[u_i][u_j] * pf_NEQinv[u_j * 4 + u_k] * pf_ceg[u_i][u_k];
      }
    }
  }
  f_hDOP = pf_DOPS[0] + pf_DOPS[1];
  if (f_hDOP >= 0.0)
  {
    pLs->DoP.hDOP = sqrtf(f_hDOP);
  }
  if (pf_DOPS[2] >= 0.0)
  {
    pLs->DoP.vDOP = sqrtf(pf_DOPS[2]);
  }
  return;
}
/***********************************************************************
* ��������: gnss_Ls_Vel
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�08/29/
***********************************************************************/
uint8_t gnss_Ls_Vel(Ls_t* pLs, uint8_t flag)
{
  uint8_t         status = FALSE;
  uint32_t        i = 0, k = 0, row = 0, colume = 0;
  float        solution[4] = {0.0f};
  float        iterCnvg = 0.0f;
  
  float* pf_BtPB = NULL;
  float* pf_BtPL = NULL;
  float* pf_BtP = NULL;
  float* pf_NEQinv = NULL;
  float* HMatrix = (float*)Sys_Malloc(sizeof(float) * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * LS_PARA_NUM);
  if ((HMatrix == NULL))
  {
    Sys_Free(HMatrix);
    return FALSE;
  }

  memset(HMatrix, 0, sizeof(float) * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM * LS_PARA_NUM);

  // Check if we can calculate velocity
  if ((pLs->lsCtrl.status & LS_STATUS_HAS_3D_POS) == 0 && (pLs->lsCtrl.status & LS_STATUS_HAS_2D_POS) == 0)
  {
    pLs->lsCtrl.status &= (~LS_STATUS_HAS_3D_VEL);
    pLs->lsCtrl.status &= (~LS_STATUS_HAS_2D_VEL);
    SYS_LOGGING(OBJ_LS, LOG_INFO, "There is no LS position for LS velocity");
    Sys_Free(HMatrix);
    return FALSE;
  }
  else if (!pLs->lsCtrl.velUpdateFlag)
  {
    pLs->lsCtrl.status &= (~LS_STATUS_HAS_3D_VEL);
    pLs->lsCtrl.status &= (~LS_STATUS_HAS_2D_VEL);
    SYS_LOGGING(OBJ_LS, LOG_INFO, "Could not do  LS velocity update");
    Sys_Free(HMatrix);
    return FALSE;
  }

  row = pLs->lsCtrl.drNum + (pLs->lsCtrl.is2DVel == 1);
  colume = 4;
  pf_BtPB = (float*)Sys_Malloc(sizeof(float) * colume * colume);
  pf_BtPL = (float*)Sys_Malloc(sizeof(float) * colume);
  pf_BtP = (float*)Sys_Malloc(sizeof(float) * colume);
  pf_NEQinv = (float*)Sys_Malloc(sizeof(float) * colume * colume);
  memset(pf_BtPB, 0, sizeof(float) * colume * colume);
  memset(pf_BtPL, 0, sizeof(float) * colume);
  memset(pf_BtP, 0, sizeof(float) * colume);
  memset(pf_NEQinv, 0, sizeof(float) * colume * colume);

  if (row < colume)
  {
    pLs->velCvg = FALSE;
    pLs->lsCtrl.status &= (~LS_STATUS_HAS_3D_VEL);
    pLs->lsCtrl.status &= (~LS_STATUS_HAS_2D_VEL);
    Sys_Free(HMatrix);
    Sys_Free(pf_BtPB);
    Sys_Free(pf_BtPL);
    Sys_Free(pf_BtP);
    Sys_Free(pf_NEQinv);
    return FALSE;
  }

  gnss_Ls_Weight_Cal(pLs, LS_VEL_ID);
  /* loops for velocity calculations */
  for (i = 0; i < LS_VEL_MAX_ITERA; i++)
  {
    memset(pf_BtPB, 0, sizeof(float) * colume * colume);
    memset(pf_NEQinv, 0, sizeof(float) * colume * colume);
    memset(pf_BtPL, 0, sizeof(float) * colume);
    memset(pf_BtP, 0, sizeof(float) * colume);
    /*1. Calculate DR residual */
    gnss_Ls_Diff_Cal(pLs, LS_VEL_ID);

    /*2. Clear and calculate Matrix */
    memset(HMatrix, 0, sizeof(float) * LS_PARA_NUM * MAX_MEAS_NUM * AGGNSS_MAX_FREQ_NUM);

    gnss_Ls_Cal_HMatrix(pLs, HMatrix, LS_VEL_ID);

    /*3. add weight to GEO matrix and diff*/
    for (k = 0; k < pLs->lsCtrl.drNum; k++)
    {
      gnss_AddingNormalEquation(colume, &HMatrix[k * LS_PARA_NUM], (float)pLs->drWeight[k], pLs->preDrRes[k], pf_BtP, pf_BtPB, pf_BtPL);
    }
    if (pLs->lsCtrl.is2DVel)
    {
      gnss_AddingNormalEquation(colume, &HMatrix[k * LS_PARA_NUM], (float)pLs->drWeight[k], pLs->preDrRes[k], pf_BtP, pf_BtPB, pf_BtPL);
    }

    /*4. LS calculation */
    status = gnss_LS_solution(colume, pf_BtPB, pf_BtPL, &solution[0], pf_NEQinv);

    /*5. Check the LS converge status */
    if (status == FALSE)
    {
      if (pLs->lsCtrl.is2DVel)
      {
        pLs->lsCtrl.status &= ~(LS_STATUS_HAS_2D_VEL);
      }
      else
      {
        pLs->lsCtrl.status &= ~(LS_STATUS_HAS_3D_VEL);
      }
      Sys_Free(HMatrix);
      Sys_Free(pf_BtPB);
      Sys_Free(pf_BtPL);
      Sys_Free(pf_BtP);
      Sys_Free(pf_NEQinv);
      return FALSE;
    }

    /*6. Update the LS velocity */
    gnss_Ls_Update(pLs, LS_VEL_ID, solution);

    /*7. calculate post residual */
    gnss_Ls_Post_Res(pLs, LS_VEL_ID, (float(*)[15])HMatrix, solution);

    /*8. Check if need step out*/
    iterCnvg = ((float)fabs(solution[0]) + (float)fabs(solution[1])
      + (float)fabs(solution[2]) + (float)fabs(solution[3]));
    if (iterCnvg <= (float)LS_VEL_CONVERGE_THRES && status == TRUE)
    {
      if (pLs->lsCtrl.is2DVel)
      {
        pLs->lsCtrl.status |= LS_STATUS_HAS_2D_VEL;
      }
      else
      {
        pLs->lsCtrl.status |= LS_STATUS_HAS_3D_VEL;
      }
      pLs->velCvg = status;

      gnssConvEcef2EnuVel(&pLs->lsEcef.v[0], pLs->vel_enu, pLs->lsLla.lla);
      /* DR RAIM Need DOP calculation */
      if (flag == TRUE)
      {
        gnss_LS_Doppler_DOP(pf_NEQinv, pLs);
      }
      Sys_Free(HMatrix);
      Sys_Free(pf_BtPB);
      Sys_Free(pf_BtPL);
      Sys_Free(pf_BtP);
      Sys_Free(pf_NEQinv);
      return TRUE;
    }
    else if ((iterCnvg - (float)fabs(solution[3])) > (float)(500000.0) && i >= 3)
    {
      status = FALSE;
      pLs->velCvg = status;
      if (pLs->lsCtrl.is2DVel)
      {
        pLs->lsCtrl.status &= ~(LS_STATUS_HAS_2D_VEL);
      }
      else
      {
        pLs->lsCtrl.status &= ~(LS_STATUS_HAS_3D_VEL);
      }
      Sys_Free(HMatrix);
      Sys_Free(pf_BtPB);
      Sys_Free(pf_BtPL);
      Sys_Free(pf_BtP);
      Sys_Free(pf_NEQinv);
      return FALSE;
    }

  }
  pLs->velCvg = FALSE;
  Sys_Free(HMatrix);
  Sys_Free(pf_BtPB);
  Sys_Free(pf_BtPL);
  Sys_Free(pf_BtP);
  Sys_Free(pf_NEQinv);
  return FALSE;
}

/***********************************************************************
* ��������: gnss_Ls_VerticalVel_Chck
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�10/13/
***********************************************************************/
uint8_t gnss_Ls_VerticalVel_Chck(Ls_t* pLs)
{
  uint8_t            ret_val = TRUE;
  static uint8_t     firstChck = FALSE;
  double           dt = 0;
  double           delta = 0.0, horizonVel = 0.0;
  LS_PVT_INFO* ls_pvt_back = &pLs->ls_Pvt_Back;

  /*
  If LS has 3D position and velocity, exit tunnel, but vertical speed
  */
  if ((pLs->lsCtrl.status & LS_STATUS_HAS_3D_POS)
    && (pLs->lsCtrl.status & LS_STATUS_HAS_3D_VEL))
  {
    if (peMode.tunnelMode.exitTunnelFlag == TRUE && fabs(pLs->vel_enu[2]) >= 50)
    {
      return FALSE;
    }
  }
  /* horizon velocity constrain */
  horizonVel = sqrt(SQR((double)pLs->vel_enu[0]) + SQR((double)pLs->vel_enu[1]));
  if ((horizonVel >= 100.0 && !g_pe_cfg.ppk_mode) || (horizonVel >= 500.0 && g_pe_cfg.ppk_mode))
  {
    SYS_LOGGING(OBJ_LS, LOG_INFO, "Abnormal horizon velocity");
    return FALSE;
  }
  if (ls_pvt_back->fix_status == FIX_STATUS_NEW)
  {
    if (firstChck == FALSE)
    {
      firstChck = TRUE;
    }
    else
    {
      dt = pLs->lsCtrl.tor - ls_pvt_back->tor;
      // If has started check and last good results has passed 120s, then restart
      if (dt > 120.0)
      {
        firstChck = FALSE;
      }
    }
  }

  // if not started check, then return
  if (firstChck == FALSE)
  {
    return ret_val;
  }
  dt = pLs->lsCtrl.tor - ls_pvt_back->tor;
  if (dt < -SECS_IN_WEEK / 2.0)
  {
    dt += (double)SECS_IN_WEEK;
  }
  else if (dt > SECS_IN_WEEK / 2.0)
  {
    dt -= (double)SECS_IN_WEEK;
  }

  if (pLs->lsCtrl.status & LS_STATUS_HAS_3D_VEL)
  {
    if (fabs(pLs->vel_enu[2]) > 0.0 && fabs(ls_pvt_back->enuVel[2]) > 0.0)
    {
      delta = fabsf(pLs->vel_enu[2] - ls_pvt_back->enuVel[2]);
      if (delta > fabs(dt) * 15)
      {
        ret_val = FALSE;
        SYS_LOGGING(OBJ_LS, LOG_INFO, "Abnormal Vertical speed:time(%6.2f),delta vel(%6.2f)", dt, delta);
      }
    }
  }

  return ret_val;
}

/***********************************************************************
* ��������: gnss_Ls_Height_Chck
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�10/30/
***********************************************************************/
uint8_t gnss_Ls_Height_Chck(Ls_t* pLs)
{
  if (pLs->lsLla.lla[2] >= pLs->pCfg->altHighLimit || pLs->lsLla.lla[2] <= pLs->pCfg->altLowLimit)
  {
    SYS_LOGGING(OBJ_LS, LOG_ERROR, "Altitude check fail %f %f %f",
      pLs->lsLla.lla[2], pLs->pCfg->altHighLimit, pLs->pCfg->altLowLimit);
    return FALSE;
  }
  else
  {
    return TRUE;
  }
}
/***********************************************************************
* ��������: gnss_Ls_QoS_Ctrl
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�10/13/
***********************************************************************/
uint8_t gnss_Ls_Dop_Qos(Ls_t* pLs)
{
  uint8_t              flag = TRUE;
  Gnss_Cfg_t* pCfg;
  double             pdop_mask;
  //GNSS_TIME* pTime;

  pCfg = pLs->pCfg;
  pdop_mask = pCfg->pdop_mask;
  //pTime = gnss_tm_get_time();

  /*
  1. First check the PDOP, if pass the threshold, then consider useful
  2. Then check the HDOP, if HDOP is small and at least one redundancy
  consider LS result is useful
  3. If HDOP is large than 4, then
  (1) HDOP > 8, false
  (2) HDOP < 8, at least one redundancy and position residual less than 25, true
  (3) HDOP < 8, check elapsed time and distance
  */
  if (pLs->DoP.pDOP > pdop_mask)
  {
    if (firstFix == FALSE)
    {
      flag = FALSE;
      goto RET_PLACE;
    }
    // Very large PDOP, then LS is invalid
    if (pLs->DoP.pDOP >= 10)
    {
      flag = FALSE;
      SYS_LOGGING(OBJ_LS, LOG_INFO, "%s,%d,PDOP=%10.5f,flag=%d", __FUNCTION__, __LINE__, pLs->DoP.pDOP, flag);
      goto RET_PLACE;
    }

    if (pLs->DoP.hDOP < 4 && (pLs->lsCtrl.prNum - pLs->lsCtrl.lsBiasNum - 3) >= 2)
    {
      flag = TRUE;
      goto RET_PLACE;
    }
    else
    {
      // 1. HDOP check, if > 8, then LS is invalid
      if (pLs->DoP.hDOP > 8)
      {
        flag = FALSE;
        SYS_LOGGING(OBJ_LS, LOG_INFO, "LS HDOP FAIL:%5.2f", pLs->DoP.hDOP);
        goto RET_PLACE;
      }

      // 2. Redundancy and residual check
      if ((pLs->lsCtrl.prNum - pLs->lsCtrl.lsBiasNum - 3) >= 3 && pLs->pos_res < pCfg->pos_res_thres)
      {
        flag = TRUE;
        goto RET_PLACE;
      }

      // 3. Time and distance check 
      //dt = pTime->tor - pTime->startTor;

      SYS_LOGGING(OBJ_LS, LOG_INFO, "LS DOP FAIL:PDOP:%5.2f,HDOP:%5.2f", pLs->DoP.pDOP, pLs->DoP.hDOP);
      flag = FALSE;
    }
  }
  else
  {
    SYS_LOGGING(OBJ_LS, LOG_INFO, "LS PDOP Passed: PDOP:%5.2f, PDOP_MASK:%5.2f", pLs->DoP.pDOP, pdop_mask);
  }

RET_PLACE:
  return flag;
}
/***********************************************************************
* ��������: gnss_Ls_QoS_Ctrl
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�08/29/
***********************************************************************/
uint8_t gnss_Ls_QoS_Ctrl(Ls_t* pLs)
{
  uint8_t              ret_flag = TRUE;
  uint8_t              dop_flag = TRUE;
  uint8_t              vel_flag = TRUE;
  uint8_t              alt_flag = TRUE;
  /* 1. Check if there is 2D or 3D position */
  if (((pLs->lsCtrl.status & LS_STATUS_HAS_3D_POS) == 0) &&
    ((pLs->lsCtrl.status & LS_STATUS_HAS_2D_POS) == 0))
  {
    ret_flag = FALSE;
    return ret_flag;
  }
  else if (((pLs->lsCtrl.status & LS_STATUS_HAS_3D_VEL) == 0) &&
    ((pLs->lsCtrl.status & LS_STATUS_HAS_2D_VEL) == 0) && (g_pe_cfg.meas_type_mask & MEAS_TYPE_MASK_DR))
  {
    ret_flag = FALSE;
    return ret_flag;
  }

  /* 2. DOP check */
  dop_flag = gnss_Ls_Dop_Qos(pLs);

  /* 3. position and velocity check */
  vel_flag = gnss_Ls_VerticalVel_Chck(pLs);
  alt_flag = gnss_Ls_Height_Chck(pLs);

  ret_flag = (dop_flag && vel_flag && alt_flag);

  if ((!firstFix) && ret_flag && (g_pe_cfg.automobile == 0))
  {
    if (pLs->lsCtrl.init_qos_crl_cnt < 3)
    {
      pLs->lsCtrl.init_qos_crl_cnt++;
      if ((pLs->pos_res > 55) || ((pLs->lsCtrl.prNum <= 10) && pLs->lsCtrl.is2DPos))
      {
        ret_flag = FALSE;
        GLOGI("not use ls reslut: prNum=%d, pos_res=%f", pLs->lsCtrl.prNum, pLs->pos_res);
      }
    }
  }

  return ret_flag;
}

/***********************************************************************
* ��������: gnss_Ls_Cal_Acc
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�04/08/
***********************************************************************/
void gnss_Ls_Cal_Acc(Ls_t* pLs)
{
  uint8_t             betaFlag = FALSE;
  uint32_t            i;
  double            acc = 0.0;
  LS_PVT_INFO* pLsPvtInfo = NULL, * pLsPvtInfoBack = NULL;
  double            deltaHeading = 0.0, velocity = 0.0;
  double            alpha = 1.0, beta = 1.0, gamma = 0.5;
  double            refVel = 10.0;


  pLsPvtInfo = pLs->ls_Pvt_Info;
  pLsPvtInfoBack = &(pLs->ls_Pvt_Back);

  if (pLsPvtInfoBack->valid == TRUE && (pLsPvtInfo->tor - pLsPvtInfoBack->tor) < 5.0)
  {
    velocity = sqrt((double)pLsPvtInfo->enuVel[0] * pLsPvtInfo->enuVel[0] + (double)pLsPvtInfo->enuVel[1] * pLsPvtInfo->enuVel[1]);
    /* calculate acceleration */
    for (i = 0; i < 2; i++)
    {
      acc +=((double)pLsPvtInfo->enuVel[i] - pLsPvtInfoBack->enuVel[i]) * ((double)pLsPvtInfo->enuVel[i] - pLsPvtInfoBack->enuVel[i]);
    }
    acc = (float)sqrt(acc) / (pLsPvtInfo->tor - pLsPvtInfoBack->tor);

    /* calculate heading change */
    if (fabs(pLsPvtInfo->tor - pLsPvtInfoBack->tor) < 1.5 && velocity >= 2.0)
    {
      deltaHeading = (double)pLsPvtInfo->heading - pLsPvtInfoBack->heading;
      if (deltaHeading < -PI) deltaHeading += PI2;
      else if (deltaHeading > PI) deltaHeading -= PI2;
      /* determine alpha according to delta heading
      1. If heading change is smaller than pi/2 and velocity residual is smaller than 1.5, then we trust velocity estimation
      and use delta heading to enlarge Q;
      2. If heading change is larger than 3*pi /4, then shrink Q;
      */
      if (fabs(deltaHeading) < (PI / 2))
      {
        if (pLsPvtInfo->vel_res < 1.50 && pLsPvtInfoBack->vel_res < 1.50)
        {
          alpha = (1 + fabs(deltaHeading) / (PI / 2));
          betaFlag = TRUE;
        }
      }
      else
      {
        if (pLsPvtInfo->vel_res < 1.0 && pLsPvtInfoBack->vel_res < 1.0)
        {
          betaFlag = TRUE;
        }
      }
      if (betaFlag)
      {
        if (pLsPvtInfo->vel_res < 0.8 && pLsPvtInfoBack->vel_res < 0.8)
        {
          refVel = 8.0;
        }
        else
        {
          refVel = 10.0;
        }
        if (velocity < 20)
        {
          beta = (1 + (velocity / refVel) * (velocity / refVel));
          gamma = 0.5;
        }
        else
        {
          beta = (1 + (20.0 / refVel) * (20.0 / refVel));
          gamma = 0.0;
        }
      }
    }
    /* adjust acceleration according to alpha and beta */
    acc *= alpha;
    acc *= beta;

    pLs->vel_std = (float)(gamma * pLs->vel_std + (1 - gamma) * acc);
  }
  else
  {
    pLs->vel_std = 0.5;
  }
  SYS_LOGGING(OBJ_LS, LOG_INFO, "LS VEL STD:%12.6f,%10.5f,%6.2f,%6.2f,%6.2f,%6.2f,%8.2f", pLsPvtInfo->tor, pLs->vel_std, velocity,
    alpha, beta, fabs(deltaHeading) * RAD2DEG, pLsPvtInfo->heading * RAD2DEG);
}

/***********************************************************************
* ��������: gnss_Ls_Fill,used for check the LS results used for start KF
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�08/29/
***********************************************************************/
void gnss_Ls_Fill(Ls_t* pLs)
{
  uint32_t            i;
  uint8_t             flag = FALSE;
  double            acc = 0.0, baseVel = 12.0, dt;
  LS_PVT_INFO* pLsPvtInfo = NULL, * pLsPvtInfoBack = NULL;
  double            deltaHeading, velocity/*, vel_back*/;
  double            biasDiffUse[GNSS_MAX_MODE];
  meas_blk_t* pMeas;

  if (pLs == NULL)
  {
    return;
  }

  pMeas = pLs->meas;
  if (pMeas->rtdUsed)
  {
    memcpy(biasDiffUse, pLs->biasDiff, GNSS_MAX_MODE * sizeof(double));
  }
  else
  {
    memcpy(biasDiffUse, pLs->biasDiffLocal, GNSS_MAX_MODE * sizeof(double));
  }

  pLsPvtInfo = pLs->ls_Pvt_Info;
  pLsPvtInfoBack = &(pLs->ls_Pvt_Back);
  if (pLsPvtInfoBack == NULL || pLsPvtInfo == NULL)
  {
    SYS_LOGGING(OBJ_LS, LOG_INFO, "Empty pointer in %s", __FUNCTION__);
    return;
  }
  else
  {
    memcpy(pLsPvtInfoBack, pLsPvtInfo, sizeof(LS_PVT_INFO));
  }
  flag = gnss_Ls_QoS_Ctrl(pLs);
  // If pass the LS QoS check, the save the LS position fix
  if (flag)
  {
    pLsPvtInfo->valid = TRUE;
    pLsPvtInfo->fix_status = FIX_STATUS_NEW;
    pLs->lsCtrl.lsCnt++;
    pLs->lsCtrl.lsInvalidCnt = 0;
  }
  else
  {
    pLsPvtInfo->valid = FALSE;
    memset(pLsPvtInfo->biasValid, 0, BIAS_NUM * sizeof(uint8_t));
    memset(pLsPvtInfo->dcbValid, 0, 2 * BIAS_NUM * sizeof(uint8_t));
    if (pLsPvtInfo->fix_status != FIX_STATUS_NONE)
    {
      pLsPvtInfo->fix_status = FIX_STATUS_OLD;
    }
    pLs->lsCtrl.lsCnt = 0;
    if (pLs->lsCtrl.lsInvalidCnt < 60)
    {
      pLs->lsCtrl.lsInvalidCnt++;
    }
  }

  if (flag == TRUE)
  {
    for (i = 0; i < 3; i++)
    {
      // ECEF position and velocity
      pLsPvtInfo->ecefPos[i] = pLs->lsEcef.p[i];
      pLsPvtInfo->ecefVel[i] = pLs->lsEcef.v[i];

      // LLA and ENU velocity
      pLsPvtInfo->lla[i] = pLs->lsLla.lla[i];
      pLsPvtInfo->enuVel[i] = pLs->vel_enu[i];
    }

    pLsPvtInfo->tor = pLs->lsCtrl.tor;

    if (!(g_pe_cfg.meas_type_mask & MEAS_TYPE_MASK_DR) && pLsPvtInfoBack->valid)
    {
      dt = pLsPvtInfo->tor - pLsPvtInfoBack->tor;
      if (dt < -SECS_IN_WEEK / 2.0)
      {
        dt += (double)SECS_IN_WEEK;
      }
      else if (dt > SECS_IN_WEEK / 2.0)
      {
        dt -= (double)SECS_IN_WEEK;
      }
      if (fabs(dt) < 1.5 && fabs(dt) > 1e-3)
      {
        for (i = 0; i < 3; i++)
        {
          pLsPvtInfo->ecefVel[i] = (float)((pLsPvtInfo->ecefPos[i] - pLsPvtInfoBack->ecefPos[i]) / dt);
        }
        gnssConvEcef2EnuVel(pLsPvtInfo->ecefVel, pLsPvtInfo->enuVel, pLsPvtInfo->lla);
      }
    }
    velocity = sqrt((double)pLsPvtInfo->enuVel[0] * pLsPvtInfo->enuVel[0] + (double)pLsPvtInfo->enuVel[1] * pLsPvtInfo->enuVel[1]);
    pLsPvtInfo->heading = (float)atan2(pLsPvtInfo->enuVel[0], pLsPvtInfo->enuVel[1]);

    gnssConvEcef2Lla(pLsPvtInfo->ecefPos, pLsPvtInfo->LLApos);
    pLsPvtInfo->clkDrift = pLs->drift;
    if (pLs->lsCtrl.status & (LS_STATUS_HAS_2D_VEL | LS_STATUS_HAS_3D_VEL))
    {
      pLsPvtInfo->isDriftValid = TRUE;
    }
    else
    {
      pLsPvtInfo->isDriftValid = FALSE;
    }

    gnss_Pe_BiasNum(pLs->meas);
    pLsPvtInfo->isBiasLink = pLs->lsCtrl.isBiasLink;
    for (i = 0; i < BIAS_NUM; i++)
    {
      if (!pLs->lsCtrl.isBiasLink/* || i == GAL_MODE*/)
      {
        pLsPvtInfo->clkBias[i] = (float)pLs->bias[i];
        pLsPvtInfo->clkDcb[i] = (float)pLs->dcb[i];
        pLsPvtInfo->clkDcb[i + GNSS_MAX_MODE] = (float)pLs->dcb[i + GNSS_MAX_MODE];
        pLsPvtInfo->biasValid[i] = ((pLs->lsCtrl.satModeCnt) >> i) & 0x1;
        if (pLs->lsCtrl.dcbIndx[i] > 0)
        {
          pLsPvtInfo->dcbValid[i] = ((pLs->lsCtrl.satModeCnt) >> (i + GNSS_MAX_MODE)) & 0x1;
        }
        if (pLs->lsCtrl.dcbIndx[i + GNSS_MAX_MODE] > 0)
        {
          pLsPvtInfo->dcbValid[i + GNSS_MAX_MODE] = ((pLs->lsCtrl.satModeCnt) >> (i + GNSS_MAX_MODE * 2)) & 0x1;
        }
      }
      else
      {
        if (pLs->lsCtrl.validSvNum[i] > 0)
        {
          pLsPvtInfo->clkBias[i] = (float)(pLs->bias[0] - biasDiffUse[i]);
          pLsPvtInfo->biasValid[i] = (pLs->lsCtrl.satModeCnt) & 0x1;
        }
        else
        {
          pLsPvtInfo->clkBias[i] = 0.0;
          pLsPvtInfo->biasValid[i] = FALSE;
        }
      }
    }

    SYS_LOGGING(OBJ_PE, LOG_INFO, "ls_dcb:%14.6f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f", pMeas->tor,
      pLs->dcb[0], pLs->dcb[1], pLs->dcb[2], pLs->dcb[3], pLs->dcb[4], pLs->dcb[5], pLs->dcb[6], pLs->dcb[7]);

    GLOGI("freqf2cnt %d %d %d %d %d %d %d %d", pMeas->fre2_satcnt[0], pMeas->fre2_satcnt[1], pMeas->fre2_satcnt[2],
      pMeas->fre2_satcnt[3], pMeas->fre2_satcnt[4], pMeas->fre2_satcnt[5], pMeas->fre2_satcnt[6], pMeas->fre2_satcnt[7]);


    pLsPvtInfo->err = pLs->err_est;
    pLsPvtInfo->pos_res = pLs->pos_res;
    pLsPvtInfo->vel_res = pLs->vel_res;

    // calculate velocity variance 
    dt = pLsPvtInfo->tor - pLsPvtInfoBack->tor;
    if (dt < -SECS_IN_WEEK / 2.0)
    {
      dt += (double)SECS_IN_WEEK;
    }
    else if (dt > SECS_IN_WEEK / 2.0)
    {
      dt -= (double)SECS_IN_WEEK;
    }
    if (pLsPvtInfoBack->valid == TRUE && fabs(dt) < 5.0 && (pLs->lsCtrl.status & (LS_STATUS_HAS_2D_VEL | LS_STATUS_HAS_3D_VEL)))
    {
      /* calculate acceleration */
      for (i = 0; i < 2; i++)
      {
        acc += ((double)pLsPvtInfo->enuVel[i] - pLsPvtInfoBack->enuVel[i]) * ((double)pLsPvtInfo->enuVel[i] - pLsPvtInfoBack->enuVel[i]);
      }
      acc = sqrt(acc) / fabs(dt);

      /* calculate heading change */
      if (fabs(dt) < 1.5 && velocity > 2.0)
      {
        deltaHeading = (double)pLsPvtInfo->heading - pLsPvtInfoBack->heading;
        if (deltaHeading < -PI) deltaHeading += PI2;
        else if (deltaHeading > PI) deltaHeading -= PI2;
      }
      else
      {
        deltaHeading = 0.0;
      }

      acc *= (1 + fabs(deltaHeading) / PI);
#if 1
      if (g_pe_cfg.chipType == SPRD)
      {
        baseVel = 8.0;
      }
      else
      {
        baseVel = 12.0;
      }
      if (velocity < 20)
      {
        acc *= (1 + (velocity / baseVel) * (velocity / baseVel));
        if (pLs->vel_std > 0)
        {
          pLs->vel_std = (float)(0.5 * pLs->vel_std + 0.5 * acc);
        }
        else
        {
          pLs->vel_std = (float)acc;
        }
      }
      else
      {
        acc *= 4.0;
        pLs->vel_std = (float)acc;
      }
#else
      pLs->vel_std = 0.5 * pLs->vel_std + 0.5 * acc;
#endif
    }
#if 0
    else if (!(g_pe_cfg.meas_type_mask & MEAS_TYPE_MASK_DR) && pLsPvtInfoBack->valid && fabs(dt) < 5.0)
    {
      velocity = sqrt(pLsPvtInfo->enuVel[0] * pLsPvtInfo->enuVel[0] + pLsPvtInfo->enuVel[1] * pLsPvtInfo->enuVel[1] + pLsPvtInfo->enuVel[2] * pLsPvtInfo->enuVel[2]);
      vel_back = sqrt(pLsPvtInfoBack->enuVel[0] * pLsPvtInfoBack->enuVel[0] + pLsPvtInfoBack->enuVel[1] * pLsPvtInfoBack->enuVel[1] + pLsPvtInfoBack->enuVel[2] * pLsPvtInfoBack->enuVel[2]);

      if (velocity > 1e-3 && vel_back > 1e-3)
      {
        /* calculate acceleration */
        for (i = 0; i < 3; i++)
        {
          acc += (pLsPvtInfo->enuVel[i] - pLsPvtInfoBack->enuVel[i]) * (pLsPvtInfo->enuVel[i] - pLsPvtInfoBack->enuVel[i]);
        }
        acc = (float)sqrt(acc) / fabs(dt);

        if (fabs(dt) < 1.5 && velocity > 2.0)
        {
          deltaHeading = (pLsPvtInfo->heading - pLsPvtInfoBack->heading) / fabs(dt);
          if (deltaHeading < -PI) deltaHeading += PI2;
          else if (deltaHeading > PI) deltaHeading -= PI2;
        }
        else
        {
          deltaHeading = 0.0;
        }

        acc += 10 * fabs(deltaHeading) / PI;

        if (pLs->vel_std > 0)
        {
          pLs->vel_std = (float)(0.5 * pLs->vel_std + 0.5 * acc);
        }
        else
        {
          pLs->vel_std = (float)acc;
        }
      }
    }
#endif
    else
    {
      pLs->vel_std = -1.0;
    }
    SYS_LOGGING(OBJ_LS, LOG_INFO, "LS VEL STD:%12.6f,%10.5f", pLsPvtInfo->tor, pLs->vel_std);

    // fix mode set
    if (pLs->lsCtrl.is2DPos)
    {
      pLsPvtInfo->fix_dim = DIM_2D;
    }
    else
    {
      pLsPvtInfo->fix_dim = DIM_3D;
    }
  }
  else
  {
    pLs->vel_std = -1.0;
    /* reset need in open sky */
    if (pLs->lsCtrl.lsInvalidCnt >= 10 && pMeas->avgCno >= 35 && pMeas->measCnt > 20 && pMeas->cpNoLLI > 12 &&
      gpz_kf_data != NULL && gpz_kf_data->posRes > 2.0 && gpz_kf_data->kf_Pvt_Info != NULL)
    {
      kfInfo.status = KF_RESET;
      gpz_kf_data->kf_Pvt_Info->kfFixStatus = FIX_STATUS_OLD;
      pLs->lsCtrl.lsInvalidCnt = 0;//init
      SYS_LOGGING(OBJ_LS, LOG_INFO, "Reset KF for bad Ls");
    }
  }

  SYS_LOGGING(OBJ_LS, LOG_INFO, "LS result: status(%d),dim(%d),fail(%d)", pLsPvtInfo->fix_status, pLsPvtInfo->fix_dim, pLs->lsCtrl.lsInvalidCnt);
}

/***********************************************************************
* ��������: gnss_Ls_Calculate_LocalBias
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�04/03/
***********************************************************************/
void gnss_Ls_Calculate_LocalBias(Ls_t* pLs)
{
  Ls_t           lsTmp;
  meas_blk_t* pMeas;
  uint32_t            k;

  pMeas = pLs->meas;
  if (pMeas->rtdUsed || pLs->lsCtrl.isBiasLink)
  {
    memcpy(&lsTmp, pLs, sizeof(Ls_t));
    lsTmp.lsCtrl.isLocalBias = TRUE;
    lsTmp.lsCtrl.isBiasLink = FALSE;
    gnss_Ls_Pos(&lsTmp);
    /*copy valid system's bias to biasLocal*/
    for (k = 0; k < 4; k++)
    {
      if (lsTmp.lsCtrl.biasIndx[k] != 0)
      {
        pLs->biasLocal[k] = lsTmp.biasLocal[k];
      }
    }
  }
  else
  {
    /*copy valid system's bias to biasLocal*/
    for (k = 0; k < 4; k++)
    {
      if (pLs->lsCtrl.biasIndx[k] != 0)
      {
        pLs->biasLocal[k] = pLs->bias[k];
      }
    }
  }
}
/***********************************************************************
* ��������: gnss_Ls_AltHold_Check
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�04/03/
***********************************************************************/
void gnss_Ls_AltHold_Check(meas_blk_t* pMeas, Ls_t* pLs)
{
  uint8_t            cvg;
  double           distance = 0.0;
  double           deltaAlt = 0.0;
  history_Info  hisInfo;

  /* LS Position */
  cvg = gnss_Ls_Pos(pLs);

  gnss_Pe_Get_HisInfo(&hisInfo);
  /*
  Use 2D constrain: (1) height error is large; (2) There is good history height;(3) Distance is within 5Km.
  */
  if (hisInfo.towForAlt > 0.0 && fabs(hisInfo.preciseAlt) > 1e-4)
  {
    if ((pLs->lsCtrl.status & LS_STATUS_HAS_3D_POS) && pLs->pos_res > 0.0 && pLs->pos_res < 20.0 && pLs->lsCtrl.prNum >= 10)
    {
#if defined(PLAYBACK_MODE)
      SYS_LOGGING(OBJ_LS, LOG_INFO, "Use 3D in LS:%14.6f,%10.4f", pLs->lsCtrl.tor, pLs->pos_res);
#endif
    }
    else
    {
      if ((pLs->lsCtrl.status & LS_STATUS_HAS_3D_POS))
      {
        distance = gnssCalPosDis(pLs->lsLla.lla, hisInfo.llaPosForAlt, 1);
        deltaAlt = pLs->lsLla.lla[2] - hisInfo.preciseAlt;
      }

      if (((pLs->lsCtrl.status & LS_STATUS_HAS_3D_POS) && distance < 5e3 && (fabs(deltaAlt) > 50.0 || pLs->DoP.pDOP > 6.0))
        || !cvg)
      {
        pLs->lsCtrl.is2DPos = TRUE;
        pLs->lsCtrl.isForce2D = TRUE;
        pLs->lsCtrl.aidHeight = (float)hisInfo.preciseAlt;
#if defined(PLAYBACK_MODE)
        SYS_LOGGING(OBJ_LS, LOG_INFO, "Use 2D in LS:%14.6f,%10.4f", pLs->lsCtrl.tor, fabs(deltaAlt));
#endif
      }
    }
  }
}
/***********************************************************************
* ��������: gnss_Ls_2dVel_Check
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�11/29/
***********************************************************************/
void gnss_Ls_2dVel_Check(meas_blk_t* pMeas, Ls_t* pLs)
{
  uint8_t     cvg;
  Ls_t   pLsTmp;

  pLs->lsCtrl.velUpdateFlag = TRUE;
  pLsTmp = *pLs;

  if ((g_pe_cfg.meas_type_mask & MEAS_TYPE_MASK_DR) == 0)
  {
    return;
  }
  else if (pLs->lsCtrl.drNum < 3)
  {
    return;
  }

  cvg = gnss_Ls_Vel(&pLsTmp, TRUE);

  if (!cvg || pLs->lsCtrl.drNum == 3 || pLsTmp.DoP.pDOP > 6.0)
  {
    pLs->lsCtrl.is2DVel = TRUE;
    pLs->lsCtrl.aidUpVel = 0.0;

    pLsTmp = *pLs;

    cvg = gnss_Ls_Vel(&pLsTmp, TRUE);
    if (!cvg || pLsTmp.DoP.hDOP > pLs->pCfg->pdop_mask)
    {
      pLs->lsCtrl.is2DVel = FALSE;
      if (cvg)
      {
        pLs->lsCtrl.velUpdateFlag = FALSE;
      }
      SYS_LOGGING(OBJ_LS, LOG_INFO, "LS convert to 2D velocity Mode Failed");
    }
    else
    {
      SYS_LOGGING(OBJ_LS, LOG_INFO, "LS convert to 2D velocity Mode");
    }
  }
}
/***********************************************************************
* ��������:
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�11/29/
***********************************************************************/
void gnss_Ls_FixMode_Manager(meas_blk_t* pMeas, Ls_t* pLs)
{
  gnss_Ls_AltHold_Check(pMeas, pLs);
  gnss_Ls_2dVel_Check(pMeas, pLs);
}
/***********************************************************************
* ��������: gnss_Ls_Main
*           (1) Prepare the data for LS
*           (2) calculate prefix residual
*           (3) RAIM for PR and DR
*           (4) QR calculate the post and velocity
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�08/08/
***********************************************************************/
void gnss_Ls_Main(meas_blk_t* pMeas, Ls_t* pLs)
{
  uint8_t  cvg;

  // init ls pos flag
  pLs->posCvg = FALSE;
  pLs->velCvg = FALSE;

  /*0. calculate valid PR number */
  gnss_Pe_PRDR_Num(pMeas);

  /*1. Prepare data */
  gnss_Ls_Prepare(pMeas, pLs);

  /*2. LS fix mode decide */
  gnss_Ls_FixMode_Manager(pMeas, pLs);

  /*3. RAIM for PR */
  gnss_Ls_Pos_Raim(pLs);

  /*4. QR LS position */
  cvg = gnss_Ls_Pos(pLs);

  /*5. Do sat system RAIM if LS couldn't converge */
  if (cvg == FALSE)
  {
    cvg = gnss_Ls_Pos_Raim_SatSys(pLs);
  }

  /*6. RAIM for DR */
  gnss_Ls_Vel_Raim(pLs);

  /*7. QR LS velocity */
  gnss_Ls_Vel(pLs, FALSE);

  /*8. Prepare for KF */
  gnss_Ls_Fill(pLs);

  /*9. LS Filter */
  gnss_Ls_Filter(pLs);

  /*10. Calculate local bias without RTD */
  if (cvg == TRUE)
  {
    gnss_Ls_Calculate_LocalBias(pLs);
  }
}
