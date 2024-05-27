
#include "macro.h"
#include "gnss.h"
#include "gnss_sd.h"
#include "gnss_sd_nm.h"
#include "gnss_sys_api.h"
#include "gnss_tm.h"
#include "gnss_sd_pos.h"
#include "gnss_pe.h"
#include "gnss_rtk.h"
#include "gnss_math.h"

#undef MODULE_TAG
#define MODULE_TAG OBJ_SD

#define abs(x) ((x) < 0 ? (-(x)) : (x))  //return abs
#define BDS_ID_IS_GEO(x)         (((x)>0&&x<=5)||((x)>=59&&(x)<=63))

static gnss_nm_t    nm_data;
gnss_nm_t* p_Nm = &nm_data;

extern uint8_t firstFix;


void gnss_sd_nm_init_glo_chn(void)
{
  memset(p_Nm->gloChannel, 127, sizeof(p_Nm->gloChannel));
}


/**********************************************************************
* Function Name:    gnss_sv_Idx
*
* Description:
*    gnss_sv_Idx() is used for sv idx calculation when EPH processing
*
* Input:
*       satMode:
*       prn:
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
int32_t gnss_sv_Idx(uint32_t gnssMode, uint32_t prn)
{
  int32_t      idx = -1;

  if (gnssMode == GPS_MODE)
  {
    if (prn > 0 && prn <= N_GPS_SVS)
    {
      idx = prn - 1;
    }
    else if (prn >= MIN_WAAS_PRN && prn <= MAX_WAAS_PRN)
    {
      idx = N_GPS_SVS + prn - MIN_WAAS_PRN;
    }
    else if (prn >= MIN_QZSS_PRN && prn <= MAX_QZSS_PRN)
    {
      idx = N_GPS_SVS + N_WAAS_SVS + prn - MIN_QZSS_PRN;
    }
  }
  else if (gnssMode == GLN_MODE)
  {
    idx = N_GPS_SVS + N_WAAS_SVS + N_QZSS_SVS + prn - 1;
  }
  else if (gnssMode == BDS_MODE)
  {
    idx = N_GPS_SVS + N_WAAS_SVS + N_QZSS_SVS + N_GLN_SVS + prn - 1;
  }
  else if (gnssMode == GAL_MODE)
  {
    idx = N_GPS_SVS + N_WAAS_SVS + N_QZSS_SVS + N_GLN_SVS + N_BDS_SVS + prn - 1;
  }

  return idx;
}
/**********************************************************************
* Function Name:    gnss_Idx_sv
*
* Description:
*    gnss_Idx_sv() is used for satmode and prn calculation when EPH processing
*
* Input:idx
*
* Return:
*		satMode:
*       prn:
* Dependency
*      None
*
* Author: 
**********************************************************************/
int32_t gnss_Idx_sv(int32_t idx, int8_t* gnssMode)
{
  int32_t prn = -1;

  //GPS
  if (idx >= 0 && idx < N_GPS_SVS)
  {
    prn = idx + 1;
    *gnssMode = GPS_MODE;
  }
  //WAAS
  else if (idx >= N_GPS_SVS && idx < N_GPS_SVS + N_WAAS_SVS)
  {
    prn = idx - N_GPS_SVS + MIN_WAAS_PRN;
    *gnssMode = GPS_MODE;
  }
  //QZSS
  else if (idx >= N_GPS_SVS + N_WAAS_SVS && idx < N_GPS_SVS + N_WAAS_SVS + N_QZSS_SVS)
  {
    prn = idx - N_GPS_SVS - N_WAAS_SVS + MIN_QZSS_PRN;
    *gnssMode = GPS_MODE;
  }
  //GLN
  else if (idx >= N_GPS_SVS + N_WAAS_SVS + N_QZSS_SVS && idx < N_GPS_SVS + N_WAAS_SVS + N_QZSS_SVS + N_GLN_SVS)
  {
    prn = idx - N_GPS_SVS - N_WAAS_SVS - N_QZSS_SVS + 1;
    *gnssMode = GLN_MODE;
  }
  //BDS
  else if (idx >= N_GPS_SVS + N_WAAS_SVS + N_QZSS_SVS + N_GLN_SVS && idx < N_GPS_SVS + N_WAAS_SVS + N_QZSS_SVS + N_GLN_SVS + N_BDS_SVS)
  {
    prn = idx - N_GPS_SVS - N_WAAS_SVS - N_QZSS_SVS - N_GLN_SVS + 1;
    *gnssMode = BDS_MODE;
  }
  //GAL
  else if (idx >= N_GPS_SVS + N_WAAS_SVS + N_QZSS_SVS + N_GLN_SVS + N_BDS_SVS && idx < N_GPS_SVS + N_WAAS_SVS + N_QZSS_SVS + N_GLN_SVS + N_BDS_SVS + N_GAL_SVS)
  {
    prn = idx - N_GPS_SVS - N_WAAS_SVS - N_QZSS_SVS - N_GLN_SVS - N_BDS_SVS + 1;
    *gnssMode = GAL_MODE;
  }

  return prn;

}
/***********************************************************************
* ��������: gnss_Sd_Nm_Del. This funciton should be called when stopped
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/22/
***********************************************************************/
void gnss_Sd_Nm_Del(void)
{
  uint32_t         i;

  for (i = 0; i < MAX_PRN_ALL_MODE; i++)
  {
    if (p_Nm->ephData[i])
    {
      Sys_Free(p_Nm->ephData[i]);
      p_Nm->ephData[i] = NULL;
    }
    if (p_Nm->ephDataBack[i])
    {
      Sys_Free(p_Nm->ephDataBack[i]);
      p_Nm->ephDataBack[i] = NULL;
    }
    if (p_Nm->almData[i])
    {
      Sys_Free(p_Nm->almData[i]);
      p_Nm->almData[i] = NULL;
    }
  }

  return;
}

void gnss_Sd_Nm_SaveNT(uint8_t eph_source, uint8_t gnssMode, uint16_t NT, uint16_t N4, GNSS_TIME* pTime)
{
  if (gnssMode != GLN_MODE || NT == 0 || N4 == 0)
  {
    return;
  }

  pTime->N4_save = N4;
  pTime->NT_save = NT;

  switch (eph_source)   // record the week update source
  {
  case EPH_SOURCE_GROM:
    pTime->timeInitSrc = TM_INIT_GROM;
    pTime->init = FALSE;
    break;
  case EPH_SOURCE_AGNSS:
    pTime->timeInitSrc = TM_INIT_AGNSS; break;
  case EPH_SOURCE_BRDC:
    pTime->timeInitSrc = TM_INIT_BRDC;  break;
  case EPH_SOURCE_EE:
    pTime->timeInitSrc = TM_INIT_EE;    break;
  case EPH_SOURCE_AGNSS_L:
    pTime->timeInitSrc = TM_INIT_AGNSS_L;    break;
  default:
    pTime->timeInitSrc = TM_INIT_NONE; break;
  }

  if (pTime->init == TRUE && (pTime->NT != NT || pTime->N4 != N4))
  {
    pTime->init = FALSE;
  }
}

void gnss_Sd_Nm_SaveWeek(uint8_t eph_source, uint8_t gnssMode, uint16_t weeknum, GNSS_TIME* pTime)
{
  //pTime->week_save[gnssMode] = weeknum;

  switch (eph_source)   // record the week update source
  {
  case EPH_SOURCE_GROM:
    pTime->timeInitSrc = TM_INIT_GROM;
    pTime->init = FALSE;
    break;
  case EPH_SOURCE_AGNSS:
    pTime->timeInitSrc = TM_INIT_AGNSS; break;
  case EPH_SOURCE_BRDC:
    pTime->timeInitSrc = TM_INIT_BRDC;  break;
  case EPH_SOURCE_EE:
    pTime->timeInitSrc = TM_INIT_EE;    break;
  case EPH_SOURCE_AGNSS_L:
    pTime->timeInitSrc = TM_INIT_AGNSS_L;    break;
  default:
    pTime->timeInitSrc = TM_INIT_NONE; break;
  }

  if (pTime->init == TRUE)
  {
    pTime->week_save[gnssMode] = weeknum;

    if (pTime->week[gnssMode] == weeknum)
    {
      pTime->weekCheckNum[gnssMode]++;
      if (pTime->weekCheckNum[gnssMode] > 1000) pTime->weekCheckNum[gnssMode] = 1000;
    }
    else
    {
      if (pTime->weekCheckNum[gnssMode] >= 1)
      {
        return;
      }
      else if (p_Nm->week_decoded[gnssMode] > 0 && (p_Nm->week_decoded[gnssMode] == weeknum))
      {
        pTime->init = FALSE;
        GLOGI("Week number check fail and update!");
      }
    }

    if (pTime->timeInitSrc != TM_INIT_AGNSS_L)
    {
      p_Nm->week_decoded[gnssMode] = weeknum;
    }
  }
  else
  {
    if (pTime->week_save[gnssMode] == 0)
    {
      pTime->week_save[gnssMode] = weeknum;
    }
    else if (pTime->week_save[gnssMode] == weeknum)
    {
      pTime->weekCheckNum[gnssMode]++;
      if (pTime->weekCheckNum[gnssMode] > 1000) pTime->weekCheckNum[gnssMode] = 1000;
    }
    else if(pTime->weekCheckNum[gnssMode] < 5)
    {
      pTime->week_save[gnssMode] = weeknum;
      pTime->weekCheckNum[gnssMode] = 0;
    }

#if 0
    if (pTime->week_save[gnssMode] == 0 || eph_source == EPH_SOURCE_AGNSS_L)
    {
      pTime->week_save[gnssMode] = weeknum;
    }
    else if (pTime->week_save[gnssMode] == weeknum)
    {
      pTime->weekCheckNum[gnssMode]++;
    }
    else
    {
      if (pTime->weekCheckNum[gnssMode] >= 1)
      {
        return;
      }
      else if (p_Nm->week_decoded[gnssMode] > 0 && (p_Nm->week_decoded[gnssMode] == weeknum))
      {
        pTime->week_save[gnssMode] = weeknum;
        pTime->weekCheckNum[gnssMode] = 1;
      }
    }
    if (pTime->timeInitSrc != TM_INIT_AGNSS_L)
    {
      p_Nm->week_decoded[gnssMode] = weeknum;
    }
#endif
  }
}
/***********************************************************************
* ��������: gps_eph_valid_check
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/22/
***********************************************************************/
uint8_t gps_eph_valid_check(GPS_EPH_INFO* pEph, uint8_t prn)
{
  if (prn > 0 && prn <= N_GPS_SVS) //GPS
  {
    if ((pEph->t_oe > 604784 || 
         pEph->t_oe < 0) || 
       (pEph->e > 0.1 || pEph->e < 0) || 
       (pEph->sqrt_A < 1000 || pEph->sqrt_A > 10000) || 
       (pEph->OMEGADOT < -6.33e-6 || pEph->OMEGADOT > 0))
    {
      GLOGE("Invalid GPS EPH,prn=%u. toe=%.1f,e=%.3e,sqrtA=%.1f,OMEGADOT=%.6e",
        prn, pEph->t_oe, pEph->e, pEph->sqrt_A, pEph->OMEGADOT);
      return FALSE;
    }
  }
  else if (prn >= MIN_QZSS_PRN && prn <= MAX_QZSS_PRN) //QZSS
  {
    if (pEph->t_oe > 604800.0 || pEph->t_oe < 0)
    {
      GLOGE("Invalid QZS EPH,prn=%u,toe-%.1f", prn, pEph->t_oe);
      return FALSE;
    }
  }
  else if (prn >= MIN_WAAS_PRN && prn <= MAX_WAAS_PRN) //WAAS
  {
    if (pEph->t_oe > 604800.0 || pEph->t_oe < 0)
    {
      GLOGE("Invalid WAAS EPH,prn=%u,toe-%.1f", prn, pEph->t_oe);
      return FALSE;
    }
  }
  else
  {
    return FALSE;
  }

  return TRUE;
}
/***********************************************************************
* ��������: gln_eph_valid_check
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/22/
***********************************************************************/
uint8_t gln_eph_valid_check(GLN_EPH_INFO* pEph)
{
  uint8_t  i;

  for (i = 0; i < 3; i++)
  {
    if (fabs(pEph->r[i]) < 1e-4 || fabs(pEph->v[i]) < 1e-4 || fabs(pEph->r[i]) > 6e7 || fabs(pEph->v[i]) > 8e3 || fabs(pEph->a[i]) > 6.2e-5)
    {
      SYS_LOGGING(OBJ_SD, LOG_WARNING, "Invalid GLN EPH. %s ", __FUNCTION__);
      return FALSE;
    }
  }

  if (pEph->NT > 2048 || (pEph->tb < 90 || pEph->tb>855000) || \
    fabs(pEph->gaman_tb) > pow(2.0, -30) || fabs(pEph->taun_tb) > pow(2.0, -9) || fabs(pEph->delta_taun) > 26e-9)
  {
    SYS_LOGGING(OBJ_SD, LOG_WARNING, "Invalid GLN EPH. %s ", __FUNCTION__);
    return FALSE;
  }

  return TRUE;
}
/***********************************************************************
* ��������: bds_eph_valid_check
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/22/
***********************************************************************/
uint8_t bds_eph_valid_check(BDS_EPH_INFO* pEph)
{
  if (pEph->toe > 604500 || pEph->sqrta < 1e-4)
  {
    SYS_LOGGING(OBJ_SD, LOG_WARNING, "Invalid BDS EPH. %s ", __FUNCTION__);
    return FALSE;
  }

  return TRUE;
}
/***********************************************************************
* ��������: gal_eph_valid_check
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/22/
***********************************************************************/
uint8_t gal_eph_valid_check(GAL_EPH_INFO* pEph)
{
  return TRUE;
}

/***********************************************************************
* ��������: gnss_Sd_Nm_AddEph
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/22/
***********************************************************************/
uint8_t gnss_Sd_Nm_AddEph(uint8_t gnssMode, uint8_t sv_id, void* src)
{
  uint32_t    reverse = IS_PPK_REVERSE;
  int32_t     idx;
  double      dt;
  double      dto = 0.0;
  GNSS_TIME*  pTime;
  uint16_t	  weeknum;

  if (NULL == src) 
  {
    GLOGE("gnss_Sd_Nm_AddEph inputs check nullptr");
    return FALSE;
  }

  //*************
  idx = gnss_sv_Idx(gnssMode, sv_id);
  if (idx < 0) 
  {
    return FALSE;
  }

  pTime = gnss_tm_get_time();

  if (gnssMode == GPS_MODE)
  {
    GPS_EPH_INFO* gps_eph_n = (GPS_EPH_INFO*)src;
    GPS_EPH_INFO* gps_eph_o = (GPS_EPH_INFO*)p_Nm->ephData[idx];

    if (fabs((double)gps_eph_n->t_oe - gps_eph_n->subframe1.t_oc) > 0.5)
    {
      GLOGE("gps eph svid %d reject, toe!=toc, toe=%.3f, toc=%.3f",
        sv_id,
        gps_eph_n->t_oe,
        gps_eph_n->subframe1.t_oc);
      return FALSE;
    }

    if (FALSE == gps_eph_valid_check(gps_eph_n, sv_id))
    {
      GLOGE("gps eph svid %d reject, valid check fail", sv_id);
      return FALSE;
    }

    if (fabs(gps_eph_n->t_oe - pTime->tor) > 3600 * 4.0 && pTime->tor > 1e-3 &&
      fabs((double)gps_eph_n->t_oe + (int64_t)SECS_IN_WEEK - pTime->tor) > 3600 * 4.0 &&
      fabs(pTime->tor + (int64_t)SECS_IN_WEEK - gps_eph_n->t_oe) > 3600 * 4.0)
    {
      GLOGW("add gps eph: toe too old, toe=%.3f, tor=%.3f,prn=%d", gps_eph_n->t_oe, pTime->tor, sv_id);
      return FALSE;
    }

    //check eph health
    if (gps_eph_n->subframe1.SV_health == 1) {
      gps_eph_n->eph_status = EPH_STATUS_INVALID;
    }

    // Set the week number,but there is no para week in AGNSSL eph
    if (gps_eph_n->eph_source != EPH_SOURCE_AGNSS_L)
    {
      if (gps_eph_n->subframe1.weeknum != 0 && !(sv_id >= MIN_QZSS_PRN && sv_id <= MAX_QZSS_PRN))
      {
        //if (gps_eph_n->subframe1.weeknum > 512 && gps_eph_n->subframe1.weeknum < 1024)gps_eph_n->subframe1.weeknum += 1024;
        //else if (gps_eph_n->subframe1.weeknum < 512) gps_eph_n->subframe1.weeknum += 2048;
        if (gps_eph_n->subframe1.weeknum < 1024)
        {
          gps_eph_n->subframe1.weeknum += 2048;
        }

        weeknum = gps_eph_n->subframe1.weeknum;
        if (pTime->torStatus[GPS_MODE] == TRUE && 
           pTime->time_ns > 0 && 
           pTime->time_ns / 1E9 < (int64_t)SECS_IN_WEEK &&
          (gps_eph_n->t_oe - pTime->time_ns / 1E9) < -302400.0 && 
          fabs(gps_eph_n->t_oe - pTime->time_ns / 1E9 + (int64_t)SECS_IN_WEEK) < (7200 + 600.0))
        {
          weeknum = gps_eph_n->subframe1.weeknum - 1;
        }

        gnss_Sd_Nm_SaveWeek(gps_eph_n->eph_source, GPS_MODE, weeknum, pTime);
      }
    }
    dto = 0.0;
    if (NULL != p_Nm->ephData[idx] && pTime->tor > 1e-3)
    {
      dto = gps_eph_o->t_oe - pTime->tor;
      if (dto < -SECS_IN_WEEK / 2.0) dto += (int64_t)SECS_IN_WEEK;
      else if (dto > SECS_IN_WEEK / 2.0) dto -= (int64_t)SECS_IN_WEEK;
    }
    //1) If there is no EPH, then just copy the new EPH
    if (p_Nm->ephData[idx] == NULL) 
    {
      if ((p_Nm->ephData[idx] = Sys_Malloc(sizeof(GPS_EPH_INFO))) == NULL) 
      {
        GLOGE("Memory Alloc Fail %s %d", __FUNCTION__, __LINE__);
        return FALSE;
      }
      memcpy(p_Nm->ephData[idx], src, sizeof(GPS_EPH_INFO));
      GLOGI("GPS_EPH updated,prn=%3u,week=%d,sec=%d",
        (unsigned)sv_id, (int)gps_eph_n->subframe1.weeknum, (uint32_t)gps_eph_n->t_oe);
    }
    else if (dto > 3600 * 2.5)
    {
      memcpy(p_Nm->ephData[idx], src, sizeof(GPS_EPH_INFO));
      GLOGW("The latest GPS eph time is wrong:toe=%.3f, tor=%.3f,prn=%d", gps_eph_o->t_oe, pTime->tor, sv_id);
    }
    else {
      //first check whether the eph is a new one
      dt = ((double)gps_eph_n->t_oe - gps_eph_o->t_oe);
      if (gps_eph_n->subframe1.weeknum != gps_eph_o->subframe1.weeknum)
      {
        dt += ((double)gps_eph_n->subframe1.weeknum - gps_eph_o->subframe1.weeknum) * (double)SECS_IN_WEEK;
      }
      else if (dt < -SECS_IN_WEEK / 2.0 && gps_eph_n->t_oe <= 7200 && gps_eph_n->eph_source == EPH_SOURCE_AGNSS_L)
      {
        dt += (double)SECS_IN_WEEK;
      }

      //check whether the decoded eph is valid,compare with the nearest one
      if (fabs(dt) < 7200 && gps_eph_n->eph_source == EPH_SOURCE_BRDC) {
        GPS_EPH_INFO          eph_new, eph_old;
        ECEF                  svp_new, svp_old;
        double                   dClk_new, dClk_old;
        double                   dopp_new, dopp_old;

        //init
        memset(&svp_new, 0, sizeof(ECEF));
        memset(&svp_old, 0, sizeof(ECEF));
        memcpy(&eph_new, gps_eph_n, sizeof(GPS_EPH_INFO));
        memcpy(&eph_old, gps_eph_o, sizeof(GPS_EPH_INFO));

        if (TRUE == gps_sd_satpos_e(&eph_new, eph_new.t_oe, sv_id, &svp_new, &dClk_new, &dopp_new, 0) &&
          TRUE == gps_sd_satpos_e(&eph_old, eph_new.t_oe, sv_id, &svp_old, &dClk_old, &dopp_old, 0))
        {
          double diff;
          diff = fabs(svp_new.p[0] - svp_old.p[0]) + fabs(svp_new.p[1] - svp_old.p[1]) + fabs(svp_new.p[2] - svp_old.p[2]);
          if (diff > 100.0) {
            GLOGE("sv pos from different ephs(gps) compared failed! pos_diff=%f, %s %d",
              diff, __FUNCTION__, __LINE__);
            goto label_end_addeph;
          }
        }
      }

      //2)time compare, the new one is old ,then compare with the backup one
      if ((!reverse && dt < -0.1) || (reverse && dt > 0.1)) {
        //check the backup eph's existence
        if (p_Nm->ephDataBack[idx] == NULL) {
          if ((p_Nm->ephDataBack[idx] = Sys_Malloc(sizeof(GPS_EPH_INFO))) == NULL) {
            SYS_LOGGING(OBJ_SD, LOG_ERROR, "Memory Alloc Fail %s %d", __FUNCTION__, __LINE__);
            return FALSE;
          }

          //add into the backup one;
          memset(p_Nm->ephDataBack[idx], 0, sizeof(GPS_EPH_INFO));
          memcpy(p_Nm->ephDataBack[idx], gps_eph_n, sizeof(GPS_EPH_INFO));
          SYS_LOGGING(OBJ_SD, LOG_INFO, "GPS_EPH BACKUP updated,prn=%u,week=%d,sec=%f",
            (unsigned)sv_id, (int)gps_eph_n->subframe1.weeknum, gps_eph_n->t_oe);
        }
        else {
          GPS_EPH_INFO* gps_eph_b = NULL;

          gps_eph_b = (GPS_EPH_INFO*)p_Nm->ephDataBack[idx];
          //compare the new one with the backup one
          dt = ((double)gps_eph_n->t_oe - gps_eph_b->t_oe);
          //week leap
          if (dt < -SECS_IN_WEEK / (2.0)) {
            dt = dt + (double)SECS_IN_WEEK;
          }
          else if (dt > SECS_IN_WEEK / (2.0)) {
            dt = dt - (double)SECS_IN_WEEK;
          }

          if ((!reverse && dt < 0.01) || (reverse && dt > -0.01)) {
            SYS_LOGGING(OBJ_SD, LOG_INFO, "GPS_EPH not updated,prn=%u,week=%d,sec=%f,dt=%f",
              (unsigned)sv_id, (int)gps_eph_n->subframe1.weeknum, gps_eph_n->t_oe, dt);
            goto label_end_addeph;
          }

          //exchange the backup one to new one
          memset(p_Nm->ephDataBack[idx], 0, sizeof(GPS_EPH_INFO));
          memcpy(p_Nm->ephDataBack[idx], gps_eph_n, sizeof(GPS_EPH_INFO));
          SYS_LOGGING(OBJ_SD, LOG_INFO, "GPS_EPH BACKUP updated,prn=%u,week=%d,sec=%f",
            (unsigned)sv_id, (int)gps_eph_n->subframe1.weeknum, gps_eph_n->t_oe);
        }
      }
      else if ((!reverse && dt > 0.1) || (reverse && dt < -0.1)) {
        //3)the new one is the newest one,then update
        //check the backup eph's existence
        if (p_Nm->ephDataBack[idx] == NULL) {
          if ((p_Nm->ephDataBack[idx] = Sys_Malloc(sizeof(GPS_EPH_INFO))) == NULL) {
            SYS_LOGGING(OBJ_SD, LOG_ERROR, "Memory Alloc Fail %s %d", __FUNCTION__, __LINE__);
            return FALSE;
          }
        }

        //copy the current to old one
        memcpy(p_Nm->ephDataBack[idx], p_Nm->ephData[idx], sizeof(GPS_EPH_INFO));
        //copy the new one to current one;
        memcpy(p_Nm->ephData[idx], gps_eph_n, sizeof(GPS_EPH_INFO));

        SYS_LOGGING(OBJ_SD, LOG_INFO, "GPS_EPH updated,prn=%3u,week=%d,sec=%d",
        (unsigned)sv_id, (int)gps_eph_n->subframe1.weeknum, (uint32_t)gps_eph_n->t_oe);
      }
      else {
        if (gps_eph_n->eph_source != EPH_SOURCE_BRDC) {
          //copy the new one to current one;
          memcpy(p_Nm->ephData[idx], gps_eph_n, sizeof(GPS_EPH_INFO));
          SYS_LOGGING(OBJ_SD, LOG_INFO, "GPS_EPH updated,prn=%3u,week=%d,sec=%d",
            (unsigned)sv_id, (int)gps_eph_n->subframe1.weeknum, (uint32_t)gps_eph_n->t_oe);
        }

        goto label_end_addeph;
      }
    }
  }
  else if (gnssMode == GLN_MODE) {
    GLN_EPH_INFO* glo_eph_n, * glo_eph_o;
    glo_eph_n = (GLN_EPH_INFO*)src;
    glo_eph_o = (GLN_EPH_INFO*)p_Nm->ephData[idx];

    if (!gln_eph_valid_check(glo_eph_n))
    {
      return FALSE;
    }
    //check eph health
    if (glo_eph_n->Health == 1)
      glo_eph_n->eph_status = EPH_STATUS_INVALID;

    if (glo_eph_n->eph_source != EPH_SOURCE_AGNSS_L)
    {
      gnss_Sd_Nm_SaveNT(glo_eph_n->eph_source, GLN_MODE, glo_eph_n->NT, glo_eph_n->N4, pTime);
    }
    // If there is no EPH, then just copy the new EPH
    if (p_Nm->ephData[idx] == NULL) {
      if ((p_Nm->ephData[idx] = Sys_Malloc(sizeof(GLN_EPH_INFO))) == NULL) {
        SYS_LOGGING(OBJ_SD, LOG_ERROR, "Memory Alloc Fail %s %d", __FUNCTION__, __LINE__);
        return FALSE;
      }
      memcpy(p_Nm->ephData[idx], src, sizeof(GLN_EPH_INFO));

      SYS_LOGGING(OBJ_SD, LOG_INFO, "GLN_EPH updated,prn=%u,NT=%u,tb=%f",
        (unsigned)sv_id, (unsigned)glo_eph_n->NT, glo_eph_n->tb);

    }
    else {
      dt = (glo_eph_n->NT - glo_eph_o->NT) * SECS_IN_DAY + (glo_eph_n->tb - glo_eph_o->tb);
      //year leap
      if (dt < -800 * SECS_IN_DAY)     dt = dt + 1461 * SECS_IN_DAY;
      else if (dt > 800 * SECS_IN_DAY) dt = dt - 1461 * SECS_IN_DAY;

      if (fabs(dt) < 1800 && fabs(dt) > 2 && glo_eph_n->eph_source == EPH_SOURCE_BRDC) {
        GLN_EPH_INFO     eph_old;
        ECEF             svp_old;
        double              dClk_old;
        double              dopp_old;

        //init
        memset(&svp_old, 0, sizeof(ECEF));
        memcpy(&eph_old, glo_eph_o, sizeof(GLN_EPH_INFO));
        if (TRUE == gln_sd_sv_pos_e(0, &eph_old, glo_eph_n->tb, &svp_old, &dClk_old, &dopp_old))
        {
          double diff;
          diff = fabs(svp_old.p[0] - glo_eph_n->r[0]) + fabs(svp_old.p[1] - glo_eph_n->r[1]) + fabs(svp_old.p[2] - glo_eph_n->r[2]);
          if (diff > 200) {
            GLOGE("sv pos from different ephs(GLN) compared failed! pos_diff=%f, %s %d",
              diff, __FUNCTION__, __LINE__);
            goto label_end_addeph;
          }
        }
      }

      //at first, should check the consistency of data source
      if (glo_eph_n->eph_source == EPH_SOURCE_BRDC && glo_eph_o->eph_source == EPH_SOURCE_BRDC) {
        dt = (glo_eph_n->NT - glo_eph_o->NT) * SECS_IN_DAY + (glo_eph_n->tb - glo_eph_o->tb);
        //year leap
        if (dt < -800 * SECS_IN_DAY)     dt = dt + 1461 * SECS_IN_DAY;
        else if (dt > 800 * SECS_IN_DAY) dt = dt - 1461 * SECS_IN_DAY;

        //time compare
        if ((!reverse && dt < 0.01) || (reverse && dt > -0.01))
          goto label_end_addeph;

        //update date
        //check the old eph's existence
        if (p_Nm->ephDataBack[idx] == NULL) {
          if ((p_Nm->ephDataBack[idx] = Sys_Malloc(sizeof(GLN_EPH_INFO))) == NULL) {
            SYS_LOGGING(OBJ_SD, LOG_ERROR, "Memory Alloc Fail %s %d", __FUNCTION__, __LINE__);
            return FALSE;
          }
        }

        //copy the current to old one
        memcpy(p_Nm->ephDataBack[idx], p_Nm->ephData[idx], sizeof(GLN_EPH_INFO));
        //copy the new one to current one;
        memcpy(p_Nm->ephData[idx], glo_eph_n, sizeof(GLN_EPH_INFO));

        SYS_LOGGING(OBJ_SD, LOG_INFO, "GLN_EPH updated,prn=%u,NT=%u,tb=%f",
          (unsigned)sv_id, (unsigned)glo_eph_n->NT, glo_eph_n->tb);
      }
      else {

        dt = (glo_eph_n->NT - glo_eph_o->NT) * SECS_IN_DAY + (glo_eph_n->tb - glo_eph_o->tb);
        //year leap
        if (dt < -800 * SECS_IN_DAY)     dt = dt + 1461 * SECS_IN_DAY;
        else if (dt > 800 * SECS_IN_DAY) dt = dt - 1461 * SECS_IN_DAY;

        //for lack of para NT in a-gnss data source, should check the valid of the old eph data
        //presuming the updating data source is the newest one, any exist eph with abs(tb_curr - tb_new)>=2eps should be abandoned.
        //for the old eph with 1 eps should check the consistency of sate pos.
        //firstly, if tb is equal, cover the one and delete the old one
        //if time is equal
        if (abs(dt) < 0.01) {
          double dDis;

          //the pos check
          dDis = abs(glo_eph_n->r[0] - glo_eph_o->r[0]) +
            abs(glo_eph_n->r[1] - glo_eph_o->r[1]) +
            abs(glo_eph_n->r[2] - glo_eph_o->r[2]);
          if (dDis < 10) {
            SYS_LOGGING(OBJ_SD, LOG_INFO, "GLN_EPH not updated,prn=%u,NT=%u,tb=%f, dis=%.3f",
              (unsigned)sv_id, (unsigned)glo_eph_n->NT, glo_eph_n->tb, dDis);
            goto label_end_addeph;
          }

          if (glo_eph_n->eph_source != EPH_SOURCE_BRDC) {
            //the  curr eph is overdue, should delete and replace
            memcpy(p_Nm->ephData[idx], src, sizeof(GLN_EPH_INFO));

            SYS_LOGGING(OBJ_SD, LOG_INFO, "GLN_EPH updated,prn=%u,NT=%u,tb=%f",
              (unsigned)sv_id, (unsigned)glo_eph_n->NT, glo_eph_n->tb);

            //check backup eph's existence and delete
            if (p_Nm->ephDataBack[idx]) {
              Sys_Free(p_Nm->ephDataBack[idx]);
              p_Nm->ephDataBack[idx] = NULL;
            }
          }

        }
        else if (abs(dt) > 0.01 && abs(dt) < (30 * 60.0 + 0.1)) {
          ECEF svpos;
          double  dClk;
          double  dDis;

          //secondly, if new eph is overdue 1 eps,check the sate pos by the curr one
          if (gln_sd_sv_pos_e(0, glo_eph_o, glo_eph_n->tb, &svpos, &dClk, NULL) == FALSE) {
            //replace the curr one and delete the backup one
            memcpy(p_Nm->ephData[idx], src, sizeof(GLN_EPH_INFO));

            SYS_LOGGING(OBJ_SD, LOG_INFO, "GLN_EPH updated,prn=%u,NT=%u,tb=%f",
              (unsigned)sv_id, (unsigned)glo_eph_n->NT, glo_eph_n->tb);

            //check backup eph's existence and delete
            if (p_Nm->ephDataBack[idx]) {
              Sys_Free(p_Nm->ephDataBack[idx]);
              p_Nm->ephDataBack[idx] = NULL;
            }

            goto label_end_addeph;
          }

          //check consistency of sate pos
          dDis = abs(svpos.p[0] - glo_eph_n->r[0]) +
            abs(svpos.p[1] - glo_eph_n->r[1]) +
            abs(svpos.p[2] - glo_eph_n->r[2]);
          if (dDis > 200) {
            if (glo_eph_n->eph_source != EPH_SOURCE_BRDC) {
              //replace the curr one and delete the backup one
              memcpy(p_Nm->ephData[idx], src, sizeof(GLN_EPH_INFO));

              SYS_LOGGING(OBJ_SD, LOG_INFO, "GLN_EPH updated,prn=%d,NT=%d,tb=%f",
                (unsigned)sv_id, (unsigned)glo_eph_n->NT, glo_eph_n->tb);
              //check backup eph's existence and delete
              if (p_Nm->ephDataBack[idx]) {
                Sys_Free(p_Nm->ephDataBack[idx]);
                p_Nm->ephDataBack[idx] = NULL;
              }
            }

            goto label_end_addeph;
          }
          else {

            //if the new one is overdue
            if ((!reverse && dt > 0) || (reverse && dt < 0)) {
              //check the backup one's existence and replace the curr one to it
              if (p_Nm->ephDataBack[idx] == NULL) {
                if ((p_Nm->ephDataBack[idx] = Sys_Malloc(sizeof(GLN_EPH_INFO))) == NULL) {
                  SYS_LOGGING(OBJ_SD, LOG_ERROR, "Memory Alloc Fail %s %d", __FUNCTION__, __LINE__);
                  return FALSE;
                }
              }

              //copy the current to backup one
              memcpy(p_Nm->ephDataBack[idx], p_Nm->ephData[idx], sizeof(GLN_EPH_INFO));
              //copy the new one to current one;
              memcpy(p_Nm->ephData[idx], glo_eph_n, sizeof(GLN_EPH_INFO));

              SYS_LOGGING(OBJ_SD, LOG_INFO, "GLN_EPH updated,prn=%3u,NT=%u,tb=%f",
                (unsigned)sv_id, (unsigned)glo_eph_n->NT, glo_eph_n->tb);

              goto label_end_addeph;
            }
            else {
              //if the new one is lag behind
              goto label_end_addeph;
            }
          }

        }
        else {
          //the curr one is too old or too new
          //delete the backup and curr one,update the new one
          if (p_Nm->ephDataBack[idx]) {
            Sys_Free(p_Nm->ephDataBack[idx]);
            p_Nm->ephDataBack[idx] = NULL;
          }

          memcpy(p_Nm->ephData[idx], src, sizeof(GLN_EPH_INFO));

          SYS_LOGGING(OBJ_SD, LOG_INFO, "GLN_EPH updated,prn=%u,NT=%u,tb=%f",
            (unsigned)sv_id, (unsigned)glo_eph_n->NT, glo_eph_n->tb);
        }
      }
    }
  }
  else if (gnssMode == BDS_MODE) {
    BDS_EPH_INFO* bds_eph_n, * bds_eph_o;
    bds_eph_n = (BDS_EPH_INFO*)src;
    bds_eph_o = (BDS_EPH_INFO*)p_Nm->ephData[idx];

    if (!bds_eph_valid_check(bds_eph_n))
    {
      return FALSE;
    }

    if (fabs(bds_eph_n->toe - pTime->tor) > 3600 * 2.0 && pTime->tor > 1e-3 &&
      fabs(pTime->tor + (int64_t)SECS_IN_WEEK - bds_eph_n->toe) > 3600 * 2.0 &&
      fabs(bds_eph_n->toe + (int64_t)SECS_IN_WEEK - pTime->tor) > 3600 * 2.0)
    {
      GLOGW("add bds eph: toe too old, toe=%d, tor=%.3f,prn=%d", bds_eph_n->toe, pTime->tor, sv_id);
      return FALSE;
    }

    //check eph health
    if (bds_eph_n->SatH1 == 1)
      bds_eph_n->eph_status = EPH_STATUS_INVALID;

    int16_t week = bds_eph_n->WN;
    if (pTime->tor < SECS_IN_WEEK / 2 && pTime->tor > 10.0 && (pTime->tor - bds_eph_n->toe) < -SECS_IN_WEEK / 2 &&
        (pTime->tor + (int64_t)SECS_IN_WEEK - bds_eph_n->toe) < 3600 * 2.0) // tor should be a valid number
    {
      week++;// in some cases week update, but bds week not update, for example, on sunday 8 am, indoor scenarion
    }


    // Set the week number,but there is no para week in AGNSSL eph
    if (bds_eph_n->eph_source != EPH_SOURCE_AGNSS_L && bds_eph_n->WN > 0)
      gnss_Sd_Nm_SaveWeek(bds_eph_n->eph_source, BDS_MODE, week, pTime);

    dto = 0.0;
    if (NULL != p_Nm->ephData[idx] && pTime->tor > 1e-3)
    {
      dto = bds_eph_o->toe - pTime->tor;
      if (dto < -SECS_IN_WEEK / 2.0) dto += (int64_t)SECS_IN_WEEK;
      else if (dto > SECS_IN_WEEK / 2.0) dto -= (int64_t)SECS_IN_WEEK;
    }
    //1) If there is no EPH, then just copy the new EPH
    if (p_Nm->ephData[idx] == NULL) {
      if ((p_Nm->ephData[idx] = Sys_Malloc(sizeof(BDS_EPH_INFO))) == NULL) {
        SYS_LOGGING(OBJ_SD, LOG_ERROR, "Memory Alloc Fail %s %d", __FUNCTION__, __LINE__);
        return FALSE;
      }
      memcpy(p_Nm->ephData[idx], src, sizeof(BDS_EPH_INFO));
      GLOGI("BDS_EPH updated,prn=%3u,week=%4d,sec=%d",
        (unsigned)sv_id, (int)bds_eph_n->WN, bds_eph_n->toe);
    }
    else if (dto > 3600 * 1.5)
    {
      memcpy(p_Nm->ephData[idx], src, sizeof(BDS_EPH_INFO));
      GLOGW("The latest BDS eph time is wrong:toe=%.3f, tor=%.3f,prn=%d", bds_eph_o->toe, pTime->tor, sv_id);
    }
    else {
      //first check whether the eph is a new one
      dt = ((double)bds_eph_n->toe - (double)bds_eph_o->toe);
      if (bds_eph_n->WN != bds_eph_o->WN)
      {
        dt += ((double)bds_eph_n->WN - (double)bds_eph_o->WN) * (int64_t)SECS_IN_WEEK;
      }
      else if (dt < -SECS_IN_WEEK / 2.0 && bds_eph_n->toe <= 7200 && bds_eph_n->eph_source == EPH_SOURCE_AGNSS_L)
      {
        dt += (int64_t)SECS_IN_WEEK;
      }

      //check whether the decoded eph is valid,compare with the nearest one
      if (fabs(dt) < 3600 && bds_eph_n->eph_source == EPH_SOURCE_BRDC) {
        BDS_EPH_INFO          eph_new, eph_old;
        ECEF                  svp_new, svp_old;
        double                   dClk_new, dClk_old;
        double                   dopp_new, dopp_old;

        //init
        memset(&svp_new, 0, sizeof(ECEF));
        memset(&svp_old, 0, sizeof(ECEF));
        memcpy(&eph_new, bds_eph_n, sizeof(BDS_EPH_INFO));
        memcpy(&eph_old, bds_eph_o, sizeof(BDS_EPH_INFO));

        if (TRUE == bds_sd_sv_pos_e(&eph_new, eph_new.toe, &svp_new, (BDS_ID_IS_GEO(sv_id)), &dClk_new, &dopp_new, 0) &&
          TRUE == bds_sd_sv_pos_e(&eph_old, eph_new.toe, &svp_old, (BDS_ID_IS_GEO(sv_id)), &dClk_old, &dopp_old, 0))
        {
          double diff;
          diff = fabs(svp_new.p[0] - svp_old.p[0]) + fabs(svp_new.p[1] - svp_old.p[1]) + fabs(svp_new.p[2] - svp_old.p[2]);
          if (diff > 200.0) {
            GLOGE("sv pos from different ephs(bds) compared failed! pos_diff=%f, %s %d",
              diff, __FUNCTION__, __LINE__);
            goto label_end_addeph;
          }
        }
      }


      //2)time compare, the new one is old ,then compare with the backup one
      if ((!reverse && dt < -0.1) || (reverse && dt > 0.1)) {
        //check the backup eph's existence
        if (p_Nm->ephDataBack[idx] == NULL) {
          if ((p_Nm->ephDataBack[idx] = Sys_Malloc(sizeof(BDS_EPH_INFO))) == NULL) {
            SYS_LOGGING(OBJ_SD, LOG_ERROR, "Memory Alloc Fail %s %d", __FUNCTION__, __LINE__);
            return FALSE;
          }

          //add into the backup one;
          memset(p_Nm->ephDataBack[idx], 0, sizeof(BDS_EPH_INFO));
          memcpy(p_Nm->ephDataBack[idx], bds_eph_n, sizeof(BDS_EPH_INFO));
          SYS_LOGGING(OBJ_SD, LOG_INFO, "BDS_EPH BACKUP updated,prn=%u,WN=%d,SEC=%d",
            (unsigned)sv_id, (int)bds_eph_n->WN, bds_eph_n->toe);
        }
        else {
          BDS_EPH_INFO* bds_eph_b = NULL;

          bds_eph_b = (BDS_EPH_INFO*)p_Nm->ephDataBack[idx];
          //compare the new one with the backup one
          dt = ((double)bds_eph_n->toe - bds_eph_b->toe);
          //week leap
          if (bds_eph_n->WN != bds_eph_o->WN)
          {
            dt += ((double)bds_eph_n->WN - bds_eph_o->WN) * (int64_t)SECS_IN_WEEK;
          }

          if ((!reverse && dt < 0.01) || (reverse && dt > -0.01))
            goto label_end_addeph;

          //exchange the backup one to new one
          memset(p_Nm->ephDataBack[idx], 0, sizeof(BDS_EPH_INFO));
          memcpy(p_Nm->ephDataBack[idx], bds_eph_n, sizeof(BDS_EPH_INFO));
          SYS_LOGGING(OBJ_SD, LOG_INFO, "BDS_EPH BACKUP updated,prn=%u,WN=%d,SEC=%d",
            (unsigned)sv_id, (int)bds_eph_n->WN, bds_eph_n->toe);
        }
      }
      else if ((!reverse && dt > 0.1) || (reverse && dt < -0.1)) {
        //3)the new one is the newest one,then update
        //check the backup eph's existence
        if (p_Nm->ephDataBack[idx] == NULL) {
          if ((p_Nm->ephDataBack[idx] = Sys_Malloc(sizeof(BDS_EPH_INFO))) == NULL) {
            SYS_LOGGING(OBJ_SD, LOG_ERROR, "Memory Alloc Fail %s %d", __FUNCTION__, __LINE__);
            return FALSE;
          }
        }

        //copy the current to old one
        memcpy(p_Nm->ephDataBack[idx], p_Nm->ephData[idx], sizeof(BDS_EPH_INFO));
        //copy the new one to current one;
        memcpy(p_Nm->ephData[idx], bds_eph_n, sizeof(BDS_EPH_INFO));

        GLOGI("BDS_EPH updated,prn=%3u,week=%4d,sec=%d",
          (unsigned)sv_id, (int)bds_eph_n->WN, bds_eph_n->toe);
      }
      else {
        if (bds_eph_n->eph_source != EPH_SOURCE_BRDC) {
          //copy the new one to current one;
          memcpy(p_Nm->ephData[idx], bds_eph_n, sizeof(BDS_EPH_INFO));
          GLOGI("BDS_EPH updated,prn=%3u,week=%4d,sec=%d",
            (unsigned)sv_id, (int)bds_eph_n->WN, bds_eph_n->toe);
        }
        goto label_end_addeph;
      }
    }
  }
  else if (gnssMode == GAL_MODE) {
    GAL_EPH_INFO* gal_eph_n, * gal_eph_o;
    gal_eph_n = (GAL_EPH_INFO*)src;
    gal_eph_o = (GAL_EPH_INFO*)p_Nm->ephData[idx];

    if (!gal_eph_valid_check(gal_eph_n))
    {
      return FALSE;
    }

    if (fabs(gal_eph_n->toe - pTime->tor) > 3600 * 3.0 && pTime->tor > 1e-3 &&
      fabs(pTime->tor + (int64_t)SECS_IN_WEEK - gal_eph_n->toe) > 3600 * 3.0 &&
      fabs(gal_eph_n->toe + (int64_t)SECS_IN_WEEK - pTime->tor) > 3600 * 3.0)
    {
      GLOGW("add gal eph: toe too old, toe=%d, tor=%.3f,prn=%d", gal_eph_n->toe, pTime->tor, sv_id);
      return FALSE;
    }

    //check eph health
    if (gal_eph_n->svHealth != 0)
      gal_eph_n->eph_status = EPH_STATUS_INVALID;

    int16_t week = gal_eph_n->WN;
    if (pTime->tor < SECS_IN_WEEK / 2 && pTime->tor > 10.0 && (pTime->tor - gal_eph_n->toe) < -SECS_IN_WEEK / 2 &&
        (pTime->tor + (int64_t)SECS_IN_WEEK - gal_eph_n->toe) < 3600 * 3.0) // tor should be a valid number
    {
      week++;// in some cases week update, but gal week not update, for example, on sunday 8 am, indoor scenarion
    }


    // Set the week number,but there is no para week in AGNSSL eph
    if (gal_eph_n->eph_source != EPH_SOURCE_AGNSS_L && gal_eph_n->WN > 0)
      gnss_Sd_Nm_SaveWeek(gal_eph_n->eph_source, GAL_MODE, week, pTime);

    dto = 0.0;
    if (NULL != p_Nm->ephData[idx] && pTime->tor > 1e-3)
    {
      dto = gal_eph_o->toe - pTime->tor;
      if (dto < -SECS_IN_WEEK / 2.0) dto += (int64_t)SECS_IN_WEEK;
      else if (dto > SECS_IN_WEEK / 2.0) dto -= (int64_t)SECS_IN_WEEK;
    }
    //1) If there is no EPH, then just copy the new EPH
    if (p_Nm->ephData[idx] == NULL) {
      if ((p_Nm->ephData[idx] = Sys_Malloc(sizeof(GAL_EPH_INFO))) == NULL) {
        SYS_LOGGING(OBJ_SD, LOG_ERROR, "Memory Alloc Fail %s %d", __FUNCTION__, __LINE__);
        return FALSE;
      }
      memcpy(p_Nm->ephData[idx], src, sizeof(GAL_EPH_INFO));
      SYS_LOGGING(OBJ_SD, LOG_INFO, "GAL_EPH updated,prn=%u,week=%d,sec=%d",
        (unsigned)sv_id, (int)gal_eph_n->WN, gal_eph_n->toe);
    }
    else if (dto > 3600 * 0.5)
    {
      memcpy(p_Nm->ephData[idx], src, sizeof(GAL_EPH_INFO));
      GLOGW("The latest GAL eph time is wrong:toe=%.3f, tor=%.3f,prn=%d", gal_eph_o->toe, pTime->tor, sv_id);
    }
    else {
      //first check whether the eph is a new one
      dt = ((double)gal_eph_n->toe - gal_eph_o->toe);
      //week leap
      if (dt < -SECS_IN_WEEK / (2.0))      dt = dt + (int64_t)SECS_IN_WEEK;
      else if (dt > SECS_IN_WEEK / (2.0))  dt = dt - (int64_t)SECS_IN_WEEK;

      //2)time compare, the new one is old ,then compare with the backup one
      if ((!reverse && dt < -0.1) || (reverse && dt > 0.1)) {
        //check the backup eph's existence
        if (p_Nm->ephDataBack[idx] == NULL) {
          if ((p_Nm->ephDataBack[idx] = Sys_Malloc(sizeof(GAL_EPH_INFO))) == NULL) {
            SYS_LOGGING(OBJ_SD, LOG_ERROR, "Memory Alloc Fail %s %d", __FUNCTION__, __LINE__);
            return FALSE;
          }

          //add into the backup one;
          memset(p_Nm->ephDataBack[idx], 0, sizeof(GAL_EPH_INFO));
          memcpy(p_Nm->ephDataBack[idx], gal_eph_n, sizeof(GAL_EPH_INFO));
          SYS_LOGGING(OBJ_SD, LOG_INFO, "GAL_EPH BACKUP updated,prn=%u,week=%d,sec=%d",
            (unsigned)sv_id, (int)gal_eph_n->WN, gal_eph_n->toe);
        }
        else {
          GAL_EPH_INFO* gal_eph_b = NULL;

          gal_eph_b = (GAL_EPH_INFO*)p_Nm->ephDataBack[idx];
          //compare the new one with the backup one
          dt = ((double)gal_eph_n->toe - gal_eph_b->toe);
          //week leap
          if (dt < -SECS_IN_WEEK / (2.0))      dt = dt + (double)SECS_IN_WEEK;
          else if (dt > SECS_IN_WEEK / (2.0))  dt = dt - (double)SECS_IN_WEEK;

          if ((!reverse && dt < 0.01) || (reverse && dt > -0.01))
            goto label_end_addeph;

          //exchange the backup one to new one
          memset(p_Nm->ephDataBack[idx], 0, sizeof(GAL_EPH_INFO));
          memcpy(p_Nm->ephDataBack[idx], gal_eph_n, sizeof(GAL_EPH_INFO));
          SYS_LOGGING(OBJ_SD, LOG_INFO, "GAL_EPH BACKUP updated,prn=%u,week=%d,sec=%d",
            (unsigned)sv_id, (int)gal_eph_n->WN, gal_eph_n->toe);
        }
      }
      else if ((!reverse && dt > 0.1) || (reverse && dt < -0.1)) {
        //3)the new one is the newest one,then update
        //check the backup eph's existence
        if (p_Nm->ephDataBack[idx] == NULL) {
          if ((p_Nm->ephDataBack[idx] = Sys_Malloc(sizeof(GAL_EPH_INFO))) == NULL) {
            SYS_LOGGING(OBJ_SD, LOG_ERROR, "Memory Alloc Fail %s %d", __FUNCTION__, __LINE__);
            return FALSE;
          }
        }

        //copy the current to old one
        memcpy(p_Nm->ephDataBack[idx], p_Nm->ephData[idx], sizeof(GAL_EPH_INFO));
        //copy the new one to current one;
        memcpy(p_Nm->ephData[idx], gal_eph_n, sizeof(GAL_EPH_INFO));

        SYS_LOGGING(OBJ_SD, LOG_INFO, "GAL_EPH updated,prn=%u,week=%d,sec=%d",
          (unsigned)sv_id, (int)gal_eph_n->WN, gal_eph_n->toe);
      }
      else {
        goto label_end_addeph;
      }
    }
  }
  else {
    SYS_LOGGING(OBJ_SD, LOG_ERROR, "Not Support System!");
    return FALSE;
  }

label_end_addeph:

#ifdef AG_GNSS_RTK_FUNCTION_IMPL
  /* update rtk nav info */
  if (1/*pTime->init*/)
  {
    gnss_rtk_nav_update(gnssMode, sv_id, p_Nm->ephData[idx], p_Nm->ephDataBack[idx]);
  }
#endif

  return TRUE;
}

/***********************************************************************
* ��������: gnss_Sd_Nm_RmSv
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/22/
***********************************************************************/
void gnss_Sd_Nm_RmSv(uint8_t gnssMode, uint8_t sv_id)
{
  int32_t           idx;

  idx = gnss_sv_Idx(gnssMode, sv_id);
  if (idx < 0) return;


  Sys_Mem_Disable();

  if (p_Nm->ephData[idx])
  {
    Sys_Free(p_Nm->ephData[idx]);
    p_Nm->ephData[idx] = NULL;
  }
  if (p_Nm->ephDataBack[idx])
  {
    Sys_Free(p_Nm->ephDataBack[idx]);
    p_Nm->ephDataBack[idx] = NULL;
  }
  if (p_Nm->almData[idx])
  {
    Sys_Free(p_Nm->almData[idx]);
    p_Nm->almData[idx] = NULL;
  }

  Sys_Mem_Enable();
  return;
}
/***********************************************************************
* Description: gnss_Sd_Nm_GetGlnChanN
* Input:
* Output:
* Return:
* Author: none
* Date: 05/10/
***********************************************************************/
int8_t gnss_Sd_Nm_GetGlnChanN(int32_t prn)
{
#ifdef ENAGLO
  if (prn<MINPRNGLO || prn>MAXPRNGLO)
  {
    return 127; //invalid
  }

  return p_Nm->gloChannel[prn - 1];
#else
  return 127;
#endif
}
/***********************************************************************
* Description: gnss_Sd_Nm_GetGlnChanN
* Input:
* Output:
* Return:
* Author: none
* Date: 05/10/
***********************************************************************/
void gnss_Sd_Nm_SetGlnChanN(int32_t prn, int8_t fcn)
{
#ifdef ENAGLO
  if (prn<MINPRNGLO || prn>MAXPRNGLO || fcn < -7 || fcn>6)    //fcn[-7,6]
  {
    return; //invalid
  }

  p_Nm->gloChannel[prn - 1] = fcn;
#endif

}

/***********************************************************************
* ��������: gnss_Sd_Nm_GetEph
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/22/
***********************************************************************/
void* gnss_Sd_Nm_GetEph(uint8_t gnssMode, uint8_t sv_id)
{
  int32_t             idx;
  GPS_EPH_INFO* pGpsEph;
  dgnss_t* pSvRtd;

  idx = gnss_sv_Idx(gnssMode, sv_id);
  if (idx < 0) {
    return NULL;
  }

  pSvRtd = gnss_sd_get_sv_rtd(gnssMode, sv_id);

  if (gnss_Pe_Get_RTD_Status() == FALSE || pSvRtd == NULL)
  {
    return p_Nm->ephData[idx];
  }
  else
  {
    if (gnssMode == GPS_MODE)
    {
      if (gnss_Pe_Get_SvRtd_Status(gnssMode, sv_id))
      {
        pGpsEph = (GPS_EPH_INFO*)p_Nm->ephData[idx];
        if (pGpsEph)
        {
          if ((pGpsEph->IODE & 0xFF) == (pSvRtd->iod & 0xFF))
          {
            return p_Nm->ephData[idx];
          }
        }
        pGpsEph = (GPS_EPH_INFO*)p_Nm->ephDataBack[idx];
        if (pGpsEph)
        {
          if ((pGpsEph->IODE & 0xFF) == (pSvRtd->iod & 0xFF))
          {
            SYS_LOGGING(OBJ_SD, LOG_INFO, "Use Back EPH:%02d %03d", gnssMode, sv_id);
            return p_Nm->ephDataBack[idx];
          }
        }
        return p_Nm->ephData[idx];

      }
      else
      {
        return p_Nm->ephData[idx];
      }
    }
    else
    {
      return p_Nm->ephData[idx];
    }
  }

  return NULL;
}

//bad sv check
uint8_t gnss_sd_badSV_check(uint8_t gnssMode, uint8_t prn) {


  if (NULL == p_Nm) {
    SYS_LOGGING(OBJ_SD, LOG_ERROR, "inputs error!, %s, %d", __FUNCTION__, __LINE__);
    return FALSE;
  }

  //check bds sv	
  if (BDS_MODE == gnssMode && prn < MAX_BDS_PRN && prn >= MIN_BDS_PRN) {
    if (p_Nm->bdsBadSvList[prn - 1] == 1)
      return FALSE;
  }

  return TRUE;
}

uint8_t gnss_Sd_Nm_Check_NoEph(uint8_t gnssMode, uint8_t sv_id)
{
  void* pEph;
  GPS_EPH_INFO* pGpsEph;
  GLN_EPH_INFO* pGlnEph;
  BDS_EPH_INFO* pBdsEph;
  GAL_EPH_INFO* pGalEph;

  pEph = gnss_Sd_Nm_GetEph(gnssMode, sv_id);

  if (pEph == NULL)
  {
    //bad sv check
    if (FALSE == gnss_sd_badSV_check(gnssMode, sv_id)) {
      SYS_LOGGING(OBJ_SD, LOG_WARNING, "bad SV(out of service)! GnssMode=%d,PRN=%d, %s, %d", gnssMode, sv_id, __FUNCTION__, __LINE__);
      return FALSE;
    }
    else {
      return TRUE;
    }
  }

  if (gnssMode == GPS_MODE)
  {
    pGpsEph = (GPS_EPH_INFO*)pEph;
    if (pGpsEph->eph_status == EPH_STATUS_EXPIRE)
    {
      return TRUE;
    }
  }
  else if (gnssMode == GLN_MODE)
  {
    pGlnEph = (GLN_EPH_INFO*)pEph;
    if (pGlnEph->eph_status == EPH_STATUS_EXPIRE)
    {
      return TRUE;
    }
  }
  else if (gnssMode == BDS_MODE)
  {
    pBdsEph = (BDS_EPH_INFO*)pEph;
    if (pBdsEph->eph_status == EPH_STATUS_EXPIRE)
    {
      return TRUE;
    }
  }
  else
  {
    pGalEph = (GAL_EPH_INFO*)pEph;
    if (pGalEph->eph_status == EPH_STATUS_EXPIRE)
    {
      return TRUE;
    }
  }

  return FALSE;
}

/***********************************************************************
* ��������: gnss_Sd_Nm_AddAlm
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/22/
***********************************************************************/
void gnss_Sd_Nm_AddAlm(uint8_t gnssMode, uint8_t sv_id, void* src)
{

  int32_t            idx;
  uint32_t            size;
  double            dt;

  //inputs check
  if (NULL == src) {
    return;
  }

  idx = gnss_sv_Idx(gnssMode, sv_id);
  if (idx < 0) return;

  if (gnssMode == GPS_MODE) {

    size = sizeof(GPS_ALM_INFO);
    if (p_Nm->almData[idx] == NULL) {
      if ((p_Nm->almData[idx] = Sys_Malloc(size)) == NULL) {
        SYS_LOGGING(OBJ_SD, LOG_ERROR, "Can't alloc memory %s %d", __FUNCTION__, __LINE__);
        return;
      }
      memcpy(p_Nm->almData[idx], src, size);
      SYS_LOGGING(OBJ_SD, LOG_INFO, "GPS_ALM updated,prn=%d", sv_id);

    }
    else {

      GPS_ALM_INFO* gps_alm_n, * gps_alm_o;

      gps_alm_n = (GPS_ALM_INFO*)src;
      gps_alm_o = (GPS_ALM_INFO*)p_Nm->almData[idx];

      dt = ((double)gps_alm_n->wn_oa - (double)gps_alm_o->wn_oa) * (int64_t)SECS_IN_WEEK +
       ((double)gps_alm_n->t_oa - gps_alm_o->t_oa);

      if (dt > (int64_t)128 * SECS_IN_WEEK)     dt = dt - (int64_t)256 * SECS_IN_WEEK;
      else if (dt < -(int64_t)128 * SECS_IN_WEEK) dt = dt + (int64_t)256 * SECS_IN_WEEK;

      if (dt < 0.1)
        return;

      memcpy(p_Nm->almData[idx], (void*)gps_alm_n, size);

      SYS_LOGGING(OBJ_SD, LOG_INFO, "GPS_ALM updated,prn=%d, week=%d, toa=%d", sv_id, gps_alm_n->wn_oa, gps_alm_n->t_oa_raw);
    }
  }
  else if (gnssMode == GLN_MODE) {

    size = sizeof(GLN_ALM_INFO);
    if (p_Nm->almData[idx] == NULL) {
      if ((p_Nm->almData[idx] = Sys_Malloc(size)) == NULL) {
        SYS_LOGGING(OBJ_SD, LOG_ERROR, "Can't alloc memory %s %d", __FUNCTION__, __LINE__);
        return;
      }
      memcpy(p_Nm->almData[idx], src, size);
      SYS_LOGGING(OBJ_SD, LOG_INFO, "GLN_ALM updated, prn = %d", sv_id);
    }
    else {
      GLN_ALM_INFO* gln_alm_n, * gln_alm_o;

      gln_alm_n = (GLN_ALM_INFO*)src;
      gln_alm_o = (GLN_ALM_INFO*)p_Nm->almData[idx];

      dt = ((double)gln_alm_n->N_A - (double)gln_alm_o->N_A);
      if (dt > 800.0)     dt = dt - 1461.0;
      else if (dt < -800.0) dt = dt + 1461.0;

      if (dt < 0.1)
        return;

      //alm update
      memcpy(p_Nm->almData[idx], gln_alm_n, size);

      SYS_LOGGING(OBJ_SD, LOG_INFO, "GLN_ALM updated,prn =%d, N4=%d, NA=%d", sv_id, gln_alm_n->N4, gln_alm_n->N_A);
    }
  }
  else if (gnssMode == BDS_MODE) {
    size = sizeof(BDS_ALM_INFO);
    if (p_Nm->almData[idx] == NULL) {
      if ((p_Nm->almData[idx] = Sys_Malloc(size)) == NULL) {
        SYS_LOGGING(OBJ_SD, LOG_ERROR, "Can't alloc memory %s %d", __FUNCTION__, __LINE__);
        return;
      }
      memcpy(p_Nm->almData[idx], src, size);
      SYS_LOGGING(OBJ_SD, LOG_INFO, "BDS_ALM updated,prn=%d", sv_id);

    }
    else {

      BDS_ALM_INFO* bds_alm_n, * bds_alm_o;

      bds_alm_n = (BDS_ALM_INFO*)src;
      bds_alm_o = (BDS_ALM_INFO*)p_Nm->almData[idx];

      dt = (double)(bds_alm_n->WNa - bds_alm_o->WNa) * (int64_t)SECS_IN_WEEK +
        (double)(bds_alm_n->toa - bds_alm_o->toa);

      if (dt > (int64_t)128* SECS_IN_WEEK)     dt = dt - (int64_t)256 *SECS_IN_WEEK;
      else if (dt < (int64_t)-128 *SECS_IN_WEEK) dt = dt + (int64_t)256 *SECS_IN_WEEK;

      if (dt < 0.1)
        return;

      memcpy(p_Nm->almData[idx], (void*)bds_alm_n, size);

      SYS_LOGGING(OBJ_SD, LOG_INFO, "BDS_ALM updated,prn=%d, wna=%d, toa=%d",
        sv_id, bds_alm_n->WNa, bds_alm_n->toa);
    }
  }
  else if (gnssMode == GAL_MODE) {
    size = sizeof(GAL_ALM_INFO);
    if (p_Nm->almData[idx] == NULL) {
      if ((p_Nm->almData[idx] = Sys_Malloc(size)) == NULL) {
        SYS_LOGGING(OBJ_SD, LOG_ERROR, "Can't alloc memory %s %d", __FUNCTION__, __LINE__);
        return;
      }
      memcpy(p_Nm->almData[idx], src, size);
      SYS_LOGGING(OBJ_SD, LOG_INFO, "GAL_ALM updated,prn=%d", sv_id);

    }
    else {

      GAL_ALM_INFO* gal_alm_n, * gal_alm_o;

      gal_alm_n = (GAL_ALM_INFO*)src;
      gal_alm_o = (GAL_ALM_INFO*)p_Nm->almData[idx];

      dt = ((double)gal_alm_n->wn_a - (double)gal_alm_o->wn_a) * (int64_t)SECS_IN_WEEK +
        (double)(gal_alm_n->t_0a - gal_alm_o->t_0a);

      if (dt > 2.0 * (int64_t)SECS_IN_WEEK)     dt = dt - 4.0 * (int64_t)SECS_IN_WEEK;
      else if (dt < -2.0 * (int64_t)SECS_IN_WEEK) dt = dt + 4.0 * (int64_t)SECS_IN_WEEK;

      if (dt < 0.1)
        return;

      memcpy(p_Nm->almData[idx], (void*)gal_alm_n, size);

      SYS_LOGGING(OBJ_SD, LOG_INFO, "GAL_ALM updated,prn=%d, week=%d, toa=%d", sv_id, gal_alm_n->wn_a, gal_alm_n->t_0a);
    }
  }
  else {
    SYS_LOGGING(OBJ_SD, LOG_WARNING, "Not supported GALLIEO, %s %d", __FUNCTION__, __LINE__);
  }

}

/***********************************************************************
* ��������: gnss_Sd_Nm_GetAlm
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/22/
***********************************************************************/
void* gnss_Sd_Nm_GetAlm(uint8_t gnssMode, uint8_t sv_id)
{
  int32_t          idx;

  idx = gnss_sv_Idx(gnssMode, sv_id);
  if (idx < 0) return NULL;

  return p_Nm->almData[idx];
}
/***********************************************************************
* ��������: gnss_Sd_Nm_AddIono
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/22/
***********************************************************************/

void gnss_Sd_Nm_AddUtc(uint8_t gnssMode, void* src)
{
  //local params
  double dt;

  if (NULL == src) {
    SYS_LOGGING(OBJ_SD, LOG_ERROR, "inputs check error. %s %d", __FUNCTION__, __LINE__);
    return;
  }

  //**************
  if (GPS_MODE == gnssMode) {
    gpsUtcModel_t* gps_utc;

    gps_utc = (gpsUtcModel_t*)src;

    //valid check
    if (gps_utc->utcDN < 1 || gps_utc->utcDN > 7)  return;
    if (gps_utc->utcTot < 0 || gps_utc->utcTot >= SECS_IN_WEEK) return;
    if (gps_utc->utcWNlsf < 0 || gps_utc->utcWNlsf  > 255) return;
    if (gps_utc->utcDeltaTlsf < 0 || gps_utc->utcDeltaTls < 0 || gps_utc->utcDeltaTlsf > 100 || gps_utc->utcDeltaTls > 100) return;
    if ((gps_utc->utcDeltaTlsf - gps_utc->utcDeltaTls) < 0 || (gps_utc->utcDeltaTlsf - gps_utc->utcDeltaTls) > 1) return;

    if ((gps_utc->utcWNt - p_Nm->gpsUtcModel.utcWNt) < -128) {
      dt = ((double)gps_utc->utcWNt - p_Nm->gpsUtcModel.utcWNt + 256) * (int64_t)SECS_IN_WEEK +
       ((double)gps_utc->utcTot - p_Nm->gpsUtcModel.utcTot) * 4096.0;
    }
    else {
      dt = ((double)gps_utc->utcWNt - p_Nm->gpsUtcModel.utcWNt) * (int64_t)SECS_IN_WEEK +
        ((double)gps_utc->utcTot - p_Nm->gpsUtcModel.utcTot) * 4096.0;
    }

    if (dt < 0.01)
      return;

    memset(&p_Nm->gpsUtcModel, 0, sizeof(gpsUtcModel_t));
    memcpy(&p_Nm->gpsUtcModel, gps_utc, sizeof(gpsUtcModel_t));
    SYS_LOGGING(OBJ_SD, LOG_INFO, "gps_utc updated. WNt=%d Tot=%d", gps_utc->utcWNt, gps_utc->utcTot);
  }
  else if (GLN_MODE == gnssMode) {
    glnUtcModel_t* gln_utc;

    gln_utc = (glnUtcModel_t*)src;

    if ((gln_utc->nA - p_Nm->glnUtcModel.nA) < -700)
      dt = ((double)gln_utc->nA - p_Nm->glnUtcModel.nA) + 1461.0;
    else
      dt = ((double)gln_utc->nA - p_Nm->glnUtcModel.nA);

    if (dt < 0.01)
      return;

    memset(&p_Nm->glnUtcModel, 0, sizeof(glnUtcModel_t));
    memcpy(&p_Nm->glnUtcModel, gln_utc, sizeof(glnUtcModel_t));
    SYS_LOGGING(OBJ_SD, LOG_INFO, "gln_utc updated. NA=%d", gln_utc->nA);
  }
  else if (BDS_MODE == gnssMode) {
    bdsUtcModel_t* bds_utc;

    bds_utc = (bdsUtcModel_t*)src;

    //valid check
    if (bds_utc->utcDN < 0 || bds_utc->utcDN > 6)  return;
    if (bds_utc->utcWNlsf < 0 || bds_utc->utcWNlsf  > 255) return;
    if (bds_utc->utcDeltaTlsf < 0 || bds_utc->utcDeltaTls < 0 || bds_utc->utcDeltaTlsf > 100 || bds_utc->utcDeltaTls > 100) return;
    if ((bds_utc->utcDeltaTlsf - bds_utc->utcDeltaTls) < 0 || (bds_utc->utcDeltaTlsf - bds_utc->utcDeltaTls) > 1) return;

    if (bds_utc->utcA0 == p_Nm->bdsUtcModel.utcA0 &&
      bds_utc->utcA1 == p_Nm->bdsUtcModel.utcA1 &&
      bds_utc->utcDeltaTls == p_Nm->bdsUtcModel.utcDeltaTls &&
      bds_utc->utcDeltaTlsf == p_Nm->bdsUtcModel.utcDeltaTlsf &&
      bds_utc->utcDN == p_Nm->bdsUtcModel.utcDN &&
      bds_utc->utcWNlsf == p_Nm->bdsUtcModel.utcWNlsf)
      return;

    memset(&p_Nm->bdsUtcModel, 0, sizeof(bdsUtcModel_t));
    memcpy(&p_Nm->bdsUtcModel, bds_utc, sizeof(bdsUtcModel_t));
    SYS_LOGGING(OBJ_SD, LOG_INFO, "bds_utc updated!");
  }
  else if (GAL_MODE == gnssMode) {
    galUtcModel_t* gal_utc;

    gal_utc = (galUtcModel_t*)src;

    if (gal_utc->a0 == p_Nm->galUtcModel.a0 &&
      gal_utc->a1 == p_Nm->galUtcModel.a1 &&
      gal_utc->deltT_ls == p_Nm->galUtcModel.deltT_ls &&
      gal_utc->deltT_lsf == p_Nm->galUtcModel.deltT_lsf &&
      gal_utc->dn == p_Nm->galUtcModel.dn &&
      gal_utc->t_0t == p_Nm->galUtcModel.t_0t &&
      gal_utc->wn_0t == p_Nm->galUtcModel.wn_0t &&
      gal_utc->wn_lsf == p_Nm->galUtcModel.wn_lsf &&
      gal_utc->have_utc == p_Nm->galUtcModel.have_utc)
      return;

    memset(&p_Nm->galUtcModel, 0, sizeof(galUtcModel_t));
    memcpy(&p_Nm->galUtcModel, gal_utc, sizeof(galUtcModel_t));
    SYS_LOGGING(OBJ_SD, LOG_INFO, "gal_utc updated!");

  }
  else {
    SYS_LOGGING(OBJ_SD, LOG_ERROR, "not supported system. %s %d", __FUNCTION__, __LINE__);
    return;
  }
}

void gnss_Sd_Nm_AddIono(uint8_t gnssMode, ION_INFO* iono)
{
  //inputs check
  if (NULL == iono) {
    SYS_LOGGING(OBJ_SD, LOG_ERROR, "inputs check error. %s %d", __FUNCTION__, __LINE__);
    return;
  }

  if (iono->alpha_0 == p_Nm->ionoData[gnssMode].alpha_0 &&
    iono->alpha_1 == p_Nm->ionoData[gnssMode].alpha_1 &&
    iono->alpha_2 == p_Nm->ionoData[gnssMode].alpha_2 &&
    iono->alpha_3 == p_Nm->ionoData[gnssMode].alpha_3 &&
    iono->beta_0 == p_Nm->ionoData[gnssMode].beta_0 &&
    iono->beta_1 == p_Nm->ionoData[gnssMode].beta_1 &&
    iono->beta_2 == p_Nm->ionoData[gnssMode].beta_2 &&
    iono->beta_3 == p_Nm->ionoData[gnssMode].beta_3 &&
    iono->have_ion == p_Nm->ionoData[gnssMode].have_ion)
    return;

  /* check if the iono paras are all valid */
  if (fabs(iono->alpha_0) > 128 * pow(2.0, -30.0) || fabs(iono->alpha_1) > 128 * pow(2.0, -27.0) ||
    fabs(iono->alpha_2) > 128 * pow(2.0, -24.0) || fabs(iono->alpha_3) > 128 * pow(2.0, -24.0) ||
    fabs(iono->beta_0) > 128 * pow(2.0, 11.0) || fabs(iono->beta_1) > 128 * pow(2.0, 14.0) ||
    fabs(iono->beta_2) > 128 * pow(2.0, 16.0) || fabs(iono->beta_3) > 128 * pow(2.0, 16.0))
  {
    iono->have_ion = 0;
    SYS_LOGGING(OBJ_SD, LOG_WARNING, "IONO data check fail");
    return;
  }
  iono->have_ion = 1;
  p_Nm->ionoData[gnssMode] = (*iono);
  SYS_LOGGING(OBJ_SD, LOG_INFO, "IONO data updated,gnss_mode=%d", gnssMode);
}

void gnss_Sd_Nm_AddIono_gal(uint8_t gnssMode, GAL_IONO_INFO iono)
{
  if (gnssMode != GAL_MODE)
    return;

  if (iono.a_i0 == p_Nm->ionoData_gal.a_i0 &&
    iono.a_i1 == p_Nm->ionoData_gal.a_i1 &&
    iono.a_i2 == p_Nm->ionoData_gal.a_i2)
    return;

  p_Nm->ionoData_gal = iono;
  SYS_LOGGING(OBJ_SD, LOG_INFO, "IONO data(GALILEO) updated");
}


/***********************************************************************
* ��������: gnss_Sd_Nm_GetIono
* ���������
* ���������
* ����ֵ��
* ���ߣ�
* ���ڣ�09/22/
***********************************************************************/
ION_INFO* gnss_Sd_Nm_GetIono(uint8_t gnssMode)
{
  return &(p_Nm->ionoData[gnssMode]);
}
