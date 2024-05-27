/************************************************************
* Copyrights(C)
* All rights Reserved
* �ļ����ƣ�gnss_tm.c
* �ļ�����������λ���Լ��Ӳ����
* �汾�ţ�1.0.0
* ���ߣ�
* ���ڣ�09/05
************************************************************/
#include "gnss_tm.h"
#include "gnss_sys_api.h"
#include "gnss_comm.h"
#include "gnss_sd_nm.h"
#include "gnss_config.h"
//#include "gnss_core_api.h"
#include "gnss_ls.h"
#include "gnss_hsm_lite_api.h"

#undef MODULE_TAG
#define MODULE_TAG OBJ_ALGO

GNSS_TIME  gnssTime;
GNSS_TIME* p_gnssTime = &gnssTime;

GNSS_TIMESYSPARAM gnssTimeSysParam;
GNSS_TIMESYSPARAM* p_gnssTimeSysParam = &gnssTimeSysParam;

extern Gnss_Cfg_t g_pe_cfg;
extern Ls_t* gpz_Ls;

extern gnss_nm_t* p_Nm;

extern rtk_t rtkctrl;

extern double leapsInUse[MAXLEAPS + 1][7];

extern rtk_t rtkctrl;            /*rtk control and results*/

extern void gnss_Kf_BiasCorrection(int8_t msCorr, uint32_t biasIdx);

/*********************leap second list************************/
uint32_t Leaplist[LEAP_NUM] =
{
  19810630, 19820630, 19830630, 19850630, 19871231, 19891231, 19901231, 19920630,
  19930630, 19940630, 19951231, 19970630, 19981231, 20051231, 20081231, 20120630,
  20150630, 20161231, 0,        0,        0,        0,        0,        0
};
static int32_t leap_year_days_list[12] = { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
static int32_t days_list[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

uint32_t LeapListInUse[LEAP_NUM] = { 0 };

/**********************************************************************
* Function Name:    gnss_tm_init
*
* Description:
*    gnss_tm_init() is used to initialize the time module
*
* Input:
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnss_tm_init(void)
{
  memset(p_gnssTime, 0, sizeof(GNSS_TIME));
  memset(p_gnssTimeSysParam, 0, sizeof(GNSS_TIMESYSPARAM));
}

/**********************************************************************
* Function Name:    gnss_tm_get_time
*
* Description:
*    gnss_tm_get_time() is used for getting time structure
*
* Input:
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
GNSS_TIME* gnss_tm_get_time(void)
{
  if (p_gnssTime)
  {
    return p_gnssTime;
  }
  else
  {
    return NULL;
  }
}

GNSS_TIMESYSPARAM* gnss_tm_get_sysparam(void)
{
  if (p_gnssTimeSysParam)
  {
    return p_gnssTimeSysParam;
  }
  else
  {
    return NULL;
  }
}

/**********************************************************************
* Function Name:    gnss_tm_leap_year
*
* Description:
*    gnss_tm_leap_year
*
* Input:
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
int32_t gnss_tm_leap_year(int32_t year)
{
  if ((year % 400) == 0)
  {
    return 1;
  }
  else if ((year % 100) == 0)
  {
    return 0;
  }
  else if ((year % 4) == 0)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}


/**********************************************************************
* Function Name:    gnss_tm_days_in_year
*
* Description:
*    gnss_tm_days_in_year
*
* Input:
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
int32_t gnss_tm_days_in_year(int32_t year)
{
  if (gnss_tm_leap_year(year))
  {
    return 366;
  }
  else
  {
    return 365;
  }
}


/**********************************************************************
* Function Name:    gnss_tm_days_in_mon
*
* Description:
*    gnss_tm_days_in_mon
*
* Input:
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
int32_t gnss_tm_days_in_mon(int32_t year, int32_t mon)
{
  if (mon < 1 || mon > 12)
  {
    return 0;
  }

  if (gnss_tm_leap_year(year))
  {
    return leap_year_days_list[mon - 1];
  }
  else
  {
    return days_list[mon - 1];
  }
}

/**********************************************************************
* Function Name:    gnss_tm_leap_secs_adjust
*
* Description:
*    gnss_tm_leap_secs_adjust
*
* Input:
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnss_tm_leap_secs_adjust(GNSS_TIME* pTime, meas_blk_t* pMeas)
{
  //int8_t              leapSecDiff;
  uint32_t             i/*, j*/;
  uint32_t             t = 0;
  uint16_t             utcWNlsf, utcFullWeek;
  uint16_t             utcDN;
  double             ep[6] = { 0 };
  //uint32_t             yearStart;
  gnss_meas_t* pSvMeas;
  gtime_t         timef = { 0 }, timenow = { 0 }/*, gpstimenow = { 0 }*/;
  //double             leapsecnow;

  // get leapSecond first
#if 0
  ep[0] = pTime->aidUtcTime.Year;
  ep[1] = pTime->aidUtcTime.Month;
  ep[2] = pTime->aidUtcTime.Day;
  ep[3] = pTime->aidUtcTime.Hour;
  ep[4] = pTime->aidUtcTime.Minute;
  ep[5] = pTime->aidUtcTime.Second;
#else
  ep[0] = pTime->utcTime.Year;
  ep[1] = pTime->utcTime.Month;
  ep[2] = pTime->utcTime.Day;
  ep[3] = pTime->utcTime.Hour;
  ep[4] = pTime->utcTime.Minute;
  ep[5] = pTime->utcTime.Second;
#endif
  timenow = gnss_epoch2time(ep);

  pTime->leapSecond = 0;
  for (i = 0; i < LEAP_NUM; i++)
  {
    ep[0] = LeapListInUse[i] / 10000;
    ep[1] = (LeapListInUse[i] % 10000) / 100;
    ep[2] = LeapListInUse[i] % 100;
    ep[3] = 23;
    ep[4] = 59;
    ep[5] = 59;
    timef = timeadd(gnss_epoch2time(ep), 1.0);
    if (LeapListInUse[i] != 0 && timediff(timenow, timef) >= 0.0)
    {
      pTime->leapSecond++;
    }
    else
    {
      break;
    }
  }

#if 0
  // First check saved leap-second and aid-leap-second, if they can't match, it means leap second jump happened
  if ((pTime->leapSecond != pTime->aidLeapSecond) && pTime->leapSecond > 0
    && pTime->aidLeapSecond > 0)
  {
    if (pTime->leapSecond > pTime->aidLeapSecond)
    {
      uint8_t leaps = 0, k;
      //delete wrong record
      for (i = 0; i < LEAP_NUM; i++)
      {
        leaps++;
        if ((leaps > pTime->aidLeapSecond) && LeapListInUse[i] > 0)
        {
          LeapListInUse[i] = 0;
        }
      }
      for (k = 0; k < 5; k++)  //delete max 5 records
      {
        if (leapsInUse[0][0] > 0.0 && (-leapsInUse[0][6]) > pTime->aidLeapSecond)
        {
          for (i = 1; i <= MAXLEAPS; i++)
          {
            memcpy(&(leapsInUse[i - 1][0]), &(leapsInUse[i][0]), 7 * sizeof(double));
            if (fabs(leapsInUse[i][0]) < 0.1)
            {
              break;
            }
          }
        }
        else
        {
          break;
        }
      }
    }
    else if ((pTime->aidLeapSecond - pTime->leapSecond) > 1)
    {
      SYS_LOGGING(OBJ_TM, LOG_INFO, "LARGE Leap DIFF: %d,%d", pTime->leapSecond, pTime->aidLeapSecond);

      leapSecDiff = (U16)(pTime->aidLeapSecond - pTime->leapSecond);

      if (pTime->aidUtcTime.Month > 0)
      {
        ep[0] = pTime->aidUtcTime.Year;
        ep[1] = pTime->aidUtcTime.Month;
        ep[2] = pTime->aidUtcTime.Day;
        ep[3] = pTime->aidUtcTime.Hour;
        ep[4] = pTime->aidUtcTime.Minute;
        ep[5] = pTime->aidUtcTime.Second;
        if (ep[1] < 7)
        {
          t = (uint32_t)((ep[0] - leapSecDiff) * 10000) + 12 * 100 + 31;
          yearStart = (uint32_t)ep[0] - (leapSecDiff - 1);
        }
        else
        {
          t = (uint32_t)((ep[0] - (leapSecDiff - 1)) * 10000) + 6 * 100 + 30;
          yearStart = (uint32_t)ep[0] - (leapSecDiff - 1);
        }

        for (j = 0; j < leapSecDiff; j++)
        {
          for (i = 0; i < LEAP_NUM; i++)
          {
            if ((LeapListInUse[i] / 10000) > yearStart)
            {
              break;
            }

            if (LeapListInUse[i] == 0)
            {
              //new time cannot be smaller than previous one
              if (i >= 1 && t > LeapListInUse[i - 1])
              {
                LeapListInUse[i] = t;
              }
              break;
            }
          }

          t += 10000;

          if (leapsInUse[0][0] < yearStart)
          {
            for (i = MAXLEAPS; i > 0; i--)
            {
              memcpy(&(leapsInUse[i][0]), &(leapsInUse[i - 1][0]), 7 * sizeof(double));
            }
            if (ep[1] <= 6)
            {
              leapsInUse[0][0] = yearStart;
              leapsInUse[0][1] = 1;
              leapsInUse[0][2] = 1;
              leapsInUse[0][6] = leapsInUse[1][6] - 1;
            }
            else if (ep[1] <= 12)
            {
              leapsInUse[0][0] = yearStart;
              leapsInUse[0][1] = 7;
              leapsInUse[0][2] = 1;
              leapsInUse[0][6] = leapsInUse[1][6] - 1;
            }
          }
          yearStart++;
        }

        goto LEAP_SEC_CHECK;
      }
    }
    else if ((pTime->aidLeapSecond - pTime->leapSecond) == 1)
    {
      // Use aided year information 
      if (pTime->aidUtcTime.Month > 0)
      {
        ep[0] = pTime->aidUtcTime.Year;
        ep[1] = pTime->aidUtcTime.Month;
        ep[2] = pTime->aidUtcTime.Day;
        ep[3] = pTime->aidUtcTime.Hour;
        ep[4] = pTime->aidUtcTime.Minute;
        ep[5] = pTime->aidUtcTime.Second;
        if (ep[1] < 7)
        {
          t = (uint32_t)(((uint32_t)ep[0] - 1) * 10000) + 12 * 100 + 31;
        }
        else
        {
          t = (uint32_t)((uint32_t)ep[0] * 10000) + 6 * 100 + 30;
        }
        for (i = 0; i < LEAP_NUM; i++)
        {
          // Only one leap second jump in one year
          if ((LeapListInUse[i] / 10000) == pTime->aidUtcTime.Year && pTime->aidUtcTime.Month >= 7)
          {
            break;
          }
          if ((LeapListInUse[i] / 10000) == (pTime->aidUtcTime.Year - 1) && pTime->aidUtcTime.Month < 7)
          {
            break;
          }
          if ((LeapListInUse[i] / 10000) > pTime->aidUtcTime.Year)
          {
            break;
          }
          if (LeapListInUse[i] == 0)
          {
            //new time cannot be smaller than previous one
            if (i >= 1 && t > LeapListInUse[i - 1])
            {
              LeapListInUse[i] = t;
            }
            break;
          }
        }
        if (leapsInUse[0][0] < ep[0])
        {
          for (i = MAXLEAPS; i > 0; i--)
          {
            memcpy(&(leapsInUse[i][0]), &(leapsInUse[i - 1][0]), 7 * sizeof(double));
          }
          if (ep[1] <= 6)
          {
            leapsInUse[0][0] = ep[0];
            leapsInUse[0][1] = 1;
            leapsInUse[0][2] = 1;
            leapsInUse[0][6] = leapsInUse[1][6] - 1;
          }
          else if (ep[1] <= 12)
          {
            leapsInUse[0][0] = ep[0];
            leapsInUse[0][1] = 7;
            leapsInUse[0][2] = 1;
            leapsInUse[0][6] = leapsInUse[1][6] - 1;
          }
        }
        goto LEAP_SEC_CHECK;
      }
    }
  }
#endif

  if (pTime->init == FALSE)
  {
    return;
  }


  /********************************************************************************************
  * 1. Update LeapListInUse and leapsInUse which are used for time convert
  * 2. Adjust Leap List according to GPS UTC Model
  * 3. Adjust Leap List according to KP
  *******************************************************************************************/
  if (p_Nm->gpsUtcModel.utcDeltaTls != p_Nm->gpsUtcModel.utcDeltaTlsf
    && p_Nm->gpsUtcModel.have_utc == TRUE)
  {
    utcWNlsf = p_Nm->gpsUtcModel.utcWNlsf;
    utcDN = p_Nm->gpsUtcModel.utcDN;
    utcFullWeek = (pTime->week[GPS_MODE] & 0xFF00) | (utcWNlsf & 0xFF);
    timef = gnss_gpst2time(utcFullWeek, utcDN * 86400.0); //future update day
    gnss_time2epoch(timeadd(timef, -1.0), ep); //trans to last day

    /* */
    if (ep[1] <= 6 && ep[1] > 0)
    {
      t = (uint32_t)(ep[0] * 10000 + 6.0 * 100 + 30);
    }
    else if (ep[1] <= 12 && ep[1] > 0)
    {
      t = (uint32_t)(ep[0] * 10000 + 12.0 * 100 + 31);
    }
    for (i = 0; i < LEAP_NUM; i++)
    {
      // Only one leap second jump in one year
      if ((LeapListInUse[i] / 10000.0) >= ep[0])
      {
        break;
      }
      if (LeapListInUse[i] == 0 && t > 0)
      {
        if (i >= 1 && t > LeapListInUse[i - 1])
        {
          LeapListInUse[i] = t;
        }
        break;
      }
    }
    gnss_time2epoch(timef, ep); //future day
    if (leapsInUse[0][0] < ep[0])
    {
      if (((int32_t)(-leapsInUse[0][6])) == p_Nm->gpsUtcModel.utcDeltaTls && p_Nm->gpsUtcModel.utcDeltaTls == p_Nm->gpsUtcModel.utcDeltaTlsf - 1) //do this for only once
      {
        for (i = MAXLEAPS; i > 0; i--)
        {
          memcpy(&(leapsInUse[i][0]), &(leapsInUse[i - 1][0]), 7 * sizeof(double));
        }
        for (i = 0; i < 6; i++)
        {
          leapsInUse[0][i] = ep[i];
        }
        leapsInUse[0][6] = -p_Nm->gpsUtcModel.utcDeltaTlsf;   //leapsInUse[1][6] - 1
        /*if (ep[1] <= 6)
        {
          leapsInUse[0][0] = ep[0];
          leapsInUse[0][1] = 1;
          leapsInUse[0][2] = 1;
          leapsInUse[0][6] = leapsInUse[1][6] - 1;
        }else if (ep[1] <= 12)
        {
          leapsInUse[0][0] = ep[0];
          leapsInUse[0][1] = 7;
          leapsInUse[0][2] = 1;
          leapsInUse[0][6] = leapsInUse[1][6] - 1;
        }*/
      }

    }
  }
  else if (p_Nm->glnUtcModel.kp != 0 && pTime->utcTime.Month > 0 && pTime->utcTime.Month < 13)
  {
    /*
    Leap Second Jump only happened when 6.30 or 12.31.
    KP will be set 1 or -1 before leap second jump
    */
    if (pTime->utcTime.Month <= 6)
    {
      t = pTime->utcTime.Year * 10000 + (6 * 100) + 30;
    }
    else/* if (pTime->utcTime.Month <= 12)*/
    {
      t = pTime->utcTime.Year * 10000 + (12 * 100) + 31;
    }
    for (i = 0; i < LEAP_NUM; i++)
    {
      // Only one leap second jump in one year
      if ((LeapListInUse[i] / 10000) == pTime->utcTime.Year)
      {
        break;
      }
      if (LeapListInUse[i] == 0)
      {
        if (i >= 1 && t > LeapListInUse[i - 1])
        {
          LeapListInUse[i] = t;
        }
        break;
      }
    }
    if (leapsInUse[0][0] < pTime->utcTime.Year)
    {
      for (i = MAXLEAPS; i > 0; i--)
      {
        memcpy(&(leapsInUse[i][0]), &(leapsInUse[i - 1][0]), 7 * sizeof(double));
      }
      if (pTime->utcTime.Month <= 6)
      {
        leapsInUse[0][0] = pTime->utcTime.Year;
        leapsInUse[0][1] = 7;
        leapsInUse[0][2] = 1;
        leapsInUse[0][6] = leapsInUse[1][6] - 1;
      }
      else/* if (pTime->utcTime.Month <= 12)*/
      {
        leapsInUse[0][0] = pTime->utcTime.Year + 1.0;
        leapsInUse[0][1] = 1;
        leapsInUse[0][2] = 1;
        leapsInUse[0][6] = leapsInUse[1][6] - 1;
      }
    }
  }

  // TODO: Need check whether current leap second is wrong or right

LEAP_SEC_CHECK:
  if (pTime->isLeapSecondCorrect == FALSE)
  {
    if (pTime->time_convert == GLN_TOW_FIRST)
    {

    }
    else
    {
      for (i = 0; i < pMeas->measCnt; i++)
      {
        pSvMeas = &pMeas->meas[i];
        if (pSvMeas->gnssMode == GLN_MODE &&
          pSvMeas->measState & AGGNSS_MEASUREMENT_STATE_TOW_DECODED)
        {
          pTime->isLeapSecondCorrect = TRUE;
          break;
        }
      }
    }
  }
}

/**********************************************************************
* Function Name:    gnss_tm_leap_secs
*
* Description:
*    gnss_tm_leap_secs
*
* Input:
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
uint32_t gnss_tm_leap_secs(uint8_t gnssMode, UtcTimeType* p_utc)
{
  int32_t    secs = 0;
  uint32_t    date = 0;
  int32_t    totalleaps = LEAP_NUM;
  uint32_t    leapSecond = 0;

  date = p_utc->Year * 10000 + p_utc->Month * 100 + p_utc->Day;

  for (secs = 0; secs < totalleaps; secs++)
  {
    if (!LeapListInUse[secs])
    {
      break;
    }

    if (date < LeapListInUse[secs])
    {
      break;
    }
    else if (date == LeapListInUse[secs])
    {
      if (p_utc->Hour == 23 && p_utc->Minute == 59 && p_utc->Second == 60)
      {
        leapSecond++;
      }

      break;
    }

    if (gnssMode == BDS_MODE)  /* BDS >= 20060101*/
    {
      if ((LeapListInUse[secs] / 10000) >= 2006)
      {
        leapSecond++;
      }
    }
    else
    {
      leapSecond++;
    }
  }

  return leapSecond;
}

/**********************************************************************
* Function Name:    gnss_tm_gps2utc
*
* Description:
*    gnss_tm_gps2utc
*
* Input:
*    rcvr_time: tor in second
* Return:
*
* Dependency
*      TOW and week number is saved in GNSS_TIME
*
* Author: 
**********************************************************************/
void gnss_tm_gps2utc(UtcTimeType* p_utc, uint16_t gpsWeek, double rcvr_time)
{
  uint16_t     nWeekNum;
  uint32_t     i, j;
  int32_t     leapsecs = 0;
  uint32_t     year = 0, mon = 0, day = 0, hour = 0, min = 0, secs = 0;
  double     dblGpsSec;
  int64_t     totalsecs = 0;
  int64_t     tmpsecs = 0;

  /* time adjust */
  nWeekNum = gpsWeek;
  dblGpsSec = rcvr_time;

  if (dblGpsSec > 7.0 * 24 * 3600.0)
  {
    SYS_LOGGING(OBJ_TM, LOG_WARNING, "Wrong input %s", __FUNCTION__);
    return;
  }

  totalsecs = (int64_t)nWeekNum * 7 * 24 * 60 * 60 + (int64_t)dblGpsSec;
  totalsecs += 5 * 24 * 3600;   /*GPS time start at 19800106 00Hour 00Min 00Sec */
  year = 1980;
  mon = 1;
  //day = 1;
  tmpsecs = totalsecs;

  while (tmpsecs > 0)
  {
    leapsecs = 0;

    for (i = 0; i < LEAP_NUM; i++)
    {
      if (!LeapListInUse[i])
      {
        break;
      }

      if (LeapListInUse[i] / 10000 == year)
      {
        leapsecs++;
      }
    }

    tmpsecs = tmpsecs - leapsecs - (int64_t)gnss_tm_days_in_year(year) * 24 * 3600;

    if (tmpsecs >= 0)
    {
      totalsecs = tmpsecs;
      year++;
    }
  }

  tmpsecs = totalsecs;

  for (i = 1; i <= 12; i++)
  {
    leapsecs = 0;

    for (j = 0; j < LEAP_NUM; j++)
    {
      if (!LeapListInUse[j])
      {
        break;
      }

      if (LeapListInUse[j] / 100 == (year * 100 + i))
      {
        leapsecs++;
      }
    }

    tmpsecs = tmpsecs - leapsecs - (int64_t)24 * 3600 * gnss_tm_days_in_mon(year, i);

    if (tmpsecs >= 0)
    {
      totalsecs = tmpsecs;
      mon++;
    }
    else
    {
      break;
    }
  }

  day = (int32_t)(totalsecs) / (24 * 3600) + 1;
  totalsecs = totalsecs - (uint64_t)(day - 1) * 24 * 3600;
  hour = (int32_t)(totalsecs) / 3600;
  totalsecs = totalsecs - (uint64_t)(hour) * 3600;
  min = (int32_t)(totalsecs) / 60;
  secs = (int32_t)totalsecs - min * 60;
  p_utc->Year = year;
  p_utc->Month = mon;
  p_utc->Day = day;
  p_utc->Hour = hour;
  p_utc->Minute = min;
  p_utc->Second = secs + (dblGpsSec - (uint32_t)dblGpsSec);
  return;
}

/**********************************************************************
* Function Name:    gnss_tm_gln2utc
*
* Description:
*    gnss_tm_gln2utc
*
* Input:
*    rcvr_time: tor in seconds
* Return:
*
* Dependency
*      Day in N4 and N4 are saved in GNSS_TIME
*
* Author: 
**********************************************************************/
void gnss_tm_gln2utc(UtcTimeType* p_utc, uint16_t NT, uint16_t N4, double rcvr_time)
{
  uint8_t      i;
  uint8_t      J = 0;
  uint16_t     year = 0;
  uint16_t     day_gln = 0;
  uint16_t     day = 0;
  uint16_t     day_in_month = 0;
  uint16_t     month = 0;
  double     total_sec = 0;

  if (NT >= 1 && NT <= 366)
  {
    J = 1;
  }
  else if (NT >= 367 && NT <= 731)
  {
    J = 2;
  }
  else if (NT >= 732 && NT <= 1096)
  {
    J = 3;
  }
  else if (NT >= 1097 && NT <= 1461)
  {
    J = 4;
  }

  year = 1996 + 4 * (N4 - 1) + (J - 1);
  day_gln = NT;

  if (J >= 2)
  {
    day_gln = NT - 366 - (J - 2) * 365;
  }

  day = 0;
  month = 1;

  if (year % 4 == 0)
  {
    for (i = 0; i < 12; i++)
    {
      day += leap_year_days_list[i];

      if (day_gln > day)
      {
        month++;
      }
      else
      {
        day_in_month = day_gln - (day - leap_year_days_list[i]);
        break;
      }
    }
  }
  else
  {
    for (i = 0; i < 12; i++)
    {
      day += days_list[i];

      if (day_gln > day)
      {
        month++;
      }
      else
      {
        day_in_month = day_gln - (day - days_list[i]);
        break;
      }
    }
  }

  total_sec = rcvr_time;
  total_sec -= 3.0 * 3600;         /* GLN UTC is 3 hours later than GLN time */

  if (total_sec < 0)
  {
    total_sec += 24.0 * 3600;

    if (day_in_month >= 2)
    {
      day_in_month -= 1;
    }
    else
    {
      if (month > 1)
      {
        if (year % 4 == 0)
        {
          day_in_month = day_in_month + leap_year_days_list[month - 2] - 1;
        }
        else
        {
          day_in_month = day_in_month + days_list[month - 2] - 1;
        }

        month -= 1;
      }
      else
      {
        day_in_month = 31;
        month = month + 12 - 1;
        year -= 1;
      }
    }
  }

  p_utc->Year = year;
  p_utc->Month = month;
  p_utc->Day = day_in_month;
  p_utc->Hour = (uint16_t)((uint32_t)total_sec / 3600);
  p_utc->Minute = (uint16_t)(((uint32_t)total_sec - p_utc->Hour * 3600) / 60);
  p_utc->Second = total_sec - p_utc->Hour * 3600.0 - p_utc->Minute * 60.0;
}

/**********************************************************************
* Function Name:    gnss_tm_bds2utc
*
* Description:
*    gnss_tm_bds2utc
*
* Input:
*    rcvr_time: tor in second
* Return:
*
* Dependency
*
*
* Author: 
**********************************************************************/
void gnss_tm_bds2utc(UtcTimeType* p_utc, uint16_t bdsWeek, double rcvr_time)
{
  uint16_t      nWeekNum;
  uint32_t      i, j;
  int32_t      leapsecs = 0;
  uint32_t      year = 0, mon = 0, day = 0, hour = 0, min = 0, secs = 0;
  double      dblBdsSec;
  int64_t      totalsecs = 0;
  int64_t      tmpsecs = 0;

  /*time set */
  nWeekNum = bdsWeek;
  dblBdsSec = rcvr_time;

  if (dblBdsSec > 7.0 * 24 * 3600.0)
  {
    return;
  }

  totalsecs = (int64_t)nWeekNum * 7 * 24 * 60 * 60 + (int64_t)dblBdsSec;
  year = 2006;
  mon = 1;
  //day = 1;
  tmpsecs = totalsecs;

  while (tmpsecs > 0)
  {
    leapsecs = 0;

    for (i = 0; i < LEAP_NUM; i++)
    {
      if (!LeapListInUse[i])
      {
        break;
      }

      if (LeapListInUse[i] / 10000 == year)
      {
        leapsecs++;
      }
    }

    tmpsecs = tmpsecs - leapsecs - (int64_t)gnss_tm_days_in_year(year) * 24 * 3600;

    if (tmpsecs >= 0)
    {
      totalsecs = tmpsecs;
      year++;
    }
  }

  tmpsecs = totalsecs;

  for (i = 1; i <= 12; i++)
  {
    leapsecs = 0;

    for (j = 0; j < LEAP_NUM; j++)
    {
      if (!LeapListInUse[j])
      {
        break;
      }

      if (LeapListInUse[j] / 100 == (year * 100 + i))
      {
        leapsecs++;
      }
    }

    tmpsecs = tmpsecs - leapsecs - (int64_t)24 * 3600 * gnss_tm_days_in_mon(year, i);

    if (tmpsecs >= 0)
    {
      totalsecs = tmpsecs;
      mon++;
    }
    else
    {
      break;
    }
  }

  day = (int32_t)(totalsecs) / (24 * 3600) + 1;
  totalsecs = totalsecs - (uint64_t)(day - 1) * 24 * 3600;
  hour = (int32_t)(totalsecs) / 3600;
  totalsecs = totalsecs - (uint64_t)(hour) * 3600;
  min = (int32_t)(totalsecs) / 60;
  secs = (int32_t)totalsecs - min * 60;
  p_utc->Year = year;
  p_utc->Month = mon;
  p_utc->Day = day;
  p_utc->Hour = hour;
  p_utc->Minute = min;
  p_utc->Second = secs + (dblBdsSec - (uint32_t)dblBdsSec);
  return;
}

/**********************************************************************
* Function Name:    gnss_tm_gal2utc
*
* Description:
*    gnss_tm_gal2utc
*
* Input:
*    rcvr_time: tor in second
* Return:
*
* Dependency
*
*
* Author: 
**********************************************************************/
void gnss_tm_gal2utc(UtcTimeType* p_utc, uint16_t gllWeek, double rcvr_time)
{
  uint16_t     nWeekNum;
  uint32_t     i, j;
  int32_t     leapsecs = 0;
  uint32_t     year = 0, mon = 0, day = 0, hour = 0, min = 0, secs = 0;
  double     dblGllSec;
  int64_t     totalsecs = 0;
  int64_t     tmpsecs = 0;

  /* time adjust */
  nWeekNum = gllWeek;
  dblGllSec = rcvr_time;

  if (dblGllSec > 7.0 * 24 * 3600.0)
  {
    SYS_LOGGING(OBJ_TM, LOG_WARNING, "Wrong input %s", __FUNCTION__);
    return;
  }

  totalsecs = (int64_t)nWeekNum * 7 * 24 * 60 * 60 + (int64_t)dblGllSec;
  totalsecs += 233 * 24 * 3600;   /*GAL time start at 19990822 00Hour 00Min 00Sec */
  year = 1999;
  mon = 1;
  //day = 1;

  totalsecs -= 13;                /* Leap second compensation */

  tmpsecs = totalsecs;

  while (tmpsecs > 0)
  {
    leapsecs = 0;

    for (i = 0; i < LEAP_NUM; i++)
    {
      if (!LeapListInUse[i])
      {
        break;
      }

      if (LeapListInUse[i] / 10000 == year)
      {
        leapsecs++;
      }
    }

    tmpsecs = tmpsecs - leapsecs - (int64_t)gnss_tm_days_in_year(year) * 24 * 3600;

    if (tmpsecs >= 0)
    {
      totalsecs = tmpsecs;
      year++;
    }
  }

  tmpsecs = totalsecs;

  for (i = 1; i <= 12; i++)
  {
    leapsecs = 0;

    for (j = 0; j < LEAP_NUM; j++)
    {
      if (!LeapListInUse[j])
      {
        break;
      }

      if (LeapListInUse[j] / 100 == (year * 100 + i))
      {
        leapsecs++;
      }
    }

    tmpsecs = tmpsecs - leapsecs - (int64_t)24 * 3600 * gnss_tm_days_in_mon(year, i);

    if (tmpsecs >= 0)
    {
      totalsecs = tmpsecs;
      mon++;
    }
    else
    {
      break;
    }
  }

  day = (int32_t)(totalsecs) / (24 * 3600) + 1;
  totalsecs = totalsecs - (uint64_t)(day - 1) * 24 * 3600;
  hour = (int32_t)(totalsecs) / 3600;
  totalsecs = totalsecs - (uint64_t)(hour) * 3600;
  min = (int32_t)(totalsecs) / 60;
  secs = (int32_t)totalsecs - min * 60;
  p_utc->Year = year;
  p_utc->Month = mon;
  p_utc->Day = day;
  p_utc->Hour = hour;
  p_utc->Minute = min;
  p_utc->Second = secs + (dblGllSec - (uint32_t)dblGllSec);
  return;
}

/**********************************************************************
* Function Name:    gnss_tm_utc2gps
*
* Description:
*    gnss_tm_utc2gps
*
* Input:
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnss_tm_utc2gps(UtcTimeType* p_utc, uint16_t* gpsWeek, double* rcvr_time)
{
  uint8_t      leap_year_cnt = 0;
  uint8_t      year_in_leap = 0;
  uint8_t      i;
  uint16_t     day_in_year;
  uint32_t     date = 0, leapsecs = 0;
  double     totalsecs = 0;
  double     secs_1 = 0;
  double     secs_2 = 0;
  double     sub_secs = (double)(p_utc->Second - (uint32_t)p_utc->Second);

  date = p_utc->Year * 10000 + p_utc->Month * 100 + p_utc->Day;

  if (date < 19800106)
  {
    return;
  }

  leap_year_cnt = (p_utc->Year - 1980) / 4;
  year_in_leap = (p_utc->Year - 1980) % 4;           /* 1980 is leap year */
  totalsecs = leap_year_cnt * 1461.0 * (int64_t)SECS_IN_DAY; /* seconds in four years */

  if (year_in_leap > 0)
  {
    secs_1 = 366.0 * SECS_IN_DAY + (year_in_leap - 1) * 365.0 * SECS_IN_DAY;
  }

  day_in_year = 0;

  if (year_in_leap)
  {
    for (i = 0; i < (p_utc->Month - 1); i++)
    {
      day_in_year += days_list[i];
    }
  }
  else
  {
    for (i = 0; i < (p_utc->Month - 1); i++)
    {
      day_in_year += leap_year_days_list[i];
    }
  }

  day_in_year += (p_utc->Day - 1);
  secs_2 = day_in_year * SECS_IN_DAY + p_utc->Hour * 3600.0 + p_utc->Minute * 60.0 + (uint32_t)p_utc->Second;

  leapsecs = gnss_tm_leap_secs(GPS_MODE, p_utc);

  SYS_LOGGING(OBJ_TM, LOG_INFO, "%s,LeapSecond:%d,%d", __FUNCTION__, leapsecs, p_gnssTime->leapSecond);

  totalsecs = totalsecs + secs_1 + secs_2 + leapsecs - 5.0 * SECS_IN_DAY;
  if (gpsWeek)
  {
    (*gpsWeek) = (int32_t)(totalsecs / (7.0 * 24 * 60 * 60));
  }
  if (rcvr_time)
  {
    (*rcvr_time) = ((uint64_t)totalsecs) % (7 * 24 * 60 * 60) + sub_secs;
  }
  //(*gpsWeek) = (*gpsWeek) & 0x3FF;
}


/**********************************************************************
* Function Name:    gnss_tm_utc2gln
*
* Description:
*    gnss_tm_utc2gln
*
* Input:
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnss_tm_utc2gln(UtcTimeType* p_utc, uint16_t* NT, uint16_t* N4, double* rcvr_time)
{
  uint8_t    J, i;
  uint8_t    year_cnt;
  uint16_t   day_gln;
  uint16_t   N4_gln;


  J = 1;
  year_cnt = p_utc->Year % 4;

  if (year_cnt == 0)
  {
    day_gln = 0;

    for (i = 0; i < p_utc->Month - 1; i++)
    {
      day_gln += leap_year_days_list[i];
    }

    day_gln += p_utc->Day;
  }
  else/* if (year_cnt > 0) */
  {
    day_gln = 366 + (year_cnt - 1) * 365;

    for (i = 0; i < p_utc->Month - 1; i++)
    {
      day_gln += days_list[i];
    }

    day_gln += p_utc->Day;
  }

  if (day_gln >= 1 && day_gln <= 366)
  {
    J = 1;
  }
  else if (day_gln >= 367 && day_gln <= 731)
  {
    J = 2;
  }
  else if (day_gln >= 732 && day_gln <= 1096)
  {
    J = 3;
  }
  else if (day_gln >= 1097 && day_gln <= 1461)
  {
    J = 4;
  }

  N4_gln = (p_utc->Year - 1996 - J + 1) / 4 + 1;
  (*N4) = N4_gln;
  (*NT) = day_gln;
  (*rcvr_time) = ((p_utc->Hour * 60.0 + p_utc->Minute) * 60 + p_utc->Second + 3.0 * 3600);
  if ((*rcvr_time) >= 86400)
  {
    (*rcvr_time) -= 86400;
    if (day_gln == 1461)
    {
      day_gln = 1;
      N4_gln++;
    }
    else
    {
      day_gln++;
    }
    (*N4) = N4_gln;
    (*NT) = day_gln;
  }
}

/**********************************************************************
* Function Name:    gnss_tm_utc2bds
*
* Description:
*    gnss_tm_utc2bds
*
* Input:
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnss_tm_utc2bds(UtcTimeType* p_utc, uint16_t* bdsWeek, double* rcvr_time)
{
  uint8_t       leap_year_cnt = 0;
  uint8_t       year_in_leap = 0;
  uint8_t       i;
  uint16_t      day_in_year;
  uint32_t      date = 0, leapsecs = 0;
  double      totalsecs = 0;
  double      secs_1 = 0;
  double      secs_2 = 0;
  double      sub_secs;

  if (p_utc == NULL)
  {
    return;
  }

  sub_secs = (double)(p_utc->Second - (uint32_t)p_utc->Second);
  date = p_utc->Year * 10000 + p_utc->Month * 100 + p_utc->Day;

  if (date < 20060101)
  {
    return;
  }

  if (p_utc->Year >= 2008)
  {
    leap_year_cnt = (p_utc->Year - 2008) / 4;
    year_in_leap = (p_utc->Year - 2008) % 4;
    totalsecs = leap_year_cnt * 1461.0 * SECS_IN_DAY + 365 * 2 * SECS_IN_DAY; //seconds in four years

    if (year_in_leap > 0)
    {
      secs_1 = 366 * SECS_IN_DAY + (year_in_leap - 1) * 365.0 * SECS_IN_DAY;
    }
  }
  else
  {
    totalsecs = 365.0 * SECS_IN_DAY * (p_utc->Year - 2006);
    year_in_leap = 1;
  }

  day_in_year = 0;

  if (year_in_leap)
  {
    for (i = 0; i < (p_utc->Month - 1); i++)
    {
      day_in_year += days_list[i];
    }
  }
  else
  {
    for (i = 0; i < (p_utc->Month - 1); i++)
    {
      day_in_year += leap_year_days_list[i];
    }
  }

  day_in_year += (p_utc->Day - 1);
  secs_2 = (uint64_t)day_in_year * SECS_IN_DAY + p_utc->Hour * 3600.0 + p_utc->Minute * 60.0 + (uint32_t)p_utc->Second;

  leapsecs = gnss_tm_leap_secs(BDS_MODE, p_utc);

  SYS_LOGGING(OBJ_TM, LOG_INFO, "%s,LeapSecond:%d,%d", __FUNCTION__, leapsecs, p_gnssTime->leapSecond);

  totalsecs = totalsecs + secs_1 + secs_2 + leapsecs;
  if (bdsWeek)
  {
    (*bdsWeek) = (int32_t)(totalsecs / (7.0 * 24 * 60 * 60));
    (*bdsWeek) = (*bdsWeek) & 0x1FFF;
  }
  if (rcvr_time)
  {
    (*rcvr_time) = ((uint64_t)totalsecs) % (7 * 24 * 60 * 60) + sub_secs;
  }
}

/**********************************************************************
* Function Name:    gnss_tm_utc2gal
*
* Description:
*    gnss_tm_utc2gal
*
* Input:
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnss_tm_utc2gal(UtcTimeType* p_utc, uint16_t* gllWeek, double* rcvr_time)
{
  uint8_t      leap_year_cnt = 0;
  uint8_t      year_in_leap = 0;
  uint8_t      i;
  uint16_t     day_in_year;
  uint32_t     date = 0, leapsecs = 0;
  double     totalsecs = 0;
  double     secs_1 = 0;
  double     secs_2 = 0;
  double     sub_secs = (double)(p_utc->Second - (uint32_t)p_utc->Second);

  date = p_utc->Year * 10000 + p_utc->Month * 100 + p_utc->Day;

  if (date < 19990822)
  {
    (*gllWeek) = 0;
    (*rcvr_time) = 0.0;
    return;
  }

  if (p_utc->Year >= 2000)
  {
    leap_year_cnt = (p_utc->Year - 2000) / 4;
    year_in_leap = (p_utc->Year - 2000) % 4;           /* 2000 is leap year */
    totalsecs = leap_year_cnt * 1461.0 * SECS_IN_DAY + 365 * SECS_IN_DAY;    /* seconds in four years */

    if (year_in_leap > 0)
    {
      secs_1 = 366 * SECS_IN_DAY + (year_in_leap - 1) * 365.0 * SECS_IN_DAY;
    }
  }
  else
  {
    totalsecs = 365.0 * SECS_IN_DAY * (p_utc->Year - 1999);
    year_in_leap = 1;
  }


  day_in_year = 0;

  if (year_in_leap)
  {
    for (i = 0; i < (p_utc->Month - 1); i++)
    {
      day_in_year += days_list[i];
    }
  }
  else
  {
    for (i = 0; i < (p_utc->Month - 1); i++)
    {
      day_in_year += leap_year_days_list[i];
    }
  }

  day_in_year += (p_utc->Day - 1);
  secs_2 = day_in_year * SECS_IN_DAY + p_utc->Hour * 3600.0 + p_utc->Minute * 60.0 + (uint32_t)p_utc->Second;

  leapsecs = gnss_tm_leap_secs(GPS_MODE, p_utc);

  totalsecs = totalsecs + secs_1 + secs_2 + leapsecs - 233 * SECS_IN_DAY;
  if (gllWeek)
  {
    (*gllWeek) = (int32_t)(totalsecs / (7.0 * 24 * 60 * 60));
    (*gllWeek) = (*gllWeek) & 0xFFF;
  }
  if (rcvr_time)
  {
    (*rcvr_time) = ((uint64_t)totalsecs) % (7 * 24 * 60 * 60) + sub_secs;
  }
}

/**********************************************************************
* Function Name:    gnss_tm_set_gps
*
* Description:
*    gnss_tm_set_gps
*
* Input:
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnss_tm_set_gps(uint16_t week, double tow)
{
  if (p_gnssTime->init == TRUE)
  {
    return;
  }

  p_gnssTime->week[GPS_MODE] = week;
  p_gnssTime->rcvr_time[GPS_MODE] = tow;
  p_gnssTime->timeStatus[GPS_MODE] = TM_STATUS_SET;

  gnss_tm_gps2utc(&(p_gnssTime->utcTime), p_gnssTime->week[GPS_MODE], p_gnssTime->rcvr_time[GPS_MODE]);
  gnss_tm_utc2gln(&(p_gnssTime->utcTime), &p_gnssTime->NT, &p_gnssTime->N4, &p_gnssTime->rcvr_time[GLN_MODE]);
  gnss_tm_utc2bds(&(p_gnssTime->utcTime), &p_gnssTime->week[BDS_MODE], &p_gnssTime->rcvr_time[BDS_MODE]);
  gnss_tm_utc2gal(&(p_gnssTime->utcTime), &p_gnssTime->week[GAL_MODE], &p_gnssTime->rcvr_time[GAL_MODE]);

  p_gnssTime->timeStatus[1] = p_gnssTime->timeStatus[0];
  p_gnssTime->timeStatus[2] = p_gnssTime->timeStatus[0];
  p_gnssTime->timeStatus[3] = p_gnssTime->timeStatus[0];

  p_gnssTime->init = TRUE;
  p_gnssTime->time_convert = GPS_TOW_FIRST;
}

/**********************************************************************
* Function Name:    gnss_tm_set_bds
*
* Description:
*    gnss_tm_set_bds
*
* Input:
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnss_tm_set_bds(uint16_t week, double tow)
{
  if (p_gnssTime->init == TRUE)
  {
    return;
  }

  p_gnssTime->week[BDS_MODE] = week;
  p_gnssTime->rcvr_time[BDS_MODE] = tow;
  p_gnssTime->timeStatus[BDS_MODE] = TM_STATUS_SET;

  gnss_tm_bds2utc(&(p_gnssTime->utcTime), p_gnssTime->week[BDS_MODE], p_gnssTime->rcvr_time[BDS_MODE]);
  gnss_tm_utc2gps(&(p_gnssTime->utcTime), &p_gnssTime->week[GPS_MODE], &p_gnssTime->rcvr_time[GPS_MODE]);
  gnss_tm_utc2gln(&(p_gnssTime->utcTime), &p_gnssTime->NT, &p_gnssTime->N4, &p_gnssTime->rcvr_time[GLN_MODE]);
  gnss_tm_utc2gal(&(p_gnssTime->utcTime), &p_gnssTime->week[GAL_MODE], &p_gnssTime->rcvr_time[GAL_MODE]);
  p_gnssTime->timeStatus[1] = p_gnssTime->timeStatus[2];
  p_gnssTime->timeStatus[0] = p_gnssTime->timeStatus[2];
  p_gnssTime->timeStatus[3] = p_gnssTime->timeStatus[2];

  p_gnssTime->init = TRUE;
  p_gnssTime->time_convert = BDS_TOW_FIRST;
}

/**********************************************************************
* Function Name:    gnss_tm_set_gln
*
* Description:
*    gnss_tm_set_gln
*
* Input:
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnss_tm_set_gln(uint16_t NT, uint16_t N4, double tow)
{
  if (p_gnssTime->init == TRUE)
  {
    return;
  }

  // set GLN time
  p_gnssTime->N4 = N4;
  p_gnssTime->NT = NT;
  p_gnssTime->rcvr_time[GLN_MODE] = tow;
  p_gnssTime->timeStatus[GLN_MODE] = TM_STATUS_SET;

  // convert to other system's time
  gnss_tm_gln2utc(&(p_gnssTime->utcTime), p_gnssTime->NT, p_gnssTime->N4, p_gnssTime->rcvr_time[GLN_MODE]);
  gnss_tm_utc2gps(&(p_gnssTime->utcTime), &p_gnssTime->week[GPS_MODE], &p_gnssTime->rcvr_time[GPS_MODE]);
  gnss_tm_utc2bds(&(p_gnssTime->utcTime), &p_gnssTime->week[BDS_MODE], &p_gnssTime->rcvr_time[BDS_MODE]);
  gnss_tm_utc2gal(&(p_gnssTime->utcTime), &p_gnssTime->week[GAL_MODE], &p_gnssTime->rcvr_time[GAL_MODE]);
  /*status assign assuming GLN status */
  p_gnssTime->timeStatus[0] = p_gnssTime->timeStatus[1];
  p_gnssTime->timeStatus[2] = p_gnssTime->timeStatus[1];
  p_gnssTime->timeStatus[3] = p_gnssTime->timeStatus[1];

  p_gnssTime->init = TRUE;
  p_gnssTime->time_convert = GLN_TOW_FIRST;
}

void gnss_tm_set_gal(uint16_t week, double tow)
{
  if (p_gnssTime->init == TRUE)
  {
    return;
  }

  p_gnssTime->week[GAL_MODE] = week;
  p_gnssTime->rcvr_time[GAL_MODE] = tow;
  p_gnssTime->timeStatus[GAL_MODE] = TM_STATUS_SET;

  gnss_tm_gal2utc(&(p_gnssTime->utcTime), p_gnssTime->week[GAL_MODE], p_gnssTime->rcvr_time[GAL_MODE]);
  gnss_tm_utc2gln(&(p_gnssTime->utcTime), &p_gnssTime->NT, &p_gnssTime->N4, &p_gnssTime->rcvr_time[GLN_MODE]);
  gnss_tm_utc2gps(&(p_gnssTime->utcTime), &p_gnssTime->week[GPS_MODE], &p_gnssTime->rcvr_time[GPS_MODE]);
  gnss_tm_utc2bds(&(p_gnssTime->utcTime), &p_gnssTime->week[BDS_MODE], &p_gnssTime->rcvr_time[BDS_MODE]);

  p_gnssTime->timeStatus[0] = p_gnssTime->timeStatus[3];
  p_gnssTime->timeStatus[1] = p_gnssTime->timeStatus[3];
  p_gnssTime->timeStatus[2] = p_gnssTime->timeStatus[3];

  p_gnssTime->init = TRUE;
  p_gnssTime->time_convert = GAL_TOW_FIRST;
}

uint8_t gnss_tm_check_bias_status(uint8_t gnssMode)
{
  if (p_gnssTime->timeStatus[gnssMode] >= TM_STATUS_ACCU &&
    p_gnssTime->bias_unc[gnssMode] < 50.0)
  {
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}
/**********************************************************************
* Function Name:    gnss_tm_overRound_check
*
* Description:
*    gnss_tm_set_gln
*
* Input:
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnss_tm_overRound_check(uint8_t  gnssMode, GNSS_TIME* pTime)
{
  if (gnssMode != GLN_MODE)
  {
    if (p_gnssTime->rcvr_time[gnssMode] >= (int64_t)SECS_IN_WEEK)
    {
      p_gnssTime->rcvr_time[gnssMode] -= (int64_t)SECS_IN_WEEK;
      p_gnssTime->rcvr_time_ns[gnssMode] -= (int64_t)((int64_t)SECS_IN_WEEK * 1e9);
      p_gnssTime->week[gnssMode] = p_gnssTime->week[gnssMode] + 1;
    }

    if (p_gnssTime->rcvr_time[gnssMode] < 0)
    {
      p_gnssTime->rcvr_time[gnssMode] += (int64_t)SECS_IN_WEEK;
      p_gnssTime->rcvr_time_ns[gnssMode] += (int64_t)((int64_t)SECS_IN_WEEK * 1e9);
      p_gnssTime->week[gnssMode] = p_gnssTime->week[gnssMode] - 1;
    }
  }
  else
  {
    if (p_gnssTime->rcvr_time[gnssMode] >= SECS_IN_DAY)
    {
      p_gnssTime->rcvr_time[gnssMode] -= SECS_IN_DAY;
      p_gnssTime->rcvr_time_ns[gnssMode] -= (int64_t)(SECS_IN_DAY * 1e9);
      p_gnssTime->NT += 1;

      if (p_gnssTime->NT > DAY_IN_FOUR_YEAR)
      {
        p_gnssTime->NT -= DAY_IN_FOUR_YEAR;
        p_gnssTime->N4 += 1;
      }
    }

    if (p_gnssTime->rcvr_time[gnssMode] < 0)
    {
      p_gnssTime->rcvr_time[gnssMode] += SECS_IN_DAY;
      p_gnssTime->rcvr_time_ns[gnssMode] += (int64_t)(SECS_IN_DAY * 1e9);

      if (p_gnssTime->NT > 1) // remove Day < 0 check since Day is U16
      {
        p_gnssTime->NT -= 1;
      }
      else
      {
        p_gnssTime->NT = DAY_IN_FOUR_YEAR;
        p_gnssTime->N4 -= 1;
      }
    }
  }
}

/**********************************************************************
* Function Name:    gnss_tm_propagate
*
* Description:
*    gnss_tm_propagate is used for time propagation
*
* Input:
*    tor: current tor
*
* Return:
*
* Dependency
*      None
*
* Author: 
**********************************************************************/
void gnss_tm_propagate(double tor, uint8_t flt_reverse)
{
  uint8_t         msCorrFlag = FALSE;
  uint32_t        i, j;
  double        dt = 0.0;
  int32_t        integer_msec_bias = 0;
  double        bias_in_msecs = 0.0, fract_msec_bias = 0.0;
  //int64_t        integer_nsec_bias = 0;
  //double        bias_in_nsecs = 0.0/*, fract_nsec_bias = 0.0*/;
  int64_t        dtNs = 0;

  if (!flt_reverse)
  {
    p_gnssTime->dt = -1.0;
  }
  else
  {
    p_gnssTime->dt = 1.0;
  }
  if (p_gnssTime->init == FALSE)
  {
    p_gnssTime->tor = tor;
    p_gnssTime->last_time_ns = p_gnssTime->time_ns;
    p_gnssTime->dt = 0.0;
    return;
  }

  // calculate dt
  /*forward filter mode*/
  if (!flt_reverse)
  {
    if (tor >= p_gnssTime->tor)
    {
      dt = tor - p_gnssTime->tor;
      dtNs = p_gnssTime->time_ns - p_gnssTime->last_time_ns;
      if (dt > 3600.0/*SECS_IN_DAY*/ && g_pe_cfg.chipType == UBLOX && tor > 0.0 && p_gnssTime->tor > 0.0)  //abnormal tor jump
      {
        GLOGW("ublox abnormal tor jump: dt=%.6f tor=%.6f lasttor=%.6f", dt, tor, p_gnssTime->tor);
        p_gnssTime->init = FALSE;
        return;
      }
    }
    else if (g_pe_cfg.chipType == UBLOX && p_gnssTime->tor > ((int64_t)SECS_IN_WEEK - 10.0) && p_gnssTime->tor < (int64_t)SECS_IN_WEEK && tor >= 0.0 && tor < 10.0) //cross week
    {
      dt = tor + (int64_t)SECS_IN_WEEK - p_gnssTime->tor;
      dtNs = (int64_t)(p_gnssTime->time_ns + ((int64_t)SECS_IN_WEEK * 1e9) - p_gnssTime->last_time_ns);
    }
    else
    {
      GLOGW("TimeNs BackWards!");
      p_gnssTime->init = FALSE;
      rtkctrl.ptor = 0.0;
      p_gnssTime->week_save[GPS_MODE] = p_gnssTime->week[GPS_MODE];
      p_gnssTime->week_save[BDS_MODE] = p_gnssTime->week[BDS_MODE];
      return;
    }
  }
  else
  {
    if (tor <= p_gnssTime->tor && tor > 0)
    {
      dt = tor - p_gnssTime->tor;
      dtNs = p_gnssTime->time_ns - p_gnssTime->last_time_ns;
      if (dt < -3600.0/*SECS_IN_DAY*/ && g_pe_cfg.chipType == UBLOX && tor > 0.0 && p_gnssTime->tor > 0.0)  //abnormal tor jump
      {
        GLOGW("ublox abnormal tor jump: dt=%.6f tor=%.6f lasttor=%.6f", dt, tor, p_gnssTime->tor);
        p_gnssTime->init = FALSE;
        return;
      }
    }
    else if (g_pe_cfg.chipType == UBLOX && tor > ((int64_t)SECS_IN_WEEK - 10.0) && tor < (int64_t)SECS_IN_WEEK && p_gnssTime->tor >= 0.0 && p_gnssTime->tor < 10.0) //cross week
    {
      dt = tor - (int64_t)SECS_IN_WEEK - p_gnssTime->tor;
      dtNs = (int64_t)(p_gnssTime->time_ns - ((int64_t)SECS_IN_WEEK * 1e9) - p_gnssTime->last_time_ns);
    }
    else
    {
      GLOGW("TimeNs BackWards!");
      p_gnssTime->init = FALSE;
      rtkctrl.ptor = 0.0;
      p_gnssTime->week_save[GPS_MODE] = p_gnssTime->week[GPS_MODE];
      p_gnssTime->week_save[BDS_MODE] = p_gnssTime->week[BDS_MODE];
      return;
    }

  }
  p_gnssTime->tor = tor;
  p_gnssTime->last_time_ns = p_gnssTime->time_ns;
  p_gnssTime->dt = dt;

  for (i = 0; i < GNSS_MAX_MODE; i++)
  {
    /* Update tor, tor_new = tor_old + delta_scount */
    p_gnssTime->rcvr_time[i] += (dt + p_gnssTime->time_ns_bias);
    p_gnssTime->rcvr_time_ns[i] += dtNs;

    //check whehter cross week/day
    gnss_tm_overRound_check(i, p_gnssTime);
    /* No matter drift is known or unknown, convert GPS time to UTC time */
    if (i == GPS_MODE)
    {
      p_gnssTime->GPSTime = gnss_gpst2time(p_gnssTime->week[i], p_gnssTime->rcvr_time[i]);
      p_gnssTime->GLNTime = timeadd(gpst2utc(p_gnssTime->GPSTime), 3 * 3600);
      p_gnssTime->BDSTime = timeadd(p_gnssTime->GPSTime, -14.0);
      gnss_tm_gps2utc(&(p_gnssTime->utcTime), p_gnssTime->week[GPS_MODE], p_gnssTime->rcvr_time[GPS_MODE]);
    }

    /* Only when bias and drift is known, then we can propagate the bias and clock drift */
    if (p_gnssTime->driftStatus == DRIFT_UNKNOWN || p_gnssTime->timeStatus[i] < TM_STATUS_ACCU)
    {
      continue;
    }

    /*
    update bias:
    bias(n) = bias(n-1) + drift * delta_scount
    bias(n) > 0.5ms      --> bias(n) -= 1;
    bias(n) < -0.5ms     --> bias(n) += 1;
    */
    bias_in_msecs = (double)1.0 / LIGHT_MSEC * (p_gnssTime->bias[i] + p_gnssTime->drift * dt);
    if (g_pe_cfg.chipType == QCOM || g_pe_cfg.chipType == SPRD)
    {
      integer_msec_bias = (int32_t)bias_in_msecs;
      fract_msec_bias = bias_in_msecs - (double)integer_msec_bias;

      if (fract_msec_bias > (double)0.6)
      {
        fract_msec_bias -= (double)1.0;
        integer_msec_bias++;
      }
      else if (fract_msec_bias < (double)(-0.6))
      {
        fract_msec_bias += (double)1.0;
        integer_msec_bias--;
      }
    }
    p_gnssTime->msec_correction[i] = 0;

    // ns process
    /*bias_in_nsecs = (double)1.0 / LIGHT_NSEC * (p_gnssTime->bias[i] + p_gnssTime->drift * dt);
    integer_nsec_bias = (int64_t)bias_in_nsecs;*/
    //fract_nsec_bias = bias_in_nsecs - integer_nsec_bias;

    if (integer_msec_bias)
    {
      if (g_pe_cfg.chipType == QCOM || g_pe_cfg.chipType == SPRD || g_pe_cfg.chipType == UBLOX)
      {
        if (msCorrFlag == FALSE)
        {
          for (j = 0; j < GNSS_MAX_MODE; j++)
          {
            /* Local time adjust */
            p_gnssTime->rcvr_time[j] -= integer_msec_bias * 0.001;
            p_gnssTime->msec_correction[j] = -integer_msec_bias;

            /* Local ns adjust */
            p_gnssTime->rcvr_time_ns[j] -= (int64_t)(integer_msec_bias * 1e6);
            if (i != j)
            {
              p_gnssTime->bias[j] -= integer_msec_bias * LIGHT_MSEC;
            }
            if (p_gnssTime->timeStatus[j] >= TM_STATUS_ACCU)
            {
              gnss_Kf_BiasCorrection(integer_msec_bias, j);
              gnss_Ls_BiasCorrection(j, integer_msec_bias, gpz_Ls);
            }

            if (j == GPS_MODE)
            {
              p_gnssTime->GPSTime = gnss_gpst2time(p_gnssTime->week[j], p_gnssTime->rcvr_time[j]);
              p_gnssTime->GLNTime = timeadd(gpst2utc(p_gnssTime->GPSTime), 3 * 3600);
              p_gnssTime->BDSTime = timeadd(p_gnssTime->GPSTime, -14.0);
              gnss_tm_gps2utc(&(p_gnssTime->utcTime), p_gnssTime->week[GPS_MODE], p_gnssTime->rcvr_time[GPS_MODE]);
            }
          }
          msCorrFlag = TRUE;
          if (rtkctrl.kfstatus & RTK_KF_RUN)
          {
            rtkctrl.kfstatus |= RTK_KF_RESET_AMB;
            GLOGW("rtk kf reset amb because local time adjust");
          }
        }
      }
      gnss_tm_overRound_check(i, p_gnssTime);
    }

    if (g_pe_cfg.chipType == UBLOX)
    {
      p_gnssTime->bias[i] = (double)(bias_in_msecs * LIGHT_MSEC);
    }
    else if (g_pe_cfg.chipType == QCOM || g_pe_cfg.chipType == SPRD)
    {
      p_gnssTime->bias[i] = (double)(fract_msec_bias * LIGHT_MSEC);
    }

    p_gnssTime->bias_unc[i] += (float)(p_gnssTime->drift_unc * fabs(dt));

    /* bias uncertainty constrain to 1s */
    if (p_gnssTime->bias_unc[i] > (float)LIGHT_SEC)
    {
      p_gnssTime->bias_unc[i] = (float)LIGHT_SEC;
    }

    p_gnssTime->drift_unc += (float)(G_DIV * fabs(dt));

    /* Drift uncertainty constrain to 750 m/s */
    if (p_gnssTime->drift_unc > MAX_CLOCK_FREQ_DRIFT)
    {
      p_gnssTime->drift_unc = MAX_CLOCK_FREQ_DRIFT;
    }

    /* time status change
    1) when bias_unc is large than 10ms, TM_STATUS_SET --> TM_STATUS_APPX
    2) when bias_unc is large than 0.1ms, TM_STATUS_ACCU --> TM_STATUS_SET
    */
    if (p_gnssTime->timeStatus[i] == TM_STATUS_APPX)
    {
      // when time is APPX, then time uncertainty should be 2~3 seconds
    }
    else if (p_gnssTime->timeStatus[i] == TM_STATUS_SET && p_gnssTime->bias_unc[i] > OSC_BIAS_SET_THRESH)
    {
      p_gnssTime->timeStatus[i] = TM_STATUS_APPX;
    }
    else if (p_gnssTime->timeStatus[i] == TM_STATUS_ACCU && p_gnssTime->bias_unc[i] > OSC_BIAS_KNOWN_THRESH)
    {
      p_gnssTime->timeStatus[i] = TM_STATUS_SET;
    }
  }
}

/**
 * @brief  get code compile time in UTC
 * @author caizhijie
 * @date   11/25 16:26:43
 */
int gnss_tm_compile_time(double* tow)
{
  static uint16_t  compile_week = 0;
  static double compile_tow = 0.0;

  if (compile_week > 0)
  {
    *tow = compile_tow;
    return compile_week;
  }

  UtcTimeType utc = { 0 };
  int iYear = 0;
  int iMonth = 0;
  int iDay = 0;
  int iHour = 0;
  int iMin = 0;
  int iSec = 0;
  int  MONTH_PER_YEAR = 12;
  char szEnglishMonth[12][4] = { "Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec" };
  char szTmpDate[40] = { 0 };
  char szTmpTime[20] = { 0 };
  char szMonth[4] = { 0 };

  sprintf(szTmpDate, "%s", __DATE__); //"Sep 18 2010"
  sprintf(szTmpTime, "%s", __TIME__);    //"10:59:19"

  (void)sscanf(szTmpDate, "%s %d %d", szMonth, &iDay, &iYear);
  (void)sscanf(szTmpTime, "%d:%d:%d", &iHour, &iMin, &iSec);

  for (int i = 0; MONTH_PER_YEAR; i++)
  {
    if (strncmp(szMonth, szEnglishMonth[i], 3) == 0)
    {
      iMonth = i + 1;
      break;
    }
  }

  utc.Year = iYear;
  utc.Month = iMonth;
  utc.Day = iDay;
  utc.Hour = iHour;
  utc.Minute = iMin;
  utc.Second = (double)iSec;

  gnss_tm_utc2gps(&utc, &compile_week, &compile_tow);

  /* minus 8hour from beijing time to utc time */
  compile_tow = compile_tow - 8.0 * 3600;
  if (compile_tow < 0) {
    compile_tow += (int64_t)SECS_IN_WEEK;
    compile_week--;
  }

  *tow = compile_tow;
  return compile_week;
}
