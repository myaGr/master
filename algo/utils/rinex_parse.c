/**@file        rinex_parse.h
 * @brief       RINEX stream parse module
 * @author      wuzhiyuan
 * @date        2023/09/06
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/09/06  <td>0.1      <td>wuzhiyuan   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "gnss_def.h"
#include "rinex_parse.h"
#include "cmn_utils.h"
#include "gnss_common.h"
#include "mw_log.h"

 /**
  * @brief get information of observation such as observed value,snr,signal type
  * @param[in]  u_constellation represent the constellation
  * @param[in]  u_svid represent the index of satellite
  * @param[in]  pu_id represent the type of observation
  * @param[in]  pu_buff represent the observation
  * @param[in]  pz_obs represent the struct of observation
  * @return     None
  */
static void rinex_getObsInfor(uint8_t u_constellation, uint8_t u_svid, char* pu_id, char* pu_buff, RinexObs_t* pz_obs)
{
  char pu_type[3] = "";
  char pu_channel[2] = "";
  sprintf(pu_type, "%.2s", pu_id);
  sprintf(pu_channel, "%.1s", pu_id + 2);
  pz_obs->sys = u_constellation;
  pz_obs->svid = u_svid;
  if (strstr(pu_type, "C"))
  {
    pz_obs->pseudorange = atof(pu_buff);
  }
  else if (strstr(pu_type, "L"))
  {
    pz_obs->carrier_phase = atof(pu_buff);
  }
  else if (strstr(pu_type, "D"))
  {
    pz_obs->doppler = atof(pu_buff);
  }
  else if (strstr(pu_type, "S"))
  {
    pz_obs->snr = (uint8_t)atof(pu_buff);
  }

  switch (u_constellation)
  {
  case C_GNSS_GPS:
    if (strstr(pu_type, "1"))
    {
      pz_obs->signal = C_GNSS_SIG_GPS_L1C;
    }
    else if (strstr(pu_type, "2"))
    {
      pz_obs->signal = C_GNSS_SIG_GPS_L2C;
    }
    else if (strstr(pu_type, "5"))
    {
      pz_obs->signal = C_GNSS_SIG_GPS_L5Q;
    }
    else
    {
      pz_obs->signal = C_GNSS_SIG_MAX;
    }
    break;
  case C_GNSS_BDS3:
    if ((strstr(pu_type, "1") || strstr(pu_type, "2")) && strstr(pu_channel, "I"))
    {
      pz_obs->signal = C_GNSS_SIG_BDS_B1I;
    }
    else if (strstr(pu_type, "6"))
    {
      pz_obs->signal = C_GNSS_SIG_BDS_B3I;
    }
    else if (strstr(pu_type, "5"))
    {
      pz_obs->signal = C_GNSS_SIG_BDS_B2A;
    }
    else
    {
      pz_obs->signal = C_GNSS_SIG_MAX;
    }
    break;
  case C_GNSS_GAL:
    if (strstr(pu_type, "1"))
    {
      pz_obs->signal = C_GNSS_SIG_GAL_E1;
    }
    else if (strstr(pu_type, "5"))
    {
      pz_obs->signal = C_GNSS_SIG_GAL_E5A;
    }
    else if (strstr(pu_type, "7"))
    {
      pz_obs->signal = C_GNSS_SIG_GAL_E5B;
    }
    else
    {
      pz_obs->signal = C_GNSS_SIG_MAX;
    }
    break;
  case C_GNSS_QZS:
    if (strstr(pu_type, "1"))
    {
      pz_obs->signal = C_GNSS_SIG_QZS_L1C;
    }
    else if (strstr(pu_type, "2"))
    {
      pz_obs->signal = C_GNSS_SIG_QZS_L2C;
    }
    else if (strstr(pu_type, "5"))
    {
      pz_obs->signal = C_GNSS_SIG_QZS_L5Q;
    }
    else
    {
      pz_obs->signal = C_GNSS_SIG_MAX;
    }
    break;
  }
}

/**
 * @brief get the index in RinexDecoder_t acoording the constellation,prn and signal type
 * @param[in]  pz_obs represent the struct of RINEX observation
 * @param[in]  pz_RinexDecoder represent the RINEX information
 * @param[in]  pq_obsNum represent the num of obs in current satellite
 * @return     tne index of RinexDecoder_t
 */
static int8_t obainRinexDataIndex(RinexObs_t* pz_obs, RinexDecoder_t* pz_RinexDecoder, int32_t* pq_obsNum)
{
  int32_t q_index = 0;
  for (q_index = (pz_RinexDecoder->nobs - *pq_obsNum); q_index < (pz_RinexDecoder->nobs); q_index++)
  {
    if (pz_obs->sys == pz_RinexDecoder->obs[q_index].sys && pz_obs->svid == pz_RinexDecoder->obs[q_index].svid && pz_obs->signal == pz_RinexDecoder->obs[q_index].signal)
    {
      pz_RinexDecoder->nobs--;
      (*pq_obsNum)--;

      return q_index;
    }
  }
  return (int32_t)(pz_RinexDecoder->nobs);
}

/**
 * @brief set string without tail space
 * @param[in]  pu_obsCode represent the type of observation
 * @param[in]  pu_buff represent the type reading from RINEX file
 * @param[in]  u_len represent the lenth of buff
 * @return     None
 */
static void setstr(char* pu_obsCode, const char* pu_buff, uint8_t u_len)
{
  char* pu_buff0 = pu_obsCode;
  const char* pu_buff1 = pu_buff;
  while (*pu_buff1 && pu_buff1 < pu_buff + u_len) *pu_buff0++ = *pu_buff1++;
  *pu_buff0-- = '\0';
  while (pu_buff0 >= pu_obsCode && *pu_buff0 == ' ') *pu_buff0-- = '\0';
}

/**
 * @brief parse broadcast ephemeris for each system
 * @param[in]  u_constellation represent the constellation
 * @param[in]  u_svid represent the satellite PRN
 * @param[in]  pd_data represent the broadcast ephemeris
 * @param[in]  pq_epoch represent the Epoch time
 * @param[in]  pz_RinexDecoder represent the RINEX information
 * @return     status -- 1: parse success, other: fail
 */
int8_t rinex_dec_eph(uint8_t u_constellation, uint8_t u_svid, const double* pd_data, uint32_t* pq_epoch, RinexDecoder_t* pz_RinexDecoder)
{
  UtcTime_t z_utc = { 0 };
  GpsTime_t z_gpst = { 0 };
  BdsTime_t z_bdt = { 0 };
  GalTime_t z_gst = { 0 };
  gnss_Ephemeris_t z_Eph = { 0 };
  EpochTime_t z_epoch = { pq_epoch[0],pq_epoch[1],pq_epoch[2],pq_epoch[3],pq_epoch[4],(float)pq_epoch[5] };

  tm_cvt_EpochToUtcTime(&z_epoch, &z_utc);
  tm_cvt_UtcTimeToGpst(&z_utc, &z_gpst);
  if (u_constellation == C_GNSS_GLO || u_constellation == C_GNSS_NONE)
  {
    return 0;
  }
  pz_RinexDecoder->ephemeris.u_constellation = u_constellation;
  if (u_constellation == C_GNSS_GPS)
  {
    z_Eph.eph.z_gpsEph.svid = u_svid;
    z_Eph.eph.z_gpsEph.week = (uint16_t)pd_data[21];
    z_Eph.eph.z_gpsEph.accuracy = (uint8_t)pd_data[23];
    z_Eph.eph.z_gpsEph.health = (uint8_t)pd_data[24];
    z_Eph.eph.z_gpsEph.iodc = (uint16_t)pd_data[26];
    z_Eph.eph.z_gpsEph.code = (uint8_t)pd_data[20];
    z_Eph.eph.z_gpsEph.fit = (uint8_t)pd_data[28];
    z_Eph.eph.z_gpsEph.iode = (uint8_t)pd_data[3];
    z_Eph.eph.z_gpsEph.tgd = pd_data[25];
    z_Eph.eph.z_gpsEph.toc = pd_data[11];
    z_Eph.eph.z_gpsEph.toe = pd_data[11];
    z_Eph.eph.z_gpsEph.sqrt_A = pd_data[10];
    z_Eph.eph.z_gpsEph.e = pd_data[8];
    z_Eph.eph.z_gpsEph.M0 = pd_data[6];
    z_Eph.eph.z_gpsEph.Omega0 = pd_data[13];
    z_Eph.eph.z_gpsEph.Omega = pd_data[17];
    z_Eph.eph.z_gpsEph.OmegaDot = pd_data[18];
    z_Eph.eph.z_gpsEph.i0 = pd_data[15];
    z_Eph.eph.z_gpsEph.idot = pd_data[19];
    z_Eph.eph.z_gpsEph.DeltaN = pd_data[5];
    z_Eph.eph.z_gpsEph.crc = pd_data[16];
    z_Eph.eph.z_gpsEph.crs = pd_data[4];
    z_Eph.eph.z_gpsEph.cuc = pd_data[7];
    z_Eph.eph.z_gpsEph.cus = pd_data[9];
    z_Eph.eph.z_gpsEph.cic = pd_data[12];
    z_Eph.eph.z_gpsEph.cis = pd_data[14];
    z_Eph.eph.z_gpsEph.af0 = pd_data[0];
    z_Eph.eph.z_gpsEph.af1 = pd_data[1];
    z_Eph.eph.z_gpsEph.af2 = pd_data[2];
    if (pd_data[21] > 0 && pd_data[21] < 2048)
    {
      z_Eph.eph.z_gpsEph.week = 2048 + z_Eph.eph.z_gpsEph.week % 1024;
    }
    tm_cvt_SetGpst(&pz_RinexDecoder->ephemeris.z_toeOfGpst, z_Eph.eph.z_gpsEph.week, z_Eph.eph.z_gpsEph.toe);
  }
  else if (u_constellation == C_GNSS_BDS3)
  {
    z_Eph.eph.z_bdsEph.svid = u_svid;
    z_Eph.eph.z_bdsEph.week = (uint16_t)pd_data[21];
    z_Eph.eph.z_bdsEph.accuracy = (uint8_t)pd_data[23];
    z_Eph.eph.z_bdsEph.health = (uint8_t)pd_data[24];
    z_Eph.eph.z_bdsEph.iodc = (uint16_t)pd_data[28];
    z_Eph.eph.z_bdsEph.iode = (uint8_t)pd_data[3];
    z_Eph.eph.z_bdsEph.tgdB1I = pd_data[25];
    z_Eph.eph.z_bdsEph.tgdB2I = pd_data[26];
    z_Eph.eph.z_bdsEph.toc = pd_data[11];
    z_Eph.eph.z_bdsEph.toe = pd_data[11];
    z_Eph.eph.z_bdsEph.sqrt_A = pd_data[10];
    z_Eph.eph.z_bdsEph.e = pd_data[8];
    z_Eph.eph.z_bdsEph.M0 = pd_data[6];
    z_Eph.eph.z_bdsEph.Omega0 = pd_data[13];
    z_Eph.eph.z_bdsEph.Omega = pd_data[17];
    z_Eph.eph.z_bdsEph.OmegaDot = pd_data[18];
    z_Eph.eph.z_bdsEph.i0 = pd_data[15];
    z_Eph.eph.z_bdsEph.idot = pd_data[19];
    z_Eph.eph.z_bdsEph.DeltaN = pd_data[5];
    z_Eph.eph.z_bdsEph.crc = pd_data[16];
    z_Eph.eph.z_bdsEph.crs = pd_data[4];
    z_Eph.eph.z_bdsEph.cuc = pd_data[7];
    z_Eph.eph.z_bdsEph.cus = pd_data[9];
    z_Eph.eph.z_bdsEph.cic = pd_data[12];
    z_Eph.eph.z_bdsEph.cis = pd_data[14];
    z_Eph.eph.z_bdsEph.af0 = pd_data[0];
    z_Eph.eph.z_bdsEph.af1 = pd_data[1];
    z_Eph.eph.z_bdsEph.af2 = pd_data[2];

    tm_cvt_SetBdt(&z_bdt, z_Eph.eph.z_bdsEph.week, z_Eph.eph.z_bdsEph.toe);
    tm_cvt_BdtToGpst(&z_bdt, &pz_RinexDecoder->ephemeris.z_toeOfGpst);
  }
  else if (u_constellation == C_GNSS_GAL)
  {
    z_Eph.eph.z_galEph.svid = u_svid;
    z_Eph.eph.z_galEph.week = (uint16_t)(pd_data[21] - 1024);
    z_Eph.eph.z_galEph.accuracy = (uint8_t)pd_data[23];
    z_Eph.eph.z_galEph.E1health = (uint8_t)pd_data[24];
    z_Eph.eph.z_galEph.E1DVS = (uint8_t)pd_data[24];
    z_Eph.eph.z_galEph.E5bhealth = (uint8_t)pd_data[24];
    z_Eph.eph.z_galEph.E5bDVS = (uint8_t)pd_data[24];
    z_Eph.eph.z_galEph.iode = (uint8_t)pd_data[3];;
    z_Eph.eph.z_galEph.code = (uint8_t)pd_data[20];
    z_Eph.eph.z_galEph.tgdE1E5a = pd_data[25];
    z_Eph.eph.z_galEph.tgdE1E5b = pd_data[26];
    z_Eph.eph.z_galEph.toc = pd_data[11];
    z_Eph.eph.z_galEph.toe = pd_data[11];
    z_Eph.eph.z_galEph.sqrt_A = pd_data[10];
    z_Eph.eph.z_galEph.e = pd_data[8];
    z_Eph.eph.z_galEph.M0 = pd_data[6];
    z_Eph.eph.z_galEph.Omega0 = pd_data[13];
    z_Eph.eph.z_galEph.Omega = pd_data[17];
    z_Eph.eph.z_galEph.OmegaDot = pd_data[18];
    z_Eph.eph.z_galEph.i0 = pd_data[15];
    z_Eph.eph.z_galEph.idot = pd_data[19];
    z_Eph.eph.z_galEph.DeltaN = pd_data[5];
    z_Eph.eph.z_galEph.crc = pd_data[16];
    z_Eph.eph.z_galEph.crs = pd_data[4];
    z_Eph.eph.z_galEph.cuc = pd_data[7];
    z_Eph.eph.z_galEph.cus = pd_data[9];
    z_Eph.eph.z_galEph.cic = pd_data[12];
    z_Eph.eph.z_galEph.cis = pd_data[14];
    z_Eph.eph.z_galEph.af0 = pd_data[0];
    z_Eph.eph.z_galEph.af1 = pd_data[1];
    z_Eph.eph.z_galEph.af2 = pd_data[2];

    tm_cvt_SetGst(&z_gst, z_Eph.eph.z_galEph.week, z_Eph.eph.z_galEph.toe);
    tm_cvt_GstToGpst(&z_gst, &pz_RinexDecoder->ephemeris.z_toeOfGpst);
  }
  else if (u_constellation == C_GNSS_QZS)
  {
    z_Eph.eph.z_qzsEph.svid = u_svid;
    z_Eph.eph.z_qzsEph.week = (uint16_t)pd_data[21];
    z_Eph.eph.z_qzsEph.accuracy = (uint8_t)pd_data[23];
    z_Eph.eph.z_qzsEph.health = (uint8_t)pd_data[24];
    z_Eph.eph.z_qzsEph.iodc = (uint16_t)pd_data[26];
    z_Eph.eph.z_qzsEph.code = (uint8_t)pd_data[20];
    z_Eph.eph.z_qzsEph.fit = pd_data[28] == 0.0 ? 1 : 2;
    z_Eph.eph.z_qzsEph.iode = (uint8_t)pd_data[3];
    z_Eph.eph.z_qzsEph.tgd = pd_data[25];
    z_Eph.eph.z_qzsEph.toc = pd_data[11];
    z_Eph.eph.z_qzsEph.toe = pd_data[11];
    z_Eph.eph.z_qzsEph.sqrt_A = pd_data[10];
    z_Eph.eph.z_qzsEph.e = pd_data[8];
    z_Eph.eph.z_qzsEph.M0 = pd_data[6];
    z_Eph.eph.z_qzsEph.Omega0 = pd_data[13];
    z_Eph.eph.z_qzsEph.Omega = pd_data[17];
    z_Eph.eph.z_qzsEph.OmegaDot = pd_data[18];
    z_Eph.eph.z_qzsEph.i0 = pd_data[15];
    z_Eph.eph.z_qzsEph.idot = pd_data[19];
    z_Eph.eph.z_qzsEph.DeltaN = pd_data[5];
    z_Eph.eph.z_qzsEph.crc = pd_data[16];
    z_Eph.eph.z_qzsEph.crs = pd_data[4];
    z_Eph.eph.z_qzsEph.cuc = pd_data[7];
    z_Eph.eph.z_qzsEph.cus = pd_data[9];
    z_Eph.eph.z_qzsEph.cic = pd_data[12];
    z_Eph.eph.z_qzsEph.cis = pd_data[14];
    z_Eph.eph.z_qzsEph.af0 = pd_data[0];
    z_Eph.eph.z_qzsEph.af1 = pd_data[1];
    z_Eph.eph.z_qzsEph.af2 = pd_data[2];
    if (pd_data[21] > 0 && pd_data[21] < 2048)
    {
      z_Eph.eph.z_qzsEph.week = 2048 + z_Eph.eph.z_qzsEph.week % 1024;
    }
    tm_cvt_SetGpst(&pz_RinexDecoder->ephemeris.z_toeOfGpst, z_Eph.eph.z_qzsEph.week, z_Eph.eph.z_qzsEph.toe);
  }
  pz_RinexDecoder->q_towMsec = z_gpst.q_towMsec;
  memcpy(&pz_RinexDecoder->ephemeris, &z_Eph, sizeof(GpsEphemeris_t));
  return 1;
}

/**
 * @brief match broadcast ephemeris for AG format
 * @param[in]  q_towMsec represent the tow time of observaton
 * @param[in]  q_towEph represent the tow time of ephemeris
 * @param[in]  u_constellation
 * @return     status -- TRUE: match success, FALSE: fail
 */
BOOL rinex_ephMatch_AG(uint32_t q_towMsec, uint32_t q_towEph, uint8_t u_constellation)
{
  int32_t q_tmax = 0;
  int32_t q_t = (int32_t)q_towMsec - (int32_t)q_towEph;
  switch (u_constellation)
  {
  case C_GNSS_GPS:
    q_tmax = 7200 * 1000;
    break;
  case C_GNSS_BDS3:
    q_tmax = 3600 * 1000;
    break;
  case C_GNSS_GAL:
    q_tmax = 600 * 1000;
    break;
  case C_GNSS_QZS:
    q_tmax = 3600 * 1000;
    break;
  }
  if (q_t <= q_tmax)
  {
    return TRUE;
  }
  return FALSE;
}

/**
 * @brief convert character to system in RINEX file
 * @param[in]  pu_buff represent the character such as "G" or "E"
 * @return     the index of system in RTK2.0
 */
uint8_t rinex_CharToSys(char* pu_buff)
{
  if (strstr(pu_buff, "G"))
  {
    return C_GNSS_GPS;
  }
  else if (strstr(pu_buff, "R"))
  {
    return C_GNSS_GLO;
  }
  else if (strstr(pu_buff, "C"))
  {
    return C_GNSS_BDS3;
  }
  else if (strstr(pu_buff, "E"))
  {
    return C_GNSS_GAL;
  }
  else if (strstr(pu_buff, "J"))
  {
    return C_GNSS_QZS;
  }
  else
  {
    return C_GNSS_NONE;
  }
}

/**
 * @brief get information from the head of RINEX file
 * @param[in]  pz_RinexDecoder represent the RINEX information
 * @param[in]  pu_buff represent the message of RINEX file
 * @param[in]  pu_obsCode represent the code information of observation
 * @param[in]  pu_obsCodeNum represent the number of observation type
 * @return     status -- -1: still in the head of file, 0: leave the head of file
 */
int8_t rinex_dec_obsf(RinexDecoder_t* pz_RinexDecoder, char* pu_buff, char pu_obsCode[][32][4], uint8_t* pu_obsCodeNum)
{
  uint8_t u_index = 0;
  uint8_t u_sys = -1;
  char* pu_buff0 = (char*)malloc(sizeof(char));
  if (strstr(pu_buff, "MARKER NAME"))
  {
    pz_RinexDecoder->satInfo.w_staid = atoi(pu_buff);
  }
  if (strstr(pu_buff, "APPROX POSITION XYZ"))
  {
    u_index = 0;
    pu_buff0 = strtok(pu_buff, " ");
    while (pu_buff0 != NULL && u_index < 3)
    {
      pz_RinexDecoder->satInfo.d_staPos[u_index++] = atof(pu_buff0);
      pu_buff0 = strtok(NULL, " ");
    }
    return -1;
  }
  if (strstr(pu_buff, "ANTENNA: DELTA H/E/N"))
  {
    u_index = 0;
    pu_buff0 = strtok(pu_buff, " ");
    while (pu_buff0 != NULL && u_index < 3)
    {
      pz_RinexDecoder->satInfo.d_antHeight = atof(pu_buff0);
      u_index++;
      pu_buff0 = strtok(NULL, " ");
    }
    return -1;
  }
  if (strstr(pu_buff, "SYS / # / OBS TYPES"))
  {
    u_index = 0;
    strncpy(pu_buff0, pu_buff, 1);
    u_sys = rinex_CharToSys(pu_buff0);
    if (u_sys == C_GNSS_NONE) {
      for (uint8_t u_j = C_GNSS_GPS; u_j < C_GNSS_MAX; u_j++)
      {
        if (pu_obsCodeNum[u_j] > 12)
        {
          u_sys = u_j;
          if (strlen(pu_obsCode[u_sys][13]) == 0)
          {
            u_index = 13;
            pu_buff0 = strtok(pu_buff, " ");
            while (pu_buff0 != NULL && !strstr(pu_buff0, "SYS"))
            {
              setstr(pu_obsCode[u_sys][u_index++], pu_buff0, 3);
              pu_buff0 = strtok(NULL, " ");
            }
          }
        }
      }
      return -1;
    }
    if (u_sys == C_GNSS_GLO) return -1;
    pu_buff0 = strtok(pu_buff, " ");
    while (pu_buff0 != NULL && !strstr(pu_buff0, "SYS"))
    {
      if (u_index == 1 && pu_buff0 != "")
      {
        pu_obsCodeNum[u_sys] = atoi(pu_buff0);
      }
      if (u_index >= 2)
      {
        setstr(pu_obsCode[u_sys][u_index - 2], pu_buff0, 3);
      }
      pu_buff0 = strtok(NULL, " ");
      u_index++;
    }
  }
  if (strstr(pu_buff, "END OF HEADER"))
  {
    return 0;
  }
  else
  {
    return -1;
  }
}

/**
 * @brief parse observation in RINEX format
 * @param[in]  pz_RinexDecoder represent the RINEX information
 * @param[in]  pu_buff represent the message of RINEX file
 * @param[in]  pu_obsCode represent the code information of observation
 * @param[in]  pu_obsCodeNum represent the number of observation type
 * @return     status -- TRUE: parse success, FALSE: parse fail
 */
int8_t rinex_dec_obsb(RinexDecoder_t* pz_RinexDecoder, char* pu_buff, const char pu_obsCode[][32][4], const uint8_t* pu_obsCodeNum)
{
  uint8_t u_index = 0;
  uint8_t u_prn = 0;
  uint8_t u_sys = C_GNSS_NONE;
  uint32_t q_obsNum = 0;
  uint32_t q_obsIndex = 0;
  uint32_t pq_epoch[6] = { 0 };
  char pu_id[5] = "";
  char pu_data[15] = { "" };
  char pu_typeNow[5] = "";
  char pu_typeNext[5] = "";
  char* pu_buff0 = (char*)malloc(sizeof(char));
  UtcTime_t z_utc = { 0 };
  GpsTime_t z_gpst = { 0 };
  RinexObs_t z_obs = { 0 };
  //updata the information of  ref station 
  if (strstr(pu_buff, "EVENT: NEW SITE OCCUPATION") || pu_buff[0] == '\n')
  {
    return 1;
  }
  if (strstr(pu_buff, "MARKER NAME"))
  {
    pz_RinexDecoder->satInfo.w_staid = atoi(pu_buff);
    return 0;
  }
  if (strstr(pu_buff, "APPROX POSITION XYZ"))
  {
    u_index = 0;
    pu_buff0 = strtok(pu_buff, " ");
    while (pu_buff0 != NULL && u_index < 3)
    {
      pz_RinexDecoder->satInfo.d_staPos[u_index++] = atof(pu_buff0);
      pu_buff0 = strtok(NULL, " ");
    }
    return 0;
  }
  if (strstr(pu_buff, "ANTENNA: DELTA H/E/N"))
  {
    u_index = 0;
    pu_buff0 = strtok(pu_buff, " ");
    while (pu_buff0 != NULL && u_index < 3)
    {
      pz_RinexDecoder->satInfo.d_antHeight = atof(pu_buff0);
      u_index++;
      pu_buff0 = strtok(NULL, " ");
    }
    return 0;
  }

  u_index = 0;
  if (strstr(pu_buff, ">"))
  {
    pu_buff0 = strtok(pu_buff + 1, " ");
    while (pu_buff0 != NULL && u_index < 6)
    {
      pq_epoch[u_index++] = atoi(pu_buff0);
      pu_buff0 = strtok(NULL, " ");
    }
    EpochTime_t z_Epoch = { pq_epoch[0],pq_epoch[1],pq_epoch[2],pq_epoch[3],pq_epoch[4],(float)pq_epoch[5] };
    tm_cvt_EpochToUtcTime(&z_Epoch, &z_utc);
    tm_cvt_UtcTimeToGpst(&z_utc, &z_gpst);
    pz_RinexDecoder->q_towMsec = z_gpst.q_towMsec - 19000;
    return 1;
  }
  if (pz_RinexDecoder->q_towMsec == 0)
  {
    return 0;
  }

  //read observation bady of RINEX
  sprintf(pu_id, "%.3s", pu_buff);
  u_sys = rinex_CharToSys(pu_id);
  u_prn = atoi(pu_id + 1);
  if (u_sys == C_GNSS_NONE || u_sys == C_GNSS_GLO || u_prn < 1)
  {
    return 0;
  }
  z_obs.tow_ms = pz_RinexDecoder->q_towMsec;
  for (u_index = 0, pu_buff0 = pu_buff + 3; u_index < pu_obsCodeNum[u_sys]; pu_buff0 += 16, u_index++)
  {
    strncpy(pu_data, pu_buff0, 14);
    sprintf(pu_id, "%.3s", pu_obsCode[u_sys][u_index]);
    sprintf(pu_typeNow, "%.2s", pu_obsCode[u_sys][u_index] + 1);
    sprintf(pu_typeNext, "%.2s", pu_obsCode[u_sys][u_index + 1] + 1);
    rinex_getObsInfor(u_sys, u_prn, pu_id, pu_data, &z_obs);
    if (strcmp(pu_typeNow, pu_typeNext))
    {
      if ((z_obs.pseudorange < 10 && z_obs.carrier_phase < 10) || z_obs.signal == C_GNSS_SIG_MAX)
      {
        continue;
      }
      q_obsNum++;
      q_obsIndex = obainRinexDataIndex(&z_obs, pz_RinexDecoder, &q_obsNum);
      pz_RinexDecoder->obs[q_obsIndex] = z_obs;
      pz_RinexDecoder->nobs++;
    }
  }
  return 0;
}

/**
 * @brief  Extract GnssMeasBlock from RINEX Decoder to GnssMeasBlock
 * @param[in]  pz_RinexDecoder represent the RINEX information
 * @param[in]  pz_MeasBlock represent the measurement block structure
 * @return none
 */
void rinex_ExtractMeasBlk(RinexDecoder_t* pz_RinexDecoder, GnssMeasBlock_t* pz_MeasBlock)
{
  uint8_t u_MaxMeasCount = sizeof(pz_MeasBlock->z_meas) / sizeof(pz_MeasBlock->z_meas[0]);

  if (any_Ptrs_Null(2, pz_RinexDecoder, pz_MeasBlock))
  {
    return;
  }

  pz_MeasBlock->u_version = VERSION_GNSS_MEAS_BLOCK;
  pz_MeasBlock->w_size = sizeof(GnssMeasBlock_t);
  pz_MeasBlock->z_Clock.z_gpsTime.w_week = pz_RinexDecoder->w_gpsWeek;
  pz_MeasBlock->z_Clock.z_gpsTime.q_towMsec = pz_RinexDecoder->q_towMsec;

  for (uint32_t i = 0; i < pz_RinexDecoder->nobs; i++)
  {
    if (i >= u_MaxMeasCount)
    {
      LOGI(TAG_SM, "Meas Index %d more than Max %d\n", i, u_MaxMeasCount);
      break;
    }

    RinexObs_t* pz_obs = &pz_RinexDecoder->obs[i];
    GnssMeas_t* pz_meas = &pz_MeasBlock->z_meas[pz_MeasBlock->w_measCount];
    pz_MeasBlock->w_measCount++;

    pz_meas->z_measStatusFlag.b_valid = TRUE;
    pz_meas->z_measStatusFlag.b_prValid = TRUE;
    pz_meas->z_measStatusFlag.b_drValid = TRUE;
    pz_meas->z_measStatusFlag.b_cpValid = TRUE;
    pz_meas->u_constellation = pz_obs->sys;
    pz_meas->u_svid = pz_obs->svid;
    pz_meas->u_signal = pz_obs->signal;
    pz_meas->u_LLI = pz_obs->lli;
    pz_meas->f_cn0 = pz_obs->snr;
    pz_meas->d_pseudoRange = pz_obs->pseudorange;
    pz_meas->d_doppler = pz_obs->doppler * wavelength(pz_obs->signal) * (-1.0);
    pz_meas->d_carrierPhase = pz_obs->carrier_phase;
  }

  return;
}

/**
 * @brief  Extract GnssCorrBlock from RINEX Decoder to GnssMeasBlock
 * @param[in]  pz_RinexDecoder represent the RINEX information
 * @param[in]  pz_CorrectBlock represent the correction block structure
 * @return none
 */
void rinex_ExtractCorrBlk(RinexDecoder_t* pz_RinexDecoder, GnssCorrBlock_t* pz_CorrectBlock)
{
  if (any_Ptrs_Null(2, pz_RinexDecoder, pz_CorrectBlock))
  {
    return;
  }

  pz_CorrectBlock->u_version = VERSION_GNSS_MEAS_BLOCK;
  pz_CorrectBlock->w_size = sizeof(GnssMeasBlock_t);
  pz_CorrectBlock->w_refStationId = pz_RinexDecoder->satInfo.w_staid;
  pz_CorrectBlock->d_refPosEcef[0] = pz_RinexDecoder->satInfo.d_staPos[0];
  pz_CorrectBlock->d_refPosEcef[1] = pz_RinexDecoder->satInfo.d_staPos[1];
  pz_CorrectBlock->d_refPosEcef[2] = pz_RinexDecoder->satInfo.d_staPos[2];
  pz_CorrectBlock->z_Clock.z_gpsTime.w_week = pz_RinexDecoder->w_gpsWeek;
  pz_CorrectBlock->z_Clock.z_gpsTime.q_towMsec = pz_RinexDecoder->q_towMsec;
  pz_CorrectBlock->w_measCount = pz_RinexDecoder->nobs;

  for (uint32_t i = 0, j = 0; i < pz_RinexDecoder->nobs; i++)
  {
    RinexObs_t* pz_obs = &pz_RinexDecoder->obs[i];
    GnssMeas_t* pz_meas = &pz_CorrectBlock->z_meas[j];

    pz_meas->z_measStatusFlag.b_valid = TRUE;
    pz_meas->z_measStatusFlag.b_prValid = TRUE;
    pz_meas->z_measStatusFlag.b_drValid = TRUE;
    pz_meas->z_measStatusFlag.b_cpValid = TRUE;
    pz_meas->u_constellation = pz_obs->sys;
    pz_meas->u_svid = pz_obs->svid;
    pz_meas->u_signal = pz_obs->signal;
    pz_meas->u_LLI = pz_obs->lli;
    pz_meas->f_cn0 = pz_obs->snr;
    pz_meas->d_pseudoRange = pz_obs->pseudorange;
    pz_meas->d_doppler = pz_obs->doppler * wavelength(pz_obs->signal) * (-1.0);
    pz_meas->d_carrierPhase = pz_obs->carrier_phase;
    j++;
  }

  return;
}