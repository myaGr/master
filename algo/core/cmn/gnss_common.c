/**@file        gnss_common.c
 * @brief       Common GNSS funtions
 * @details     1. Coordinate, Time, Frequency convert functions
 * @author      caizhijie
 * @date        2022/04/25
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/04/25  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "gnss_common.h"
#include <math.h>
#include "sd_api.h"
#include "cmn_utils.h"
#include "mw_alloc.h"
#include "mw_log.h"
#include "qxsi_ids_ssr2los_interface.h"
#include "loc_core_api.h"

/* constants -----------------------------------------------------------------*/
/* J2000, UTC */
static const EpochTime_t J2000 = { 2000,1,1,12,0,0.0f };

/* GPS Time Teference, UTC */
static const EpochTime_t gz_epoch_gpst0 = { 1980,  1,  6,  0,  0,  0.0 };

/* Leap Second List Table */
#define LEAP_SECOND_TABLE_SIZE (20)
static struct {
  uint16_t  year;
  uint8_t   month;
  uint8_t   day;
  uint8_t   hour;
  uint8_t   min;
  uint8_t   second;
  uint8_t   leapsecond;
  uint64_t  t_unixMsec;
} g_LeapSecond_Table[LEAP_SECOND_TABLE_SIZE] = {
  {2017, 1, 1, 0, 0, 0, 18, 1483228800000},
  {2015, 7, 1, 0, 0, 0, 17, 1435708800000},
  {2012, 7, 1, 0, 0, 0, 16, 1341100800000},
  {2009, 1, 1, 0, 0, 0, 15, 1230768000000},
  {2006, 1, 1, 0, 0, 0, 14, 1136073600000},
  {1999, 1, 1, 0, 0, 0, 13,  915148800000},
  {1997, 7, 1, 0, 0, 0, 12,  867715200000},
  {1996, 1, 1, 0, 0, 0, 11,  820454400000},
  {1994, 7, 1, 0, 0, 0, 10,  773020800000},
  {1993, 7, 1, 0, 0, 0,  9,  741484800000},
  {1992, 7, 1, 0, 0, 0,  8,  709948800000},
  {1991, 1, 1, 0, 0, 0,  7,  662688000000},
  {1990, 1, 1, 0, 0, 0,  6,  631152000000},
  {1988, 1, 1, 0, 0, 0,  5,  567993600000},
  {1985, 7, 1, 0, 0, 0,  4,  489024000000},
  {1983, 7, 1, 0, 0, 0,  3,  425865600000},
  {1982, 7, 1, 0, 0, 0,  2,  394329600000},
  {1981, 7, 1, 0, 0, 0,  1,  362793600000},
  {   0, 0, 1, 0, 0, 0,  0,             0},
  {   0, 0, 1, 0, 0, 0,  0,             0},
};

/* gnss_SignalTypeEnumVal */
static const uint8_t sys_signal[C_GNSS_MAX][MAX_GNSS_SIGNAL_FREQ] = {
        {0,0,0},
        {C_GNSS_SIG_GPS_L1C, C_GNSS_SIG_GPS_L2C, C_GNSS_SIG_GPS_L5Q},   /* GPS */
        {C_GNSS_SIG_GLO_G1,  C_GNSS_SIG_GLO_G2,  C_GNSS_SIG_MAX    },   /* GLO */
        {C_GNSS_SIG_BDS_B1I, C_GNSS_SIG_BDS_B3I, C_GNSS_SIG_BDS_B2I},   /* BDS2 */
        {C_GNSS_SIG_BDS_B1I, C_GNSS_SIG_BDS_B3I, C_GNSS_SIG_BDS_B2A},   /* BDS3 */
        {C_GNSS_SIG_GAL_E1,  C_GNSS_SIG_GAL_E5B, C_GNSS_SIG_GAL_E5A},   /* GAL */
        {C_GNSS_SIG_QZS_L1C, C_GNSS_SIG_QZS_L2C, C_GNSS_SIG_QZS_L5Q},   /* QZSS */
        {0,0,0}
};

/* gnss_SignalTypeEnumVal */
static const uint8_t signal_pos[C_GNSS_SIG_MAX] = {
  C_GNSS_FREQ_TYPE_L1,    //C_GNSS_SIG_GPS_L1C
  C_GNSS_FREQ_TYPE_L2,    //C_GNSS_SIG_GPS_L2C
  C_GNSS_FREQ_TYPE_L5,    //C_GNSS_SIG_GPS_L5Q
  C_GNSS_FREQ_TYPE_L1,    //C_GNSS_SIG_GLO_G1
  C_GNSS_FREQ_TYPE_L2,    //C_GNSS_SIG_GLO_G2
  C_GNSS_FREQ_TYPE_L1,    //C_GNSS_SIG_BDS_B1I
  C_GNSS_FREQ_TYPE_L5,    //C_GNSS_SIG_BDS_B2I
  C_GNSS_FREQ_TYPE_L2,    //C_GNSS_SIG_BDS_B3I
  C_GNSS_FREQ_TYPE_MAX,   //C_GNSS_SIG_BDS_B1C
  C_GNSS_FREQ_TYPE_L5,    //C_GNSS_SIG_BDS_B2A
  C_GNSS_FREQ_TYPE_MAX,   //C_GNSS_SIG_BDS_B2B
  C_GNSS_FREQ_TYPE_L1,    //C_GNSS_SIG_GAL_E1
  C_GNSS_FREQ_TYPE_L5,    //C_GNSS_SIG_GAL_E5A
  C_GNSS_FREQ_TYPE_L2,    //C_GNSS_SIG_GAL_E5B
  C_GNSS_FREQ_TYPE_L1,    //C_GNSS_SIG_QZS_L1C
  C_GNSS_FREQ_TYPE_L2,    //C_GNSS_SIG_QZS_L2C
  C_GNSS_FREQ_TYPE_L5,    //C_GNSS_SIG_QZS_L5Q
};

/* system char */
static const char system_char_s[C_GNSS_MAX + 1] = "NGRCCEJI";
const char system_char[C_GNSS_MAX][8] = {
  {"NONE"},
  {"GPS"},
  {"GLO"},
  {"BDS2"},
  {"BDS"},
  {"GAL"},
  {"QZS"},
  {"NavIC"},
};

/**
 * @brief Get the wavelength of a signal
 * @param[in] signal type
 * @return Signal's wavelength
 */
double wavelength(gnss_SignalType signal)
{
  double wl = 1.0;
  switch (signal)
  {
  case C_GNSS_SIG_GPS_L1C:
    wl = GPS_L1C_WAVE;
    break;
  case C_GNSS_SIG_GPS_L2C:
    wl = GPS_L2C_WAVE;
    break;
  case C_GNSS_SIG_GPS_L5Q:
    wl = GPS_L5Q_WAVE;
    break;
  case C_GNSS_SIG_GLO_G1:
    break;
  case C_GNSS_SIG_GLO_G2:
    break;
  case C_GNSS_SIG_BDS_B1I:
    wl = BDS_B1I_WAVE;
    break;
  case C_GNSS_SIG_BDS_B2I:
    wl = BDS_B2I_WAVE;
    break;
  case C_GNSS_SIG_BDS_B3I:
      wl = BDS_B3I_WAVE;
      break;
  case C_GNSS_SIG_BDS_B1C:
    wl = BDS_B1C_WAVE;
    break;
  case C_GNSS_SIG_BDS_B2A:
    wl = BDS_B2A_WAVE;
    break;
  case C_GNSS_SIG_BDS_B2B:
    wl = BDS_B2B_WAVE;
    break;
  case C_GNSS_SIG_GAL_E1:
    wl = GAL_E1_WAVE;
    break;
  case C_GNSS_SIG_GAL_E5A:
    wl = GAL_E5A_WAVE;
    break;
  case C_GNSS_SIG_GAL_E5B:
    wl = GAL_E5B_WAVE;
    break;
  case C_GNSS_SIG_QZS_L1C:
    wl = QZS_L1C_WAVE;
    break;
  case C_GNSS_SIG_QZS_L2C:
    wl = QZS_L2C_WAVE;
    break;
  case C_GNSS_SIG_QZS_L5Q:
    wl = QZS_L5Q_WAVE;
    break;
  default:
    break;
  }
  return wl;
}

/**
 * @brief Convert a signal type to frequency type
 * @param[in] signal type
 * @return frequency type
 */
gnss_FreqType gnss_cvt_Sig2FreqType(gnss_SignalType signal)
{
  gnss_FreqType f = C_GNSS_FREQ_TYPE_MAX;
  switch (signal)
  {
  case C_GNSS_SIG_GPS_L1C:
    f = C_GNSS_FREQ_TYPE_L1;
    break;
  case C_GNSS_SIG_GPS_L2C:
    f = C_GNSS_FREQ_TYPE_L2;
    break;
  case C_GNSS_SIG_GPS_L5Q:
    f = C_GNSS_FREQ_TYPE_L5;
    break;
  case C_GNSS_SIG_GLO_G1:
    f = C_GNSS_FREQ_TYPE_L1;
    break;
  case C_GNSS_SIG_GLO_G2:
    f = C_GNSS_FREQ_TYPE_L2;
    break;
  case C_GNSS_SIG_BDS_B1I:
    f = C_GNSS_FREQ_TYPE_L1;
    break;
  case C_GNSS_SIG_BDS_B2I:
    f = C_GNSS_FREQ_TYPE_L5;
    break;
  case C_GNSS_SIG_BDS_B3I:
    f = C_GNSS_FREQ_TYPE_L2;
    break;
  case C_GNSS_SIG_BDS_B1C:
    f = C_GNSS_FREQ_TYPE_MAX;
    break;
  case C_GNSS_SIG_BDS_B2A:
    f = C_GNSS_FREQ_TYPE_L5;
    break;
  case C_GNSS_SIG_BDS_B2B:
    f = C_GNSS_FREQ_TYPE_MAX;
    break;
  case C_GNSS_SIG_GAL_E1:
    f = C_GNSS_FREQ_TYPE_L1;
    break;
  case C_GNSS_SIG_GAL_E5A:
    f = C_GNSS_FREQ_TYPE_L5;
    break;
  case C_GNSS_SIG_GAL_E5B:
    f = C_GNSS_FREQ_TYPE_L2;
    break;
  case C_GNSS_SIG_QZS_L1C:
    f = C_GNSS_FREQ_TYPE_L1;
    break;
  case C_GNSS_SIG_QZS_L2C:
    f = C_GNSS_FREQ_TYPE_L2;
    break;
  case C_GNSS_SIG_QZS_L5Q:
    f = C_GNSS_FREQ_TYPE_L5;
    break;
  default:
    f = C_GNSS_FREQ_TYPE_MAX;
    break;
  }
  return f;
}

/**
 * @brief Convert frequency type to amb type
 * @param[in] frequency type
 * @return gnss_fixedAmbType
 */
gnss_fixedAmbType gnss_cvt_FreqType2AmbType(gnss_FreqType freq)
{
  gnss_fixedAmbType z_type = GNSS_NONE_AMB_FIXED;
  switch (freq)
  {
  case C_GNSS_FREQ_TYPE_L1:
    z_type = GNSS_L1_AMB_FIXED;
    break;
  case C_GNSS_FREQ_TYPE_L2:
    z_type = GNSS_L2_AMB_FIXED;
    break;
  case C_GNSS_FREQ_TYPE_L5:
    z_type = GNSS_L3_AMB_FIXED;
    break;
  default:
    z_type = GNSS_NONE_AMB_FIXED;
    break;
  }
  return z_type;
}

/**
* @brief Convert internal constellation type to Loc engine constellation type
* @param[in]   u_ConstellationType - Input constellation type
* @return      u_type - Output loc constellation type
*/
loc_api_gnssConstellationType gnss_cvt_ConstellationTypeToLoc(gnss_ConstellationType u_ConstellationType)
{
  loc_api_gnssConstellationType u_type = LOC_API_GNSS_MAX;

  switch (u_ConstellationType)
  {
  case C_GNSS_GPS:
    u_type = LOC_API_GNSS_GPS;
    break;
  case C_GNSS_GLO:
    u_type = LOC_API_GNSS_GLO;
    break;
  case C_GNSS_BDS2:
    u_type = LOC_API_GNSS_BDS;
    break;
  case C_GNSS_BDS3:
    u_type = LOC_API_GNSS_BDS;
    break;
  case C_GNSS_GAL:
    u_type = LOC_API_GNSS_GAL;
    break;
  case C_GNSS_QZS:
    u_type = LOC_API_GNSS_QZS;
    break;
  default:
    u_type = LOC_API_GNSS_MAX;
    break;
  }
  return  u_type;
}

/**
 * @brief Convert a signal type to constellation type
 * @param[in] signal type
 * @return Constellation type
 */
gnss_ConstellationType gnss_cvt_Signal2Constellation(gnss_SignalType signal)
{
  gnss_ConstellationType u_Constellation = C_GNSS_NONE;
  switch (signal)
  {
  case C_GNSS_SIG_GPS_L1C:
  case C_GNSS_SIG_GPS_L2C:
  case C_GNSS_SIG_GPS_L5Q:
    u_Constellation = C_GNSS_GPS;
    break;
  case C_GNSS_SIG_GLO_G1:
  case C_GNSS_SIG_GLO_G2:
    u_Constellation = C_GNSS_GLO;
    break;
  case C_GNSS_SIG_BDS_B1I:
  case C_GNSS_SIG_BDS_B2I:
  case C_GNSS_SIG_BDS_B1C:
  case C_GNSS_SIG_BDS_B2A:
  case C_GNSS_SIG_BDS_B2B:
  case C_GNSS_SIG_BDS_B3I:
    u_Constellation = C_GNSS_BDS3;
    break;
  case C_GNSS_SIG_GAL_E1:
  case C_GNSS_SIG_GAL_E5A:
  case C_GNSS_SIG_GAL_E5B:
    u_Constellation = C_GNSS_GAL;
    break;
  case C_GNSS_SIG_QZS_L1C:
  case C_GNSS_SIG_QZS_L2C:
  case C_GNSS_SIG_QZS_L5Q:
    u_Constellation = C_GNSS_QZS;
    break;
  default:
    break;
  }
  return u_Constellation;
}

/**
 * @brief Convert a signal type to code string
 * @param[in] signal type
 * @param[out] signal str
 * @return none
 */
void gnss_cvt_Sig2CodeStr(gnss_SignalType signal,char* str)
{
  if (NULL == str) return;
  switch (signal)
  {
  case C_GNSS_SIG_GPS_L1C:
    strcpy(str,"1C");
    break;
  case C_GNSS_SIG_GPS_L2C:
    strcpy(str, "2C");
    break;
  case C_GNSS_SIG_GPS_L5Q:
    strcpy(str, "5Q");
    break;
  case C_GNSS_SIG_GLO_G1:
    strcpy(str, "1P");
    break;
  case C_GNSS_SIG_GLO_G2:
    strcpy(str, "2P");
    break;
  case C_GNSS_SIG_BDS_B1I:
    strcpy(str, "2I");
    break;
  case C_GNSS_SIG_BDS_B2I:
    strcpy(str, "7I");
    break;
  case C_GNSS_SIG_BDS_B3I:
    strcpy(str, "6I");
    break;
  case C_GNSS_SIG_BDS_B1C:
    strcpy(str, "1C");
    break;
  case C_GNSS_SIG_BDS_B2A:
    strcpy(str, "5P");
    break;
  case C_GNSS_SIG_BDS_B2B:
    strcpy(str, "7Q");
    break;
  case C_GNSS_SIG_GAL_E1:
    strcpy(str, "1C");
    break;
  case C_GNSS_SIG_GAL_E5A:
    strcpy(str, "5Q");
    break;
  case C_GNSS_SIG_GAL_E5B:
    strcpy(str, "7I");
    break;
  case C_GNSS_SIG_QZS_L1C:
    strcpy(str, "1C");
    break;
  case C_GNSS_SIG_QZS_L2C:
    strcpy(str, "2C");
    break;
  case C_GNSS_SIG_QZS_L5Q:
    strcpy(str, "5Q");
    break;
  default:
    strcpy(str, "");
    break;
  }
}

/**
 * @brief Get the ionospheric coefficient base the L1 frequency
 * @param[in] constellation
 * @return the frequency of  base L1
 */
double gnss_BaseL1Freq(uint8_t constellation)
{
  if (C_GNSS_GPS == constellation)
  {
    return GPS_L1C_FREQ;
  }
  else if (C_GNSS_GLO == constellation)
  {
    return 1.0;
  }
  else if (C_GNSS_BDS3 == constellation || C_GNSS_BDS2 == constellation)
  {
    return BDS_B1I_FREQ;
  }
  else if (C_GNSS_GAL == constellation)
  {
    return GAL_E1_FREQ;
  }
  else if (C_GNSS_QZS == constellation)
  {
    return QZS_L1C_FREQ;
  }
  return 1.0;
}

/**
 * @brief Get the ionospheric coefficient base the L1 frequency
 * @param[in] constellation
 * @param[in] signal
 * @return the ionospheric coefficient base the L1 frequency
 */
double gnss_ionoCoefBaseL1Freq(uint8_t constellation, gnss_SignalType signal)
{
    double d_ionoCoef = 1.0;
    double d_L1_wave = 0.0;
    double d_wave = 0.0;
    gnss_FreqType z_f = gnss_cvt_Sig2FreqType(signal);
    if (C_GNSS_FREQ_TYPE_L2 == z_f || C_GNSS_FREQ_TYPE_L5 == z_f)
    {
        if (C_GNSS_GPS == constellation)
        {
            d_L1_wave = wavelength(C_GNSS_SIG_GPS_L1C);
        }
        else if (C_GNSS_GLO == constellation)
        {
            d_L1_wave = wavelength(C_GNSS_SIG_GLO_G1);
        }
        else if (C_GNSS_BDS3 == constellation || C_GNSS_BDS2 == constellation)
        {
            d_L1_wave = wavelength(C_GNSS_SIG_BDS_B1I);
        }
        else if (C_GNSS_GAL == constellation)
        {
            d_L1_wave = wavelength(C_GNSS_SIG_GAL_E1);
        }
        else if (C_GNSS_QZS == constellation)
        {
            d_L1_wave = wavelength(C_GNSS_SIG_QZS_L1C);
        }
        d_wave= wavelength(signal);
        if (d_L1_wave > 1e-3)
        {
          d_ionoCoef = SQR(d_wave / d_L1_wave);
        }
    }

    return d_ionoCoef;
}

/**
 * @brief Convert a frequency type to frequency used in the algorithm
 * @param[in] z_f frequency type
 * @return frequency used in the algorithm
 */
algo_useFreq gnss_freqType2Algo(gnss_FreqType z_f)
{
    algo_useFreq z_algoFreq = ALGO_NON_FREQ;
    if (C_GNSS_FREQ_TYPE_L1 == z_f)
    {
        z_algoFreq = ALGO_L1_FREQ;
    }
    else if (C_GNSS_FREQ_TYPE_L2 == z_f)
    {
        z_algoFreq = ALGO_L2_FREQ;
    }
    else if (C_GNSS_FREQ_TYPE_L5 == z_f)
    {
        z_algoFreq = ALGO_L5_FREQ;
    }
    return z_algoFreq;
}
/**
 * @brief Get satellite type from svid
 * @param[in] constellation
 * @param[in] svid
 * @return satellite type
 */
gnss_SatelliteType svid2SatType(uint8_t constellation, uint8_t svid)
{
  switch (constellation)
  {
  case C_GNSS_BDS2:
  case C_GNSS_BDS3:
  {
    if (svid <= 5 || 18 == svid || (svid >= 59 && svid <= 61))
    {
      return C_SAT_TYPE_GEO;
    }
    else if (svid <= 10 || 13 == svid || 16 == svid ||
      (svid >= 38 && svid <= 40) || 31 == svid || 56 == svid)
    {
      return C_SAT_TYPE_IGSO;
    }
    else
    {
      return C_SAT_TYPE_MEO;
    }
    
  }
  default:
    return C_SAT_TYPE_MEO;
  }
}
/**
 * @brief Convert a constellation type to constellation used in the algorithm
 * @param[in] z_constellation constellation type
 * @return constellation used in the algorithm
 */
algo_useConstellation gnss_satType2Algo(gnss_ConstellationType z_c)
{
    algo_useConstellation z_algoConstellation = ALGO_NON_SYS;
    if (C_GNSS_GPS == z_c)
    {
        z_algoConstellation = ALGO_GPS_SYS;
    }
    else if (C_GNSS_GLO == z_c)
    {
        z_algoConstellation = ALGO_GLO_SYS;
    }
    else if (C_GNSS_BDS3 == z_c || C_GNSS_BDS2 == z_c)
    {
        z_algoConstellation = ALGO_BDS_SYS;
    }
    else if (C_GNSS_GAL == z_c)
    {
        z_algoConstellation = ALGO_GAL_SYS;
    }
    else if (C_GNSS_QZS == z_c)
    {
        z_algoConstellation = ALGO_QZS_SYS;
    }
    return z_algoConstellation;
}

/**
 * @brief         get more than one min member's index in arr[]
 * @param[in]     pd_arr[] the double array !!WARN!! 1e9 MAX
 * @param[in]     u_size size of arr[]
 * @param[out]    pu_min_indices[] num_min min index in arr[]
 * @param[in]     u_num_min number of min
 * @return 0: success, other: fail
 */
uint8_t cmn_find_min_indices(double pd_arr[], uint8_t u_size, int16_t pu_min_indices[], uint8_t u_num_min)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_k = 0;
  uint8_t u_status = 0;
  if (u_size < u_num_min)
  {
    return 1;
  }
  for (u_i = 0; u_i < u_num_min; u_i++)
  {
    pu_min_indices[u_i] = -1;
  }
  for (u_i = 0; u_i < u_num_min; u_i++)
  {
    double min_value = 1e9;
    int min_index = -1;

    for (u_j = 0; u_j < u_size; u_j++)
    {
      if (pd_arr[u_j] < min_value)
      {
        int already_used = 0;
        for (u_k = 0; u_k < u_i; u_k++)
        {
          if (pu_min_indices[u_k] == u_j) {
            already_used = 1;
            break;
          }
        }
        if (!already_used) {
          min_value = pd_arr[u_j];
          min_index = u_j;
        }
      }
    }

    if (min_index != -1) {
      pu_min_indices[u_i] = min_index;
    }
    else {
      // Handle the case where there are less than num_min distinct values in the array
      // You might want to handle this differently based on your requirements
      pu_min_indices[u_i] = -1;
      u_status = 1;
    }
  }
  return u_status;
}

/**
 * @brief descending sort the float array
 * @param[in] pf_data,float array,it will be sort by ascending
 * @param[in] q_num,the number of float array
 * @return void
 */
int gnss_descSortFloat(float* pf_data, uint32_t q_num)
{
  uint32_t q_i = 0;
  uint32_t q_j = 0;
  float f_temp = 0.0;

  if (pf_data == NULL) return 0;

  for (q_i = q_num - 1; q_i > 0; q_i--)
  {
    for (q_j = 0; q_j < q_i; q_j++)
    {
      if (pf_data[q_j] < pf_data[q_j + 1])
      {
        f_temp = pf_data[q_j];
        pf_data[q_j] = pf_data[q_j + 1];
        pf_data[q_j + 1] = f_temp;
      }
    }
  }
  return 1;
}

/**
 * @brief ascending sort the float array
 * @param[in] pf_data,float array,it will be sort by ascending
 * @param[in] q_num,the number of float array
 * @return void
 */
void gnss_ascSortFloat(float* pf_data, uint32_t q_num)
{
    uint32_t q_i = 0;
    uint32_t q_j = 0;
    float f_temp = 0.0;
    for (q_i = 0; q_i < q_num - 1; ++q_i)
    {
        for (q_j = q_i + 1; q_j < q_num; ++q_j)
        {
            if (pf_data[q_j] < pf_data[q_i])
            {
                f_temp = pf_data[q_i];
                pf_data[q_i] = pf_data[q_j];
                pf_data[q_j] = f_temp;
            }
        }
    }
    return;
}
/**
 * @brief ascending sort the double array
 * @param[in] pd_data,double array,it will be sorted by ascending
 * @param[in] q_num,the number of double array
 * @return void
 */
void gnss_ascSortDouble(double* pd_data, uint32_t q_num)
{
    uint32_t q_i = 0;
    uint32_t q_j = 0;
    double d_temp = 0.0;
    for (q_i = 0; q_i < q_num - 1; ++q_i)
    {
        for (q_j = q_i + 1; q_j < q_num; ++q_j)
        {
            if (pd_data[q_j] < pd_data[q_i])
            {
                d_temp = pd_data[q_i];
                pd_data[q_i] = pd_data[q_j];
                pd_data[q_j] = d_temp;
            }
        }
    }
    return;
}
/**
 * @brief get the median value from the float array
 * @param[in] pf_data,float array,it will be sort by ascending
 * @param[in] q_num,the number of float array
 * @param[out] pf_median,the median value of float array
 * @return uint8_t,0 represent failure and 1 represent success
 */
uint8_t gnss_ascSortMedianFloat(const float* pf_data, uint32_t q_num, float* pf_median)
{
    uint8_t u_status = 0;
    uint32_t q_loc = q_num / 2;
    uint32_t q_i = 0;
    float* pf_dataTemp = NULL;
    *pf_median = 0.0;
    if (q_num >= 2)
    {
        pf_dataTemp = (float*)OS_MALLOC_FAST(q_num * sizeof(float));
        for (q_i = 0; q_i < q_num; ++q_i)
        {
            pf_dataTemp[q_i] = pf_data[q_i];
        }
        u_status = 1;
        gnss_ascSortFloat(pf_dataTemp, q_num);
        if (0 == q_num % 2)
        {
            *pf_median = (pf_dataTemp[q_loc - 1] + pf_dataTemp[q_loc]) * 0.5f;
        }
        else
        {
            *pf_median = pf_dataTemp[q_loc];
        }
        //if (NULL != pf_dataTemp)
        {
            OS_FREE(pf_dataTemp);
        }
    }
    return u_status;
}
/**
 * @brief get the median value from the double array
 * @param[in] pd_data,double array,it will be sorted by ascending
 * @param[in] q_num,the number of double array
 * @param[out] pd_median,the median value of double array
 * @return uint8_t,0 represent failure and 1 represent success
 */
uint8_t gnss_ascSortMedianDouble(const double* pd_data, uint32_t q_num, double* pd_median)
{
    uint8_t u_status = 0;
    uint32_t q_loc = q_num / 2;
    uint32_t q_i = 0;
    double* pd_dataTemp = NULL;
    *pd_median = 0.0;
    if (q_num >= 2)
    {
        pd_dataTemp = (double*)OS_MALLOC_FAST(q_num * sizeof(double));
        for (q_i = 0; q_i < q_num; ++q_i)
        {
            pd_dataTemp[q_i] = pd_data[q_i];
        }
        u_status = 1;
        gnss_ascSortDouble(pd_dataTemp, q_num);
        if (0 == q_num % 2)
        {
            *pd_median = (pd_dataTemp[q_loc - 1] + pd_dataTemp[q_loc]) * 0.5;
        }
        else
        {
            *pd_median = pd_dataTemp[q_loc];
        }
        OS_FREE(pd_dataTemp);
    }
    return u_status;
}
/**
 * @brief get the median and std value from the double array
 * @param[in] pd_data,double array,it will be sorted by ascending
 * @param[in] q_num,the number of double array
 * @param[out] pd_median,the median value of double array
 * @param[out] pd_std,the std value of double array
 * @return uint8_t,0 represent failure and 1 represent success
 */
uint8_t gnss_ascSortMedianStdDouble(const double* pd_data, uint32_t q_num, double* pd_median, double* pd_std)
{
  uint8_t u_status = 0;
  uint32_t q_loc = q_num / 2;
  uint32_t q_i = 0;
  double d_median = 0.0;
  double d_std = 0.0;
  double* pd_dataTemp = NULL;

  if (q_num >= 2)
  {
    pd_dataTemp = (double*)OS_MALLOC_FAST(q_num * sizeof(double));
    for (q_i = 0; q_i < q_num; ++q_i)
    {
      pd_dataTemp[q_i] = pd_data[q_i];
    }
    u_status = 1;
    gnss_ascSortDouble(pd_dataTemp, q_num);
    if (0 == q_num % 2)
    {
      d_median = (pd_dataTemp[q_loc - 1] + pd_dataTemp[q_loc]) * 0.5;
    }
    else
    {
      d_median = pd_dataTemp[q_loc];
    }

    for (q_i = 0; q_i < q_num; ++q_i)
    {
      d_std += (pd_data[q_i] - d_median) * (pd_data[q_i] - d_median);
    }
    d_std = sqrt(d_std / (q_num - 1));
    OS_FREE(pd_dataTemp);
  }
  if (NULL != pd_median)
  {
    *pd_median = d_median;
  }
  if (NULL != pd_std)
  {
    *pd_std = d_std;
  }
  return u_status;
}
/**
 * @brief MAD method for the double array
 * @param[in] pd_data,double array,it will be sorted by ascending
 * @param[in] q_num,the number of double array
 * @param[out] pd_dataAbs,the abs of minus median value of double array
 * @param[out] pd_median,the median value of pd_data array
 * @param[out] pd_medianAbs,the median value of pd_dataAbs array
 * @return uint8_t,0 represent failure and 1 represent success
 */
uint8_t gnss_MadDouble(const double* pd_data, uint32_t q_num, double* pd_dataAbs, double* pd_median, double* pd_medianAbs)
{
    uint8_t u_status = 0;
    uint32_t q_i = 0;
    double d_median = 0.0;
    *pd_median = 0.0;
    if (q_num < 1)
    {
        return 0;
    }
    u_status = gnss_ascSortMedianDouble(pd_data, q_num, &d_median);
    if (0 == u_status)
    {
        return u_status;
    }
    else if (1 == u_status)
    {
      *pd_median = d_median;
    }
    for (q_i = 0; q_i < q_num; ++q_i)
    {
        pd_dataAbs[q_i] = fabs(pd_data[q_i] - d_median);
    }
    u_status = gnss_ascSortMedianDouble(pd_dataAbs, q_num, &d_median);
    if (1 == u_status)
    {
        *pd_medianAbs = d_median;
    }
    return u_status;
}

/**
 * @brief MAD method for the float array
 * @param[in] pf_data,float array
 * @param[in] q_num,the number of float array
 * @param[out] pf_dataAbs,the abs of minus median value of float array
 * @param[out] pf_median,the median value of pf_dataAbs array
 * @return uint8_t,0 represent failure and 1 represent success
 */
uint8_t gnss_MadFloat(const float* pf_data, uint32_t q_num, float* pf_dataAbs, float* pf_median)
{
    uint8_t u_status = 0;
    uint32_t q_i = 0;
    float f_median = 0.0;
    *pf_median = 0.0;
    if (q_num < 1)
    {
        return 0;
    }
    u_status = gnss_ascSortMedianFloat(pf_data, q_num, &f_median);
    if (0 == u_status)
    {
        return u_status;
    }
    for (q_i = 0; q_i < q_num; ++q_i)
    {
        pf_dataAbs[q_i] = fabsf(pf_data[q_i] - f_median);
    }
    u_status = gnss_ascSortMedianFloat(pf_dataAbs, q_num, &f_median);
    if (1 == u_status)
    {
        *pf_median = f_median;
    }
    return u_status;
}

/*----------------------------------------------------------------------------*/
/* coordinates transformation ------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/**
 * @brief transform ecef position to geodetic position
 * @param[in] r ecef position {x,y,z} (m)
 * @param[out] pos geodetic position {lat,lon,h} (rad,m)
 */
extern void gnss_Ecef2Lla(const double* ecef, double* lla)
{
  double e2 = FE_WGS84 * (2.0 - FE_WGS84);
  double r2 = gnss_Dot(ecef, ecef, 2);
  double z = 0.0;
  double zk = 0.0;
  double v = RE_WGS84;
  double sinp = 0.0;

  for (z = ecef[2], zk = 0.0; fabs(z - zk) >= 1E-4;) 
  {
    zk = z;
    sinp = z / sqrt(r2 + z * z);
    v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);
    z = ecef[2] + v * e2 * sinp;
  }
  lla[0] = r2 > 1E-12 ? atan(z / sqrt(r2)) : (ecef[2] > 0.0 ? PI / 2.0 : -PI / 2.0);
  lla[1] = r2 > 1E-12 ? atan2(ecef[1], ecef[0]) : 0.0;
  lla[2] = sqrt(r2 + z * z) - v;
}

/**
 * @brief transform geodetic position to ecef position
 * @param[in] pos geodetic position {lat,lon,h} (rad,m)
 * @param[out] r ecef position {x,y,z} (m)
 */
extern void gnss_Lla2Ecef(const double* lla, double* ecef)
{
  double sinp = sin(lla[0]);
  double cosp = cos(lla[0]);
  double sinl = sin(lla[1]);
  double cosl = cos(lla[1]);
  double e2 = FE_WGS84 * (2.0 - FE_WGS84);
  double v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);

  ecef[0] = (v + lla[2]) * cosp * cosl;
  ecef[1] = (v + lla[2]) * cosp * sinl;
  ecef[2] = (v * (1.0 - e2) + lla[2]) * sinp;
}


/**
 * @brief compute ecef to local coordinate transfromation matrix
 * @param[in] pos geodetic position {lat,lon} (rad)
 * @param[out] E ecef to local coord transformation matrix (3x3)
 * notes: matirix stored by column-major order (fortran convention)
 */
extern void gnss_Pos2EnuMat(const double* pos, double* E)
{
  double sinp = sin(pos[0]), cosp = cos(pos[0]), sinl = sin(pos[1]), cosl = cos(pos[1]);

  E[0] = -sinl;      E[3] = cosl;       E[6] = 0.0;
  E[1] = -sinp * cosl; E[4] = -sinp * sinl; E[7] = cosp;
  E[2] = cosp * cosl;  E[5] = cosp * sinl;  E[8] = sinp;
}

/**
 * @brief transform ecef vector to local tangental coordinate
 * @param[in] pos geodetic position {lat,lon} (rad)
 * @param[in] r vector in ecef coordinate {x,y,z}
 * @param[out] e vector in local tangental coordinate {e,n,u}
 */
extern void gnss_Ecef2Enu(const double* pos, const double* r, double* e)
{
  double E[9] = { .0 };

  gnss_Pos2EnuMat(pos, E);
  gnss_MatrixMultiply("NN", 3, 1, 3, 1.0, E, r, 0.0, e);
}

/**
 * @brief compute ecef to local coordinate transfromation matrix
 * @param[in] pos geodetic position {lat,lon} (rad)
 * @param[out] E ecef to local coord transformation matrix (3x3)
 * notes: matirix stored by column-major order (fortran convention)
 */
extern void gnss_Pos2EnuMat_row(const double* pos, double* E)
{
  double sinp = sin(pos[0]);
  double cosp = cos(pos[0]);
  double sinl = sin(pos[1]);
  double cosl = cos(pos[1]);

  E[0] = -sinl;      E[1] = cosl;       E[2] = 0.0;
  E[3] = -sinp * cosl; E[4] = -sinp * sinl; E[5] = cosp;
  E[6] = cosp * cosl;  E[7] = cosp * sinl;  E[8] = sinp;
}

/**
 * @brief transform ecef vector to local tangental coordinate
 * @param[in] pos geodetic position {lat,lon} (rad)
 * @param[in] r vector in ecef coordinate {x,y,z}
 * @param[out] e vector in local tangental coordinate {e,n,u}
 */
extern void gnss_Ecef2Enu_edit(const double* pos, double* r, double* e)
{
  double E[9] = {.0};
  gnss_Pos2EnuMat_row(pos, E);

  const matrix_t* m_E = matrix_new_from_buffer_const(3, 3, E);
  const matrix_t* m_r = matrix_new_from_buffer_const(3, 1, r);
  matrix_t* m_e = matrix_new_from_buffer(3, 1, e);

  matrix_mul(m_E, m_r, m_e);
  matrix_free_batch(2, &m_E, &m_r);
}


/**
 * @brief transform local tangental coordinate vector to ecef
 * @param[in] pos geodetic position {lat,lon} (rad)
 * @param[in] e vector in local tangental coordinate {e,n,u}
 * @param[out] r vector in ecef coordinate {x,y,z}
 */
extern void gnss_Enu2Ecef(const double* pos, const double* e, double* r)
{
  double E[9] = { .0 };

  gnss_Pos2EnuMat(pos, E);
  gnss_MatrixMultiply("TN", 3, 1, 3, 1.0, E, e, 0.0, r);
}


/**
 * @brief transform ecef covariance to local tangental coordinate
 * @param[in] pos geodetic position {lat,lon} (rad)
 * @param[in] P covariance in ecef coordinate
 * @param[out] Q covariance in local tangental coordinate
 */
extern void gnss_CovEcef2Enu(const double* pos, const double* P, double* Q)
{
  double E[9] = { .0 }, EP[9] = { .0 };

  gnss_Pos2EnuMat(pos, E);
  gnss_MatrixMultiply("NN", 3, 3, 3, 1.0, E, P, 0.0, EP);
  gnss_MatrixMultiply("NT", 3, 3, 3, 1.0, EP, E, 0.0, Q);
}

/**
 * @brief transform local enu covariance to xyz-ecef coordinate
 * @param[in] pos geodetic position {lat,lon} (rad)
 * @param[out] Q covariance in local enu coordinate
 * @param[out ] P covariance in xyz-ecef coordinate
 */
extern void gnss_CovEnu2Ecef(const double* pos, const double* Q, double* P)
{
  double E[9], EQ[9];

  gnss_Pos2EnuMat(pos, E);
  gnss_MatrixMultiply("TN", 3, 3, 3, 1.0, E, Q, 0.0, EQ);
  gnss_MatrixMultiply("NN", 3, 3, 3, 1.0, EQ, E, 0.0, P);
}

/**
 * @brief Compute horizontal speed(m/s) from input east and north velocity
 * @param[in] f_VelE - east velocity(m/s)
 * @param[in] f_VelN - north velocity(m/s)
 * @return   computed horizontal speed
 */
float gnss_CovEnVel2Speed(float f_VelE, float f_VelN)
{
  return sqrtf(f_VelE * f_VelE + f_VelN * f_VelN);
}

/**
 * @brief astronomical arguments: f={l,l',F,D,OMG} (rad)
 * @param[in] t
 * @param[out] f
 */
static void ast_args(double t, double* f)
{
  static const double fc[][5] = { /* coefficients for iau 1980 nutation */
      { 134.96340251, 1717915923.2178,  31.8792,  0.051635, -0.00024470},
      { 357.52910918,  129596581.0481,  -0.5532,  0.000136, -0.00001149},
      {  93.27209062, 1739527262.8478, -12.7512, -0.001037,  0.00000417},
      { 297.85019547, 1602961601.2090,  -6.3706,  0.006593, -0.00003169},
      { 125.04455501,   -6962890.2665,   7.4722,  0.007702, -0.00005939}
  };
  double tt[4];
  int i, j;

  for (tt[0] = t, i = 1; i < 4; i++) tt[i] = tt[i - 1] * t;
  for (i = 0; i < 5; i++) {
    f[i] = fc[i][0] * 3600.0;
    for (j = 0; j < 4; j++) f[i] += fc[i][j + 1] * tt[j];
    f[i] = fmod(f[i] * ARCSEC2RAD, 2.0 * PI);
  }
}


/**
 * @brief iau 1980 nutation
 * @param t
 * @param f
 * @param dpsi
 * @param deps
 */
static void nut_iau1980(double t, const double* f, double* dpsi, double* deps)
{
  static const double nut[106][10] = {
      {   0,   0,   0,   0,   1, -6798.4, -171996, -174.2, 92025,   8.9},
      {   0,   0,   2,  -2,   2,   182.6,  -13187,   -1.6,  5736,  -3.1},
      {   0,   0,   2,   0,   2,    13.7,   -2274,   -0.2,   977,  -0.5},
      {   0,   0,   0,   0,   2, -3399.2,    2062,    0.2,  -895,   0.5},
      {   0,  -1,   0,   0,   0,  -365.3,   -1426,    3.4,    54,  -0.1},
      {   1,   0,   0,   0,   0,    27.6,     712,    0.1,    -7,   0.0},
      {   0,   1,   2,  -2,   2,   121.7,    -517,    1.2,   224,  -0.6},
      {   0,   0,   2,   0,   1,    13.6,    -386,   -0.4,   200,   0.0},
      {   1,   0,   2,   0,   2,     9.1,    -301,    0.0,   129,  -0.1},
      {   0,  -1,   2,  -2,   2,   365.2,     217,   -0.5,   -95,   0.3},
      {  -1,   0,   0,   2,   0,    31.8,     158,    0.0,    -1,   0.0},
      {   0,   0,   2,  -2,   1,   177.8,     129,    0.1,   -70,   0.0},
      {  -1,   0,   2,   0,   2,    27.1,     123,    0.0,   -53,   0.0},
      {   1,   0,   0,   0,   1,    27.7,      63,    0.1,   -33,   0.0},
      {   0,   0,   0,   2,   0,    14.8,      63,    0.0,    -2,   0.0},
      {  -1,   0,   2,   2,   2,     9.6,     -59,    0.0,    26,   0.0},
      {  -1,   0,   0,   0,   1,   -27.4,     -58,   -0.1,    32,   0.0},
      {   1,   0,   2,   0,   1,     9.1,     -51,    0.0,    27,   0.0},
      {  -2,   0,   0,   2,   0,  -205.9,     -48,    0.0,     1,   0.0},
      {  -2,   0,   2,   0,   1,  1305.5,      46,    0.0,   -24,   0.0},
      {   0,   0,   2,   2,   2,     7.1,     -38,    0.0,    16,   0.0},
      {   2,   0,   2,   0,   2,     6.9,     -31,    0.0,    13,   0.0},
      {   2,   0,   0,   0,   0,    13.8,      29,    0.0,    -1,   0.0},
      {   1,   0,   2,  -2,   2,    23.9,      29,    0.0,   -12,   0.0},
      {   0,   0,   2,   0,   0,    13.6,      26,    0.0,    -1,   0.0},
      {   0,   0,   2,  -2,   0,   173.3,     -22,    0.0,     0,   0.0},
      {  -1,   0,   2,   0,   1,    27.0,      21,    0.0,   -10,   0.0},
      {   0,   2,   0,   0,   0,   182.6,      17,   -0.1,     0,   0.0},
      {   0,   2,   2,  -2,   2,    91.3,     -16,    0.1,     7,   0.0},
      {  -1,   0,   0,   2,   1,    32.0,      16,    0.0,    -8,   0.0},
      {   0,   1,   0,   0,   1,   386.0,     -15,    0.0,     9,   0.0},
      {   1,   0,   0,  -2,   1,   -31.7,     -13,    0.0,     7,   0.0},
      {   0,  -1,   0,   0,   1,  -346.6,     -12,    0.0,     6,   0.0},
      {   2,   0,  -2,   0,   0, -1095.2,      11,    0.0,     0,   0.0},
      {  -1,   0,   2,   2,   1,     9.5,     -10,    0.0,     5,   0.0},
      {   1,   0,   2,   2,   2,     5.6,      -8,    0.0,     3,   0.0},
      {   0,  -1,   2,   0,   2,    14.2,      -7,    0.0,     3,   0.0},
      {   0,   0,   2,   2,   1,     7.1,      -7,    0.0,     3,   0.0},
      {   1,   1,   0,  -2,   0,   -34.8,      -7,    0.0,     0,   0.0},
      {   0,   1,   2,   0,   2,    13.2,       7,    0.0,    -3,   0.0},
      {  -2,   0,   0,   2,   1,  -199.8,      -6,    0.0,     3,   0.0},
      {   0,   0,   0,   2,   1,    14.8,      -6,    0.0,     3,   0.0},
      {   2,   0,   2,  -2,   2,    12.8,       6,    0.0,    -3,   0.0},
      {   1,   0,   0,   2,   0,     9.6,       6,    0.0,     0,   0.0},
      {   1,   0,   2,  -2,   1,    23.9,       6,    0.0,    -3,   0.0},
      {   0,   0,   0,  -2,   1,   -14.7,      -5,    0.0,     3,   0.0},
      {   0,  -1,   2,  -2,   1,   346.6,      -5,    0.0,     3,   0.0},
      {   2,   0,   2,   0,   1,     6.9,      -5,    0.0,     3,   0.0},
      {   1,  -1,   0,   0,   0,    29.8,       5,    0.0,     0,   0.0},
      {   1,   0,   0,  -1,   0,   411.8,      -4,    0.0,     0,   0.0},
      {   0,   0,   0,   1,   0,    29.5,      -4,    0.0,     0,   0.0},
      {   0,   1,   0,  -2,   0,   -15.4,      -4,    0.0,     0,   0.0},
      {   1,   0,  -2,   0,   0,   -26.9,       4,    0.0,     0,   0.0},
      {   2,   0,   0,  -2,   1,   212.3,       4,    0.0,    -2,   0.0},
      {   0,   1,   2,  -2,   1,   119.6,       4,    0.0,    -2,   0.0},
      {   1,   1,   0,   0,   0,    25.6,      -3,    0.0,     0,   0.0},
      {   1,  -1,   0,  -1,   0, -3232.9,      -3,    0.0,     0,   0.0},
      {  -1,  -1,   2,   2,   2,     9.8,      -3,    0.0,     1,   0.0},
      {   0,  -1,   2,   2,   2,     7.2,      -3,    0.0,     1,   0.0},
      {   1,  -1,   2,   0,   2,     9.4,      -3,    0.0,     1,   0.0},
      {   3,   0,   2,   0,   2,     5.5,      -3,    0.0,     1,   0.0},
      {  -2,   0,   2,   0,   2,  1615.7,      -3,    0.0,     1,   0.0},
      {   1,   0,   2,   0,   0,     9.1,       3,    0.0,     0,   0.0},
      {  -1,   0,   2,   4,   2,     5.8,      -2,    0.0,     1,   0.0},
      {   1,   0,   0,   0,   2,    27.8,      -2,    0.0,     1,   0.0},
      {  -1,   0,   2,  -2,   1,   -32.6,      -2,    0.0,     1,   0.0},
      {   0,  -2,   2,  -2,   1,  6786.3,      -2,    0.0,     1,   0.0},
      {  -2,   0,   0,   0,   1,   -13.7,      -2,    0.0,     1,   0.0},
      {   2,   0,   0,   0,   1,    13.8,       2,    0.0,    -1,   0.0},
      {   3,   0,   0,   0,   0,     9.2,       2,    0.0,     0,   0.0},
      {   1,   1,   2,   0,   2,     8.9,       2,    0.0,    -1,   0.0},
      {   0,   0,   2,   1,   2,     9.3,       2,    0.0,    -1,   0.0},
      {   1,   0,   0,   2,   1,     9.6,      -1,    0.0,     0,   0.0},
      {   1,   0,   2,   2,   1,     5.6,      -1,    0.0,     1,   0.0},
      {   1,   1,   0,  -2,   1,   -34.7,      -1,    0.0,     0,   0.0},
      {   0,   1,   0,   2,   0,    14.2,      -1,    0.0,     0,   0.0},
      {   0,   1,   2,  -2,   0,   117.5,      -1,    0.0,     0,   0.0},
      {   0,   1,  -2,   2,   0,  -329.8,      -1,    0.0,     0,   0.0},
      {   1,   0,  -2,   2,   0,    23.8,      -1,    0.0,     0,   0.0},
      {   1,   0,  -2,  -2,   0,    -9.5,      -1,    0.0,     0,   0.0},
      {   1,   0,   2,  -2,   0,    32.8,      -1,    0.0,     0,   0.0},
      {   1,   0,   0,  -4,   0,   -10.1,      -1,    0.0,     0,   0.0},
      {   2,   0,   0,  -4,   0,   -15.9,      -1,    0.0,     0,   0.0},
      {   0,   0,   2,   4,   2,     4.8,      -1,    0.0,     0,   0.0},
      {   0,   0,   2,  -1,   2,    25.4,      -1,    0.0,     0,   0.0},
      {  -2,   0,   2,   4,   2,     7.3,      -1,    0.0,     1,   0.0},
      {   2,   0,   2,   2,   2,     4.7,      -1,    0.0,     0,   0.0},
      {   0,  -1,   2,   0,   1,    14.2,      -1,    0.0,     0,   0.0},
      {   0,   0,  -2,   0,   1,   -13.6,      -1,    0.0,     0,   0.0},
      {   0,   0,   4,  -2,   2,    12.7,       1,    0.0,     0,   0.0},
      {   0,   1,   0,   0,   2,   409.2,       1,    0.0,     0,   0.0},
      {   1,   1,   2,  -2,   2,    22.5,       1,    0.0,    -1,   0.0},
      {   3,   0,   2,  -2,   2,     8.7,       1,    0.0,     0,   0.0},
      {  -2,   0,   2,   2,   2,    14.6,       1,    0.0,    -1,   0.0},
      {  -1,   0,   0,   0,   2,   -27.3,       1,    0.0,    -1,   0.0},
      {   0,   0,  -2,   2,   1,  -169.0,       1,    0.0,     0,   0.0},
      {   0,   1,   2,   0,   1,    13.1,       1,    0.0,     0,   0.0},
      {  -1,   0,   4,   0,   2,     9.1,       1,    0.0,     0,   0.0},
      {   2,   1,   0,  -2,   0,   131.7,       1,    0.0,     0,   0.0},
      {   2,   0,   0,   2,   0,     7.1,       1,    0.0,     0,   0.0},
      {   2,   0,   2,  -2,   1,    12.8,       1,    0.0,    -1,   0.0},
      {   2,   0,  -2,   0,   1,  -943.2,       1,    0.0,     0,   0.0},
      {   1,  -1,   0,  -2,   0,   -29.3,       1,    0.0,     0,   0.0},
      {  -1,   0,   0,   1,   1,  -388.3,       1,    0.0,     0,   0.0},
      {  -1,  -1,   0,   2,   1,    35.0,       1,    0.0,     0,   0.0},
      {   0,   1,   0,   1,   0,    27.3,       1,    0.0,     0,   0.0}
  };
  double ang;
  int i, j;

  *dpsi = *deps = 0.0;

  for (i = 0; i < 106; i++) {
    ang = 0.0;
    for (j = 0; j < 5; j++) ang += nut[i][j] * f[j];
    *dpsi += (nut[i][6] + nut[i][7] * t) * sin(ang);
    *deps += (nut[i][8] + nut[i][9] * t) * cos(ang);
  }
  *dpsi *= 1E-4 * ARCSEC2RAD; /* 0.1 mas -> rad */
  *deps *= 1E-4 * ARCSEC2RAD;
}

/* coordinate rotation matrix */
#define RotionMatrixX(t,X) do { \
    (X)[0]=1.0; (X)[1]=(X)[2]=(X)[3]=(X)[6]=0.0; \
    (X)[4]=(X)[8]=cos(t); (X)[7]=sin(t); (X)[5]=-(X)[7]; \
} while (0)

#define RotionMatrixY(t,X) do { \
    (X)[4]=1.0; (X)[1]=(X)[3]=(X)[5]=(X)[7]=0.0; \
    (X)[0]=(X)[8]=cos(t); (X)[2]=sin(t); (X)[6]=-(X)[2]; \
} while (0)

#define RotionMatrixZ(t,X) do { \
    (X)[8]=1.0; (X)[2]=(X)[5]=(X)[6]=(X)[7]=0.0; \
    (X)[0]=(X)[4]=cos(t); (X)[3]=sin(t); (X)[1]=-(X)[3]; \
} while (0)

/**
 * @brief eci to ecef transformation matrix
 * @param[in] gpst
 * @param[in] erpv erp values {xp,yp,ut1_utc,lod} (rad,rad,s,s/d)
 * @param[out] U eci to ecef transformation matrix (3 x 3)
 * @param[in/out] gmst greenwich mean sidereal time (rad)
 *                     (NULL: no output)
 * @return none
 * note   : see ref [3] chap 5
 *          TODO: not thread-safe
 */
extern void gnss_Eci2Ecef(const GpsTime_t* gpst, const double* erpv, double* U, double* gmst)
{
  GpsTime_t gpst_;
  UtcTime_t utc, utc0;
  static double U_[9], gmst_;
  double eps, ze, th, z, t, t2, t3, dpsi, deps, gast, f[5];
  double R1[9], R2[9], R3[9], R[9], W[9], N[9], P[9], NP[9];
  int i;

  if (fabs(tm_GpsTimeDiff(gpst, &gpst_)) < 0.01) { /* read cache */
    for (i = 0; i < 9; i++) U[i] = U_[i];
    if (gmst) *gmst = gmst_;
    return;
  }
  gpst_ = *gpst;


  /* terrestrial time */
  tm_cvt_EpochToUtcTime(&J2000, &utc0);
  tm_cvt_GpstToUtc(gpst, &utc);
  double diff = tm_UtcTimeDiff(&utc, &utc0);
  t = diff / 86400.0 / 36525.0;
  t2 = t * t;
  t3 = t2 * t;

  /* astronomical arguments */
  ast_args(t, f);

  /* iau 1976 precession */
  ze = (2306.2181 * t + 0.30188 * t2 + 0.017998 * t3) * ARCSEC2RAD;
  th = (2004.3109 * t - 0.42665 * t2 - 0.041833 * t3) * ARCSEC2RAD;
  z = (2306.2181 * t + 1.09468 * t2 + 0.018203 * t3) * ARCSEC2RAD;
  eps = (84381.448 - 46.8150 * t - 0.00059 * t2 + 0.001813 * t3) * ARCSEC2RAD;
  RotionMatrixZ(-z, R1); RotionMatrixY(th, R2); RotionMatrixZ(-ze, R3);
  gnss_MatrixMultiply("NN", 3, 3, 3, 1.0, R1, R2, 0.0, R);
  gnss_MatrixMultiply("NN", 3, 3, 3, 1.0, R, R3, 0.0, P); /* P=Rz(-z)*Ry(th)*Rz(-ze) */

  /* iau 1980 nutation */
  nut_iau1980(t, f, &dpsi, &deps);
  RotionMatrixX(-eps - deps, R1); RotionMatrixZ(-dpsi, R2); RotionMatrixX(eps, R3);
  gnss_MatrixMultiply("NN", 3, 3, 3, 1.0, R1, R2, 0.0, R);
  gnss_MatrixMultiply("NN", 3, 3, 3, 1.0, R, R3, 0.0, N); /* N=Rx(-eps)*Rz(-dspi)*Rx(eps) */

  /* greenwich aparent sidereal time (rad) */
  gmst_ = tm_cvt_GpstToGmst(&gpst_);
  gast = gmst_ + dpsi * cos(eps);
  gast += (0.00264 * sin(f[4]) + 0.000063 * sin(2.0 * f[4])) * ARCSEC2RAD;

  /* eci to ecef transformation matrix */
  RotionMatrixY(-erpv[0], R1); RotionMatrixX(-erpv[1], R2); RotionMatrixZ(gast, R3);
  gnss_MatrixMultiply("NN", 3, 3, 3, 1.0, R1, R2, 0.0, W);
  gnss_MatrixMultiply("NN", 3, 3, 3, 1.0, W, R3, 0.0, R); /* W=Ry(-xp)*Rx(-yp) */
  gnss_MatrixMultiply("NN", 3, 3, 3, 1.0, N, P, 0.0, NP);
  gnss_MatrixMultiply("NN", 3, 3, 3, 1.0, R, NP, 0.0, U_); /* U=W*Rz(gast)*N*P */

  for (i = 0; i < 9; i++)
  {
    U[i] = U_[i];
  }
  if (gmst)
  {
    *gmst = gmst_;
  }
  return;
}

/* sun and moon position in eci (ref [4] 5.1.1, 5.2.1) -----------------------*/
static void sunmoonpos_eci(const GpsTime_t* gpst, double ut1_utc, double* rsun, double* rmoon)
{
  double diff ,t, f[5], eps, Ms, ls, rs, lm, pm, rm, sine, cose, sinp, cosp, sinl, cosl;
  UtcTime_t utc_ep, utc;

  /* diff form J2000 */
  tm_cvt_EpochToUtcTime(&J2000, &utc_ep);
  tm_UtcTimeAdd(&utc_ep, ut1_utc);
  tm_cvt_GpstToUtc(gpst, &utc);
  diff = tm_UtcTimeDiff(&utc, &utc_ep);

  /* astronomical arguments */
  t = diff / 86400.0 / 36525.0;
  ast_args(t, f);

  /* obliquity of the ecliptic */
  eps = 23.439291 - 0.0130042 * t;
  sine = sin(eps * DEG2RAD); cose = cos(eps * DEG2RAD);

  /* sun position in eci */
  if (rsun)
  {
    Ms = 357.5277233 + 35999.05034 * t;
    ls = 280.460 + 36000.770 * t + 1.914666471 * sin(Ms * DEG2RAD) + 0.019994643 * sin(2.0 * Ms * DEG2RAD);
    rs = ASU * (1.000140612 - 0.016708617 * cos(Ms * DEG2RAD) - 0.000139589 * cos(2.0 * Ms * DEG2RAD));
    sinl = sin(ls * DEG2RAD); cosl = cos(ls * DEG2RAD);
    rsun[0] = rs * cosl;
    rsun[1] = rs * cose * sinl;
    rsun[2] = rs * sine * sinl;
  }

  /* moon position in eci */
  if (rmoon)
  {
    lm = 218.32 + 481267.883 * t + 6.29 * sin(f[0]) 
      - 1.27 * sin(f[0] - 2.0 * f[3]) 
      + 0.66 * sin(2.0 * f[3]) 
      + 0.21 * sin(2.0 * f[0]) 
      - 0.19 * sin(f[1]) 
      - 0.11 * sin(2.0 * f[2]);
    
    pm = 5.13 * sin(f[2]) 
      + 0.28 * sin(f[0] + f[2]) 
      - 0.28 * sin(f[2] - f[0]) 
      - 0.17 * sin(f[2] - 2.0 * f[3]);
    
    rm = RE_WGS84 / sin((0.9508 + 0.0518 * cos(f[0]) 
      + 0.0095 * cos(f[0] - 2.0 * f[3]) 
      + 0.0078 * cos(2.0 * f[3]) 
      + 0.0028 * cos(2.0 * f[0])) * DEG2RAD);
    
    sinl = sin(lm * DEG2RAD); 
    cosl = cos(lm * DEG2RAD);
    sinp = sin(pm * DEG2RAD); 
    cosp = cos(pm * DEG2RAD);
    rmoon[0] = rm * cosp * cosl;
    rmoon[1] = rm * (cose * cosp * sinl - sine * sinp);
    rmoon[2] = rm * (sine * cosp * sinl + cose * sinp);
  }
  return;
}

/**
 * @brief get sun and moon position in ecef
 * @param[in] gpst
 * @param[in] erpv erp value {xp,yp,ut1_utc,lod} (rad,rad,s,s/d)
 * @param[in/out] rsun sun position in ecef  (m) (NULL: not output)
 * @param[in/out] rmoon moon position in ecef (m) (NULL: not output)
 * @param[out] gmst
 */
extern void gnss_SunMoonPos(const GpsTime_t* gpst, const double* erpv, double* rsun,
  double* rmoon, double* gmst)
{
  double rs[3], rm[3], U[9], gmst_;

  /* sun and moon position in eci */
  sunmoonpos_eci(gpst, erpv[2], rsun ? rs : NULL, rmoon ? rm : NULL);

  /* eci to ecef transformation matrix */
  gnss_Eci2Ecef(gpst, erpv, U, &gmst_);

  /* sun and moon postion in ecef */
  if (rsun)
  {
    gnss_MatrixMultiply("NN", 3, 1, 3, 1.0, U, rs, 0.0, rsun);
  }
  if (rmoon)
  {
    gnss_MatrixMultiply("NN", 3, 1, 3, 1.0, U, rm, 0.0, rmoon);
  }
  if (gmst)
  {
    *gmst = gmst_;
  }

  return;
}

/**
 * @brief convert coordinate from ITRF to WGS84
 * @param[in] gpst tor
 * @param[in] pd_pos_itrf ITRF
 * @param[out] pd_pos_wgs84 WGS84
 * @return None
 */
void gnss_ITRF2WGS84(const GpsTime_t* gpst, const double* pd_pos_itrf, double* pd_pos_wgs84)
{
  int32_t q_status = 1;

  const qxsi_ids_ssr2los_interface_t* p_ssr2los_interface = NULL;
#ifdef FEATURE_USE_QXWZ_SSR
  p_ssr2los_interface = qxsi_ids_get_ssr2los_interface();
  if (NULL != p_ssr2los_interface)
  {
    qxsi_sta_pos_t z_qxis_sta_pos = {0};
    qxsi_coor_xyz_t z_qxsi_coor_xyz = {0};
    z_qxis_sta_pos.pos.x = pd_pos_itrf[0];
    z_qxis_sta_pos.pos.y = pd_pos_itrf[1];
    z_qxis_sta_pos.pos.z = pd_pos_itrf[2];
    z_qxis_sta_pos.time.week_num = (int) gpst->w_week;
    z_qxis_sta_pos.time.sec_of_week = (double) (gpst->q_towMsec * TIME_MSEC_INV);
    q_status = p_ssr2los_interface->coordinate_transform(&z_qxis_sta_pos, &z_qxsi_coor_xyz);
    if (q_status == 0) {
        pd_pos_wgs84[0] = z_qxsi_coor_xyz.x;
        pd_pos_wgs84[1] = z_qxsi_coor_xyz.y;
        pd_pos_wgs84[2] = z_qxsi_coor_xyz.z;
    }
  }
#endif

  if (q_status != 0)
  {
    double d_TXYZ[21] = {0.0};
    double d_xyz_tmp[3] = {0.0};
    double d_xmas = -2.2188207730;
    double d_ymas = -7.8305482761;
    double d_zmas = 18.6523322862;
    double d_sec2rad = (1.0e-3) * atan(1.0) / 45.0 / 3600.0;
    double d_TRANS[7] = {-0.02008, -0.13688, -0.00336, 0.01123E-6, d_xmas * d_sec2rad, d_ymas * d_sec2rad,
                         d_zmas * d_sec2rad};
    /*double d_TRANS[7] = {-0.04008, -0.04688, -0.00336, 0.01123E-6,d_xmas * d_sec2rad , d_ymas * d_sec2rad,d_zmas * d_sec2rad}; //skdy */

    d_TXYZ[0] = d_TXYZ[4] = d_TXYZ[8] = 1;
    d_TXYZ[9] = pd_pos_itrf[0]; /*X1*/
    d_TXYZ[10] = pd_pos_itrf[1];/*Y1*/
    d_TXYZ[11] = pd_pos_itrf[2];/*Z1*/
    d_TXYZ[13] = pd_pos_itrf[2];     /*Z1 */
    d_TXYZ[14] = -1 * pd_pos_itrf[1];/*-Y1*/
    d_TXYZ[15] = -1 * pd_pos_itrf[2];/*-Z1*/
    d_TXYZ[17] = pd_pos_itrf[0];     /*X1 */
    d_TXYZ[18] = pd_pos_itrf[1];/*Y1 */
    d_TXYZ[19] = -1 * pd_pos_itrf[0];/*-X1*/

    gnss_MatrixMultiply("NN", 3, 1, 7, 1.0, d_TXYZ, d_TRANS, 0.0, d_xyz_tmp);

    pd_pos_wgs84[0] = pd_pos_itrf[0] + d_xyz_tmp[0];
    pd_pos_wgs84[1] = pd_pos_itrf[1] + d_xyz_tmp[1];
    pd_pos_wgs84[2] = pd_pos_itrf[2] + d_xyz_tmp[2];
  }
}

/*----------------------------------------------------------------------------*/
/* matrix routines -----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/**
 * @brief inner product
 * @param a[in]
 * @param b[in] vector a,b (n x 1)
 * @param n[in] size of vector a,b
 * @return
 */
extern double gnss_Dot(const double* a, const double* b, int n)
{
  double c = 0.0;

  while (--n >= 0) c += a[n] * b[n];
  return c;
}

/**
 * @brief euclid norm
 * @param a[in] vector a (n x 1)
 * @param n[in] size of vector a
 * @return || a ||
 */
extern double gnss_Norm(const double* a, int n)
{
  return sqrt(gnss_Dot(a, a, n));
}

/**
 * @brief Normalize vector
 * @param[in] n
 * @param[in] src vector a (n x 1)
 * @param[out] dst normlized vector (n x 1) || b || = 1
 */
extern BOOL gnss_UnitVector(const double *src, double *dst, int n)
{
  double dist = gnss_Norm(src, n);
  if (fabs(dist) < 1.0e-12)
  {
    return FALSE;
  }
  for (int i = 0; i < n; i++)
  {
    dst[i] = src[i] / dist;
  }
  return TRUE;
}

/**
 * @brief Outer product of 3d vectors
 * @param[in] a
 * @param[in] b vector a,b (3 x 1)
 * @param[out] c outer product (a x b) (3 x 1)
 */
extern void gnss_CrossMultiply(const double* a, const double* b, double* c)
{
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
}

/**
 * @brief multiply matrix, C = alpha*A*B + beta*C
 * @param[in] tr 'TT'/'TN'/'NT'; each one for matrix A and B; T: transposition;
 * @param[in] n
 * @param[in] k A: n*k
 * @param[in] m B: k*m
 * @param[in] alpha
 * @param[in] A
 * @param[in] B
 * @param[in] beta
 * @param[in/out] C
 */
extern void gnss_MatrixMultiply(const char* tr, int n, int k, int m, double alpha,
  const double* A, const double* B, double beta, double* C)
{
  double d;
  int i, j, x, f = tr[0] == 'N' ? (tr[1] == 'N' ? 1 : 2) : (tr[1] == 'N' ? 3 : 4);

  for (i = 0; i < n; i++) for (j = 0; j < k; j++) {
    d = 0.0;
    switch (f) {
    case 1: for (x = 0; x < m; x++) d += A[i + x * n] * B[x + j * m]; break;
    case 2: for (x = 0; x < m; x++) d += A[i + x * n] * B[j + x * k]; break;
    case 3: for (x = 0; x < m; x++) d += A[x + i * m] * B[x + j * m]; break;
    case 4: for (x = 0; x < m; x++) d += A[x + i * m] * B[j + x * k]; break;
    }
    if (beta == 0.0) C[i + j * n] = alpha * d;
    else C[i + j * n] = alpha * d + beta * C[i + j * n];
  }
}

/**
 * @brief keep decimal place
 * @param[in] val
 * @param[in] decimal
 * @return vaule that kepp decimal place
 */
extern double gnss_Round(double val, int decimal) 
{
  int64_t integer = (int64_t)(val);
  double decimals = fmod(val, 1);

  double cef = pow(10, decimal);
  double nearest = round(decimals * cef) / cef;

  return integer + nearest;
}


/**
 * @brief mean
 * @param[in] val 
 * @param[in] n 
 * @return 
*/
extern double gnss_MeanDouble(const double* val, uint16_t n)
{
  double mean = 0.0;
  double sum = 0.0;
  if (0 != n)
  {
    for (uint16_t i = 0; i < n; i++)
    {
      sum += val[i];
    }
    mean = sum / n;
  }
  return mean;
}

/**
 * @brief mean
 * @param[in] val
 * @param[in] n
 * @return
*/
extern float gnss_MeanFloat(const float* val, uint16_t n)
{
  float mean = 0.0;
  float sum = 0.0;
  if (0 != n)
  {
    for (uint16_t i = 0; i < n; i++)
    {
      sum += val[i];
    }
    mean = sum / n;
  }
  return mean;
}

/**
 * @brief Check the input time is aligned with second
 * @param[in] t_fullMsec - Full Millisecond start at UTC 1970-01-01 00:00:00, unit: 1ms
 * @param[in] pq_preSecondAlignMsec - Previous aligned time
 * @param[in] f_AlignInterval - Interval setting
 * @return TRUE - align with second
 *         FALSE - not align with second
 */
uint8_t gnss_CheckMeasSecondAlign(uint64_t t_fullMsec,
  uint64_t* pq_preSecondAlignMsec, float f_AlignInterval)
{
  uint32_t q_AlignIntervalMs = (uint32_t)(1000 * f_AlignInterval);
  uint32_t q_Threshold = q_AlignIntervalMs / 10;
  uint64_t q_N = t_fullMsec / q_AlignIntervalMs;

  if (*pq_preSecondAlignMsec == 0)
  {
    *pq_preSecondAlignMsec = t_fullMsec;
    return FALSE;
  }

  if (t_fullMsec > (q_AlignIntervalMs / 2 + q_AlignIntervalMs * q_N))
  {
    q_N++;
  }

  if ((t_fullMsec > (q_N * q_AlignIntervalMs - q_Threshold))
    && (t_fullMsec < (q_N* q_AlignIntervalMs + q_Threshold)))
  {
    if (t_fullMsec > (q_AlignIntervalMs / 2+ *pq_preSecondAlignMsec))
    {
      *pq_preSecondAlignMsec = t_fullMsec;
      return TRUE;
    }
  }

  return FALSE;
}

/**
 * @brief  Check the input time is overrun second, (q_TowMsec - *pq_preSecondAlignMsec) >=f_AlignInterval
 * @param[in] t_fullMsec - Full Millisecond start at UTC 1970-01-01 00:00:00, unit: 1ms
 * @param[in] pq_preSecondAlignMsec - Previous aligned time
 * @param[in] f_AlignInterval - Interval setting
 * @return TRUE - align with second
 *         FALSE - not align with second
 */
uint8_t gnss_CheckMeasSecondOverrun(uint64_t t_fullMsec,
                                    uint64_t* pq_preSecondAlignMsec, float f_AlignInterval)
{
  if ((int64_t)(t_fullMsec - *pq_preSecondAlignMsec) >= f_AlignInterval * TIME_MSEC)
  {
    *pq_preSecondAlignMsec = t_fullMsec;
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}

/**
 * @brief convert the satellite coordinate from the transmit time to receiver time
 * @param[in] pd_siteEcef the site coordinate in ECEF
 * @param[in] pd_satEcef the satellite coordinate of transmit time in ECEF
 * @param[out] pd_satEcefRot the satellite coordinate of receiver time in ECEF
 * @return void
 */
void gnss_satRot(const double pd_siteEcef[3], const double pd_satEcef[3], double pd_satEcefRot[3])
{
    uint8_t u_i = 0;
    double pd_deltaCoor[3] = { 0.0 };
    double d_dist = 0.0;
    double d_rotateAngle = 0.0;
    double d_sinRotAngle = 0.0;
    double d_cosRotAngle = 0.0;
    for (u_i = 0; u_i < 3; ++u_i)
    {
        pd_satEcefRot[u_i] = 0.0;
    }
    // correct for earth rotation apply ClockWise(negative rotation)
    pd_deltaCoor[0] = pd_satEcef[0] - pd_siteEcef[0];
    pd_deltaCoor[1] = pd_satEcef[1] - pd_siteEcef[1];
    pd_deltaCoor[2] = pd_satEcef[2] - pd_siteEcef[2];
    d_dist = sqrt(pd_deltaCoor[0] * pd_deltaCoor[0] + pd_deltaCoor[1] * pd_deltaCoor[1] + pd_deltaCoor[2] * pd_deltaCoor[2]);
    d_rotateAngle = (d_dist / CLIGHT) * OMGE_GPS;
    d_sinRotAngle = sin(d_rotateAngle);
    d_cosRotAngle = cos(d_rotateAngle);
    pd_satEcefRot[0] = pd_satEcef[0] * d_cosRotAngle + pd_satEcef[1] * d_sinRotAngle;
    pd_satEcefRot[1] = -pd_satEcef[0] * d_sinRotAngle + pd_satEcef[1] * d_cosRotAngle;
    pd_satEcefRot[2] = pd_satEcef[2];
}

/**
 * @brief satellite azimuth/elevation angle
 * @param pos geodetic position {lat,lon,h} (rad,m)
 * @param e receiver-to-satellilte unit vevtor (ecef)
 * @param azel azimuth/elevation {az,el} (rad) (NULL: no output)
 *                               (0.0<=azel[0]<2*pi,-pi/2<=azel[1]<=pi/2)
 */
double gnss_Satazel(const double *pos, const double *e, float* azel)
{
  float az=0.0;
  float el = (float)(PI / 2.0);
  double enu[3] = {0};

  if (pos[2] > -RE_WGS84)
  {
    gnss_Ecef2Enu(pos, e, enu);
    az = (float)(gnss_Dot(enu, enu, 2) < 1E-12 ? 0.0 : atan2(enu[0], enu[1]));
    if (az < 0.0) az += (float)(2 * PI);
    el = (float)(asin(enu[2]));
  }
  if (azel)
  {
    azel[0] = az;
    azel[1] = el;
  }
  return el;
}

/**
 * @brief calculate the unit vector from satellite to the site
 * @param[in] pd_siteEcef the site coordinate in ECEF
 * @param[in] pd_satEcef the satellite coordinate of receiver time in ECEF
 * @param[out] pd_unitVector the unit vector of receiver time in ECEF
 * @return the distance from satellite to site
 */
double gnss_unitVector(const double pd_siteEcef[3], const double pd_satEcef[3], double pd_unitVector[3])
{
    uint8_t u_i = 0;
    double d_dist = 0.0;
    for (u_i = 0; u_i < 3; ++u_i)
    {
        pd_unitVector[u_i] = pd_satEcef[u_i] - pd_siteEcef[u_i];
    }
    d_dist = sqrt(pd_unitVector[0] * pd_unitVector[0] + pd_unitVector[1] * pd_unitVector[1] + pd_unitVector[2] * pd_unitVector[2]);
    for (u_i = 0; u_i < 3; ++u_i)
    {
        pd_unitVector[u_i] /= d_dist;
    }
    return d_dist;
}

/**
 * @brief initilize the GPS Time
 * @param[in] GpsTime_t*
 * @return void
 */
void tm_initGpsTime(GpsTime_t* pz_gpsTime)
{
    if (NULL != pz_gpsTime)
    {
        pz_gpsTime->q_towMsec = 0;
        pz_gpsTime->q_subNsec = 0;
        pz_gpsTime->w_week = 0;
        pz_gpsTime->t_fullMsec = 0;
    }
    return;
}
/**
 * @brief Convert Epoch time to time stamp millisecond
 * @param[in] epoch
 * @return t_UnixMsec - unix time stamp millisecond
 */
uint64_t tm_cvt_EpochToUnixMsec(const EpochTime_t* pz_epoch)
{
  const uint64_t doy[] = { 1,32,60,91,121,152,182,213,244,274,305,335 };
  uint64_t year = pz_epoch->year;
  uint64_t month = pz_epoch->month;
  uint64_t day = pz_epoch->day;
  uint64_t t_unixMsec = 0;

  if ((pz_epoch->year < 1970) ||
    (pz_epoch->month < 1 || pz_epoch->month > 12) ||
    (pz_epoch->day < 1 || pz_epoch->day > 31) ||
    (pz_epoch->hour > 24) ||
    (pz_epoch->min > 60) || (pz_epoch->second > 60.0))
  {
    return 0;
  }

  uint64_t days = (year - 1970) * 365 +
    (year - 1969) / 4 +
    doy[month - 1] + day - 2 +
    (year % 4 == 0 && month >= 3 ? 1 : 0);
  t_unixMsec = (uint64_t)days * DAY_MSEC +
                    (uint64_t)pz_epoch->hour * HOUR_MSEC +
                    (uint64_t)pz_epoch->min * MINUTE_MSEC +
                    (uint64_t)(1000.0 * pz_epoch->second);
  return t_unixMsec;
}

/**
 * @brief Convert Epoch time to utc time
 * @param[in] pz_epoch
 * @param[out] pz_utcTime
 */
void tm_cvt_EpochToUtcTime(const EpochTime_t* pz_epoch, UtcTime_t* pz_utcTime)
{
  float f_subNsec = (pz_epoch->second * 1000 - (uint32_t)(pz_epoch->second * 1000));
  pz_utcTime->q_subNsec = (uint32_t)(f_subNsec * 1000000);
  pz_utcTime->t_unixMsec = tm_cvt_EpochToUnixMsec(pz_epoch);
}

/**
 * @brief Convert unix time stamp millisecond to Epoch time
 * @param[in] t_UnixMsec
 * @return Epoch time
 */
void tm_cvt_UnixMsecToEpoch(uint64_t t_UnixMsec, EpochTime_t* pz_epoch)
{
  const uint16_t mday[] = { /* # of days in a month */
      31,28,31,30,31,30,31,31,30,31,30,31,
      31,28,31,30,31,30,31,31,30,31,30,31,
      31,29,31,30,31,30,31,31,30,31,30,31,
      31,28,31,30,31,30,31,31,30,31,30,31
  };

  uint16_t days = 0;
  uint16_t day = 0;
  uint16_t mon = 0;
  uint32_t sec = 0;
  uint16_t msec = 0;

  /* leap year if year%4==0 in 1901-2099 */
  days = (uint16_t)(t_UnixMsec / (86400 * 1000));
  sec = (uint32_t)((t_UnixMsec / 1000) % 86400);
  msec = (uint16_t)(t_UnixMsec % 1000);
  for (day = days % 1461, mon = 0; mon < 48; mon++)
  {
    if (day < mday[mon])
    {
      break;
    }
    day -= mday[mon];
  }

  pz_epoch->year = 1970 + days / 1461 * 4 + mon / 12;
  pz_epoch->month = mon % 12 + 1;
  pz_epoch->day = day + 1;
  pz_epoch->hour = sec / 3600;
  pz_epoch->min = sec % 3600 / 60;
  pz_epoch->second = sec % 60 + msec * 0.001f;

  return;
}

/**
 * @brief Convert Epoch time to utc time
 * @param[in] t_UnixMsec
 * @param[out] pz_utcTime
 */
void tm_cvt_UnixMsecToUtcTime(uint64_t t_UnixMsec, UtcTime_t* pz_utcTime)
{
  pz_utcTime->t_unixMsec = t_UnixMsec;
  pz_utcTime->q_subNsec = 0;
}

/**
 * @brief Convert utc time to Epoch time
 * @param[in] pz_utcTime
 * @param[out] pz_epoch
 */
void tm_cvt_UtcTimeToEpoch(const UtcTime_t* pz_utcTime, EpochTime_t* pz_epoch)
{
  tm_cvt_UnixMsecToEpoch(pz_utcTime->t_unixMsec, pz_epoch);
  pz_epoch->second += pz_utcTime->q_subNsec * 0.000001f;
}

/**
 * @brief Convert Utc time to Unix Millsecond
 * @param[in] pz_utcTime
 * @return t_UnixMsec - unix time stamp millisecond
 */
uint64_t tm_cvt_UtcTimeToUnixMsec(UtcTime_t* pz_utcTime)
{
  return pz_utcTime->t_unixMsec;
}
/**
 * @brief Convert Utc time to GPST
 * @param[in] pz_utcTime
 * @return
 */
void tm_cvt_UtcTimeToGpst(UtcTime_t* pz_utcTime, GpsTime_t* gpst)
{
  UtcTime_t utc0 = { 0 };
  uint16_t week = 0;
  double tow = 0;

  // base gpst0
  tm_cvt_EpochToUtcTime(&gz_epoch_gpst0, &utc0);

  double diff = tm_UtcTimeDiff(pz_utcTime, &utc0);

  // week + tow
  week = (uint16_t)(diff / (86400 * 7.0));
  tow = diff - (double)week * 86400 * 7;

  // tow->gpst
  tm_cvt_SetGpst(gpst, week, tow);
  return ;
}

double tm_cvt_Utc2Sec(const UtcTime_t* utc, UtcTime_t* newday)
{
  double sec = 0.0;
  EpochTime_t epoch = {0};
  tm_cvt_UtcTimeToEpoch(utc, &epoch);
  sec = ((double)(epoch.hour)) * ((double)(HOUR_SEC)) + ((double)(epoch.min)) * ((double)(MINUTE_SEC)) + (double)epoch.second;

  epoch.hour = epoch.min = 0;
  epoch.second = 0.0f;
  tm_cvt_EpochToUtcTime(&epoch, newday);
  return sec;
}

/**
 * @brief Insert a new latest leap second record into list table
 * @param[in] year
 * @param[in] month
 * @param[in] leap_second
 * @return TRUE - Insert Success
 *         FALSE - Insert Fail
 */
uint8_t tm_LeapSecond_Insert(uint16_t year, uint16_t month, uint16_t leap_second)
{
  uint8_t u_table_size = LEAP_SECOND_TABLE_SIZE;

  uint16_t latest_year = g_LeapSecond_Table[0].year;
  uint16_t latest_month = g_LeapSecond_Table[0].month;
  uint16_t latest_leap = g_LeapSecond_Table[0].leapsecond;
  uint8_t b_NeedInsert = FALSE;

  if (leap_second > latest_leap)
  {
    if (year > latest_year)
    {
      b_NeedInsert = TRUE;
    }
    else if ((year == latest_year) && (month > latest_month))
    {
      b_NeedInsert = TRUE;
    }
  }

  if (FALSE == b_NeedInsert)
  {
    return FALSE;
  }

  for (int i = u_table_size - 1; i > 0; i--)
  {
    memcpy(&g_LeapSecond_Table[i], &g_LeapSecond_Table[i - 1], sizeof(g_LeapSecond_Table[0]));
  }

  g_LeapSecond_Table[0].year = year;
  g_LeapSecond_Table[0].month = (uint8_t)month;
  g_LeapSecond_Table[0].day = 1;
  g_LeapSecond_Table[0].hour = 0;
  g_LeapSecond_Table[0].min = 0;
  g_LeapSecond_Table[0].second = 0;
  g_LeapSecond_Table[0].leapsecond = (uint8_t)leap_second;

  EpochTime_t epoch = {
    g_LeapSecond_Table[0].year,
    g_LeapSecond_Table[0].month,
    g_LeapSecond_Table[0].day,
    g_LeapSecond_Table[0].hour,
    g_LeapSecond_Table[0].min,
    g_LeapSecond_Table[0].second * 1.0f
  };
  g_LeapSecond_Table[0].t_unixMsec = tm_cvt_EpochToUnixMsec(&epoch);

  return TRUE;
}

/**
 * @brief Get Current Leap Second from a input epoch time
 * @param[in] epoch
 * @return leap_second - the current leap second
 */
uint8_t tm_LeapSecond_FromEpoch(const EpochTime_t* z_epoch)
{
  uint8_t u_table_size = LEAP_SECOND_TABLE_SIZE;
  uint8_t u_leap_second = 0;
  uint16_t cur_year = z_epoch->year;
  uint16_t cur_month = z_epoch->month;

  for (int i = 0; i < u_table_size; i++)
  {
    uint16_t temp_year = g_LeapSecond_Table[i].year;
    uint16_t temp_month = g_LeapSecond_Table[i].month;
    uint8_t  temp_leap_second = g_LeapSecond_Table[i].leapsecond;

    if (cur_year > temp_year)
    {
      u_leap_second = temp_leap_second;
      break;
    }
    else if ((cur_year == temp_year) && (cur_month > temp_month))
    {
      u_leap_second = temp_leap_second;
      break;
    }
  }
  return u_leap_second;
}

/**
 * @brief Get Current Leap Second from a input time stamp millisecond
 * @param[in] t_unixMsec
 * @return leap_second - the current leap second
 */
uint8_t tm_LeapSecond_FromUnixMsec(uint64_t t_unixMsec)
{
  uint8_t u_table_size = LEAP_SECOND_TABLE_SIZE;
  uint8_t u_leap_second = 0;

  for (int i = 0; i < u_table_size; i++)
  {
    if (t_unixMsec > g_LeapSecond_Table[i].t_unixMsec)
    {
      u_leap_second = g_LeapSecond_Table[i].leapsecond;
      break;
    }
  }

  return u_leap_second;
}

/**
 * @brief Convert Full Millisecond to unix time stamp millisecond
 * @param[in] t_FullMsec
 * @return time stamp millisecond
 */
uint64_t tm_cvt_FullMsecToUnixMsec(uint64_t t_FullMsec)
{
  uint64_t t_unixMsec = 0;
  uint8_t u_table_size = LEAP_SECOND_TABLE_SIZE;
  uint64_t leap_second = 0;

  for (int i = 0; i < u_table_size; i++)
  {
    if ((t_FullMsec - g_LeapSecond_Table[i].leapsecond) >= g_LeapSecond_Table[i].t_unixMsec)
    {
      leap_second = g_LeapSecond_Table[i].leapsecond;
      break;
    }
  }

  t_unixMsec = t_FullMsec - 1000 * leap_second;
  return t_unixMsec;
}

/**
 * @brief Convert unix time stamp millisecond to Full Millisecond
 * @param[in] t_unixMsec
 * @return Full Millisecond
 */
uint64_t tm_cvt_UnixMsecToFullMsec(uint64_t t_unixMsec)
{
  uint64_t t_fullMsec = 0;
  uint8_t u_table_size = LEAP_SECOND_TABLE_SIZE;
  uint64_t leap_second = 0;

  for (int i = 0; i < u_table_size; i++)
  {
    if (t_unixMsec >= g_LeapSecond_Table[i].t_unixMsec)
    {
      leap_second = g_LeapSecond_Table[i].leapsecond;
      break;
    }
  }

  t_fullMsec = t_unixMsec + 1000 * leap_second;
  return t_fullMsec;
}

/**
 * @brief Convert GPS week and tow to full GNSS millisecond
 * @param[in] w_GpsWeek - GPS Week
 * @param[in] d_GpsTow - GPS tow
 * @param[out] t_FullMsec - Full GNSS Millisecond
 * @return
 */
uint64_t tm_cvt_GpsWeekTowToFullMsec(uint16_t w_GpsWeek, uint32_t q_GpsTowMsec)
{
  uint64_t t_FullMsec = tm_cvt_EpochToUnixMsec(&gz_epoch_gpst0);
  t_FullMsec += (uint64_t)w_GpsWeek * WEEK_MSEC + (uint64_t)q_GpsTowMsec;
  return t_FullMsec;
}

/**
 * @brief Convert full GNSS second to GPS week and tow
 * @param[in] w_time - integer of full GNSS second
 * @param[in] d_sec - decimal of full GNSS second
 * @param[out] w_GpsWeek - GPS Week
 * @param[out] d_GpsTow - GPS tow
 * @return
 */
void tm_cvt_FullMsecToGpsWeekTow(uint32_t w_time, double d_sec, uint16_t* w_GpsWeek, double* q_GpsTow)
{
  uint64_t t_FullMsec = tm_cvt_EpochToUnixMsec(&gz_epoch_gpst0);
  double dt = w_time * 1000.0 - t_FullMsec;
  if (dt <= 0)
  {
    *w_GpsWeek = 0;
    *q_GpsTow = 0.0;
    return;
  }
  dt = dt / 1000.0;
  *w_GpsWeek = (int)(dt / (86400 * 7.0));
  *q_GpsTow = (double)(dt - (double)(*w_GpsWeek) * 86400 * 7) + d_sec;

  return;
}

/**
 * @brief Convert GPS time to GAL Time
 * @param[in] gpst - GPS time structure
 * @param[out] gst - GAL time structure
 * @return
 */
void tm_cvt_GpstToGst(GpsTime_t* gpst, GalTime_t* gst)
{
  if (gpst->w_week < 1024)
  {
    gst->w_week = gpst->w_week + 2048 - GPS_GAL_OFFSET_WEEK;
  }
  else
  {
    gst->w_week = gpst->w_week - GPS_GAL_OFFSET_WEEK;
  }

  gst->q_towMsec = gpst->q_towMsec;
  gst->q_subNsec = gpst->q_subNsec;
  gst->t_fullMsec = tm_cvt_GpsWeekTowToFullMsec(gpst->w_week, gpst->q_towMsec);
}

/**
 * @brief Convert GAL time to GPS Time
 * @param[in] gst - GAL time structure
 * @param[out] gpst - GPS time structure
 * @return
 */
void tm_cvt_GstToGpst(GalTime_t* gst, GpsTime_t* gpst)
{
  gpst->w_week = gst->w_week + GPS_GAL_OFFSET_WEEK;
  gpst->q_towMsec = gst->q_towMsec;
  gpst->q_subNsec = gst->q_subNsec;
  gpst->t_fullMsec = tm_cvt_GpsWeekTowToFullMsec(gpst->w_week, gpst->q_towMsec);
}

/**
 * @brief Convert GPS time to BDS Time
 * @param[in] gpst - GPS time structure
 * @param[out] bdt - BDS time structure
 * @return
 */
void tm_cvt_GpstToBdt(GpsTime_t* gpst, BdsTime_t* bdt)
{
  if (gpst->q_towMsec >= GPS_BDS_OFFSET_TOW_MSEC)
  {
    /* week roll case */
    if (gpst->w_week < 1024)
    {
      bdt->w_week = gpst->w_week + 2048 - GPS_BDS_OFFSET_WEEK;
    }
    else
    {
      bdt->w_week = gpst->w_week - GPS_BDS_OFFSET_WEEK;
    }

    bdt->q_towMsec = gpst->q_towMsec - GPS_BDS_OFFSET_TOW_MSEC;
  }
  else
  {
    /* week roll case */
    if (gpst->w_week < 1024)
    {
      bdt->w_week = gpst->w_week + 2048 - GPS_BDS_OFFSET_WEEK - 1;
    }
    else
    {
      bdt->w_week = gpst->w_week - GPS_BDS_OFFSET_WEEK - 1;
    }

    bdt->q_towMsec = gpst->q_towMsec + WEEK_MSEC - GPS_BDS_OFFSET_TOW_MSEC;
  }

  bdt->q_subNsec = gpst->q_subNsec;
  bdt->t_fullMsec = tm_cvt_GpsWeekTowToFullMsec(gpst->w_week, gpst->q_towMsec);
}

/**
 * @brief Convert BDS time to GPS Time
 * @param[in] bdt - BDS time structure
 * @param[out] gpst - GPS time structure
 * @return
 */
void tm_cvt_BdtToGpst(BdsTime_t* bdt, GpsTime_t* gpst)
{
  gpst->q_towMsec = bdt->q_towMsec + GPS_BDS_OFFSET_TOW_MSEC;
  gpst->w_week = bdt->w_week + GPS_BDS_OFFSET_WEEK;
  gpst->q_subNsec = bdt->q_subNsec;
  if (gpst->q_towMsec > WEEK_MSEC)
  {
    gpst->q_towMsec -= WEEK_MSEC;
    gpst->w_week++;
  }
  gpst->t_fullMsec = tm_cvt_GpsWeekTowToFullMsec(gpst->w_week, gpst->q_towMsec);
}

/**
 * @brief Set a GPS Time by input GPS week and tow
 * @param[in] w_GpsWeek - GPS Week
 * @param[in] d_GpsTow - GPS tow
 * @param[out] gpst - GPS Time Struct
 * @return
 */
void tm_cvt_SetGpst(GpsTime_t* gpst, uint16_t w_GpsWeek, double d_GpsTow)
{
  gpst->w_week = w_GpsWeek;
  gpst->q_towMsec = (uint32_t)(d_GpsTow * TIME_MSEC);
  gpst->q_subNsec = (uint32_t)(1e6 * (d_GpsTow * TIME_MSEC - gpst->q_towMsec));
  if (w_GpsWeek != 0)
  {
    gpst->t_fullMsec = tm_cvt_GpsWeekTowToFullMsec(gpst->w_week, gpst->q_towMsec);
  }
  else
  {
    gpst->t_fullMsec = 0;
  }
}

/**
 * @brief Set a GAL Time by input GAL week and tow
 * @param[in] w_GalWeek - GAL Week
 * @param[in] d_GalTow - GAL tow
 * @param[out] gst - GAL Time Struct
 * @return
 */
void tm_cvt_SetGst(GalTime_t* gst, uint16_t w_GalWeek, double d_GalTow)
{
  gst->w_week = w_GalWeek;
  gst->q_towMsec = (uint32_t)(d_GalTow * TIME_MSEC + 0.5);
  gst->q_subNsec = (uint32_t)(1e6 * (d_GalTow * TIME_MSEC - gst->q_towMsec));

  {
    GpsTime_t gpst = { 0 };
    tm_cvt_GstToGpst(gst, &gpst);
    gst->t_fullMsec = gpst.t_fullMsec;
  }
}

/**
 * @brief Set a BDS Time by input BDS week and tow
 * @param[in] w_BDSWeek - BDS Week
 * @param[in] d_BDSTow - BDS tow
 * @param[out] bdt - BDS Time Struct
 * @return
 */
void tm_cvt_SetBdt(BdsTime_t* bdt, uint16_t w_BdsWeek, double d_BdsTow)
{
  bdt->w_week = w_BdsWeek;
  bdt->q_towMsec = (uint32_t)(d_BdsTow * TIME_MSEC + 0.5);
  bdt->q_subNsec = (uint32_t)(1e6 * (d_BdsTow * TIME_MSEC - bdt->q_towMsec));
  {
    GpsTime_t gpst = { 0 };
    tm_cvt_BdtToGpst(bdt, &gpst);
    bdt->t_fullMsec = gpst.t_fullMsec;
  }
}

/**
 * @brief Convert GPS Time to Utc time
 * @param[in] gpst - GPS Time Week
 * @return UtcTime_t
 */
void tm_cvt_GpstToUtc(const GpsTime_t* gpst, UtcTime_t* pz_utcTime)
{
  uint64_t t_fullMsec = gpst->t_fullMsec;
  pz_utcTime->t_unixMsec = tm_cvt_FullMsecToUnixMsec(t_fullMsec);
  pz_utcTime->q_subNsec = gpst->q_subNsec;
  return;
}

/**
 * @brief Convert GAL Time to Utc time
 * @param[in] gst - GAL Time Week
 * @return UtcTime_t
 */
void tm_cvt_GstToUtc(GalTime_t* gst, UtcTime_t* pz_utcTime)
{
  uint64_t t_fullMsec = gst->t_fullMsec;
  pz_utcTime->t_unixMsec = tm_cvt_FullMsecToUnixMsec(t_fullMsec);
  pz_utcTime->q_subNsec = gst->q_subNsec;
  return;
}

/**
 * @brief Convert BDS Time to Utc time
 * @param[in] bdt - BDS Time Week
 * @return UtcTime_t
 */
void tm_cvt_BdtToUtc(BdsTime_t* bdt, UtcTime_t* pz_utcTime)
{
  uint64_t t_fullMsec = bdt->t_fullMsec;
  pz_utcTime->t_unixMsec = tm_cvt_FullMsecToUnixMsec(t_fullMsec);
  pz_utcTime->q_subNsec = bdt->q_subNsec;
  return;
}

/**
 * @brief Convert GPS Time to Epoch time
 * @param[in] gpst - GPS Time Week
 * @return EpochTime_t
 */
void tm_cvt_GpstToEpoch(const GpsTime_t* gpst, EpochTime_t* pz_epoch)
{
  if (gpst->t_fullMsec <= FABS_ZEROS)
  {
    return;
  }
  uint64_t t_fullMsec = gpst->t_fullMsec;
  uint64_t t_unixMsec = tm_cvt_FullMsecToUnixMsec(t_fullMsec);
  tm_cvt_UnixMsecToEpoch(t_unixMsec, pz_epoch);
  return;
}

/**
 * @brief Convert GAL Time to Epoch time
 * @param[in] gst - GAL Time Week
 * @return EpochTime_t
 */
void tm_cvt_GstToEpoch(GalTime_t* gst, EpochTime_t* pz_epoch)
{
  uint64_t t_fullMsec = gst->t_fullMsec;
  uint64_t t_unixMsec = tm_cvt_FullMsecToUnixMsec(t_fullMsec);
  tm_cvt_UnixMsecToEpoch(t_unixMsec, pz_epoch);
  return;
}

/**
 * @brief Convert BDS Time to Epoch time
 * @param[in] bdt - BDS Time Week
 * @return EpochTime_t
 */
void tm_cvt_BdtToEpoch(BdsTime_t* bdt, EpochTime_t* pz_epoch)
{
  uint64_t t_fullMsec = bdt->t_fullMsec;
  uint64_t t_unixMsec = tm_cvt_FullMsecToUnixMsec(t_fullMsec);
  tm_cvt_UnixMsecToEpoch(t_unixMsec, pz_epoch);
  return;
}

/**
 * @brief convert gpst to gmst (Greenwich mean sidereal time)
 * @param[in] gpst
 * @param[in] ut1_utc
 * @return gmst (rad)
 */
double tm_cvt_GpstToGmst(GpsTime_t* gpst)
{
  UtcTime_t utc_ep = { 0 }, utc_cur = { 0 }, ut_day = { 0};
  double ut, diff = 0;

  // from J2000
  tm_cvt_EpochToUtcTime(&J2000, &utc_ep);
  tm_cvt_GpstToUtc(gpst, &utc_cur);
  ut =  tm_cvt_Utc2Sec(&utc_cur, &ut_day);
  diff = tm_UtcTimeDiff(&ut_day, &utc_ep);

  // cal
  double t1 = diff / 86400.0 / 36525.0;
  double t2 = t1 * t1;
  double t3 = t2 * t1;
  double gmst0 = 24110.54841 + 8640184.812866 * t1 + 0.093104 * t2 - 6.2E-6 * t3;
  double gmst = gmst0 + 1.002737909350795 * ut;

  return fmod(gmst, 86400.0) * PI / 43200.0; /* 0 <= gmst <= 2*PI */
}

/**
 * @brief Add Gpst with t seccond
 * @param[in] gpst - GPS time
 * @param[in] t - Delta second
 * @return
 */
void tm_GpstTimeAdd(GpsTime_t* gpst, double t)
{
  int32_t q_tMesc = (int32_t)(t * TIME_MSEC);
  int32_t q_tSubNesc = (int32_t)(1e6 * (t * TIME_MSEC - q_tMesc));

  if (gpst->t_fullMsec == 0)
  {
    gpst->t_fullMsec = tm_cvt_GpsWeekTowToFullMsec(gpst->w_week, gpst->q_towMsec);
  }

  if (q_tSubNesc < 0)
  {
    gpst->q_subNsec += 1000000;
    gpst->t_fullMsec--;
  }

  gpst->q_subNsec += q_tSubNesc;
  gpst->t_fullMsec += q_tMesc;
  if (gpst->q_subNsec > 999999)
  {
    gpst->q_subNsec -= 1000000;
    gpst->t_fullMsec++;
  }

  uint64_t t_NewFullMsec = gpst->t_fullMsec - tm_cvt_GpsWeekTowToFullMsec(0, 0);
  gpst->w_week = (uint16_t)(t_NewFullMsec / WEEK_MSEC);
  gpst->q_towMsec = (uint32_t)(t_NewFullMsec % WEEK_MSEC);
}

/**
 * @brief Add Gst with t seccond
 * @param[in] gst - GAL time
 * @param[in] t - Delta second
 * @return
 */
void tm_GstTimeAdd(GalTime_t* gst, double t)
{
  GpsTime_t gpst = { 0 };
  tm_cvt_GstToGpst(gst, &gpst);
  tm_GpstTimeAdd(&gpst, t);
  tm_cvt_GpstToGst(&gpst, gst);
}

/**
 * @brief Add Bdt with t seccond
 * @param[in] Bdt - BDS time
 * @param[in] t - Delta second
 * @return
 */
void tm_BdtTimeAdd(BdsTime_t* bdt, double t)
{
  GpsTime_t gpst = { 0 };
  tm_cvt_BdtToGpst(bdt, &gpst);
  tm_GpstTimeAdd(&gpst, t);
  tm_cvt_GpstToBdt(&gpst, bdt);
}

/**
 * @brief Calculate the difference of gpst1 and gpst2
 * @param[in] gpst1
 * @param[in] gpst2
 * @return
 */
double tm_GpsTimeDiff(const GpsTime_t* gpst1, const GpsTime_t* gpst2)
{
  double diff = 0.0;
  uint64_t t_fullMsec1 = 0;
  uint64_t t_fullMsec2 = 0;

  if (0 == gpst1->t_fullMsec)
  {
    t_fullMsec1 = tm_cvt_GpsWeekTowToFullMsec(gpst1->w_week, gpst1->q_towMsec);
  }
  else
  {
    t_fullMsec1 = gpst1->t_fullMsec;
  }

  if (0 == gpst2->t_fullMsec)
  {
    t_fullMsec2 = tm_cvt_GpsWeekTowToFullMsec(gpst2->w_week, gpst2->q_towMsec);
  }
  else
  {
    t_fullMsec2 = gpst2->t_fullMsec;
  }

  int32_t diff_Nsec = gpst1->q_subNsec - gpst2->q_subNsec;
  int64_t diff_Msec = t_fullMsec1 - t_fullMsec2;

  diff = diff_Msec * TIME_MSEC_INV + diff_Nsec * TIME_NSEC_INV;
  return diff;
}


/**
 * @brief Get mjd (Modified Julian Day)
 * @param[in] gpst GPST
 * @return seconds
 */
extern long double tm_GetMJD(const GpsTime_t * gpst)
{
  UtcTime_t utc0 = { 0 }, utc = { 0 };

  // mjd = 51544.5; utc
  const EpochTime_t epbase= { 2000,1,1,12,0,0 };

  tm_cvt_EpochToUtcTime(&epbase, &utc0);

  tm_cvt_GpstToUtc(gpst, &utc);

  double diff = tm_UtcTimeDiff(&utc, &utc0);
  long double mjd = 51544.5 + diff / 86400.0;
  return mjd;
}

/**
 * @brief Get jd (Julian Day)
 * @param t gpst
 * @return seconds
 */
extern long double tm_GetJD(const GpsTime_t* t)
{
  return tm_GetMJD(t) + MJD_JDAY;
}

/**
 * @brief Add Utc time with t seccond
 * @param[in] gpst - GPS time
 * @param[in] t - Delta second
 * @return
 */
void tm_UtcTimeAdd(UtcTime_t* utc, double t)
{
  uint64_t fms = tm_cvt_UnixMsecToFullMsec(utc->t_unixMsec);
  double full = fms * TIME_MSEC_INV + utc->q_subNsec * TIME_NSEC_INV + t;
  uint64_t unixmsec =  tm_cvt_FullMsecToUnixMsec((uint64_t)full * TIME_MSEC);
  tm_cvt_UnixMsecToUtcTime(unixmsec, utc);
}

/**
 * @brief Calculate the difference of utc1 and utc2
 * @param[in] utc1
 * @param[in] utc2
 * @return difference
 */
double tm_UtcTimeDiff(const UtcTime_t* utc1, const UtcTime_t* utc2)
{
  uint64_t fms1 = tm_cvt_UnixMsecToFullMsec(utc1->t_unixMsec);
  uint64_t fms2 = tm_cvt_UnixMsecToFullMsec(utc2->t_unixMsec);
  double diff = (fms1 - fms2) * TIME_MSEC_INV + (utc1->q_subNsec - utc2->q_subNsec) * TIME_NSEC_INV;
  return diff;
}

/**
 * @brief Map svid and constellation to index of measurement buffer
 * @param[in] svid
 * @param[in] constellation
 * @return idx, 0 ~ ALL_GNSS_SYS_SV_NUMBER-1: success
                ALL_GNSS_SYS_SV_NUMBER:fail
 */
uint32_t gnss_cvt_Svid2SvIndex(uint8_t svid, uint8_t constellation)
{
  uint32_t idx = ALL_GNSS_SYS_SV_NUMBER;
  switch (constellation) {
    case C_GNSS_GPS:
      if (svid >= MIN_GPS_SVID && svid <= MAX_GPS_SVID) {
        idx = svid - 1;
      }
      break;
    case C_GNSS_GLO:
      if (svid >= MIN_GLO_SVID && svid <= MAX_GLO_SVID) {
        idx = N_GPS_SV + svid - 1;
      }
      break;
    case C_GNSS_BDS2:
    case C_GNSS_BDS3:
      if (svid >= MIN_BDS_SVID && svid <= MAX_BDS_SVID) {
        idx = N_GPS_SV + N_GLO_SV + svid - 1;
      }
      break;
    case C_GNSS_GAL:
      if (svid >= MIN_GAL_SVID && svid <= MAX_GAL_SVID) {
        idx = N_GPS_SV + N_GLO_SV + N_BDS_SV + svid - 1;
      }
      break;
    case C_GNSS_QZS:
      if (svid >= MIN_QZS_SVID && svid <= MAX_QZS_SVID) {
        idx = N_GPS_SV + N_GLO_SV + N_BDS_SV + N_GAL_SV + svid - 1;
      }
      break;
    default:
      idx = ALL_GNSS_SYS_SV_NUMBER;
      break;
  }
  return idx;
}

/**
 * @brief sat svid and system from index
 * @param[in] idx
 * @param[out] sys NULL: no output
 * @return sat prn, ALL_GNSS_SYS_SV_NUMBER:error
 */
uint8_t gnss_cvt_SvIndex2Svid(uint8_t idx, uint8_t *sys)
{
  uint8_t svid = ALL_GNSS_SYS_SV_NUMBER;
  uint8_t sys_ = C_GNSS_NONE;
  if (idx < ALL_GNSS_SYS_SV_NUMBER) {
    if (idx < N_GPS_SV) {
      svid = idx + 1;
      sys_ = C_GNSS_GPS;
    }
    else if (idx < N_GPS_SV + N_GLO_SV) {
      svid = idx - N_GPS_SV + 1;
      sys_ = C_GNSS_GLO;
    }
    else if (idx < N_GPS_SV + N_GLO_SV + N_BDS_SV) {
      svid = idx - N_GPS_SV - N_GLO_SV + 1;
      sys_ = C_GNSS_BDS3;
      if (svid <= 18)
      {
        sys_ = C_GNSS_BDS2;
      }
    }
    else if (idx < N_GPS_SV + N_GLO_SV + N_BDS_SV + N_GAL_SV) {
      svid = idx - N_GPS_SV - N_GLO_SV - N_BDS_SV + 1;
      sys_ = C_GNSS_GAL;
    }
    // idx < N_GPS_SV + N_GLO_SV + N_BDS_SV + N_GAL_SV + N_QZS_SV
    else {
      svid = idx - N_GPS_SV - N_GLO_SV - N_BDS_SV - N_GAL_SV + 1;
      sys_ = C_GNSS_QZS;
    }
  }
  if (sys != NULL) {
    *sys = sys_;
  }
  return svid;
}

/**
 * @brief judge the satellite whether belong to BDS3
 * @param[in] svid
 * @param[in] constellation
 * @return FALSE represent not belong to BDS3 and TRUE represent belong to BDS3
 */
BOOL gnss_isBDS3Sat(uint8_t svid, uint8_t constellation)
{
  BOOL q_isBDS3Sat = FALSE;
  if ((C_GNSS_BDS3 == constellation || C_GNSS_BDS2 == constellation) && svid > 18)
  {
    q_isBDS3Sat = TRUE;
  }
  return q_isBDS3Sat;
}
/**
 * @brief signal ID -> index
 * @param[in] signal
 * @return index, MAX_GNSS_SIGNAL_FREQ:error
 */
uint8_t gnss_Signal2FreqIdx(uint8_t signal)
{
  uint8_t index = C_GNSS_FREQ_TYPE_MAX;
  if (signal < C_GNSS_SIG_MAX) 
  {
    index = signal_pos[signal];
  }
  return index;
}

/**
 * @brief signal ID -> 1C,2W...
 * @param[in] e_ag_signal_ch
 * @param[out] c_sig
 * @return none
 */
void gnss_Signal2Char(gnss_SignalType e_ag_signal_ch, char* c_sig)
{
  memcpy(c_sig, " ", sizeof(char));
  if (C_GNSS_SIG_GPS_L1C == e_ag_signal_ch)
  {
    memcpy(c_sig, "1C", sizeof(char)*2);
  }
  else if (C_GNSS_SIG_GPS_L2C == e_ag_signal_ch)
  {
    memcpy(c_sig, "2C", sizeof(char) * 2);
  }
  else if (C_GNSS_SIG_GPS_L5Q == e_ag_signal_ch)
  {
    memcpy(c_sig, "5Q", sizeof(char) * 2);
  }
  else if (C_GNSS_SIG_GLO_G1 == e_ag_signal_ch)
  {
    memcpy(c_sig, "1C", sizeof(char) * 2);
  }
  else if (C_GNSS_SIG_GLO_G2 == e_ag_signal_ch)
  {
    memcpy(c_sig, "2C", sizeof(char) * 2);
  }
  else if (C_GNSS_SIG_BDS_B1I == e_ag_signal_ch)
  {
    memcpy(c_sig, "2I", sizeof(char) * 2);
  }
  else if (C_GNSS_SIG_BDS_B2I == e_ag_signal_ch)
  {
    memcpy(c_sig, "7I", sizeof(char) * 2);
  }
  else if (C_GNSS_SIG_BDS_B3I == e_ag_signal_ch)
  {
    memcpy(c_sig, "6I", sizeof(char) * 2);
  }
  else if (C_GNSS_SIG_BDS_B1C == e_ag_signal_ch)
  {
    memcpy(c_sig, "1X", sizeof(char) * 2);
  }
  else if (C_GNSS_SIG_BDS_B2A == e_ag_signal_ch)
  {
    memcpy(c_sig, "5P", sizeof(char) * 2);
  }
  else if (C_GNSS_SIG_BDS_B2B == e_ag_signal_ch)
  {
    memcpy(c_sig, "7P", sizeof(char) * 2);
  }
  else if (C_GNSS_SIG_GAL_E1 == e_ag_signal_ch)
  {
    memcpy(c_sig, "1C", sizeof(char) * 2);
  }
  else if (C_GNSS_SIG_GAL_E5A == e_ag_signal_ch)
  {
    memcpy(c_sig, "5Q", sizeof(char) * 2);
  }
  else if (C_GNSS_SIG_GAL_E5B == e_ag_signal_ch)
  {
    memcpy(c_sig, "7Q", sizeof(char) * 2);
  }
  else if (C_GNSS_SIG_QZS_L1C == e_ag_signal_ch)
  {
    memcpy(c_sig, "1C", sizeof(char) * 2);
  }
  else if (C_GNSS_SIG_QZS_L2C == e_ag_signal_ch)
  {
    memcpy(c_sig, "2C", sizeof(char) * 2);
  }
  else if (C_GNSS_SIG_QZS_L5Q == e_ag_signal_ch)
  {
    memcpy(c_sig, "5Q", sizeof(char) * 2);
  }
}

/**
 * @brief index -> signal ID
 * @param[in] index
 * @param[in] sys
 * @return signal, 0 ~ C_GNSS_SIG_MAX-1. C_GNSS_SIG_MAX:error
 */
uint8_t gnss_FreqIdx2Signal(uint8_t index, uint8_t sys)
{
  uint8_t signal = C_GNSS_SIG_MAX;
  if (index < MAX_GNSS_SIGNAL_FREQ && sys < C_GNSS_MAX) 
  {
    signal = sys_signal[sys][index];
  }
  return signal;
}

/**
 * @brief satellite string, eg.G02
 * @param[in] svid 
 * @param[in] sys 
 * @param[out] s 
 */
void svid_SatString(uint8_t svid, uint8_t sys, char* s)
{
  if (sys >= C_GNSS_MAX) 
  {
    snprintf(s, 4, "   ");
  }
  else 
  {
    snprintf(s, 4, "%c%02d", system_char_s[sys], svid);
  }
}
/**
 * @brief satellite string, eg.G02
 * @param[in] satidx
 * @param[out] s
 */
void satidx_SatString(uint8_t satidx, char* s)
{
  uint8_t sys = 0;
  uint8_t svid = 0;
  svid = gnss_cvt_SvIndex2Svid(satidx, &sys);
  if (sys == C_GNSS_MAX || sys == C_GNSS_NONE) {
    snprintf(s, 4, "   ");
  }
  else {
    snprintf(s, 4, "%c%02d", system_char_s[sys], svid);
  }
}

/**
 * @brief Refresh measurement collect to remove expired measurement
          Note: the valid time limit of satellite measurement is 0.15s, tot~=80ms
                the valid time limit of satellite measurement is 0.5s,tor interval=1000ms
 * @param[in]   pz_SatSigMeasCollect
 * @return      None
 */
void gnss_SatSigMeasCollect_Refresh(gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect)
{
  GpsTime_t* pz_tor = &(pz_SatSigMeasCollect->z_tor);
  uint8_t u_satMeasCount = 0;
  uint8_t b_ValidSatMeasEnough = FALSE;

  for (uint32_t u_svIdx = 0; u_svIdx < ALL_GNSS_SYS_SV_NUMBER; u_svIdx++)
  {
    gnss_SatelliteMeas_t* pz_SatMeas = pz_SatSigMeasCollect->pz_satMeas[u_svIdx];
    if (NULL == pz_SatMeas)
    {
      continue;
    }

    uint8_t b_hasValidSignalMeas = FALSE;
    for (uint8_t u_FreqId = 0; u_FreqId < MAX_GNSS_SIGNAL_FREQ; u_FreqId++)
    {
      if (NULL == pz_SatMeas->pz_signalMeas[u_FreqId])
      {
        continue;
      }

      GpsTime_t* pz_latest_tor = &(pz_SatMeas->pz_signalMeas[u_FreqId]->z_latest_tor);
      double time_diff2 = tm_GpsTimeDiff(pz_tor, pz_latest_tor);
      if (time_diff2 > FABS_ZEROS)
      {
        //LOGI(TAG_PPP, "MeasCollect Refresh delete old signal field Cons %02d Svid %d Freq %d\n",
        //  pz_SatMeas->pz_signalMeas[u_FreqId]->u_constellation,
        //  pz_SatMeas->pz_signalMeas[u_FreqId]->u_svid,
        //  u_FreqId);
        memset(pz_SatMeas->pz_signalMeas[u_FreqId], 0, sizeof(gnss_SignalMeas_t));
        OS_FREE(pz_SatMeas->pz_signalMeas[u_FreqId]);
        pz_SatMeas->pz_signalMeas[u_FreqId] = NULL;
      }
      else
      {
        b_hasValidSignalMeas = TRUE;
      }
    }

    GpsTime_t* pz_tot = &(pz_SatMeas->z_tot);
    double time_diff = tm_GpsTimeDiff(pz_tor, pz_tot);
    if ((!b_hasValidSignalMeas) || (fabs(time_diff) > 0.15) || (b_ValidSatMeasEnough))
    {
      /* Free all signal measurement memory */
      for (int j = 0; j < MAX_GNSS_SIGNAL_FREQ; j++)
      {
        if (NULL == pz_SatMeas->pz_signalMeas[j])
        {
          continue;
        }
        OS_FREE(pz_SatMeas->pz_signalMeas[j]);
        pz_SatMeas->pz_signalMeas[j] = NULL;
      }
      //LOGI(TAG_PPP, "MeasCollect Refresh delete old whole field Cons %02d Svid %d\n",
      //  pz_SatSigMeasCollect->pz_satMeas[u_svIdx]->u_constellation,
      //  pz_SatSigMeasCollect->pz_satMeas[u_svIdx]->u_svid);
      memset(pz_SatSigMeasCollect->pz_satMeas[u_svIdx], 0, sizeof(gnss_SatelliteMeas_t));
      OS_FREE(pz_SatSigMeasCollect->pz_satMeas[u_svIdx]);
      pz_SatSigMeasCollect->pz_satMeas[u_svIdx] = NULL;
    }
    else
    {
      u_satMeasCount++;
      if (u_satMeasCount >= MAX_GNSS_ACTIVE_SAT_NUMBER)
      {
        b_ValidSatMeasEnough = TRUE;
      }
    }
  }

  pz_SatSigMeasCollect->u_satMeasCount = 0;
  memset(pz_SatSigMeasCollect->u_satMeasIdxTable, 0,
         sizeof(pz_SatSigMeasCollect->u_satMeasIdxTable));
  for (uint32_t idx = 0; idx < ALL_GNSS_SYS_SV_NUMBER; idx++)
  {
    if (NULL != pz_SatSigMeasCollect->pz_satMeas[idx])
    {
      pz_SatSigMeasCollect->u_satMeasIdxTable[pz_SatSigMeasCollect->u_satMeasCount] = idx;
      pz_SatSigMeasCollect->u_satMeasCount++;
    }
  }

  return;
}

/**
 * @brief Clear satellite signal measurement collect.
 * @param[in]   pz_SatSigMeasCollect
 * @return      None
 */
void gnss_SatSigMeasCollect_Clear(gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect)
{
  for (uint32_t sat_idx = 0; sat_idx < ALL_GNSS_SYS_SV_NUMBER; sat_idx++)
  {
    if (NULL == pz_SatSigMeasCollect->pz_satMeas[sat_idx])
    {
      continue;
    }

    gnss_SatelliteMeas_t* pz_SatSigMeas = pz_SatSigMeasCollect->pz_satMeas[sat_idx];
    for (int signal_idx = 0; signal_idx < MAX_GNSS_SIGNAL_FREQ; signal_idx++)
    {
      if (NULL == pz_SatSigMeas->pz_signalMeas[signal_idx])
      {
        continue;
      }
      OS_FREE(pz_SatSigMeas->pz_signalMeas[signal_idx]);
      pz_SatSigMeas->pz_signalMeas[signal_idx] = NULL;
    }
    OS_FREE(pz_SatSigMeasCollect->pz_satMeas[sat_idx]);
    pz_SatSigMeasCollect->pz_satMeas[sat_idx] = NULL;
  }
  memset(pz_SatSigMeasCollect, 0, sizeof(gnss_SatSigMeasCollect_t));
  return;
}

/**
 * @brief Copy satellite signal measurement collect.
 * @param[in]   pz_SatSigMeasCollect_src
 * @param[in]   pz_SatSigMeasCollect_dst
 * @return      None
 */
void gnss_SatSigMeasCollect_Copy(
  gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect_src,
  gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect_dst)
{
  gnss_SatSigMeasCollect_t* pz_src = pz_SatSigMeasCollect_src;
  gnss_SatSigMeasCollect_t* pz_dst = pz_SatSigMeasCollect_dst;

  pz_dst->z_tor = pz_src->z_tor;
  pz_dst->q_clk_jump = pz_src->q_clk_jump;
  pz_dst->z_positionFix = pz_src->z_positionFix;

  for (uint32_t sat_idx = 0; sat_idx < ALL_GNSS_SYS_SV_NUMBER; sat_idx++)
  {
    gnss_SatelliteMeas_t* pz_SatSigMeas_Src = pz_src->pz_satMeas[sat_idx];

    if (NULL == pz_SatSigMeas_Src)
    {
      continue;
    }

    if (NULL == pz_dst->pz_satMeas[sat_idx])
    {
      pz_dst->pz_satMeas[sat_idx] =
        (gnss_SatelliteMeas_t*)OS_MALLOC_FAST(sizeof(gnss_SatelliteMeas_t));
      if (NULL == pz_dst->pz_satMeas[sat_idx])
      {
        continue;
      }
    }

    gnss_SatelliteMeas_t* pz_SatSigMeas_Dst = pz_dst->pz_satMeas[sat_idx];

    pz_SatSigMeas_Dst->z_tot = pz_SatSigMeas_Src->z_tot;
    pz_SatSigMeas_Dst->u_constellation = pz_SatSigMeas_Src->u_constellation;
    pz_SatSigMeas_Dst->u_svid = pz_SatSigMeas_Src->u_svid;
    pz_SatSigMeas_Dst->z_satPosVelClk = pz_SatSigMeas_Src->z_satPosVelClk;
    pz_SatSigMeas_Dst->u_eclips = pz_SatSigMeas_Src->u_eclips;
    memcpy(pz_SatSigMeas_Dst->pz_signalQuality, pz_SatSigMeas_Src->pz_signalQuality, sizeof(pz_SatSigMeas_Src->pz_signalQuality));
    memcpy(pz_SatSigMeas_Dst->f_gf, pz_SatSigMeas_Src->f_gf, sizeof(pz_SatSigMeas_Src->f_gf));
    memcpy(pz_SatSigMeas_Dst->f_mw, pz_SatSigMeas_Src->f_mw, sizeof(pz_SatSigMeas_Src->f_mw));
    memcpy(pz_SatSigMeas_Dst->q_mw_cnt, pz_SatSigMeas_Src->q_mw_cnt, sizeof(pz_SatSigMeas_Src->q_mw_cnt));
    memcpy(pz_SatSigMeas_Dst->f_mw_avg, pz_SatSigMeas_Src->f_mw_avg, sizeof(pz_SatSigMeas_Src->f_mw_avg));
    memcpy(pz_SatSigMeas_Dst->f_mw_std, pz_SatSigMeas_Src->f_mw_std, sizeof(pz_SatSigMeas_Src->f_mw_std));

    for (int signal_idx = 0; signal_idx < MAX_GNSS_SIGNAL_FREQ; signal_idx++)
    {
      gnss_SignalMeas_t* pz_SigMeas_Src = pz_SatSigMeas_Src->pz_signalMeas[signal_idx];
      if (NULL == pz_SigMeas_Src)
      {
        continue;
      }

      if (NULL == pz_SatSigMeas_Dst->pz_signalMeas[signal_idx])
      {
        pz_SatSigMeas_Dst->pz_signalMeas[signal_idx] =
          (gnss_SignalMeas_t*)OS_MALLOC_FAST(sizeof(gnss_SignalMeas_t));
        if (NULL == pz_SatSigMeas_Dst->pz_signalMeas[signal_idx])
        {
          continue;
        }
      }
      memcpy(pz_SatSigMeas_Dst->pz_signalMeas[signal_idx],
        pz_SatSigMeas_Src->pz_signalMeas[signal_idx], sizeof(gnss_SignalMeas_t));
    }
  }

  /* Refresh measurement collect to remove expired measurement */
  gnss_SatSigMeasCollect_Refresh(pz_SatSigMeasCollect_dst);

  return;
}

/**
 * @brief Pack a spare satellite signal measurement collect structure as a
          tight satellite signal measurement collect structure.
          The tight structure is used to transmit by IPC
 * @param[in] pz_satSigMeasCollect
 * @param[in] pz_pvt_result
 * @param[out] pz_tightSatSigMeasCollect
 */
void gnss_SatSigMeasCollect_Pack(const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect, 
                            const gnss_PVTResult_t* pz_pvt_result,
                            gnss_TightSatSigMeasCollect_t* pz_tightSatSigMeasCollect)
{
  pz_tightSatSigMeasCollect->z_tor = pz_satSigMeasCollect->z_tor;
  pz_tightSatSigMeasCollect->q_clk_jump = pz_satSigMeasCollect->q_clk_jump;
  pz_tightSatSigMeasCollect->z_positionFix = pz_satSigMeasCollect->z_positionFix;
  pz_tightSatSigMeasCollect->d_ZTD = pz_satSigMeasCollect->d_ZTD;
  pz_tightSatSigMeasCollect->d_zhd_emp = pz_satSigMeasCollect->d_zhd_emp;
  pz_tightSatSigMeasCollect->d_zwd_emp = pz_satSigMeasCollect->d_zwd_emp;

  for (uint32_t sat_idx = 0; sat_idx < ALL_GNSS_SYS_SV_NUMBER; sat_idx++)
  {
    gnss_SatelliteMeas_t* pz_satMeas = pz_satSigMeasCollect->pz_satMeas[sat_idx];
    if (NULL == pz_satMeas)
    {
      continue;
    }

    if (pz_tightSatSigMeasCollect->u_satMeasCount > MAX_GNSS_ACTIVE_SAT_NUMBER)
    {
      break;
    }

    gnss_TightSatelliteMeas_t* pz_tightSatMeas =
      &pz_tightSatSigMeasCollect->pz_tightSatMeas[pz_tightSatSigMeasCollect->u_satMeasCount++];
    pz_tightSatMeas->z_tot = pz_satMeas->z_tot;
    pz_tightSatMeas->u_constellation = pz_satMeas->u_constellation;
    pz_tightSatMeas->u_svid = pz_satMeas->u_svid;
    pz_tightSatMeas->z_satPosVelClk = pz_satMeas->z_satPosVelClk;
    pz_tightSatMeas->u_eclips = pz_satMeas->u_eclips;
    pz_tightSatMeas->d_wetMap = pz_satMeas->d_wetMap;
    pz_tightSatMeas->d_dryMap = pz_satMeas->d_dryMap;
    pz_tightSatMeas->d_Iono = pz_satMeas->d_Iono;
    pz_tightSatMeas->d_Tropo = pz_satMeas->d_Tropo;
    memcpy(pz_tightSatMeas->f_gf, pz_satMeas->f_gf, sizeof(pz_satMeas->f_gf));
    memcpy(pz_tightSatMeas->f_mw, pz_satMeas->f_mw, sizeof(pz_satMeas->f_mw));


    for (uint32_t freq_idx = 0; freq_idx < MAX_GNSS_SIGNAL_FREQ; freq_idx++)
    {
      gnss_SignalMeas_t* pz_signalMeas = pz_satMeas->pz_signalMeas[freq_idx];
      gnss_SignalMeas_t* pz_TightSignalMeas = &pz_tightSatMeas->pz_signalMeas[freq_idx];
      if (pz_signalMeas != NULL)
      {
        memcpy(pz_TightSignalMeas, pz_signalMeas, sizeof(gnss_SignalMeas_t));
      }
      else
      {
        memset(pz_TightSignalMeas, 0, sizeof(gnss_SignalMeas_t));
      }
      for (uint32_t i = 0; i < pz_pvt_result->u_meas_count; ++i)
      {
        if((pz_pvt_result->pz_meas_qos[i].u_constellation == pz_tightSatMeas->u_constellation) &&
           (pz_pvt_result->pz_meas_qos[i].u_svid == pz_tightSatMeas->u_svid) &&
           (pz_pvt_result->pz_meas_qos[i].u_freq_idx == freq_idx))
        {
          memcpy(&pz_tightSatMeas->pz_measQos[freq_idx], &pz_pvt_result->pz_meas_qos[i], sizeof(gnss_MeasQos_t));
          break;
        }
      }
    }
  }

  return;
}

/**
 * @brief Uppack a tight satellite signal measurement collect structure as 
          a spare satellite signal measurement collect structure
          The tight structure is used to transmit by IPC
 * @param[in]   pz_tightSatSigMeasCollect
 * @param[out]  pz_SatSigMeasCollect
 * @return      None
 */
void gnss_SatSigMeasCollect_Unpack(
        const gnss_TightSatSigMeasCollect_t* pz_tightSatSigMeasCollect,
        gnss_SatSigMeasCollect_t* pz_satSigMeasCollect)
{
  pz_satSigMeasCollect->z_positionFix = pz_tightSatSigMeasCollect->z_positionFix;
  pz_satSigMeasCollect->z_tor = pz_tightSatSigMeasCollect->z_tor;
  pz_satSigMeasCollect->q_clk_jump = pz_tightSatSigMeasCollect->q_clk_jump;
  pz_satSigMeasCollect->d_ZTD = pz_tightSatSigMeasCollect->d_ZTD;
  pz_satSigMeasCollect->d_zhd_emp = pz_tightSatSigMeasCollect->d_zhd_emp;
  pz_satSigMeasCollect->d_zwd_emp = pz_tightSatSigMeasCollect->d_zwd_emp;
  pz_satSigMeasCollect->f_earthTide[0] = 0.0f;
  pz_satSigMeasCollect->f_earthTide[1] = 0.0f;
  pz_satSigMeasCollect->f_earthTide[2] = 0.0f;

  for (uint32_t i = 0; i < pz_tightSatSigMeasCollect->u_satMeasCount; i++)
  {
    const gnss_TightSatelliteMeas_t* pz_tightSatMeas =
      &pz_tightSatSigMeasCollect->pz_tightSatMeas[i];
    gnss_ConstellationType u_constellation = pz_tightSatMeas->u_constellation;
    uint8_t u_svid = pz_tightSatMeas->u_svid;
    uint32_t sat_idx = gnss_cvt_Svid2SvIndex(u_svid, u_constellation);

    if (sat_idx == ALL_GNSS_SYS_SV_NUMBER)
    {
      continue;
    }

    if (NULL == pz_satSigMeasCollect->pz_satMeas[sat_idx])
    {
      pz_satSigMeasCollect->pz_satMeas[sat_idx] =
        (gnss_SatelliteMeas_t*)OS_MALLOC_FAST(sizeof(gnss_SatelliteMeas_t));
      if (NULL == pz_satSigMeasCollect->pz_satMeas[sat_idx])
      {
        continue;
      }
    }

    gnss_SatelliteMeas_t* pz_SatSignalMeas = pz_satSigMeasCollect->pz_satMeas[sat_idx];
    memset(pz_SatSignalMeas->pz_signalQuality, 0, sizeof(pz_SatSignalMeas->pz_signalQuality));
    pz_SatSignalMeas->z_tot = pz_tightSatMeas->z_tot;
    pz_SatSignalMeas->u_constellation = pz_tightSatMeas->u_constellation;
    pz_SatSignalMeas->u_svid = pz_tightSatMeas->u_svid;
    pz_SatSignalMeas->z_satPosVelClk = pz_tightSatMeas->z_satPosVelClk;
    pz_SatSignalMeas->u_eclips = pz_tightSatMeas->u_eclips;
    pz_SatSignalMeas->d_wetMap = pz_tightSatMeas->d_wetMap;
    pz_SatSignalMeas->d_dryMap = pz_tightSatMeas->d_dryMap;
    pz_SatSignalMeas->d_Iono = pz_tightSatMeas->d_Iono;
    pz_SatSignalMeas->d_Tropo = pz_tightSatMeas->d_Tropo;
    pz_SatSignalMeas->d_Gravitation = 0.0;
    memcpy(pz_SatSignalMeas->f_gf, pz_tightSatMeas->f_gf, sizeof(pz_tightSatMeas->f_gf));
    memcpy(pz_SatSignalMeas->f_mw, pz_tightSatMeas->f_mw, sizeof(pz_tightSatMeas->f_mw));

    for (uint32_t freq_idx = 0; freq_idx < MAX_GNSS_SIGNAL_FREQ; freq_idx++)
    {
      if (!pz_tightSatMeas->pz_signalMeas[freq_idx].z_measStatusFlag.b_valid)
      {
        continue;
      }

      if (NULL == pz_SatSignalMeas->pz_signalMeas[freq_idx])
      {
        pz_SatSignalMeas->pz_signalMeas[freq_idx] =
          (gnss_SignalMeas_t*)OS_MALLOC_FAST(sizeof(gnss_SignalMeas_t));
        if (NULL == pz_SatSignalMeas->pz_signalMeas[freq_idx])
        {
          continue;
        }
        LOGI(TAG_PPP, "MeasCollect unpack alloc new feild Cons %02d Svid %d Freq %d\n",
          pz_tightSatMeas->u_constellation, pz_tightSatMeas->u_svid, freq_idx);
      }
      memcpy(pz_SatSignalMeas->pz_signalMeas[freq_idx],
        &pz_tightSatMeas->pz_signalMeas[freq_idx], sizeof(gnss_SignalMeas_t));
      pz_SatSignalMeas->pz_signalMeas[freq_idx]->u_slipMethodValid = 0;
      if (0x1 != (pz_tightSatMeas->pz_measQos[freq_idx].u_status & 0x1))
      {
        pz_SatSignalMeas->pz_signalMeas[freq_idx]->z_measStatusFlag.b_prValid = 0;
      }
      else
      {
        pz_SatSignalMeas->pz_signalMeas[freq_idx]->z_measStatusFlag.b_prValid = 1;
      }

      if (0x2 != (pz_tightSatMeas->pz_measQos[freq_idx].u_status & 0x2))
      {
        pz_SatSignalMeas->pz_signalMeas[freq_idx]->z_measStatusFlag.b_drValid = 0;
      }
      else
      {
        pz_SatSignalMeas->pz_signalMeas[freq_idx]->z_measStatusFlag.b_drValid = 1;
      }

      if (0x8 != (pz_tightSatMeas->pz_measQos[freq_idx].u_status & 0x8))
      {
        pz_SatSignalMeas->pz_signalMeas[freq_idx]->z_measStatusFlag.b_cpValid = 0;
      }
      else
      {
        pz_SatSignalMeas->pz_signalMeas[freq_idx]->z_measStatusFlag.b_cpValid = 1;
      }
      pz_SatSignalMeas->pz_signalMeas[freq_idx]->d_pr_unc = pz_tightSatMeas->pz_measQos[freq_idx].d_pr_unc;
      pz_SatSignalMeas->pz_signalMeas[freq_idx]->d_dr_unc = pz_tightSatMeas->pz_measQos[freq_idx].d_dr_unc;
    }
  }

  /* Refresh measurement collect to remove expired measurement */
  gnss_SatSigMeasCollect_Refresh(pz_satSigMeasCollect);

  return;
}

/**
 * @brief make tdcp gnss measurement
 * @param[in] pz_SatSigMeasCollect satellite signal measurement
 * @param[out] pz_tdcpMeas TDCP measurement
 */
static void
gnss_MakeTdcpSatInfo(const gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect, gnss_TdcpMeasBlock_t* pz_tdcpMeas)
{
  double pd_satPosRot[3] = {0.0};
  double pd_unitVector[3] = {0.0};
  uint8_t sat_num = 0;
  const double* d_xyz =  pz_tdcpMeas->z_posSol.d_xyz;

  if (gnss_Norm(d_xyz, 3) < FABS_ZEROS)
  {
    return;
  }

  for (int i = 0; i < ALL_GNSS_SYS_SV_NUMBER; ++i)
  {
    if (NULL == pz_SatSigMeasCollect->pz_satMeas[i])
    {
      continue;
    }
    if (sat_num >= MAX_GNSS_ACTIVE_SAT_NUMBER)
    {
      break;
    }
    gnss_DcpSatInfo_t* pz_satInfo = (pz_tdcpMeas->pz_satInfo) + sat_num;
    memset(pd_satPosRot, 0, 3 * sizeof(double));
    pz_satInfo->u_valid = FALSE;
    pz_satInfo->u_constellation = pz_SatSigMeasCollect->pz_satMeas[i]->u_constellation;
    pz_satInfo->u_svid = pz_SatSigMeasCollect->pz_satMeas[i]->u_svid;

    pz_satInfo->u_isCalculateDist = FALSE;
    if ((pz_SatSigMeasCollect->pz_satMeas[i]->z_satPosVelClk.q_iode) >= 0 &&
        (pz_SatSigMeasCollect->pz_satMeas[i]->z_satPosVelClk.f_elevation > FABS_ZEROS) &&
        (pz_SatSigMeasCollect->pz_satMeas[i]->z_satPosVelClk.f_azimuth >= FABS_ZEROS))
    {
      pz_satInfo->u_valid = TRUE;
      pz_satInfo->u_isCalculateDist = TRUE;
      pz_satInfo->f_elevation = pz_SatSigMeasCollect->pz_satMeas[i]->z_satPosVelClk.f_elevation * (float)RAD2DEG;
      pz_satInfo->f_azimuth = pz_SatSigMeasCollect->pz_satMeas[i]->z_satPosVelClk.f_azimuth * (float)RAD2DEG;
      pz_satInfo->q_iode = pz_SatSigMeasCollect->pz_satMeas[i]->z_satPosVelClk.q_iode ;
      pz_satInfo->pd_satPosClk[0] = pz_SatSigMeasCollect->pz_satMeas[i]->z_satPosVelClk.d_satPosClk[0];
      pz_satInfo->pd_satPosClk[1] = pz_SatSigMeasCollect->pz_satMeas[i]->z_satPosVelClk.d_satPosClk[1];
      pz_satInfo->pd_satPosClk[2] = pz_SatSigMeasCollect->pz_satMeas[i]->z_satPosVelClk.d_satPosClk[2];
      pz_satInfo->pd_satPosClk[3] = pz_SatSigMeasCollect->pz_satMeas[i]->z_satPosVelClk.d_satPosClk[3];
      pz_satInfo->pd_satVelClk[0] = pz_SatSigMeasCollect->pz_satMeas[i]->z_satPosVelClk.d_satVelClk[0];
      pz_satInfo->pd_satVelClk[1] = pz_SatSigMeasCollect->pz_satMeas[i]->z_satPosVelClk.d_satVelClk[1];
      pz_satInfo->pd_satVelClk[2] = pz_SatSigMeasCollect->pz_satMeas[i]->z_satPosVelClk.d_satVelClk[2];
      pz_satInfo->pd_satVelClk[3] = pz_SatSigMeasCollect->pz_satMeas[i]->z_satPosVelClk.d_satVelClk[3];
      pz_satInfo->d_sat2SiteDist = gnss_unitVector(pz_SatSigMeasCollect->z_positionFix.d_xyz, pz_satInfo->pd_satPosClk,
                                                   pd_unitVector);
      pz_satInfo->pd_site2SatUnit[0] = -pd_unitVector[0];
      pz_satInfo->pd_site2SatUnit[1] = -pd_unitVector[1];
      pz_satInfo->pd_site2SatUnit[2] = -pd_unitVector[2];
      sat_num++;
    }
  }
}

/**
 * @brief make tdcp gnss measurement
 * @param[in] pz_SatSigMeasCollect satellite signal measurement
 * @param[out] pz_tdcpMeas TDCP measurement
 */
static void gnss_MakeTdcpGnssMeas(const gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect, gnss_TdcpMeasBlock_t* pz_tdcpMeas)
{
  uint8_t meas_num = 0; // meas num

  for (int i = 0; i < ALL_GNSS_SYS_SV_NUMBER; ++i)
  {
    if (NULL == pz_SatSigMeasCollect->pz_satMeas[i])
    {
      continue;
    }

    for (int j = 0; j < MAX_GNSS_SIGNAL_FREQ; ++j)
    {
      if (NULL == pz_SatSigMeasCollect->pz_satMeas[i]->pz_signalMeas[j])
      {
        continue;
      }
      if (meas_num >= MAX_GNSS_TRK_MEAS_NUMBER)
      {
        break;
      }
      GnssMeas_t* pz_gnss_meas = &pz_tdcpMeas->pz_meas[meas_num];
      gnss_SignalMeas_t* pz_sigmeas = pz_SatSigMeasCollect->pz_satMeas[i]->pz_signalMeas[j];

      pz_gnss_meas->z_measStatusFlag = pz_sigmeas->z_measStatusFlag;
      pz_gnss_meas->u_constellation = pz_sigmeas->u_constellation;
      pz_gnss_meas->u_svid = pz_sigmeas->u_svid;
      pz_gnss_meas->u_signal = pz_sigmeas->u_signal;
      pz_gnss_meas->u_LLI = pz_sigmeas->u_LLI;
      pz_gnss_meas->d_carrierPhase = pz_sigmeas->d_carrierPhase;
      pz_gnss_meas->d_pseudoRange = pz_sigmeas->d_pseudoRange;
      pz_gnss_meas->d_doppler = pz_sigmeas->d_doppler;
      pz_gnss_meas->f_cn0 = pz_sigmeas->f_cn0;
      ++meas_num;
    }
  }
  pz_tdcpMeas->w_measNum = meas_num;
}

/**
 * @brief convert satellite signal measurement collect to TDCP measurement
 * @param[in] pz_SatSigMeasCollect satellite signal measurement
 * @param[in] pz_PositionFix assign specific position solution
 * @param[out] pz_tdcpMeas TDCP measurement
 */
void
gnss_cvt_SatSigMeas2TdcpMeas(const gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect,
                             const gnss_PositionFix_t* pz_PositionFix, gnss_TdcpMeasBlock_t* pz_tdcpMeas)
{
  pz_tdcpMeas->u_version = VERSION_TDCP_MEAS_BLOCK;
  pz_tdcpMeas->w_size = sizeof(gnss_TdcpMeasBlock_t);
  pz_tdcpMeas->z_obsTime = pz_SatSigMeasCollect->z_tor;
  pz_tdcpMeas->z_posSol = (NULL == pz_PositionFix) ? pz_SatSigMeasCollect->z_positionFix : *pz_PositionFix;
  pz_tdcpMeas->w_satNum = pz_SatSigMeasCollect->u_satMeasCount;
  pz_tdcpMeas->pz_measExt->u_multiPath = 0;

  gnss_MakeTdcpGnssMeas(pz_SatSigMeasCollect, pz_tdcpMeas);
  gnss_MakeTdcpSatInfo(pz_SatSigMeasCollect, pz_tdcpMeas);
}
/**
 * @brief calculate time of transmit for meas
 * @param[in,out] pz_SatSigMeasCollect
 */
void gnss_FillSatSigMeasTot(gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect)
{
  for (int i = 0; i < ALL_GNSS_SYS_SV_NUMBER; ++i)
  {
    if (NULL == pz_SatSigMeasCollect->pz_satMeas[i])
    {
      continue;
    }
    GpsTime_t tot = pz_SatSigMeasCollect->z_tor;
    if (gnss_CalMeasTot(pz_SatSigMeasCollect->pz_satMeas[i], &tot))
    {
      pz_SatSigMeasCollect->pz_satMeas[i]->z_tot = tot;
    }
    else
    {
      memset(&pz_SatSigMeasCollect->pz_satMeas[i]->z_tot, 0, sizeof(GpsTime_t));
    }
  }
}

/**
 * @brief Append time-adjusted measurement into satellite signal measurement collect.
          If the satellite measure pointer was used, just update without allocating new memory.
          If the satellite measure pointer is NULL, allocate new memory.
          After appending new measurement, the collect data maybe contain old measurement.
          If don't use them, it need to delete the old measurement and free their memory
 * @param[in]   pz_MeasBlock
 * @param[in]   pz_SatSigMeasCollect
 * @return      None
 */
void gnss_AppendMeasBlkToSatSigMeasCollect(GnssMeasBlock_t* pz_MeasBlock, gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect)
{
  uint8_t u_i = 0;
  if (any_Ptrs_Null(2, pz_MeasBlock, pz_SatSigMeasCollect))
  {
    return;
  }

  pz_SatSigMeasCollect->u_satMeasCount = 0;
  memset(pz_SatSigMeasCollect->u_satMeasIdxTable, 0,
    sizeof(pz_SatSigMeasCollect->u_satMeasIdxTable));

  pz_SatSigMeasCollect->z_tor = pz_MeasBlock->z_Clock.z_gpsTime;

  for (uint32_t idx = 0; idx < pz_MeasBlock->w_measCount; idx++)
  {
    GnssMeas_t* pz_meas = &(pz_MeasBlock->z_meas[idx]);
    gnss_ConstellationType u_constellation = pz_meas->u_constellation;
    uint8_t u_svid = pz_meas->u_svid;
    uint8_t u_signal = pz_meas->u_signal;
    uint32_t u_svIdx = gnss_cvt_Svid2SvIndex(u_svid, u_constellation);
    gnss_FreqType u_freq_type = gnss_cvt_Sig2FreqType(u_signal);

    if ((u_svIdx == ALL_GNSS_SYS_SV_NUMBER) || (u_freq_type == C_GNSS_FREQ_TYPE_MAX))
    {
      continue;
    }
    else if (C_GNSS_GLO == pz_meas->u_constellation)
    {
      continue;
    }

    /* Alloc a new memory for empty SV */
    if (NULL == pz_SatSigMeasCollect->pz_satMeas[u_svIdx])
    {
      pz_SatSigMeasCollect->pz_satMeas[u_svIdx] =
        (gnss_SatelliteMeas_t*)OS_MALLOC_FAST(sizeof(gnss_SatelliteMeas_t));
      if (NULL == pz_SatSigMeasCollect->pz_satMeas[u_svIdx])
      {
        continue;
      }
    }
    gnss_SatelliteMeas_t* pz_SatSignalMeas = pz_SatSigMeasCollect->pz_satMeas[u_svIdx];

    if (NULL == pz_SatSignalMeas)
    {
      continue;
    }
    for (u_i = 0; u_i < MAX_GNSS_SIGNAL_FREQ; ++u_i)
    {
      pz_SatSignalMeas->pz_signalQuality[u_i].u_CPquality = CP_QUALITY_NORM;
      pz_SatSignalMeas->pz_signalQuality[u_i].u_PRquality = PR_QUALITY_NORM;
    }
    pz_SatSignalMeas->u_constellation = u_constellation;
    pz_SatSignalMeas->u_svid = u_svid;
    pz_SatSignalMeas->z_satPosVelClk.q_iode = -1;

    if (NULL == pz_SatSignalMeas->pz_signalMeas[u_freq_type])
    {
      pz_SatSignalMeas->pz_signalMeas[u_freq_type] =
        (gnss_SignalMeas_t*)OS_MALLOC_FAST(sizeof(gnss_SignalMeas_t));
      if (NULL == pz_SatSignalMeas->pz_signalMeas[u_freq_type])
      {
        continue;
      }
    }

    gnss_SignalMeas_t* pz_signalMeas = pz_SatSignalMeas->pz_signalMeas[u_freq_type];
    if (NULL == pz_signalMeas)
    {
      continue;
    }

    memset(pz_signalMeas, 0, sizeof(gnss_SignalMeas_t));
    pz_signalMeas->z_latest_tor = pz_SatSigMeasCollect->z_tor;
    pz_signalMeas->z_measStatusFlag = pz_meas->z_measStatusFlag;
    pz_signalMeas->u_constellation = u_constellation;
    pz_signalMeas->u_svid = u_svid;
    pz_signalMeas->u_signal = u_signal;
    pz_signalMeas->u_LLI = pz_meas->u_LLI;
    pz_signalMeas->f_cn0 = pz_meas->f_cn0;
    pz_signalMeas->d_pseudoRange = pz_meas->d_pseudoRange;
    pz_signalMeas->d_doppler = pz_meas->d_doppler;
    pz_signalMeas->d_carrierPhase = pz_meas->d_carrierPhase;
  }

  /* calculate time of transmit for meas */
  gnss_FillSatSigMeasTot(pz_SatSigMeasCollect);

  /* remove expired measurement */
  gnss_SatSigMeasCollect_Refresh(pz_SatSigMeasCollect);
}

/**
 * @brief Append time-adjusted correction data struct into satellite signal measurement collect.
          If the satellite measure pointer was used, just update without allocating new memory.
          If the satellite measure pointer is NULL, allocate new memory.
          After appending new measurement, the collect data maybe contain old measurement.
          If don't use them, it need to delete the old measurement and free their memory
 * @param[in]   pz_corrBlock
 * @param[in]   pz_SatSigMeasCollect
 * @return      None
 */
void gnss_AppendCorrBlkToSatSigMeasCollect(const GnssCorrBlock_t* pz_corrBlock, gnss_SatSigMeasCollect_t* pz_SatSigMeasCollect)
{
  uint8_t u_i = 0;
  uint32_t q_index = 0;
  const GnssMeas_t* pz_meas = NULL;
  gnss_SatelliteMeas_t* pz_SatSignalMeas = NULL;
  gnss_SignalMeas_t* pz_signalMeas = NULL;
  const gnss_SatPosClkInfo_t* pz_satPosClkInfo = NULL;
  gnss_ConstellationType u_constellation = 0;
  uint8_t u_svid = 0;
  uint8_t u_signal = 0;
  uint32_t u_svIdx = 0;
  gnss_FreqType u_freq_type= C_GNSS_FREQ_TYPE_L1;
  if (any_Ptrs_Null(2, pz_corrBlock, pz_SatSigMeasCollect))
  {
    return;
  }

  pz_SatSigMeasCollect->u_satMeasCount = 0;
  memset(pz_SatSigMeasCollect->u_satMeasIdxTable, 0, sizeof(pz_SatSigMeasCollect->u_satMeasIdxTable));
  pz_SatSigMeasCollect->z_tor = pz_corrBlock->z_Clock.z_gpsTime;
  //fillup observations
  for (q_index = 0; q_index < pz_corrBlock->w_measCount; ++q_index)
  {
    pz_meas = &(pz_corrBlock->z_meas[q_index]);
    u_constellation = pz_meas->u_constellation;
    u_svid = pz_meas->u_svid;
    u_signal = pz_meas->u_signal;
    u_svIdx = gnss_cvt_Svid2SvIndex(u_svid, u_constellation);
    u_freq_type = gnss_cvt_Sig2FreqType(u_signal);
    if ((ALL_GNSS_SYS_SV_NUMBER == u_svIdx) || (C_GNSS_FREQ_TYPE_MAX == u_freq_type))
    {
      continue;
    }
    /* Alloc a new memory for empty SV */
    if (NULL == pz_SatSigMeasCollect->pz_satMeas[u_svIdx])
    {
      pz_SatSigMeasCollect->pz_satMeas[u_svIdx] = (gnss_SatelliteMeas_t*)OS_MALLOC_FAST(sizeof(gnss_SatelliteMeas_t));
      if (NULL == pz_SatSigMeasCollect->pz_satMeas[u_svIdx])
      {
        continue;
      }
    }
    pz_SatSignalMeas = pz_SatSigMeasCollect->pz_satMeas[u_svIdx];

    if (NULL == pz_SatSignalMeas)
    {
      continue;
    }
    for (u_i = 0; u_i < MAX_GNSS_SIGNAL_FREQ; ++u_i)
    {
      pz_SatSignalMeas->pz_signalQuality[u_i].u_CPquality = CP_QUALITY_NORM;
      pz_SatSignalMeas->pz_signalQuality[u_i].u_PRquality = PR_QUALITY_NORM;
    }
    pz_SatSignalMeas->u_constellation = u_constellation;
    pz_SatSignalMeas->u_svid = u_svid;
    pz_SatSignalMeas->z_satPosVelClk.q_iode = -1;

    if (NULL == pz_SatSignalMeas->pz_signalMeas[u_freq_type])
    {
      pz_SatSignalMeas->pz_signalMeas[u_freq_type] = (gnss_SignalMeas_t*)OS_MALLOC_FAST(sizeof(gnss_SignalMeas_t));
      if (NULL == pz_SatSignalMeas->pz_signalMeas[u_freq_type])
      {
        continue;
      }
    }

    pz_signalMeas = pz_SatSignalMeas->pz_signalMeas[u_freq_type];
    if (NULL == pz_signalMeas)
    {
      continue;
    }

    memset(pz_signalMeas, 0, sizeof(gnss_SignalMeas_t));
    pz_signalMeas->z_latest_tor = pz_SatSigMeasCollect->z_tor;
    pz_signalMeas->z_measStatusFlag = pz_meas->z_measStatusFlag;
    pz_signalMeas->u_constellation = u_constellation;
    pz_signalMeas->u_svid = u_svid;
    pz_signalMeas->u_signal = u_signal;
    pz_signalMeas->u_LLI = pz_meas->u_LLI;
    pz_signalMeas->f_cn0 = pz_meas->f_cn0;
    pz_signalMeas->d_pseudoRange = pz_meas->d_pseudoRange;
    pz_signalMeas->d_doppler = pz_meas->d_doppler;
    pz_signalMeas->d_carrierPhase = pz_meas->d_carrierPhase;
  }

  //fillup satellite information
  for (q_index = 0; q_index < (pz_corrBlock->w_satCount); ++q_index)
  {
    pz_satPosClkInfo = &(pz_corrBlock->pz_satPosClk[q_index]);
    u_constellation = pz_satPosClkInfo->u_constellation;
    u_svid = pz_satPosClkInfo->u_svid;
    u_svIdx = gnss_cvt_Svid2SvIndex(u_svid, u_constellation);
    if ((ALL_GNSS_SYS_SV_NUMBER == u_svIdx))
    {
      continue;
    }
    if (NULL == pz_SatSigMeasCollect->pz_satMeas[u_svIdx])
    {
      continue;
    }
    pz_SatSigMeasCollect->pz_satMeas[u_svIdx]->z_satPosVelClk.u_constellation = pz_satPosClkInfo->u_constellation;
    pz_SatSigMeasCollect->pz_satMeas[u_svIdx]->z_satPosVelClk.u_svid = pz_satPosClkInfo->u_svid;
    pz_SatSigMeasCollect->pz_satMeas[u_svIdx]->z_satPosVelClk.q_iode = pz_satPosClkInfo->q_iode;
    for (u_i = 0; u_i < 4; ++u_i)
    {
      pz_SatSigMeasCollect->pz_satMeas[u_svIdx]->z_satPosVelClk.d_satPosClk[u_i] = pz_satPosClkInfo->d_satPosClk[u_i];
    }
  }

  /* calculate time of transmit for meas */
  gnss_FillSatSigMeasTot(pz_SatSigMeasCollect);

  /* remove expired measurement */
  gnss_SatSigMeasCollect_Refresh(pz_SatSigMeasCollect);
  return;
}

/**
 * @brief convert receiver measurement block structure to TDCP measurement
 * @param[int] pz_MeasBlock
 * @param[out] pz_tdcpMeas
 */
void gnss_cvt_Meas2TdcpMeas(const GnssMeasBlock_t* pz_MeasBlock, gnss_TdcpMeasBlock_t* pz_tdcpMeas)
{
  pz_tdcpMeas->u_version = pz_MeasBlock->u_version;
  pz_tdcpMeas->w_size = sizeof(gnss_TdcpMeasBlock_t);
  pz_tdcpMeas->z_obsTime = pz_MeasBlock->z_Clock.z_gpsTime;
  pz_tdcpMeas->w_satNum = 0;
  memset(pz_tdcpMeas->pz_satInfo, 0, sizeof(gnss_DcpSatInfo_t) * MAX_GNSS_ACTIVE_SAT_NUMBER);
  pz_tdcpMeas->w_measNum = pz_MeasBlock->w_measCount;
  for (uint16_t u_i = 0; u_i < (pz_MeasBlock->w_measCount); ++u_i)
  {
    if (u_i >= MAX_GNSS_TRK_MEAS_NUMBER)
    {
      break;
    }
    pz_tdcpMeas->pz_meas[u_i] = pz_MeasBlock->z_meas[u_i];
    pz_tdcpMeas->pz_measExt[u_i] = pz_MeasBlock->z_measExt[u_i];
  }
}

/**
 * @brief Compute Satellite Postion Velocity and Clock from SD polynomial.
          Store the satellite information into the z_satPosVelClk of measurement.
 * @param[in]   pz_SatSigMeasCollect
 * @return      None
 */
void gnss_ComputeSatPosVelClk(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect)
{
  if (NULL == pz_satSigMeasCollect)
  {
    return;
  }

  BOOL b_success = FALSE;
  sd_SatPosVelClkRequest_t z_SatPvtReq = { 0 };
  z_SatPvtReq.z_ReqTime = pz_satSigMeasCollect->z_tor;
  tm_GpstTimeAdd(&z_SatPvtReq.z_ReqTime, SAT_POLYNOMIAL_REQ_ADVANCED_TIME);

  for (uint32_t sat_idx = 0; sat_idx < ALL_GNSS_SYS_SV_NUMBER; sat_idx++)
  {
    gnss_SatelliteMeas_t* pz_satMeas = pz_satSigMeasCollect->pz_satMeas[sat_idx];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    if (C_GNSS_GLO == pz_satMeas->u_constellation)
    {
      continue;
    }

    b_success = sd_api_SatPosVelClk_Get(pz_satMeas->u_constellation, pz_satMeas->u_svid,
      &pz_satMeas->z_tot, &(pz_satMeas->z_satPosVelClk));
    if ((FALSE == b_success) || (pz_satMeas->z_satPosVelClk.f_dt > SAT_POLYNOMIAL_REQ_TIME))
    {
      M_SET_BIT(z_SatPvtReq.t_gnssReqMask[pz_satMeas->u_constellation], pz_satMeas->u_svid);
      continue;
    }
  }

  LOGI(TAG_SD, "Polynominal request %d\n", pz_satSigMeasCollect->z_tor.q_towMsec);
  sd_api_SatPosVelClkPolynomial_Request(&z_SatPvtReq);

  return;
}

/**
 * @brief Gnss Measurement quality compare function
 * @param[in]   ma
 * @param[in]   mb
 * @return      None
 */
static BOOL gnss_MeasureQualityCompare(void* ma, void* mb)
{
  if (((GnssMeas_t*)ma)->f_cn0 > ((GnssMeas_t*)mb)->f_cn0)
  {
    return TRUE;
  }
  return FALSE;
}

/**
 * @brief Raw receiver measurement don't have Week number, it needs to
          get week from ephemeris and create a valid receiver time from
          measurement tow and ephemeris week.
 * @param [in]w_measCount meas count of pz_meas
 * @param [in]pz_meas Measurement
 * @param [in/out]pz_gpsTime GPS time
 * @return TRUE:success  FALSE:failed
 */
BOOL gnss_AdjustMeasBlockWeek(uint16_t w_measCount, const GnssMeas_t* pz_meas, GpsTime_t* pz_gpsTime)
{
  GnssMeas_t z_GnssMeas[MAX_GNSS_TRK_MEAS_NUMBER] = {0};
  uint16_t w_BestWeek = 0;
  uint16_t w_2ndBestWeek = 0;
  double d_BestTow = pz_gpsTime->q_towMsec * 1e-3 + pz_gpsTime->q_subNsec * 1e-9;

  memcpy(&z_GnssMeas[0], pz_meas, MAX_GNSS_TRK_MEAS_NUMBER * sizeof(GnssMeas_t));
  loc_sort((uint8_t*) z_GnssMeas, sizeof(GnssMeas_t), MAX_GNSS_TRK_MEAS_NUMBER, gnss_MeasureQualityCompare);

  /* Find out the best quality measurement's Week Number */
  for (int i = 0; i < w_measCount; i++)
  {
    uint8_t constellation = z_GnssMeas[i].u_constellation;
    uint8_t svid = z_GnssMeas[i].u_svid;
    gnss_Ephemeris_t eph = {0};

    if (TRUE == sd_api_GnssEphemeris_Get(NULL, constellation, svid, -1, &eph))
    {
      double time_diff = 0.001 * ((double) pz_gpsTime->q_towMsec - (double) eph.z_toeOfGpst.q_towMsec);
      if ((time_diff > 0) && (time_diff < (double) WEEK_SEC / 2))
      {
        w_BestWeek = eph.z_toeOfGpst.w_week;
        break;
      }
    }
  }

  if (0 == w_BestWeek)
  {
    /* Find out the second best quality measurement's Week Number */
    for (int i = 0; i < w_measCount; i++)
    {
      uint8_t constellation = z_GnssMeas[i].u_constellation;
      uint8_t svid = z_GnssMeas[i].u_svid;
      gnss_Ephemeris_t eph = { 0 };

      if (TRUE == sd_api_GnssEphemeris_Get(NULL, constellation, svid, -1, &eph))
      {
        double time_diff = 0.001 * ((double)pz_gpsTime->q_towMsec - (double)eph.z_toeOfGpst.q_towMsec);
        if ((0.001 * (double)pz_gpsTime->q_towMsec < 1800.0) && ((double)WEEK_SEC + time_diff < 7220.0))
        {
          w_2ndBestWeek = eph.z_toeOfGpst.w_week + 1;
          break;
        }
      }
    }

    if (0 != w_2ndBestWeek)
    {
      /* Update measurement block time */
      tm_cvt_SetGpst(pz_gpsTime, w_2ndBestWeek, d_BestTow);
      return TRUE;
    }
    else
    {
      return FALSE;
    }
  }

  /* Update measurement block time */
  tm_cvt_SetGpst(pz_gpsTime, w_BestWeek, d_BestTow);
  return TRUE;
}

/**
 * @brief Compensate for 20ms*n jump for ST chip
 * @param[in,out] pz_satSigMeasCollect
 */
void gnss_SatSigMeasTimeJumpST(gnss_SatSigMeasCollect_t* pz_satSigMeasCollect)
{
  char u_satid[4] = {0};
  double d_cpdif = 0.0;
  double d_prdif = 0.0;
  double d_dist = 0.0;
  double d_wave = 0.0;
  double pd_siteCoor[3] = { 0.0 };
  double pd_satPosRot[3] = { 0.0 };
  double pd_unitVector[3] = { 0.0 };
  uint8_t u_carrier_compensation = 0;
  gnss_SatelliteMeas_t* pz_satMeas = NULL;
  gnss_SignalMeas_t* pz_signal = NULL;

  /*site coordinate*/
  for (int u_i = 0; u_i < 3; ++u_i)
  {
    pd_siteCoor[u_i] = pz_satSigMeasCollect->z_positionFix.d_xyz[u_i];
  }
  if (gnss_Norm(pd_siteCoor, 3) < FABS_ZEROS)
  {
    return;
  }

  for (uint8_t u_i = 0; u_i < pz_satSigMeasCollect->u_satMeasCount; ++u_i)
  {
    uint8_t u_idx  = pz_satSigMeasCollect->u_satMeasIdxTable[u_i];
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[u_idx];
    const double* pd_satPosClk=  pz_satMeas->z_satPosVelClk.d_satPosClk;
    // Distance
    if (gnss_Norm(pd_satPosClk, 3) < FABS_ZEROS)
    {
      for (uint8_t u_j = 0; u_j < MAX_GNSS_SIGNAL_FREQ; ++u_j)
      {
        if (pz_satMeas->pz_signalMeas[u_j] != NULL)
        {
          pz_satMeas->pz_signalMeas[u_j]->z_measStatusFlag.b_prValid = 0;
        }
      }
      continue;
    }

    gnss_satRot(pd_siteCoor, pd_satPosClk, pd_satPosRot);
    d_dist = gnss_unitVector(pd_siteCoor, pd_satPosRot, pd_unitVector);
    // diff
    for (uint8_t u_j = 0; u_j < MAX_GNSS_SIGNAL_FREQ; ++u_j)
    {
      pz_signal = pz_satMeas->pz_signalMeas[u_j];
      if ((NULL == pz_signal) || (fabs(pz_signal->d_pseudoRange) < FABS_ZEROS))
      {
        continue;
      }
      d_cpdif = 0.0;
      d_wave = wavelength(pz_signal->u_signal);
      if ((d_wave < 1.0 - FABS_ZEROS) && (pz_signal->d_carrierPhase != 0.0))
      {
        d_cpdif = pz_signal->d_carrierPhase * d_wave - d_dist;
      }
      d_prdif = pz_signal->d_pseudoRange - d_dist;
      if (fabs(d_prdif) > CLIGHT * 0.01)
      {
        double d_bitNum = d_prdif / CLIGHT / 0.02;
        double d_bn = round(d_bitNum);
        if ((fabs(d_bn - d_bitNum) < 0.1) || ((d_bn > 1) && (fabs(d_bn - d_bitNum)) < 0.3))
        {
          /* PR */
          pz_signal->d_pseudoRange -= d_bn * CLIGHT * 0.02;
          tm_GpstTimeAdd(&pz_satMeas->z_tot, d_bitNum * 0.02);
          /* CP */
          if ((d_cpdif != 0.0) && (round((d_cpdif - d_prdif)/ (CLIGHT * 0.02)) == 0.0))
          {
            pz_signal->d_carrierPhase -= d_bn * CLIGHT * 0.02 / d_wave;
            u_carrier_compensation = 1;
          }
          else
          {
            u_carrier_compensation = 0;
          }
          satidx_SatString(u_idx, u_satid);
          LOGI(TAG_HPP, "ST 20ms offset check: sat: %s, %d, frqidx: %d, pr: 1, cp:%d, jump: %.2f * 20ms \n", u_satid, u_idx,
              u_j, u_carrier_compensation, d_bn);
        }
      }
    }
  }
}

/**
 * @brief Using the result of PVT to fill up the mask of observations
 * @param[in]      pz_pvt_result is information of pvt
 * @param[in/out]  pz_satSigMeasCollect is the observation information
 * @return         void
 */
void gnss_fillUpObsMask(const gnss_PVTResult_t* pz_pvt_result, gnss_SatSigMeasCollect_t* pz_satSigMeasCollect)
{
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  uint8_t u_i = 0;
  gnss_SatelliteMeas_t* pz_satMeas = NULL;
  gnss_SignalMeas_t* pz_sigMeas = NULL;
  const gnss_MeasQos_t* pz_measQOS = NULL;
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; ++u_signalIndex)
    {
      pz_satMeas->pz_signalQuality[u_signalIndex].u_PRquality = PR_QUALITY_NORM;
      pz_satMeas->pz_signalQuality[u_signalIndex].u_CPquality = CP_QUALITY_NORM;
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas)
      {
        continue;
      }
      pz_sigMeas->u_slipMethodValid = 0;
      pz_sigMeas->z_measStatusFlag.b_prValid = 1;
      pz_sigMeas->z_measStatusFlag.b_drValid = 1;
      pz_sigMeas->z_measStatusFlag.b_cpValid = 1;
      for (u_i = 0; u_i < (pz_pvt_result->u_meas_count); ++u_i)
      {
        pz_measQOS = &(pz_pvt_result->pz_meas_qos[u_i]);
        if ((pz_measQOS->u_constellation == pz_sigMeas->u_constellation) && (pz_measQOS->u_svid == pz_sigMeas->u_svid)
          && (pz_measQOS->u_freq_idx == u_signalIndex))
        {
          if (0x1 != (pz_measQOS->u_status & 0x1))
          {
            pz_sigMeas->z_measStatusFlag.b_prValid = 0;
          }

          if (0x2 != (pz_measQOS->u_status & 0x2))
          {
            pz_sigMeas->z_measStatusFlag.b_drValid = 0;
          }

          if (0x8 != (pz_measQOS->u_status & 0x8))
          {
            pz_sigMeas->z_measStatusFlag.b_cpValid = 0;
          }
          break;
        }
      }
    }
  }
  return;
}

/**
 * @brief malloc the memory used in the struct gnss_fixedSignalAmbPool_t
 * @param[out]      pz_fixedAmbPool is the ambiguity pool of fixed signal
 * @param[in]       pz_satSigMeasCollect is the observation information
 * @return          0 represent successful, other represent failure
 */
uint8_t malloc_fixedSignalAmbPool(gnss_fixedSignalAmbPool_t** pz_fixedAmbPool, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  uint8_t u_status = 0;

  if (NULL == (*pz_fixedAmbPool))
  {
    (*pz_fixedAmbPool) = (gnss_fixedSignalAmbPool_t*)OS_MALLOC_FAST(sizeof(gnss_fixedSignalAmbPool_t));
    if (NULL == (*pz_fixedAmbPool))
    {
      u_status = 1;
      return u_status;
    }
    (*pz_fixedAmbPool)->u_fixStatus = GNSS_NONE_AMB_FIXED;
    (*pz_fixedAmbPool)->u_ppp_use_sat_num_fix = 0;
    (*pz_fixedAmbPool)->u_ppp_use_sat_num_wideLane = 0;
    for (u_i = 0; u_i < ALL_GNSS_SYS_SV_NUMBER; u_i++)
    {
      if (NULL == pz_satSigMeasCollect->pz_satMeas[u_i])
      {
        continue;
      }
      (*pz_fixedAmbPool)->pz_fixedAmbSet[u_i] = (gnss_fixedSignalAmb_t*)OS_MALLOC_FAST(sizeof(gnss_fixedSignalAmb_t));
      if (NULL == (*pz_fixedAmbPool)->pz_fixedAmbSet[u_i])
      {
        u_status = 2;
        return u_status;
      }
      for (u_j = 0; u_j < MAX_GNSS_SIGNAL_FREQ; u_j++)
      {
        (*pz_fixedAmbPool)->pz_fixedAmbSet[u_i]->u_freqType[u_j] = C_GNSS_SIG_MAX;
      }
    }
    (*pz_fixedAmbPool)->pd_x_fix = (double*)OS_MALLOC_FAST(GNSS_MAX_FILTER_STATE_NUM * sizeof(double));
    (*pz_fixedAmbPool)->pd_q_fix = (double*)OS_MALLOC_FAST(NUTM(GNSS_MAX_FILTER_STATE_NUM) * sizeof(double));
    if (NULL == (*pz_fixedAmbPool)->pd_x_fix || NULL == (*pz_fixedAmbPool)->pd_q_fix)
    {
      u_status = 3;
      return u_status;
    }
  }
  return u_status;
}
/**
 * @brief free the memory used in the struct gnss_fixedSignalAmbPool_t
 * @param[in]      pz_fixedAmbPool is the ambiguity pool of fixed signal
 * @return         void
 */
void free_fixedSignalAmbPool(gnss_fixedSignalAmbPool_t** pz_fixedAmbPool)
{
  uint8_t u_i = 0;
  if (NULL != pz_fixedAmbPool && NULL != (*pz_fixedAmbPool))
  {
    for (u_i = 0; u_i < ALL_GNSS_SYS_SV_NUMBER; u_i++)
    {
      OS_FREE((*pz_fixedAmbPool)->pz_fixedAmbSet[u_i]);
    }
    OS_FREE((*pz_fixedAmbPool)->pd_x_fix);
    OS_FREE((*pz_fixedAmbPool)->pd_q_fix);
    OS_FREE((*pz_fixedAmbPool));
  }
  if (NULL != pz_fixedAmbPool)
  {
    (*pz_fixedAmbPool) = NULL;
  }
  return;
}


/* *************************************************************************************************************************************************
****** Common function of Ambguity resolution ***********/

#define LOOPMAX     (10000)           /* maximum count of search loop */
#define SGN(x)      ((x)<=0.0?-1.0:1.0)
#define SWAP(x,y)   do {double tmp_; tmp_=x; x=y; y=tmp_;} while (0)

/**
 * @brief LD factorization (Q=L'*diag(D)*L)
 */
extern int8_t LD(uint32_t n, const double* Q, double* L, double* D)
{
  int8_t info = 0;
  int32_t i = 0;
  int32_t j = 0;
  int32_t k = 0;
  double a = 0;
  double* A = OS_MALLOC_FAST(sizeof(double) * n * n);

  if (NULL == A)
  {
    return -1;
  }

  memcpy(A, Q, sizeof(double) * n * n);
  for (i = n - 1; i >= 0; i--)
  {
    if ((D[i] = A[i + i * n]) <= 0.0)
    {
      info = -1; break;
    }
    a = sqrt(D[i]);
    for (j = 0; j <= i; j++)
    {
      L[i + j * n] = A[i + j * n] / a;
    }
    for (j = 0; j <= i - 1; j++)
    {
      for (k = 0; k <= j; k++)
      {
        A[j + k * n] -= L[i + k * n] * L[i + j * n];
      }
    }
    for (j = 0; j <= i; j++)
    {
      L[i + j * n] /= L[i + i * n];
    }
  }
  OS_FREE(A);
  A = NULL;
  if (info)
  {
    LOGE(TAG_HPP, ": LD factorization error\n");
  }
  return info;
}

/**
 * @brief integer gauss transformation
 */
extern void gauss(uint32_t n, double* L, double* Z, uint32_t i, uint32_t j)
{
  uint32_t k = 0;
  int64_t mu = 0;

  if (0 != (mu = (int64_t)gnss_Round(L[i + j * n], 0)))
  {
    for (k = i; k < n; k++)
    {
      L[k + n * j] -= (double)mu * L[k + i * n];
    }
    for (k = 0; k < n; k++)
    {
      Z[k + n * j] -= (double)mu * Z[k + i * n];
    }
  }
}

/**
 * @brief permutations
 */
extern void perm(int32_t n, double* L, double* D, int32_t j, double del, double* Z)
{
  int32_t k = 0;
  double eta = 0.0;
  double lam = 0.0;
  double a0 = 0.0;
  double a1 = 0.0;

  eta = D[j] / del;
  lam = D[j + 1] * L[j + 1 + j * n] / del;
  D[j] = eta * D[j + 1];
  D[j + 1] = del;
  for (k = 0; k <= j - 1; k++)
  {
    a0 = L[j + k * n]; a1 = L[j + 1 + k * n];
    L[j + k * n] = -L[j + 1 + j * n] * a0 + a1;
    L[j + 1 + k * n] = eta * a0 + lam * a1;
  }
  L[j + 1 + j * n] = lam;
  for (k = j + 2; k < n; k++)
  {
    SWAP(L[k + j * n], L[k + (j + 1) * n]);
  }
  for (k = 0; k < n; k++)
  {
    SWAP(Z[k + j * n], Z[k + (j + 1) * n]);
  }
}

/**
 * @brief lambda reduction (z=Z'*a, Qz=Z'*Q*Z=L'*diag(D)*L) (ref.[1])
 */
static  void reduction(uint32_t n, double* L, double* D, double* Z, uint32_t* index)
{
  uint32_t i = 0;
  uint32_t temp = 0;
  int32_t j = 0;
  int32_t k = 0;
  double del = 0.0;
  j = n - 2;
  k = n - 2;
  while (j >= 0)
  {
    if (j <= k)
    {
      for (i = j + 1; i < n; i++)
      {
        gauss(n, L, Z, i, j);
      }
    }
    del = D[j] + L[j + 1 + j * n] * L[j + 1 + j * n] * D[j + 1];
    if (del + 1E-6 < D[j + 1]) /* compared considering numerical error */
    {
      perm(n, L, D, j, del, Z);
      if (NULL != index)
      {
        temp = index[j];
        index[j] = index[j + 1];
        index[j + 1] = temp;
      }
      k = j;
      j = n - 2;
    }
    else j--;
  }
}

/**
 * @brief modified lambda (mlambda) search (ref. [2])
 */
static int8_t search(int32_t n, int32_t m, const double* L, const double* D,
  const double* zs, double* zn, double* s)
{
  int32_t i = 0;
  int32_t j = 0;
  int32_t k = 0;
  int32_t c = 0;
  int32_t nn = 0;
  int32_t imax = 0;
  double newdist = 0.0;
  double maxdist = 1E99;
  double y = 0.0;
  double* S = OS_MALLOC_FAST(n * n * sizeof(double));
  double* dist = OS_MALLOC_FAST(n * sizeof(double));
  double* zb = OS_MALLOC_FAST(n * sizeof(double));
  double* z = OS_MALLOC_FAST(n * sizeof(double));
  double* step = OS_MALLOC_FAST(n * sizeof(double));

  if (NULL == S || NULL == dist || NULL == zb || NULL == z || NULL == step)
  {
    if (NULL != S)
    {
      OS_FREE(S);
      S = NULL;
    }
    if (NULL != dist)
    {
      OS_FREE(dist);
      dist = NULL;
    }
    if (NULL != zb)
    {
      OS_FREE(zb);
      zb = NULL;
    }
    if (NULL != z)
    {
      OS_FREE(z);
      z = NULL;
    }
    if (NULL != step)
    {
      OS_FREE(step);
      step = NULL;
    }
    return -1;
  }

  k = n - 1;
  dist[k] = 0.0;
  zb[k] = zs[k];
  z[k] = gnss_Round(zb[k], 0);
  y = zb[k] - z[k];
  step[k] = SGN(y);
  for (i = 0; i < n * n; i++)
  {
    S[i] = 0.0;
  }
  for (c = 0; c < LOOPMAX; c++)
  {
    newdist = dist[k] + y * y / D[k];
    if (newdist < maxdist)
    {
      if (0 != k)
      {
        dist[--k] = newdist;
        for (i = 0; i <= k; i++)
        {
          S[k + i * n] = S[k + 1 + i * n] + (z[k + 1] - zb[k + 1]) * L[k + 1 + i * n];
        }
        zb[k] = zs[k] + S[k + k * n];
        z[k] = gnss_Round(zb[k], 0);
        y = zb[k] - z[k];
        step[k] = SGN(y);
      }
      else
      {
        if (nn < m)
        {
          if (nn == 0 || newdist > s[imax])
          {
            imax = nn;
          }
          for (i = 0; i < n; i++)
          {
            zn[i + nn * n] = z[i];
          }
          s[nn++] = newdist;
        }
        else
        {
          if (newdist < s[imax])
          {
            for (i = 0; i < n; i++)
            {
              zn[i + imax * n] = z[i];
            }
            s[imax] = newdist;
            for (i = imax = 0; i < m; i++)
            {
              if (s[imax] < s[i])
              {
                imax = i;
              }
            }
          }
          maxdist = s[imax];
        }
        z[0] += step[0];
        y = zb[0] - z[0];
        step[0] = -step[0] - SGN(step[0]);
      }
    }
    else
    {
      if (k == (n - 1))
      {
        break;
      }
      else
      {
        k++;
        z[k] += step[k];
        y = zb[k] - z[k];
        step[k] = -step[k] - SGN(step[k]);
      }
    }
  }
  for (i = 0; i < m - 1; i++) /* sort by s */
  {
    for (j = i + 1; j < m; j++)
    {
      if (s[i] < s[j])
      {
        continue;
      }
      SWAP(s[i], s[j]);
      for (k = 0; k < n; k++)
      {
        SWAP(zn[k + i * n], zn[k + j * n]);
      }
    }
  }
  OS_FREE(S);
  S = NULL;
  OS_FREE(dist);
  dist = NULL;
  OS_FREE(zb);
  zb = NULL;
  OS_FREE(z);
  z = NULL;
  OS_FREE(step);
  step = NULL;

  if (c >= LOOPMAX)
  {
    LOGE(TAG_HPP, "search loop count overflow\n");
    return -1;
  }
  return 0;
}

/**
 * @brief calculation of normal distribution
 */
static double norm_distribution(const double z)
{
  static const double gamma = 0.231641900;
  static const double   a1 = 0.319381530;
  static const double  a2 = -0.356563782;
  static const double a3 = 1.781477973;
  static const double  a4 = -1.821255978;
  static const double a5 = 1.330274429;

  double k = 1.0 / (1 + fabs(z) * gamma);
  double n = k * (a1 + k * (a2 + k * (a3 + k * (a4 + k * a5))));

  /* this guards against overflow */
  if (z > 6)
  {
    return 1;
  }
  if (z < -6)
  {
    return 0;
  }

  n = 1 - exp((-1) * z * z / 2) / sqrt(2 * PI) * n;
  if (z < 0)
  {
    return (1.0 - n);
  }

  return n;
}

/**
 * @brief calculation of bootstrap from D
 */
static double bootstrap(const double* D, const uint32_t n)
{
  uint32_t i = 0;
  double* np = NULL;
  double boot = 1.0;

  np = OS_MALLOC_FAST(n * sizeof(double));
  if (NULL == np)
  {
    return 0.0;
  }

  for (i = 0; i < n; i++)
  {
    np[i] = 2 * norm_distribution(1 / (2 * sqrt(D[i]))) - 1;
    boot *= np[i];
  }
  OS_FREE(np);
  np = NULL;
  return boot;
}

/**
 * @brief solve linear equation
 */
static int8_t solve(matrix_t* A, matrix_t* Y, double* X)
{
  matrix_t* B = NULL;
  matrix_t* x = NULL;
  int8_t info = 1;

  B = matrix_new(A->row, A->col);
  x = matrix_new_from_list(A->col, X);
  if (NULL == B || NULL == x)
  {
    if (NULL != B)
    {
      matrix_free(&B);
      B = NULL;
    }
    if (NULL != x)
    {
      matrix_free(&x);
      x = NULL;
    }
    return -1;
  }

  if (matrix_inverse(A, B))
  {
    info = 0;
    matrix_mul(B, Y, x);
    memcpy(X, x->data, x->col * x->row * sizeof(double));
  }
  matrix_free(&B);
  B = NULL;
  matrix_free(&x);
  x = NULL;
  return info;
}

/**
 * @brief Best integer equivariant
 * @param[in]   n        -  number of float parameters
 * @param[in]   m        -  number of candidate set
 * @param[in]   zdouble  -  float parameters
 * @param[in]   zint     -  int parameters
 * @param[in]   s        -  sum of squared residulas of fixed solutions
 * @param[out]  zw       -  result of parameters computed by Best integer equivariant
 * @return status (0:ok,other:error)
 */
static int8_t zweight(uint32_t n, uint32_t m, const double* zdouble, const double* zint, const double* s, double* zw)
{
  uint32_t i = 0;
  uint32_t j = 0;
  uint32_t nselect = 0;
  double sumweight = 0.0;
  double* wtemp = NULL;
  double* w = NULL;
  double coef = 1.0;

  for (i = 0; i < m; i++)
  {
    if (s[i] / s[0] > 100.0)
    {
      break;
    }
  }
  coef = 12 / s[0];

  nselect = i < 10 ? 10 : i;
  wtemp = OS_MALLOC_FAST(nselect * sizeof(double));
  w = OS_MALLOC_FAST(nselect * sizeof(double));
  if (NULL == wtemp || NULL == w)
  {
    if (NULL != wtemp)
    {
      OS_FREE(wtemp);
      wtemp = NULL;
    }
    if (NULL != w)
    {
      OS_FREE(w);
      w = NULL;
    }
    return -1;
  }

  for (i = 0; i < nselect; i++)
  {
    wtemp[i] = exp(-0.5 * coef * s[i]) + 1e-200; /* for wtem==0.0; */
    sumweight += wtemp[i];
  }
  for (i = 0; i < nselect; i++)
  {
    w[i] = wtemp[i] / sumweight;
  }
  for (i = 0; i < n; i++)
  {
    zw[i] = zdouble[i];
    for (j = 0; j < nselect; j++)
    {
      zw[i] += (zint[i + n * j] - zdouble[i]) * w[j];
    }
  }

  OS_FREE(wtemp);
  wtemp = NULL;
  OS_FREE(w);
  w = NULL;
  return 0;
}

/**
 * @brief lambda/mlambda integer least-square estimation
 * @param[in]   n  -  number of float parameters
 * @param[in]   m  -  number of fixed solutions
 * @param[in]   a  -  float parameters (n x 1)
 * @param[in]   Q  -  covariance matrix of float parameters (n x n)
 * @param[out]  F  -  fixed solutions (n x m)
 * @param[out]  s  -  sum of squared residulas of fixed solutions (1 x m)
 * @param[out]  boot_rate  -  rate of bootstrap
 * @param[out]  pf_adop
 * @return status (0:ok,other:error)
 */
extern int8_t amb_lambda(uint32_t n, uint32_t m, const double* a, const double* Q, double* F,
  double* s, double* boot_rate, float* pf_adop)
{
  uint8_t u_i;
  int8_t info = 0;
  double d_adop = 1.0;
  matrix_t* L = NULL;
  matrix_t* D = NULL;
  matrix_t* Z = NULL;
  matrix_t* z = NULL;
  matrix_t* E = NULL;
  matrix_t* A = NULL;

  if (NULL != pf_adop) *pf_adop = -1.0;
  if (NULL != boot_rate) *boot_rate = 0.0;
  if (0 == n || 0 == m)
  {
    return -1;
  }
  L = matrix_new(n, n);
  D = matrix_new(n, 1);
  Z = matrix_ones(n);
  z = matrix_new(n, 1);
  E = matrix_new(n, m);
  A = matrix_new_from_list(n, a);
  if (NULL == L || NULL == D || NULL == Z || NULL == z || NULL == E || NULL == A)
  {
    if (NULL != L)
    {
      matrix_free(&L);
      L = NULL;
    }
    if (NULL != D)
    {
      matrix_free(&D);
      D = NULL;
    }
    if (NULL != Z)
    {
      matrix_free(&Z);
      Z = NULL;
    }
    if (NULL != z)
    {
      matrix_free(&z);
      z = NULL;
    }
    if (NULL != E)
    {
      matrix_free(&E);
      E = NULL;
    }
    if (NULL != A)
    {
      matrix_free(&A);
      A = NULL;
    }
    return -1;
  }

  /* LD factorization */
  if (!(info = LD(n, Q, L->data, D->data)))
  {
    for (u_i = 0; u_i < n; u_i++)
    {
      d_adop *= D->data[u_i];
    }
    if (d_adop > 0.0 && NULL != pf_adop)
    {
      *pf_adop = (float)(100.0 * pow(d_adop, 1.0 / (2.0 * n)));
    }
    if (boot_rate)
    {
      *boot_rate = bootstrap(D->data, n);
    }
    /* lambda reduction */
    reduction(n, L->data, D->data, Z->data, NULL);
    matrix_mul(Z, A, z); /* z=Z'*a */

    /* mlambda search */
    if (!(info = search(n, m, L->data, D->data, z->data, E->data, s)))
    {
      info = solve(Z, E, F); /* F=Z'\E */
    }
  }
  matrix_free(&L);
  L = NULL;
  matrix_free(&D);
  D = NULL;
  matrix_free(&Z);
  Z = NULL;
  matrix_free(&z);
  z = NULL;
  matrix_free(&E);
  E = NULL;
  matrix_free(&A);
  A = NULL;
  return info;
}

/**
 * @brief lambda/mlambda best integer equivariant
 * @param[in]   n  -  number of float parameters
 * @param[in]   m  -  number of fixed solutions
 * @param[in]   a  -  float parameters (n x 1)
 * @param[in]   Q  -  covariance matrix of float parameters (n x n)
 * @param[out]  F  -  fixed solutions (n x m)
 * @param[out]  s  -  sum of squared residulas of fixed solutions (1 x m)
 * @param[out]  boot_rate  -  rate of bootstrap
 * @param[out]  pf_adop
 * @return status (0:ok,other:error)
 */
extern int8_t lambdaBie(uint32_t n, uint32_t m, const double* a, const double* Q, double* F,
  double* s, float* boot_rate, float* pf_adop)
{
  uint8_t u_i;
  int8_t info = 0;
  double d_adop = 1.0;
  matrix_t* L = NULL;
  matrix_t* D = NULL;
  matrix_t* Z = NULL;
  matrix_t* z = NULL;
  matrix_t* E = NULL;
  matrix_t* zw = NULL;
  matrix_t* A = NULL;

  if (NULL != pf_adop) *pf_adop = -1.0;
  if (NULL != boot_rate) *boot_rate = 0.0;
  if (0 == n || 0 == m)
  {
    return -1;
  }
  L = matrix_new(n, n);
  D = matrix_new(n, 1);
  Z = matrix_ones(n);
  z = matrix_new(n, 1);
  E = matrix_new(n, m);
  zw = matrix_new(n, 1);
  A = matrix_new_from_list(n, a);
  if (NULL == L || NULL == D || NULL == Z || NULL == z || NULL == E || NULL == zw || NULL == A)
  {
    if (NULL != L)
    {
      matrix_free(&L);
      L = NULL;
    }
    if (NULL != D)
    {
      matrix_free(&D);
      D = NULL;
    }
    if (NULL != Z)
    {
      matrix_free(&Z);
      Z = NULL;
    }
    if (NULL != z)
    {
      matrix_free(&z);
      z = NULL;
    }
    if (NULL != E)
    {
      matrix_free(&E);
      E = NULL;
    }
    if (NULL != zw)
    {
      matrix_free(&zw);
      zw = NULL;
    }
    if (NULL != A)
    {
      matrix_free(&A);
      A = NULL;
    }
    return -1;
  }

  /* LD factorization */
  if (!(info = LD(n, Q, L->data, D->data)))
  {
    for (u_i = 0; u_i < n; u_i++)
    {
      d_adop *= D->data[u_i];
    }
    if (d_adop > 0.0 && NULL != pf_adop)
    {
      *pf_adop = (float)(100.0 * pow(d_adop, 1.0 / (2.0 * n)));
    }

    if (NULL != boot_rate)
    {
      *boot_rate = (float)bootstrap(D->data, n);
    }
    /* lambda reduction */
    reduction(n, L->data, D->data, Z->data, NULL);
    matrix_mul(Z, A, z);   /* z=Z'*a */

    /* mlambda search */
    if (!(info = search(n, m, L->data, D->data, z->data, E->data, s)))
    {
      if (!(info = zweight(n, m, z->data, E->data, s, zw->data)))
      {
        info = solve(Z, zw, F); /* F=Z'\E */
      }
    }
  }
  matrix_free(&L);
  L = NULL;
  matrix_free(&D);
  D = NULL;
  matrix_free(&Z);
  Z = NULL;
  matrix_free(&z);
  z = NULL;
  matrix_free(&E);
  E = NULL;
  matrix_free(&zw);
  zw = NULL;
  matrix_free(&A);
  A = NULL;
  return info;
}
/****** End of common function of Ambguity resolution ***********/

/**
 * @brief get satellite transmit time
 * @param[in] pz_meas measurement
 * @param[in,out] gpst time of receiver
 * @return FALSE:success, TRUE: failed
 */
BOOL gnss_CalMeasTot(const gnss_SatelliteMeas_t* pz_meas, GpsTime_t* tor)
{
  BOOL status = FALSE;
  double d_pr = 0.0;
  for (int i = 0; i < MAX_GNSS_SIGNAL_FREQ; ++i)
  {
    if (NULL != pz_meas->pz_signalMeas[i])
    {
      if (pz_meas->pz_signalMeas[i]->d_pseudoRange > 0)
      {
        d_pr = pz_meas->pz_signalMeas[i]->d_pseudoRange;
        tm_GpstTimeAdd(tor, -d_pr / CLIGHT);
        status = TRUE;
        break;
      }
    }
  }
  return status;
}

/**
 * @brief calculate time of transmit of satellite by the matched gnss measurement
 * @param[in] pz_meas GNSS Measurement structure
 * @param[in] n number of the pz_meas
 * @param[in] u_constellation system
 * @param[in] u_svid PRN
 * @param[in,out] tor time of recevier[in],  time of transmit[out]
 * @return FALSE: failed; TRUE: success
 */
BOOL gnss_CalGnssMeasTot(const GnssMeas_t* pz_meas, uint8_t n, gnss_ConstellationType u_constellation, uint8_t u_svid, GpsTime_t* tor)
{
  BOOL status = FALSE;
  double d_pseudoRange = 0.0;
  for (uint8_t i = 0; i < n; ++i)
  {
    if ((u_constellation == pz_meas[i].u_constellation) &&
        (u_svid == pz_meas[i].u_svid) &&
        (pz_meas[i].d_pseudoRange > FABS_ZEROS))
    {
      d_pseudoRange = pz_meas[i].d_pseudoRange;
      break;
    }
  }
  if (d_pseudoRange > FABS_ZEROS)
  {
    tm_GpstTimeAdd(tor, -d_pseudoRange / CLIGHT);
    status = TRUE;
  }
  return status;
}
/**
 * @brief compute DOP (dilution of precision)
 * @param[in] u_ns -- number of satellite
 * @param[in] pf_azel -- satellite azimuth/elevation angle (rad)
 * @param[out] pf_dop -- DOPs {GDOP,PDOP,HDOP,VDOP}
 * @return none
 */
extern void gnss_dops(uint8_t u_ns, const float* pf_azel, float* pf_dop)
{
  uint8_t u_i = 0;
  double cosel = 0.0;
  double sinel = 0.0;
  matrix_t* pd_H = NULL;
  matrix_t* pd_HT = NULL;
  matrix_t* pd_Q = NULL;
  matrix_t* pd_Qinver = NULL;

  if (u_ns < 4)
  {
    pf_dop[0] = 999.0f;
    return;
  }
  pd_H = matrix_new(u_ns, 4);
  pd_Q = matrix_new(4, 4);
  pd_Qinver = matrix_new(4, 4);;

  if (NULL == pd_H || NULL == pd_Q ||NULL== pd_Qinver)
  {
    matrix_free(&pd_H);
    matrix_free(&pd_Q);
    matrix_free(&pd_Qinver);
    return;
  }
  for (u_i = 0; u_i < 4; u_i++) pf_dop[u_i] = 0.0;
  for (u_i = 0; u_i < u_ns; u_i++) {
    cosel = cos(pf_azel[1 + u_i * 2]);
    sinel = sin(pf_azel[1 + u_i * 2]);
    pd_H->data[4 * u_i] = cosel * sin(pf_azel[u_i * 2]);
    pd_H->data[1 + 4 * u_i] = cosel * cos(pf_azel[u_i * 2]);
    pd_H->data[2 + 4 * u_i] = sinel;
    pd_H->data[3 + 4 * u_i] = 1.0;
  }
  pd_HT = matrix_new_from_array(pd_H->row, pd_H->col, pd_H->data);
  //if(NULL != pd_H )
  {
    matrix_transpose(pd_HT);
    matrix_mul(pd_HT, pd_H, pd_Q);
    if (matrix_inverse(pd_Q, pd_Qinver))
    {
      pf_dop[0] = (float)sqrt(pd_Qinver->data[0] + pd_Qinver->data[5] + pd_Qinver->data[10] + pd_Qinver->data[15]); /* GDOP */
      pf_dop[1] = (float)sqrt(pd_Qinver->data[0] + pd_Qinver->data[5] + pd_Qinver->data[10]);       /* PDOP */
      pf_dop[2] = (float)sqrt(pd_Qinver->data[0] + pd_Qinver->data[5]);             /* HDOP */
      pf_dop[3] = (float)sqrt(pd_Qinver->data[10]);                 /* VDOP */
    }
  }
  matrix_free(&pd_H);
  matrix_free(&pd_HT);
  matrix_free(&pd_Q);
  matrix_free(&pd_Qinver);

  return;
}

/**
 * @brief initilize the struct of GnssMeasFeedbackInsBlock_t
 * @param[out] pz_feedbackInsMeasBlock the measure blcok that GNSS feedback to INS
 * @return void
 */
void cmn_InitGnssMeasFeedbackInsBlock(gnss_FeedbackInsMeasBlock_t* pz_GnssFeedbackInsMeasBlock)
{
  uint8_t u_i = 0;
  uint8_t u_j = 0;
  gnss_FeedbackInsMeasUnit_t* pz_GnssFeedbackInsMeasUnit = NULL;
  if (NULL != pz_GnssFeedbackInsMeasBlock)
  {
    memset(&pz_GnssFeedbackInsMeasBlock->z_GpsTime, 0, sizeof(GpsTime_t));
    pz_GnssFeedbackInsMeasBlock->u_FreqType = C_GNSS_FREQ_TYPE_L1;
    pz_GnssFeedbackInsMeasBlock->u_CurFixFlag = GNSS_FIX_FLAG_INVALID;
    for (u_i = 0; u_i < 3; ++u_i)
    {
      pz_GnssFeedbackInsMeasBlock->d_CurLLA[u_i] = 0.0;
      pz_GnssFeedbackInsMeasBlock->d_PreLLA[u_i] = 0.0;
    }
    pz_GnssFeedbackInsMeasBlock->f_QuasiGeoidHeight = 0.0f;
    pz_GnssFeedbackInsMeasBlock->u_PreFixFlag = GNSS_FIX_FLAG_INVALID;
    pz_GnssFeedbackInsMeasBlock->f_interval = 0.0;
    pz_GnssFeedbackInsMeasBlock->u_FeedbackMeasType = GNSS_FEEDBACK_PR_DOPPLER;
    pz_GnssFeedbackInsMeasBlock->u_MeasCount = 0;
    for (u_i = 0; u_i < MAX_FEEDBACK_INS_MEAS_NUM; ++u_i)
    {
      pz_GnssFeedbackInsMeasUnit = &(pz_GnssFeedbackInsMeasBlock->z_GnssFeedbackInsMeasUnit[u_i]);
      pz_GnssFeedbackInsMeasUnit->u_constellation = C_GNSS_NONE;
      pz_GnssFeedbackInsMeasUnit->u_svid = 0;
      pz_GnssFeedbackInsMeasUnit->u_cn0 = 0;
      pz_GnssFeedbackInsMeasUnit->u_obs_valid = FALSE;
      pz_GnssFeedbackInsMeasUnit->f_epoch_diff_obs = 0.0;
      pz_GnssFeedbackInsMeasUnit->f_epoch_diff_res = 0.0;
      pz_GnssFeedbackInsMeasUnit->f_doppler = 0.0;
      pz_GnssFeedbackInsMeasUnit->d_pseudorange = 0.0;
      pz_GnssFeedbackInsMeasUnit->f_elevation = 0.0f;
      pz_GnssFeedbackInsMeasUnit->f_azimuth = 0.0f;
      for (u_j = 0; u_j < 3; ++u_j)
      {
        pz_GnssFeedbackInsMeasUnit->f_unit_dir_vect[u_j] = 0.0f;
        pz_GnssFeedbackInsMeasUnit->d_sat_pos[u_j] = 0.0;
        pz_GnssFeedbackInsMeasUnit->d_sat_vel[u_j] = 0.0;
        pz_GnssFeedbackInsMeasUnit->d_pre_sat_pos[u_j] = 0.0;
      }
      pz_GnssFeedbackInsMeasUnit->d_pr_var = 0.0;
      pz_GnssFeedbackInsMeasUnit->d_dr_var = 0.0;
    }
  }
  return;
}

/**
 * @brief Data log of protection level
 * @param[in] u_solFlag algorithm solution status, MAX_FILTER_POS_STATUS, ppp default is 0
 * @param[in] pz_pl protection level result
 * @param[in] gpst GSPT
 */
extern void
gnss_log_ProtectionLevel(uint8_t u_solFlag, const integ_PLResult_t* pz_pl, const gnss_PositionFix_t* pz_posSolution)
{
  if (!DATA_LOG_ENABLE())
  {
    return;
  }
  if (pz_posSolution->u_fixFlag == GNSS_FIX_FLAG_INVALID)
  {
    return;
  }

  const GpsTime_t* gpst = &pz_posSolution->z_gpsTime;

  EpochTime_t z_epochT = {0};
  tm_cvt_GpstToEpoch(gpst, &z_epochT);

  DATA_LOG_HEADER(DLOG_GNSS_PL, "week, tow, utc_time, flag, solFlag, pl_e, pl_n, pl_u, pl_h, pl_v\n");
  // $week,sec,utc_time,flag,solFlag,pl_e,pl_n,pl_u,pl_h,pl_v
  DATA_LOG(DLOG_GNSS_PL, "%d, %.1f, %4d/%02d/%02d %02d:%02d:%05.2f, %d, %d, %.2f, %.2f, %.2f, %.2f, %.2f\n", gpst->w_week,
           gpst->q_towMsec * TIME_MSEC_INV, z_epochT.year, z_epochT.month, z_epochT.day, z_epochT.hour, z_epochT.min,
           z_epochT.second, pz_posSolution->u_fixFlag, u_solFlag, pz_pl->f_enu_protection_level[0],
           pz_pl->f_enu_protection_level[1], pz_pl->f_enu_protection_level[2], pz_pl->f_hv_protection_level[0],
           pz_pl->f_hv_protection_level[1]);

  DATA_LOG_HEADER(DLOG_GNSS_SCENE, "week, tow, utc_time, flag, solFlag, sceneType\n");
  DATA_LOG(DLOG_GNSS_SCENE, "%d, %.1f, %4d/%02d/%02d %02d:%02d:%05.2f, %d, %d, %d\n", gpst->w_week,
    gpst->q_towMsec * TIME_MSEC_INV, z_epochT.year, z_epochT.month, z_epochT.day, z_epochT.hour, z_epochT.min,
    z_epochT.second, pz_posSolution->u_fixFlag, u_solFlag, pz_pl->u_sceneType);
}

 /**
  * @brief Data log of STD
  * @param[in] u_solFlag algorithm solution status, gnss_FixFlagType
  * @param[in] pz_posSolution
  */
extern void gnss_log_STD(uint8_t u_solFlag, const gnss_PositionFix_t* pz_posSolution)
{
  if (!DATA_LOG_ENABLE())
  {
    return;
  }
  if (pz_posSolution->u_fixFlag == GNSS_FIX_FLAG_INVALID)
  {
    return;
  }

  const GpsTime_t* gpst = &pz_posSolution->z_gpsTime;

  EpochTime_t z_epochT = {0};
  tm_cvt_GpstToEpoch(gpst, &z_epochT);

  float std_hor = pz_posSolution->f_posLlaUnc[0] * 1.414f;
  DATA_LOG_HEADER(DLOG_GNSS_STD, "week, tow, utc_time, flag, solFlag, std_lat, std_lon, std_alt, std_hor\n");
  // week, tow, utc_time, flag, solFlag, std_lat, std_lon, std_alt
  DATA_LOG(DLOG_GNSS_STD, "%d, %.1f, %4d/%02d/%02d %02d:%02d:%05.2f, %d, %d, %.2f, %.2f, %.2f, %.2f\n", gpst->w_week,
           gpst->q_towMsec * TIME_MSEC_INV, z_epochT.year, z_epochT.month, z_epochT.day, z_epochT.hour, z_epochT.min,
           z_epochT.second, pz_posSolution->u_fixFlag,
           u_solFlag, pz_posSolution->f_posLlaUnc[0], pz_posSolution->f_posLlaUnc[1],pz_posSolution->f_posLlaUnc[2],
           std_hor);
}

/**
 * @brief Position uncertainty default value that by position solution status
 * @param[in,out] pz_posSolution
 */
extern void gnss_PositionUncertaintyMajic(gnss_PositionFix_t* pz_posSolution)
{
  float f_default_NE[4] = {10.0f, 5.0f, 2.0f, 0.05f};   /* SPP, DGNSS, FIXED, FLOAT, unit: m */
  double d_posLlaUnc[3] = {0.0};
  double d_posXyzUnc[3] = {0.0};
  uint8_t index = 0;

  switch (pz_posSolution->u_fixFlag)
  {
    case GNSS_FIX_FLAG_SPS:
      index = 0;
      break;
    case GNSS_FIX_FLAG_DGNSS:
      index = 1;
      break;
    case GNSS_FIX_FLAG_FLOATING:
      index = 2;
      break;
    case GNSS_FIX_FLAG_FIXED:
      index = 3;
      break;
    default:
      index = 0;
  }
  /* SPP have uncertainty */
  if (pz_posSolution->u_fixFlag == GNSS_FIX_FLAG_SPS)
  {
    return;
  }
  /* LLA uncertain, lla */
  pz_posSolution->f_posLlaUnc[0] = f_default_NE[index];
  pz_posSolution->f_posLlaUnc[1] = f_default_NE[index];
  pz_posSolution->f_posLlaUnc[2] = f_default_NE[index] * 2;

  /* Position uncertain, xyz */
  d_posLlaUnc[0] = (double) pz_posSolution->f_posLlaUnc[0];
  d_posLlaUnc[1] = (double) pz_posSolution->f_posLlaUnc[1];
  d_posLlaUnc[2] = (double) pz_posSolution->f_posLlaUnc[2];
  gnss_Enu2Ecef(pz_posSolution->d_lla, d_posLlaUnc, d_posXyzUnc);
  pz_posSolution->f_posXyzUnc[0] = (float) d_posXyzUnc[0];
  pz_posSolution->f_posXyzUnc[1] = (float) d_posXyzUnc[1];
  pz_posSolution->f_posXyzUnc[2] = (float) d_posXyzUnc[2];
}

/**
 * @brief convert PL struct to API
 * @param[in] pz_src gnss_ProtectionLevel_t
 * @param[in,out] pz_dst loc_api_protectionLevel_t
 */
static void gnss_cvt_ProtectionLevel_To_ApiProtectionLevel(const gnss_ProtectionLevel_t* pz_src, loc_api_protectionLevel_t* pz_dst)
{
  pz_dst->f_protection_level = pz_src->f_protection_level;
  pz_dst->u_flag = pz_src->u_flag;
}

/**
 * @brief convert integrity struct to API
 * @param[in] pz_src gnss_Integrity_t
 * @param[in,out] pz_dst loc_api_integrity_t
 */
static void gnss_cvt_Integrity_To_ApiIntegrity(const gnss_Integrity_t* pz_src, loc_api_integrity_t* pz_dst)
{
  gnss_cvt_ProtectionLevel_To_ApiProtectionLevel(&pz_src->z_pos_longitudinal_pl,&pz_dst->z_pos_longitudinal_pl );
  gnss_cvt_ProtectionLevel_To_ApiProtectionLevel(&pz_src->z_pos_lateral_pl     ,&pz_dst->z_pos_lateral_pl      );
  gnss_cvt_ProtectionLevel_To_ApiProtectionLevel(&pz_src->z_pos_north_pl       ,&pz_dst->z_pos_north_pl        );
  gnss_cvt_ProtectionLevel_To_ApiProtectionLevel(&pz_src->z_pos_east_pl        ,&pz_dst->z_pos_east_pl         );
  gnss_cvt_ProtectionLevel_To_ApiProtectionLevel(&pz_src->z_pos_hor_pl         ,&pz_dst->z_pos_hor_pl          );
  gnss_cvt_ProtectionLevel_To_ApiProtectionLevel(&pz_src->z_pos_ver_pl         ,&pz_dst->z_pos_ver_pl          );
  gnss_cvt_ProtectionLevel_To_ApiProtectionLevel(&pz_src->z_vel_longitudinal_pl,&pz_dst->z_vel_longitudinal_pl );
  gnss_cvt_ProtectionLevel_To_ApiProtectionLevel(&pz_src->z_vel_lateral_pl     ,&pz_dst->z_vel_lateral_pl      );
  gnss_cvt_ProtectionLevel_To_ApiProtectionLevel(&pz_src->z_vel_north_pl       ,&pz_dst->z_vel_north_pl        );
  gnss_cvt_ProtectionLevel_To_ApiProtectionLevel(&pz_src->z_vel_east_pl        ,&pz_dst->z_vel_east_pl         );
  gnss_cvt_ProtectionLevel_To_ApiProtectionLevel(&pz_src->z_vel_hor_pl         ,&pz_dst->z_vel_hor_pl          );
  gnss_cvt_ProtectionLevel_To_ApiProtectionLevel(&pz_src->z_vel_ver_pl         ,&pz_dst->z_vel_ver_pl          );
  gnss_cvt_ProtectionLevel_To_ApiProtectionLevel(&pz_src->z_roll_pl            ,&pz_dst->z_roll_pl             );
  gnss_cvt_ProtectionLevel_To_ApiProtectionLevel(&pz_src->z_pitch_pl           ,&pz_dst->z_pitch_pl            );
  gnss_cvt_ProtectionLevel_To_ApiProtectionLevel(&pz_src->z_yaw_pl             ,&pz_dst->z_yaw_pl              );
}

/**
 * @brief make loc_api_location_report_t
 * @param[in] pz_PositionFix_report position information from algorithm
 * @param[out] z_api_location_report location info for report
 */
void gnss_cvt_GnssPositionFix_To_LocApiPositionFix(const gnss_PositionFix_t* pz_PositionFix_report,
  loc_api_ConsoildatedPositionFix_t* pz_ConsoildatedPositionFix)
{
  float f_VelN =0.0f,f_VelE = 0.0f,f_heading = 0.0f;
  if ((NULL == pz_PositionFix_report) || (NULL == pz_ConsoildatedPositionFix))
  {
    LOGE(TAG_SM, "Null pointer input");
    return;
  }

  pz_ConsoildatedPositionFix->u_version = VERSION_CONSOILDATE_POSITION_FIX;
  pz_ConsoildatedPositionFix->w_size = sizeof(loc_api_ConsoildatedPositionFix_t);
  pz_ConsoildatedPositionFix->q_towMsec = pz_PositionFix_report->z_gpsTime.q_towMsec;
  pz_ConsoildatedPositionFix->w_week = pz_PositionFix_report->z_gpsTime.w_week;
  pz_ConsoildatedPositionFix->u_leapsec = pz_PositionFix_report->u_leapsec;
  pz_ConsoildatedPositionFix->u_DcpPosType = pz_PositionFix_report->u_DcpPosType;
  f_VelN = pz_PositionFix_report->f_velEnu[1];
  f_VelE = pz_PositionFix_report->f_velEnu[0];
  f_heading = atan2f(f_VelE, f_VelN) * (float)RAD2DEG;
  pz_ConsoildatedPositionFix->f_heading = f_heading < 0 ? 360 + f_heading : f_heading;
  pz_ConsoildatedPositionFix->f_speed = gnss_CovEnVel2Speed(f_VelE, f_VelN);

  switch (pz_PositionFix_report->u_fixSource)
  {
  case FIX_SOURCE_INVALID:
    pz_ConsoildatedPositionFix->u_fixSource = LOC_API_FIX_SOURCE_NONE;
    break;
  case FIX_SOURCE_PVT_WLS:
  case FIX_SOURCE_PVT_KF:
  case FIX_SOURCE_HPP_V1:
  case FIX_SOURCE_HPP_V2:
  case FIX_SOURCE_DCP:
  {
    pz_ConsoildatedPositionFix->u_fixSource = LOC_API_FIX_SOURCE_PVT;
    break;
  }
  case FIX_SOURCE_RTK:
  case FIX_SOURCE_RTD:
  case FIX_SOURCE_RTK_FLOAT:
  case FIX_SOURCE_RTK_FIX:
  {
    pz_ConsoildatedPositionFix->u_fixSource = LOC_API_FIX_SOURCE_RTK;
    break;
  }
  case FIX_SOURCE_PPP:
  case FIX_SOURCE_PPP_FLOAT:
  case FIX_SOURCE_PPP_FIX:
    pz_ConsoildatedPositionFix->u_fixSource = LOC_API_FIX_SOURCE_PPP;
    break;
  case FIX_SOURCE_VDR:
    pz_ConsoildatedPositionFix->u_fixSource = LOC_API_FIX_SOURCE_INS;
    pz_ConsoildatedPositionFix->f_heading = pz_PositionFix_report->z_ins.f_heading;
    /* INS related data */
    pz_ConsoildatedPositionFix->f_roll = pz_PositionFix_report->z_ins.f_roll;
    pz_ConsoildatedPositionFix->f_pitch = pz_PositionFix_report->z_ins.f_pitch;
    pz_ConsoildatedPositionFix->f_gyrobias_x = pz_PositionFix_report->z_ins.f_gyrobias_x;
    pz_ConsoildatedPositionFix->f_gyrobias_y = pz_PositionFix_report->z_ins.f_gyrobias_y;
    pz_ConsoildatedPositionFix->f_gyrobias_z = pz_PositionFix_report->z_ins.f_gyrobias_z;
    pz_ConsoildatedPositionFix->f_accbias_x = pz_PositionFix_report->z_ins.f_accbias_x;
    pz_ConsoildatedPositionFix->f_accbias_y = pz_PositionFix_report->z_ins.f_accbias_y;
    pz_ConsoildatedPositionFix->f_accbias_z = pz_PositionFix_report->z_ins.f_accbias_z;
    pz_ConsoildatedPositionFix->f_roll_std = pz_PositionFix_report->z_ins.f_roll_std;
    pz_ConsoildatedPositionFix->f_pitch_std = pz_PositionFix_report->z_ins.f_pitch_std;
    pz_ConsoildatedPositionFix->f_yaw_std = pz_PositionFix_report->z_ins.f_yaw_std;
    pz_ConsoildatedPositionFix->f_mis_roll = pz_PositionFix_report->z_ins.f_mis_roll;
    pz_ConsoildatedPositionFix->f_mis_pitch = pz_PositionFix_report->z_ins.f_mis_pitch;
    pz_ConsoildatedPositionFix->f_mis_yaw = pz_PositionFix_report->z_ins.f_mis_yaw;
    pz_ConsoildatedPositionFix->u_drposflag = pz_PositionFix_report->z_ins.u_drposflag;
    pz_ConsoildatedPositionFix->f_whlspd_sf[0] = pz_PositionFix_report->z_ins.f_whlspd_sf[0];
    pz_ConsoildatedPositionFix->f_whlspd_sf[1] = pz_PositionFix_report->z_ins.f_whlspd_sf[1];
    pz_ConsoildatedPositionFix->f_whlspd_sf[2] = pz_PositionFix_report->z_ins.f_whlspd_sf[2];
    pz_ConsoildatedPositionFix->f_whlspd_sf[3] = pz_PositionFix_report->z_ins.f_whlspd_sf[3];
    pz_ConsoildatedPositionFix->q_kfmeastype = pz_PositionFix_report->z_ins.q_kfmeastype;
    pz_ConsoildatedPositionFix->f_la_imu2gnss[0] = pz_PositionFix_report->z_ins.f_la_imu2gnss[0];
    pz_ConsoildatedPositionFix->f_la_imu2gnss[1] = pz_PositionFix_report->z_ins.f_la_imu2gnss[1];
    pz_ConsoildatedPositionFix->f_la_imu2gnss[2] = pz_PositionFix_report->z_ins.f_la_imu2gnss[2];
    pz_ConsoildatedPositionFix->f_la_imu2rearmid[0] = pz_PositionFix_report->z_ins.f_la_imu2rearmid[0];
    pz_ConsoildatedPositionFix->f_la_imu2rearmid[1] = pz_PositionFix_report->z_ins.f_la_imu2rearmid[1];
    pz_ConsoildatedPositionFix->f_la_imu2rearmid[2] = pz_PositionFix_report->z_ins.f_la_imu2rearmid[2];
    pz_ConsoildatedPositionFix->f_la_rear2rear = pz_PositionFix_report->z_ins.f_la_rear2rear;
    pz_ConsoildatedPositionFix->f_misdualant_roll = pz_PositionFix_report->z_ins.f_misdualant_roll;
    pz_ConsoildatedPositionFix->f_misdualant_pitch = pz_PositionFix_report->z_ins.f_misdualant_pitch;
    pz_ConsoildatedPositionFix->f_misdualant_yaw = pz_PositionFix_report->z_ins.f_misdualant_yaw;
    break;
  default:
    pz_ConsoildatedPositionFix->u_fixSource = LOC_API_FIX_SOURCE_NONE;
    break;
  }
  /* integrity protection level */
  gnss_cvt_Integrity_To_ApiIntegrity(&pz_PositionFix_report->z_integrity, &pz_ConsoildatedPositionFix->z_integity);

  pz_ConsoildatedPositionFix->u_fixFlag = pz_PositionFix_report->u_fixFlag;
  for (int i = 0; i < 3; i++)
  {
    pz_ConsoildatedPositionFix->d_xyz[i] = pz_PositionFix_report->d_xyz[i];
    pz_ConsoildatedPositionFix->d_lla[i] = pz_PositionFix_report->d_lla[i];
    pz_ConsoildatedPositionFix->f_velXyz[i] = pz_PositionFix_report->f_velXyz[i];
    pz_ConsoildatedPositionFix->f_velEnu[i] = pz_PositionFix_report->f_velEnu[i];
    pz_ConsoildatedPositionFix->f_posXyzUnc[i] = pz_PositionFix_report->f_posXyzUnc[i];
    pz_ConsoildatedPositionFix->f_posLlaUnc[i] = pz_PositionFix_report->f_posLlaUnc[i];
    pz_ConsoildatedPositionFix->f_velEnuUnc[i] = pz_PositionFix_report->f_velEnuUnc[i];
  }

  if (pz_ConsoildatedPositionFix->d_lla[0] > 180.0)
  {
    pz_ConsoildatedPositionFix->d_lla[0] -= 180.0;
  }
  else if (pz_ConsoildatedPositionFix->d_lla[0] < -180.0)
  {
    pz_ConsoildatedPositionFix->d_lla[0] += 180.0;
  }

  if (pz_ConsoildatedPositionFix->d_lla[1] > 180)
  {
    pz_ConsoildatedPositionFix->d_lla[1] -= 180.0;
  }
  else if (pz_ConsoildatedPositionFix->d_lla[1] < -180.0)
  {
    pz_ConsoildatedPositionFix->d_lla[1] += 180.0;
  }
  if(FIX_SOURCE_VDR != pz_PositionFix_report->u_fixSource){
    pz_ConsoildatedPositionFix->u_svUsed = pz_PositionFix_report->z_SvStatus.u_SvInUseCount;
    pz_ConsoildatedPositionFix->d_avgCN0 = pz_PositionFix_report->d_avgCN0;
    pz_ConsoildatedPositionFix->d_quasiGeoidHeight = pz_PositionFix_report->d_quasiGeoidHeight;
    pz_ConsoildatedPositionFix->d_clockBias = pz_PositionFix_report->d_clockBias;
    pz_ConsoildatedPositionFix->d_clockDrift = pz_PositionFix_report->d_clockDrift;
    pz_ConsoildatedPositionFix->f_gdop = pz_PositionFix_report->z_dops.f_gdop;
    pz_ConsoildatedPositionFix->f_pdop = pz_PositionFix_report->z_dops.f_pdop;
    pz_ConsoildatedPositionFix->f_hdop = pz_PositionFix_report->z_dops.f_hdop;
    pz_ConsoildatedPositionFix->f_vdop = pz_PositionFix_report->z_dops.f_vdop;
    pz_ConsoildatedPositionFix->f_tdop = pz_PositionFix_report->z_dops.f_tdop;
    pz_ConsoildatedPositionFix->q_StationID = pz_PositionFix_report->z_rtk.q_StationID;
    pz_ConsoildatedPositionFix->f_age = pz_PositionFix_report->z_rtk.f_age;
    memcpy(pz_ConsoildatedPositionFix->pd_StationCoordinate, pz_PositionFix_report->z_rtk.pd_StationCoordinate, sizeof(pz_PositionFix_report->z_rtk.pd_StationCoordinate));

    const gnss_PositionFix_SvStatus_t* pz_Src = &(pz_PositionFix_report->z_SvStatus);
    loc_api_SvStatus_t* pz_Dst = &(pz_ConsoildatedPositionFix->z_SvStatus);

    pz_Dst->u_MeasTrackCount = pz_Src->u_MeasTrackCount;
    pz_Dst->u_MeasInUseCount = pz_Src->u_MeasInUseCount;
    pz_Dst->u_SvTrackCount = pz_Src->u_SvTrackCount;
    pz_Dst->u_SvInUseCount = pz_Src->u_SvInUseCount;

    for (uint8_t k = 0; k < C_GNSS_MAX; k++)
    {
      loc_api_gnssConstellationType u_type = gnss_cvt_ConstellationTypeToLoc(k);

      if (LOC_API_GNSS_MAX == u_type)
      {
        continue;
      }

      pz_Dst->t_SvTrackMask[u_type] = pz_Src->t_SvTrackMask[k];
      pz_Dst->t_SvInUseMask[u_type] = pz_Src->t_SvInUseMask[k];
    }

    for (uint8_t k = 0; k < LOC_API_MAX_GNSS_TRK_MEAS_NUMBER; k++)
    {
      pz_Dst->z_SV[k].u_constellation = pz_Src->z_SV[k].u_constellation;
      pz_Dst->z_SV[k].u_svid = pz_Src->z_SV[k].u_svid;
      pz_Dst->z_SV[k].u_signal = pz_Src->z_SV[k].u_signal;
      pz_Dst->z_SV[k].f_cn0 = pz_Src->z_SV[k].f_cn0;
      pz_Dst->z_SV[k].f_elevation = pz_Src->z_SV[k].f_elevation;
      pz_Dst->z_SV[k].f_azimuth = pz_Src->z_SV[k].f_azimuth;
    }
  }
}

/**
 * @brief make loc_api_location_report_t
 * @param[in] z_api_location_report location info for report
 * @param[out] pz_PositionFix_report position information from algorithm
 */
void gnss_cvt_LocApiPositionFix_To_GnssPositionFix(const loc_api_ConsoildatedPositionFix_t* pz_ConsoildatedPositionFix,
  gnss_PositionFix_t* pz_PositionFix_report)
{
  if ((NULL == pz_PositionFix_report) || (NULL == pz_ConsoildatedPositionFix))
  {
    LOGE(TAG_SM, "Null pointer input");
    return;
  }

  pz_PositionFix_report->u_version = VERSION_POSITION_FIX;
  pz_PositionFix_report->w_size = sizeof(gnss_PositionFix_t);
  tm_cvt_SetGpst(&pz_PositionFix_report->z_gpsTime, pz_ConsoildatedPositionFix->w_week, 0.001 * pz_ConsoildatedPositionFix->q_towMsec);
  pz_PositionFix_report->u_leapsec = pz_ConsoildatedPositionFix->u_leapsec;
  pz_PositionFix_report->u_DcpPosType = pz_ConsoildatedPositionFix->u_DcpPosType;
  pz_PositionFix_report->u_fixFlag = pz_ConsoildatedPositionFix->u_fixFlag;
  pz_PositionFix_report->z_SvStatus.u_SvInUseCount = pz_ConsoildatedPositionFix->u_svUsed;
  pz_PositionFix_report->d_avgCN0 = pz_ConsoildatedPositionFix->d_avgCN0;
  pz_PositionFix_report->d_quasiGeoidHeight = pz_ConsoildatedPositionFix->d_quasiGeoidHeight;
  for (int i = 0; i < 3; i++)
  {
    pz_PositionFix_report->d_xyz[i] = pz_ConsoildatedPositionFix->d_xyz[i];
    pz_PositionFix_report->d_lla[i] = pz_ConsoildatedPositionFix->d_lla[i];
    pz_PositionFix_report->f_velXyz[i] = pz_ConsoildatedPositionFix->f_velXyz[i];
    pz_PositionFix_report->f_velEnu[i] = pz_ConsoildatedPositionFix->f_velEnu[i];
    pz_PositionFix_report->f_posXyzUnc[i] = pz_ConsoildatedPositionFix->f_posXyzUnc[i];
    pz_PositionFix_report->f_posLlaUnc[i] = pz_ConsoildatedPositionFix->f_posLlaUnc[i];
    pz_PositionFix_report->f_velEnuUnc[i] = pz_ConsoildatedPositionFix->f_velEnuUnc[i];
  }

  switch (pz_ConsoildatedPositionFix->u_fixSource)
  {
  case  LOC_API_FIX_SOURCE_NONE:
    pz_PositionFix_report->u_fixSource = FIX_SOURCE_INVALID;
    break;
  case LOC_API_FIX_SOURCE_PVT:
    pz_PositionFix_report->u_fixSource = FIX_SOURCE_HPP_V2;
    break;
  case LOC_API_FIX_SOURCE_RTK:
    pz_PositionFix_report->u_fixSource = FIX_SOURCE_RTK;
    break;
  case LOC_API_FIX_SOURCE_PPP:
    pz_PositionFix_report->u_fixSource = FIX_SOURCE_PPP;
    break;
  case LOC_API_FIX_SOURCE_INS:
    pz_PositionFix_report->u_fixSource = FIX_SOURCE_VDR;
    break;
  default:
    pz_PositionFix_report->u_fixSource = FIX_SOURCE_INVALID;
    break;
  }

  if (pz_PositionFix_report->d_lla[0] > 180.0)
  {
    pz_PositionFix_report->d_lla[0] -= 180.0;
  }
  else if (pz_PositionFix_report->d_lla[0] < -180.0)
  {
    pz_PositionFix_report->d_lla[0] += 180.0;
  }

  if (pz_PositionFix_report->d_lla[1] > 180)
  {
    pz_PositionFix_report->d_lla[1] -= 180.0;
  }
  else if (pz_PositionFix_report->d_lla[1] < -180.0)
  {
    pz_PositionFix_report->d_lla[1] += 180.0;
  }

  pz_PositionFix_report->d_clockBias = pz_ConsoildatedPositionFix->d_clockBias;
  pz_PositionFix_report->d_clockDrift = pz_ConsoildatedPositionFix->d_clockDrift;
  pz_PositionFix_report->z_dops.f_gdop = pz_ConsoildatedPositionFix->f_gdop;
  pz_PositionFix_report->z_dops.f_pdop = pz_ConsoildatedPositionFix->f_pdop;
  pz_PositionFix_report->z_dops.f_hdop = pz_ConsoildatedPositionFix->f_hdop;
  pz_PositionFix_report->z_dops.f_vdop = pz_ConsoildatedPositionFix->f_vdop;
  pz_PositionFix_report->z_dops.f_tdop = pz_ConsoildatedPositionFix->f_tdop;
  pz_PositionFix_report->z_rtk.q_StationID = pz_ConsoildatedPositionFix->q_StationID;
  pz_PositionFix_report->z_rtk.f_age = pz_ConsoildatedPositionFix->f_age;
  pz_PositionFix_report->z_ins.f_roll = pz_ConsoildatedPositionFix->f_roll;
  pz_PositionFix_report->z_ins.f_pitch = pz_ConsoildatedPositionFix->f_pitch;
  pz_PositionFix_report->z_ins.f_gyrobias_x = pz_ConsoildatedPositionFix->f_gyrobias_x;
  pz_PositionFix_report->z_ins.f_gyrobias_y = pz_ConsoildatedPositionFix->f_gyrobias_y;
  pz_PositionFix_report->z_ins.f_gyrobias_z = pz_ConsoildatedPositionFix->f_gyrobias_z;
  pz_PositionFix_report->z_ins.f_accbias_x = pz_ConsoildatedPositionFix->f_accbias_x;
  pz_PositionFix_report->z_ins.f_accbias_y = pz_ConsoildatedPositionFix->f_accbias_y;
  pz_PositionFix_report->z_ins.f_accbias_z = pz_ConsoildatedPositionFix->f_accbias_z;
  pz_PositionFix_report->z_ins.f_roll_std = pz_ConsoildatedPositionFix->f_roll_std;
  pz_PositionFix_report->z_ins.f_pitch_std = pz_ConsoildatedPositionFix->f_pitch_std;
  pz_PositionFix_report->z_ins.f_yaw_std = pz_ConsoildatedPositionFix->f_yaw_std;
  pz_PositionFix_report->z_ins.f_mis_roll = pz_ConsoildatedPositionFix->f_mis_roll;
  pz_PositionFix_report->z_ins.f_mis_pitch = pz_ConsoildatedPositionFix->f_mis_pitch;
  pz_PositionFix_report->z_ins.f_mis_yaw = pz_ConsoildatedPositionFix->f_mis_yaw;
  pz_PositionFix_report->z_ins.u_drposflag = pz_ConsoildatedPositionFix->u_drposflag;

  gnss_PositionFix_SvStatus_t* pz_Dst = &(pz_PositionFix_report->z_SvStatus);
  const loc_api_SvStatus_t* pz_Src = &(pz_ConsoildatedPositionFix->z_SvStatus);

  pz_Dst->u_MeasTrackCount = pz_Src->u_MeasTrackCount;
  pz_Dst->u_MeasInUseCount = pz_Src->u_MeasInUseCount;
  pz_Dst->u_SvTrackCount = pz_Src->u_SvTrackCount;
  pz_Dst->u_SvInUseCount = pz_Src->u_SvInUseCount;

  for (uint8_t k = 0; k < C_GNSS_MAX; k++)
  {
    loc_api_gnssConstellationType u_type = gnss_cvt_ConstellationTypeToLoc(k);

    if (LOC_API_GNSS_MAX == u_type)
    {
      continue;
    }

    pz_Dst->t_SvTrackMask[k] = pz_Src->t_SvTrackMask[u_type];
    pz_Dst->t_SvInUseMask[k] = pz_Src->t_SvInUseMask[u_type];
  }

  for (uint8_t k = 0; k < LOC_API_MAX_GNSS_TRK_MEAS_NUMBER; k++)
  {
    pz_Dst->z_SV[k].u_constellation = pz_Src->z_SV[k].u_constellation;
    pz_Dst->z_SV[k].u_svid = pz_Src->z_SV[k].u_svid;
    pz_Dst->z_SV[k].u_signal = pz_Src->z_SV[k].u_signal;
    pz_Dst->z_SV[k].f_cn0 = pz_Src->z_SV[k].f_cn0;
    pz_Dst->z_SV[k].f_elevation = pz_Src->z_SV[k].f_elevation;
    pz_Dst->z_SV[k].f_azimuth = pz_Src->z_SV[k].f_azimuth;
  }
}
/**
 * @brief identify the GNSS measure whether it is multi-frequency data that frequency number greater than 2
 * @param[in] pz_meas GNSS Measurement structure
 * @param[in] w_measCount number of the pz_meas
 * @return    TRUE: is multi-frequency data
 */
BOOL gnss_IdentifyMultiFreq(const GnssMeas_t* pz_meas, uint16_t w_measCount)
{
  BOOL q_isMultiFreqMeas = FALSE;
  uint8_t u_i = 0;
  uint16_t w_i = 0;
  gnss_FreqType u_freqType = C_GNSS_FREQ_TYPE_MAX;
  uint8_t pu_satNumPerFreq[C_GNSS_FREQ_TYPE_MAX] = { 0 };
  for (u_i = 0; u_i < C_GNSS_FREQ_TYPE_MAX; ++u_i)
  {
    pu_satNumPerFreq[u_i] = 0;
  }
  for (w_i = 0; w_i < w_measCount; ++w_i)
  {
    if (pz_meas[w_i].d_pseudoRange <= FABS_ZEROS)
    {
      continue;
    }
    else if (C_GNSS_GLO == pz_meas[w_i].u_constellation)
    {
      continue;
    }
    u_freqType = gnss_cvt_Sig2FreqType(pz_meas[w_i].u_signal);
    if (u_freqType >= C_GNSS_FREQ_TYPE_MAX)
    {
      continue;
    }
    ++pu_satNumPerFreq[u_freqType];
  }
  if (pu_satNumPerFreq[C_GNSS_FREQ_TYPE_L1] > 0 && pu_satNumPerFreq[C_GNSS_FREQ_TYPE_L2] > 0 && pu_satNumPerFreq[C_GNSS_FREQ_TYPE_L5] > 0)
  {
    q_isMultiFreqMeas = TRUE;
  }
  return q_isMultiFreqMeas;
}

/**
 * @brief      get heading & pitch, calculate their uncertainty
 * @param[in]  pd_mainAntXyz is coordinate of main antenna (ecef)
 * @param[in]  pd_auxiAntXyz is coordinate of auxi antenna (ecef)
 * @param[in]  pd_xyzUnc uncertainty of main-auxi-baseline (ecef)
 * @param[out] pz_ortResult is the result of orient that include heading,pitch and roll
 * @return     0 represent successs and other failture
 */
uint8_t gnss_getOrtHeadingPitch(const double* pd_mainAntXyz, const double* pd_auxiAntXyz, const double* pd_xyzUnc, gnss_headingPitchRoll_t* pz_ortResult)
{
  uint8_t u_i = 0;
  double d_2r = 0.0;
  double d_2rSquare = 0.0;
  double d_3rSquare = 0.0;
  double d_horizontalVar = 0.0;
  double pd_refLla[3] = { 0.0 };
  double pd_vectorM2A[3] = { 0.0 };
  double pd_enu[3] = { 0.0 };
  double pd_enuVar[3] = { 0.0 };
  double pd_E[9] = { 0.0 };
  double pd_part[3] = { 0.0 };

  pz_ortResult->f_heading = 0.0;
  pz_ortResult->f_pitch = 0.0;
  pz_ortResult->f_roll = 0.0;
  pz_ortResult->f_headingStd = 0.0;
  pz_ortResult->f_pitchStd = 0.0;
  pz_ortResult->f_rollStd = 0.0;

  if ((gnss_Dot(pd_mainAntXyz, pd_mainAntXyz, 3) < FABS_ZEROS) ||
    (gnss_Dot(pd_auxiAntXyz, pd_auxiAntXyz, 3) < FABS_ZEROS) ||
    (gnss_Dot(pd_xyzUnc, pd_xyzUnc, 3) < FABS_ZEROS))
  {
    return 1;
  }

  gnss_Ecef2Lla(pd_mainAntXyz, pd_refLla);
  for (u_i = 0; u_i < 3; u_i++)
  {
    pd_vectorM2A[u_i] = pd_auxiAntXyz[u_i] - pd_mainAntXyz[u_i];
  }

  gnss_Pos2EnuMat(pd_refLla, pd_E);
  gnss_MatrixMultiply("NN", 3, 1, 3, 1.0, pd_E, pd_vectorM2A, 0.0, pd_enu);
  /*for E variance*/
  pd_part[0] = pd_E[0] * pd_xyzUnc[0];
  pd_part[1] = pd_E[3] * pd_xyzUnc[1];
  pd_part[2] = pd_E[6] * pd_xyzUnc[2];
  pd_enuVar[0] = pd_part[0] * pd_part[0] + pd_part[1] * pd_part[1] + pd_part[2] * pd_part[2];
  /*for N variance*/
  pd_part[0] = pd_E[1] * pd_xyzUnc[0];
  pd_part[1] = pd_E[4] * pd_xyzUnc[1];
  pd_part[2] = pd_E[7] * pd_xyzUnc[2];
  pd_enuVar[1] = pd_part[0] * pd_part[0] + pd_part[1] * pd_part[1] + pd_part[2] * pd_part[2];
  /*for U variance*/
  pd_part[0] = pd_E[2] * pd_xyzUnc[0];
  pd_part[1] = pd_E[5] * pd_xyzUnc[1];
  pd_part[2] = pd_E[8] * pd_xyzUnc[2];
  pd_enuVar[2] = pd_part[0] * pd_part[0] + pd_part[1] * pd_part[1] + pd_part[2] * pd_part[2];

  d_2rSquare = pd_enu[0] * pd_enu[0] + pd_enu[1] * pd_enu[1];
  d_2r = sqrt(d_2rSquare);
  d_3rSquare = pd_enu[0] * pd_enu[0] + pd_enu[1] * pd_enu[1] + pd_enu[2] * pd_enu[2];

  if (gnss_Dot(pd_enu, pd_enu, 2) < 1e-12)
  {
    (pz_ortResult->f_heading) = 0.0f;
    pz_ortResult->f_headingStd = 360.0f;
  }
  else
  {
    (pz_ortResult->f_heading) = (float)(atan2(pd_enu[0], pd_enu[1]) * RAD2DEG);
    pz_ortResult->f_headingStd = (float)(sqrt((pd_enu[0] * pd_enu[0] * pd_enuVar[1] + pd_enu[1] * pd_enu[1] * pd_enuVar[0]) / (d_2rSquare * d_2rSquare)) * RAD2DEG);
  }
  if ((pz_ortResult->f_heading) < 0.0f)
  {
    (pz_ortResult->f_heading) += 360.0f;
  }

  if ((pd_enu[2] * pd_enu[2] + d_2rSquare) < 1e-12)
  {
    pz_ortResult->f_pitch = 0.0;
    pz_ortResult->f_pitchStd = 180.0f;
  }
  else
  {
    pz_ortResult->f_pitch = (float)(atan2(pd_enu[2], d_2r) * RAD2DEG);
    d_horizontalVar = (pd_enu[0] * pd_enu[0] * pd_enuVar[0] + pd_enu[1] * pd_enu[1] * pd_enuVar[1]) / (d_2rSquare);
    pz_ortResult->f_pitchStd = (float)(sqrt((pd_enu[2] * pd_enu[2] * d_horizontalVar + d_2rSquare * pd_enuVar[2]) / (d_3rSquare * d_3rSquare)) * RAD2DEG);
  }
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pz_ortResult->f_ENU[u_i] = (float)(pd_enu[u_i]);
    pz_ortResult->f_deltaXYZ[u_i] = (float)(pd_vectorM2A[u_i]);
  }
  return 0;
}
/**
  * @brief make gnss_OrientFix_t
  * @param[in] pz_locApiOrientReport location info for report
  * @param[out] pz_orientFix orient information from algorithm
  */
void gnss_cvt_LocApiOrientFix_To_GnssOrientFix(const loc_api_orient_report_t* pz_locApiOrientReport, gnss_OrientFix_t* pz_orientFix)
{
  uint8_t u_i = 0;
  if (NULL == pz_locApiOrientReport || NULL == pz_orientFix)
  {
    LOGE(TAG_SM, "Null pointer input");
    return;
  }
  pz_orientFix->u_version = VERSION_GNSS_ORT_FIX;
  pz_orientFix->w_size = sizeof(gnss_OrientFix_t);
  tm_cvt_SetGpst(&(pz_orientFix->z_gpsTime), pz_locApiOrientReport->w_week, pz_locApiOrientReport->d_tow);
  pz_orientFix->z_epoch.year = pz_locApiOrientReport->w_utcYear;
  pz_orientFix->z_epoch.month = pz_locApiOrientReport->w_utcMonth;
  pz_orientFix->z_epoch.day = pz_locApiOrientReport->w_utcDay;
  pz_orientFix->z_epoch.hour = pz_locApiOrientReport->w_utcHour;
  pz_orientFix->z_epoch.min = pz_locApiOrientReport->w_utcMinute;
   pz_orientFix->z_epoch.second = (float)(pz_locApiOrientReport->d_utcSecond);
  pz_orientFix->u_leapsec = pz_locApiOrientReport->u_leapsec;
  pz_orientFix->u_DcpPosType = GNSS_DCP_POS_INVALID;
  pz_orientFix->u_ortFixFlag = pz_locApiOrientReport->u_ortFixQuality;
  pz_orientFix->u_SvTrackCount = pz_locApiOrientReport->u_SvTrackCount;
  pz_orientFix->u_SvTrackCountEleThres = pz_locApiOrientReport->u_SvTrackCountEleThres;
  pz_orientFix->u_SvInUseCount = pz_locApiOrientReport->u_SvInUseCount;
  pz_orientFix->u_SvInUseCountMuliFreq = pz_locApiOrientReport->u_SvInUseCountMuliFreq;
  pz_orientFix->u_ortSolStatus = pz_locApiOrientReport->u_ortSolStatus;
  pz_orientFix->u_ortPosVelType = pz_locApiOrientReport->u_ortPosVelType;
  pz_orientFix->u_GPSsignalUsedMask = pz_locApiOrientReport->u_GPSsignalUsedMask;
  pz_orientFix->u_GLOsignalUsedMask = pz_locApiOrientReport->u_GLOsignalUsedMask;
  pz_orientFix->u_GalSignalUsedMask = pz_locApiOrientReport->u_GalSignalUsedMask;
  pz_orientFix->u_BDSsignalUsedMask = pz_locApiOrientReport->u_BDSsignalUsedMask;
  pz_orientFix->f_age = pz_locApiOrientReport->f_age;
  pz_orientFix->z_ortResult.f_heading= pz_locApiOrientReport->f_heading;
  pz_orientFix->z_ortResult.f_pitch = pz_locApiOrientReport->f_pitch;
  pz_orientFix->z_ortResult.f_roll = pz_locApiOrientReport->f_roll;
  pz_orientFix->z_ortResult.f_headingStd = pz_locApiOrientReport->f_headingStd;
  pz_orientFix->z_ortResult.f_pitchStd = pz_locApiOrientReport->f_pitchStd;
  pz_orientFix->z_ortResult.f_rollStd = pz_locApiOrientReport->f_rollStd;
  pz_orientFix->z_mainAntInfo.u_posFixFlag = pz_locApiOrientReport->u_posFixQuality;
  pz_orientFix->z_mainAntInfo.u_SvInUseCount = pz_locApiOrientReport->u_mainAntSvInUseCount;
  for (u_i = 0; u_i < 3; ++u_i)
  {
    pz_orientFix->z_ortResult.f_ENU[u_i] = pz_locApiOrientReport->f_ENU[u_i];
    pz_orientFix->z_mainAntInfo.d_xyz[u_i] = pz_locApiOrientReport->d_mainAntXyz[u_i];
    pz_orientFix->z_mainAntInfo.d_lla[u_i] = pz_locApiOrientReport->d_mainAntLLA[u_i];
    pz_orientFix->z_mainAntInfo.f_velXyz[u_i] = pz_locApiOrientReport->f_mainAntVelXyz[u_i];
    pz_orientFix->z_mainAntInfo.f_velEnu[u_i] = pz_locApiOrientReport->f_mainAntVelEnu[u_i];
    pz_orientFix->z_mainAntInfo.pd_refStationCoordinate[u_i] = pz_locApiOrientReport->pd_refStationCoordinate[u_i];
  }
  return;
}

/**
 * @brief fill up fix flag and the type of position and velocity for orient
 * @param[in]  u_fixFlag is the type of fix falg
 * @return     orient type of position and velocity
 */
gnss_orientPosVelType gnss_cvt_ortFixFlag2PosVelType(gnss_FixFlagType u_fixFlag)
{
  gnss_orientPosVelType u_posVelType = C_GNSS_ORT_POS_VEL_TYPE_NONE;
  if (GNSS_FIX_FLAG_SPS == u_fixFlag)
  {
    u_posVelType = C_GNSS_ORT_POS_VEL_TYPE_SINGLE;
  }
  else if (GNSS_FIX_FLAG_DGNSS == u_fixFlag)
  {
    u_posVelType = C_GNSS_ORT_POS_VEL_TYPE_PSRDIFF;
  }
  else if (GNSS_FIX_FLAG_FIXED == u_fixFlag)
  {
    u_posVelType = C_GNSS_ORT_POS_VEL_TYPE_L1_INT;
  }
  else if (GNSS_FIX_FLAG_FLOATING == u_fixFlag)
  {
    u_posVelType = C_GNSS_ORT_POS_VEL_TYPE_L1_FLOAT;
  }
  return u_posVelType;
}

uint8_t gnss_CvtConstellation(uint8_t u_Constl, uint8_t u_Svid)
{
  uint8_t out = C_GNSS_NONE;
  switch (u_Constl)
  {
  case C_GNSS_GPS:
    out = C_GNSS_GPS;
    break;
  case C_GNSS_GLO:
    out = C_GNSS_GLO;
    break;
  case C_GNSS_BDS3:
    if (u_Svid > 16)
    {
      out = C_GNSS_BDS3;
    }
    else
    {
      out = C_GNSS_BDS3;
    }
    break;
  case C_GNSS_GAL:
    out = C_GNSS_GAL;
    break;
  case C_GNSS_QZS:
    out = C_GNSS_QZS;
    break;
  case C_GNSS_MAX:
    out = C_GNSS_NONE;
    break;
  default:
    out = C_GNSS_NONE;
    break;
  }

  return out;
}

/**
 * @brief whether skip C_GNSS_BDS2 variable in a loop
 * @param[in]  q_isSperateBDS2And3 whether sperate for BDS2 and BDS3
 * @param[in]  u_constellation is the type of constellation
 * @return     TRUE represent skip C_GNSS_BDS2 variable in a loop, and FALSE represent do not skip C_GNSS_BDS2 variable in a loop
 */
BOOL gnss_WhetherSkipBDS2VarInLoop(BOOL q_isSperateBDS2And3, gnss_ConstellationType u_constellation)
{
  BOOL q_skipBDS2 = FALSE;
  if (FALSE == q_isSperateBDS2And3 && C_GNSS_BDS2 == u_constellation)
  {
    q_skipBDS2 = TRUE;
  }
  return q_skipBDS2;
}

/**
 * @brief get the contellation enum value in a loop
 * @param[in]  q_isSperateBDS2And3 whether sperate for BDS2 and BDS3
 * @param[in]  u_constellation is the type of constellation
 * @return     the contellation enum value in a loop
 */
gnss_ConstellationType gnss_getConstellationEnumValueInLoop(BOOL q_isSperateBDS2And3, gnss_ConstellationType u_constellation)
{
  gnss_ConstellationType u_constellationEnumInLoop = u_constellation;
  if (FALSE == q_isSperateBDS2And3 && C_GNSS_BDS2 == u_constellation)
  {
    u_constellationEnumInLoop = C_GNSS_BDS3;
  }
  return u_constellationEnumInLoop;
}

/**
 * @brief according the signal type to determine whether BDS2 and BDS3 are separate for gnss_SatSigMeasCollect_t struct
 * @param[in]  z_targetFreq is the target frequency
 * @param[in]  pz_satSigMeasCollect is the observation information
 * @return     TRUE represent BDS2 and BDS3 will be separated and FALSE represent BDS2 and BDS3 will not be separated
 */
BOOL gnss_isSeparateBDS2And3ForSatSignalCollect(gnss_FreqType z_targetFreq, const gnss_SatSigMeasCollect_t* pz_satSigMeasCollect)
{
  BOOL q_isSeparateBDS2And3 = FALSE;
  BOOL q_isBDS3 = FALSE;
  uint32_t q_satIndex = 0;
  uint8_t u_signalIndex = 0;
  gnss_FreqType e_curFreqType = C_GNSS_FREQ_TYPE_MAX;
  gnss_SignalType u_BDS2SignalType = C_GNSS_SIG_MAX;
  gnss_SignalType u_BDS3SignalType = C_GNSS_SIG_MAX;
  const gnss_SatelliteMeas_t* pz_satMeas = NULL;
  const gnss_SignalMeas_t* pz_sigMeas = NULL;
  for (q_satIndex = 0; q_satIndex < ALL_GNSS_SYS_SV_NUMBER; ++q_satIndex)
  {
    if (C_GNSS_SIG_MAX != u_BDS2SignalType && C_GNSS_SIG_MAX != u_BDS3SignalType)
    {
      break;
    }
    pz_satMeas = pz_satSigMeasCollect->pz_satMeas[q_satIndex];
    if (NULL == pz_satMeas)
    {
      continue;
    }
    //only for BDS constellation
    if (C_GNSS_BDS2 != (pz_satMeas->u_constellation) && C_GNSS_BDS3 != (pz_satMeas->u_constellation))
    {
      continue;
    }
    for (u_signalIndex = 0; u_signalIndex < MAX_GNSS_SIGNAL_FREQ; ++u_signalIndex)
    {
      if (C_GNSS_SIG_MAX != u_BDS2SignalType && C_GNSS_SIG_MAX != u_BDS3SignalType)
      {
        break;
      }
      pz_sigMeas = pz_satMeas->pz_signalMeas[u_signalIndex];
      if (NULL == pz_sigMeas)
      {
        continue;
      }
      e_curFreqType = gnss_cvt_Sig2FreqType(pz_sigMeas->u_signal);
      if (e_curFreqType >= C_GNSS_FREQ_TYPE_MAX || e_curFreqType != z_targetFreq)
      {
        continue;
      }
      q_isBDS3 = gnss_isBDS3Sat(pz_sigMeas->u_svid, pz_sigMeas->u_constellation);
      if (TRUE == q_isBDS3)
      {
        u_BDS3SignalType = pz_sigMeas->u_signal;
      }
      else
      {
        u_BDS2SignalType = pz_sigMeas->u_signal;
      }
    }
  }
  if (C_GNSS_SIG_MAX != u_BDS2SignalType && C_GNSS_SIG_MAX != u_BDS3SignalType && u_BDS2SignalType != u_BDS3SignalType)
  {
    q_isSeparateBDS2And3 = TRUE;
  }
  return q_isSeparateBDS2And3;
}

/**
 * @brief  generate integrity flag, AL default 3.0 m
 * @param[in] f_protection_level >=0
 * @param[in] f_al >=0
 * @return
 */
gnss_integrityFlag gnss_genIntegrityFlag(float f_protection_level, float f_al)
{
  gnss_integrityFlag u_flag = GNSS_INTEGRITY_FLAG_MONITORED_OK;

  if (f_protection_level <= FABS_ZEROS)
  {
    u_flag = GNSS_INTEGRITY_FLAG_NOT_MONITORED;
  }
  else if (f_protection_level > f_al)
  {
    u_flag = GNSS_INTEGRITY_FLAG_MONITORED_FAIL;
  }
  else
  {
    u_flag= GNSS_INTEGRITY_FLAG_MONITORED_OK;
  }
  return u_flag;
}

/**
 * @brief generate gnss_ProtectionLevel_t by PL and AL
 * @param[in] f_protection_level
 * @return gnss_ProtectionLevel_t that include pl and flag
 */
gnss_ProtectionLevel_t gnss_genProtectionLevelStruct(float f_protection_level)
{
  const loc_ConfigParamGroup_t* pz_ConfigParamGrp = loc_cfg_getConfigParamGroup();
  float f_al = pz_ConfigParamGrp->sm_cfg.f_al;

  gnss_ProtectionLevel_t z_pl = {0};
  z_pl.f_protection_level = f_protection_level;
  z_pl.u_flag = gnss_genIntegrityFlag(f_protection_level, f_al);
  return z_pl;
}

/**
 * @brief Initialize gnss_Integrity_t
 * @param[in,out] pz_itg integrity in multiple coordinate system
 */
void gnss_InitIntegrityStruct(gnss_Integrity_t* pz_itg)
{
  pz_itg->z_pos_longitudinal_pl.u_flag = GNSS_INTEGRITY_FLAG_NOT_MONITORED;
  pz_itg->z_pos_lateral_pl.u_flag = GNSS_INTEGRITY_FLAG_NOT_MONITORED;
  pz_itg->z_pos_north_pl.u_flag = GNSS_INTEGRITY_FLAG_NOT_MONITORED;
  pz_itg->z_pos_east_pl.u_flag = GNSS_INTEGRITY_FLAG_NOT_MONITORED;
  pz_itg->z_pos_hor_pl.u_flag = GNSS_INTEGRITY_FLAG_NOT_MONITORED;
  pz_itg->z_pos_ver_pl.u_flag = GNSS_INTEGRITY_FLAG_NOT_MONITORED;
  pz_itg->z_vel_longitudinal_pl.u_flag = GNSS_INTEGRITY_FLAG_NOT_MONITORED;
  pz_itg->z_vel_lateral_pl.u_flag = GNSS_INTEGRITY_FLAG_NOT_MONITORED;
  pz_itg->z_vel_north_pl.u_flag = GNSS_INTEGRITY_FLAG_NOT_MONITORED;
  pz_itg->z_vel_east_pl.u_flag = GNSS_INTEGRITY_FLAG_NOT_MONITORED;
  pz_itg->z_vel_hor_pl.u_flag = GNSS_INTEGRITY_FLAG_NOT_MONITORED;
  pz_itg->z_vel_ver_pl.u_flag = GNSS_INTEGRITY_FLAG_NOT_MONITORED;
  pz_itg->z_roll_pl.u_flag = GNSS_INTEGRITY_FLAG_NOT_MONITORED;
  pz_itg->z_pitch_pl.u_flag = GNSS_INTEGRITY_FLAG_NOT_MONITORED;
  pz_itg->z_yaw_pl.u_flag = GNSS_INTEGRITY_FLAG_NOT_MONITORED;
}