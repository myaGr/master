/**********************************************************************************
* @note
* @par History :
*<table>
* <tr><th>Date        <th>Version  <th>Author       <th>Description
* <tr><td>2023/06/01  <td> 0.1     <td>ZhangXuecheng<td>Init version
* < / table>
*
**********************************************************************************
*/
#ifndef __CMN_MP_DETECT_H__
#define __CMN_MP_DETECT_H__
#include "cmn_def.h"
#include "gnss_type.h"
BEGIN_DECL

#define OBS_MP_LEVEL_UNKNOWEN  0  /*multipath is unknowen*/
#define OBS_MP_LEVEL_NORMAL    1  /*multipath is not significant*/
#define OBS_MP_LEVEL_LOW       2  /*multipath is in low level*/
#define OBS_MP_LEVEL_HIGH      3  /*multipath is in high level*/

/*the obs's multipath information which detected by some obs combines, 
   the combination observations include CMC, GIF and SNR combination */
typedef struct
{
  uint16_t   pw_CMCepochNum[MAX_GNSS_SIGNAL_FREQ];/*epoch number of CMC average arc*/
  uint16_t   w_GIFepochNum;/*epoch number of GIF average arc*/
  uint16_t   w_SNRepochNum;/*epoch number of SNR combination average arc*/
  /*reset arc when 3 slips occur in 4 epochs*/
  uint8_t   pu_CMCslipCount[MAX_GNSS_SIGNAL_FREQ];/*slip count of CMC average arc*/
  uint8_t   u_GIFslipCount;/*slip count of GIF average arc*/
  uint8_t   u_SNRslipCount;/*slip count of SNR combination average arc*/
  uint8_t   pu_CMCmultipathLevel[MAX_GNSS_SIGNAL_FREQ];/*MP level of CMC method*/
  uint8_t   u_GIFmultipathLevel;/*MP level of GIF method*/
  uint8_t   pu_SNRmultipathLevel[MAX_GNSS_SIGNAL_FREQ];/*MP level of SNR method*/
  float     f_eleA;/*the elevation of satellite*/
  /*the value of CMC(code minus carrier) combination observations: L1-L2 L2-L1 L5-L1*/
  double    pd_CMCvalue[MAX_GNSS_SIGNAL_FREQ];
  double    d_GIFvalue;/*the value of GIF(geo-iono free) combination observations*/
  double    d_SNRcombineValue;/*the value of SNR combination observations*/
  double    pd_CMCvalueSTD[MAX_GNSS_SIGNAL_FREQ];/*the value's std of CMC combination observations*/
  double    d_GIFvalueSTD;/*the value's std of GIF(geo-iono free) combination observations*/
  double    d_SNRcombineValueSTD;/*the value's std of SNR combination observations*/
  GpsTime_t pz_CMCendTime[MAX_GNSS_SIGNAL_FREQ];/*the end time of GF or MW combination observations*/
  GpsTime_t z_GIFendTime;/*the end time of GF or MW combination observations*/
  GpsTime_t z_SNRendTime;/*the end time of GF or MW combination observations*/
}cmn_oneSatMultipathDetectInfo;

// the obs multipath detecter set
typedef struct
{
  float f_sample;  /* background process sample */
  double* pd_ele_snr_bound_upper;  /* upper bound of normal ele-snr distribution */
  double* pd_ele_snr_bound_lower;  /* lower bound of normal ele-snr distribution */
  cmn_oneSatMultipathDetectInfo* pz_multiPathSet[ALL_GNSS_SYS_SV_NUMBER];
}cmn_multipathDetectSet;

/**
 * @brief initilize the multipath detecter
 * @param[out]  pz_multiPathDetecter the obs's multipath information which detected by some obs combines
 * @return      void
 */
void cmn_initMPdetectorInfo(cmn_oneSatMultipathDetectInfo* pz_multiPathDetecter);

/**
 * @brief update flag of the multipath detecter between epoch
 * @param[out]  pz_MPdetecterSet is the obs multipath detecter set
 * @return      void
 */
void cmn_epochUpdateMPdetectorInfo(cmn_multipathDetectSet* pz_MPdetecterSet);

/**
 * @brief the inteface of deinitilizing the MP detecter information
 * @param[out]  pz_MPdetecterSet is the obs multipath detecter set
 * @return     void
 */
void cmn_deinitMPdetectorSet(cmn_multipathDetectSet* pz_MPdetecterSet);

/**
 * @brief the inteface of initilizing the MP detecter information
 * @param[out]  pz_MPdetecterSet is the obs multipath detecter set
 * @param[in]  f_sample background process sample
 * @return     void
 */
void cmn_initMPdetectorSet(cmn_multipathDetectSet* pz_MPdetecterSet, float f_sample);

/**
 * @brief the inteface of detect multipath
 * @param[out]  pz_satSignalCur is the observation information
 * @param[out]  pz_multiPathDetecter the obs's multipath information which detected by some obs combines
 * @param[out]  pz_RTKmultiPathFlag the Multipath flag, see MULTIPATH_FLAG_...
 * @return     1 represent success and 0 failure
 */
uint8_t cmn_detectMultiPath(gnss_SatSigMeasCollect_t* pz_satSignalCur, cmn_multipathDetectSet* pz_MPdetecterSet,
  GnssMultiPathFlag_t** pz_RTKmultiPathFlag, uint8_t pu_sigMeasCount[C_GNSS_MAX][C_GNSS_FREQ_TYPE_MAX], uint8_t* pu_threeFreqTag);

END_DECL
#endif