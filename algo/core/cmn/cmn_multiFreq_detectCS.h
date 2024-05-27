/**@file        cmn_multiFreq_detectCS.h
 * @brief       the method of detecting cycle slip using multi-frequency observations
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/02/20  <td>0.1      <td>            <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#ifndef __CMN_MULTI_FREQ_DETECT_CS_H__
#define __CMN_MULTI_FREQ_DETECT_CS_H__
#include "cmn_def.h"
#include "gnss_type.h"
BEGIN_DECL
typedef struct
{
	double				d_GIFvalue;
	double				d_GFL1L2value;
	double        d_GFL2L5value;
	double        d_GIFsigma;
	double        d_GFL1L2sigma;
	double        d_GFL2L5sigma;
	double        d_alpha;
	double        d_beta;
	double        d_gamma;
}CalMultiFreqCombineResultInfo_t;   /*Result used by the current ephemeris in the multi-frequency slip detection*/
typedef struct
{
	uint8_t       u_countEpoch;						  /*the mask of whether is first epoch or second epoch*/
	GpsTime_t     z_obsTimeBeg;                             /* begain observation time*/
	GpsTime_t     z_obsTimePre;                             /* privious observation time*/
	double        d_GIFValue;							  /*the value of GIF combination observations*/
	double        d_GF_L2L5Value[2];				      /*the value of GF_L2_L5 combination observations of the last two epoach*/
	double        d_GF_L1L2Value[2];			          /*the value of GF_L1_L2 combination observations of the last two epoach*/
} cmn_oneObsmultiFreqDetectInfo;
typedef struct
{
	cmn_oneObsmultiFreqDetectInfo* pz_amultiFreqSet[ALL_GNSS_SYS_SV_NUMBER];			  /*the  multi - frequency observation information used to  detect cycle slip */
}cmn_multiFreqDetectSet;
/**
 * @brief  initilizing the inteface which inculde the information of using multi-frequency  to detect cycle slip
 * @param[in] pz_MLFreqDetectSet is the  multi-frequencyobservation information used to  detect cycle slip
 * @return
 */
void cmn_initMultiFreqDetectCyleSlip(cmn_multiFreqDetectSet* pz_MLFreqDetectSet);
/**
 * @brief de-initilizing the inteface which inculde the information of using multi-frequency  to detect cycle slip
 * @param[in] pz_MLFreqDetectSet is the  multi-frequencyobservation information used to  detect cycle slip
 * @return none
 */
void cmn_deinitmultiFreqDetectCyleSlip(cmn_multiFreqDetectSet* pz_MLFreqDetectSet);
/**
 * @brief the inteface of using multi-frequency method to detect cycle slip
 * @param[in]  pz_satSignalCur is the observation information
 * @param[in] pz_MLFreqDetectSet is the  multi-frequency observation information used to  detect cycle slip
 * @return
 */
uint8_t cmn_detectCycleSlipUsingMultiFreqObs(gnss_SatSigMeasCollect_t* pz_satSignalCur, cmn_multiFreqDetectSet* pz_MLFreqDetectSet);
END_DECL
#endif