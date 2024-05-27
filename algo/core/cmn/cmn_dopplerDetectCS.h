/**@file        cmn_dopplerDetectCS.h
 * @brief       using the doppler observations to detect the cycle slip of carrier phase
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/02/16  <td>0.1      <td>chenjinhe   <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#ifndef __CMN_DOPPLER_DETECT_CS_H__
#define __CMN_DOPPLER_DETECT_CS_H__
#include "cmn_def.h"
#include "gnss_type.h"

BEGIN_DECL
typedef struct
{
  GpsTime_t          z_obsTime;                             /*observation time*/
  uint16_t           w_measNum;                             /*the number of observations*/
  GnssMeas_t         pz_meas[MAX_GNSS_TRK_MEAS_NUMBER];     /*observations*/
  GnssMeasExt_t      pz_measExt[MAX_GNSS_TRK_MEAS_NUMBER];  /*extern observations*/
} gnss_dopplerDetectBlock_t;

typedef struct
{
  BOOL                       q_isSeparateBDS2And3;/*whether BDS2 and BDS3 are separate*/
  gnss_dopplerDetectBlock_t* pz_preBlock;         /*the observation information used by doppler detect cycle slip for previous epoch*/
  gnss_dopplerDetectBlock_t* pz_curBlock;         /*the observation information used by doppler detect cycle slip for current epoch*/
}gnss_dopplerDetectPairingBlock_t;

/**
 * @brief initialize the module of using doppler observations to detect cycle slip
 * @param[in/out] pz_dopplerDetectPairingBlock is the observation information used by doppler detect cycle slip
 * @return void
 */
void cmn_initDopplerDetectCyleSlip(gnss_dopplerDetectPairingBlock_t* pz_dopplerDetectPairingBlock);
/**
 * @brief de-initialize the module of using doppler observations to detect cycle slip
 * @param[in/out] pz_dopplerDetectPairingBlock is the observation information used by doppler detect cycle slip
 * @return void
 */
void cmn_deinitDopplerDetectCyleSlip(gnss_dopplerDetectPairingBlock_t* pz_dopplerDetectPairingBlock);
 /**
  * @brief using doppler observations to detect cycle slip
  * @param[in/out] pz_satSignalCur is the observation information
  * @param[in/out] pz_dopplerDetectPairingBlock is the observation information used by doppler detect cycle slip
  * @return the delta time between current epoch and previous epoch 
  */
double cmn_dopplerDetectCycleSlip(gnss_SatSigMeasCollect_t* pz_satSignalCur, gnss_dopplerDetectPairingBlock_t* pz_dopplerDetectPairingBlock);
END_DECL
#endif