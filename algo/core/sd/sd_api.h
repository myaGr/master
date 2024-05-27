/**@file        sd_api.h
 * @brief       Satellite Datebase Process API
 * @details     1. Ephemeris/Almanac calculate
 *              2. SSR data calculate
 * @author      caizhijie
 * @date        2022/04/26
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/04/26  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __SD_API_H__
#define __SD_API_H__

#include "gnss_type.h"
#include "mw_ipctask.h"

BEGIN_DECL

#define VERSION_SAT_POS_VEL_CLK_REQ  (0)
typedef struct {
  uint8_t            u_version;
  GpsTime_t          z_ReqTime;                 /** satellite transmit time */
  gnss_PositionFix_t z_positionFix;
  uint64_t           t_gnssReqMask[C_GNSS_MAX];
} sd_SatPosVelClkRequest_t;

/**
 * @brief SD IPC main proc function
 * @param[in]   p_ipc - pointer to IPC message
 * @return      None
 */
void sd_Proc(ipc_t* p_ipc);

/**
 * @brief SD IPC process task
 * @param[in]   args - task configuration
 * @return      None
 */
void* sd_task(void* args);

/**
 * @brief SD API to initilization
 * @return      None
 */
BOOL sd_api_Init();

/**
 * @brief SD API to start
 * @return      None
 */
BOOL sd_api_Start();

/**
 * @brief SD API to stop
 * @return      None
 */
BOOL sd_api_Stop();

/**
 * @brief SD API to release
 * @return      None
 */
BOOL sd_api_Release();

/**
 * @brief SD API Put Ephemeris
 * @return      None
 */
BOOL sd_api_Ephemeris_Put(gnss_Ephemeris_t* eph);

/**
 * @brief SD API to request compute satellite PVT information
 * @param[in]   pz_SatPvtReq
 * @return      None
 */
void sd_api_SatPosVelClkPolynomial_Request(sd_SatPosVelClkRequest_t* pz_SatPvtReq);

/**
 * @brief Get satellite ephemeris
 * @param[in] pz_tot signal time of transmit
 * @param[in] u_constellation system
 * @param[in] u_svid satellite id
 * @param[in] q_iode if is -1, don`t match the iode in ephemeris
 * @param[out] pz_eph ephemeris
 * @return
 */
BOOL sd_api_GnssEphemeris_Get(const GpsTime_t* pz_tot, uint8_t u_constellation, uint8_t u_svid, int32_t q_iode,
                              gnss_Ephemeris_t* pz_eph);

/**
 * @brief
 * @param[in] pz_tot signal time of transmit, =TOR-P/c, no clk correction
 * @param[in] u_constellation system
 * @param[in] u_svid satellite number
 * @param[in] q_iode if is -1, don`t match the iode in ephemeris
 * @param[in] u_clkCor TRUE/FALSE, TRUE: correct satellite clock, FALSE: don`t
 * @param[out] pz_satPosVelClk
 * @return false: failed. true: sucess
 */
BOOL sd_api_SatPositionFromBrdc(const GpsTime_t* pz_tot, uint8_t u_constellation, uint8_t u_svid, int32_t q_iode,
                                BOOL u_clkCor, gnss_SatPosVelClk_t* pz_satPosVelClk);

/**
 * @brief Get satellite Position, Velocity and Clock
 * @param[in]   constellation
 * @param[in]   svid
 * @param[in]   tot by GPST
 * @param[out]  pz_SatPosVelClk
 * @Note  This function has a mutex to support multi-task call.
 */
BOOL sd_api_SatPosVelClk_Get(uint8_t u_constellation, uint8_t u_svid, GpsTime_t* pz_tot,
  gnss_SatPosVelClk_t* pz_SatPosVelClk);

/**
 * @brief delete expire ephemeris
 * @param gpst GPST
 */
void sd_Ephemeris_refresh(const GpsTime_t* gpst);

END_DECL

#endif