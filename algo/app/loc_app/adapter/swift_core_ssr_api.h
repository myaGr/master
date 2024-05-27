/**@file        swift_core_ssr_api.h
 * @brief       ssr data of swift convert functions
 *              1. swift SSR data struct to Asensing SSR struct
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/11/20  <td>0.1      <td>liuguo      <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __SWIFT_CORE_SSR_API_H__
#define __SWIFT_CORE_SSR_API_H__

//#define APP_SWIFT_SSR

#if defined(APP_SWIFT_SSR)

#include "loc_core_ssr_api.h"

// initialize adapter
void swift_adapter_init();

void SwiftSdk_StartUp();

/**
 * @brief Ntrip client recv callback
 * @param[in]
 * @return      None
 */
void ntrip_client_callback_receive_data_handler(uint8_t* buf, uint32_t len);


uint64_t SwiftSdk_GetSSRSum_len(void);

/**
  * @brief sdk running of swift
  * @param[in] pz_GnssSatPosBlk - information of satllites
  * @return    none
  */
void swift_sdk_runing(const loc_api_GnssMeasSatBlock_t * pz_GnssSatPosBlk);

#endif

#endif // APP_SWIFT_SSR
