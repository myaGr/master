/**@file        tsx_adapter_integration.h
 * @brief       TSX adapter 
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/11/07  <td>0.1      <td>houxiaowei   <td>Init version
 * </table>
 *
 **********************************************************************************
 */


#ifndef __TSX_ADAPTER_INTEGRATION_H__
#define __TSX_ADAPTER_INTEGRATION_H__

#include "cmn_def.h"

BEGIN_DECL

#if defined(APP_HEXAGON_SSR)
#include "tsxadapter.h"
#include "tsxadapter_types.h"
#include "tsxadapter_corrections_types.h"
#include "loc_core_api.h"

/**
 * @brief Init
 * @param[in] file_name DCF
 * @return uint8_t 0:success, others: failed
 */
extern uint8_t tsx_adapter_init(const char *file_name);

/**
 * @brief  Log tsx version
 * @return 0: success, others: failed
 */
extern uint8_t tsx_print_version();

/**
 * @brief SSR get LOS by call TSX adapter and inject to Algo
 * @param[in] pz_LocGnssMeasBlk
 * @param[in,out] pz_SsrCorBlk
 */
extern void tsx_inject_ApiGnssMeasSat(const loc_api_GnssMeasSatBlock_t *pz_ApiGnssMeasSatBlk);

/**
 * @brief TSX inject navigation
 * @param[in] pz_ApiNavData 
 */
extern void tsx_inject_navigation_data(loc_api_NavigationData_t *pz_ApiNavData);

/**
 * @brief TSX inject correction data
 * @param[in] q_length 
 * @param[in] buffer 
 * @return int32_t status: see TSXAdapterResult
 */
extern int32_t tsx_injectCorrectionData(uint32_t q_length, const uint8_t* buffer);

#endif // APP_HEXAGON_SSR

END_DECL
#endif //__TSX_ADAPTER_INTEGRATION_H__