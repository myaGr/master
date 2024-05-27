/**@file        ipc_parse.h
 * @brief       
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/10/11  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __IPC_PARSE_H__
#define __IPC_PARSE_H__

#include "cmn_def.h"
#include "gnss_type.h"
#include "mw_ipctask.h"
#include "loc_core_report.h"

BEGIN_DECL

/**
 * @brief Configurate IPC parse module output file
 * @param[in]   dir   - output file path
 * @param[in]   fname - output file name
 * @return      None
 */
void ipc_parse_config_output_file(char* drive, char* dir, char* prefix, char* fname);

/**
 * @brief Release, close file
 *
*/
void ipc_parse_release();

/**
 * @brief Parse a IPC message
 * @param[in]   pz_ipc   - pointer to IPC message
 * @param[out]  parse_output - output parsed string
 * @return      None
 */
uint32_t ipc_parse_execute(ipc_t* pz_ipc, char* parse_output);

/**
 * @brief Parse a package message
 *
 *    Package format
 * | 1Byte 1Byte | 2Byte    4Byte |  2Byte | Package data length | 2Byte   |
 * | SYNC0 SYNC1 | LOG_TYPE LENGTH| Pkg ID | Package data        | CHECKSUM|
 * |             | ---------Check Sum Compute data-------------- |         |
 *
 * @param[in]   id      - IPC id
 * @param[in]   data    - IPC data buffer
 * @param[in]   length  - IPC data length
 * @param[out]  parse_output - output parsed string
 * @return      None
 */
uint32_t package_parse_execute(uint8_t* p_data, uint32_t length, char* parse_output);

/**
 * @brief Get config info parse success flag
 * @param[in]   None
 * @param[out]  None
 * @return      parse success flag
 */
uint8_t get_monitor_flag();

/**
 * @brief Get config info
 * @param[in]   None
 * @param[out]  None
 * @return      config info
 */
loc_MonitorStructType* get_monitor_info();

END_DECL

#endif