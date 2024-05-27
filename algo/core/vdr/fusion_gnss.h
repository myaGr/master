/**@file        fusion_gnss.h
 * @brief		handle gnss measurement header file
 * @version     V1.0
 * @copyright   Copyright (c) 2022-2022  Guangzhou Asensing Techology Co.,Ltd.
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/01/21  <td>0.1      <td>shaobing   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __FUSION_GNSS_H__
#define __FUSION_GNSS_H__

#include "fusion_global.h"
#include "fusion_api.h"
#include <stdint.h>

#define GNSS_BUF_SIZE 4

typedef struct
{
  double d_gnss_meas[UNIFY_MEAS_SIZE];
} GNSSFormMeas_t;

typedef struct
{
  AsensingGNSSPara_t z_gnss[GNSS_BUF_SIZE];
  uint8_t u_read_index;
  uint8_t u_write_index;
} GnssParaBuf_t;

void gnss_form_meas_init(void);
void gnss_form_unify_meas(GNSSFormMeas_t* pz_form_meas);
void gnss_put_data_ringbuf(AsensingGNSSPara_t* pz_gnss);
GNSSFormMeas_t* gnss_get_form_meas(void);
int8_t gnss_get_data_ringbuf(void);
uint8_t gnss_get_posflag(void);

#endif // !__FUSION_GNSS_H__
