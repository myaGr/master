/**@file        fusion_gnss.c
 * @brief		handle gnss measurement
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

#include "fusion_gnss.h"
#include "fusion_if.h"
#include "fusion_global.h"
#include "fusion_config.h"
#include "fusion_api.h"
#include <math.h>
#include <string.h>
#include "mw_log.h"
#ifdef INS_DEBUG
#include "fusion_reference_debug.h"
#endif

static GNSSFormMeas_t gz_gnss_meas;
static GnssParaBuf_t gz_gnss_buf;

/**
 * @brief GNSS struct init
 * @param[in] None
 * @param[out] None
 * @return
*/
void gnss_form_meas_init(void)
{
  memset(&gz_gnss_meas, 0, sizeof(gz_gnss_meas));
  memset(&gz_gnss_buf, 0, sizeof(gz_gnss_buf));
}

GNSSFormMeas_t* gnss_get_form_meas(void)
{
  return &gz_gnss_meas;
}

int8_t gnss_get_data_ringbuf(void)
{
  int8_t s_val = 1;

  if (gz_gnss_buf.u_read_index != gz_gnss_buf.u_write_index)
  {
    AsensingGNSSPara_t* pz_gnss_para = fusion_get_gnss_para();
    gz_gnss_buf.u_read_index = (gz_gnss_buf.u_read_index) & (GNSS_BUF_SIZE - 1);
    memcpy(pz_gnss_para, &gz_gnss_buf.z_gnss[gz_gnss_buf.u_read_index], sizeof(AsensingGNSSPara_t));
    gz_gnss_buf.u_read_index++;
  }
  else
  {
    s_val = -1;
  }
  return s_val;
}

void gnss_put_data_ringbuf(AsensingGNSSPara_t* pz_gnss)
{
  if (gz_gnss_buf.u_write_index != (gz_gnss_buf.u_read_index ^ GNSS_BUF_SIZE))
  {
    gz_gnss_buf.u_write_index = (gz_gnss_buf.u_write_index) & (GNSS_BUF_SIZE - 1);
    memcpy(&gz_gnss_buf.z_gnss[gz_gnss_buf.u_write_index], pz_gnss, sizeof(AsensingGNSSPara_t));
    gz_gnss_buf.u_write_index++;
  }
}

uint8_t gnss_get_posflag(void)
{
  return *((uint8_t*)(&gz_gnss_meas.d_gnss_meas[14]));
}

// #define OLD_BDDB10
/* BDDB10 without SNR */
void gnss_flag_check(GNSSFormMeas_t* pz_form_meas, AsensingGNSSPara_t* pz_gnss)
{
  if (((int)pz_gnss->e_posflag > 0) &&
    ((pz_gnss->f_heading < 0.0) || ((int)pz_gnss->u_satused < 1)))
  {
    pz_form_meas->d_gnss_meas[14] = 0.0;
  }
#ifndef OLD_BDDB10
  if ((int)pz_gnss->e_posflag > 0 && (pz_gnss->f_avgsnr < 20.f))
  {
    pz_form_meas->d_gnss_meas[14] = 0.0;
  }
#endif
  if (((int)pz_gnss->e_posflag > 0) && ((int)pz_gnss->u_postype == 0) && (pz_gnss->f_hdop > 30.0f))
  {
    pz_form_meas->d_gnss_meas[14] = 0.0;
  }
}

void gnss_dispose_meas_std(GNSSFormMeas_t* pz_meas)
{
  float f_var_ratio = 0.f;
  float f_var_thre = 0.f;
  uint8_t u_posflag = *((uint8_t*)(&pz_meas->d_gnss_meas[14]));
  NavConfig_t* pNavConfig = fusion_get_navconfig();
  FusionAlgoConfig_t* pz_algocfg = fusion_get_algconfig();

  switch (u_posflag)
  {
  case TYPE_POS_FLAG_GNSSONLY:
      f_var_ratio = pNavConfig->f_pos_scl_sps * SQR(0.5f);
      break;
  case TYPE_POS_FLAG_DGNSS:
      f_var_ratio = pNavConfig->f_pos_scl_dif;
      break;
  case TYPE_POS_FLAG_RTK_FIX:
      f_var_ratio = pNavConfig->f_pos_scl_rtk;
      break;
  case TYPE_POS_FLAG_RTK_FLOAT:
      f_var_ratio = pNavConfig->f_pos_scl_flt;
      break;
  default:
      f_var_ratio = pNavConfig->f_pos_scl_sps;
      break;
  }

  if (u_posflag == TYPE_POS_FLAG_GNSSONLY)
  {
    f_var_thre = 0.25 * 0.25f;

    pz_meas->d_gnss_meas[8] *= f_var_ratio;
    pz_meas->d_gnss_meas[8] = pz_meas->d_gnss_meas[8] < f_var_thre ? f_var_thre : pz_meas->d_gnss_meas[8];
    pz_meas->d_gnss_meas[9] *= f_var_ratio;
    pz_meas->d_gnss_meas[9] = pz_meas->d_gnss_meas[9] < f_var_thre ? f_var_thre : pz_meas->d_gnss_meas[9];
    pz_meas->d_gnss_meas[10] *= SQR(0.5);
    pz_meas->d_gnss_meas[10] = pz_meas->d_gnss_meas[10] < 2.0 * f_var_thre ? 2.0 * f_var_thre : pz_meas->d_gnss_meas[10];
  }
  else if (u_posflag == TYPE_POS_FLAG_DGNSS)
  {
    f_var_thre = 0.05f * 0.05f;
    pz_meas->d_gnss_meas[8] *= f_var_ratio;
    pz_meas->d_gnss_meas[8] = pz_meas->d_gnss_meas[8] < f_var_thre ? f_var_thre : pz_meas->d_gnss_meas[8];
    pz_meas->d_gnss_meas[9] *= f_var_ratio;
    pz_meas->d_gnss_meas[9] = pz_meas->d_gnss_meas[9] < f_var_thre ? f_var_thre : pz_meas->d_gnss_meas[9];
    pz_meas->d_gnss_meas[10] *= SQR(0.5);
    pz_meas->d_gnss_meas[10] = pz_meas->d_gnss_meas[10] < 2.0 * f_var_thre ? 2.0 * f_var_thre : pz_meas->d_gnss_meas[10];
  }
  else if (u_posflag == TYPE_POS_FLAG_RTK_FLOAT)
  {
    f_var_thre = 0.03f * 0.03f;
    pz_meas->d_gnss_meas[8] *= f_var_ratio;
    pz_meas->d_gnss_meas[8] = pz_meas->d_gnss_meas[8] < f_var_thre ? f_var_thre : pz_meas->d_gnss_meas[8];
    pz_meas->d_gnss_meas[9] *= f_var_ratio;
    pz_meas->d_gnss_meas[9] = pz_meas->d_gnss_meas[9] < f_var_thre ? f_var_thre : pz_meas->d_gnss_meas[9];
    pz_meas->d_gnss_meas[10] *= SQR(0.5);
    pz_meas->d_gnss_meas[10] = pz_meas->d_gnss_meas[10] < 2.0 * f_var_thre ? 2.0 * f_var_thre : pz_meas->d_gnss_meas[10];
  }
  else
  {
    f_var_thre = 0.01f * 0.01f;
    pz_meas->d_gnss_meas[8] *= f_var_ratio;
    pz_meas->d_gnss_meas[8] = pz_meas->d_gnss_meas[8] < f_var_thre ? f_var_thre : pz_meas->d_gnss_meas[8];
    pz_meas->d_gnss_meas[9] *= f_var_ratio;
    pz_meas->d_gnss_meas[9] = pz_meas->d_gnss_meas[9] < f_var_thre ? f_var_thre : pz_meas->d_gnss_meas[9];
    pz_meas->d_gnss_meas[10] *= 1.0;
    pz_meas->d_gnss_meas[10] = pz_meas->d_gnss_meas[10] < f_var_thre ? f_var_thre : pz_meas->d_gnss_meas[10];

    float f_var_thre_max = 0.02f * 0.02f;
    pz_meas->d_gnss_meas[8] = pz_meas->d_gnss_meas[8] > f_var_thre_max ? f_var_thre_max : pz_meas->d_gnss_meas[8];
    pz_meas->d_gnss_meas[9] = pz_meas->d_gnss_meas[9] > f_var_thre_max ? f_var_thre_max : pz_meas->d_gnss_meas[9];
    pz_meas->d_gnss_meas[10] = pz_meas->d_gnss_meas[10] > f_var_thre_max * 9.0 ? f_var_thre_max * 9.0 : pz_meas->d_gnss_meas[10];
  }
}

void gnss_form_unify_meas(GNSSFormMeas_t* pz_form_meas)
{
  if (gnss_get_data_ringbuf() > 0)
  {
    AsensingGNSSPara_t* pz_gnss = fusion_get_gnss_para();

    double d_vn = 0.0, d_ve = 0.0, d_vd = 0.0;
    double d_latstd = 0.0, d_lonstd = 0.0, d_altstd = 0.0;
    double d_vnstd = 0.0, d_vestd = 0.0, d_vdstd = 0.0;

#ifdef INS_DEBUG
    double error[6];
    int ret = ref_CalGnssError(pz_gnss, error);
    float horiStd = sqrtf(pz_gnss->f_latstd * pz_gnss->f_latstd + pz_gnss->f_lonstd * pz_gnss->f_lonstd);
    double refHoriError, refVelHoriError;
    if (ret > 0)
    {
      refHoriError = sqrt(error[0] * error[0] + error[1] * error[1]);
      refVelHoriError = sqrt(error[3] * error[3] + error[4] * error[4]);
      LOGE(TAG_VDR, "$GNSS_ERROR %f %d %f %f %f %f %f %d %f\n",
        pz_gnss->d_tow / 1000.0, pz_gnss->e_posflag, pz_gnss->f_avgsnr, pz_gnss->f_hdop, pz_gnss->f_latstd, pz_gnss->f_lonstd,
        horiStd, pz_gnss->u_satused, refHoriError);
    }
#endif

    if (pz_gnss->u_postype == 0)
    {
      float f_hor_std = pz_gnss->f_hor_accu;
      d_vn = pz_gnss->f_speed * cos(pz_gnss->f_heading * DEG2RAD);
      d_ve = pz_gnss->f_speed * sin(pz_gnss->f_heading * DEG2RAD);
      d_vd = 0.0;

      f_hor_std = f_hor_std > 1e-10 ? f_hor_std : 1.f;

      d_latstd = (double)f_hor_std;
      d_lonstd = (double)f_hor_std;
      d_altstd = (double)f_hor_std * 2.0;
      d_vnstd = (double)f_hor_std;
      d_vestd = (double)f_hor_std;
      d_vdstd = (double)f_hor_std * 2.0;
    }
    if (pz_gnss->u_postype == 1)
    {
      d_vn = pz_gnss->f_vn;
      d_ve = pz_gnss->f_ve;
      d_vd = pz_gnss->f_vd;

      d_latstd = pz_gnss->f_latstd;
      d_lonstd = pz_gnss->f_lonstd;
      d_altstd = pz_gnss->f_altstd;

      d_vnstd = pz_gnss->f_vnstd;
      d_vestd = pz_gnss->f_vestd;
      d_vdstd = pz_gnss->f_vdstd;
    }
    pz_form_meas->d_gnss_meas[0] = -1.0;
    pz_form_meas->d_gnss_meas[1] = (double)(pz_gnss->t_timestamp) * 0.001;
    pz_form_meas->d_gnss_meas[2] = pz_gnss->d_lat * DEG2RAD;
    pz_form_meas->d_gnss_meas[3] = pz_gnss->d_lon * DEG2RAD;
    pz_form_meas->d_gnss_meas[4] = pz_gnss->f_alt;
    pz_form_meas->d_gnss_meas[5] = d_vn;
    pz_form_meas->d_gnss_meas[6] = d_ve;
    pz_form_meas->d_gnss_meas[7] = d_vd;
    pz_form_meas->d_gnss_meas[8] = d_latstd * d_latstd;
    pz_form_meas->d_gnss_meas[9] = d_lonstd * d_lonstd;
    pz_form_meas->d_gnss_meas[10] = d_altstd * d_altstd;
    pz_form_meas->d_gnss_meas[11] = d_vnstd * d_vnstd;
    pz_form_meas->d_gnss_meas[12] = d_vestd * d_vestd;
    pz_form_meas->d_gnss_meas[13] = d_vdstd * d_vdstd;
    *((uint8_t*)(&pz_form_meas->d_gnss_meas[14])) = ((uint8_t)(pz_gnss->e_posflag));
    pz_form_meas->d_gnss_meas[15] = pz_gnss->f_heading;
    pz_form_meas->d_gnss_meas[16] = pz_gnss->f_hdop;
    pz_form_meas->d_gnss_meas[17] = pz_gnss->f_speed;
    pz_form_meas->d_gnss_meas[18] = (double)pz_gnss->u_satused;
    pz_form_meas->d_gnss_meas[19] = pz_gnss->f_avgsnr;

    LOGI(TAG_VDR, "[gnss:MeaNoise]:%.3lf,StdRaw:%.3f,%.3f,%.3f\n", pz_gnss->d_tow / 1000.0,
      d_latstd, d_lonstd, d_altstd);

    gnss_flag_check(pz_form_meas, pz_gnss);
    gnss_dispose_meas_std(pz_form_meas);

    LOGI(TAG_VDR, "[gnss:MeaNoise]:%.3lf,GnssFlagRaw:%d,GnssFlagChecked:%d,Std:%.3f,%.3f,%.3f\n", pz_gnss->d_tow / 1000.0,
      (uint8_t)(pz_gnss->e_posflag), *((uint8_t*)(&pz_form_meas->d_gnss_meas[14])),
      sqrt(pz_form_meas->d_gnss_meas[8]), sqrt(pz_form_meas->d_gnss_meas[9]), sqrt(pz_form_meas->d_gnss_meas[10]));
  }
}