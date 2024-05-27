/**@file        ag_bddb_parse.c
 * @brief       asensing bddb data parse source file
 * @version     V1.0
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/05/29  <td>0.1      <td>caizhijie   <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#include "ag_bddb_parse.h"
#include "gnss_def.h"
#include "fusion_data_encode.h"

#define AG_IMU_BDDB_SYNC0 0xBD
#define AG_IMU_BDDB_SYNC1 0xDB

#define KWS_INIT 0.00863

static uint8_t ag_imu_bddb_checksum(const uint8_t* p, uint32_t len)
{
  uint32_t i;
  uint8_t check;
  check = 0u;
  for (i = 0u; i < len; i++)
  {
    check ^= p[i];
  }

  return check;
}

static float ag_bddb_byte2float(const uint8_t* input)
{
  uint32_t buf;
  float ret;

  buf = (((uint32_t)input[0]) << 0u) + (((uint32_t)input[1]) << 8u) + (((uint32_t)input[2]) << 16u) + (((uint32_t)input[3]) << 24u);
  
  ret = *((float*)(&buf));

  return ret;
}

static int32_t ag_bddb_byte2int32(const uint8_t* input)
{
  uint32_t buf;
  int32_t ret;

  buf = (((uint32_t)input[0]) << 0u) + (((uint32_t)input[1]) << 8u) + (((uint32_t)input[2]) << 16u) + (((uint32_t)input[3]) << 24u);


  ret = *((int32_t*)(&buf));

  return ret;
}

static uint32_t ag_bddb_byte2uint32(const uint8_t* input)
{
  uint32_t buf;

  buf = (((uint32_t)input[0]) << 0u) + (((uint32_t)input[1]) << 8u) + (((uint32_t)input[2]) << 16u) + (((uint32_t)input[3]) << 24u);

  return buf;
}

static int16_t ag_bddb_byte2int16(const uint8_t* input)
{
  uint16_t buf;
  int16_t ret;

  buf = (uint16_t)((((uint32_t)input[0]) << 0u) + (((uint32_t)input[1]) << 8u));

  ret = *((int16_t*)(&buf));

  return ret;
}

static uint16_t ag_bddb_byte2uint16(const uint8_t* input)
{
  uint16_t buf;

  buf = (uint16_t)((((uint32_t)input[0]) << 0u) + (((uint32_t)input[1]) << 8u));

  return buf;
}

uint8_t ag_imu_bddb_decode(ag_imu_bddb_decoder_t* pz_decoder, uint8_t ch, uint8_t* p_data, uint32_t* length)
{
  uint8_t cur_ch = ch;

  /* synchronize frame */
  if (pz_decoder->nbyte == 0)
  {
    if (!((AG_IMU_BDDB_SYNC0 == pz_decoder->prev_ch) && (AG_IMU_BDDB_SYNC1 == cur_ch)))
    {
      pz_decoder->prev_ch = cur_ch;
      return 0;
    }
    pz_decoder->buf[0] = pz_decoder->prev_ch;
    pz_decoder->buf[1] = cur_ch;
    pz_decoder->nbyte = 2;
    return 0;
  }

  if (pz_decoder->nbyte > sizeof(pz_decoder->buf))
  {
    pz_decoder->nbyte = 0;
    memset(pz_decoder->buf, 0, sizeof(pz_decoder->buf));
    return 0;
  }
  pz_decoder->buf[pz_decoder->nbyte++] = ch;

  if (pz_decoder->nbyte == 3)
  {
    pz_decoder->u_type = pz_decoder->buf[2];
    switch (pz_decoder->u_type)
    {
    case 0x0A: 
      pz_decoder->length = LEN_BD_DB_0A;
      break;
    case 0x0B:
      pz_decoder->length = LEN_BD_DB_0B;
      break;
    case 0x10:
      pz_decoder->length = LEN_BD_DB_10;
      break;
    default:
      pz_decoder->nbyte = 0;
      pz_decoder->prev_ch = 0;
      break;
    }
  }

  if (pz_decoder->nbyte < 3 || pz_decoder->nbyte < pz_decoder->length)
  {
    return 0;
  }
  pz_decoder->nbyte = 0;


  uint8_t checksum = ag_imu_bddb_checksum(pz_decoder->buf, pz_decoder->length - 1);

  if (checksum != pz_decoder->buf[pz_decoder->length - 1])
  {
    return 0;
  }

  memcpy(p_data, pz_decoder->buf, pz_decoder->length);
  if (length)
  {
    *length = pz_decoder->length;
  }

  return pz_decoder->u_type;
}

void ag_imu_bddb_parse_0x0A(uint8_t* p_data, uint32_t q_length, ag_imu_data_t* pz_ImuData)
{
  if (NULL == pz_ImuData)
  {
    return;
  }
  pz_ImuData->f_gyro[0] = (float)(DEG2RAD * ag_bddb_byte2float(&p_data[3]));
  pz_ImuData->f_gyro[1] = (float)(DEG2RAD * ag_bddb_byte2float(&p_data[7]));
  pz_ImuData->f_gyro[2] = (float)(DEG2RAD * ag_bddb_byte2float(&p_data[11]));
  pz_ImuData->f_accl[0] = (float)(Gravity_GuangZhou * ag_bddb_byte2float(&p_data[15]));
  pz_ImuData->f_accl[1] = (float)(Gravity_GuangZhou * ag_bddb_byte2float(&p_data[19]));
  pz_ImuData->f_accl[2] = (float)(Gravity_GuangZhou * ag_bddb_byte2float(&p_data[23]));
  pz_ImuData->f_temperature = (float)(200.0f / 32768.0f * ag_bddb_byte2int16(&p_data[27]));
  pz_ImuData->q_tow_msec = ag_bddb_byte2int32(&p_data[29]) / 4;
}

uint8_t ag_imu_bddb_parse_0x0B(uint8_t* p_data, uint32_t q_length, INSResults_t* pz_InsData)
{
	static uint8_t readnum = 0;
	switch (p_data[56])
	{
	case 14u:
		pz_InsData->f_la_imu2gnss[0] = (float)(ag_bddb_byte2int16(&p_data[46])) / 1000.0f;
		pz_InsData->f_la_imu2gnss[1] = (float)(ag_bddb_byte2int16(&p_data[48])) / 1000.0f;
		pz_InsData->f_la_imu2gnss[2] = (float)(ag_bddb_byte2int16(&p_data[50])) / 1000.0f;
		readnum++;
		break;
	case 17u:
		pz_InsData->f_la_imu2rearmid[0] = (float)(ag_bddb_byte2int16(&p_data[46])) / 1000.0f;
		pz_InsData->f_la_imu2rearmid[1] = (float)(ag_bddb_byte2int16(&p_data[48])) / 1000.0f;
		pz_InsData->f_la_imu2rearmid[2] = (float)(ag_bddb_byte2int16(&p_data[50])) / 1000.0f;
		readnum++;
		break;
	case 27u:
		pz_InsData->f_whlspd_sf[2] = (float)(1.0 - (float)(ag_bddb_byte2int16(&p_data[46])) / 1e5f / KWS_INIT);
		pz_InsData->f_whlspd_sf[3] = (float)(1.0 - (float)(ag_bddb_byte2int16(&p_data[48])) / 1e5f / KWS_INIT);
		pz_InsData->f_la_rear2rear = (float)(ag_bddb_byte2int16(&p_data[50])) / 1e3f;
		readnum++;
		break;
	case 10u:
		pz_InsData->f_mis_roll = (float)(ag_bddb_byte2int16(&p_data[46])) / 32768.0f * 360.0f;
		pz_InsData->f_mis_pitch = (float)(ag_bddb_byte2int16(&p_data[48])) / 32768.0f * 360.0f;
		pz_InsData->f_mis_yaw = (float)(ag_bddb_byte2int16(&p_data[50])) / 32768.0f * 360.0f;
		readnum++;
		break;
	case 24u:
		pz_InsData->f_misdualant_roll = (float)(ag_bddb_byte2int16(&p_data[46])) / 32768.0f * 360.0f;
		pz_InsData->f_misdualant_pitch = (float)(ag_bddb_byte2int16(&p_data[48])) / 32768.0f * 360.0f;
		pz_InsData->f_misdualant_yaw = (float)(ag_bddb_byte2int16(&p_data[50])) / 32768.0f * 360.0f;
		readnum++;
		break;	
	default:
		break;
	}

	return readnum;
}

void ag_imu_bddb_parse_0x20(uint8_t* p_data, uint32_t q_length, loc_api_wheel_t* pz_Wheel)
{
  pz_Wheel->q_fl_whpulse = ag_bddb_byte2uint16(&p_data[3]);
  pz_Wheel->q_fr_whpulse = ag_bddb_byte2uint16(&p_data[5]);
  pz_Wheel->t_timestamp = ag_bddb_byte2uint32(&p_data[7]);
  pz_Wheel->q_rl_whpulse = ag_bddb_byte2uint16(&p_data[12]);
  pz_Wheel->q_rr_whpulse = ag_bddb_byte2uint16(&p_data[14]);
  pz_Wheel->e_gear = p_data[21];
  pz_Wheel->f_angle_front = (float)(ag_bddb_byte2uint16(&p_data[26]) / 10.0);
}

void ag_imu_bddb_parse_0x10(uint8_t* p_data, uint32_t q_length, GnssFixType* pz_GnssRst)
{
  /*Longitude (deg)*/
  pz_GnssRst->Lon_deg = ((double)ag_bddb_byte2int32(&p_data[3])) / 1e7L;

  /* ¾­¶È¾«¶È */
  pz_GnssRst->LonStd = ((float)ag_bddb_byte2uint16(&p_data[7])) / 1e3f;

  /*Latitude (deg)*/
  pz_GnssRst->Lat_deg = ((double)ag_bddb_byte2int32(&p_data[9])) / 1e7L;

  /* Î³¶È¾«¶È */
  pz_GnssRst->LatStd = ((float)ag_bddb_byte2uint16(&p_data[13])) / 1e3f;

  /*Altitude (m)*/
  pz_GnssRst->Alt_m = ((float)ag_bddb_byte2int32(&p_data[15])) / 1e3f;

  /* ¸ß¶È¾«¶È */
  pz_GnssRst->AltStd = ((float)ag_bddb_byte2uint16(&p_data[19])) / 1e3f;

  /* ´¹Ïò¾«¶È */
  pz_GnssRst->SAcc = ((float)ag_bddb_byte2uint16(&p_data[21])) / 1e3f;

  /* ²î·ÖÑÓ³Ù */
  pz_GnssRst->RtkAge_s = (float)(p_data[23]);

  /* Gnss Êý¾ÝÀàÐÍ */
  pz_GnssRst->MsgType = (uint32_t)(p_data[24]);

  //output->flags.U = ag_bddb_byte2uint32(&input[25]);

  /* 位置、速度、航向标志位 */
  pz_GnssRst->PosFlag = p_data[25];

  /* Ë®Æ½ËÙ¶È */
  pz_GnssRst->LastHspd = pz_GnssRst->Hspd_mps;
  pz_GnssRst->Hspd_mps = ((float)ag_bddb_byte2int16(&p_data[29])) / 1e2f;

  /* º½¼£ deg */
  pz_GnssRst->Trk_deg = ((float)ag_bddb_byte2int16(&p_data[31])) / 1e2f;

  /* ´¹ÏòËÙ¶È */
  pz_GnssRst->LastVspd = pz_GnssRst->Vspd_mps;
  pz_GnssRst->Vspd_mps = ((float)ag_bddb_byte2int16(&p_data[33])) / 1e2f;

  /* ËÙ¶È²âÁ¿ÑÓ³Ù */
  pz_GnssRst->LatencyVel_s = ((float)ag_bddb_byte2int16(&p_data[35])) / 1e3f;

  /* »ùÏß³¤¶È */
  pz_GnssRst->BaseLineLength_m = ((float)ag_bddb_byte2int16(&p_data[37])) / 1e3f;

  /* º½Ïò½Ç */
  pz_GnssRst->Heading_deg = ((float)ag_bddb_byte2int16(&p_data[39])) / 1e2f;

  /* º½Ïò½Ç¾«¶È */
  pz_GnssRst->HeadingStd = ((float)ag_bddb_byte2int16(&p_data[41])) / 1e3f;

  /* ¸©Ñö½Ç */
  pz_GnssRst->Pitch_deg = ((float)ag_bddb_byte2int16(&p_data[43])) / 1e3f;

  /* ¸©Ñö½Ç¾«¶È */
  pz_GnssRst->PitchStd = ((float)ag_bddb_byte2int16(&p_data[45])) / 1e3f;

  /* GSPÖÜÊý */
  pz_GnssRst->GPSWeek = ((uint32_t)ag_bddb_byte2int16(&p_data[47])) >> 6u;
  pz_GnssRst->GPSWeek += 2000;

  /* Äê */
  pz_GnssRst->UtcYear = ((uint32_t)ag_bddb_byte2int16(&p_data[47])) & 0x3fu;
  pz_GnssRst->UtcYear += 2000;

  /* ÔÂ */
  pz_GnssRst->UtcMon = p_data[49];

  /* ÈÕ */
  pz_GnssRst->UtcDay = p_data[50];

  /* Ê± */
  pz_GnssRst->UtcHour = p_data[51];

  /* ·Ö */
  pz_GnssRst->UtcMin = p_data[52];

  /* Ãë */
  pz_GnssRst->UtcSec = (uint8_t)(((float)ag_bddb_byte2uint16(&p_data[53])) / 1e3f);

  /* ¶¨Î»Ö¡ÖÜÄÚÃë */
  pz_GnssRst->ItowPos = (double)(ag_bddb_byte2uint32(&p_data[55])) / 1e3L;

  /* ËÙ¶ÈÖ¡ÖÜÄÚÃë */
  pz_GnssRst->ItowVel = (double)(ag_bddb_byte2uint32(&p_data[59])) / 1e3L;

  /* º½ÏòÖ¡ÖÜÄÚÃë */
  pz_GnssRst->ItowHeading = (double)(ag_bddb_byte2uint32(&p_data[63])) / 1e3L;

  /* ±êÖ¾Î» */
  pz_GnssRst->RecMsgEkf = p_data[67];
  pz_GnssRst->RecMsgOutput = p_data[67];

  /* ´ÓÌìÏßÎÀÐÇÊý */
  pz_GnssRst->SvNumMst = p_data[68];
  pz_GnssRst->SvNumSlv = p_data[68];

  if (LEN_BD_DB_10 == 74u)
  {
    pz_GnssRst->avg_CN0 = ((float)ag_bddb_byte2int16(&p_data[71])) / 1e2f;
  }

  pz_GnssRst->VelN_mps = (float)(pz_GnssRst->Hspd_mps * cos(pz_GnssRst->Trk_deg * DEG2RAD));
  pz_GnssRst->VelE_mps = (float)(pz_GnssRst->Hspd_mps * sin(pz_GnssRst->Trk_deg * DEG2RAD));
  pz_GnssRst->VelD_mps = (float)(-pz_GnssRst->Vspd_mps);

  pz_GnssRst->HAcc = pz_GnssRst->LatStd > pz_GnssRst->LonStd? pz_GnssRst->LatStd : pz_GnssRst->LonStd;

  //if (output->MsgType != 0xfeu)//²»´øsAccµÄÊý¾Ý
  //{
  //  flag = output->flags.B.Vel;
  //  if (((flag == NOVATEL_FLAG_POSVEL_STATE_DOPPLER_VELOCITY) || (flag == NOVATEL_FLAG_POSVEL_STATE_SINGLE) || (flag == NOVATEL_FLAG_POSVEL_STATE_PSRDIFF) || (flag == NOVATEL_FLAG_POSVEL_STATE_NARROW_INT) || (flag == NOVATEL_FLAG_POSVEL_STATE_NARROW_FLOAT)) && (output->SvNumMst >= 5))
  //  {
  //    output->SAcc = 10.0f / ((float)output->SvNumMst);
  //  }
  //  else
  //  {
  //    output->SAcc = 100.0f;
  //  }

  //}

  pz_GnssRst->SysTime_ms = 0.0;
  pz_GnssRst->SyncTime_ms = pz_GnssRst->ItowPos * 1000.0;

  return;
}

static float AgMsg_Pdata2TrueVal(float in)
{
	return (logf(in) * 50.f);
}

uint32_t ag_imu_bddb_encode_0x0A(AsensingIMU_t* pz_ImuData, int8_t* p_output)
{
	if (NULL == pz_ImuData)
	{
		return -1;
	}
	uint8_t i = 0;
	uint32_t index = 0;

	uint8_t Check_A = 0;
	uint8_t Check_B = 0;

	float* pf;
	float buf;
	uint32_t buf_u32;
	uint16_t buf_u16;
	int16_t buf_s16;

	p_output[index++] = 0xBD;
	p_output[index++] = 0xDB;
	p_output[index++] = 0x0A;

	buf = (float)(pz_ImuData->f_gyro[0] * RAD2DEG);
	pf = &buf;
	buf_u32 = *((uint32_t*)pf);
	p_output[index++] = (uint8_t)(buf_u32 >> 0u);
	p_output[index++] = (uint8_t)(buf_u32 >> 8u);
	p_output[index++] = (uint8_t)(buf_u32 >> 16u);
	p_output[index++] = (uint8_t)(buf_u32 >> 24u);

	buf = (float)(pz_ImuData->f_gyro[1] * RAD2DEG);
	pf = &buf;
	buf_u32 = *((uint32_t*)pf);
	p_output[index++] = (uint8_t)(buf_u32 >> 0u);
	p_output[index++] = (uint8_t)(buf_u32 >> 8u);
	p_output[index++] = (uint8_t)(buf_u32 >> 16u);
	p_output[index++] = (uint8_t)(buf_u32 >> 24u);

	buf = (float)(pz_ImuData->f_gyro[2] * RAD2DEG);
	pf = &buf;
	buf_u32 = *((uint32_t*)pf);
	p_output[index++] = (uint8_t)(buf_u32 >> 0u);
	p_output[index++] = (uint8_t)(buf_u32 >> 8u);
	p_output[index++] = (uint8_t)(buf_u32 >> 16u);
	p_output[index++] = (uint8_t)(buf_u32 >> 24u);

	buf = pz_ImuData->f_accl[0] / Gravity_GuangZhou;
	pf = &buf;
	buf_u32 = *((uint32_t*)pf);
	p_output[index++] = (uint8_t)(buf_u32 >> 0u);
	p_output[index++] = (uint8_t)(buf_u32 >> 8u);
	p_output[index++] = (uint8_t)(buf_u32 >> 16u);
	p_output[index++] = (uint8_t)(buf_u32 >> 24u);

	buf = pz_ImuData->f_accl[1] / Gravity_GuangZhou;
	pf = &buf;
	buf_u32 = *((uint32_t*)pf);
	p_output[index++] = (uint8_t)(buf_u32 >> 0u);
	p_output[index++] = (uint8_t)(buf_u32 >> 8u);
	p_output[index++] = (uint8_t)(buf_u32 >> 16u);
	p_output[index++] = (uint8_t)(buf_u32 >> 24u);

	buf = pz_ImuData->f_accl[2] / Gravity_GuangZhou;
	pf = &buf;
	buf_u32 = *((uint32_t*)pf);
	p_output[index++] = (uint8_t)(buf_u32 >> 0u);
	p_output[index++] = (uint8_t)(buf_u32 >> 8u);
	p_output[index++] = (uint8_t)(buf_u32 >> 16u);
	p_output[index++] = (uint8_t)(buf_u32 >> 24u);

	buf_s16 = (int16_t)(pz_ImuData->f_temp * 32768.0f / 200.0f);
	buf_u16 = *((uint16_t*)(&buf_s16));
	p_output[index++] = (uint8_t)(buf_u16 >> 0u);
	p_output[index++] = (uint8_t)(buf_u16 >> 8u);

	buf_u32 = (uint32_t)(pz_ImuData->t_timestamp * 4.0);
	p_output[index++] = (uint8_t)(buf_u32 >> 0u);
	p_output[index++] = (uint8_t)(buf_u32 >> 8u);
	p_output[index++] = (uint8_t)(buf_u32 >> 16u);
	p_output[index++] = (uint8_t)(buf_u32 >> 24u);


	for (i = 0u; i < index; i++)
	{
		Check_A ^= p_output[i];
		Check_B ^= Check_A;
	}
	p_output[index++] = Check_A;
	p_output[index++] = Check_B;

	return index;
}

uint32_t ag_imu_bddb_encode_0x20(AsensingWhpulse_t* pz_WheelData, int8_t* p_output)
{
	uint32_t j, i;
	uint8_t Check_A = 0u;
	uint8_t Check_B = 0u;

	j = 0u;
	p_output[j++] = 0xBDu;
	p_output[j++] = 0xDBu;
	p_output[j++] = 0x20u;

	p_output[j++] = (uint8_t)((uint32_t)pz_WheelData->q_fl_whpulse);
	p_output[j++] = (uint8_t)((uint32_t)pz_WheelData->q_fl_whpulse >> 8u);

	p_output[j++] = (uint8_t)((uint32_t)pz_WheelData->q_fr_whpulse);
	p_output[j++] = (uint8_t)((uint32_t)pz_WheelData->q_fr_whpulse >> 8u);

	p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp));
	p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp) >> 8u);
	p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp) >> 16u);
	p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp) >> 24u);

	p_output[j++] = 0u;

	p_output[j++] = (uint8_t)((uint32_t)pz_WheelData->q_rl_whpulse);
	p_output[j++] = (uint8_t)((uint32_t)pz_WheelData->q_rl_whpulse >> 8u);

	p_output[j++] = (uint8_t)((uint32_t)pz_WheelData->q_rr_whpulse);
	p_output[j++] = (uint8_t)((uint32_t)pz_WheelData->q_rr_whpulse >> 8u);

	p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp));
	p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp) >> 8u);
	p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp) >> 16u);
	p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp) >> 24u);

	p_output[j++] = 0u;

	p_output[j++] = (uint8_t)pz_WheelData->e_gear;

	p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp));
	p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp) >> 8u);
	p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp) >> 16u);
	p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp) >> 24u);

	p_output[j++] = (uint8_t)(pz_WheelData->f_angle_front * 10.0f);
	p_output[j++] = (uint8_t)((uint32_t)(pz_WheelData->f_angle_front * 10.0f) >> 8u);

	p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp));
	p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp) >> 8u);
	p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp) >> 16u);
	p_output[j++] = (uint8_t)(((uint32_t)pz_WheelData->t_timestamp) >> 24u);
	p_output[j++] = 0u;

	for (i = 0u; i < j; i++)
	{
		Check_A ^= p_output[i];
		Check_B ^= Check_A;
	}


	p_output[j++] = Check_A;
	p_output[j++] = Check_B;

	return j;
}

static uint32_t transfer_flag(uint32_t flag)
{
	int ret = 0;
	switch (flag)
	{
	case 1:
		ret = 16;
		break;
	case 2:
		ret = 17;
		break;
	case 4:
		ret = 50;
		break;
	case 5:
		ret = 34;
		break;
	default:
		break;
	}
	return ret;
}

uint32_t ag_imu_bddb_encode_0x10(AsensingGNSSPara_t* pz_GnssFix, int8_t* p_output)
{
	uint8_t i = 0u, j = 0u;
	uint8_t checkbyte_a = 0u, checkbyte_b = 0u;
	uint32_t uintTemp;
	int32_t intTemp;

	j = 0u;
	p_output[j++] = 0xBDu;
	p_output[j++] = 0xDBu;
	p_output[j++] = 0x10u;

	intTemp = LONG2INT32(pz_GnssFix->d_lon);
	p_output[j++] = (uint8_t)LSB_32((uint32_t)intTemp);
	p_output[j++] = (uint8_t)MID1_32((uint32_t)intTemp);
	p_output[j++] = (uint8_t)MID2_32((uint32_t)intTemp);
	p_output[j++] = (uint8_t)MSB_32((uint32_t)intTemp);

	uintTemp = X2UINT16(pz_GnssFix->f_lonstd * 1e3f);
	p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
	p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

	intTemp = LAT2INT32(pz_GnssFix->d_lat);
	p_output[j++] = (uint8_t)LSB_32((uint32_t)intTemp);
	p_output[j++] = (uint8_t)MID1_32((uint32_t)intTemp);
	p_output[j++] = (uint8_t)MID2_32((uint32_t)intTemp);
	p_output[j++] = (uint8_t)MSB_32((uint32_t)intTemp);

	uintTemp = X2UINT16(pz_GnssFix->f_latstd * 1e3f);
	p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
	p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

	intTemp = H2INT32(pz_GnssFix->f_alt);
	p_output[j++] = (uint8_t)LSB_32((uint32_t)intTemp);
	p_output[j++] = (uint8_t)MID1_32((uint32_t)intTemp);
	p_output[j++] = (uint8_t)MID2_32((uint32_t)intTemp);
	p_output[j++] = (uint8_t)MSB_32((uint32_t)intTemp);

	uintTemp = X2UINT16(pz_GnssFix->f_altstd * 1e3f);
	p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
	p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

	uintTemp = X2UINT16(pz_GnssFix->f_hor_accu * 1e3f);
	p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
	p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

	p_output[j++] = (uint8_t)(0 + 0.5f);

	p_output[j++] = 0;

	uint32_t flag = 0;
	flag = transfer_flag((uint32_t)pz_GnssFix->e_posflag);
	p_output[j++] = flag;
	p_output[j++] = flag;
	p_output[j++] = flag;
	p_output[j++] = flag;

	intTemp = X2INT16(pz_GnssFix->f_speed * 1e2f);
	p_output[j++] = (uint8_t)LSB_16((uint32_t)intTemp);
	p_output[j++] = (uint8_t)MSB_16((uint32_t)intTemp);

	intTemp = X2INT16(pz_GnssFix->f_heading * 1e2f);
	p_output[j++] = (uint8_t)LSB_16((uint32_t)intTemp);
	p_output[j++] = (uint8_t)MSB_16((uint32_t)intTemp);

	intTemp = X2INT16(pz_GnssFix->f_vd * 1e2f * (-1.0));
	p_output[j++] = (uint8_t)LSB_16((uint32_t)intTemp);
	p_output[j++] = (uint8_t)MSB_16((uint32_t)intTemp);

	intTemp = X2INT16(0 * 1e3f);
	p_output[j++] = (uint8_t)LSB_16((uint32_t)intTemp);
	p_output[j++] = (uint8_t)MSB_16((uint32_t)intTemp);

	intTemp = X2INT16(0 * 1e3f);
	p_output[j++] = (uint8_t)LSB_16((uint32_t)intTemp);
	p_output[j++] = (uint8_t)MSB_16((uint32_t)intTemp);

	intTemp = X2INT16(pz_GnssFix->f_heading_deg * 1e2f);
	p_output[j++] = (uint8_t)LSB_16((uint32_t)intTemp);
	p_output[j++] = (uint8_t)MSB_16((uint32_t)intTemp);

	intTemp = X2INT16(pz_GnssFix->f_heading_std * 1e3f);
	p_output[j++] = (uint8_t)LSB_16((uint32_t)intTemp);
	p_output[j++] = (uint8_t)MSB_16((uint32_t)intTemp);

	intTemp = X2INT16(pz_GnssFix->f_pitch_deg * 1e3f);
	p_output[j++] = (uint8_t)LSB_16((uint32_t)intTemp);
	p_output[j++] = (uint8_t)MSB_16((uint32_t)intTemp);

	intTemp = X2INT16(pz_GnssFix->f_pitch_std * 1e3f);
	p_output[j++] = (uint8_t)LSB_16((uint32_t)intTemp);
	p_output[j++] = (uint8_t)MSB_16((uint32_t)intTemp);

	uintTemp = (((pz_GnssFix->w_week % 2000u) & 0x3ffu) << 6u) + ((pz_GnssFix->UtcYear % 2000u) & 0x3fu);
	p_output[j++] = (uint8_t)LSB_16(uintTemp);
	p_output[j++] = (uint8_t)MSB_16(uintTemp);

	p_output[j++] = (uint8_t)(pz_GnssFix->UtcMon);
	p_output[j++] = (uint8_t)(pz_GnssFix->UtcDay);
	p_output[j++] = (uint8_t)(pz_GnssFix->UtcHour);
	p_output[j++] = (uint8_t)(pz_GnssFix->UtcMin);

	uintTemp = X2UINT16(pz_GnssFix->UtcSec * 1000u);
	p_output[j++] = (uint8_t)LSB_16(uintTemp);
	p_output[j++] = (uint8_t)MSB_16(uintTemp);

	uintTemp = X2UINT32(pz_GnssFix->d_tow);
	p_output[j++] = (uint8_t)LSB_32(uintTemp);
	p_output[j++] = (uint8_t)MID1_32(uintTemp);
	p_output[j++] = (uint8_t)MID2_32(uintTemp);
	p_output[j++] = (uint8_t)MSB_32(uintTemp);

	uintTemp = X2UINT32(pz_GnssFix->d_tow);
	p_output[j++] = (uint8_t)LSB_32(uintTemp);
	p_output[j++] = (uint8_t)MID1_32(uintTemp);
	p_output[j++] = (uint8_t)MID2_32(uintTemp);
	p_output[j++] = (uint8_t)MSB_32(uintTemp);

	uintTemp = X2UINT32(pz_GnssFix->d_tow); //itow_heading
	p_output[j++] = (uint8_t)LSB_32(uintTemp);
	p_output[j++] = (uint8_t)MID1_32(uintTemp);
	p_output[j++] = (uint8_t)MID2_32(uintTemp);
	p_output[j++] = (uint8_t)MSB_32(uintTemp);

	p_output[j++] = (uint8_t)LSB_32(7);

	p_output[j++] = pz_GnssFix->u_satused;

	for (i = 0u; i < j; i++)
	{
		checkbyte_a ^= p_output[i];
		checkbyte_b ^= checkbyte_a;
	}
	p_output[j++] = checkbyte_a;
	p_output[j++] = checkbyte_b;

	intTemp = X2INT16(pz_GnssFix->f_avgsnr * 1e2f);
	p_output[j++] = (uint8_t)LSB_16((uint32_t)intTemp);
	p_output[j++] = (uint8_t)MSB_16((uint32_t)intTemp);
	checkbyte_a = 0u;

	for (i = 0u; i < j; i++)
	{
		checkbyte_a ^= p_output[i];
	}
	p_output[j++] = checkbyte_a;

	return j;
}

uint32_t ag_imu_bddb_encode_0x0B(INSResults_t* pz_InsPositionFix, int8_t* p_output)
{
	static uint8_t Index = 0u;
	uint8_t i = 0u, j;
	uint8_t checkbyte = 0u;
	uint16_t buf_u16;
	int16_t buf_i16;
	int32_t buf_i32;
	uint32_t buf_u32;
	int16_t temp_P[3] = {0};

	p_output[i++] = 0xBDu;
	p_output[i++] = 0xDBu;
	p_output[i++] = 0x0Bu;

	buf_i16 = (int16_t)(pz_InsPositionFix->f_roll / (360.0f / 32768.0f));
	buf_u16 = *((uint16_t*)(&buf_i16));
	p_output[i++] = (uint8_t)(buf_u16);
	p_output[i++] = (uint8_t)(buf_u16 >> 8u);

	buf_i16 = (int16_t)(pz_InsPositionFix->f_pitch / (360.0f / 32768.0f));
	buf_u16 = *((uint16_t*)(&buf_i16));
	p_output[i++] = (uint8_t)(buf_u16);
	p_output[i++] = (uint8_t)(buf_u16 >> 8u);

	buf_i16 = (int16_t)(pz_InsPositionFix->f_heading / (360.0f / 32768.0f));
	buf_u16 = *((uint16_t*)(&buf_i16));
	p_output[i++] = (uint8_t)(buf_u16);
	p_output[i++] = (uint8_t)(buf_u16 >> 8u);

	buf_i16 = (int16_t)(0.0 / (300.0f / 32768.0f));
	buf_u16 = *((uint16_t*)(&buf_i16));
	p_output[i++] = (uint8_t)(buf_u16);
	p_output[i++] = (uint8_t)(buf_u16 >> 8u);

	buf_i16 = (int16_t)(0.0 / (300.0f / 32768.0f));
	buf_u16 = *((uint16_t*)(&buf_i16));
	p_output[i++] = (uint8_t)(buf_u16);
	p_output[i++] = (uint8_t)(buf_u16 >> 8u);

	buf_i16 = (int16_t)(0.0 / (300.0f / 32768.0f));
	buf_u16 = *((uint16_t*)(&buf_i16));
	p_output[i++] = (uint8_t)(buf_u16);
	p_output[i++] = (uint8_t)(buf_u16 >> 8u);

	buf_i16 = (int16_t)(0.0 / (12.0f / 32768.0f));
	buf_u16 = *((uint16_t*)(&buf_i16));
	p_output[i++] = (uint8_t)(buf_u16);
	p_output[i++] = (uint8_t)(buf_u16 >> 8u);

	buf_i16 = (int16_t)(0.0 / (12.0f / 32768.0f));
	buf_u16 = *((uint16_t*)(&buf_i16));
	p_output[i++] = (uint8_t)(buf_u16);
	p_output[i++] = (uint8_t)(buf_u16 >> 8u);

	buf_i16 = (int16_t)(0.0 / (12.0f / 32768.0f));
	buf_u16 = *((uint16_t*)(&buf_i16));
	p_output[i++] = (uint8_t)(buf_u16);
	p_output[i++] = (uint8_t)(buf_u16 >> 8u);

	buf_i32 = (int32_t)(pz_InsPositionFix->d_latitude * RAD2DEG * 1e7);
	buf_u32 = *((uint32_t*)(&buf_i32));
	p_output[i++] = (uint8_t)(buf_u32);
	p_output[i++] = (uint8_t)(buf_u32 >> 8u);
	p_output[i++] = (uint8_t)(buf_u32 >> 16u);
	p_output[i++] = (uint8_t)(buf_u32 >> 24u);

	buf_i32 = (int32_t)(pz_InsPositionFix->d_longitude * RAD2DEG *1e7);
	buf_u32 = *((uint32_t*)(&buf_i32));
	p_output[i++] = (uint8_t)(buf_u32);
	p_output[i++] = (uint8_t)(buf_u32 >> 8u);
	p_output[i++] = (uint8_t)(buf_u32 >> 16u);
	p_output[i++] = (uint8_t)(buf_u32 >> 24u);

	buf_i32 = (int32_t)(pz_InsPositionFix->f_altitude * 1e3);
	buf_u32 = *((uint32_t*)(&buf_i32));
	p_output[i++] = (uint8_t)(buf_u32);
	p_output[i++] = (uint8_t)(buf_u32 >> 8u);
	p_output[i++] = (uint8_t)(buf_u32 >> 16u);
	p_output[i++] = (uint8_t)(buf_u32 >> 24u);

	buf_i16 = (int16_t)(pz_InsPositionFix->f_vn / (100.0f / 32768.0f));
	buf_u16 = *((uint16_t*)(&buf_i16));
	p_output[i++] = (uint8_t)(buf_u16);
	p_output[i++] = (uint8_t)(buf_u16 >> 8u);

	buf_i16 = (int16_t)(pz_InsPositionFix->f_ve / (100.0f / 32768.0f));
	buf_u16 = *((uint16_t*)(&buf_i16));
	p_output[i++] = (uint8_t)(buf_u16);
	p_output[i++] = (uint8_t)(buf_u16 >> 8u);

	buf_i16 = (int16_t)(pz_InsPositionFix->f_vd / (100.0f / 32768.0f));
	buf_u16 = *((uint16_t*)(&buf_i16));
	p_output[i++] = (uint8_t)(buf_u16);
	p_output[i++] = (uint8_t)(buf_u16 >> 8u);

	//p_output[i++] = (uint8_t)(15);
	p_output[i++] = (uint8_t)(pz_InsPositionFix->e_drposflag >= 6 ? 31 : 1);

	p_output[i++] = (uint8_t)(pz_InsPositionFix->q_kfmeastype >> 0u);
	p_output[i++] = (uint8_t)(pz_InsPositionFix->q_kfmeastype >> 8u);
	p_output[i++] = (uint8_t)(pz_InsPositionFix->q_kfmeastype >> 16u);
	p_output[i++] = (uint8_t)(pz_InsPositionFix->q_kfmeastype >> 24u);

	p_output[i++] = (uint8_t)1;

	p_output[i++] = (uint8_t)0;

  switch (Index)
  {
  case 0u:		//Ppos
    temp_P[0] = (int16_t)AgMsg_Pdata2TrueVal(pz_InsPositionFix->f_lat_std * pz_InsPositionFix->f_lat_std);
    temp_P[1] = (int16_t)AgMsg_Pdata2TrueVal(pz_InsPositionFix->f_lon_std * pz_InsPositionFix->f_lon_std);
    temp_P[2] = (int16_t)AgMsg_Pdata2TrueVal(pz_InsPositionFix->f_alt_std * pz_InsPositionFix->f_alt_std);
    break;
  case 1u:		//Pv
    temp_P[0] = (int16_t)AgMsg_Pdata2TrueVal(pz_InsPositionFix->f_vn_std * pz_InsPositionFix->f_vn_std);
    temp_P[1] = (int16_t)AgMsg_Pdata2TrueVal(pz_InsPositionFix->f_ve_std * pz_InsPositionFix->f_ve_std);
    temp_P[2] = (int16_t)AgMsg_Pdata2TrueVal(pz_InsPositionFix->f_vd_std * pz_InsPositionFix->f_vd_std);
    break;
  case 2u:		//Patt
    temp_P[0] = (int16_t)AgMsg_Pdata2TrueVal(pz_InsPositionFix->f_roll_std * pz_InsPositionFix->f_roll_std);
    temp_P[1] = (int16_t)AgMsg_Pdata2TrueVal(pz_InsPositionFix->f_pitch_std * pz_InsPositionFix->f_pitch_std);
    temp_P[2] = (int16_t)AgMsg_Pdata2TrueVal(pz_InsPositionFix->f_yaw_std * pz_InsPositionFix->f_yaw_std);
    break;
  case 3u:		//Pba
    temp_P[0] = (int16_t)0;
    temp_P[1] = (int16_t)0;
    temp_P[2] = (int16_t)0;
    break;
  case 4u:		//Pbg
    temp_P[0] = (int16_t)0;
    temp_P[1] = (int16_t)0;
    temp_P[2] = (int16_t)0;
    break;
  case 5u:		//Xpos
    temp_P[0] = (int16_t)0;
    temp_P[1] = (int16_t)0;
    temp_P[2] = (int16_t)0;
    break;
  case 6u:		//Xv
    temp_P[0] = (int16_t)0;
    temp_P[1] = (int16_t)0;
    temp_P[2] = (int16_t)0;
    break;
  case 7u:		//Xatt
    temp_P[0] = (int16_t)0;
    temp_P[1] = (int16_t)0;
    temp_P[2] = (int16_t)0;
    break;
  case 8u:		//Xba
    temp_P[0] = (int16_t)0;
    temp_P[1] = (int16_t)0;
    temp_P[2] = (int16_t)0;
    break;
  case 9u:		 //Xbg
    temp_P[0] = (int16_t)0;
    temp_P[1] = (int16_t)0;
    temp_P[2] = (int16_t)0;
    break;
  case 10u:
    temp_P[0] = (int16_t)(pz_InsPositionFix->f_mis_roll / 360.0f * 32768.0f);
    temp_P[1] = (int16_t)(pz_InsPositionFix->f_mis_pitch / 360.0f * 32768.0f);
    temp_P[2] = (int16_t)(pz_InsPositionFix->f_mis_yaw / 360.0f * 32768.0f);
    break;
  case 11:		//PCtb
    temp_P[0] = (int16_t)0;
    temp_P[1] = (int16_t)0;
    temp_P[2] = (int16_t)0;
    break;
  case 12:		 //XCtb
    temp_P[0] = (int16_t)0;
    temp_P[1] = (int16_t)0;
    temp_P[2] = (int16_t)0;
    break;
  case 13u:
    temp_P[0] = (int16_t)0;
    temp_P[1] = (int16_t)0;
    temp_P[2] = (int16_t)0;
    break;
  case 14u:
    temp_P[0] = (int16_t)(pz_InsPositionFix->f_la_imu2gnss[0] * 1000.0f);
    temp_P[1] = (int16_t)(pz_InsPositionFix->f_la_imu2gnss[1] * 1000.0f);
    temp_P[2] = (int16_t)(pz_InsPositionFix->f_la_imu2gnss[2] * 1000.0f);
    break;
  case 17u:
    temp_P[0] = (int16_t)(pz_InsPositionFix->f_la_imu2rearmid[0] * 1000.0f);
    temp_P[1] = (int16_t)(pz_InsPositionFix->f_la_imu2rearmid[1] * 1000.0f);
    temp_P[2] = (int16_t)(pz_InsPositionFix->f_la_imu2rearmid[2] * 1000.0f);
    break;
  case 20u:
    temp_P[0] = (int16_t)(pz_InsPositionFix->f_gyrobias_x * 32768.0f);
    temp_P[1] = (int16_t)(pz_InsPositionFix->f_gyrobias_y * 32768.0f);
    temp_P[2] = (int16_t)(pz_InsPositionFix->f_gyrobias_z * 32768.0f);
    break;
  case 21u:
    temp_P[0] = (int16_t)(pz_InsPositionFix->f_accbias_x / Gravity_GuangZhou / 0.1f * 32768.0f);
    temp_P[1] = (int16_t)(pz_InsPositionFix->f_accbias_y / Gravity_GuangZhou / 0.1f * 32768.0f);
    temp_P[2] = (int16_t)(pz_InsPositionFix->f_accbias_z / Gravity_GuangZhou / 0.1f * 32768.0f);
    break;
  case 22u:
    temp_P[0] = (int16_t)0;
		temp_P[1] = (int16_t)0;
		temp_P[2] = (int16_t)0;
    break;
  case 23u:
    temp_P[0] = X2INT16(0);
    temp_P[1] = X2INT16(0);
    temp_P[2] = X2INT16(0);
    break;
  case 24:
    temp_P[0] = (int16_t)(pz_InsPositionFix->f_misdualant_roll / 360.0f * 32768.0f);
    temp_P[1] = (int16_t)(pz_InsPositionFix->f_misdualant_pitch / 360.0f * 32768.0f);
    temp_P[2] = (int16_t)(pz_InsPositionFix->f_misdualant_yaw / 360.0f * 32768.0f);
    break;
  case 27u:
    temp_P[0] = (int16_t)(KWS_INIT * (1.0 - pz_InsPositionFix->f_whlspd_sf[2]) * 1e5f);
    temp_P[1] = (int16_t)(KWS_INIT * (1.0 - pz_InsPositionFix->f_whlspd_sf[3]) * 1e5f);
    temp_P[2] = (int16_t)(pz_InsPositionFix->f_la_rear2rear * 1e3f);
    break;
  case 30u:
    temp_P[0] = (int16_t)0;
    temp_P[1] = (int16_t)0;
    temp_P[2] = (int16_t)0;
    break;
  case 32u:		//gps
    temp_P[0] = (int16_t)0; // GET_NOVATEL_FLAG_POS(gpsdata->flags);
    temp_P[1] = (int16_t)0; // gpsdata->SatNumPos;
    temp_P[2] = (int16_t)0; //GET_NOVATEL_FLAG_HEADING(gpsdata->flags);
    break;
  case 33u:  //571 old 
    temp_P[0] = X2INT16(0 * 1e2);
		temp_P[1] = X2INT16(0 * 1e2);
		temp_P[2] = X2INT16(0 * 1e2);
    break;
  case 36u:
    temp_P[0] = X2INT16(0);
    temp_P[1] = X2INT16(0);
		temp_P[2] = X2INT16(0);
    break;
  case 44u:
    temp_P[0] = X2INT16(0);
    temp_P[1] = X2INT16(0);
    temp_P[2] = X2INT16(0);
    break;
  case 45u:
    temp_P[0] = X2INT16(0);
    temp_P[1] = X2INT16(0);
    temp_P[2] = X2INT16(0);
    break;
  case 46u:
    temp_P[0] = X2INT16(0);
    temp_P[1] = X2INT16(0);
		temp_P[2] = X2INT16(0);
    break;
  case 51u:
    temp_P[0] = (int16_t)(0 * 1e3f);
    temp_P[1] = (int16_t)(0 * 1e3f);
    temp_P[2] = (int16_t)(0 * 1e3f);
    break;
  case 52u:
    temp_P[0] = X2INT16(0 * 1e10f);
    temp_P[1] = X2INT16(0 * 1e10f);
    temp_P[2] = X2INT16(0 * 1e3f);
    break;
  case 54u:
    temp_P[0] = X2INT16(0);
    temp_P[1] = X2INT16(0);
    temp_P[2] = X2INT16(0);
    break;
  case 55u:
    temp_P[0] = X2INT16(0 * 1e3f);
    temp_P[1] = X2INT16(0);
		temp_P[2] = X2INT16(0);
    break;
  default:
    temp_P[0] = 0u;
    temp_P[1] = 0u;
    temp_P[2] = 0u;
    break;
  }

	p_output[i++] = (uint8_t)(temp_P[0]);
	p_output[i++] = (uint8_t)(temp_P[0] >> 8u);
	p_output[i++] = (uint8_t)(temp_P[1]);
	p_output[i++] = (uint8_t)(temp_P[1] >> 8u);
	p_output[i++] = (uint8_t)(temp_P[2]);
	p_output[i++] = (uint8_t)(temp_P[2] >> 8u);

	buf_u32 = (uint32_t)(pz_InsPositionFix->q_tow * 4.0);
	p_output[i++] = (uint8_t)(buf_u32);
	p_output[i++] = (uint8_t)(buf_u32 >> 8u);
	p_output[i++] = (uint8_t)(buf_u32 >> 16u);
	p_output[i++] = (uint8_t)(buf_u32 >> 24u);

	p_output[i++] = Index++;
	if (Index > 55u)
	{
		Index = 0u;
	}

	for (j = 0u; j < i; j++)
	{
		checkbyte ^= p_output[j];
	}
	p_output[i++] = (uint8_t)checkbyte;

	return i;
}

uint32_t ag_imu_bddb_encode_0x0D(INSResults_t* pz_InsPositionFix, int8_t* p_output)
{
	uint8_t i = 0u, j = 0u;
	uint8_t Check_A = 0u;
	uint8_t Check_B = 0u;
	uint32_t uintTemp;
	//int32_t intTemp;

	j = 0u;
	p_output[j++] = 0xBDu;
	p_output[j++] = 0xDBu;
	p_output[j++] = 0x0Du;

	uintTemp = X2UINT16(pz_InsPositionFix->z_inspl.f_pos_longitudinal_pl * 1e2f);
	p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
	p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

	uintTemp = X2UINT16(pz_InsPositionFix->z_inspl.f_pos_lateral_pl * 1e2f);
	p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
	p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

	uintTemp = X2UINT16(pz_InsPositionFix->z_inspl.f_pos_hpl * 1e2f);
	p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
	p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

	uintTemp = X2UINT16(pz_InsPositionFix->z_inspl.f_pos_vpl * 1e2f);
	p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
	p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

	uintTemp = X2UINT16(pz_InsPositionFix->z_inspl.f_pos_north_pl * 1e2f);
	p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
	p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

	uintTemp = X2UINT16(pz_InsPositionFix->z_inspl.f_pos_east_pl * 1e2f);
	p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
	p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

	uintTemp = X2UINT16(pz_InsPositionFix->z_inspl.f_vel_longitudinal_pl * 1e2f);
	p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
	p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

	uintTemp = X2UINT16(pz_InsPositionFix->z_inspl.f_vel_lateral_pl * 1e2f);
	p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
	p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

	uintTemp = X2UINT16(pz_InsPositionFix->z_inspl.f_vel_hpl * 1e2f);
	p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
	p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

	uintTemp = X2UINT16(pz_InsPositionFix->z_inspl.f_vel_vpl * 1e2f);
	p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
	p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

	uintTemp = X2UINT16(pz_InsPositionFix->z_inspl.f_vel_north_pl * 1e2f);
	p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
	p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

	uintTemp = X2UINT16(pz_InsPositionFix->z_inspl.f_vel_east_pl * 1e2f);
	p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
	p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

	uintTemp = X2UINT16(pz_InsPositionFix->z_inspl.f_roll_pl * 1e2f);
	p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
	p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

	uintTemp = X2UINT16(pz_InsPositionFix->z_inspl.f_pitch_pl * 1e2f);
	p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
	p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

	uintTemp = X2UINT16(pz_InsPositionFix->z_inspl.f_yaw_pl * 1e2f);
	p_output[j++] = (uint8_t)LSB_16((uint32_t)uintTemp);
	p_output[j++] = (uint8_t)MSB_16((uint32_t)uintTemp);

	//p_output[i++] = (uint8_t)(pz_InsPositionFix->z_inspl.e_drposflag >= 6 ? 31 : 1);
	p_output[i++] = (uint8_t)(pz_InsPositionFix->z_inspl.e_drposflag);

	p_output[j++] = (uint8_t)(((uint32_t)pz_InsPositionFix->z_inspl.t_timestamp));
	p_output[j++] = (uint8_t)(((uint32_t)pz_InsPositionFix->z_inspl.t_timestamp) >> 8u);
	p_output[j++] = (uint8_t)(((uint32_t)pz_InsPositionFix->z_inspl.t_timestamp) >> 16u);
	p_output[j++] = (uint8_t)(((uint32_t)pz_InsPositionFix->z_inspl.t_timestamp) >> 24u);

	for (i = 0u; i < j; i++)
	{
		Check_A ^= p_output[i];
		Check_B ^= Check_A;
	}

	p_output[j++] = Check_A;
	p_output[j++] = Check_B; /*delete?*/

	return j;
}
