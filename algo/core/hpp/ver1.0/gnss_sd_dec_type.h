#ifndef __GNSS__SD__DEC__TYPE__H__
#define __GNSS__SD__DEC__TYPE__H__

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "gnss_types.h"


#ifdef __cplusplus
extern "C" {
#endif

//common paras
#define GLOEPH_STRNUM_IN_MSG               (5)
#define GLOALM_STRNUM_IN_MSG               (2)

#define MAX_BDS_NUM_D1                     (30)
#define MAX_BDS_NUM_D2                     (5)

#define  AODN_NAV_DATABITS_LEN_GPS         (40)
#define  AODN_NAV_DATABITS_LEN_GLN         (11)
#define  AODN_NAV_DATABITS_LEN_BDS         (40)
#define  AODN_NAV_DATABITS_LEN_GAL_F       (30)
#define  AODN_NAV_DATABITS_LEN_GAL_I       (29)
#define  AODN_NAV_DATABITS_LEN_MAX         (40)

#define  WORD_LEN_I_GAL                    (16)

	//******************GPS bit msg structrue**********************
	typedef struct subfr1_GPS
	{
		//word 1(TLM)
		//uint8_t  tlm_preamble;
		//U16 tlm_msg;
		//uint8_t  tlm_reserved;
		//uint8_t  tlm_integrity_status_flag;

		//word 2(HOW)
		uint32_t tow;
		uint8_t  alert_flag;
		uint8_t  anti_spoof_flag;
		uint8_t  subfr_id;
		//uint8_t  parity_check_bits01;

		//word 3
		uint16_t week;
		uint8_t  code_on_L2;
		uint8_t  ura;
		uint8_t  health;
		uint16_t iodc;

		//word 4
		uint8_t  L2_P_data_flag;
		//uint32_t reserved_bits_one;

		////word 5
		//uint32_t reserved_bits_two;

		////word 6
		//uint32_t reserved_bits_three;

		////word 7
		//U16 reserved_bits_four;
		int8_t  tgd;

		//word 8
		uint8_t  iode;
		uint16_t toc;

		//word 9
		int8_t  af2;
		int16_t af1;

		//word 10
		int32_t af0;
		//uint8_t  parity_check_bits02;
	}subfr1_GPS_t;

	typedef struct subfr2_GPS
	{
		//word 1(TLM)
		//uint8_t  tlm_preamble;
		//U16 tlm_msg;
		//uint8_t  tlm_reserved;
		//uint8_t  tlm_integrity_status_flag;

		//word 2(HOW)
		uint32_t tow;
		uint8_t  alert_flag;
		uint8_t  anti_spoof_flag;
		uint8_t  subfr_id;
		//uint8_t  parity_check_bits01;

		//word 3
		uint8_t  iode;
		int16_t crs;

		//word 4
		int16_t delta_N;

		//word 5
		int32_t m0;

		//word 6
		int16_t cuc;

		//word 7
		uint32_t e;

		//word 8
		int16_t cus;

		//word 9;
		uint32_t sqrt_A;

		//word 10
		uint16_t toe;
		uint8_t  fit_interval_flag;
		uint8_t  aodo;
		//uint8_t  parity_check_bits02;

	}subfr2_GPS_t;

	typedef struct subfr3_GPS
	{
		//word 1(TLM)
		//uint8_t  tlm_preamble;
		//U16 tlm_msg;
		//uint8_t  tlm_reserved;
		//uint8_t  tlm_integrity_status_flag;

		//word 2(HOW)
		uint32_t tow;
		uint8_t  alert_flag;
		uint8_t  anti_spoof_flag;
		uint8_t  subfr_id;
		//uint8_t  parity_check_bits01;

		//word 3
		int16_t cic;

		//word 4
		int32_t omega0;

		//word 5
		int16_t cis;

		//word 6
		int32_t  i0;

		//word 7
		int16_t crc;

		//word 8
		int32_t w;

		//word 9
		int32_t omega_dot;

		//word 10
		uint8_t  iode;
		int16_t idot;
		//uint8_t  parity_check_bits02;

	}subfr3_GPS_t;

	typedef struct subfr45_page24_GPS
	{
		//word 1(TLM)
		//uint8_t  tlm_preamble;
		//U16 tlm_msg;
		//uint8_t  tlm_reserved;
		//uint8_t  tlm_integrity_status_flag;

		//word 2(HOW)
		uint32_t tow;
		uint8_t  alert_flag;
		uint8_t  anti_spoof_flag;
		uint8_t  subfr_id;
		//uint8_t  parity_check_bits01;

		//word 3
		uint8_t  data_id;
		uint8_t  sv_id;
		uint16_t e;

		//word 4
		uint8_t  toa;
		int16_t sigma_i;

		//word 5
		int16_t omega_dot;
		uint8_t  health;

		//word 6
		uint32_t sqrt_A;

		//word 7
		int32_t omega0;

		//word 8
		int32_t w;

		//word 9
		int32_t m0;

		//word 10
		int16_t af0;
		int16_t af1;
		//uint8_t  parity_check_bits02;

	}subfr45_page24_GPS_t;

	//structure of page 25 in subframe 5
	typedef struct subfr5_page25_GPS
	{
		//word 1(TLM)
		//uint8_t  tlm_preamble;
		//U16 tlm_msg;
		//uint8_t  tlm_reserved;
		//uint8_t  tlm_integrity_status_flag;

		//word 2(HOW)
		uint32_t tow;
		uint8_t  alert_flag;
		uint8_t  anti_spoof_flag;
		uint8_t  subfr_id;
		//uint8_t  parity_check_bits01;

		//word 3
		uint8_t  data_id;
		uint8_t  sv_id;
		uint8_t  toa;
		uint8_t  wna;

		//word 4-9
		uint8_t  health_sv[24];

		//word 10
		//uint8_t  reserved_bits_one;
		//U16 reserved_bits_two;
		//uint8_t  parity_check_bits02;

	}subfr5_page25_GPS_t;

	//structure of page 18 in subframe 4
	typedef struct subfr4_page18_GPS
	{
		//word 1(TLM)
		//uint8_t  tlm_preamble;
		//U16 tlm_msg;
		//uint8_t  tlm_reserved;
		//uint8_t  tlm_integrity_status_flag;

		//word 2(HOW)
		uint32_t tow;
		uint8_t  alert_flag;
		uint8_t  anti_spoof_flag;
		uint8_t  subfr_id;
		//uint8_t  parity_check_bits01;

		//word 3
		uint8_t  data_id;
		uint8_t  sv_id;
		int8_t  alfa0;
		int8_t  alfa1;

		//word 4
		int8_t  alfa2;
		int8_t  alfa3;
		int8_t  beta0;

		//word 5
		int8_t  beta1;
		int8_t  beta2;
		int8_t  beta3;

		//word 6
		int32_t A1;

		//word 7
		int32_t A0;

		//word 8
		uint8_t  tot;
		uint8_t  week;

		//word 9
		int8_t  deltT_LS;
		uint8_t  week_LSF;
		int8_t  DN;

		//word 10
		int8_t  deltT_LSF;
		//U16 reserved_bits_one;
		//uint8_t  parity_check_bits02;

	}subfr4_page18_GPS_t;

	//structure of page 25 in subframe 4
	typedef struct subfr4_page25_GPS
	{
		//word 1(TLM)
		//uint8_t  tlm_preamble;
		//U16 tlm_msg;
		//uint8_t  tlm_reserved;
		//uint8_t  tlm_integrity_status_flag;

		//word 2(HOW)
		uint32_t tow;
		uint8_t  alert_flag;
		uint8_t  anti_spoof_flag;
		uint8_t  subfr_id;
		//uint8_t  parity_check_bits01;

		//word 3 - 9
		uint8_t  data_id;
		uint8_t  sv_id;
		uint8_t  aSpoof_sv[32];
		uint8_t  reserved_bits_one;

		uint8_t  health_sv[8];

		//word 10
		//uint8_t  reserved_bits_two;
		//uint8_t  parity_check_bits02;

	}subfr4_page25_GPS_t;

	//subframe five
	typedef struct subfr5_GPS
	{
		subfr45_page24_GPS_t pageT24[24];
		subfr5_page25_GPS_t pageP25;
	}subfr5_GPS_t;

	//subframe four
	typedef struct subfr4_GPS
	{
		subfr45_page24_GPS_t pageT8[8];
		subfr4_page18_GPS_t pageP18;
		subfr4_page25_GPS_t pageP25;
	}subfr4_GPS_t;

	//将帧1、2、3的结构组合在一起，作为广播星历的结构
	typedef struct subframePart123
	{
		subfr1_GPS_t subfr01;
		subfr2_GPS_t subfr02;
		subfr3_GPS_t subfr03;
	}subframePart123_t;

	//将帧4,5的结构组合在一起，作为历书的结构
	typedef struct subframePart45
	{
		subfr4_GPS_t subfr04;
		subfr5_GPS_t subfr05;
	}subframePart45_t;

	//******************GLONASS bit msg structrue******************
	typedef struct string1_GLO
	{
		double  tor;

		uint8_t   stringNumber;
		uint8_t   reserved_bits_one;
		uint8_t   P1;
		uint16_t  tk;
		int32_t  x_dot;
		int8_t   x_dot2;
		int32_t  x;
	}string1_GLO_t;

	//串2的结构
	typedef struct string2_GLO
	{
		double  tor;

		uint8_t   stringNumber;
		uint8_t   Bn;
		uint8_t   P2;
		uint8_t   tb;
		uint8_t   reserved_bits_one;
		int32_t  y_dot;
		int8_t   y_dot2;
		int32_t  y;
	}string2_GLO_t;

	//串3的结构
	typedef struct string3_GLO
	{
		double  tor;

		uint8_t   stringNumber;
		uint8_t   P3;
		int16_t  gamma;
		uint8_t   reserved_bits_one;
		uint8_t   P;
		uint8_t   ln;
		int32_t  z_dot;
		int8_t   z_dot2;
		int32_t  z;
	}string3_GLO_t;

	//串4的结构
	typedef struct string4_GLO
	{
		double  tor;

		uint8_t   stringNumber;
		int32_t  tau;
		int8_t   delt_tau;
		uint8_t   E;
		uint16_t  reserved_bits_one;
		uint8_t   P4;
		uint8_t   FT;
		uint8_t   reserved_bits_two;
		uint16_t  NT;
		uint8_t   n;
		uint8_t   M;
	}string4_GLO_t;

	//串5的结构
	typedef struct string5_GLO
	{
		double  tor;

		uint8_t   stringNumber;
		uint16_t  NA;
		int32_t  tauC;
		uint8_t   reserved_bits_one;
		uint8_t   N4;
		int32_t  tauGPS;
		uint8_t   ln;
	}string5_GLO_t;

	//串6/8/10/12/14的结构
	typedef struct string6_GLO
	{
		double  tor;

		uint8_t   stringNumber;
		uint8_t   CA;
		uint8_t   MA;
		uint8_t   nA;
		int16_t  tauA;
		int32_t  lambdaA;
		int32_t  delt_iA;
		uint16_t  epsilonA;
	}string6_GLO_t;

	//串7/8/11/13/15的结构
	typedef struct string7_GLO
	{
		double  tor;

		uint8_t   stringNumber;
		int16_t  omegaA;
		uint32_t  t_lambdaA;
		int32_t  deltTA;
		int8_t   deltTA_dot;
		uint8_t   HA;
		uint8_t   ln;
	}string7_GLO_t;

	//第五帧串14的结构
	typedef struct string14_GLO
	{
		double  tor;

		uint8_t   stringNumber;
		int16_t  B1;
		int16_t  B2;
		uint8_t   KP;
	}string14_GLO_t;

	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	//for SPRD interface
	//前5串，星历部分结构
	typedef struct stringPart_Eph
	{
		string1_GLO_t  str1;
		string2_GLO_t  str2;
		string3_GLO_t  str3;
		string4_GLO_t  str4;
		string5_GLO_t  str5;
	}stringPart_Eph_t;

	//后10串，历书数据部分
	typedef struct stringPart_Alm
	{
		string6_GLO_t  str6;
		string7_GLO_t  str7;
	}stringPart_Alm_t;

	//时间参考
	typedef struct timeRef_GLO
	{
		uint8_t       flagN;
		uint8_t       flagB;

		uint16_t      N_A;   //闰年内天数            
		uint16_t      N4;    //闰年数    

		int16_t      B1;                    
		int16_t      B2;
		uint8_t       KP;
	}timeRef_GLO_t;

	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	//for Qualcomm interface
	//前4串，星历部分结构
	typedef struct stringPart_Eph_N{
		string1_GLO_t  str1;
		string2_GLO_t  str2;
		string3_GLO_t  str3;
		string4_GLO_t  str4;
	}stringPart_Eph_N_t;
	//后11串，历书数据部分
	typedef struct stringPart_Alm_N{
		uint8_t             subfr_id;
		string5_GLO_t  str5;
		string6_GLO_t  str6[5];
		string7_GLO_t  str7[5];
		string14_GLO_t str14;
	}stringPart_Alm_N_t;

	//******************BDS bit msg structrue***********************
	//for BDS_D2
	typedef struct subfr1_D1_BDS
	{
		//para
		//U16 pre;
		//uint8_t  rev1;
		uint8_t  fraID;
		uint32_t sow;

		uint8_t  satH1;
		uint8_t  aodc;
		uint8_t  urai;
		uint16_t wn;
		uint32_t toc;
		int16_t tgd1;
		int16_t tgd2;
		int8_t  alfa0;
		int8_t  alfa1;
		int8_t  alfa2;
		int8_t  alfa3;
		int8_t  beta0;
		int8_t  beta1;
		int8_t  beta2;
		int8_t  beta3;
		int16_t a2;
		int32_t a0;
		int32_t a1;
		uint8_t  aode;

	}subfr1_D1_BDS_t;

	typedef struct subfr2_D1_BDS
	{
		//para
		//U16 pre;
		//uint8_t  rev1;
		uint8_t  fraID;
		uint32_t sow;

		int16_t deltN;
		int32_t cuc;
		int32_t m0;
		uint32_t e;
		int32_t cus;
		int32_t crc;
		int32_t crs;
		uint32_t sqrtA;
		uint8_t  toe_U2;
	}subfr2_D1_BDS_t;

	typedef struct subfr3_D1_BDS
	{
		//para
		//U16 pre;
		//uint8_t  rev1;
		uint8_t  fraID;
		uint32_t sow;

		uint16_t toe_L15;
		int32_t i0;
		int32_t cic;
		int32_t omega_dot;
		int32_t cis;
		int32_t idot;
		int32_t omega0;
		int32_t w;
		//uint8_t  rev2;
	}subfr3_D1_BDS_t;

	typedef struct subfr45_page1_D1_BDS
	{
		//para
		//U16 pre;
		//uint8_t  rev1;
		uint8_t  fraID;
		uint32_t sow;

		//uint8_t  rev2;
		uint8_t  pnum;
		uint32_t sqrtA;
		int16_t a1;
		int16_t a0;
		int32_t omega0;
		uint32_t e;
		int16_t deltaI;
		uint8_t  toa;
		int32_t omega_dot;
		int32_t w;
		int32_t m0;
		//uint8_t  rev3;
	}subfr45_page1_D1_BDS_t;

	typedef struct subfr5_page8_D1_BDS
	{
		//para
		//U16 pre;
		//uint8_t  rev1;
		uint8_t  fraID;
		uint32_t sow;

		//uint8_t  rev2;
		uint8_t  pnum;
		//U16 hea[11];
		uint8_t  wna;
		uint8_t  toa;
	}subfr5_page8_D1_BDS_t;

	typedef struct subfr5_page9_D1_BDS
	{
		//para
		//U16 pre;
		//uint8_t  rev1;
		uint8_t  fraID;
		uint32_t sow;

		//uint8_t  rev2;
		uint8_t  pnum;
		//uint32_t rev3;
		int16_t a0_gps;
		int16_t a1_gps;
		int16_t a0_gal;
		int16_t a1_gal;
		int16_t a0_glo;
		int16_t a1_glo;
	}subfr5_page9_D1_BDS_t;

	typedef struct subfr5_page10_D1_BDS
	{
		//para
		//U16 pre;
		//uint8_t  rev1;
		uint8_t  fraID;
		uint32_t sow;

		//uint8_t  rev2;
		uint8_t  pnum;
		int8_t  deltT_LS;
		int8_t  deltT_LSF;
		uint8_t  wn_LSF;
		int32_t a0_UTC;
		int32_t a1_UTC;
		uint8_t  dn;
	}subfr5_page10_D1_BDS_t;

	typedef struct subfr123_D1_BDS
	{
		subfr1_D1_BDS_t subfr1;
		subfr2_D1_BDS_t subfr2;
		subfr3_D1_BDS_t subfr3;
	}subfr123_D1_BDS_t;

	typedef struct subfr45_D1_BDS
	{
		subfr45_page1_D1_BDS_t subfr4_PageT24[24];
		subfr45_page1_D1_BDS_t subfr5_PageT6[6];
		subfr5_page8_D1_BDS_t subfr5_page8;
		subfr5_page9_D1_BDS_t subfr5_page9;
		subfr5_page10_D1_BDS_t subfr5_page10;
	}subfr45_D1_BDS_t;

	//for BDS_D1
	//D2电文
	typedef struct subfr1_page1_D2_BDS
	{
		//主要参数
		//U16 pre;
		//uint8_t  rev1;
		uint8_t  fraID;
		uint32_t sow;
		uint8_t  pnum1;

		uint8_t  satH1;
		uint8_t  AODC;

		uint8_t  urai;
		uint16_t wn;
		uint32_t toc;

		int16_t tgd1;
		int16_t tgd2;
		//U16 rev2;

	}subfr1_page1_D2_BDS_t;

	typedef struct subfr1_page2_D2_BDS
	{
		////主要参数
		//U16 pre;
		//uint8_t  rev1;
		uint8_t  fraID;
		uint32_t  sow;
		uint8_t  pnum1;

		int8_t alfa0;
		int8_t alfa1;
		int8_t alfa2;
		int8_t alfa3;

		int8_t beta0;
		int8_t beta1;
		int8_t beta2;
		int8_t beta3;

		//uint8_t rev2;

	}subfr1_page2_D2_BDS_t;

	typedef struct subfr1_page3_D2_BDS
	{
		//主要参数
		//U16 pre;
		//uint8_t  rev1;
		uint8_t  fraID;
		uint32_t sow;
		uint8_t  pnum1;

		//uint8_t  rev2;
		//uint32_t rev3;
		//U16 rev4;
		int32_t a0;
		uint8_t  a1_U4;
		//uint8_t  rev5;
	}subfr1_page3_D2_BDS_t;

	typedef struct subfr1_page4_D2_BDS
	{
		//主要参数
		//U16 pre;
		//uint8_t  rev1;
		uint8_t  fraID;
		uint32_t sow;
		uint8_t  pnum1;

		uint32_t a1_L18;
		int16_t a2;
		uint8_t  aode;
		int16_t deltN;
		uint16_t cuc_U14;
		//uint8_t  rev2;
	}subfr1_page4_D2_BDS_t;

	typedef struct subfr1_page5_D2_BDS
	{
		//主要参数
		//U16 pre;
		//uint8_t  rev1;
		uint8_t  fraID;
		uint32_t  sow;
		uint8_t  pnum1;

		uint16_t cuc_L4;
		int32_t m0;
		int32_t cus;
		uint16_t e_U10;
		//uint8_t  rev2;
	}subfr1_page5_D2_BDS_t;

	typedef struct subfr1_page6_D2_BDS
	{
		//主要参数
		//U16 pre;
		//uint8_t  rev1;
		uint8_t  fraID;
		uint32_t  sow;
		uint8_t  pnum1;

		uint32_t e_L22;
		uint32_t sqrtA;
		uint16_t cic_U10;
		//uint8_t  rev2;
	}subfr1_page6_D2_BDS_t;

	typedef struct subfr1_page7_D2_BDS
	{
		//主要参数
		//U16 pre;
		//uint8_t  rev1;
		uint8_t  fraID;
		uint32_t sow;
		uint8_t  pnum1;

		uint8_t  cic_L8;
		int32_t cis;
		uint32_t toe;
		uint32_t i0_U21;
		//uint8_t  rev2;
	}subfr1_page7_D2_BDS_t;

	typedef struct subfr1_page8_D2_BDS
	{
		//主要参数
		//U16 pre;
		//uint8_t  rev1;
		uint8_t  fraID;
		uint32_t sow;
		uint8_t  pnum1;

		uint16_t  i0_L11;
		int32_t crc;
		int32_t crs;
		uint32_t omega_dot_U19;
		//uint8_t  rev2;
	}subfr1_page8_D2_BDS_t;

	typedef struct subfr1_page9_D2_BDS
	{
		//主要参数
		//U16 pre;
		//uint8_t  rev1;
		uint8_t  fraID;
		uint32_t sow;
		uint8_t  pnum1;

		uint8_t  omega_dot_L5;
		int32_t omega0;
		uint32_t w_U27;
		//uint8_t  rev2;
	}subfr1_page9_D2_BDS_t;

	typedef struct subfr1_page10_D2_BDS
	{
		//主要参数
		//U16 pre;
		//uint8_t  rev1;
		uint8_t  fraID;
		uint32_t sow;
		uint8_t  pnum1;

		uint8_t  w_L5;
		int16_t idot;
		//U16 rev2;
		//uint32_t rev3;
		//uint32_t rev4;
	}subfr1_page10_D2_BDS_t;

	typedef struct subfr5_page36_D2_BDS
	{
		//U16 pre;
		//uint8_t  rev1;
		uint8_t  fraID;
		uint32_t sow;
		//uint8_t  rev2;
		uint8_t  pnum;

		//U16 hea[11];
		uint8_t  wna;
		uint8_t  toa;
	}subfr5_page36_D2_BDS_t;

	typedef struct subfr5_page3760_D2_BDS
	{
		//U16 pre;
		//uint8_t  rev1;
		uint8_t  fraID;
		uint32_t sow;
		//uint8_t  rev2;
		uint8_t  pnum;
		uint32_t sqrtA;
		int16_t a1;
		int16_t a0;
		int32_t omega0;
		uint32_t e;
		int16_t deltaI;
		uint8_t  toa;
		int32_t omega_dot;
		int32_t w;
		int32_t m0;
		//uint8_t  rev3;
	}subfr5_page3760_D2_BDS_t;

	typedef struct subfr5_page101_D2_BDS
	{
		//U16 pre;
		//uint8_t  rev1;
		uint8_t  fraID;
		uint32_t sow;
		//uint8_t  rev2;
		uint8_t  pnum;

		//uint32_t rev3;
		int16_t a0_gps;
		int16_t a1_gps;
		int16_t a0_gal;
		int16_t a1_gal;
		int16_t a0_glo;
		int16_t a1_glo;
	}subfr5_page101_D2_BDS_t;

	typedef struct subfr5_page102_D2_BDS
	{
		//U16 pre;
		//uint8_t  rev1;
		uint8_t  fraID;
		uint32_t sow;
		//uint8_t  rev2;
		uint8_t  pnum;

		int8_t  deltT_LS;
		int8_t  deltT_LSF;
		uint8_t  wn_LSF;
		int32_t a0_UTC;
		int32_t a1_UTC;
		uint8_t  dn;
	}subfr5_page102_D2_BDS_t;

	//subframe1全部内容 
	typedef struct subfr1_D2_BDS
	{
		//10页
		subfr1_page1_D2_BDS_t page1;
		subfr1_page2_D2_BDS_t page2;
		subfr1_page3_D2_BDS_t page3;
		subfr1_page4_D2_BDS_t page4;
		subfr1_page5_D2_BDS_t page5;
		subfr1_page6_D2_BDS_t page6;
		subfr1_page7_D2_BDS_t page7;
		subfr1_page8_D2_BDS_t page8;
		subfr1_page9_D2_BDS_t page9;
		subfr1_page10_D2_BDS_t page10;
	}subfr1_D2_BDS_t;

	typedef struct subfr5_D2_BDS
	{
		subfr5_page36_D2_BDS_t page36;
		subfr5_page3760_D2_BDS_t pageT30[30];
		subfr5_page101_D2_BDS_t page101;
		subfr5_page102_D2_BDS_t page102;
	}subfr5_D2_BDS_t;

	//******************GALILEO bit msg structrue******************
	//******************F/NAV*************************
	typedef struct Page1_F_GAL{
		uint8_t       type_num;
		uint8_t       svID;
		uint16_t      iod_nav;

		uint16_t      toc;
		int32_t      a_f0;
		int32_t      a_f1;
		int8_t       a_f2;

		uint8_t       sisa;
		
		uint16_t      a_i0;
		int16_t      a_i1;
		int16_t      a_i2;
		uint8_t       region_1;
		uint8_t       region_2;
		uint8_t       region_3;
		uint8_t       region_4;
		uint8_t       region_5;

		int16_t      bgd_E1_E5a;
		uint8_t       E5a_HS;

		uint16_t      wn;
		uint32_t      tow;

		uint8_t       E5a_DVS;
	}Page1_F_GAL_t;

	typedef struct Page2_F_GAL{
		uint8_t       type_num;
		uint16_t      iod_nav;
		int32_t      m0;
		int32_t      omega_dot;
		uint32_t      e;
		uint32_t      sqrt_a;
		int32_t      omega_0;
		int16_t      i_dot;
		uint16_t      wn;
		uint32_t      tow;
	}Page2_F_GAL_t;

	typedef struct Page3_F_GAL{
		uint8_t       type_num;
		uint16_t      iod_nav;
		int32_t      i0;
		int32_t      w;
		int16_t      delt_n;
		int16_t      cuc;
		int16_t      cus;
		int16_t      crc;
		int16_t      crs;
		uint16_t      toe;
		uint16_t      wn;
		uint32_t      tow;    
	}Page3_F_GAL_t;

	typedef struct Page4_F_GAL{
		uint8_t       type_num;
		uint16_t      iod_nav;
		int16_t      cic;
		int16_t      cis;
		int32_t      a0;
		int32_t      a1;
		int8_t       delt_tls;
		uint8_t       t0_t;
		uint8_t       wn0_t;
		uint8_t       wn_lsf;
		uint8_t       dn;
		int8_t       delt_tlsf;
		uint8_t       t0_g;
		int16_t      a0_g;
		int16_t      a1_g;
		uint8_t       wn0_g;
		uint32_t      tow;
	}Page4_F_GAL_t;

	typedef struct Page5_F_GAL{
		uint8_t       type_num;
		uint8_t       iod_a;
		uint8_t       wn_a;
		uint16_t      t0_a;
		uint8_t       svID_1;
		int16_t      sqrt_a_1;
		uint16_t      e_1;
		int16_t      w_1;
		int16_t      sigma_i_1;
		int16_t      omega0_1;
		int16_t      omega_dot_1;
		int16_t      m0_1;
		int16_t      a_f0_1;
		int16_t      a_f1_1;
		uint8_t       e5a_hs_1;
		uint8_t       svID_2;
		int16_t      sqrt_a_2;
		uint16_t      e_2;
		int16_t      w_2;
		int16_t      sigma_i_2;
		uint8_t       omega0_U4_2;	 
	}Page5_F_GAL_t;

	typedef struct Page6_F_GAL{
		uint8_t       type_num;
		uint8_t       iod_a;
		uint16_t      omega0_L12_2;
		int16_t      omega_dot_2;
		int16_t      m0_2;
		int16_t      a_f0_2;
		int16_t      a_f1_2;
		uint8_t       e5a_hs_2;
		uint8_t       svID_3;
		int16_t      sqrt_a_3;
		uint16_t      e_3;
		int16_t      w_3;
		int16_t      sigma_i_3;
		int16_t      omega0_3;
		int16_t      omega_dot_3;
		int16_t      m0_3;
		int16_t      a_f0_3;
		int16_t      a_f1_3;
		uint8_t       e5a_hs_3;	 
	}Page6_F_GAL_t;

	//the formal 4 pages， store EPH info;
	typedef struct pagePart_Eph_F_GAL{
		Page1_F_GAL_t  page1;
		Page2_F_GAL_t  page2;
		Page3_F_GAL_t  page3;
		Page4_F_GAL_t  page4;
	}pagePart_Eph_F_GAL_t;

	//last 2 pages, store ALM info; 
	typedef struct pagePart_Alm_F_GAL{
		Page5_F_GAL_t  page5;
		Page6_F_GAL_t  page6;
	}pagePart_Alm_F_GAL_t;

    //******************I/NAV*************************
	//page melt
	typedef struct page_odd_I_GAL{
		uint8_t  have_data;  //0:No data 1:have data;
		double tor;
		uint8_t  msg[AODN_NAV_DATABITS_LEN_MAX];
	}page_odd_I_GAL_t;

	typedef struct page_even_I_GAL{
		uint8_t  have_data;  //0:No data 1:have data;
		double tor;
		uint8_t  msg[AODN_NAV_DATABITS_LEN_MAX];
	}page_even_I_GAL_t;

	typedef struct page_all_I_GAL{
		page_odd_I_GAL_t  page_odd;
		page_even_I_GAL_t page_even;
	}page_all_I_GAL_t;

	//decode word
	typedef struct Word1_I_GAL{
		uint8_t       type_num;
		uint16_t      iod_nav;

		uint16_t      toe;
		int32_t      m0;
		uint32_t      e;
		uint32_t      sqrt_A;
	}Word1_I_GAL_t;

	typedef struct Word2_I_GAL{
		uint8_t       type_num;
		uint16_t      iod_nav;

		int32_t      omega_0;
		int32_t      i0;
		int32_t      w;
		int16_t      i_dot;
	}Word2_I_GAL_t;

	typedef struct Word3_I_GAL{
		uint8_t       type_num;
		uint16_t      iod_nav;

		int32_t      omega_dot;
		int16_t      delt_n;
		int16_t      cuc;
		int16_t      cus;
		int16_t      crc;
		int16_t      crs;
		uint8_t       SISA_E1_E5b;

	}Word3_I_GAL_t;

	typedef struct Word4_I_GAL{
		uint8_t       type_num;
		uint16_t      iod_nav;

		uint8_t       svID;
		int16_t      cic;
		int16_t      cis;
		uint16_t      toc;
		int32_t      a_f0;
		int32_t      a_f1;
		int8_t       a_f2;
	}Word4_I_GAL_t;

	typedef struct Word5_I_GAL{
		uint8_t       type_num;

		uint16_t      a_i0;
		int16_t      a_i1;
		int16_t      a_i2;
		uint8_t       region_1;
		uint8_t       region_2;
		uint8_t       region_3;
		uint8_t       region_4;
		uint8_t       region_5;

		int16_t      bgd_E1_E5a;
		int16_t      bgd_E1_E5b;
		uint8_t       E5b_HS;
		uint8_t       E1B_HS;
		uint8_t       E5b_DVS;
		uint8_t       E1B_DVS;
		uint16_t      wn;
		uint32_t      tow;
	}Word5_I_GAL_t;

	typedef struct Word6_I_GAL{
		uint8_t       type_num;

		int32_t      a0;
		int32_t      a1;
		int8_t       delt_tls;
		uint8_t       t0_t;
		uint8_t       wn0_t;
		uint8_t       wn_lsf;
		uint8_t       dn;
		int8_t       delt_tlsf;
		uint32_t      tow;
	}Word6_I_GAL_t;

	typedef struct Word7_I_GAL{
		uint8_t       type_num;
		uint8_t       iod_a;

		uint8_t       wn_a;
		uint16_t      t0_a;
		uint8_t       svID_1;
		int16_t      sqrt_a_1;
		uint16_t      e_1;
		int16_t      w_1;
		int16_t      sigma_i_1;
		int16_t      omega0_1;
		int16_t      omega_dot_1;
		int16_t      m0_1;
	}Word7_I_GAL_t;

	typedef struct Word8_I_GAL{
		uint8_t       type_num;
		uint8_t       iod_a;

		int16_t      a_f0_1;
		int16_t      a_f1_1;
		uint8_t       E5b_HS_1;
		uint8_t       E1B_HS_1;

		uint8_t       svID_2;
		int16_t      sqrt_a_2;
		uint16_t      e_2;
		int16_t      w_2;
		int16_t      sigma_i_2;
		int16_t      omega0_2;	
		int16_t      omega_dot_2;
	}Word8_I_GAL_t;

	typedef struct Word9_I_GAL{
		uint8_t       type_num;
		uint8_t       iod_a;

		uint8_t       wn_a;
		uint16_t      t0_a;
		int16_t      m0_2;
		int16_t      a_f0_2;
		int16_t      a_f1_2;
		uint8_t       E5b_HS_2;
		uint8_t       E1B_HS_2;
		uint8_t       svID_3;
		int16_t      sqrt_a_3;
		uint16_t      e_3;
		int16_t      w_3;
		int16_t      sigma_i_3; 
	}Word9_I_GAL_t;

	typedef struct Word10_I_GAL{
		uint8_t       type_num;
		uint8_t       iod_a;

		int16_t      omega0_3;
		int16_t      omega_dot_3;
		int16_t      m0_3;
		int16_t      a_f0_3;
		int16_t      a_f1_3;
		uint8_t       E5b_HS_3;
		uint8_t       E1B_HS_3;
		int16_t      a0_g;
		int16_t      a1_g;
		uint8_t       t0_g;
		uint8_t       wn0_g;
	}Word10_I_GAL_t;

	typedef struct wordPart_Eph_I_GAL{
		Word1_I_GAL_t  word1;
		Word2_I_GAL_t  word2;
		Word3_I_GAL_t  word3;
		Word4_I_GAL_t  word4;
		Word5_I_GAL_t  word5;
		Word6_I_GAL_t  word6;
	}wordPart_Eph_I_GAL_t;

	typedef struct wordPart_Alm_I_GAL{
		Word7_I_GAL_t  word7;
		Word8_I_GAL_t  word8;
		Word9_I_GAL_t  word9;
		Word10_I_GAL_t word10;
	}wordPart_Alm_I_GAL_t;

#ifdef __cplusplus
}
#endif

#endif