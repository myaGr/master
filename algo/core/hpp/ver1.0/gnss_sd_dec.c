/************************************************************
* Copyrights(C) 
* All rights Reserved
* �ļ����ƣ�msg_sd_dec.c
* �ļ�����������չѶ�ӿ�rawbits������
* �汾�ţ�1.0.0
* ���ڣ�10/08
************************************************************/

#include <math.h>
#include "gnss_sd_dec.h"
#include "gnss.h"
#include "gnss_sys_api.h"
#include "gnss_tm.h"
#include "gnss_comm.h"

#undef MODULE_TAG
#define MODULE_TAG OBJ_SD

#if defined(_WIN32) || (defined(__GNUC__) && defined(__linux__))

//*********************************************************
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//temping caches, for decoding databits off-line
void *GPS_EPH[1000];
uint32_t GPS_EPH_NUM = 0;
void *GLO_EPH[1000];
uint32_t GLO_EPH_NUM = 0;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//temping caches, for decoding databits on-line
//GPS+QZSS
subframePart123_t     gps_subfrPart123[N_GPS_SVS + N_QZSS_SVS];
subframePart45_t      gps_subfrPart45[N_GPS_SVS + N_QZSS_SVS];
//GLONASS
timeRef_GLO_t         glo_timeRef[N_GLN_SVS];
stringPart_Eph_N_t    glo_strPartEph_N[N_GLN_SVS];
stringPart_Alm_N_t    glo_strPartAlm_N[N_GLN_SVS];
//BDS
subfr123_D1_BDS_t     bds_subfrPartEph_D1_N[MAX_BDS_NUM_D1];
subfr45_D1_BDS_t      bds_subfrPartAlm_D1_N[MAX_BDS_NUM_D1];
subfr1_D2_BDS_t       bds_subfrPartEph_D2_N[MAX_BDS_NUM_D2];
subfr5_D2_BDS_t       bds_subfrPartAlm_D2_N[MAX_BDS_NUM_D2];

//GALILEO -- F/NAV
pagePart_Eph_F_GAL_t  gal_pagePartEph_F_N[N_GAL_SVS];
pagePart_Alm_F_GAL_t  gal_pagePartAlm_F_N[N_GAL_SVS];

//GALILEO -- I/NAV
page_all_I_GAL_t      gal_page_all_I_N[N_GAL_SVS];
wordPart_Eph_I_GAL_t  gal_wordPartEph_I_N[N_GAL_SVS];
wordPart_Alm_I_GAL_t  gal_wordPartAlm_I_N[N_GAL_SVS];


//time ref others
extern GNSS_TIMESYSPARAM* p_gnssTimeSysParam;

//********************************************************

//********************internal_functions******************
void get_bitPos( int32_t pos, int32_t interval, int32_t *iCount, int32_t *iPos )
{
	//�ж�һ�����ڵڼ���ڼ�λ

	//��ʼ�����
	*iCount = 0;
	*iPos = 0;

	if ( pos % interval == 0 )
	{
		*iCount = (int32_t)floor( (double)pos / interval ) - 1;
		*iPos = interval;
	}
	else
	{
		*iCount = (int32_t)floor( (double)pos / interval );
		*iPos = pos % interval;
	}
}

uint8_t  get_unsigned_8bits( uint8_t *msg, int32_t sPos, int32_t ePos )
{
	//�ֲ�����
	uint8_t u8a,u8b,u8c;


	int32_t sBit,eBit; //����bit
	int32_t s1,s2;    //����λ
	int32_t valid_bits;

	//��ʼ�����
	uint8_t res = 0;

	//���
	valid_bits = ePos - sPos;
	if ( valid_bits < 0 || valid_bits > 8 )
	{
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"valid bit over 8,%s,%d\n",__FUNCTION__,__LINE__);
		return FALSE;
	}

	//�������ڵ�bits��λ��
	get_bitPos( sPos, 8, &sBit, &s1 );
	get_bitPos( ePos, 8, &eBit, &s2 );

	//�鿴�Ƿ���ͬһbyte
	if( sBit == eBit )
	{
		u8a = (uint8_t)( msg[sBit] << ( s1 - 1 ) );
		u8b = (uint8_t)( u8a >> ( 8 - valid_bits) );
		res = u8b;
	}
	else
	{
		u8a = (uint8_t)( msg[sBit] << (s1 -1) );
		u8b = (uint8_t)( msg[eBit] >> ( 8 - ( s1 - 1 ) ) );
		u8c = (uint8_t)( u8a | u8b );
		res = u8c >> ( 8 - valid_bits );
	}

	return res;

}

int8_t  get_signed_8bits( uint8_t *msg, int32_t sPos, int32_t ePos, int32_t sys )
{
	//sys -- ϵͳ��0ΪGPS��1ΪGLONASS

	//�ֲ�����
	uint8_t u8a,u8b,u8c;
	int8_t s8a;


	int32_t sBit,eBit; //����bit
	int32_t s1,s2;    //����λ
	int32_t valid_bits;
	uint8_t  iSign;  //GLONASS�ķ���λ

	//��ʼ�����
	int8_t res = 0;

	//���
	valid_bits = ePos - sPos;
	if ( valid_bits < 0 || valid_bits > 8 )
	{
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"valid bit over 8,%s,%d\n",__FUNCTION__,__LINE__);
		return FALSE;
	}

	//*************************************************

	//�������ڵ�bits��λ��
	get_bitPos( sPos, 8, &sBit, &s1 );
	get_bitPos( ePos, 8, &eBit, &s2 );

	if( sys == 0 )
	{
		//�鿴�Ƿ���ͬһbyte
		if( sBit == eBit )
		{
			u8a = (uint8_t)( msg[sBit] << ( s1 - 1 ) );
			u8b = (uint8_t)( u8a >> ( 8 - valid_bits) );
			res = (int32_t)u8b;
		}
		else
		{
			u8a = (uint8_t)( msg[sBit] << (s1 -1) );
			u8b = (uint8_t)( msg[eBit] >> ( 8 - ( s1 - 1 ) ) );
			s8a = (int8_t)( u8a | u8b );
			res = s8a >> ( 8 - valid_bits );
		}
	}
	else if( sys == 1 )
	{
		//���Ȼ�ȡ����λ
		u8a = ( msg[sBit] << ( s1 - 1 ) );
		u8b = u8a >> 7;
		iSign = u8b;
		u8c = get_unsigned_8bits( msg, sPos + 1, ePos );
		res = iSign?(-1)*u8c : u8c;
	}

	return res;

}

uint16_t get_unsigned_16bits( uint8_t *msg, int32_t sPos, int32_t ePos )
{
	//�ֲ�����
	uint16_t u16a,u16b,u16c,u16d;


	int32_t sBit,eBit; //����bit
	int32_t s1,s2;    //����λ
	int32_t valid_bits;

	//��ʼ�����
	uint16_t res = 0;

	//���
	valid_bits = ePos - sPos;
	if ( valid_bits < 0 || valid_bits > 16 )
	{
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"valid bit over 16,%s,%d\n",__FUNCTION__,__LINE__);
		return FALSE;
	}

	//�������ڵ�bits��λ��
	get_bitPos( sPos, 8, &sBit, &s1 );
	get_bitPos( ePos, 8, &eBit, &s2 );

	//�鿴�Ƿ����1��byte
	if( eBit - sBit == 1 )
	{
		u16a = (uint16_t)( msg[sBit] << ( s1 - 1 + 8 ) );
		u16b = (uint16_t)( msg[eBit] << ( s1 - 1 ) );
		u16c = (uint16_t)( u16a | u16b );
		u16c = (uint16_t)( u16c >> ( 16 - valid_bits ) );
		res = u16c;
	}
	else if( eBit - sBit == 2 )
	{
		u16a = (uint16_t)( msg[sBit] << ( s1 - 1 + 8 ) );
		u16b = (uint16_t)( msg[sBit+1] << ( s1 - 1 ) );
		u16c = (uint16_t)( msg[eBit] >> ( 8 - (s1 - 1 ) ) );
		u16d = (uint16_t)( u16a | u16b | u16c );
		res  = (uint16_t)( u16d >> ( 16 - valid_bits ) );
	}

	return res;

}

int16_t get_signed_16bits( uint8_t *msg, int32_t sPos, int32_t ePos, int32_t sys )
{
	//�ֲ�����
	uint8_t  u8a,u8b;
	uint16_t u16a,u16b,u16c;
	int16_t s16a,s16b;


	int32_t sBit,eBit; //����bit
	int32_t s1,s2;    //����λ
	int32_t valid_bits;
	uint8_t iSign;

	//��ʼ�����
	int16_t res = 0;


	//���
	valid_bits = ePos - sPos;
	if ( valid_bits < 0 || valid_bits > 16 )
	{
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"valid bit over 16,%s,%d\n",__FUNCTION__,__LINE__);
		return FALSE;
	}

	//�������ڵ�bits��λ��
	get_bitPos( sPos, 8, &sBit, &s1 );
	get_bitPos( ePos, 8, &eBit, &s2 );


	if( sys == 0 )
	{

		//�鿴�Ƿ����1��byte
		if( eBit - sBit == 1 )
		{
			u16a = (uint16_t)( msg[sBit] << ( s1 - 1 + 8 ) );
			u16b = (uint16_t)( msg[eBit] << ( s1 - 1 ) );
			s16a = (int16_t)( u16a | u16b );
			s16b = (int16_t)( s16a >> ( 16 - valid_bits ) );
			res = s16b;
		}
		else if( eBit - sBit == 2 )
		{
			u16a = (uint16_t)( msg[sBit] << ( s1 - 1 + 8 ) );
			u16b = (uint16_t)( msg[sBit+1] << ( s1 - 1 ) );
			u16c = (uint16_t)( msg[eBit] >> ( 8 - (s1 - 1 ) ) );
			s16a = (int16_t)( u16a | u16b | u16c );
			res  = (int16_t)( s16a >> ( 16 - valid_bits ) );
		}

	}
	else if( sys == 1 )
	{
		//��ȡ����λ
		u8a = (uint8_t)( msg[sBit] << ( s1 - 1 ) );
		u8b = (uint8_t)( u8a >> 7 );
		iSign = u8b;

		u16a = get_unsigned_16bits( msg, sPos+1, ePos );
		res = iSign?(-1)*u16a : u16a;
	}

	return res;

}

uint32_t get_unsigned_32bits( uint8_t *msg, int32_t sPos, int32_t ePos )
{
	//�ֲ�����
	uint32_t u32a,u32b,u32c,u32d,u32e,u32f;

	int32_t sBit,eBit; //����bit
	int32_t s1,s2;    //����λ
	int32_t valid_bits;

	//��ʼ�����
	uint32_t res = 0;

	//���
	valid_bits = ePos - sPos;
	if ( valid_bits < 0 || valid_bits > 32 )
	{
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"valid bit over 32,%s,%d\n",__FUNCTION__,__LINE__);
		return FALSE;
	}

	//�������ڵ�bits��λ��
	get_bitPos( sPos, 8, &sBit, &s1 );
	get_bitPos( ePos, 8, &eBit, &s2 );

	//�鿴�Ƿ����2��byte
	if( eBit - sBit == 2 )
	{
		u32a = (uint32_t)( msg[sBit] << ( s1 - 1 + 24 ) );
		u32b = (uint32_t)( msg[sBit+1] << ( s1 - 1 + 16) );
		u32c = (uint32_t)( msg[eBit] << ( s1 - 1 + 8 ) );
		u32d = (uint32_t)( u32a | u32b | u32c );
		u32e = (uint32_t)( u32d >> ( 32 - valid_bits ) );
		res = u32e;
	}
	else if( eBit - sBit == 3 )
	{
		u32a = (uint32_t)( msg[sBit] << ( s1 - 1 + 24 ) );
		u32b = (uint32_t)( msg[sBit+1] << ( s1 - 1 + 16) );
		u32c = (uint32_t)( msg[sBit+2] << ( s1 - 1 + 8) );
		u32d = (uint32_t)( msg[eBit] << ( s1 - 1 ) );
		u32e = (uint32_t)( u32a | u32b | u32c | u32d );
		u32f = (uint32_t)( u32e >> ( 32 - valid_bits ) );
		res = u32f;
	}
	else if( eBit - sBit == 4 )
	{
		u32a = (uint32_t)( msg[sBit] << ( s1 - 1 + 24 ) );
		u32b = (uint32_t)( msg[sBit+1] << ( s1 - 1 + 16) );
		u32c = (uint32_t)( msg[sBit+2] << ( s1 - 1 + 8) );
		u32d = (uint32_t)( msg[sBit+3] << ( s1 - 1 ) );
		u32e = (uint32_t)( msg[eBit] >> ( 8 - (s1 - 1 ) ) );

		u32f = (uint32_t)( u32a | u32b | u32c | u32d | u32e );
		res  = (uint32_t)( u32f >> ( 32 - valid_bits ) );
	}

	return res;

}

int32_t get_signed_32bits( uint8_t *msg, int32_t sPos, int32_t ePos, int32_t sys )
{
	//�ֲ�����
	uint8_t  u8a,u8b;
	uint32_t u32a,u32b,u32c,u32d,u32e;
	int32_t s32a,s32b;


	int32_t sBit,eBit; //����bit
	int32_t s1,s2;    //����λ
	int32_t valid_bits;
	uint8_t iSign;

	//��ʼ�����
	int32_t res = 0;


	//���
	valid_bits = ePos - sPos;
	if ( valid_bits < 0 || valid_bits > 32 )
	{
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"valid bit over 32,%s,%d\n",__FUNCTION__,__LINE__);
		return FALSE;
	}

	//�������ڵ�bits��λ��
	get_bitPos( sPos, 8, &sBit, &s1 );
	get_bitPos( ePos, 8, &eBit, &s2 );


	if( sys == 0 )
	{

		//�鿴�Ƿ����2��byte
		if( eBit - sBit == 2 )
		{
			u32a = (uint32_t)( msg[sBit] << ( s1 - 1 + 24 ) );
			u32b = (uint32_t)( msg[sBit+1] << ( s1 - 1 + 16) );
			u32c = (uint32_t)( msg[eBit] << ( s1 - 1 + 8 ) );
			s32a = (int32_t)( u32a | u32b | u32c );
			s32b = (int32_t)( s32a >> ( 32 - valid_bits ) );
			res = s32b;
		}
		else if( eBit - sBit == 3 )
		{
			u32a = (uint32_t)( msg[sBit] << ( s1 - 1 + 24 ) );
			u32b = (uint32_t)( msg[sBit+1] << ( s1 - 1 + 16) );
			u32c = (uint32_t)( msg[sBit+2] << ( s1 - 1 + 8) );
			u32d = (uint32_t)( msg[eBit] << ( s1 - 1 ) );
			s32a = (int32_t)( u32a | u32b | u32c | u32d );
			s32b = (int32_t)( s32a >> ( 32 - valid_bits ) );
			res = s32b;
		}
		else if( eBit - sBit == 4 )
		{
			u32a = (uint32_t)( msg[sBit] << ( s1 - 1 + 24 ) );
			u32b = (uint32_t)( msg[sBit+1] << ( s1 - 1 + 16) );
			u32c = (uint32_t)( msg[sBit+2] << ( s1 - 1 + 8) );
			u32d = (uint32_t)( msg[sBit+3] << ( s1 - 1 ) );
			u32e = (uint32_t)( msg[eBit] >> ( 8 - (s1 - 1 ) ) );

			s32a = (int32_t)( u32a | u32b | u32c | u32d | u32e );
			res  = (uint32_t)( s32a >> ( 32 - valid_bits ) );
		}

	}
	else if( sys == 1 )
	{
		//��ȡ����λ
		u8a = (uint8_t)( msg[sBit] << ( s1 - 1 ) );
		u8b = (uint8_t)( u8a >> 7 );
		iSign = u8b;

		u32a = get_unsigned_32bits( msg, sPos+1, ePos );
		res = iSign?(-1)*u32a : u32a;
	}

	return res;

}
//*****************GPS_PART***************************
uint8_t exchange_gps_eph( subframePart123_t *gps_subfrPart123,  GPS_EPH_INFO *gps_eph )
{
	float ura;
	//check inputs
	if( gps_subfrPart123 == NULL  || gps_eph == NULL ){
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs err!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}

	//init outputs
	memset( gps_eph, 0, sizeof(GPS_EPH_INFO) );

	//data valid check
	if( gps_subfrPart123->subfr02.e == 0 ||
		gps_subfrPart123->subfr02.sqrt_A == 0 )
	{
		SYS_LOGGING(OBJ_SD,LOG_WARNING,"empty rawbits for gps_eph decoded!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}

	//exchange data
	gps_eph->eph_status   = EPH_STATUS_VALID;
	gps_eph->eph_source   = EPH_SOURCE_BRDC;
	gps_eph->IODE         = gps_subfrPart123->subfr01.iode;              /* As in ICD-200  */
	gps_eph->fit_interval = gps_subfrPart123->subfr02.fit_interval_flag;      /* As in ICD-200 */

	gps_eph->C_rs         = (float)(gps_subfrPart123->subfr02.crs*P2_05);              /* Meters */
	gps_eph->C_rc         = (float)(gps_subfrPart123->subfr03.crc*P2_05);              /* Meters  */
	gps_eph->C_uc         = (float)(gps_subfrPart123->subfr02.cuc*P2_29);              /* Radians */
	gps_eph->C_us         = (float)(gps_subfrPart123->subfr02.cus*P2_29);              /* Radians */
	gps_eph->C_ic         = (float)(gps_subfrPart123->subfr03.cic*P2_29);              /* Radians  */
	gps_eph->C_is         = (float)(gps_subfrPart123->subfr03.cis*P2_29);              /* Radians*/
	gps_eph->t_oe         = (float)(gps_subfrPart123->subfr02.toe*16);              /* Seconds */
	gps_eph->delta_n      = gps_subfrPart123->subfr02.delta_N*P2_43*PI;           /* Radians/sec */
	gps_eph->M_0          = gps_subfrPart123->subfr02.m0*P2_31*PI;               /* Radians  */
	gps_eph->e            = gps_subfrPart123->subfr02.e*P2_33;                 /* Dimensionless */
	gps_eph->sqrt_A       = gps_subfrPart123->subfr02.sqrt_A*P2_19;            /* Meters**-1/2  */
	gps_eph->OMEGA_0      = gps_subfrPart123->subfr03.omega0*P2_31*PI;           /* Radians   */
	gps_eph->i_0          = gps_subfrPart123->subfr03.i0*P2_31*PI;               /* Radians */
	gps_eph->omega        = gps_subfrPart123->subfr03.w*P2_31*PI;                 /* Radians */
	gps_eph->OMEGADOT     = gps_subfrPart123->subfr03.omega_dot*P2_43*PI;          /* Radians */
	gps_eph->IDOT         = gps_subfrPart123->subfr03.idot*P2_43*PI;              /* Radians */
	/* ---The following items are derived from broadcast eph block---   */
	gps_eph->CDTR         = 0;              /* Meters */
	gps_eph->Axe          = 0;               /* Meters*/
	gps_eph->Axis         = 0;              /* Meters*/
	gps_eph->n            = 0;                 /* radians/sec */
	gps_eph->r1me2        = 0;             /* Dimensionless*/
	gps_eph->OMEGA_n      = 0;           /* Radians*/
	gps_eph->ODOT_n       = 0;            /* Radians */
	gps_eph->sinEk        = 0;

	gps_eph->subframe1.weeknum    = gps_subfrPart123->subfr01.week;        /* GPS week number of applicability*/
	gps_eph->subframe1.codeL2     = gps_subfrPart123->subfr01.code_on_L2;         /* Code on L2 flag */
	gps_eph->subframe1.L2Pdata    = gps_subfrPart123->subfr01.L2_P_data_flag;     /* L2-P data flag */
	gps_eph->subframe1.udre       = gps_subfrPart123->subfr01.ura;           /* SF1 raw accuracy factor */
	gps_eph->subframe1.SV_health  = gps_subfrPart123->subfr01.health;      /* SV health byte */
	gps_eph->subframe1.IODC       = gps_subfrPart123->subfr01.iodc;           /* IODC -- 10 LSBs*/
	gps_eph->subframe1.T_GD       = (float)(gps_subfrPart123->subfr01.tgd*P2_31);           /* Group delay time factor; seconds */
	gps_eph->subframe1.t_oc       = (float)(gps_subfrPart123->subfr01.toc*16);           /* Time of this block's applicability;*/
	/* GPS time of week; seconds                */
	gps_eph->subframe1.a_f2       = (float)(gps_subfrPart123->subfr01.af2*P2_55);           /* SV clock coef2; sec/sec^2*/
	gps_eph->subframe1.a_f1       = (float)(gps_subfrPart123->subfr01.af1*P2_43);           /* SV clock coef1; sec/sec */
	gps_eph->subframe1.a_f0       = (float)(gps_subfrPart123->subfr01.af0*P2_31);           /* SV clock coef0; sec*/
	
	ura = gps_subfrPart123->subfr01.ura;
	if( ura < 6 ) gps_eph->subframe1.SVacc  = (float)(pow( 2,(1.0 + ura/2) ));          /* SV accuracy; meters*/
	else          gps_eph->subframe1.SVacc  = (float)(pow( 2,(ura - 2.0) ));          /* SV accuracy; meters*/

	//clear the cache structure
	memset( gps_subfrPart123, 0, sizeof( subframePart123_t ) );


	return TRUE;
}

uint8_t exchange_gps_alm( subframePart45_t  *gps_subfrPart45, GPS_ALM_INFO *gps_alm )
{
	int32_t i;
	//check inputs
	if( gps_subfrPart45 == NULL || gps_alm == NULL ){
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs err!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}

	//init outputs
	memset( gps_alm, 0, sizeof(GPS_ALM_INFO)*MAX_GPS_PRN );

	//exchange the data, begin with the subframe 5
	//at first, clear the data with differ toa
	for( i = 0; i < 24; i++ ){
		if( gps_subfrPart45->subfr05.pageT24[i].subfr_id == 5 &&
			gps_subfrPart45->subfr05.pageT24[i].toa == gps_subfrPart45->subfr05.pageP25.toa )
		{

			//data valid check
			if( gps_subfrPart45->subfr05.pageT24[i].e == 0 ||
				gps_subfrPart45->subfr05.pageT24[i].sqrt_A == 0 )
			{
				SYS_LOGGING(OBJ_SD,LOG_WARNING,"empty rawbits for gps_alm decoded!,%s,%d",__FUNCTION__,__LINE__);
				continue;
			}
			gps_alm[i].t_oa_raw    = gps_subfrPart45->subfr05.pageP25.toa;           /* Raw time of almanac; LSB 2^12 sec      */
			gps_alm[i].SV_health   = gps_subfrPart45->subfr05.pageT24[i].health;      /* As in ICD-200                          */
			gps_alm[i].e           = (float)(gps_subfrPart45->subfr05.pageT24[i].e*P2_21);           /* dimensionless                          */
			gps_alm[i].t_oa        = (float)(gps_subfrPart45->subfr05.pageT24[i].toa*pow(2,12));  /* Almanac time of week in seconds        */
			gps_alm[i].i_0         = (float)(gps_subfrPart45->subfr05.pageT24[i].sigma_i*P2_19*PI);     /* Radians                                */
			gps_alm[i].OMEGADOT    = (float)(gps_subfrPart45->subfr05.pageT24[i].omega_dot*P2_38*PI);   /* Radians                                */
			gps_alm[i].sqrt_A      = (float)(gps_subfrPart45->subfr05.pageT24[i].sqrt_A*P2_11);      /* Meters^0.5                             */
			gps_alm[i].OMEGA_0     = (float)(gps_subfrPart45->subfr05.pageT24[i].omega0*P2_23*PI);      /* Radians                                */
			gps_alm[i].omega       = (float)(gps_subfrPart45->subfr05.pageT24[i].w*P2_23*PI);           /* Radians                                */
			gps_alm[i].M_0         = (float)(gps_subfrPart45->subfr05.pageT24[i].m0*P2_23*PI);          /* Radians                                */
			gps_alm[i].a_f0        = (float)(gps_subfrPart45->subfr05.pageT24[i].af0*P2_20);         /* Seconds                                */
			gps_alm[i].a_f1        = (float)(gps_subfrPart45->subfr05.pageT24[i].af1*P2_38);         /* Seconds/Seconds                        */
			/* ---The following items are derived from broadcast eph block---      */
			gps_alm[i].Axis    = 0;             /* Meters                                 */
			gps_alm[i].n       = 0;                /* Radians/Seconds                        */
			gps_alm[i].OMEGA_n = 0;          /* Radians                                */
			gps_alm[i].ODOT_n  = 0;           /* Radians                                */
			gps_alm[i].r1me2   = 0;            /* Dimensionless                          */
			gps_alm[i].Axen    = 0;             /* Meters                                 */
			gps_alm[i].t_zc    = 0;             /* time-z count of alm collection         */
			gps_alm[i].weeknum = 0;          /* Week number of alm collection          */
			gps_alm[i].wn_oa   = gps_subfrPart45->subfr05.pageP25.wna;    /* Week number of alm block applicability */
			gps_alm[i].alm_src = 0;          /* Source of alm block; used internally   */
			gps_alm[i].coll_sv = 0;          /* SV the alm block was collected from    */
			gps_alm[i].alm_status = TRUE;
			gps_alm[i].alm_src    = 0;
		}
	}

	//exchange the data, begin with the subframe 4
	for( i = 0; i < 8; i++ ){
		if( gps_subfrPart45->subfr04.pageT8[i].subfr_id == 4 &&
			gps_subfrPart45->subfr04.pageT8[i].toa == gps_subfrPart45->subfr05.pageP25.toa )
		{
			//data valid check
			if( gps_subfrPart45->subfr04.pageT8[i].e == 0 ||
				gps_subfrPart45->subfr04.pageT8[i].sqrt_A == 0 )
			{
				SYS_LOGGING(OBJ_SD,LOG_WARNING,"empty rawbits for gps_alm decoded!,%s,%d",__FUNCTION__,__LINE__);
				continue;
			}
			gps_alm[i+24].t_oa_raw    = gps_subfrPart45->subfr04.pageT8[i].toa;                /* Raw time of almanac; LSB 2^12 sec      */
			gps_alm[i+24].SV_health   = gps_subfrPart45->subfr04.pageT8[i].health;             /* As in ICD-200                          */
			gps_alm[i+24].e           = (float)(gps_subfrPart45->subfr04.pageT8[i].e*P2_21);            /* dimensionless                          */
			gps_alm[i+24].t_oa        = (float)(gps_subfrPart45->subfr05.pageP25.toa*4096);        /* Almanac time of week in seconds        */
			gps_alm[i+24].i_0         = (float)(gps_subfrPart45->subfr04.pageT8[i].sigma_i*P2_19*PI);   /* Radians                                */
			gps_alm[i+24].OMEGADOT    = (float)(gps_subfrPart45->subfr04.pageT8[i].omega_dot*P2_38*PI); /* Radians                                */
			gps_alm[i+24].sqrt_A      = (float)(gps_subfrPart45->subfr04.pageT8[i].sqrt_A*P2_11);       /* Meters^0.5                             */
			gps_alm[i+24].OMEGA_0     = (float)(gps_subfrPart45->subfr04.pageT8[i].omega0*P2_23*PI);    /* Radians                                */
			gps_alm[i+24].omega       = (float)(gps_subfrPart45->subfr04.pageT8[i].w*P2_23*PI);         /* Radians                                */
			gps_alm[i+24].M_0         = (float)(gps_subfrPart45->subfr04.pageT8[i].m0*P2_23*PI);        /* Radians                                */
			gps_alm[i+24].a_f0        = (float)(gps_subfrPart45->subfr04.pageT8[i].af0*P2_20);          /* Seconds                                */
			gps_alm[i+24].a_f1        = (float)(gps_subfrPart45->subfr04.pageT8[i].af1*P2_38);          /* Seconds/Seconds                        */
			/* ---The following items are derived from broadcast eph block---      */
			gps_alm[i+24].Axis    = 0;          /* Meters                                 */
			gps_alm[i+24].n       = 0;          /* Radians/Seconds                        */
			gps_alm[i+24].OMEGA_n = 0;          /* Radians                                */
			gps_alm[i+24].ODOT_n  = 0;          /* Radians                                */
			gps_alm[i+24].r1me2   = 0;          /* Dimensionless                          */
			gps_alm[i+24].Axen    = 0;          /* Meters                                 */
			gps_alm[i+24].t_zc    = 0;          /* time-z count of alm collection         */
			gps_alm[i+24].weeknum = 0;          /* Week number of alm collection          */
			gps_alm[i+24].wn_oa   = gps_subfrPart45->subfr05.pageP25.wna;                      /* Week number of alm block applicability */
			gps_alm[i+24].alm_src = 0;          /* Source of alm block; used internally   */
			gps_alm[i+24].coll_sv = 0;          /* SV the alm block was collected from    */
			gps_alm[i+24].alm_status = TRUE;
			gps_alm[i+24].alm_src    = 0;
		}
	}

	//clear cache structure
	memset( gps_subfrPart45->subfr04.pageT8, 0, sizeof( subfr45_page24_GPS_t )* 8 );
	memset( gps_subfrPart45->subfr05.pageT24, 0, sizeof( subfr45_page24_GPS_t )* 24 );

	return TRUE;
}

uint8_t exchange_gps_ionoAndUtc( subframePart45_t *gps_subfrPart45, ION_INFO *gps_iono, gpsUtcModel_t *gps_utc )
{
	//check inputs
	if( gps_subfrPart45 == NULL || gps_iono == NULL || gps_utc == NULL ){
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs err!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}

	//init outputs
	memset( gps_iono, 0, sizeof(ION_INFO) );
	memset( gps_utc, 0, sizeof(gpsUtcModel_t) );

	//exchange the data
	gps_iono->alpha_0   = (float)(gps_subfrPart45->subfr04.pageP18.alfa0*P2_30);       /* Seconds                                  */
	gps_iono->alpha_1   = (float)(gps_subfrPart45->subfr04.pageP18.alfa1*P2_27);       /* Sec. per semicircle                      */
	gps_iono->alpha_2   = (float)(gps_subfrPart45->subfr04.pageP18.alfa2*P2_24);       /* Sec. per semicircle^2                    */
	gps_iono->alpha_3   = (float)(gps_subfrPart45->subfr04.pageP18.alfa3*P2_24);       /* Sec. per semicircle^3                    */
	gps_iono->beta_0    = ((double)gps_subfrPart45->subfr04.pageP18.beta0*2048);   /* Seconds                                  */
	gps_iono->beta_1    = ((double)gps_subfrPart45->subfr04.pageP18.beta1*16384);   /* Sec. per semicircle                      */
	gps_iono->beta_2    = ((double)gps_subfrPart45->subfr04.pageP18.beta2*65536);   /* Sec. per semicircle^2                    */
	gps_iono->beta_3    = ((double)gps_subfrPart45->subfr04.pageP18.beta3*65536);   /* Sec. per semicircle^3                    */
	gps_iono->have_ion  = 1;                                                  /* Boolean flag                             */

	gps_utc->utcA0        = (double)(gps_subfrPart45->subfr04.pageP18.A0*P2_30);
	gps_utc->utcA1        = (double)(gps_subfrPart45->subfr04.pageP18.A1*P2_50);
	gps_utc->utcDeltaTls  = gps_subfrPart45->subfr04.pageP18.deltT_LS;
	gps_utc->utcDeltaTlsf = gps_subfrPart45->subfr04.pageP18.deltT_LSF;
	gps_utc->utcDN        = gps_subfrPart45->subfr04.pageP18.DN;
	gps_utc->utcTot       = gps_subfrPart45->subfr04.pageP18.tot*4096;
	gps_utc->utcWNlsf     = gps_subfrPart45->subfr04.pageP18.week_LSF;
	gps_utc->utcWNt       = gps_subfrPart45->subfr04.pageP18.week;
	gps_utc->have_utc     = 1;

	//clear the cache structure
	memset( &gps_subfrPart45->subfr04.pageP18, 0, sizeof( subfr4_page18_GPS_t ) );

	return TRUE;
}

uint8_t msg_decode_gps( 
	uint8_t *msg, 
	int32_t *header, 
	subframePart123_t *gps_subfrPart123, 
	subframePart45_t *gps_subfrPart45, 
	int32_t *flag )
{
	int32_t i, j, prn, subfr_id;
    uint32_t tow;

	//inputs chcck
	if( msg == NULL || header == NULL ){
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs err!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}

	//sys check
	if( msg[0] != 139 ){
		SYS_LOGGING(OBJ_SD,LOG_WARNING,"the tlm word check failed!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}

	//subframeID compare
	subfr_id = get_unsigned_8bits( msg, 44, 47);
	if( subfr_id != header[5] ){
		SYS_LOGGING(OBJ_SD,LOG_WARNING,"subframe ID compared failed!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}

	//tow check
	tow = get_unsigned_32bits( msg, 25, 42);
	if( tow*6 > 604800 ){
		SYS_LOGGING(OBJ_SD,LOG_WARNING,"tow is out of range,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}

	//***********decode msg
	prn = header[1] - 1; //sate number

	//subfrme one
	if( subfr_id == 1 ){
		//gps_subfrPart123[prn].subfr01.tlm_preamble = get_unsigned_8bits( msg , 1, 9 );
		//gps_subfrPart123[prn].subfr01.tlm_msg = get_unsigned_16bits( msg, 9, 23 );
		//gps_subfrPart123[prn].subfr01.tlm_integrity_status_flag = get_unsigned_8bits( msg, 23, 24);
		//gps_subfrPart123[prn].subfr01.tlm_reserved = get_unsigned_8bits( msg, 23, 25);
		gps_subfrPart123[prn].subfr01.tow = tow;
		gps_subfrPart123[prn].subfr01.alert_flag = get_unsigned_8bits( msg, 42, 43);
		gps_subfrPart123[prn].subfr01.anti_spoof_flag = get_unsigned_8bits( msg, 43, 44);
		gps_subfrPart123[prn].subfr01.subfr_id = get_unsigned_8bits( msg, 44, 47);
		//gps_subfrPart123[prn].subfr01.parity_check_bits01 = get_unsigned_8bits( msg , 47, 49);

		//��������
		gps_subfrPart123[prn].subfr01.week = get_unsigned_16bits( msg, 49, 59 );
		gps_subfrPart123[prn].subfr01.code_on_L2 = get_unsigned_8bits( msg, 59, 61 );
		gps_subfrPart123[prn].subfr01.ura = get_unsigned_8bits( msg, 61, 65 );
		gps_subfrPart123[prn].subfr01.health = get_unsigned_8bits( msg, 65, 71 );

		gps_subfrPart123[prn].subfr01.iodc = (uint16_t)(( get_unsigned_8bits( msg, 71, 73 ) << 8 )| get_unsigned_8bits( msg, 169, 177 ));

		gps_subfrPart123[prn].subfr01.L2_P_data_flag = get_unsigned_8bits( msg, 73, 74 );
		//gps_subfrPart123[prn].subfr01.reserved_bits_one = get_unsigned_32bits( msg, 74, 97 );
		//gps_subfrPart123[prn].subfr01.reserved_bits_two = get_unsigned_32bits( msg, 97, 121 );
		//gps_subfrPart123[prn].subfr01.reserved_bits_three = get_unsigned_32bits( msg, 121, 145 );
		//gps_subfrPart123[prn].subfr01.reserved_bits_four = get_unsigned_16bits( msg, 145, 161 );
		gps_subfrPart123[prn].subfr01.tgd = get_signed_8bits( msg, 161, 169, 0 );
		gps_subfrPart123[prn].subfr01.iode = get_unsigned_8bits( msg, 169, 177 );
		gps_subfrPart123[prn].subfr01.toc = get_unsigned_16bits( msg, 177, 193 );
		gps_subfrPart123[prn].subfr01.af2 = get_signed_8bits( msg, 193, 201, 0 );
		gps_subfrPart123[prn].subfr01.af1 = get_signed_16bits( msg, 201, 217, 0 );
		gps_subfrPart123[prn].subfr01.af0 = get_signed_32bits( msg , 217, 239, 0 );
		//gps_subfrPart123[prn].subfr01.parity_check_bits02 = get_unsigned_8bits( msg, 239, 241 );
	}

	//subframe two
	if( subfr_id == 2 )
	{
		//gps_subfrPart123[prn].subfr02.tlm_preamble = get_unsigned_8bits( msg , 1, 9 );
		//gps_subfrPart123[prn].subfr02.tlm_msg = get_unsigned_16bits( msg, 9, 23 );
		//gps_subfrPart123[prn].subfr02.tlm_integrity_status_flag = get_unsigned_8bits( msg, 23, 24);
		//gps_subfrPart123[prn].subfr02.tlm_reserved = get_unsigned_8bits( msg, 23, 25);
		gps_subfrPart123[prn].subfr02.tow = tow;
		gps_subfrPart123[prn].subfr02.alert_flag = get_unsigned_8bits( msg, 42, 43);
		gps_subfrPart123[prn].subfr02.anti_spoof_flag = get_unsigned_8bits( msg, 43, 44);
		gps_subfrPart123[prn].subfr02.subfr_id = get_unsigned_8bits( msg, 44, 47);
		//gps_subfrPart123[prn].subfr02.parity_check_bits01 = get_unsigned_8bits( msg , 47, 49);

		//��������
		gps_subfrPart123[prn].subfr02.iode = get_unsigned_8bits( msg, 49, 57 );
		gps_subfrPart123[prn].subfr02.crs = get_signed_16bits( msg, 57, 73, 0 );
		gps_subfrPart123[prn].subfr02.delta_N = get_signed_16bits( msg, 73, 89, 0 );
		gps_subfrPart123[prn].subfr02.m0 = get_signed_32bits( msg, 89, 121, 0 );
		gps_subfrPart123[prn].subfr02.cuc = get_signed_16bits( msg, 121, 137, 0 );
		gps_subfrPart123[prn].subfr02.e = get_unsigned_32bits( msg, 137, 169 );
		gps_subfrPart123[prn].subfr02.cus = get_signed_16bits( msg, 169, 185, 0 );
		gps_subfrPart123[prn].subfr02.sqrt_A = get_unsigned_32bits( msg, 185, 217 );
		gps_subfrPart123[prn].subfr02.toe = get_unsigned_16bits( msg, 217, 233 );
		gps_subfrPart123[prn].subfr02.fit_interval_flag = get_unsigned_8bits( msg, 233, 234 );
		gps_subfrPart123[prn].subfr02.aodo = get_unsigned_8bits( msg, 234, 239 );
		//gps_subfrPart123[prn].subfr02.parity_check_bits02 = get_unsigned_8bits( msg, 239, 241 );
	}//subfr_id==2
	
	//subframe three  
	//check the eph data update in this subframe
	if( subfr_id == 3 )
	{
		//gps_subfrPart123[prn].subfr03.tlm_preamble = get_unsigned_8bits( msg , 1, 9 );
		//gps_subfrPart123[prn].subfr03.tlm_msg = get_unsigned_16bits( msg, 9, 23 );
		//gps_subfrPart123[prn].subfr03.tlm_integrity_status_flag = get_unsigned_8bits( msg, 23, 24);
		//gps_subfrPart123[prn].subfr03.tlm_reserved = get_unsigned_8bits( msg, 23, 25);
		gps_subfrPart123[prn].subfr03.tow = tow;
		gps_subfrPart123[prn].subfr03.alert_flag = get_unsigned_8bits( msg, 42, 43);
		gps_subfrPart123[prn].subfr03.anti_spoof_flag = get_unsigned_8bits( msg, 43, 44);
		gps_subfrPart123[prn].subfr03.subfr_id = get_unsigned_8bits( msg, 44, 47);
		//gps_subfrPart123[prn].subfr03.parity_check_bits01 = get_unsigned_8bits( msg , 47, 49);

		//��������
		gps_subfrPart123[prn].subfr03.cic = get_signed_16bits( msg, 49, 65, 0 );
		gps_subfrPart123[prn].subfr03.omega0 = get_signed_32bits( msg, 65, 97, 0 );
		gps_subfrPart123[prn].subfr03.cis = get_signed_16bits( msg, 97, 113, 0 );
		gps_subfrPart123[prn].subfr03.i0 = get_signed_32bits( msg, 113, 145, 0 );
		gps_subfrPart123[prn].subfr03.crc = get_signed_16bits( msg, 145, 161, 0 );
		gps_subfrPart123[prn].subfr03.w = get_signed_32bits( msg, 161, 193, 0 );
		gps_subfrPart123[prn].subfr03.omega_dot = get_signed_32bits( msg, 193, 217, 0 );
		gps_subfrPart123[prn].subfr03.iode = get_unsigned_8bits( msg, 217, 225 );
		gps_subfrPart123[prn].subfr03.idot = get_signed_16bits( msg, 225, 239, 0 );
		//gps_subfrPart123[prn].subfr03.parity_check_bits02 = get_unsigned_8bits( msg, 239, 241 );
	}

	//check whether the eph data is ready
	if( gps_subfrPart123[prn].subfr01.subfr_id == 1 &&
		gps_subfrPart123[prn].subfr02.subfr_id == 2 && 
		gps_subfrPart123[prn].subfr03.subfr_id == 3 &&
		( gps_subfrPart123[prn].subfr01.iode == gps_subfrPart123[prn].subfr02.iode ) &&
		( gps_subfrPart123[prn].subfr01.iode == gps_subfrPart123[prn].subfr03.iode ) )
	{
		flag[0] = 1;
		return TRUE;
	}

	//for QZSS ,do not parse almanac
	if( prn + 1 > MAX_GPS_PRN ){
		return TRUE;
	}

	//subframe four
	//check the iono data update in this subframe
	if( subfr_id == 4 )
	{
		uint8_t data_id,svID;
		//ҳ�����
		data_id = get_unsigned_8bits( msg, 49, 51 );
		svID = get_unsigned_8bits( msg, 51, 57 );

		//��Ӧ������֡�е�2��3��4��5��7��8��9��10ҳ
		if( svID >= 25 && svID <= 32 )
		{
			//gps_subfrPart45[prn].subfr04.pageT8[svID - 25].tlm_preamble = get_unsigned_8bits( msg , 1, 9 );
			//gps_subfrPart45[prn].subfr04.pageT8[svID - 25].tlm_msg = get_unsigned_16bits( msg, 9, 23 );
			//gps_subfrPart45[prn].subfr04.pageT8[svID - 25].tlm_integrity_status_flag = get_unsigned_8bits( msg, 23, 24);
			//gps_subfrPart45[prn].subfr04.pageT8[svID - 25].tlm_reserved = get_unsigned_8bits( msg, 23, 25);
			gps_subfrPart45[prn].subfr04.pageT8[svID - 25].tow = tow;
			gps_subfrPart45[prn].subfr04.pageT8[svID - 25].alert_flag = get_unsigned_8bits( msg, 42, 43);
			gps_subfrPart45[prn].subfr04.pageT8[svID - 25].anti_spoof_flag = get_unsigned_8bits( msg, 43, 44);
			gps_subfrPart45[prn].subfr04.pageT8[svID - 25].subfr_id = get_unsigned_8bits( msg, 44, 47);
			//gps_subfrPart45[prn].subfr04.pageT8[svID - 25].parity_check_bits01 = get_unsigned_8bits( msg , 47, 49);
			gps_subfrPart45[prn].subfr04.pageT8[svID - 25].data_id = data_id;
			gps_subfrPart45[prn].subfr04.pageT8[svID - 25].sv_id = svID;


			//��������
			gps_subfrPart45[prn].subfr04.pageT8[svID - 25].e = get_unsigned_16bits( msg, 57, 73 );
			gps_subfrPart45[prn].subfr04.pageT8[svID - 25].toa = get_unsigned_8bits( msg, 73, 81 );
			gps_subfrPart45[prn].subfr04.pageT8[svID - 25].sigma_i = get_signed_16bits( msg, 81, 97, 0 );
			gps_subfrPart45[prn].subfr04.pageT8[svID - 25].omega_dot = get_signed_16bits( msg, 97, 113, 0 );
			gps_subfrPart45[prn].subfr04.pageT8[svID - 25].health = get_unsigned_8bits( msg, 113, 121 );
			gps_subfrPart45[prn].subfr04.pageT8[svID - 25].sqrt_A = get_unsigned_32bits( msg, 121, 145 );
			gps_subfrPart45[prn].subfr04.pageT8[svID - 25].omega0 = get_signed_32bits( msg, 145, 169, 0 );
			gps_subfrPart45[prn].subfr04.pageT8[svID - 25].w = get_signed_32bits( msg, 169, 193, 0 );
			gps_subfrPart45[prn].subfr04.pageT8[svID - 25].m0 = get_signed_32bits( msg, 193, 217, 0 );
			gps_subfrPart45[prn].subfr04.pageT8[svID - 25].af0 = ( (int16_t)( (get_unsigned_8bits(msg,217,225) << 8) | (get_unsigned_8bits(msg,236,239) << 5) ) ) >> 5;
			gps_subfrPart45[prn].subfr04.pageT8[svID - 25].af1 = get_signed_16bits( msg, 225, 236, 0 );
			//gps_subfrPart45[prn].subfr04.pageT8[svID - 25].parity_check_bits02 = get_unsigned_8bits( msg, 239, 241 );
		}

		//��Ӧ��18ҳ
		if( svID == 56 )  
		{
			//gps_subfrPart45[prn].subfr04.pageP18.tlm_preamble = get_unsigned_8bits( msg , 1, 9 );
			//gps_subfrPart45[prn].subfr04.pageP18.tlm_msg = get_unsigned_16bits( msg, 9, 23 );
			//gps_subfrPart45[prn].subfr04.pageP18.tlm_integrity_status_flag = get_unsigned_8bits( msg, 23, 24);
			//gps_subfrPart45[prn].subfr04.pageP18.tlm_reserved = get_unsigned_8bits( msg, 23, 25);
			gps_subfrPart45[prn].subfr04.pageP18.tow = tow;
			gps_subfrPart45[prn].subfr04.pageP18.alert_flag = get_unsigned_8bits( msg, 42, 43);
			gps_subfrPart45[prn].subfr04.pageP18.anti_spoof_flag = get_unsigned_8bits( msg, 43, 44);
			gps_subfrPart45[prn].subfr04.pageP18.subfr_id = get_unsigned_8bits( msg, 44, 47);
			//gps_subfrPart45[prn].subfr04.pageP18.parity_check_bits01 = get_unsigned_8bits( msg , 47, 49);
			gps_subfrPart45[prn].subfr04.pageP18.data_id = data_id;
			gps_subfrPart45[prn].subfr04.pageP18.sv_id = svID;

			//��������
			gps_subfrPart45[prn].subfr04.pageP18.alfa0 = get_signed_8bits( msg, 57, 65, 0 );
			gps_subfrPart45[prn].subfr04.pageP18.alfa1 = get_signed_8bits( msg, 65, 73, 0 );
			gps_subfrPart45[prn].subfr04.pageP18.alfa2 = get_signed_8bits( msg, 73, 81, 0 );
			gps_subfrPart45[prn].subfr04.pageP18.alfa3 = get_signed_8bits( msg, 81, 89, 0 );
			gps_subfrPart45[prn].subfr04.pageP18.beta0 = get_signed_8bits( msg, 89, 97, 0 );
			gps_subfrPart45[prn].subfr04.pageP18.beta1 = get_signed_8bits( msg, 97, 105, 0 );
			gps_subfrPart45[prn].subfr04.pageP18.beta2 = get_signed_8bits( msg, 105, 113, 0 );
			gps_subfrPart45[prn].subfr04.pageP18.beta3 = get_signed_8bits( msg, 113, 121, 0 );
			gps_subfrPart45[prn].subfr04.pageP18.A1 = get_signed_32bits( msg, 121, 145, 0 );
			gps_subfrPart45[prn].subfr04.pageP18.A0 = get_signed_32bits( msg, 145, 177, 0 );
			gps_subfrPart45[prn].subfr04.pageP18.tot = get_unsigned_8bits( msg, 177, 185 );
			gps_subfrPart45[prn].subfr04.pageP18.week = get_unsigned_8bits( msg, 185, 193 ); 
			gps_subfrPart45[prn].subfr04.pageP18.deltT_LS = get_signed_8bits( msg, 193, 201, 0 );
			gps_subfrPart45[prn].subfr04.pageP18.week_LSF = get_unsigned_8bits( msg, 201, 209 );
			gps_subfrPart45[prn].subfr04.pageP18.DN = get_signed_8bits( msg, 209, 217, 0 );
			gps_subfrPart45[prn].subfr04.pageP18.deltT_LSF = get_signed_8bits( msg, 217, 225, 0 );
			//gps_subfrPart45[prn].subfr04.pageP18.reserved_bits_one = get_unsigned_16bits( msg, 225, 239 );
			//gps_subfrPart45[prn].subfr04.pageP18.parity_check_bits02 = get_unsigned_8bits( msg, 239, 241 );

			//IONO data update flag
			flag[2] = 1;
			return TRUE;
		}

		//��Ӧ��25ҳ
		if( svID == 63 )
		{
			//gps_subfrPart45[prn].subfr04.pageP25.tlm_preamble = get_unsigned_8bits( msg , 1, 9 );
			//gps_subfrPart45[prn].subfr04.pageP25.tlm_msg = get_unsigned_16bits( msg, 9, 23 );
			//gps_subfrPart45[prn].subfr04.pageP25.tlm_integrity_status_flag = get_unsigned_8bits( msg, 23, 24);
			//gps_subfrPart45[prn].subfr04.pageP25.tlm_reserved = get_unsigned_8bits( msg, 23, 25);
			gps_subfrPart45[prn].subfr04.pageP25.tow = tow;
			gps_subfrPart45[prn].subfr04.pageP25.alert_flag = get_unsigned_8bits( msg, 42, 43);
			gps_subfrPart45[prn].subfr04.pageP25.anti_spoof_flag = get_unsigned_8bits( msg, 43, 44);
			gps_subfrPart45[prn].subfr04.pageP25.subfr_id = get_unsigned_8bits( msg, 44, 47);
			//gps_subfrPart45[prn].subfr04.pageP25.parity_check_bits01 = get_unsigned_8bits( msg , 47, 49);
			gps_subfrPart45[prn].subfr04.pageP25.data_id = data_id;
			gps_subfrPart45[prn].subfr04.pageP25.sv_id = svID;


			//��������
			for( i = 0; i < 32; i++)
			{
				gps_subfrPart45[prn].subfr04.pageP25.aSpoof_sv[i] = get_unsigned_8bits( msg, 57 + i*4, 61 + i*4 );
			}

			gps_subfrPart45[prn].subfr04.pageP25.reserved_bits_one = get_unsigned_8bits( msg, 185, 187 );

			for( i = 0; i < 8; i++ )
			{
				gps_subfrPart45[prn].subfr04.pageP25.health_sv[i] = get_unsigned_8bits( msg, 187 + 6*i, 193 + 6*i );
			}
			//gps_subfrPart45[prn].subfr04.pageP25.reserved_bits_two = get_unsigned_8bits( msg, 235, 239 );
			//gps_subfrPart45[prn].subfr04.pageP25.parity_check_bits02 = get_unsigned_8bits( msg, 239, 241 );
		}
	}//subfr_id ==4

	//subframe five
	//check the alm data update in this subframe
	if( subfr_id == 5 )
	{
		uint8_t data_id,svID;
		//ҳ�����
		data_id = get_unsigned_8bits( msg, 49, 51 );
		svID = get_unsigned_8bits( msg, 51, 57 );

		//��Ӧ��1-24ҳ
		if( svID > 0 && svID < 25 )
		{
			//gps_subfrPart45[prn].subfr05.pageT24[svID - 1].tlm_preamble = get_unsigned_8bits( msg , 1, 9 );
			//gps_subfrPart45[prn].subfr05.pageT24[svID - 1].tlm_msg = get_unsigned_16bits( msg, 9, 23 );
			//gps_subfrPart45[prn].subfr05.pageT24[svID - 1].tlm_integrity_status_flag = get_unsigned_8bits( msg, 23, 24);
			//gps_subfrPart45[prn].subfr05.pageT24[svID - 1].tlm_reserved = get_unsigned_8bits( msg, 23, 25);
			gps_subfrPart45[prn].subfr05.pageT24[svID - 1].tow = tow;
			gps_subfrPart45[prn].subfr05.pageT24[svID - 1].alert_flag = get_unsigned_8bits( msg, 42, 43);
			gps_subfrPart45[prn].subfr05.pageT24[svID - 1].anti_spoof_flag = get_unsigned_8bits( msg, 43, 44);
			gps_subfrPart45[prn].subfr05.pageT24[svID - 1].subfr_id = get_unsigned_8bits( msg, 44, 47);
			//gps_subfrPart45[prn].subfr05.pageT24[svID - 1].parity_check_bits01 = get_unsigned_8bits( msg , 47, 49);
			gps_subfrPart45[prn].subfr05.pageT24[svID - 1].data_id = data_id;
			gps_subfrPart45[prn].subfr05.pageT24[svID - 1].sv_id = svID;

			//��������
			gps_subfrPart45[prn].subfr05.pageT24[svID - 1].e = get_unsigned_16bits( msg, 57, 73 );
			gps_subfrPart45[prn].subfr05.pageT24[svID - 1].toa = get_unsigned_8bits( msg, 73, 81 );
			gps_subfrPart45[prn].subfr05.pageT24[svID - 1].sigma_i = get_signed_16bits( msg, 81, 97, 0 );
			gps_subfrPart45[prn].subfr05.pageT24[svID - 1].omega_dot = get_signed_16bits( msg, 97, 113, 0 );
			gps_subfrPart45[prn].subfr05.pageT24[svID - 1].health = get_unsigned_8bits( msg, 113, 121 );
			gps_subfrPart45[prn].subfr05.pageT24[svID - 1].sqrt_A = get_unsigned_32bits( msg, 121, 145 );
			gps_subfrPart45[prn].subfr05.pageT24[svID - 1].omega0 = get_signed_32bits( msg, 145, 169, 0 );
			gps_subfrPart45[prn].subfr05.pageT24[svID - 1].w = get_signed_32bits( msg, 169, 193, 0 );
			gps_subfrPart45[prn].subfr05.pageT24[svID - 1].m0 = get_signed_32bits( msg, 193, 217, 0 );
			gps_subfrPart45[prn].subfr05.pageT24[svID - 1].af0 = ( (int16_t)( (get_unsigned_8bits(msg,217,225) << 8) | (get_unsigned_8bits(msg,236,239) << 5) ) ) >> 5;
			gps_subfrPart45[prn].subfr05.pageT24[svID - 1].af1 = get_signed_16bits( msg, 225, 236, 0 );
			//gps_subfrPart45[prn].subfr05.pageT24[svID - 1].parity_check_bits02 = get_unsigned_8bits( msg, 239, 241 );
		}

		//��Ӧ��25ҳ
		if( svID == 51 )
		{
			uint32_t dt;
			//gps_subfrPart45[prn].subfr05.pageP25.tlm_preamble = get_unsigned_8bits( msg , 1, 9 );
			//gps_subfrPart45[prn].subfr05.pageP25.tlm_msg = get_unsigned_16bits( msg, 9, 23 );
			//gps_subfrPart45[prn].subfr05.pageP25.tlm_integrity_status_flag = get_unsigned_8bits( msg, 23, 24);
			//gps_subfrPart45[prn].subfr05.pageP25.tlm_reserved = get_unsigned_8bits( msg, 23, 25);
			gps_subfrPart45[prn].subfr05.pageP25.tow = tow;
			gps_subfrPart45[prn].subfr05.pageP25.alert_flag = get_unsigned_8bits( msg, 42, 43);
			gps_subfrPart45[prn].subfr05.pageP25.anti_spoof_flag = get_unsigned_8bits( msg, 43, 44);
			gps_subfrPart45[prn].subfr05.pageP25.subfr_id = get_unsigned_8bits( msg, 44, 47);
			//gps_subfrPart45[prn].subfr05.pageP25.parity_check_bits01 = get_unsigned_8bits( msg , 47, 49);
			gps_subfrPart45[prn].subfr05.pageP25.data_id = data_id;
			gps_subfrPart45[prn].subfr05.pageP25.sv_id = svID;

			//��������
			gps_subfrPart45[prn].subfr05.pageP25.toa = get_unsigned_8bits( msg, 57, 65 );
			gps_subfrPart45[prn].subfr05.pageP25.wna = get_unsigned_8bits( msg, 65, 73 );
			for( i = 0; i < 24; i++ )
			{
				gps_subfrPart45[prn].subfr05.pageP25.health_sv[i] = get_unsigned_8bits( msg, 73 + i*6, 79 + i*6 );
			}
			//gps_subfrPart45[prn].subfr05.pageP25.reserved_bits_one = get_unsigned_8bits( msg, 217, 223 );
			//gps_subfrPart45[prn].subfr05.pageP25.reserved_bits_two = get_unsigned_16bits( msg, 223, 239 );
			//gps_subfrPart45[prn].subfr05.pageP25.parity_check_bits02 = get_unsigned_8bits( msg, 239, 241 );

			//check tow and delete the sate out of range
			//check data in subfame 5
			for( i = 0; i < 24; i++ ){
				dt = gps_subfrPart45[prn].subfr05.pageP25.tow - gps_subfrPart45[prn].subfr05.pageT24[i].tow;
				if( dt != (24 - i)*5 )
					memset( &gps_subfrPart45[prn].subfr05.pageT24[i], 0, sizeof(subfr45_page24_GPS_t) );
			}

			//check data in subframe 4
			for( i = 0; i < 8; i++ ){
				if( i < 4 ) j = i + 1;
				else        j = i + 2;

				dt = gps_subfrPart45[prn].subfr05.pageP25.tow - gps_subfrPart45[prn].subfr04.pageT8[i].tow;
				if( dt != (24 - j)*5 + 1 )
					memset( &gps_subfrPart45[prn].subfr04.pageT8[i], 0, sizeof(subfr45_page24_GPS_t) );
			}

			flag[1] = 1;
			return TRUE;
		}
	} //subfr_id == 5

	return TRUE;
}
//*****************GLONASS_PART************************
uint8_t novAtel_glnStringParityCheck( uint32_t* pWord )
{
	uint8_t     i;
	uint8_t     sum;
	uint8_t     sum1;
	uint8_t     C[8];
	uint8_t     idx[8];
	uint8_t     nIdx = 0;
	uint8_t     tbits[85];
	uint8_t     bits[85];
	uint8_t     beta[8];
	uint8_t     b[85] = {0};
	uint8_t     bitsum ;
	uint8_t     ii[] = {9, 10, 12, 13, 15, 17, 19, 20, 22, 24, 26, 28, 30, 32, 34, 35, 37, 39, 41, 43, 45, 47, 49, 51, 53, 55, 57, 59, 61, 63, 65, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84};
	uint8_t     jj[] = {9, 11, 12, 14, 15, 18, 19, 21, 22, 25, 26, 29, 30, 33, 34, 36, 37, 40, 41, 44, 45, 48, 49, 52, 53, 56, 57, 60, 61, 64, 65, 67, 68, 71, 72, 75, 76, 79, 80, 83, 84};
	uint8_t     kk[] = {10, 11, 12, 16, 17, 18, 19, 23, 24, 25, 26, 31, 32, 33, 34, 38, 39, 40, 41, 46, 47, 48, 49, 54, 55, 56, 57, 62, 63, 64, 65, 69, 70, 71, 72, 77, 78, 79, 80, 85};
	uint8_t     ll[] = {13, 14, 15, 16, 17, 18, 19, 27, 28, 29, 30, 31, 32, 33, 34, 42, 43, 44, 45, 46, 47, 48, 49, 58, 59, 60, 61, 62, 63, 64, 65, 73, 74, 75, 76, 77, 78, 79, 80};
	uint8_t     mm[] = {20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 81, 82, 83, 84, 85};
	uint8_t     nn[] = {35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65};
	uint8_t     pp[] = {66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85};
	uint32_t    RetVal = TRUE;
	bitsum = 0;

	for (i = 0; i < 32; i++)
	{
		tbits[31 - i] = (pWord[0] >> i) & 1;
	}

	for (i = 0; i < 32; i++)
	{
		tbits[31 - i + 32] = (pWord[1] >> i) & 1;
	}

	for (i = 0; i < 21; i++)
	{
		tbits[20 - i + 64] = (pWord[2] >> (i + 11)) & 1;
	}

	for (i = 0; i < 85; i++)
	{
		bits[i] = tbits[84 - i];
	}

	for (i = 0; i < 8; i++)
	{
		beta[i] = bits[i];
	}

	for (i = 8; i <= 84; i++)
	{
		b[i] = bits[i];
	}

	sum = 0;

	for (i = 0; i < 41; i++)
	{
		sum += b[ii[i] - 1];
	}

	C[0] = beta[0] ^ (sum % 2);
	sum = 0;

	for (i = 0; i < 41; i++)
	{
		sum += b[jj[i] - 1];
	}

	C[1] = beta[1] ^ (sum % 2);
	sum = 0;

	for (i = 0; i < 40; i++)
	{
		sum += b[kk[i] - 1];
	}

	C[2] = beta[2] ^ (sum % 2);
	sum = 0;

	for (i = 0; i < 39; i++)
	{
		sum += b[ll[i] - 1];
	}

	C[3] = beta[3] ^ (sum % 2);
	sum = 0;

	for (i = 0; i < 36; i++)
	{
		sum += b[mm[i] - 1];
	}

	C[4] = beta[4] ^ (sum % 2);
	sum = 0;

	for (i = 0; i < 31; i++)
	{
		sum += b[nn[i] - 1];
	}

	C[5] = beta[5] ^ (sum % 2);
	sum = 0;

	for (i = 0; i < 20; i++)
	{
		sum += b[pp[i] - 1];
	}

	C[6] = beta[6] ^ (sum % 2);
	sum = 0;

	for (i = 9; i <= 85; i++)
	{
		sum += b[i - 1];
	}

	sum1 = 0;

	for (i = 1; i <= 8; i++)
	{
		sum1 += beta[i - 1];
	}

	C[7] = (sum1 % 2) ^ (sum % 2); //CSum=xor(mod(sum(beta(1:8)),2),  mod(sum(b(9:85)),2));

	for (i = 0; i <= 6; i++)
	{
		if (C[i] == 1)
		{
			idx[nIdx++] = i;
		}
	}

	for (i = 0; i < 80; i++)
	{
		bitsum += bits[i];
	}

	if ((nIdx == 0 && C[7] == 0)) //
	{
		RetVal = TRUE;
	}
	else
	{
		RetVal = FALSE;
	}

	return RetVal;
}

uint8_t novAtel_glnHanM_Check( uint8_t *msg )
{
	//�ֲ�����
	uint32_t word[3]={0};
	uint32_t u32a,u32b,u32c;

	//������
	if( msg == NULL )
	{
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs err!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}

	//**************************************
	//���Ƚ�8λ���ֽ�ת��32λ
	u32a = (uint32_t)( ( msg[0] << 24 ) | ( msg[1] << 16 ) | ( msg[2] << 8 ) | msg[3] );
	u32b = (uint32_t)( ( msg[4] << 24 ) | ( msg[5] << 16 ) | ( msg[6] << 8 ) | msg[7] );
	u32c = (uint32_t)( ( msg[8] << 24 ) | ( msg[9] << 16 ) | ( msg[10] << 8 ) | msg[11] );

	word[0] = (uint32_t)( ( u32a << 3 ) | ( u32b >> 29 ) );
	word[1] = (uint32_t)( ( u32b << 3 ) | ( u32c >> 29 ) );
	word[2] = (uint32_t)( u32c << 3 );


	if( novAtel_glnStringParityCheck( word ) == FALSE )
	{
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"HanMing Check failed,%s,%d\n",__FUNCTION__,__LINE__);
		return FALSE;
	}

	//У��ͨ��
	return TRUE;
}

uint8_t exchange_glo_alm( stringPart_Alm_t glo_strPart_alm, timeRef_GLO_t glo_timeRef, GLN_ALM_INFO *glo_alm )
{
	//local paras

	//inputs check;
	if( glo_alm == NULL ){
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs error,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}

	//N4 check
	if( glo_timeRef.flagN == 0 ){
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"para N4 isn't ready,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}

	//init outputs
	memset( glo_alm,0,sizeof(GLN_ALM_INFO) );

	//*******************
	glo_alm->alm_status  = TRUE;       //valid flag
	glo_alm->AlmFlag     = 0;          //collect status flag
	glo_alm->Cn          = glo_strPart_alm.str6.CA;                                      
	glo_alm->MnA         = glo_strPart_alm.str6.MA;                   
	glo_alm->nA          = glo_strPart_alm.str6.nA;                    
	glo_alm->HnA         = glo_strPart_alm.str7.HA;                   
	glo_alm->ln          = glo_strPart_alm.str7.ln;                    
                    
	glo_alm->TaunA       = glo_strPart_alm.str6.tauA*P2_20*4;                 
	glo_alm->LamdanA     = glo_strPart_alm.str6.lambdaA*P2_20*PI;               
	glo_alm->DeltainA    = glo_strPart_alm.str6.delt_iA*P2_20*PI;              
	glo_alm->EpsilonnA   = glo_strPart_alm.str6.epsilonA*P2_20;             

	glo_alm->wnA         = glo_strPart_alm.str7.omegaA*P2_15*PI;                   
	glo_alm->tLamdanA    = glo_strPart_alm.str7.t_lambdaA*P2_05;              
	glo_alm->DeltaTnA    = glo_strPart_alm.str7.deltTA*P2_11*4;              
	glo_alm->DeltaTdotnA = glo_strPart_alm.str7.deltTA_dot*P2_15*2;           

	//first check para N_A/N4/B1/B2 
	if( glo_timeRef.flagN != 0 ){
		glo_alm->N_A     = glo_timeRef.N_A;                   
		glo_alm->N4      = glo_timeRef.N4;  
	}
	if( glo_timeRef.flagB != 0 ){
		glo_alm->B1     = glo_timeRef.B1*P2_11*2;                   
		glo_alm->B2     = glo_timeRef.B2*P2_17*2;  
	}
    
	return TRUE;
}

uint8_t exchange_glo_alm_N( stringPart_Alm_N_t *glo_strPartAlm_N, uint8_t tar_pos, GLN_ALM_INFO *glo_alm ){
	//inputs check;
	if( glo_alm == NULL || glo_strPartAlm_N == NULL ){
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs error,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}
	memset( glo_alm,0,sizeof(GLN_ALM_INFO) );
	//***********
	glo_alm->alm_status  = TRUE;        //valid flag
	glo_alm->AlmFlag     = 0;       //collect status flag
	glo_alm->Cn          = glo_strPartAlm_N->str6[tar_pos].CA;                                      
	glo_alm->MnA         = glo_strPartAlm_N->str6[tar_pos].MA;                   
	glo_alm->nA          = glo_strPartAlm_N->str6[tar_pos].nA;                    
	glo_alm->HnA         = glo_strPartAlm_N->str7[tar_pos].HA;                   
	glo_alm->ln          = glo_strPartAlm_N->str7[tar_pos].ln;                    

	glo_alm->TaunA       = glo_strPartAlm_N->str6[tar_pos].tauA*P2_20*4;                 
	glo_alm->LamdanA     = glo_strPartAlm_N->str6[tar_pos].lambdaA*P2_20*PI;               
	glo_alm->DeltainA    = glo_strPartAlm_N->str6[tar_pos].delt_iA*P2_20*PI;              
	glo_alm->EpsilonnA   = glo_strPartAlm_N->str6[tar_pos].epsilonA*P2_20;             

	glo_alm->wnA         = glo_strPartAlm_N->str7[tar_pos].omegaA*P2_15*PI;                   
	glo_alm->tLamdanA    = glo_strPartAlm_N->str7[tar_pos].t_lambdaA*P2_05;              
	glo_alm->DeltaTnA    = glo_strPartAlm_N->str7[tar_pos].deltTA*P2_11*4;              
	glo_alm->DeltaTdotnA = glo_strPartAlm_N->str7[tar_pos].deltTA_dot*P2_15*2;   

	glo_alm->N_A         = glo_strPartAlm_N->str5.NA;                  
	glo_alm->N4          = glo_strPartAlm_N->str5.N4;

	//clear cache
	memset( &glo_strPartAlm_N->str6[tar_pos], 0, sizeof(string6_GLO_t) );
	memset( &glo_strPartAlm_N->str7[tar_pos], 0, sizeof(string7_GLO_t) );

	return TRUE;
}

uint8_t exchange_glo_eph( stringPart_Eph_t glo_strPart_eph, GLN_EPH_INFO *glo_eph )
{
	//local paras

	//check inputs
	if(  glo_eph == NULL ){
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs error,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}

	//init outputs;
	memset( glo_eph, 0, sizeof(GLN_EPH_INFO) );

	//**************************
	glo_eph->eph_status = EPH_STATUS_VALID;              //??    
	glo_eph->eph_source = EPH_SOURCE_BRDC;              /*0-broadcast 1-agps;2-EE */
	glo_eph->EphDecFlag = 0;
	glo_eph->Health     = glo_strPart_eph.str3.ln;


	glo_eph->NT         = glo_strPart_eph.str4.NT;
	glo_eph->NA         = glo_strPart_eph.str5.NA;
	glo_eph->N4         = glo_strPart_eph.str5.N4;

	glo_eph->M          = glo_strPart_eph.str4.M ;                     /*00: GLONASS, 01:GLONASS-M1 */
	glo_eph->P1         = glo_strPart_eph.str1.P1;                     /*00:0 min;01:30 min; 10:45 min;11:60 min; */
	glo_eph->P2         = glo_strPart_eph.str2.P2;                    
	glo_eph->P3         = glo_strPart_eph.str3.P3;
	glo_eph->P4         = glo_strPart_eph.str4.P4;
	glo_eph->P          = glo_strPart_eph.str3.P ;
	glo_eph->Bn         = glo_strPart_eph.str2.Bn;
	glo_eph->ln1        = glo_strPart_eph.str3.ln;
	glo_eph->ln2        = glo_strPart_eph.str5.ln;
	glo_eph->FT         = glo_strPart_eph.str4.FT;
	glo_eph->n          = glo_strPart_eph.str4.n ;
	glo_eph->En         = glo_strPart_eph.str4.E ;

	/* satellite acceleration */
	glo_eph->a[0]       = glo_strPart_eph.str1.x_dot2*P2_30*1000;   //Unit:M
	glo_eph->a[1]       = glo_strPart_eph.str2.y_dot2*P2_30*1000;
	glo_eph->a[2]       = glo_strPart_eph.str3.z_dot2*P2_30*1000;
	/* satellite velocity  */
	glo_eph->v[0]       = glo_strPart_eph.str1.x_dot*P2_20*1000 ; 
	glo_eph->v[1]       = glo_strPart_eph.str2.y_dot*P2_20*1000 ;
	glo_eph->v[2]       = glo_strPart_eph.str3.z_dot*P2_20*1000 ;
	/* satellite position */
	glo_eph->r[0]       = glo_strPart_eph.str1.x*P2_11*1000;
	glo_eph->r[1]       = glo_strPart_eph.str2.y*P2_11*1000;
	glo_eph->r[2]       = glo_strPart_eph.str3.z*P2_11*1000;

	glo_eph->tk         = glo_strPart_eph.str1.tk;
	glo_eph->tb         = (double)glo_strPart_eph.str2.tb*15*60; //unit:s

	glo_eph->gaman_tb   = glo_strPart_eph.str3.gamma*P2_40;
	glo_eph->taun_tb    = glo_strPart_eph.str4.tau*P2_30;
	glo_eph->delta_taun = glo_strPart_eph.str4.delt_tau*P2_30;

	glo_eph->tauc       = glo_strPart_eph.str5.tauC*P2_31;
	glo_eph->tauGPS     = glo_strPart_eph.str5.tauGPS*P2_30;

	if( abs(glo_strPart_eph.str5.tauGPS) < 100 ){
		p_gnssTimeSysParam->tauGPS[GLN_MODE] = glo_eph->tauGPS;
		SYS_LOGGING(OBJ_SD,LOG_INFO,"GLN and GPS time diff : prn:%02d %16.12f",glo_eph->n,p_gnssTimeSysParam->tauGPS[GLN_MODE]);
	}

	return TRUE;
}

uint8_t exchange_glo_eph_N( stringPart_Eph_N_t *glo_strPartEph_N, GLN_EPH_INFO *glo_eph ){
	//check inputs
	if(  glo_eph == NULL || glo_strPartEph_N == NULL ){
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs error,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}
	memset( glo_eph, 0, sizeof(GLN_EPH_INFO) );
	//**************
	glo_eph->eph_status = EPH_STATUS_VALID;              //??    
	glo_eph->eph_source = EPH_SOURCE_BRDC;              /*0-broadcast 1-agps;2-EE */
	glo_eph->EphDecFlag = 0;
	glo_eph->Health     = glo_strPartEph_N->str3.ln;


	glo_eph->NT         = glo_strPartEph_N->str4.NT;
	glo_eph->NA         = 0;
	glo_eph->N4         = 0;

	glo_eph->M          = glo_strPartEph_N->str4.M ;                     /*00: GLONASS, 01:GLONASS-M1 */
	glo_eph->P1         = glo_strPartEph_N->str1.P1;                     /*00:0 min;01:30 min; 10:45 min;11:60 min; */
	glo_eph->P2         = glo_strPartEph_N->str2.P2;                    
	glo_eph->P3         = glo_strPartEph_N->str3.P3;
	glo_eph->P4         = glo_strPartEph_N->str4.P4;
	glo_eph->P          = glo_strPartEph_N->str3.P ;
	glo_eph->Bn         = glo_strPartEph_N->str2.Bn;
	glo_eph->ln1        = glo_strPartEph_N->str3.ln;
	glo_eph->ln2        = 0;
	glo_eph->FT         = glo_strPartEph_N->str4.FT;
	glo_eph->n          = glo_strPartEph_N->str4.n ;
	glo_eph->En         = glo_strPartEph_N->str4.E ;

	/* satellite acceleration */
	glo_eph->a[0]       = glo_strPartEph_N->str1.x_dot2*P2_30*1000;   //Unit:M
	glo_eph->a[1]       = glo_strPartEph_N->str2.y_dot2*P2_30*1000;
	glo_eph->a[2]       = glo_strPartEph_N->str3.z_dot2*P2_30*1000;
	/* satellite velocity  */
	glo_eph->v[0]       = glo_strPartEph_N->str1.x_dot*P2_20*1000 ; 
	glo_eph->v[1]       = glo_strPartEph_N->str2.y_dot*P2_20*1000 ;
	glo_eph->v[2]       = glo_strPartEph_N->str3.z_dot*P2_20*1000 ;
	/* satellite position */
	glo_eph->r[0]       = glo_strPartEph_N->str1.x*P2_11*1000;
	glo_eph->r[1]       = glo_strPartEph_N->str2.y*P2_11*1000;
	glo_eph->r[2]       = glo_strPartEph_N->str3.z*P2_11*1000;

	glo_eph->tk         = glo_strPartEph_N->str1.tk;
	glo_eph->tb         = (double)glo_strPartEph_N->str2.tb*15*60; //unit:s

	glo_eph->gaman_tb   = glo_strPartEph_N->str3.gamma*P2_40;
	glo_eph->taun_tb    = glo_strPartEph_N->str4.tau*P2_30;
	glo_eph->delta_taun = glo_strPartEph_N->str4.delt_tau*P2_30;

	glo_eph->tauc       = 0;
	glo_eph->tauGPS     = 0;

	//clear cache
	memset( glo_strPartEph_N, 0, sizeof(stringPart_Eph_N_t) );

	return TRUE;
}

uint8_t exchange_glo_utcModel_N( stringPart_Alm_N_t *glo_strPartAlm_N, glnUtcModel_t *gln_utc ){
	if(  gln_utc == NULL || glo_strPartAlm_N == NULL ){
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs error,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}
	memset( gln_utc, 0, sizeof(glnUtcModel_t) );
	//**************
	gln_utc->b1   = (double)(glo_strPartAlm_N->str14.B1*P2_11*2);
	gln_utc->b2   = (double)(glo_strPartAlm_N->str14.B2*P2_17*2);
	gln_utc->kp   = glo_strPartAlm_N->str14.KP;
	gln_utc->nA   = glo_strPartAlm_N->str5.NA;
	gln_utc->tauC = glo_strPartAlm_N->str5.tauC;
	gln_utc->have_utc = 1;

	//clear cache
	memset( &glo_strPartAlm_N->str14, 0, sizeof(string14_GLO_t) );

	return TRUE;
}

uint8_t msg_decode_glo(
	uint8_t *msg,
	int32_t *header,
	stringPart_Eph_t *glo_strPart_eph,
	stringPart_Alm_t *glo_strPart_alm,
	timeRef_GLO_t *glo_timeRef, 
	int32_t *flag )
{
	//inner paras
	//uint8_t firstBit;
	uint8_t  strNo;
	int32_t i,subfrID;
	uint8_t  prn;

	//inputs check
	if( msg == NULL || header == NULL || glo_strPart_eph == NULL || glo_strPart_alm == NULL || flag == NULL ){
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs err!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}

	// init outputs
	flag[0] = 0;
	flag[1] = 0;

	//************************************
	//first bit check
	//firstBit = get_unsigned_8bits( msg, 4, 5 );
	//if( firstBit != 0 ){
	//	printf("firstBit check error\n");
	//	return FALSE;
	//}

	//decode string one by one;
	subfrID = header[5];
	prn = (uint8_t)header[1];

	//decode dataBits by subfrID 
	//first the eph data
	if( subfrID == 5 ){

		//include five strings,decode one by one
		for( i = 0; i < GLOEPH_STRNUM_IN_MSG; i++ ){
			uint8_t temp[10] = {0};

			//cutoff target string
			memcpy( temp, &msg[i*10], 10 );

			//decode this string
			//check stringNo
			strNo = get_unsigned_8bits( temp, 5, 9 );
			if( strNo < 1 || strNo > 15 || (strNo != i+1) ){
				SYS_LOGGING(OBJ_SD,LOG_WARNING,"stringNumber check failed,%s,%d\n",__FUNCTION__,__LINE__);
				return FALSE;
			}

			//decode the dataBits according strNo
			if( strNo == 1 ){
				glo_strPart_eph->str1.stringNumber = strNo;
				glo_strPart_eph->str1.reserved_bits_one = get_unsigned_8bits( temp, 9, 11 );
				glo_strPart_eph->str1.P1 = get_unsigned_8bits( temp, 11, 13 );
				glo_strPart_eph->str1.tk = get_unsigned_16bits( temp, 13, 25 );
				glo_strPart_eph->str1.x_dot = get_signed_32bits( temp, 25, 49, 1 );
				glo_strPart_eph->str1.x_dot2 = get_signed_8bits( temp, 49, 54, 1 );
				glo_strPart_eph->str1.x = get_signed_32bits( temp, 54, 81, 1 );
			}

			//string 2
			if( strNo == 2 )
			{
				glo_strPart_eph->str2.stringNumber = strNo;
				glo_strPart_eph->str2.Bn = get_unsigned_8bits( temp, 9, 12 );
				glo_strPart_eph->str2.P2 = get_unsigned_8bits( temp, 12, 13 );
				glo_strPart_eph->str2.tb = get_unsigned_8bits( temp, 13, 20 );
				glo_strPart_eph->str2.reserved_bits_one = get_unsigned_8bits( temp, 20, 25 );
				glo_strPart_eph->str2.y_dot = get_signed_32bits( temp, 25, 49, 1 );
				glo_strPart_eph->str2.y_dot2 = get_signed_8bits( temp, 49, 54, 1 );
				glo_strPart_eph->str2.y = get_signed_32bits( temp, 54, 81, 1 );
			}

			//string 3
			if( strNo == 3 )
			{
				glo_strPart_eph->str3.stringNumber = strNo; 
				glo_strPart_eph->str3.P3 = get_unsigned_8bits( temp, 9 , 10 );
				glo_strPart_eph->str3.gamma = get_signed_16bits( temp, 10, 21, 1 );
				glo_strPart_eph->str3.reserved_bits_one = get_unsigned_8bits( temp, 21, 22 );
				glo_strPart_eph->str3.P = get_unsigned_8bits( temp, 22, 24 );
				glo_strPart_eph->str3.ln = get_unsigned_8bits( temp, 24, 25 );
				glo_strPart_eph->str3.z_dot = get_signed_32bits( temp, 25, 49, 1 );
				glo_strPart_eph->str3.z_dot2 = get_signed_8bits( temp, 49, 54, 1 );
				glo_strPart_eph->str3.z = get_signed_32bits( temp, 54, 81, 1 );
			}

			//string 4
			if( strNo == 4 )
			{
				glo_strPart_eph->str4.stringNumber = strNo; 
				glo_strPart_eph->str4.tau = get_signed_32bits( temp, 9, 31, 1 );
				glo_strPart_eph->str4.delt_tau = get_signed_8bits( temp, 31, 36, 1 );
				glo_strPart_eph->str4.E = get_unsigned_8bits( temp, 36, 41 );
				glo_strPart_eph->str4.reserved_bits_one = get_unsigned_16bits( temp, 41, 55 );
				glo_strPart_eph->str4.P4 = get_unsigned_8bits( temp, 55, 56 );
				glo_strPart_eph->str4.FT = get_unsigned_8bits( temp, 56, 60 );
				glo_strPart_eph->str4.reserved_bits_two = get_unsigned_8bits( temp, 60, 63 );
				glo_strPart_eph->str4.NT = get_unsigned_16bits( temp, 63, 74 );
				glo_strPart_eph->str4.n = get_unsigned_8bits( temp, 74, 79 );
				glo_strPart_eph->str4.M = get_unsigned_8bits( temp, 79, 81 );

				//prn���
				if( prn != glo_strPart_eph->str4.n ){
					SYS_LOGGING(OBJ_SD,LOG_WARNING,"glonass svID compare failed,%s,%d\n",__FUNCTION__,__LINE__);
				    return FALSE;
				}
			}

			//��5
			if( strNo == 5 )
			{
				glo_strPart_eph->str5.stringNumber = strNo; 
				glo_strPart_eph->str5.NA = get_unsigned_16bits( temp, 9, 20 );
				glo_strPart_eph->str5.tauC = get_signed_32bits( temp, 20, 52, 1 );
				glo_strPart_eph->str5.reserved_bits_one = get_unsigned_8bits( temp, 52, 53 );
				glo_strPart_eph->str5.N4 = get_unsigned_8bits( temp, 53, 58 );
				glo_strPart_eph->str5.tauGPS = get_signed_32bits( temp, 58, 80, 1 );
				glo_strPart_eph->str5.ln = get_unsigned_8bits( temp, 80, 81 );
			}
		}

		//update mark
		if( glo_strPart_eph->str1.stringNumber ==  1 &&
			glo_strPart_eph->str2.stringNumber ==  2 &&
			glo_strPart_eph->str3.stringNumber ==  3 &&
			glo_strPart_eph->str4.stringNumber ==  4 &&
			glo_strPart_eph->str5.stringNumber ==  5 )
		{
				flag[0] = 1;
		}
	}

	//second the alm data
	if( subfrID == 2 ){
		string6_GLO_t str6;
		string7_GLO_t str7;
		uint8_t nA = 0;

		memset(&str6,0,sizeof(string6_GLO_t));
		memset(&str7,0,sizeof(string7_GLO_t));

		//include two strings,decode one by one
		for( i = 0; i < GLOALM_STRNUM_IN_MSG; i++ ){
			uint8_t temp[10] = {0};

			//cutoff one string
            memcpy( temp, &msg[i*10], 10 );

			strNo = get_unsigned_8bits( temp, 5, 9 );
			if( strNo < 6 || strNo > 15 ){
				SYS_LOGGING(OBJ_SD,LOG_WARNING,"stringNumber check failed,%s,%d\n",__FUNCTION__,__LINE__);
				return FALSE;
			}

			if( strNo == 6 || strNo == 8 || strNo == 10 || strNo == 12 || strNo == 14 ){

				//find prn
				str6.stringNumber = strNo; 
				str6.CA =get_unsigned_8bits( temp, 9 ,10 );
				str6.MA = get_unsigned_8bits( temp, 10, 12 );
				str6.nA = get_unsigned_8bits( temp, 12, 17 );
				str6.tauA = get_signed_16bits( temp, 17, 27, 1 );
				str6.lambdaA = get_signed_32bits( temp, 27, 48, 1 );
				str6.delt_iA = get_signed_32bits( temp, 48, 66, 1 );
			    str6.epsilonA = get_signed_16bits( temp, 66, 81, 1 );
				nA = str6.nA;
				//check nA;

			}

			if( strNo == 7 || strNo == 9 || strNo == 11 || strNo == 13 || strNo == 15 ){
				str7.stringNumber = strNo; 
				str7.omegaA = get_signed_16bits( temp, 9, 25, 1 );
				str7.t_lambdaA = get_unsigned_32bits( temp, 25, 46 );
				str7.deltTA = get_signed_32bits( temp, 46, 68, 1 );
				str7.deltTA_dot = get_signed_8bits( temp, 68, 75, 1 );
				str7.HA = get_unsigned_8bits( temp, 75, 80 );
				str7.ln = get_unsigned_8bits( temp, 80, 81 );
			}
		}

		//save data
		if( str6.stringNumber > 5 && str7.stringNumber > 5 && nA > 0 && nA < 25 ){
			memcpy( &glo_strPart_alm->str6, &str6, sizeof(string6_GLO_t));
			memcpy( &glo_strPart_alm->str7, &str7, sizeof(string7_GLO_t));
			flag[1] = nA;
	        return TRUE;
		}
	}
	
	//third the string 14
	if( subfrID == 14 ){

		uint8_t temp[10] = {0};

		memcpy( temp, msg, 10 );

		strNo = get_unsigned_8bits( temp, 5, 9 );
		if( strNo != 14 ){
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"stringNumber check failed,%s,%d\n",__FUNCTION__,__LINE__);
			return FALSE;
		}

		glo_timeRef->flagB = 1;
		glo_timeRef->B1 = get_signed_16bits( temp, 9, 20, 1 );
		glo_timeRef->B2 = get_signed_16bits( temp, 20, 30, 1 );
		glo_timeRef->KP = get_unsigned_8bits( temp, 30, 38 );
	}
	
	return TRUE;
}

uint8_t msg_decode_glo_N( 
	uint8_t *msg, 
	int32_t *header, 
	stringPart_Eph_N_t *glo_strPartEph_N, 
	stringPart_Alm_N_t *glo_strPartAlm_N, 
	int32_t *flag )
{
	//local paras
	uint8_t  strNo,subfrID,prn;
	GNSS_TIME*     pTime;
	
	//inputs check
	if( NULL == msg || NULL == header || NULL == glo_strPartEph_N || NULL == glo_strPartAlm_N || NULL == flag ){
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs err!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}
	//*********************************
	subfrID = (uint8_t)header[4];
	prn     = (uint8_t)header[1];

	//get the time
	pTime = gnss_tm_get_time();
	if(pTime->init == FALSE){
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"time init not!,%s,%d",__FUNCTION__,__LINE__);
	    return FALSE;
	}

	//check string ID
	strNo = get_unsigned_8bits( msg, 5, 9 );
	if( strNo != header[5]  ){
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"strID compare failed!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}

	//decode the dataBits according strNo
	if( strNo == 1 ){
		glo_strPartEph_N[prn - 1].str1.tor = pTime->rcvr_time[GLN_MODE];

		glo_strPartEph_N[prn - 1].str1.stringNumber = strNo;
		glo_strPartEph_N[prn - 1].str1.reserved_bits_one = get_unsigned_8bits( msg, 9, 11 );
		glo_strPartEph_N[prn - 1].str1.P1 = get_unsigned_8bits( msg, 11, 13 );
		glo_strPartEph_N[prn - 1].str1.tk = get_unsigned_16bits( msg, 13, 25 );
		glo_strPartEph_N[prn - 1].str1.x_dot = get_signed_32bits( msg, 25, 49, 1 );
		glo_strPartEph_N[prn - 1].str1.x_dot2 = get_signed_8bits( msg, 49, 54, 1 );
		glo_strPartEph_N[prn - 1].str1.x = get_signed_32bits( msg, 54, 81, 1 );
	}

	//��2
	if( strNo == 2 )
	{
		glo_strPartEph_N[prn - 1].str2.tor = pTime->rcvr_time[GLN_MODE];

		glo_strPartEph_N[prn - 1].str2.stringNumber = strNo;
		glo_strPartEph_N[prn - 1].str2.Bn = get_unsigned_8bits( msg, 9, 12 );
		glo_strPartEph_N[prn - 1].str2.P2 = get_unsigned_8bits( msg, 12, 13 );
		glo_strPartEph_N[prn - 1].str2.tb = get_unsigned_8bits( msg, 13, 20 );
		glo_strPartEph_N[prn - 1].str2.reserved_bits_one = get_unsigned_8bits( msg, 20, 25 );
		glo_strPartEph_N[prn - 1].str2.y_dot = get_signed_32bits( msg, 25, 49, 1 );
		glo_strPartEph_N[prn - 1].str2.y_dot2 = get_signed_8bits( msg, 49, 54, 1 );
		glo_strPartEph_N[prn - 1].str2.y = get_signed_32bits( msg, 54, 81, 1 );
	}

	//��3
	if( strNo == 3 )
	{
		glo_strPartEph_N[prn - 1].str3.tor = pTime->rcvr_time[GLN_MODE];

		glo_strPartEph_N[prn - 1].str3.stringNumber = strNo; 
		glo_strPartEph_N[prn - 1].str3.P3 = get_unsigned_8bits( msg, 9 , 10 );
		glo_strPartEph_N[prn - 1].str3.gamma = get_signed_16bits( msg, 10, 21, 1 );
		glo_strPartEph_N[prn - 1].str3.reserved_bits_one = get_unsigned_8bits( msg, 21, 22 );
		glo_strPartEph_N[prn - 1].str3.P = get_unsigned_8bits( msg, 22, 24 );
		glo_strPartEph_N[prn - 1].str3.ln = get_unsigned_8bits( msg, 24, 25 );
		glo_strPartEph_N[prn - 1].str3.z_dot = get_signed_32bits( msg, 25, 49, 1 );
		glo_strPartEph_N[prn - 1].str3.z_dot2 = get_signed_8bits( msg, 49, 54, 1 );
		glo_strPartEph_N[prn - 1].str3.z = get_signed_32bits( msg, 54, 81, 1 );
	}

	//��4
	if( strNo == 4 )
	{
		glo_strPartEph_N[prn - 1].str4.tor = pTime->rcvr_time[GLN_MODE];

		glo_strPartEph_N[prn - 1].str4.stringNumber = strNo; 
		glo_strPartEph_N[prn - 1].str4.tau = get_signed_32bits( msg, 9, 31, 1 );
		glo_strPartEph_N[prn - 1].str4.delt_tau = get_signed_8bits( msg, 31, 36, 1 );
		glo_strPartEph_N[prn - 1].str4.E = get_unsigned_8bits( msg, 36, 41 );
		glo_strPartEph_N[prn - 1].str4.reserved_bits_one = get_unsigned_16bits( msg, 41, 55 );
		glo_strPartEph_N[prn - 1].str4.P4 = get_unsigned_8bits( msg, 55, 56 );
		glo_strPartEph_N[prn - 1].str4.FT = get_unsigned_8bits( msg, 56, 60 );
		glo_strPartEph_N[prn - 1].str4.reserved_bits_two = get_unsigned_8bits( msg, 60, 63 );
		glo_strPartEph_N[prn - 1].str4.NT = get_unsigned_16bits( msg, 63, 74 );
		glo_strPartEph_N[prn - 1].str4.n = get_unsigned_8bits( msg, 74, 79 );
		glo_strPartEph_N[prn - 1].str4.M = get_unsigned_8bits( msg, 79, 81 );

		//prn���
		if( prn != glo_strPartEph_N[prn - 1].str4.n ){
			memset( &glo_strPartEph_N[prn - 1].str4, 0, sizeof(string4_GLO_t) );
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"glonass svID compare failed,%s,%d\n",__FUNCTION__,__LINE__);
			return FALSE;
		}
	}

	//��5
	if( strNo == 5 )
	{
		glo_strPartAlm_N[prn - 1].str5.tor = pTime->rcvr_time[GLN_MODE];

		glo_strPartAlm_N[prn - 1].str5.stringNumber = strNo; 
		glo_strPartAlm_N[prn - 1].str5.NA = get_unsigned_16bits( msg, 9, 20 );
		glo_strPartAlm_N[prn - 1].str5.tauC = get_signed_32bits( msg, 20, 52, 1 );
		glo_strPartAlm_N[prn - 1].str5.reserved_bits_one = get_unsigned_8bits( msg, 52, 53 );
		glo_strPartAlm_N[prn - 1].str5.N4 = get_unsigned_8bits( msg, 53, 58 );
		glo_strPartAlm_N[prn - 1].str5.tauGPS = get_signed_32bits( msg, 58, 80, 1 );
		glo_strPartAlm_N[prn - 1].str5.ln = get_unsigned_8bits( msg, 80, 81 );
	}

	if( strNo == 6 || strNo == 8 || strNo == 10 || strNo == 12 || strNo == 14 ){
		uint8_t  cnt,nA;
		uint8_t  subfr_id = 0;

		//*************
		//subframe 5 & string 15
		if(subfrID == 5 && strNo == 14  ){
			glo_strPartAlm_N[prn - 1].str14.tor = pTime->rcvr_time[GLN_MODE];

			glo_strPartAlm_N[prn - 1].str14.stringNumber = strNo; 
			glo_strPartAlm_N[prn - 1].str14.B1 = get_signed_16bits( msg, 9, 20, 1 );
			glo_strPartAlm_N[prn - 1].str14.B2 = get_signed_16bits( msg, 20, 30, 1 );
			glo_strPartAlm_N[prn - 1].str14.KP = get_unsigned_8bits( msg, 30, 32 );

			if( glo_strPartAlm_N[prn - 1].str5.stringNumber == 5 ){
			    double dt;

				dt = glo_strPartAlm_N[prn - 1].str14.tor - glo_strPartAlm_N[prn - 1].str5.tor;
				if( dt < -(double)(SECS_IN_DAY/2) ) dt = dt + SECS_IN_DAY;
				if( dt > 17 || dt < 19 )
					flag[2] = 1;
			}
		}
		else{
			cnt = (uint8_t)(strNo/2) - 3;
			//find prn
			glo_strPartAlm_N[prn - 1].str6[cnt].tor = pTime->rcvr_time[GLN_MODE];

			glo_strPartAlm_N[prn - 1].str6[cnt].stringNumber = strNo; 
			glo_strPartAlm_N[prn - 1].str6[cnt].CA =get_unsigned_8bits( msg, 9 ,10 );
			glo_strPartAlm_N[prn - 1].str6[cnt].MA = get_unsigned_8bits( msg, 10, 12 );
			glo_strPartAlm_N[prn - 1].str6[cnt].nA = get_unsigned_8bits( msg, 12, 17 );
			glo_strPartAlm_N[prn - 1].str6[cnt].tauA = get_signed_16bits( msg, 17, 27, 1 );
			glo_strPartAlm_N[prn - 1].str6[cnt].lambdaA = get_signed_32bits( msg, 27, 48, 1 );
			glo_strPartAlm_N[prn - 1].str6[cnt].delt_iA = get_signed_32bits( msg, 48, 66, 1 );
			glo_strPartAlm_N[prn - 1].str6[cnt].epsilonA = get_signed_16bits( msg, 66, 81, 1 );

			//check nA
			nA = glo_strPartAlm_N[prn - 1].str6[cnt].nA;
			if( nA < 1 || nA > 24){
				memset( &glo_strPartAlm_N[prn - 1].str6[cnt], 0, sizeof(string6_GLO_t));
				SYS_LOGGING(OBJ_SD,LOG_WARNING,"glonass svID(nA) in string 6 check error,%s,%d\n",__FUNCTION__,__LINE__);
				return FALSE;
			}

			//check nA to compare the subframeID;
			if( nA >= 1 && nA <= 5 )          subfr_id = 1;
			else if( nA >= 6  && nA <= 10 )    subfr_id = 2;
			else if( nA >= 11 && nA <= 15 )    subfr_id = 3;
			else if( nA >= 16 && nA <= 20 )    subfr_id = 4;
			else if( nA >= 21 && nA <= 24 )    subfr_id = 5;

			if( subfr_id != subfrID ){
				memset( &glo_strPartAlm_N[prn - 1].str6[cnt], 0, sizeof(string6_GLO_t));
				SYS_LOGGING(OBJ_SD,LOG_WARNING,"glonass subfrID in string 6 check error,%s,%d\n",__FUNCTION__,__LINE__);
				return FALSE;
			}

			glo_strPartAlm_N[prn - 1].subfr_id = subfr_id;
		}


	}

	if( strNo == 7 || strNo == 9 || strNo == 11 || strNo == 13 || strNo == 15 ){
		
		if(subfrID == 5 && strNo == 15  ){
			return TRUE;
		}
		else{
			uint8_t  cnt;

			cnt = (uint8_t)((strNo + 1)/2) - 4;

			glo_strPartAlm_N[prn - 1].str7[cnt].tor = pTime->rcvr_time[GLN_MODE];

			glo_strPartAlm_N[prn - 1].str7[cnt].stringNumber = strNo; 
			glo_strPartAlm_N[prn - 1].str7[cnt].omegaA = get_signed_16bits( msg, 9, 25, 1 );
			glo_strPartAlm_N[prn - 1].str7[cnt].t_lambdaA = get_unsigned_32bits( msg, 25, 46 );
			glo_strPartAlm_N[prn - 1].str7[cnt].deltTA = get_signed_32bits( msg, 46, 68, 1 );
			glo_strPartAlm_N[prn - 1].str7[cnt].deltTA_dot = get_signed_8bits( msg, 68, 75, 1 );
			glo_strPartAlm_N[prn - 1].str7[cnt].HA = get_unsigned_8bits( msg, 75, 80 );
			glo_strPartAlm_N[prn - 1].str7[cnt].ln = get_unsigned_8bits( msg, 80, 81 );

			//�ж��������
			if( glo_strPartAlm_N[prn - 1].str5.stringNumber == 5 && 
				glo_strPartAlm_N[prn - 1].str6[cnt].stringNumber == strNo - 1)
			{
				double dt;
				uint8_t  mark = 0;

				dt = glo_strPartAlm_N[prn - 1].str6[cnt].tor - glo_strPartAlm_N[prn - 1].str5.tor;
				if( dt < -(double)(SECS_IN_DAY/2) ) dt = dt + SECS_IN_DAY;
				if( dt < ((cnt+1.0)*4.0 - 3.0) || dt > ((cnt+1.0)*4.0 - 1.0) ) mark = 1;

				dt = glo_strPartAlm_N[prn - 1].str7[cnt].tor - glo_strPartAlm_N[prn - 1].str5.tor;
				if( dt < -(double)(SECS_IN_DAY/2) ) dt = dt + SECS_IN_DAY;
				if( dt < ((cnt+1.0)*4.0 - 1.0) || dt > ((cnt+1.0)*4.0 + 1.0) ) mark = 1;

				if( mark == 0 )
				    flag[1] = cnt+1;
			}
		}
	}

	//�ж���������
	if( glo_strPartEph_N[prn - 1].str1.stringNumber ==  1 &&
		glo_strPartEph_N[prn - 1].str2.stringNumber ==  2 &&
		glo_strPartEph_N[prn - 1].str3.stringNumber ==  3 &&
		glo_strPartEph_N[prn - 1].str4.stringNumber ==  4 )
	{
		double t1,t2,t3,t4;

		t1 = floor(glo_strPartEph_N[prn - 1].str1.tor / 1800.0);
		t2 = floor(glo_strPartEph_N[prn - 1].str2.tor / 1800.0);
		t3 = floor(glo_strPartEph_N[prn - 1].str3.tor / 1800.0);
		t4 = floor(glo_strPartEph_N[prn - 1].str4.tor / 1800.0);

		if( (t1 == t2)&&(t1 == t3)&&(t1 == t4) )
			flag[0] = prn;
	}
	return TRUE;
}

//*****************BDS_PART****************************
uint8_t exchange_bds_eph_N( 
	uint8_t prn, 
	subfr123_D1_BDS_t *bds_subfrPartEph_D1_N, 
	subfr1_D2_BDS_t *bds_subfrPartEph_D2_N, 
	BDS_EPH_INFO *bds_eph )
{
	//local paras 
	uint32_t u32a,u32b,u32c;
	int32_t s32a;

	memset( bds_eph, 0, sizeof( BDS_EPH_INFO ) );
	//***********
	if( prn < 6 ){
		//inputs check
		if( NULL == bds_eph || bds_subfrPartEph_D2_N == NULL  ){
			SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs err!,%s,%d",__FUNCTION__,__LINE__);
			return FALSE;
		}

		//data valid check
		if( bds_subfrPartEph_D2_N->page6.sqrtA == 0 ){
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"empty rawbits for bds_eph decoded!,%s,%d",__FUNCTION__,__LINE__);
			return FALSE;
		}

		bds_eph->prn = prn;
		bds_eph->eph_source = EPH_SOURCE_BRDC;
		bds_eph->eph_status = EPH_STATUS_VALID;
		bds_eph->af0 = bds_subfrPartEph_D2_N->page3.a0*P2_33;

		u32a = (uint32_t)( bds_subfrPartEph_D2_N->page3.a1_U4 << 28 );
		u32b = (uint32_t)( bds_subfrPartEph_D2_N->page4.a1_L18 << 10 );
		s32a = (int32_t)( u32a | u32b );
		s32a >>= 10;
		bds_eph->af1 = (float)(s32a*P2_50);

		bds_eph->af2 = (float)(bds_subfrPartEph_D2_N->page4.a2*P2_55*P2_11);

		u32a = (uint32_t)( bds_subfrPartEph_D2_N->page6.cic_U10 << 22 );
		u32b = (uint32_t)( bds_subfrPartEph_D2_N->page7.cic_L8 << 14 );
		s32a = (int32_t)( u32a | u32b );
		s32a >>= 14;
		bds_eph->cic = (float)(s32a*P2_31);

		bds_eph->cis = (float)(bds_subfrPartEph_D2_N->page7.cis*P2_31);
		bds_eph->crc = (float)(bds_subfrPartEph_D2_N->page8.crc*P2_06);
		bds_eph->crs = (float)(bds_subfrPartEph_D2_N->page8.crs*P2_06);

		u32a = (uint32_t)( bds_subfrPartEph_D2_N->page4.cuc_U14 << 18 );
		u32b = (uint32_t)( bds_subfrPartEph_D2_N->page5.cuc_L4 << 14 );
		s32a = (int32_t)( u32a | u32b );
		s32a >>= 14;
		bds_eph->cuc = (float)(s32a*P2_31);

		bds_eph->cus = (float)(bds_subfrPartEph_D2_N->page5.cus*P2_31);
		bds_eph->delta_n = bds_subfrPartEph_D2_N->page4.deltN*P2_43*PI;

		u32a = (uint32_t)( bds_subfrPartEph_D2_N->page5.e_U10 << 22 );
		u32b = (uint32_t)( bds_subfrPartEph_D2_N->page6.e_L22 );
		u32c = (uint32_t)( u32a | u32b );
		bds_eph->ecc = u32c*P2_33;

		u32a = (uint32_t)( bds_subfrPartEph_D2_N->page7.i0_U21 << 11 );
		u32b = (uint32_t)( bds_subfrPartEph_D2_N->page8.i0_L11 );
		s32a = (int32_t)( u32a | u32b );
		bds_eph->i0 = s32a*P2_31*PI;

		bds_eph->idot = bds_subfrPartEph_D2_N->page10.idot*P2_43*PI;
		bds_eph->M0 = bds_subfrPartEph_D2_N->page5.m0*P2_31*PI;
		bds_eph->OMEGA_0 = bds_subfrPartEph_D2_N->page9.omega0*P2_31*PI;

		u32a = (uint32_t)( bds_subfrPartEph_D2_N->page8.omega_dot_U19 << 13 );
		u32b = (uint32_t)( bds_subfrPartEph_D2_N->page9.omega_dot_L5 << 8 );
		s32a = (int32_t)( u32a | u32b );
		s32a >>= 8;
		bds_eph->OMEGA_Dot = s32a*P2_43*PI;
		bds_eph->SatH1 = bds_subfrPartEph_D2_N->page1.satH1;
		bds_eph->sqrta = bds_subfrPartEph_D2_N->page6.sqrtA*P2_19;
		bds_eph->TGD1 = (float)(bds_subfrPartEph_D2_N->page1.tgd1*0.1);
		bds_eph->TGD2 = (float)(bds_subfrPartEph_D2_N->page1.tgd2*0.1);
		bds_eph->toc = bds_subfrPartEph_D2_N->page1.toc*8;
		bds_eph->toe = bds_subfrPartEph_D2_N->page7.toe*8;

		u32a = (uint32_t)( bds_subfrPartEph_D2_N->page9.w_U27 << 5 );
		u32b = (uint32_t)( bds_subfrPartEph_D2_N->page10.w_L5 );
		s32a = (int32_t)( u32a | u32b );
		bds_eph->w = s32a*P2_31*PI;

		bds_eph->WN = bds_subfrPartEph_D2_N->page1.wn;
		bds_eph->URAI = bds_subfrPartEph_D2_N->page1.urai;
		bds_eph->IODC = bds_subfrPartEph_D2_N->page1.AODC;
		bds_eph->IODE = bds_subfrPartEph_D2_N->page4.aode;

		//δ��ֵ
	    bds_eph->EphDecFlag = 0;
		bds_eph->sinEk = 0;

		//clear cache
		memset( bds_subfrPartEph_D2_N, 0, sizeof(subfr1_D2_BDS_t) );
	}
	else{
		//inputs check
		if( NULL == bds_eph || bds_subfrPartEph_D1_N == NULL ){
			SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs err!,%s,%d",__FUNCTION__,__LINE__);
			return FALSE;
		}

		//data valid check
		if( bds_subfrPartEph_D1_N->subfr2.sqrtA == 0 ){
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"empty rawbits for bds_eph decoded!,%s,%d",__FUNCTION__,__LINE__);
			return FALSE;
		}

		bds_eph->prn = prn;
		bds_eph->eph_source = EPH_SOURCE_BRDC;
		bds_eph->eph_status = EPH_STATUS_VALID;
		bds_eph->af0 = bds_subfrPartEph_D1_N->subfr1.a0*P2_33;
		bds_eph->af1 = (float)(bds_subfrPartEph_D1_N->subfr1.a1*P2_50);
		bds_eph->af2 = (float)(bds_subfrPartEph_D1_N->subfr1.a2*P2_55*P2_11);
		bds_eph->cic = (float)(bds_subfrPartEph_D1_N->subfr3.cic*P2_31);
		bds_eph->cis = (float)(bds_subfrPartEph_D1_N->subfr3.cis*P2_31);
		bds_eph->crc = (float)(bds_subfrPartEph_D1_N->subfr2.crc*P2_06);
		bds_eph->crs = (float)(bds_subfrPartEph_D1_N->subfr2.crs*P2_06);
		bds_eph->cuc = (float)(bds_subfrPartEph_D1_N->subfr2.cuc*P2_31);
		bds_eph->cus = (float)(bds_subfrPartEph_D1_N->subfr2.cus*P2_31);
		bds_eph->delta_n = bds_subfrPartEph_D1_N->subfr2.deltN*P2_43*PI;
		bds_eph->ecc = bds_subfrPartEph_D1_N->subfr2.e*P2_33;
		bds_eph->i0 = bds_subfrPartEph_D1_N->subfr3.i0*P2_31*PI;
		bds_eph->idot = bds_subfrPartEph_D1_N->subfr3.idot*P2_43*PI;
		bds_eph->IODC = bds_subfrPartEph_D1_N->subfr1.aodc;
		bds_eph->IODE = bds_subfrPartEph_D1_N->subfr1.aode;
		bds_eph->M0 = bds_subfrPartEph_D1_N->subfr2.m0*P2_31*PI;
		bds_eph->OMEGA_0 = bds_subfrPartEph_D1_N->subfr3.omega0*P2_31*PI;
		bds_eph->OMEGA_Dot = bds_subfrPartEph_D1_N->subfr3.omega_dot*P2_43*PI;
		bds_eph->SatH1 = bds_subfrPartEph_D1_N->subfr1.satH1;
		bds_eph->sqrta = bds_subfrPartEph_D1_N->subfr2.sqrtA*P2_19;
		bds_eph->TGD1 = (float)(bds_subfrPartEph_D1_N->subfr1.tgd1*0.1);
		bds_eph->TGD2 = (float)(bds_subfrPartEph_D1_N->subfr1.tgd2*0.1);
		bds_eph->toc = bds_subfrPartEph_D1_N->subfr1.toc*8;

		u32a = (uint32_t)( bds_subfrPartEph_D1_N->subfr2.toe_U2 << 15 );
		u32b = (uint32_t)( bds_subfrPartEph_D1_N->subfr3.toe_L15 );
		u32c = (uint32_t)( u32a | u32b );
		bds_eph->toe = u32c*8;

		bds_eph->URAI = bds_subfrPartEph_D1_N->subfr1.urai;
		bds_eph->w = bds_subfrPartEph_D1_N->subfr3.w*P2_31*PI;
		bds_eph->WN = bds_subfrPartEph_D1_N->subfr1.wn;

		//***
		bds_eph->EphDecFlag = 0;
		bds_eph->sinEk = 0;

		//clear cache
		memset( bds_subfrPartEph_D1_N, 0, sizeof(subfr123_D1_BDS_t) );
	}
	
	//toc/toe���
	if( bds_eph->toe != bds_eph->toc ){
		SYS_LOGGING(OBJ_SD,LOG_WARNING,"BDS toc/toe dismatch,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}
	
	return TRUE;
}

uint8_t exchange_bds_alm_N( 
	uint16_t sys_id,
	uint8_t  alm_pos,
	subfr45_D1_BDS_t *bds_subfrPartAlm_D1_N,
	subfr5_D2_BDS_t  *bds_subfrPartAlm_D2_N, 
	BDS_ALM_INFO  *bds_alm )
{
	memset( bds_alm, 0, sizeof(BDS_ALM_INFO) );
	//*******************

	if( sys_id == AGGNSS_NAVIGATION_MESSAGE_TYPE_BDS_D1 ){
		//inputs check
		if( NULL == bds_alm || bds_subfrPartAlm_D1_N == NULL ){
			SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs err!,%s,%d",__FUNCTION__,__LINE__);
			return FALSE;
		}

		if( alm_pos < 24 ){

			//data valid check
			if( bds_subfrPartAlm_D1_N->subfr4_PageT24[alm_pos].e  == 0 ||
				bds_subfrPartAlm_D1_N->subfr4_PageT24[alm_pos].sqrtA  == 0 )
			{
				SYS_LOGGING(OBJ_SD,LOG_WARNING,"empty rawbits for BDS_ALM decoded!,%s,%d",__FUNCTION__,__LINE__);
				return FALSE;
			}

			bds_alm->prn        = alm_pos + 1;
			bds_alm->alm_source = 0;
			bds_alm->alm_status = TRUE;
			bds_alm->AlmFlag    = 0;

			bds_alm->a0         = (float)(bds_subfrPartAlm_D1_N->subfr4_PageT24[alm_pos].a0*P2_20);
			bds_alm->a1         = (float)(bds_subfrPartAlm_D1_N->subfr4_PageT24[alm_pos].a1*P2_38);

			bds_alm->deltai     = bds_subfrPartAlm_D1_N->subfr4_PageT24[alm_pos].deltaI*P2_19*PI;
			bds_alm->ecc        = (float)(bds_subfrPartAlm_D1_N->subfr4_PageT24[alm_pos].e*P2_21);
			bds_alm->M0         = bds_subfrPartAlm_D1_N->subfr4_PageT24[alm_pos].m0*P2_23*PI;
			bds_alm->omega0     = bds_subfrPartAlm_D1_N->subfr4_PageT24[alm_pos].omega0*P2_23*PI;
			bds_alm->omegaDot   = bds_subfrPartAlm_D1_N->subfr4_PageT24[alm_pos].omega_dot*P2_38*PI;
			bds_alm->sqrta      = bds_subfrPartAlm_D1_N->subfr4_PageT24[alm_pos].sqrtA*P2_11;
			bds_alm->toa        = bds_subfrPartAlm_D1_N->subfr4_PageT24[alm_pos].toa*4096;
			bds_alm->toa1       = bds_subfrPartAlm_D1_N->subfr5_page8.toa*4096;
			bds_alm->w          = bds_subfrPartAlm_D1_N->subfr4_PageT24[alm_pos].w*P2_23*PI;
			bds_alm->WNa        = bds_subfrPartAlm_D1_N->subfr5_page8.wna;

			//clear cache
			memset( &bds_subfrPartAlm_D1_N->subfr4_PageT24[alm_pos], 0, sizeof( subfr45_page1_D1_BDS_t) );
		}
		else{

			//data valid check
			if( bds_subfrPartAlm_D1_N->subfr5_PageT6[alm_pos - 24].e  == 0 ||
				bds_subfrPartAlm_D1_N->subfr5_PageT6[alm_pos - 24].sqrtA  == 0 )
			{
				SYS_LOGGING(OBJ_SD,LOG_WARNING,"empty rawbits for BDS_ALM decoded!,%s,%d",__FUNCTION__,__LINE__);
				return FALSE;
			}

			bds_alm->prn        = alm_pos + 1;
			bds_alm->alm_source = EPH_SOURCE_BRDC;
			bds_alm->alm_status = TRUE;
			bds_alm->AlmFlag    = 0;

			bds_alm->a0         = (float)(bds_subfrPartAlm_D1_N->subfr5_PageT6[alm_pos - 24].a0*P2_20);
			bds_alm->a1         = (float)(bds_subfrPartAlm_D1_N->subfr5_PageT6[alm_pos - 24].a1*P2_38);

			bds_alm->deltai     = bds_subfrPartAlm_D1_N->subfr5_PageT6[alm_pos - 24].deltaI*P2_19*PI;
			bds_alm->ecc        = (float)(bds_subfrPartAlm_D1_N->subfr5_PageT6[alm_pos - 24].e*P2_21);
			bds_alm->M0         = bds_subfrPartAlm_D1_N->subfr5_PageT6[alm_pos - 24].m0*P2_23*PI;
			bds_alm->omega0     = bds_subfrPartAlm_D1_N->subfr5_PageT6[alm_pos - 24].omega0*P2_23*PI;
			bds_alm->omegaDot   = bds_subfrPartAlm_D1_N->subfr5_PageT6[alm_pos - 24].omega_dot*P2_38*PI;
			bds_alm->sqrta      = bds_subfrPartAlm_D1_N->subfr5_PageT6[alm_pos - 24].sqrtA*P2_11;
			bds_alm->toa        = bds_subfrPartAlm_D1_N->subfr5_PageT6[alm_pos - 24].toa*4096;
			bds_alm->toa1       = bds_subfrPartAlm_D1_N->subfr5_page8.toa*4096;
			bds_alm->w          = bds_subfrPartAlm_D1_N->subfr5_PageT6[alm_pos - 24].w*P2_23*PI;
			bds_alm->WNa        = bds_subfrPartAlm_D1_N->subfr5_page8.wna;

			//clear cache
			memset( &bds_subfrPartAlm_D1_N->subfr5_PageT6[alm_pos - 24], 0, sizeof( subfr45_page1_D1_BDS_t) );
		}
	}
	else if( sys_id == AGGNSS_NAVIGATION_MESSAGE_TYPE_BDS_D2 ){
		//inputs check
		if( NULL == bds_alm || bds_subfrPartAlm_D2_N == NULL ){
			SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs err!,%s,%d",__FUNCTION__,__LINE__);
			return FALSE;
		}

		if( bds_subfrPartAlm_D2_N->pageT30[alm_pos].e  == 0 ||
			bds_subfrPartAlm_D2_N->pageT30[alm_pos].sqrtA  == 0 )
		{
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"empty rawbits for BDS_ALM decoded!,%s,%d",__FUNCTION__,__LINE__);
			return FALSE;
		}

		bds_alm->prn            = alm_pos + 1;
		bds_alm->alm_source     = EPH_SOURCE_BRDC;
		bds_alm->alm_status     = TRUE;
		bds_alm->AlmFlag        = 0;

		bds_alm->a0             = (float)(bds_subfrPartAlm_D2_N->pageT30[alm_pos].a0*P2_20);
		bds_alm->a1             = (float)(bds_subfrPartAlm_D2_N->pageT30[alm_pos].a1*P2_38);

		bds_alm->deltai         = bds_subfrPartAlm_D2_N->pageT30[alm_pos].deltaI*P2_19*PI;
		bds_alm->ecc            = (float)(bds_subfrPartAlm_D2_N->pageT30[alm_pos].e*P2_21);
		bds_alm->M0             = bds_subfrPartAlm_D2_N->pageT30[alm_pos].m0*P2_23*PI;
		bds_alm->omega0         = bds_subfrPartAlm_D2_N->pageT30[alm_pos].omega0*P2_23*PI;
		bds_alm->omegaDot       = bds_subfrPartAlm_D2_N->pageT30[alm_pos].omega_dot*P2_38*PI;
		bds_alm->sqrta          = bds_subfrPartAlm_D2_N->pageT30[alm_pos].sqrtA*P2_11;
		bds_alm->toa            = bds_subfrPartAlm_D2_N->pageT30[alm_pos].toa*4096;
		bds_alm->toa1           = bds_subfrPartAlm_D2_N->page36.toa*4096;
		bds_alm->w              = bds_subfrPartAlm_D2_N->pageT30[alm_pos].w*P2_23*PI;
		bds_alm->WNa            = bds_subfrPartAlm_D2_N->page36.wna;

		//clear cache
		memset( &bds_subfrPartAlm_D2_N->pageT30[alm_pos], 0, sizeof(subfr5_page3760_D2_BDS_t) );
	}
	else{
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"unsupported msg type in BDS!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}

	return TRUE;
}

uint8_t exchange_bds_iono_N( 
	uint16_t sys_id, 
	subfr123_D1_BDS_t  *bds_subfrPartEph_D1_N, 
	subfr1_D2_BDS_t *bds_subfrPartEph_D2_N, 
	ION_INFO *bds_iono )
{
	memset( bds_iono, 0, sizeof(ION_INFO) );
	//****************
	if(sys_id == AGGNSS_NAVIGATION_MESSAGE_TYPE_BDS_D1){
		//inputs check
		if( NULL == bds_iono || bds_subfrPartEph_D1_N == NULL ){
			SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs err!,%s,%d",__FUNCTION__,__LINE__);
			return FALSE;
		}

		bds_iono->alpha_0  = (float)(bds_subfrPartEph_D1_N->subfr1.alfa0*P2_30);
		bds_iono->alpha_1  = (float)(bds_subfrPartEph_D1_N->subfr1.alfa1*P2_27);
		bds_iono->alpha_2  = (float)(bds_subfrPartEph_D1_N->subfr1.alfa2*P2_24);
		bds_iono->alpha_3  = (float)(bds_subfrPartEph_D1_N->subfr1.alfa3*P2_24);
		bds_iono->beta_0   = (float)(bds_subfrPartEph_D1_N->subfr1.beta0*2048.0);
		bds_iono->beta_1   = (float)(bds_subfrPartEph_D1_N->subfr1.beta1*16384.0);
		bds_iono->beta_2   = (float)(bds_subfrPartEph_D1_N->subfr1.beta2*65536.0);
		bds_iono->beta_3   = (float)(bds_subfrPartEph_D1_N->subfr1.beta3*65536.0);
		bds_iono->have_ion = 1;

	}
	else if( sys_id == AGGNSS_NAVIGATION_MESSAGE_TYPE_BDS_D2 ){
		//inputs check
		if( NULL == bds_iono || bds_subfrPartEph_D2_N == NULL ){
			SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs err!,%s,%d",__FUNCTION__,__LINE__);
			return FALSE;
		}

		bds_iono->alpha_0  = (float)(bds_subfrPartEph_D2_N->page2.alfa0*P2_30);
	    bds_iono->alpha_1  = (float)(bds_subfrPartEph_D2_N->page2.alfa1*P2_27);
		bds_iono->alpha_2  = (float)(bds_subfrPartEph_D2_N->page2.alfa2*P2_24);
		bds_iono->alpha_3  = (float)(bds_subfrPartEph_D2_N->page2.alfa3*P2_24);
		bds_iono->beta_0   = (float)(bds_subfrPartEph_D2_N->page2.beta0*2048.0);
		bds_iono->beta_1   = (float)(bds_subfrPartEph_D2_N->page2.beta1*16384.0);
		bds_iono->beta_2   = (float)(bds_subfrPartEph_D2_N->page2.beta2*65536.0);
		bds_iono->beta_3   = (float)(bds_subfrPartEph_D2_N->page2.beta3*65536.0);
		bds_iono->have_ion = 1;
	}
	else{
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"unsupported msg type in BDS!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}

	return TRUE;
}

uint8_t exchange_bds_utc_N( 
	uint16_t sys_id, 
	subfr45_D1_BDS_t *bds_subfrPartAlm_D1_N, 
	subfr5_D2_BDS_t *bds_subfrPartAlm_D2_N, 
	bdsUtcModel_t *bds_utc )
{
	memset( bds_utc, 0, sizeof(bdsUtcModel_t) );
	//********************
	if( sys_id == AGGNSS_NAVIGATION_MESSAGE_TYPE_BDS_D1 ){
		//inputs check
		if( NULL == bds_utc || bds_subfrPartAlm_D1_N == NULL ){
			SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs err!,%s,%d",__FUNCTION__,__LINE__);
			return FALSE;
		}

		bds_utc->utcA0        = (double)(bds_subfrPartAlm_D1_N->subfr5_page10.a0_UTC*P2_30);
		bds_utc->utcA1        = (double)(bds_subfrPartAlm_D1_N->subfr5_page10.a1_UTC*P2_50);
		bds_utc->utcDeltaTls  = bds_subfrPartAlm_D1_N->subfr5_page10.deltT_LS;
		bds_utc->utcDeltaTlsf = bds_subfrPartAlm_D1_N->subfr5_page10.deltT_LSF;
		bds_utc->utcDN        = bds_subfrPartAlm_D1_N->subfr5_page10.dn;
		bds_utc->utcWNlsf     = bds_subfrPartAlm_D1_N->subfr5_page10.wn_LSF;

		//clear cache
		memset( &bds_subfrPartAlm_D1_N->subfr5_page10, 0, sizeof(subfr5_page10_D1_BDS_t) );
	}
	else if( sys_id == AGGNSS_NAVIGATION_MESSAGE_TYPE_BDS_D2 ){
		//inputs check
		if( NULL == bds_utc || bds_subfrPartAlm_D2_N == NULL ){
			SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs err!,%s,%d",__FUNCTION__,__LINE__);
			return FALSE;
		}

		bds_utc->utcA0        = (double)(bds_subfrPartAlm_D2_N->page102.a0_UTC*P2_30);
		bds_utc->utcA1        = (double)(bds_subfrPartAlm_D2_N->page102.a1_UTC*P2_50);
		bds_utc->utcDeltaTls  = bds_subfrPartAlm_D2_N->page102.deltT_LS;
		bds_utc->utcDeltaTlsf = bds_subfrPartAlm_D2_N->page102.deltT_LSF;
		bds_utc->utcDN        = bds_subfrPartAlm_D2_N->page102.dn;
		bds_utc->utcWNlsf     = bds_subfrPartAlm_D2_N->page102.wn_LSF;

		//clear cache
		memset( &bds_subfrPartAlm_D2_N->page102, 0, sizeof(subfr5_page102_D2_BDS_t) );

	}
	else{
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"unsupported msg type in BDS!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}

	return TRUE;
}

uint8_t msg_decode_bds_N( 
	uint8_t *msg, 
	int32_t *header, 
	subfr123_D1_BDS_t *bds_subfrPartEph_D1_N, 
	subfr45_D1_BDS_t *bds_subfrPartAlm_D1_N, 
	subfr1_D2_BDS_t *bds_subfrPartEph_D2_N, 
	subfr5_D2_BDS_t *bds_subfrPartAlm_D2_N, 
	int32_t *flag )
{
	//local paras
	uint8_t  subfrID,prn,fraID,pnum;
	uint16_t pre;
	uint32_t sow;

	//inputs check
	if( NULL == msg || NULL == header || NULL == bds_subfrPartEph_D1_N || NULL == bds_subfrPartAlm_D1_N ||
		NULL == bds_subfrPartEph_D2_N || NULL == bds_subfrPartAlm_D2_N || NULL == flag ){
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs err!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}
	//****************
	subfrID = (uint8_t)header[5];
	prn     = (uint8_t)header[1];

	
	pre = get_unsigned_16bits( msg, 1, 12 );  	//pre
	fraID = get_unsigned_8bits( msg, 16, 19 );  //fraID�ͼ��
	sow = get_unsigned_32bits( msg, 19, 39 );   //sow

	//pre check
	if( pre != 1810 ){
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"pre check err!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}

	//subframe check
	if( fraID != subfrID ){
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"subframe compare failed!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}
	
	if( prn >= 1 && prn <= 5 ){
		if( fraID == 1 ){
			pnum = get_unsigned_8bits( msg, 39, 43);

			//if( pnum != header[4] ){
			//	SYS_LOGGING(OBJ_SD,LOG_ERROR,"pnum compare failed!,%s,%d",__FUNCTION__,__LINE__);
			//	return FALSE;
			//}

			//page 1
			if( 1 == pnum ){
				bds_subfrPartEph_D2_N[prn - 1].page1.fraID = fraID;
				bds_subfrPartEph_D2_N[prn - 1].page1.sow = sow;
				bds_subfrPartEph_D2_N[prn - 1].page1.pnum1 = pnum;

				bds_subfrPartEph_D2_N[prn - 1].page1.satH1 = get_unsigned_8bits( msg, 43, 44 ); 
				bds_subfrPartEph_D2_N[prn - 1].page1.AODC  = get_unsigned_8bits( msg, 44, 49 );
				bds_subfrPartEph_D2_N[prn - 1].page1.urai  = get_unsigned_8bits( msg, 49, 53 );
				bds_subfrPartEph_D2_N[prn - 1].page1.wn    = get_unsigned_16bits( msg, 53, 66 );
				bds_subfrPartEph_D2_N[prn - 1].page1.toc   = get_unsigned_32bits( msg, 66, 83 );
				bds_subfrPartEph_D2_N[prn - 1].page1.tgd1  = get_signed_16bits( msg, 83, 93 , 0 );
				bds_subfrPartEph_D2_N[prn - 1].page1.tgd2  = get_signed_16bits( msg, 93, 103 , 0 );
			}

			//page 2
			if( 2 == pnum ){
				bds_subfrPartEph_D2_N[prn - 1].page2.fraID = fraID;
				bds_subfrPartEph_D2_N[prn - 1].page2.sow = sow;
				bds_subfrPartEph_D2_N[prn - 1].page2.pnum1 = pnum;

				bds_subfrPartEph_D2_N[prn - 1].page2.alfa0 = get_signed_8bits( msg, 43, 51 ,0 );
				bds_subfrPartEph_D2_N[prn - 1].page2.alfa1 = get_signed_8bits( msg, 51, 59 ,0 );
				bds_subfrPartEph_D2_N[prn - 1].page2.alfa2 = get_signed_8bits( msg, 59, 67 ,0 );
				bds_subfrPartEph_D2_N[prn - 1].page2.alfa3 = get_signed_8bits( msg, 67, 75 ,0 );
				bds_subfrPartEph_D2_N[prn - 1].page2.beta0 = get_signed_8bits( msg, 75, 83 ,0 );
				bds_subfrPartEph_D2_N[prn - 1].page2.beta1 = get_signed_8bits( msg, 83, 91 ,0 );
				bds_subfrPartEph_D2_N[prn - 1].page2.beta2 = get_signed_8bits( msg, 91, 99 ,0 );
				bds_subfrPartEph_D2_N[prn - 1].page2.beta3 = get_signed_8bits( msg, 99, 107 ,0 );

				//IONO UPDATE
				flag[2] = 1;
			}

			//page 3
			if( 3 == pnum ){
				bds_subfrPartEph_D2_N[prn - 1].page3.fraID = fraID;
				bds_subfrPartEph_D2_N[prn - 1].page3.sow = sow;
				bds_subfrPartEph_D2_N[prn - 1].page3.pnum1 = pnum;

				bds_subfrPartEph_D2_N[prn - 1].page3.a0   = get_signed_32bits( msg, 81, 105, 0 );
				bds_subfrPartEph_D2_N[prn - 1].page3.a1_U4 = get_unsigned_8bits( msg, 105, 109 );
			}

			//page 4
			if( 4 == pnum ){
				bds_subfrPartEph_D2_N[prn - 1].page4.fraID = fraID;
				bds_subfrPartEph_D2_N[prn - 1].page4.sow = sow;
				bds_subfrPartEph_D2_N[prn - 1].page4.pnum1 = pnum;

				bds_subfrPartEph_D2_N[prn - 1].page4.a1_L18 = get_unsigned_32bits( msg, 43, 61 );
				bds_subfrPartEph_D2_N[prn - 1].page4.a2 = get_signed_16bits( msg, 61, 72 ,0 );
				bds_subfrPartEph_D2_N[prn - 1].page4.aode = get_unsigned_8bits( msg, 72, 77 );
				bds_subfrPartEph_D2_N[prn - 1].page4.deltN = get_signed_16bits( msg, 77, 93 ,0 );
				bds_subfrPartEph_D2_N[prn - 1].page4.cuc_U14 = get_unsigned_16bits( msg, 93, 107 );
			}

			//page 5
			if( 5 == pnum ){
				bds_subfrPartEph_D2_N[prn - 1].page5.fraID = fraID;
				bds_subfrPartEph_D2_N[prn - 1].page5.sow = sow;
				bds_subfrPartEph_D2_N[prn - 1].page5.pnum1 = pnum;

				bds_subfrPartEph_D2_N[prn - 1].page5.cuc_L4 = get_unsigned_8bits( msg, 43, 47 );
				bds_subfrPartEph_D2_N[prn - 1].page5.m0 = get_signed_32bits( msg, 47, 79, 0 );
				bds_subfrPartEph_D2_N[prn - 1].page5.cus = get_signed_32bits( msg, 79, 97, 0 );
				bds_subfrPartEph_D2_N[prn - 1].page5.e_U10 = get_unsigned_16bits( msg, 97, 107 );
			}

			//page 6
			if( 6 == pnum ){
				bds_subfrPartEph_D2_N[prn - 1].page6.fraID = fraID;
				bds_subfrPartEph_D2_N[prn - 1].page6.sow = sow;
				bds_subfrPartEph_D2_N[prn - 1].page6.pnum1 = pnum;

				bds_subfrPartEph_D2_N[prn - 1].page6.e_L22 = get_unsigned_32bits( msg, 43, 65 );
				bds_subfrPartEph_D2_N[prn - 1].page6.sqrtA = get_unsigned_32bits( msg, 65, 97 );
				bds_subfrPartEph_D2_N[prn - 1].page6.cic_U10 = get_unsigned_16bits( msg, 97, 107 );
			}

			//page 7
			if( 7 == pnum ){
				bds_subfrPartEph_D2_N[prn - 1].page7.fraID = fraID;
				bds_subfrPartEph_D2_N[prn - 1].page7.sow = sow;
				bds_subfrPartEph_D2_N[prn - 1].page7.pnum1 = pnum;

				//��������
				bds_subfrPartEph_D2_N[prn - 1].page7.cic_L8 = get_unsigned_8bits( msg, 43, 51 );
				bds_subfrPartEph_D2_N[prn - 1].page7.cis = get_signed_32bits( msg, 51, 69, 0 );
				bds_subfrPartEph_D2_N[prn - 1].page7.toe = get_unsigned_32bits( msg, 69, 86 );
				bds_subfrPartEph_D2_N[prn - 1].page7.i0_U21 = get_unsigned_32bits( msg, 86, 107 );
			}

			//page 8
			if( 8 == pnum ){
				bds_subfrPartEph_D2_N[prn - 1].page8.fraID = fraID;
				bds_subfrPartEph_D2_N[prn - 1].page8.sow = sow;
				bds_subfrPartEph_D2_N[prn - 1].page8.pnum1 = pnum;

				//��������
				bds_subfrPartEph_D2_N[prn - 1].page8.i0_L11 = get_unsigned_16bits( msg, 43, 54 );
				bds_subfrPartEph_D2_N[prn - 1].page8.crc = get_signed_32bits( msg, 54, 72, 0 );
				bds_subfrPartEph_D2_N[prn - 1].page8.crs = get_signed_32bits( msg, 72, 90, 0 );
				bds_subfrPartEph_D2_N[prn - 1].page8.omega_dot_U19 = get_unsigned_32bits( msg, 90, 109 );
			}

			//page 9
			if( 9 == pnum ){
				bds_subfrPartEph_D2_N[prn - 1].page9.fraID = fraID;
				bds_subfrPartEph_D2_N[prn - 1].page9.sow = sow;
				bds_subfrPartEph_D2_N[prn - 1].page9.pnum1 = pnum;

				//��������
				bds_subfrPartEph_D2_N[prn - 1].page9.omega_dot_L5 = get_unsigned_8bits( msg, 43, 48 );
				bds_subfrPartEph_D2_N[prn - 1].page9.omega0 = get_signed_32bits( msg, 48, 80, 0 );
				bds_subfrPartEph_D2_N[prn - 1].page9.w_U27 = get_unsigned_32bits( msg, 80, 107 );
			}

			//page 10
			if( 10 == pnum ){
				bds_subfrPartEph_D2_N[prn - 1].page10.fraID = fraID;
				bds_subfrPartEph_D2_N[prn - 1].page10.sow = sow;
				bds_subfrPartEph_D2_N[prn - 1].page10.pnum1 = pnum;

				//��������
				bds_subfrPartEph_D2_N[prn - 1].page10.w_L5 = get_unsigned_8bits( msg, 43, 48 );
				bds_subfrPartEph_D2_N[prn - 1].page10.idot = get_signed_16bits( msg, 48, 62, 0 );
			}
		}

		if( fraID == 5 ){
			pnum = get_unsigned_8bits( msg, 40, 47 );
			
			if( pnum == 36 )
			{
				bds_subfrPartAlm_D2_N[prn - 1].page36.fraID =fraID;
				bds_subfrPartAlm_D2_N[prn - 1].page36.sow = sow;
				bds_subfrPartAlm_D2_N[prn - 1].page36.pnum = pnum;

				bds_subfrPartAlm_D2_N[prn - 1].page36.wna = get_unsigned_8bits( msg, 146, 154 );
				bds_subfrPartAlm_D2_N[prn - 1].page36.toa = get_unsigned_8bits( msg, 154, 162 );
			}

			if( ( pnum >= 37 && pnum <= 60 ) || ( pnum >= 95 && pnum <= 100 ) )
			{
				int cnt;

				if( pnum < 80 ) cnt = pnum - 37;
				else            cnt = pnum - 95 + 24;

				bds_subfrPartAlm_D2_N[prn - 1].pageT30[cnt].fraID =fraID;
				bds_subfrPartAlm_D2_N[prn - 1].pageT30[cnt].sow = sow;
				bds_subfrPartAlm_D2_N[prn - 1].pageT30[cnt].pnum = pnum;

				bds_subfrPartAlm_D2_N[prn - 1].pageT30[cnt].sqrtA = get_unsigned_32bits( msg, 47, 71 );
				bds_subfrPartAlm_D2_N[prn - 1].pageT30[cnt].a1 = get_signed_16bits( msg, 71, 82, 0 );
				bds_subfrPartAlm_D2_N[prn - 1].pageT30[cnt].a0 = get_signed_16bits( msg, 82, 93, 0 );
				bds_subfrPartAlm_D2_N[prn - 1].pageT30[cnt].omega0 = get_signed_32bits( msg, 93, 117, 0 );
				bds_subfrPartAlm_D2_N[prn - 1].pageT30[cnt].e = get_unsigned_32bits( msg, 117, 134 );
				bds_subfrPartAlm_D2_N[prn - 1].pageT30[cnt].deltaI = get_signed_16bits( msg, 134, 150, 0 );
				bds_subfrPartAlm_D2_N[prn - 1].pageT30[cnt].toa = get_unsigned_8bits( msg, 150, 158 );
				bds_subfrPartAlm_D2_N[prn - 1].pageT30[cnt].omega_dot = get_signed_32bits( msg, 158, 175, 0 );
				bds_subfrPartAlm_D2_N[prn - 1].pageT30[cnt].w = get_signed_32bits( msg, 175, 199, 0 );
				bds_subfrPartAlm_D2_N[prn - 1].pageT30[cnt].m0 = get_signed_32bits( msg, 199, 223, 0 );
			}

			if( pnum == 101 )
			{
				bds_subfrPartAlm_D2_N[prn - 1].page101.fraID =fraID;
				bds_subfrPartAlm_D2_N[prn - 1].page101.sow = sow;
				bds_subfrPartAlm_D2_N[prn - 1].page101.pnum = pnum;

				bds_subfrPartAlm_D2_N[prn - 1].page101.a0_gps = get_signed_16bits( msg, 77, 91, 0 );
				bds_subfrPartAlm_D2_N[prn - 1].page101.a1_gps = get_signed_16bits( msg, 91, 107, 0 );
				bds_subfrPartAlm_D2_N[prn - 1].page101.a0_gal = get_signed_16bits( msg, 107, 121, 0 );
				bds_subfrPartAlm_D2_N[prn - 1].page101.a1_gal = get_signed_16bits( msg, 121, 137, 0 );
				bds_subfrPartAlm_D2_N[prn - 1].page101.a0_glo = get_signed_16bits( msg, 137, 151, 0 );
				bds_subfrPartAlm_D2_N[prn - 1].page101.a0_glo = get_signed_16bits( msg, 151, 167, 0 );
			}

			if( pnum == 102 )
			{
				bds_subfrPartAlm_D2_N[prn - 1].page102.fraID =fraID;
				bds_subfrPartAlm_D2_N[prn - 1].page102.sow = sow;
				bds_subfrPartAlm_D2_N[prn - 1].page102.pnum = pnum;

				//��������
				bds_subfrPartAlm_D2_N[prn - 1].page102.deltT_LS = get_signed_8bits( msg, 47, 55, 0 );
				bds_subfrPartAlm_D2_N[prn - 1].page102.deltT_LSF = get_signed_8bits( msg, 55, 63, 0 );
				bds_subfrPartAlm_D2_N[prn - 1].page102.wn_LSF = get_unsigned_8bits( msg, 63, 71 );
				bds_subfrPartAlm_D2_N[prn - 1].page102.a0_UTC = get_signed_32bits( msg, 71, 103, 0 );
				bds_subfrPartAlm_D2_N[prn - 1].page102.a1_UTC = get_signed_32bits( msg, 103, 127, 0 );
				bds_subfrPartAlm_D2_N[prn - 1].page102.dn = get_unsigned_8bits( msg, 127, 135 );

				//UTC model update
				flag[3] = 1; 
			}
		}

		//eph update check
		if( bds_subfrPartEph_D2_N[prn - 1].page1.pnum1 == 1 &&
			bds_subfrPartEph_D2_N[prn - 1].page2.pnum1 == 2 &&
			bds_subfrPartEph_D2_N[prn - 1].page3.pnum1 == 3 &&
			bds_subfrPartEph_D2_N[prn - 1].page4.pnum1 == 4 &&
			bds_subfrPartEph_D2_N[prn - 1].page5.pnum1 == 5 &&
			bds_subfrPartEph_D2_N[prn - 1].page6.pnum1 == 6 &&
			bds_subfrPartEph_D2_N[prn - 1].page7.pnum1 == 7 &&
			bds_subfrPartEph_D2_N[prn - 1].page8.pnum1 == 8 &&
			bds_subfrPartEph_D2_N[prn - 1].page9.pnum1 == 9 &&
			bds_subfrPartEph_D2_N[prn - 1].page10.pnum1 == 10 )
		{
			double dt1,dt2,dt3,dt4,dt5,dt6,dt7,dt8,dt9,dt10;

			dt1  = floor((double)bds_subfrPartEph_D2_N[prn - 1].page1.sow/3600.0 );
			dt2  = floor((double)bds_subfrPartEph_D2_N[prn - 1].page2.sow/3600.0 );
			dt3  = floor((double)bds_subfrPartEph_D2_N[prn - 1].page3.sow/3600.0 );
			dt4  = floor((double)bds_subfrPartEph_D2_N[prn - 1].page4.sow/3600.0 );
			dt5  = floor((double)bds_subfrPartEph_D2_N[prn - 1].page5.sow/3600.0 );
			dt6  = floor((double)bds_subfrPartEph_D2_N[prn - 1].page6.sow/3600.0 );
			dt7  = floor((double)bds_subfrPartEph_D2_N[prn - 1].page7.sow/3600.0 );
			dt8  = floor((double)bds_subfrPartEph_D2_N[prn - 1].page8.sow/3600.0 );
			dt9  = floor((double)bds_subfrPartEph_D2_N[prn - 1].page9.sow/3600.0 );
			dt10 = floor((double)bds_subfrPartEph_D2_N[prn - 1].page10.sow/3600.0 );

			if( (dt1 == dt2) && (dt1 == dt3) && (dt1 == dt4) && (dt1 == dt5)  && (dt1 == dt6) &&
				(dt1 == dt7) && (dt1 == dt8) && (dt1 == dt9) && (dt1 == dt10) )
			{
				flag[0] = 1;
			}
		}

		//alm update check
		if( bds_subfrPartAlm_D2_N[prn - 1].page36.pnum == 36 ){
			flag[1] = 1;
		}
	}
	else if( prn >= 6 && prn <= 35 )
	{
		switch(fraID){
		case 1:
			{
				bds_subfrPartEph_D1_N[prn - 6].subfr1.fraID = fraID;
				bds_subfrPartEph_D1_N[prn - 6].subfr1.sow = sow;

				bds_subfrPartEph_D1_N[prn - 6].subfr1.satH1 = get_unsigned_8bits( msg, 39, 40 );
				bds_subfrPartEph_D1_N[prn - 6].subfr1.aodc = get_unsigned_8bits( msg, 40, 45 );
				bds_subfrPartEph_D1_N[prn - 6].subfr1.urai = get_unsigned_8bits( msg, 45, 49 );
				bds_subfrPartEph_D1_N[prn - 6].subfr1.wn = get_unsigned_16bits( msg, 49, 62 );
				bds_subfrPartEph_D1_N[prn - 6].subfr1.toc = get_unsigned_32bits( msg, 62, 79 );
				bds_subfrPartEph_D1_N[prn - 6].subfr1.tgd1 = get_signed_16bits( msg, 79, 89, 0 );
				bds_subfrPartEph_D1_N[prn - 6].subfr1.tgd2 = get_signed_16bits( msg, 89, 99, 0 );
				bds_subfrPartEph_D1_N[prn - 6].subfr1.alfa0 = get_signed_8bits( msg, 99, 107, 0 );
				bds_subfrPartEph_D1_N[prn - 6].subfr1.alfa1 = get_signed_8bits( msg, 107, 115, 0 );
				bds_subfrPartEph_D1_N[prn - 6].subfr1.alfa2 = get_signed_8bits( msg, 115, 123, 0 );
				bds_subfrPartEph_D1_N[prn - 6].subfr1.alfa3 = get_signed_8bits( msg, 123, 131, 0 );
				bds_subfrPartEph_D1_N[prn - 6].subfr1.beta0 = get_signed_8bits( msg, 131, 139, 0 );
				bds_subfrPartEph_D1_N[prn - 6].subfr1.beta1 = get_signed_8bits( msg, 139, 147, 0 );
				bds_subfrPartEph_D1_N[prn - 6].subfr1.beta2 = get_signed_8bits( msg, 147, 155, 0 );
				bds_subfrPartEph_D1_N[prn - 6].subfr1.beta3 = get_signed_8bits( msg, 155, 163, 0 );
				bds_subfrPartEph_D1_N[prn - 6].subfr1.a2 = get_signed_16bits( msg, 163, 174, 0 );
				bds_subfrPartEph_D1_N[prn - 6].subfr1.a0 = get_signed_32bits( msg, 174, 198, 0 );
				bds_subfrPartEph_D1_N[prn - 6].subfr1.a1 = get_signed_32bits( msg, 198, 220, 0 );
				bds_subfrPartEph_D1_N[prn - 6].subfr1.aode = get_unsigned_8bits( msg, 220, 225 );

				//iono update
				flag[2] = 1;

				break;
			}
		case 2:
			{
				//Ϊ�������ָ�ֵ
				bds_subfrPartEph_D1_N[prn - 6].subfr2.fraID = fraID;
				bds_subfrPartEph_D1_N[prn - 6].subfr2.sow = sow;

				//��������
				bds_subfrPartEph_D1_N[prn - 6].subfr2.deltN = get_signed_16bits( msg, 39, 55, 0 );
				bds_subfrPartEph_D1_N[prn - 6].subfr2.cuc = get_signed_32bits( msg, 55, 73, 0 );
				bds_subfrPartEph_D1_N[prn - 6].subfr2.m0 = get_signed_32bits( msg, 73, 105, 0);
				bds_subfrPartEph_D1_N[prn - 6].subfr2.e = get_unsigned_32bits( msg, 105, 137 );
				bds_subfrPartEph_D1_N[prn - 6].subfr2.cus = get_signed_32bits( msg, 137, 155, 0 );
				bds_subfrPartEph_D1_N[prn - 6].subfr2.crc = get_signed_32bits( msg, 155, 173, 0 );
				bds_subfrPartEph_D1_N[prn - 6].subfr2.crs = get_signed_32bits( msg, 173, 191, 0 );
				bds_subfrPartEph_D1_N[prn - 6].subfr2.sqrtA = get_unsigned_32bits( msg, 191, 223 );
				bds_subfrPartEph_D1_N[prn - 6].subfr2.toe_U2 = get_unsigned_8bits( msg, 223, 225 );

				break;
			}
		case 3:
			{
				//Ϊ�������ָ�ֵ
				bds_subfrPartEph_D1_N[prn - 6].subfr3.fraID = fraID;
				bds_subfrPartEph_D1_N[prn - 6].subfr3.sow = sow;

				//��������
				bds_subfrPartEph_D1_N[prn - 6].subfr3.toe_L15 = get_unsigned_16bits( msg, 39, 54 );
				bds_subfrPartEph_D1_N[prn - 6].subfr3.i0 = get_signed_32bits( msg, 54, 86, 0 );
				bds_subfrPartEph_D1_N[prn - 6].subfr3.cic = get_signed_32bits( msg, 86, 104, 0 );
				bds_subfrPartEph_D1_N[prn - 6].subfr3.omega_dot = get_signed_32bits( msg, 104, 128, 0 );
				bds_subfrPartEph_D1_N[prn - 6].subfr3.cis = get_signed_32bits( msg, 128, 146, 0 );
				bds_subfrPartEph_D1_N[prn - 6].subfr3.idot = get_signed_16bits( msg, 146, 160, 0 );
				bds_subfrPartEph_D1_N[prn - 6].subfr3.omega0 = get_signed_32bits( msg, 160, 192, 0 );
				bds_subfrPartEph_D1_N[prn - 6].subfr3.w = get_signed_32bits( msg, 192, 224, 0 );

				break;
			}
		case 4:
			{
				pnum = get_unsigned_8bits( msg, 40, 47 );

				//if( pnum != header[4] ){
				//	SYS_LOGGING(OBJ_SD,LOG_ERROR,"pnum compare failed!,%s,%d",__FUNCTION__,__LINE__);
				//	return FALSE;
				//}

				if( pnum > 0 && pnum < 25 )
				{
					bds_subfrPartAlm_D1_N[prn - 6].subfr4_PageT24[pnum - 1].fraID = fraID;
					bds_subfrPartAlm_D1_N[prn - 6].subfr4_PageT24[pnum - 1].sow = sow;
					bds_subfrPartAlm_D1_N[prn - 6].subfr4_PageT24[pnum - 1].pnum = pnum;

					//��������
					bds_subfrPartAlm_D1_N[prn - 6].subfr4_PageT24[pnum - 1].sqrtA = get_unsigned_32bits( msg, 47, 71 );
					bds_subfrPartAlm_D1_N[prn - 6].subfr4_PageT24[pnum - 1].a1 = get_signed_16bits( msg, 71, 82, 0 );
					bds_subfrPartAlm_D1_N[prn - 6].subfr4_PageT24[pnum - 1].a0 = get_signed_16bits( msg, 82, 93, 0 );
					bds_subfrPartAlm_D1_N[prn - 6].subfr4_PageT24[pnum - 1].omega0 = get_signed_32bits( msg, 93, 117, 0 );
					bds_subfrPartAlm_D1_N[prn - 6].subfr4_PageT24[pnum - 1].e = get_unsigned_32bits( msg, 117, 134 );
					bds_subfrPartAlm_D1_N[prn - 6].subfr4_PageT24[pnum - 1].deltaI = get_signed_16bits( msg, 134, 150, 0 );
					bds_subfrPartAlm_D1_N[prn - 6].subfr4_PageT24[pnum - 1].toa = get_unsigned_8bits( msg, 150, 158 );
					bds_subfrPartAlm_D1_N[prn - 6].subfr4_PageT24[pnum - 1].omega_dot = get_signed_32bits( msg, 158, 175, 0 );
					bds_subfrPartAlm_D1_N[prn - 6].subfr4_PageT24[pnum - 1].w = get_signed_32bits( msg, 175, 199, 0 );
					bds_subfrPartAlm_D1_N[prn - 6].subfr4_PageT24[pnum - 1].m0 = get_signed_32bits( msg, 199, 223, 0 );
				}

				break;
			}
		case 5:
			{
				pnum = get_unsigned_8bits( msg, 40, 47 );

				//if( pnum != header[4] ){
				//	SYS_LOGGING(OBJ_SD,LOG_ERROR,"pnum compare failed!,%s,%d",__FUNCTION__,__LINE__);
				//	return FALSE;
				//}

				if( pnum > 0 && pnum < 7 )
				{
					//Ϊ�������ָ�ֵ
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_PageT6[pnum - 1].fraID =fraID;
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_PageT6[pnum - 1].sow = sow;
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_PageT6[pnum - 1].pnum = pnum;

					//��������
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_PageT6[pnum - 1].sqrtA = get_unsigned_32bits( msg, 47, 71 );
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_PageT6[pnum - 1].a1 = get_signed_16bits( msg, 71, 82, 0 );
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_PageT6[pnum - 1].a0 = get_signed_16bits( msg, 82, 93, 0 );
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_PageT6[pnum - 1].omega0 = get_signed_32bits( msg, 93, 117, 0 );
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_PageT6[pnum - 1].e = get_unsigned_32bits( msg, 117, 134 );
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_PageT6[pnum - 1].deltaI = get_signed_16bits( msg, 134, 150, 0 );
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_PageT6[pnum - 1].toa = get_unsigned_8bits( msg, 150, 158 );
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_PageT6[pnum - 1].omega_dot = get_signed_32bits( msg, 158, 175, 0 );
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_PageT6[pnum - 1].w = get_signed_32bits( msg, 175, 199, 0 );
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_PageT6[pnum - 1].m0 = get_signed_32bits( msg, 199, 223, 0 );
				}

				if( pnum == 8 )
				{
					//Ϊ�������ָ�ֵ
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_page8.fraID = fraID;
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_page8.sow = sow;
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_page8.pnum = pnum;

					bds_subfrPartAlm_D1_N[prn - 6].subfr5_page8.wna = get_unsigned_8bits( msg, 146, 154 );
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_page8.toa = get_unsigned_8bits( msg, 154, 162 );
				}

				if( pnum == 9 )
				{
					//Ϊ�������ָ�ֵ
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_page9.fraID =fraID;
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_page9.sow = sow;
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_page9.pnum = pnum;

					//��������
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_page9.a0_gps = get_signed_16bits( msg, 77, 91, 0 );
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_page9.a1_gps = get_signed_16bits( msg, 91, 107, 0 );
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_page9.a0_gal = get_signed_16bits( msg, 107, 121, 0 );
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_page9.a1_gal = get_signed_16bits( msg, 121, 137, 0 );
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_page9.a0_glo = get_signed_16bits( msg, 137, 151, 0 );
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_page9.a0_glo = get_signed_16bits( msg, 151, 167, 0 );
				}

				if( pnum == 10 )
				{
					//Ϊ�������ָ�ֵ
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_page10.fraID =fraID;
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_page10.sow = sow;
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_page10.pnum = pnum;

					//��������
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_page10.deltT_LS = get_signed_8bits( msg, 47, 55, 0 );
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_page10.deltT_LSF = get_signed_8bits( msg, 55, 63, 0 );
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_page10.wn_LSF = get_unsigned_8bits( msg, 63, 71 );
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_page10.a0_UTC = get_signed_32bits( msg, 71, 103, 0 );
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_page10.a1_UTC = get_signed_32bits( msg, 103, 127, 0 );
					bds_subfrPartAlm_D1_N[prn - 6].subfr5_page10.dn = get_unsigned_8bits( msg, 127, 135 );

					//UTC model
					flag[3] = 1;
				}

				break;
			}
		}

		//eph update 
		if( bds_subfrPartEph_D1_N[prn - 6].subfr1.fraID == 1 &&
			bds_subfrPartEph_D1_N[prn - 6].subfr2.fraID == 2 &&
			bds_subfrPartEph_D1_N[prn - 6].subfr3.fraID == 3 )
		{
			double dt1, dt2, dt3;

			dt1 = floor((double)(bds_subfrPartEph_D1_N[prn - 6].subfr1.sow)/3600.0);
			dt2 = floor((double)(bds_subfrPartEph_D1_N[prn - 6].subfr2.sow)/3600.0);
			dt3 = floor((double)(bds_subfrPartEph_D1_N[prn - 6].subfr3.sow)/3600.0);
			
			if( (dt1 == dt2) && (dt2 == dt3) )
			    flag[0] = 1;
		}

		//alm update
		if( bds_subfrPartAlm_D1_N[prn - 6].subfr5_page8.pnum == 8 ){
			flag[1] = 1;
		}

	}
	return TRUE;
}

//*****************GALILEO_PART************************
//*******F/NAV*************
uint8_t exchange_gal_iono_N( uint8_t nav_type, pagePart_Eph_F_GAL_t gal_pagePartEph_F_N, wordPart_Eph_I_GAL_t gal_wordPartEph_I_N, GAL_IONO_INFO *gal_iono )
{
	//inputs check
	if( NULL == gal_iono ){
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs err!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}
	memset( gal_iono, 0, sizeof(GAL_IONO_INFO) );

	//*****
	if( nav_type == 0 ){
		gal_iono->a_i0 = gal_pagePartEph_F_N.page1.a_i0*0.25;
		gal_iono->a_i1 = gal_pagePartEph_F_N.page1.a_i1*P2_06*0.25;
		gal_iono->a_i2 = gal_pagePartEph_F_N.page1.a_i1*P2_15;
		gal_iono->sf1  = gal_pagePartEph_F_N.page1.region_1;
		gal_iono->sf2  = gal_pagePartEph_F_N.page1.region_2;
		gal_iono->sf3  = gal_pagePartEph_F_N.page1.region_3;
		gal_iono->sf4  = gal_pagePartEph_F_N.page1.region_4;
		gal_iono->sf5  = gal_pagePartEph_F_N.page1.region_5;
		gal_iono->have_iono = 1;
	}
	else if( nav_type == 1 ){
		gal_iono->a_i0 = gal_wordPartEph_I_N.word5.a_i0*0.25;
		gal_iono->a_i1 = gal_wordPartEph_I_N.word5.a_i1*P2_06*0.25;
		gal_iono->a_i2 = gal_wordPartEph_I_N.word5.a_i1*P2_15;
		gal_iono->sf1  = gal_wordPartEph_I_N.word5.region_1;
		gal_iono->sf2  = gal_wordPartEph_I_N.word5.region_2;
		gal_iono->sf3  = gal_wordPartEph_I_N.word5.region_3;
		gal_iono->sf4  = gal_wordPartEph_I_N.word5.region_4;
		gal_iono->sf5  = gal_wordPartEph_I_N.word5.region_5;
		gal_iono->have_iono = 1;
	}
	else{
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"Not supported iono type for GAL!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}
	return TRUE;
}

uint8_t exchange_gal_utc_N( uint8_t nav_type, pagePart_Eph_F_GAL_t gal_pagePartEph_F_N, wordPart_Eph_I_GAL_t gal_wordPartEph_I_N, galUtcModel_t *gal_utc )
{
	//inputs check
	if( NULL == gal_utc ){
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs err!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}

	memset( gal_utc, 0, sizeof(galUtcModel_t) );

	if( nav_type == 0 ){
		gal_utc->a0        = gal_pagePartEph_F_N.page4.a0*P2_30;
		gal_utc->a1        = gal_pagePartEph_F_N.page4.a1*P2_50;
		gal_utc->deltT_ls  = gal_pagePartEph_F_N.page4.delt_tls;
		gal_utc->deltT_lsf = gal_pagePartEph_F_N.page4.delt_tlsf;
		gal_utc->dn        = gal_pagePartEph_F_N.page4.dn;
		gal_utc->t_0t      = (double)gal_pagePartEph_F_N.page4.t0_t*3600;
		gal_utc->wn_0t     = gal_pagePartEph_F_N.page4.wn0_t;
		gal_utc->wn_lsf    = gal_pagePartEph_F_N.page4.wn_lsf;
		gal_utc->have_utc  = 1;
	}
	else if( nav_type == 1 ){
		gal_utc->a0        = gal_wordPartEph_I_N.word6.a0*P2_30;
		gal_utc->a1        = gal_wordPartEph_I_N.word6.a1*P2_50;
		gal_utc->deltT_ls  = gal_wordPartEph_I_N.word6.delt_tls;
		gal_utc->deltT_lsf = gal_wordPartEph_I_N.word6.delt_tlsf;
		gal_utc->dn        = gal_wordPartEph_I_N.word6.dn;
		gal_utc->t_0t      = (double)gal_wordPartEph_I_N.word6.t0_t*3600;
		gal_utc->wn_0t     = gal_wordPartEph_I_N.word6.wn0_t;
		gal_utc->wn_lsf    = gal_wordPartEph_I_N.word6.wn_lsf;
		gal_utc->have_utc  = 1;
	}
	else{
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"Not supported utc model type for GAL!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}

	return TRUE;
}

uint8_t exchange_gal_eph_N( 
	uint8_t nav_type, 
	uint16_t prn, 
	pagePart_Eph_F_GAL_t *gal_pagePartEph_F_N, 
	wordPart_Eph_I_GAL_t *gal_wordPartEph_I_N, 
	GAL_EPH_INFO *gal_eph )
{
	//inputs check
	if( NULL == gal_eph ){
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs err!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}

	memset( gal_eph, 0, sizeof(GAL_EPH_INFO) );

	//F/NAV
	if( nav_type == 0 ){
		gal_eph->eph_source          = EPH_SOURCE_BRDC;
		gal_eph->eph_status          = EPH_STATUS_VALID;
		gal_eph->prn                 = (uint8_t)(prn);
		gal_eph->svHealth            = gal_pagePartEph_F_N->page1.E5a_DVS;

		gal_eph->af0                 = gal_pagePartEph_F_N->page1.a_f0*P2_35*2;
		gal_eph->af1                 = (float)(gal_pagePartEph_F_N->page1.a_f1*P2_48*4);
		gal_eph->af2                 = (float)(gal_pagePartEph_F_N->page1.a_f2*P2_48*P2_11);
		gal_eph->cic                 = (float)(gal_pagePartEph_F_N->page4.cic*P2_30*2);
		gal_eph->cis                 = (float)(gal_pagePartEph_F_N->page4.cis*P2_30*2);
		gal_eph->crc                 = (float)(gal_pagePartEph_F_N->page3.crc*P2_06*2);
		gal_eph->crs                 = (float)(gal_pagePartEph_F_N->page3.crs*P2_06*2);
		gal_eph->cuc                 = (float)(gal_pagePartEph_F_N->page3.cuc*P2_30*2);
		gal_eph->cus                 = (float)(gal_pagePartEph_F_N->page3.cus*P2_30*2);
		gal_eph->delta_n             = gal_pagePartEph_F_N->page3.delt_n*P2_43*PI;
		gal_eph->ecc                 = gal_pagePartEph_F_N->page2.e*P2_33;
		gal_eph->i0                  = gal_pagePartEph_F_N->page3.i0*P2_33*4*PI;
		gal_eph->idot                = gal_pagePartEph_F_N->page2.i_dot*P2_43*PI;
		gal_eph->M0                  = gal_pagePartEph_F_N->page2.m0*P2_33*4*PI;
		gal_eph->OMEGA_0             = gal_pagePartEph_F_N->page2.omega_0*P2_33*4*PI;
		gal_eph->OMEGA_Dot           = gal_pagePartEph_F_N->page2.omega_dot*P2_43*PI;
		gal_eph->sqrta               = gal_pagePartEph_F_N->page2.sqrt_a*P2_19;
		gal_eph->toc                 = gal_pagePartEph_F_N->page1.toc*60;
		gal_eph->toe                 = gal_pagePartEph_F_N->page3.toe*60;
		gal_eph->w                   = gal_pagePartEph_F_N->page3.w*P2_33*4*PI;
		gal_eph->WN                  = gal_pagePartEph_F_N->page1.wn;
		gal_eph->bgd_e5a             = gal_pagePartEph_F_N->page1.bgd_E1_E5a*P2_32;
		gal_eph->bgd_e5b             = 0;

		//clear cache
		memset( gal_pagePartEph_F_N, 0, sizeof(pagePart_Eph_F_GAL_t) );

	}
	else if( nav_type == 1 ){
		gal_eph->eph_source          = EPH_SOURCE_BRDC;
		gal_eph->eph_status          = EPH_STATUS_VALID;
		gal_eph->prn                 = (uint8_t)(prn);
		gal_eph->svHealth            = gal_wordPartEph_I_N->word5.E1B_DVS;

		gal_eph->af0                 = gal_wordPartEph_I_N->word4.a_f0*P2_35*2;
		gal_eph->af1                 = (float)(gal_wordPartEph_I_N->word4.a_f1*P2_48*4);
		gal_eph->af2                 = (float)(gal_wordPartEph_I_N->word4.a_f2*P2_48*P2_11);
		gal_eph->cic                 = (float)(gal_wordPartEph_I_N->word4.cic*P2_30*2);
		gal_eph->cis                 = (float)(gal_wordPartEph_I_N->word4.cis*P2_30*2);
		gal_eph->crc                 = (float)(gal_wordPartEph_I_N->word3.crc*P2_06*2);
		gal_eph->crs                 = (float)(gal_wordPartEph_I_N->word3.crs*P2_06*2);
		gal_eph->cuc                 = (float)(gal_wordPartEph_I_N->word3.cuc*P2_30*2);
		gal_eph->cus                 = (float)(gal_wordPartEph_I_N->word3.cus*P2_30*2);
		gal_eph->delta_n             = gal_wordPartEph_I_N->word3.delt_n*P2_43*PI;
		gal_eph->ecc                 = gal_wordPartEph_I_N->word1.e*P2_33;
		gal_eph->i0                  = gal_wordPartEph_I_N->word2.i0*P2_33*4*PI;
		gal_eph->idot                = gal_wordPartEph_I_N->word2.i_dot*P2_43*PI;
		gal_eph->M0                  = gal_wordPartEph_I_N->word1.m0*P2_33*4*PI;
		gal_eph->OMEGA_0             = gal_wordPartEph_I_N->word2.omega_0*P2_33*4*PI;
		gal_eph->OMEGA_Dot           = gal_wordPartEph_I_N->word3.omega_dot*P2_43*PI;
		gal_eph->sqrta               = gal_wordPartEph_I_N->word1.sqrt_A*P2_19;
		gal_eph->toc                 = gal_wordPartEph_I_N->word4.toc*60;
		gal_eph->toe                 = gal_wordPartEph_I_N->word1.toe*60;
		gal_eph->w                   = gal_wordPartEph_I_N->word2.w*P2_33*4*PI;
		gal_eph->WN                  = gal_wordPartEph_I_N->word5.wn;
		gal_eph->bgd_e5a             = gal_wordPartEph_I_N->word5.bgd_E1_E5a*P2_32;
		gal_eph->bgd_e5b             = gal_wordPartEph_I_N->word5.bgd_E1_E5b*P2_32;

		//clear cache
		memset( &gal_wordPartEph_I_N->word1, 0, sizeof( Word1_I_GAL_t ) );
		memset( &gal_wordPartEph_I_N->word2, 0, sizeof( Word2_I_GAL_t ) );
		memset( &gal_wordPartEph_I_N->word3, 0, sizeof( Word3_I_GAL_t ) );
		memset( &gal_wordPartEph_I_N->word4, 0, sizeof( Word4_I_GAL_t ) );
		memset( &gal_wordPartEph_I_N->word5, 0, sizeof( Word5_I_GAL_t ) );
	}
	else{
		SYS_LOGGING(OBJ_SD,LOG_ERROR," not supported nav type of GAL!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}

	return TRUE;
}

uint8_t exchange_gal_alm_N( uint8_t nav_type, pagePart_Alm_F_GAL_t *gal_pagePartAlm_F_N, wordPart_Alm_I_GAL_t *gal_wordPartAlm_I_N, GAL_ALM_INFO *gal_alm )
{
	uint8_t   temp_str[3] = "\0";
	uint16_t  temp1;

	//inputs check
	if( NULL == gal_alm ){
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs err!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}

	//*******************************
	memset( gal_alm, 0, sizeof(GAL_ALM_INFO)*3 );
	if( nav_type == 0 ){
		//SVID_1
		gal_alm[0].alm_status       = TRUE;
		gal_alm[0].alm_source       = 0;
		gal_alm[0].svID             = gal_pagePartAlm_F_N->page5.svID_1;
		gal_alm[0].af0              = gal_pagePartAlm_F_N->page5.a_f0_1 *P2_19;
		gal_alm[0].af1              = gal_pagePartAlm_F_N->page5.a_f1_1*P2_21*P2_17;
		gal_alm[0].e                = gal_pagePartAlm_F_N->page5.e_1*P2_17*2;
		gal_alm[0].IOD_a            = gal_pagePartAlm_F_N->page5.iod_a;
		gal_alm[0].m0               = gal_pagePartAlm_F_N->page5.m0_1*P2_15;
		gal_alm[0].omega_0          = gal_pagePartAlm_F_N->page5.omega0_1*P2_15;
		gal_alm[0].omega_dot        = gal_pagePartAlm_F_N->page5.omega_dot_1*P2_33;
		gal_alm[0].sigma_i          = gal_pagePartAlm_F_N->page5.sigma_i_1*P2_15*2;
		gal_alm[0].sqrt_A           = gal_pagePartAlm_F_N->page5.sqrt_a_1*P2_11*4;
		gal_alm[0].t_0a             = (double)gal_pagePartAlm_F_N->page5.t0_a*600;
		gal_alm[0].w                = gal_pagePartAlm_F_N->page5.w_1*P2_15;
		gal_alm[0].wn_a             = gal_pagePartAlm_F_N->page5.wn_a;
		gal_alm[0].e1b_hs           = 0;
		gal_alm[0].e5a_hs           = gal_pagePartAlm_F_N->page5.e5a_hs_1;
		gal_alm[0].e5b_hs           = 0;


		//SVID_2
		gal_alm[1].svID             = gal_pagePartAlm_F_N->page5.svID_2;
		gal_alm[1].af0              = gal_pagePartAlm_F_N->page6.a_f0_2*P2_19;
		gal_alm[1].af1              = gal_pagePartAlm_F_N->page6.a_f1_2*P2_21*P2_17;
		gal_alm[1].e                = gal_pagePartAlm_F_N->page5.e_2*P2_17*2;
		gal_alm[1].IOD_a            = gal_pagePartAlm_F_N->page5.iod_a;
		gal_alm[1].m0               = gal_pagePartAlm_F_N->page6.m0_2*P2_15;

		temp_str[0]                 = (uint8_t)(gal_pagePartAlm_F_N->page6.omega0_L12_2 >> 8);
		temp_str[0]                 = (uint8_t)( temp_str[0] | (gal_pagePartAlm_F_N->page5.omega0_U4_2 << 4) );
		temp_str[1]                 = (uint8_t)(gal_pagePartAlm_F_N->page6.omega0_L12_2 & 0x00FF);
		temp1                       = get_signed_16bits( temp_str, 1, 17, 0 );
		gal_alm[1].omega_0          = temp1*P2_15;

		gal_alm[1].omega_dot        = gal_pagePartAlm_F_N->page6.omega_dot_2*P2_33;
		gal_alm[1].sigma_i          = gal_pagePartAlm_F_N->page5.sigma_i_2*P2_15*2;
		gal_alm[1].sqrt_A           = gal_pagePartAlm_F_N->page5.sqrt_a_2*P2_11*4;
		gal_alm[1].t_0a             = (double)gal_pagePartAlm_F_N->page5.t0_a*600;
		gal_alm[1].w                = gal_pagePartAlm_F_N->page5.w_2*P2_15;
		gal_alm[1].wn_a             = gal_pagePartAlm_F_N->page5.wn_a;
		gal_alm[1].e1b_hs           = 0;
		gal_alm[1].e5a_hs           = gal_pagePartAlm_F_N->page6.e5a_hs_2;
		gal_alm[1].e5b_hs           = 0;

		//SVID_3
		gal_alm[2].svID             = gal_pagePartAlm_F_N->page6.svID_3;
		gal_alm[2].af0              = gal_pagePartAlm_F_N->page6.a_f0_3*P2_19;
		gal_alm[2].af1              = gal_pagePartAlm_F_N->page6.a_f1_3*P2_21*P2_17;
		gal_alm[2].e                = gal_pagePartAlm_F_N->page6.e_3*P2_17*2;
		gal_alm[2].IOD_a            = gal_pagePartAlm_F_N->page6.iod_a;
		gal_alm[2].m0               = gal_pagePartAlm_F_N->page6.m0_3*P2_15;
		gal_alm[2].omega_0          = gal_pagePartAlm_F_N->page6.omega0_3*P2_15;
		gal_alm[2].omega_dot        = gal_pagePartAlm_F_N->page6.omega_dot_3*P2_33;
		gal_alm[2].sigma_i          = gal_pagePartAlm_F_N->page6.sigma_i_3*P2_15*2;
		gal_alm[2].sqrt_A           = gal_pagePartAlm_F_N->page6.sqrt_a_3*P2_11*4;
		gal_alm[2].t_0a             = (double)gal_pagePartAlm_F_N->page5.t0_a*600;
		gal_alm[2].w                = gal_pagePartAlm_F_N->page6.w_3*P2_15;
		gal_alm[2].wn_a             = gal_pagePartAlm_F_N->page5.wn_a;
		gal_alm[2].e1b_hs           = 0;
		gal_alm[2].e5a_hs           = gal_pagePartAlm_F_N->page6.e5a_hs_3;
		gal_alm[2].e5b_hs           = 0;

		//clear cache
		memset( gal_pagePartAlm_F_N, 0, sizeof(pagePart_Alm_F_GAL_t) );
	}
	else if( nav_type == 1 ){
		//SVID_1
		gal_alm[0].svID             = gal_wordPartAlm_I_N->word7.svID_1;
		gal_alm[0].af0              = gal_wordPartAlm_I_N->word8.a_f0_1*P2_19;
		gal_alm[0].af1              = gal_wordPartAlm_I_N->word8.a_f1_1*P2_21*P2_17;
		gal_alm[0].e                = gal_wordPartAlm_I_N->word7.e_1*P2_17*2;
		gal_alm[0].IOD_a            = gal_wordPartAlm_I_N->word7.iod_a;
		gal_alm[0].m0               = gal_wordPartAlm_I_N->word7.m0_1*P2_15;
		gal_alm[0].omega_0          = gal_wordPartAlm_I_N->word7.omega0_1*P2_15;
		gal_alm[0].omega_dot        = gal_wordPartAlm_I_N->word7.omega_dot_1*P2_33;
		gal_alm[0].sigma_i          = gal_wordPartAlm_I_N->word7.sigma_i_1*P2_15*2;
		gal_alm[0].sqrt_A           = gal_wordPartAlm_I_N->word7.sqrt_a_1*P2_11*4;
		gal_alm[0].t_0a             = (double)gal_wordPartAlm_I_N->word7.t0_a*600;
		gal_alm[0].w                = gal_wordPartAlm_I_N->word7.w_1*P2_15;
		gal_alm[0].wn_a             = gal_wordPartAlm_I_N->word7.wn_a;
		gal_alm[0].e1b_hs           = gal_wordPartAlm_I_N->word8.E1B_HS_1;
		gal_alm[0].e5a_hs           = 0;
		gal_alm[0].e5b_hs           = gal_wordPartAlm_I_N->word8.E5b_HS_1;

		//SVID_2
		gal_alm[1].svID             = gal_wordPartAlm_I_N->word8.svID_2;
		gal_alm[1].af0              = gal_wordPartAlm_I_N->word9.a_f0_2*P2_19;
		gal_alm[1].af1              = gal_wordPartAlm_I_N->word9.a_f1_2*P2_21*P2_17;
		gal_alm[1].e                = gal_wordPartAlm_I_N->word8.e_2*P2_17*2;
		gal_alm[1].IOD_a            = gal_wordPartAlm_I_N->word8.iod_a;
		gal_alm[1].m0               = gal_wordPartAlm_I_N->word9.m0_2*P2_15;
		gal_alm[1].omega_0          = gal_wordPartAlm_I_N->word8.omega0_2*P2_15;
		gal_alm[1].omega_dot        = gal_wordPartAlm_I_N->word8.omega_dot_2*P2_33;
		gal_alm[1].sigma_i          = gal_wordPartAlm_I_N->word8.sigma_i_2*P2_15*2;
		gal_alm[1].sqrt_A           = gal_wordPartAlm_I_N->word8.sqrt_a_2*P2_11*4;
		gal_alm[1].t_0a             = (double)gal_wordPartAlm_I_N->word7.t0_a*600;
		gal_alm[1].w                = gal_wordPartAlm_I_N->word8.w_2*P2_15;
		gal_alm[1].wn_a             = gal_wordPartAlm_I_N->word7.wn_a;
		gal_alm[1].e1b_hs           = gal_wordPartAlm_I_N->word9.E1B_HS_2;
		gal_alm[1].e5a_hs           = 0;
		gal_alm[1].e5b_hs           = gal_wordPartAlm_I_N->word9.E5b_HS_2;

		//SVID_3
		gal_alm[2].svID             = gal_wordPartAlm_I_N->word9.svID_3;
		gal_alm[2].af0              = gal_wordPartAlm_I_N->word10.a_f0_3*P2_19;
		gal_alm[2].af1              = gal_wordPartAlm_I_N->word10.a_f1_3*P2_21*P2_17;
		gal_alm[2].e                = gal_wordPartAlm_I_N->word9.e_3*P2_17*2;
		gal_alm[2].IOD_a            = gal_wordPartAlm_I_N->word10.iod_a;
		gal_alm[2].m0               = gal_wordPartAlm_I_N->word10.m0_3*P2_15;
		gal_alm[2].omega_0          = gal_wordPartAlm_I_N->word10.omega0_3*P2_15;
		gal_alm[2].omega_dot        = gal_wordPartAlm_I_N->word10.omega_dot_3*P2_33;
		gal_alm[2].sigma_i          = gal_wordPartAlm_I_N->word9.sigma_i_3*P2_15*2;
		gal_alm[2].sqrt_A           = gal_wordPartAlm_I_N->word9.sqrt_a_3*P2_11*4;
		gal_alm[2].t_0a             = (double)gal_wordPartAlm_I_N->word9.t0_a*600;
		gal_alm[2].w                = gal_wordPartAlm_I_N->word9.w_3*P2_15;
		gal_alm[2].wn_a             = gal_wordPartAlm_I_N->word9.wn_a;
		gal_alm[2].e1b_hs           = gal_wordPartAlm_I_N->word10.E1B_HS_3;
		gal_alm[2].e5a_hs           = 0;
		gal_alm[2].e5b_hs           = gal_wordPartAlm_I_N->word10.E5b_HS_3;

		//clear cache
		memset( gal_wordPartAlm_I_N, 0, sizeof(wordPart_Alm_I_GAL_t) );
	}
	else{
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"Not supported alm model type for GAL!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}

	return TRUE;
}

uint8_t msg_decode_gal_F_N( 
	uint8_t  *msg, 
	int32_t *header, 
	pagePart_Eph_F_GAL_t *gal_pagePartEph_F_N, 
	pagePart_Alm_F_GAL_t *gal_pagePartAlm_F_N, 
	int32_t *flag )
{
	//local params
	uint8_t page_type, page_num, skip_bits;
	uint8_t prn;

	//inputs check
	if( NULL == msg || NULL == header || NULL == gal_pagePartEph_F_N || NULL == gal_pagePartAlm_F_N || NULL == flag ){
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs err!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}

	//*****************
	//page type ���
	page_num  = (uint8_t)header[5];
	skip_bits = 2;  //for 2bits in MSB first of msg is invalid;
	page_type = get_unsigned_8bits( msg, 1+skip_bits, 7+skip_bits );
	if( page_type != page_num ){
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"page type check failed!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}

	prn = (uint8_t)header[1];
	//page type 1
	if( page_type == 1 ){
		gal_pagePartEph_F_N[prn - 1].page1.type_num        = page_type;
		gal_pagePartEph_F_N[prn - 1].page1.svID            = get_unsigned_8bits( msg,  7+skip_bits,   13+skip_bits );
		gal_pagePartEph_F_N[prn - 1].page1.iod_nav         = get_unsigned_16bits( msg, 13+skip_bits,  23+skip_bits );
		gal_pagePartEph_F_N[prn - 1].page1.toc             = get_unsigned_16bits( msg, 23+skip_bits,  37+skip_bits );
		gal_pagePartEph_F_N[prn - 1].page1.a_f0            = get_signed_32bits( msg,   37+skip_bits,  68+skip_bits, 0 );
		gal_pagePartEph_F_N[prn - 1].page1.a_f1            = get_signed_32bits( msg,   68+skip_bits,  89+skip_bits, 0 );
		gal_pagePartEph_F_N[prn - 1].page1.a_f2            = get_signed_8bits( msg,    89+skip_bits,  95+skip_bits, 0 );
		gal_pagePartEph_F_N[prn - 1].page1.sisa            = get_unsigned_8bits( msg,  95+skip_bits,  103+skip_bits );
		gal_pagePartEph_F_N[prn - 1].page1.a_i0            = get_unsigned_16bits( msg, 103+skip_bits, 114+skip_bits );
		gal_pagePartEph_F_N[prn - 1].page1.a_i1            = get_signed_16bits( msg,   114+skip_bits, 125+skip_bits, 0 );
		gal_pagePartEph_F_N[prn - 1].page1.a_i2            = get_signed_16bits( msg,   125+skip_bits, 139+skip_bits, 0 );
		gal_pagePartEph_F_N[prn - 1].page1.region_1        = get_unsigned_8bits( msg,  139+skip_bits, 140+skip_bits );
		gal_pagePartEph_F_N[prn - 1].page1.region_2        = get_unsigned_8bits( msg,  140+skip_bits, 141+skip_bits );
		gal_pagePartEph_F_N[prn - 1].page1.region_3        = get_unsigned_8bits( msg,  141+skip_bits, 142+skip_bits );
		gal_pagePartEph_F_N[prn - 1].page1.region_4        = get_unsigned_8bits( msg,  142+skip_bits, 143+skip_bits );
		gal_pagePartEph_F_N[prn - 1].page1.region_5        = get_unsigned_8bits( msg,  143+skip_bits, 144+skip_bits );
		gal_pagePartEph_F_N[prn - 1].page1.bgd_E1_E5a      = get_signed_16bits( msg,   144+skip_bits, 154+skip_bits, 0 );
		gal_pagePartEph_F_N[prn - 1].page1.E5a_HS          = get_unsigned_8bits( msg,  154+skip_bits, 156+skip_bits );
		gal_pagePartEph_F_N[prn - 1].page1.wn              = get_unsigned_16bits( msg, 156+skip_bits, 168+skip_bits );
		gal_pagePartEph_F_N[prn - 1].page1.tow             = get_unsigned_32bits( msg, 168+skip_bits, 188+skip_bits );
		gal_pagePartEph_F_N[prn - 1].page1.E5a_DVS         = get_unsigned_8bits( msg,  188+skip_bits, 189+skip_bits );

		//svID check
		if( prn != gal_pagePartEph_F_N[prn - 1].page1.svID ){
			memset( &gal_pagePartEph_F_N[prn - 1].page1, 0, sizeof(Page1_F_GAL_t) );
			SYS_LOGGING(OBJ_SD,LOG_ERROR,"svID in page1 dismatch! %s,%d",__FUNCTION__,__LINE__);
			return FALSE;
		}

		//iono model update
		flag[2] = 1;
	}
	//page type 2
	if( page_type == 2 ){
		gal_pagePartEph_F_N[prn - 1].page2.type_num          = page_type;
		gal_pagePartEph_F_N[prn - 1].page2.iod_nav           = get_unsigned_16bits( msg, 7+skip_bits,   17+skip_bits );
		gal_pagePartEph_F_N[prn - 1].page2.m0                = get_signed_32bits( msg,   17+skip_bits,  49+skip_bits, 0 );
		gal_pagePartEph_F_N[prn - 1].page2.omega_dot         = get_signed_32bits( msg,   49+skip_bits,  73+skip_bits, 0 );
		gal_pagePartEph_F_N[prn - 1].page2.e                 = get_unsigned_32bits( msg, 73+skip_bits,  105+skip_bits );
		gal_pagePartEph_F_N[prn - 1].page2.sqrt_a            = get_unsigned_32bits( msg, 105+skip_bits, 137+skip_bits );
		gal_pagePartEph_F_N[prn - 1].page2.omega_0           = get_signed_32bits( msg,   137+skip_bits, 169+skip_bits, 0 );
		gal_pagePartEph_F_N[prn - 1].page2.i_dot             = get_signed_16bits( msg,   169+skip_bits, 183+skip_bits, 0 );
		gal_pagePartEph_F_N[prn - 1].page2.wn                = get_unsigned_16bits( msg, 183+skip_bits, 195+skip_bits );
		gal_pagePartEph_F_N[prn - 1].page2.tow               = get_unsigned_32bits( msg, 195+skip_bits, 215+skip_bits );
	}
	//page type 3
	if( page_type == 3 ){
		gal_pagePartEph_F_N[prn - 1].page3.type_num          = page_type;
		gal_pagePartEph_F_N[prn - 1].page3.iod_nav           = get_unsigned_16bits( msg, 7+skip_bits,   17+skip_bits );
		gal_pagePartEph_F_N[prn - 1].page3.i0                = get_signed_32bits( msg,   17+skip_bits,  49+skip_bits, 0 );
		gal_pagePartEph_F_N[prn - 1].page3.w                 = get_signed_32bits( msg,   49+skip_bits,  81+skip_bits, 0 );
		gal_pagePartEph_F_N[prn - 1].page3.delt_n            = get_signed_16bits( msg,   81+skip_bits,  97+skip_bits, 0 );
		gal_pagePartEph_F_N[prn - 1].page3.cuc               = get_signed_16bits( msg,   97+skip_bits,  113+skip_bits, 0 );
		gal_pagePartEph_F_N[prn - 1].page3.cus               = get_signed_16bits( msg,   113+skip_bits, 129+skip_bits, 0 );
		gal_pagePartEph_F_N[prn - 1].page3.crc               = get_signed_16bits( msg,   129+skip_bits, 145+skip_bits, 0 );
		gal_pagePartEph_F_N[prn - 1].page3.crs               = get_signed_16bits( msg,   145+skip_bits, 161+skip_bits, 0 );
		gal_pagePartEph_F_N[prn - 1].page3.toe               = get_unsigned_16bits( msg, 161+skip_bits, 175+skip_bits );
		gal_pagePartEph_F_N[prn - 1].page3.wn                = get_unsigned_16bits( msg, 175+skip_bits, 187+skip_bits );
		gal_pagePartEph_F_N[prn - 1].page3.tow               = get_unsigned_32bits( msg, 197+skip_bits, 207+skip_bits );
	}
	//page_type 4
	if( page_type == 4 ){
		gal_pagePartEph_F_N[prn - 1].page4.type_num          = page_type;
		gal_pagePartEph_F_N[prn - 1].page4.iod_nav           = get_unsigned_16bits( msg, 7+skip_bits,   17+skip_bits );
		gal_pagePartEph_F_N[prn - 1].page4.cic               = get_signed_16bits( msg,   17+skip_bits,  33+skip_bits, 0 );
		gal_pagePartEph_F_N[prn - 1].page4.cis               = get_signed_16bits( msg,   33+skip_bits,  49+skip_bits, 0 );
		gal_pagePartEph_F_N[prn - 1].page4.a0                = get_signed_32bits( msg,   49+skip_bits,  81+skip_bits, 0 );
		gal_pagePartEph_F_N[prn - 1].page4.a1                = get_signed_32bits( msg,   81+skip_bits,  105+skip_bits, 0 );
		gal_pagePartEph_F_N[prn - 1].page4.delt_tls          = get_signed_8bits( msg,    105+skip_bits, 113+skip_bits, 0 );
		gal_pagePartEph_F_N[prn - 1].page4.t0_t              = get_unsigned_8bits( msg,  113+skip_bits, 121+skip_bits );
		gal_pagePartEph_F_N[prn - 1].page4.wn0_t             = get_unsigned_8bits( msg,  121+skip_bits, 129+skip_bits );
		gal_pagePartEph_F_N[prn - 1].page4.wn_lsf            = get_unsigned_8bits( msg,  129+skip_bits, 137+skip_bits );
		gal_pagePartEph_F_N[prn - 1].page4.dn                = get_unsigned_8bits( msg,  137+skip_bits, 140+skip_bits );
		gal_pagePartEph_F_N[prn - 1].page4.delt_tlsf         = get_signed_8bits( msg,    140+skip_bits, 148+skip_bits, 0 );
		gal_pagePartEph_F_N[prn - 1].page4.t0_g              = get_unsigned_8bits( msg,  148+skip_bits, 156+skip_bits );
		gal_pagePartEph_F_N[prn - 1].page4.a0_g              = get_signed_16bits( msg,   156+skip_bits, 172+skip_bits, 0 );
		gal_pagePartEph_F_N[prn - 1].page4.a1_g              = get_signed_16bits( msg,   172+skip_bits, 184+skip_bits, 0 );
		gal_pagePartEph_F_N[prn - 1].page4.wn0_g             = get_unsigned_8bits( msg,  184+skip_bits, 190+skip_bits );
		gal_pagePartEph_F_N[prn - 1].page4.tow               = get_unsigned_32bits( msg, 190+skip_bits, 210+skip_bits );

		//UTC model update
		flag[3] = 1;
	}
	//page_type 5
	if( page_type == 5 ){
		gal_pagePartAlm_F_N[prn - 1].page5.type_num          = page_type;
		gal_pagePartAlm_F_N[prn - 1].page5.iod_a             = get_unsigned_8bits( msg,  7+skip_bits,   11+skip_bits );
		gal_pagePartAlm_F_N[prn - 1].page5.wn_a              = get_unsigned_8bits( msg,  11+skip_bits,  13+skip_bits );
		gal_pagePartAlm_F_N[prn - 1].page5.t0_a              = get_unsigned_8bits( msg,  13+skip_bits,  23+skip_bits );
		gal_pagePartAlm_F_N[prn - 1].page5.svID_1            = get_unsigned_8bits( msg,  23+skip_bits,  29+skip_bits );
		gal_pagePartAlm_F_N[prn - 1].page5.sqrt_a_1          = get_signed_16bits( msg,   29+skip_bits,  42+skip_bits, 0 );
		gal_pagePartAlm_F_N[prn - 1].page5.e_1               = get_unsigned_16bits( msg, 42+skip_bits,  53+skip_bits );
		gal_pagePartAlm_F_N[prn - 1].page5.w_1               = get_signed_16bits( msg,   53+skip_bits,  69+skip_bits, 0 );
		gal_pagePartAlm_F_N[prn - 1].page5.sigma_i_1         = get_signed_16bits( msg,   69+skip_bits,  80+skip_bits, 0 );
		gal_pagePartAlm_F_N[prn - 1].page5.omega0_1          = get_signed_16bits( msg,   80+skip_bits,  96+skip_bits, 0 );
		gal_pagePartAlm_F_N[prn - 1].page5.omega_dot_1       = get_signed_16bits( msg,   96+skip_bits,  107+skip_bits, 0 );
		gal_pagePartAlm_F_N[prn - 1].page5.m0_1              = get_signed_16bits( msg,   107+skip_bits, 123+skip_bits, 0 );
		gal_pagePartAlm_F_N[prn - 1].page5.a_f0_1            = get_signed_16bits( msg,   123+skip_bits, 139+skip_bits, 0 );
		gal_pagePartAlm_F_N[prn - 1].page5.a_f1_1            = get_signed_16bits( msg,   139+skip_bits, 152+skip_bits, 0 );
		gal_pagePartAlm_F_N[prn - 1].page5.e5a_hs_1          = get_unsigned_8bits( msg,  152+skip_bits, 154+skip_bits );
		gal_pagePartAlm_F_N[prn - 1].page5.svID_2            = get_unsigned_8bits( msg,  154+skip_bits, 160+skip_bits );
		gal_pagePartAlm_F_N[prn - 1].page5.sqrt_a_2          = get_signed_16bits( msg,   160+skip_bits, 173+skip_bits, 0 );
		gal_pagePartAlm_F_N[prn - 1].page5.e_2               = get_unsigned_16bits( msg, 173+skip_bits, 184+skip_bits );
		gal_pagePartAlm_F_N[prn - 1].page5.w_2               = get_signed_16bits( msg,   184+skip_bits, 200+skip_bits, 0 );
		gal_pagePartAlm_F_N[prn - 1].page5.sigma_i_2         = get_signed_16bits( msg,   200+skip_bits, 211+skip_bits, 0 );
		gal_pagePartAlm_F_N[prn - 1].page5.omega0_U4_2       = get_unsigned_8bits( msg,  211+skip_bits, 215+skip_bits );
	}
	//page_type 6
	if( page_type == 6 ){
		gal_pagePartAlm_F_N[prn - 1].page6.type_num          = page_type;
		gal_pagePartAlm_F_N[prn - 1].page6.iod_a             = get_unsigned_8bits( msg,  7+skip_bits,   11+skip_bits );
		gal_pagePartAlm_F_N[prn - 1].page6.omega0_L12_2      = get_unsigned_16bits( msg, 11+skip_bits,  23+skip_bits );
		gal_pagePartAlm_F_N[prn - 1].page6.omega_dot_2       = get_signed_16bits( msg,   23+skip_bits,  34+skip_bits, 0 );
		gal_pagePartAlm_F_N[prn - 1].page6.m0_2              = get_signed_16bits( msg,   34+skip_bits,  50+skip_bits, 0 );
		gal_pagePartAlm_F_N[prn - 1].page6.a_f0_2            = get_signed_16bits( msg,   50+skip_bits,  66+skip_bits, 0 );
		gal_pagePartAlm_F_N[prn - 1].page6.a_f1_2            = get_signed_16bits( msg,   66+skip_bits,  79+skip_bits, 0 );
		gal_pagePartAlm_F_N[prn - 1].page6.e5a_hs_2          = get_unsigned_8bits( msg,  79+skip_bits,  81+skip_bits );
		gal_pagePartAlm_F_N[prn - 1].page6.svID_3            = get_unsigned_8bits( msg,  81+skip_bits,  87+skip_bits );
		gal_pagePartAlm_F_N[prn - 1].page6.sqrt_a_3          = get_signed_16bits( msg,   87+skip_bits,  100+skip_bits, 0 );
		gal_pagePartAlm_F_N[prn - 1].page6.e_3               = get_unsigned_16bits( msg, 100+skip_bits, 111+skip_bits );
		gal_pagePartAlm_F_N[prn - 1].page6.w_3               = get_signed_16bits( msg,   111+skip_bits, 127+skip_bits, 0 );
		gal_pagePartAlm_F_N[prn - 1].page6.sigma_i_3         = get_signed_16bits( msg,   127+skip_bits, 138+skip_bits, 0 );
		gal_pagePartAlm_F_N[prn - 1].page6.omega0_3          = get_signed_16bits( msg,   138+skip_bits, 154+skip_bits, 0 );
		gal_pagePartAlm_F_N[prn - 1].page6.omega_dot_3       = get_signed_16bits( msg,   154+skip_bits, 165+skip_bits, 0 );
		gal_pagePartAlm_F_N[prn - 1].page6.m0_3              = get_signed_16bits( msg,   165+skip_bits, 181+skip_bits, 0 );
		gal_pagePartAlm_F_N[prn - 1].page6.a_f0_3            = get_signed_16bits( msg,   181+skip_bits, 197+skip_bits, 0 );
		gal_pagePartAlm_F_N[prn - 1].page6.a_f1_3            = get_signed_16bits( msg,   197+skip_bits, 210+skip_bits, 0 );
		gal_pagePartAlm_F_N[prn - 1].page6.e5a_hs_3          = get_unsigned_8bits( msg,  210+skip_bits, 212+skip_bits );
	}

	// check eph update
	if( gal_pagePartEph_F_N[prn - 1].page1.type_num == 1 &&
		gal_pagePartEph_F_N[prn - 1].page2.type_num == 2 &&
		gal_pagePartEph_F_N[prn - 1].page3.type_num == 3 &&
		gal_pagePartEph_F_N[prn - 1].page4.type_num == 4 && 
		(gal_pagePartEph_F_N[prn - 1].page1.iod_nav == gal_pagePartEph_F_N[prn - 1].page2.iod_nav) &&
		(gal_pagePartEph_F_N[prn - 1].page1.iod_nav == gal_pagePartEph_F_N[prn - 1].page3.iod_nav) &&
		(gal_pagePartEph_F_N[prn - 1].page1.iod_nav == gal_pagePartEph_F_N[prn - 1].page4.iod_nav) )
	{
		flag[0] = 1;
	}

	//check alm update
	if( gal_pagePartAlm_F_N[prn - 1].page5.type_num == 5 && 
		gal_pagePartAlm_F_N[prn - 1].page6.type_num == 6 &&
		gal_pagePartAlm_F_N[prn - 1].page5.iod_a == gal_pagePartAlm_F_N[prn - 1].page6.iod_a )
	{
		flag[1] = 1;
	}


	return TRUE;
}

//******I/NAV*************
uint8_t msg_pageMelt_gal_I_N( uint8_t *msg,  page_all_I_GAL_t *gal_page_all_I_N, uint8_t databits[AODN_NAV_DATABITS_LEN_MAX], uint8_t *flag )
{
	uint8_t E1B_odd_even, E1B_page_valid; 
	uint8_t data_melt[AODN_NAV_DATABITS_LEN_GAL_I] = "\0";

	//inputs check
	if( NULL == msg  || NULL == gal_page_all_I_N || NULL == databits || NULL == flag ){
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs err!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}
	//output init
	*flag = 0;
	memset( databits, 0, AODN_NAV_DATABITS_LEN_MAX );

	//***********
	//get page info
	E1B_odd_even   = get_unsigned_8bits( msg, 119, 120 );
	E1B_page_valid = get_unsigned_8bits( msg, 120, 121 );

	if( E1B_odd_even == 1 && E1B_page_valid == 0 ){
		gal_page_all_I_N->page_odd.have_data = 1;
		memcpy( gal_page_all_I_N->page_odd.msg, msg, AODN_NAV_DATABITS_LEN_MAX );
	}
	else if( E1B_odd_even == 0 && E1B_page_valid == 0 ){
		gal_page_all_I_N->page_even.have_data = 1;
		memcpy( gal_page_all_I_N->page_even.msg, msg, AODN_NAV_DATABITS_LEN_MAX );
	}

	//check prepared or not
	if( gal_page_all_I_N->page_odd.have_data == 1 && gal_page_all_I_N->page_even.have_data == 1 ){
		//CRC check
		uint8_t      i;
		uint32_t     CRC24Q_A, CRC24Q_B;
		int32_t     len;

		//even part
		for( i = 14; i < 28; i++ ){
			data_melt[i - 14] = (uint8_t)((gal_page_all_I_N->page_even.msg[i] << 6) | (gal_page_all_I_N->page_even.msg[i+1] >> 2));
		}
		data_melt[14] = (uint8_t)( gal_page_all_I_N->page_even.msg[28] << 6 );

		//odd part
		data_melt[14] = (uint8_t)( (data_melt[14]) | ((gal_page_all_I_N->page_odd.msg[14] << 2) & 0x0C) );
		data_melt[14] = (uint8_t)( (data_melt[14]) | ((gal_page_all_I_N->page_odd.msg[15] >> 4) & 0x0F) );

		for( i = 15; i < 27; i++){
			data_melt[i] = (uint8_t)((gal_page_all_I_N->page_odd.msg[i] << 4) | (gal_page_all_I_N->page_odd.msg[i+1] >> 4));
		}
		data_melt[27] = (gal_page_all_I_N->page_odd.msg[27] << 4);

		//CRC24 check
		CRC24Q_A =  get_unsigned_32bits( data_melt, 197, 221 );
		len = 28;
        CRC24Q_B =  rtk_crc24q( data_melt, len );

		if( CRC24Q_A == CRC24Q_B ){
			uint8_t   msg_word[WORD_LEN_I_GAL] = "\0";

		    //get bits
			memcpy( msg_word, &gal_page_all_I_N->page_even.msg[15],14);     //get 14 bits from odd msg
			memcpy( &msg_word[14],&gal_page_all_I_N->page_odd.msg[15], 2);  //get 2 bits from odd msg

			*flag = 1; 
			//clear cache
			memset( gal_page_all_I_N, 0, sizeof(page_all_I_GAL_t) );
		}
	}
	return TRUE;
}

uint8_t msg_decode_gal_I_N( 
	uint8_t  *msg,
	int32_t *header, 
	wordPart_Eph_I_GAL_t  *gal_wordPartEph_I_N, 
	wordPart_Alm_I_GAL_t  *gal_wordPartAlm_I_N, 
	int32_t *flag )
{
	uint8_t prn, word_num,word_type;

	//inputs check
	if( NULL == msg || NULL == header || NULL == gal_wordPartEph_I_N || NULL == gal_wordPartAlm_I_N || NULL == flag ){
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs err!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}

	//****************
	//word type ���
	word_num  = (uint8_t)header[5];
	word_type = get_unsigned_8bits( msg, 1, 7 );
	if( word_type != word_num ){
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"word type check failed!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}

	prn = (uint8_t)header[1];
	//page_type = 1
	if( word_type == 1 ){
		gal_wordPartEph_I_N[prn - 1].word1.type_num     = word_type;
		gal_wordPartEph_I_N[prn - 1].word1.iod_nav      = get_unsigned_16bits( msg, 7,  17 );

		gal_wordPartEph_I_N[prn - 1].word1.toe          = get_unsigned_16bits( msg, 17, 31 );
		gal_wordPartEph_I_N[prn - 1].word1.m0           = get_signed_32bits( msg,   31, 63, 0 );
		gal_wordPartEph_I_N[prn - 1].word1.e            = get_unsigned_32bits( msg, 63, 95 );
		gal_wordPartEph_I_N[prn - 1].word1.sqrt_A       = get_signed_32bits( msg,   95, 127, 0 );
	}
	//page_type = 2
	if( word_type == 2 ){
		gal_wordPartEph_I_N[prn - 1].word2.type_num     = word_type;
		gal_wordPartEph_I_N[prn - 1].word2.iod_nav      = get_unsigned_16bits( msg, 7,   17 );

		gal_wordPartEph_I_N[prn - 1].word2.omega_0      = get_signed_32bits( msg,   17,  49, 0 );
		gal_wordPartEph_I_N[prn - 1].word2.i0           = get_signed_32bits( msg,   49,  71, 0 );
		gal_wordPartEph_I_N[prn - 1].word2.w            = get_signed_32bits( msg,   71,  103, 0 );
		gal_wordPartEph_I_N[prn - 1].word2.i_dot        = get_signed_16bits( msg,   103, 127, 0 );
	}
	//page_type = 3
	if( word_type == 3 ){
		gal_wordPartEph_I_N[prn - 1].word3.type_num     = word_type;
		gal_wordPartEph_I_N[prn - 1].word3.iod_nav      = get_unsigned_16bits( msg, 7,   17 );

		gal_wordPartEph_I_N[prn - 1].word3.omega_dot    = get_signed_32bits( msg,   17,  41, 0 );
		gal_wordPartEph_I_N[prn - 1].word3.delt_n       = get_signed_16bits( msg,   41,  57, 0 );
		gal_wordPartEph_I_N[prn - 1].word3.cuc          = get_signed_16bits( msg,   57,  73, 0 );
		gal_wordPartEph_I_N[prn - 1].word3.cus          = get_signed_16bits( msg,   73,  89, 0 );
		gal_wordPartEph_I_N[prn - 1].word3.crc          = get_signed_16bits( msg,   89,  105, 0 );
		gal_wordPartEph_I_N[prn - 1].word3.crs          = get_signed_16bits( msg,   105, 121, 0 );
		gal_wordPartEph_I_N[prn - 1].word3.SISA_E1_E5b  = get_unsigned_8bits( msg, 121, 129 );
	}
	//page_type = 4
	if( word_type == 4 ){
		gal_wordPartEph_I_N[prn - 1].word4.type_num     = word_type;
		gal_wordPartEph_I_N[prn - 1].word4.iod_nav      = get_unsigned_16bits( msg, 7,   17 );

		gal_wordPartEph_I_N[prn - 1].word4.svID         = get_unsigned_8bits( msg,  17,  23 );
		if( gal_wordPartEph_I_N[prn - 1].word4.svID != prn ){
			memset( &gal_wordPartEph_I_N[prn - 1].word4, 0, sizeof( Word4_I_GAL_t) );
			SYS_LOGGING(OBJ_SD,LOG_ERROR,"svID in word4 dismatch!,%s,%d",__FUNCTION__,__LINE__);
			return FALSE;
		}

		gal_wordPartEph_I_N[prn - 1].word4.cic          = get_signed_16bits( msg,   23,  39, 0 );
		gal_wordPartEph_I_N[prn - 1].word4.cis          = get_signed_16bits( msg,   39,  55, 0 );
		gal_wordPartEph_I_N[prn - 1].word4.toc          = get_unsigned_16bits( msg, 55,  69 );
		gal_wordPartEph_I_N[prn - 1].word4.a_f0         = get_signed_32bits( msg,   69,  100, 0 );
		gal_wordPartEph_I_N[prn - 1].word4.a_f1         = get_signed_32bits( msg,   100, 121, 0 );
		gal_wordPartEph_I_N[prn - 1].word4.a_f2         = get_signed_8bits( msg,    121, 127, 0);
	}
	//page_type = 5
	if( word_type == 5 ){
		gal_wordPartEph_I_N[prn - 1].word5.type_num    = word_type;
		gal_wordPartEph_I_N[prn - 1].word5.a_i0        = get_unsigned_16bits( msg,   7,   18 );
		gal_wordPartEph_I_N[prn - 1].word5.a_i1        = get_signed_16bits( msg,     18,  29, 0 );
		gal_wordPartEph_I_N[prn - 1].word5.a_i2        = get_signed_16bits( msg,     29,  43, 0 );
		gal_wordPartEph_I_N[prn - 1].word5.region_1    = get_unsigned_8bits( msg,    43,  44 );
		gal_wordPartEph_I_N[prn - 1].word5.region_2    = get_unsigned_8bits( msg,    44,  45 );
		gal_wordPartEph_I_N[prn - 1].word5.region_3    = get_unsigned_8bits( msg,    45,  46 );
		gal_wordPartEph_I_N[prn - 1].word5.region_4    = get_unsigned_8bits( msg,    46,  47 );
		gal_wordPartEph_I_N[prn - 1].word5.region_5    = get_unsigned_8bits( msg,    47,  48 );
		gal_wordPartEph_I_N[prn - 1].word5.bgd_E1_E5a  = get_signed_16bits( msg,     48,  58, 0 );
		gal_wordPartEph_I_N[prn - 1].word5.bgd_E1_E5b  = get_signed_16bits( msg,     58,  68, 0 );
		gal_wordPartEph_I_N[prn - 1].word5.E5b_HS      = get_unsigned_8bits( msg,    68,  70 );
		gal_wordPartEph_I_N[prn - 1].word5.E1B_HS      = get_unsigned_8bits( msg,    70,  72 );
		gal_wordPartEph_I_N[prn - 1].word5.E5b_DVS     = get_unsigned_8bits( msg,    72,  73 );
		gal_wordPartEph_I_N[prn - 1].word5.E1B_DVS     = get_unsigned_8bits( msg,    73,  74 );
		gal_wordPartEph_I_N[prn - 1].word5.wn          = get_unsigned_16bits( msg,   74,  86 );
		gal_wordPartEph_I_N[prn - 1].word5.tow         = get_unsigned_32bits( msg,   86,  106 );

		//IONO update check
		flag[2] = 1;
	}
	//page_type = 6
	if( word_type == 6 ){
		gal_wordPartEph_I_N[prn - 1].word6.type_num    = word_type;
		gal_wordPartEph_I_N[prn - 1].word6.a0          = get_signed_32bits( msg,   7,   39, 0 );
		gal_wordPartEph_I_N[prn - 1].word6.a1          = get_signed_32bits( msg,   39,  63, 0 );
		gal_wordPartEph_I_N[prn - 1].word6.delt_tls    = get_signed_8bits( msg,    63,  71, 0 );
		gal_wordPartEph_I_N[prn - 1].word6.t0_t        = get_unsigned_8bits( msg,  71,  79 );
		gal_wordPartEph_I_N[prn - 1].word6.wn0_t       = get_unsigned_8bits( msg,  79,  87 );
		gal_wordPartEph_I_N[prn - 1].word6.wn_lsf      = get_unsigned_8bits( msg,  87,  95 );
		gal_wordPartEph_I_N[prn - 1].word6.dn          = get_unsigned_8bits( msg,  95,  98 );
		gal_wordPartEph_I_N[prn - 1].word6.delt_tlsf   = get_signed_8bits( msg,    98,  106, 0);
		gal_wordPartEph_I_N[prn - 1].word6.tow         = get_unsigned_32bits( msg, 106, 126 );

		//UTC model update check
		flag[3] = 1;
	}
	//page_type = 7
	if( word_type == 7 ){
		gal_wordPartAlm_I_N[prn - 1].word7.type_num    = word_type;
		gal_wordPartAlm_I_N[prn - 1].word7.iod_a       = get_unsigned_8bits( msg,  7,  11 );
		gal_wordPartAlm_I_N[prn - 1].word7.wn_a        = get_unsigned_8bits( msg,  11, 13 );
		gal_wordPartAlm_I_N[prn - 1].word7.t0_a        = get_unsigned_16bits( msg, 13, 23 );
		gal_wordPartAlm_I_N[prn - 1].word7.svID_1      = get_unsigned_8bits( msg,  23, 29 );
		gal_wordPartAlm_I_N[prn - 1].word7.sqrt_a_1    = get_unsigned_16bits( msg, 29, 42 );
		gal_wordPartAlm_I_N[prn - 1].word7.e_1         = get_unsigned_16bits( msg, 42, 53 );
		gal_wordPartAlm_I_N[prn - 1].word7.w_1         = get_signed_16bits( msg,   53, 69, 0 );
		gal_wordPartAlm_I_N[prn - 1].word7.sigma_i_1   = get_signed_16bits( msg,   69, 80, 0 );
		gal_wordPartAlm_I_N[prn - 1].word7.omega0_1    = get_signed_16bits( msg,   80, 96, 0 );
		gal_wordPartAlm_I_N[prn - 1].word7.omega_dot_1 = get_signed_16bits( msg,   96, 107, 0 );
		gal_wordPartAlm_I_N[prn - 1].word7.m0_1        = get_signed_16bits( msg,   107, 123, 0);
	}
	//page_type = 8
	if( word_type == 8 ){
		gal_wordPartAlm_I_N[prn - 1].word8.type_num    = word_type;
		gal_wordPartAlm_I_N[prn - 1].word8.iod_a       = get_unsigned_8bits( msg,  7,  11 );
		gal_wordPartAlm_I_N[prn - 1].word8.a_f0_1      = get_signed_16bits( msg,   11, 27, 0 );
		gal_wordPartAlm_I_N[prn - 1].word8.a_f1_1      = get_signed_16bits( msg,   27, 40, 0 );
		gal_wordPartAlm_I_N[prn - 1].word8.E5b_HS_1    = get_unsigned_8bits( msg,  40, 42 );
		gal_wordPartAlm_I_N[prn - 1].word8.E1B_HS_1    = get_unsigned_8bits( msg,  42, 44 );
		gal_wordPartAlm_I_N[prn - 1].word8.svID_2      = get_unsigned_8bits( msg,  44, 50 );
		gal_wordPartAlm_I_N[prn - 1].word8.sqrt_a_2    = get_unsigned_16bits( msg, 50, 63 );
		gal_wordPartAlm_I_N[prn - 1].word8.e_2         = get_unsigned_16bits( msg, 63, 74 );
		gal_wordPartAlm_I_N[prn - 1].word8.w_2         = get_signed_16bits( msg,   74, 90, 0 );
		gal_wordPartAlm_I_N[prn - 1].word8.sigma_i_2   = get_signed_16bits( msg,   90, 101, 0 );
		gal_wordPartAlm_I_N[prn - 1].word8.omega0_2    = get_signed_16bits( msg,   101, 117, 0 );
		gal_wordPartAlm_I_N[prn - 1].word8.omega_dot_2 = get_signed_16bits( msg,   117, 128, 0 );
	}
	//page_type = 9
	if( word_type == 9 ){
		gal_wordPartAlm_I_N[prn - 1].word9.type_num    = word_type;
		gal_wordPartAlm_I_N[prn - 1].word9.iod_a       = get_unsigned_8bits( msg,  7,  11 );
		gal_wordPartAlm_I_N[prn - 1].word9.wn_a        = get_unsigned_8bits( msg,  11, 13 );
		gal_wordPartAlm_I_N[prn - 1].word9.t0_a        = get_unsigned_16bits( msg, 13, 23 );

		gal_wordPartAlm_I_N[prn - 1].word9.m0_2        = get_signed_16bits( msg,   23, 39, 0);
		gal_wordPartAlm_I_N[prn - 1].word9.a_f0_2      = get_signed_16bits( msg,   39, 55, 0 );
		gal_wordPartAlm_I_N[prn - 1].word9.a_f1_2      = get_signed_16bits( msg,   55, 68, 0 );
		gal_wordPartAlm_I_N[prn - 1].word9.E5b_HS_2    = get_unsigned_8bits( msg,  68, 70 );
		gal_wordPartAlm_I_N[prn - 1].word9.E1B_HS_2    = get_unsigned_8bits( msg,  70, 72 );
		gal_wordPartAlm_I_N[prn - 1].word9.svID_3      = get_unsigned_8bits( msg,  72, 78 );
		gal_wordPartAlm_I_N[prn - 1].word9.sqrt_a_3    = get_unsigned_16bits( msg, 78, 91 );
		gal_wordPartAlm_I_N[prn - 1].word9.e_3         = get_unsigned_16bits( msg, 91, 102 );
		gal_wordPartAlm_I_N[prn - 1].word9.w_3         = get_signed_16bits( msg,   102, 118, 0 );
		gal_wordPartAlm_I_N[prn - 1].word9.sigma_i_3   = get_signed_16bits( msg,   118, 129, 0 );
	}
	//page_type = 10
	if( word_type == 10 ){
		gal_wordPartAlm_I_N[prn - 1].word10.type_num    = word_type;
		gal_wordPartAlm_I_N[prn - 1].word10.iod_a       = get_unsigned_8bits( msg,  7,   11 );
		gal_wordPartAlm_I_N[prn - 1].word10.omega0_3    = get_signed_16bits( msg,   11,  27, 0 );
		gal_wordPartAlm_I_N[prn - 1].word10.omega_dot_3 = get_signed_16bits( msg,   27,  38, 0 );
		gal_wordPartAlm_I_N[prn - 1].word10.m0_3        = get_signed_16bits( msg,   38,  54, 0);
		gal_wordPartAlm_I_N[prn - 1].word10.a_f0_3      = get_signed_16bits( msg,   54,  70, 0 );
		gal_wordPartAlm_I_N[prn - 1].word10.a_f1_3      = get_signed_16bits( msg,   70,  83, 0 );
		gal_wordPartAlm_I_N[prn - 1].word10.E5b_HS_3    = get_unsigned_8bits( msg,  83,  85 );
		gal_wordPartAlm_I_N[prn - 1].word10.E1B_HS_3    = get_unsigned_8bits( msg,  85,  87 );
		gal_wordPartAlm_I_N[prn - 1].word10.a0_g        = get_signed_16bits( msg,   87,  103, 0 );
		gal_wordPartAlm_I_N[prn - 1].word10.a1_g        = get_signed_16bits( msg,   103, 115, 0 );
		gal_wordPartAlm_I_N[prn - 1].word10.t0_g        = get_unsigned_8bits( msg,  115, 123 );
		gal_wordPartAlm_I_N[prn - 1].word10.wn0_g       = get_unsigned_8bits( msg,  123, 129 );
	}

	//check eph update
	if( gal_wordPartEph_I_N[prn - 1].word1.type_num == 1 &&
		gal_wordPartEph_I_N[prn - 1].word2.type_num == 2 &&
		gal_wordPartEph_I_N[prn - 1].word3.type_num == 3 &&
		gal_wordPartEph_I_N[prn - 1].word4.type_num == 4 &&
		gal_wordPartEph_I_N[prn - 1].word5.type_num == 5 &&
		(gal_wordPartEph_I_N[prn - 1].word1.iod_nav == gal_wordPartEph_I_N[prn - 1].word2.iod_nav) &&
		(gal_wordPartEph_I_N[prn - 1].word1.iod_nav == gal_wordPartEph_I_N[prn - 1].word3.iod_nav) &&
		(gal_wordPartEph_I_N[prn - 1].word1.iod_nav == gal_wordPartEph_I_N[prn - 1].word4.iod_nav) )
	{
		flag[0] = 1;
	}

	//check alm update
	if( gal_wordPartAlm_I_N[prn - 1].word7.type_num == 7 &&
		gal_wordPartAlm_I_N[prn - 1].word8.type_num == 8 &&
		gal_wordPartAlm_I_N[prn - 1].word9.type_num == 9 &&
		gal_wordPartAlm_I_N[prn - 1].word10.type_num == 10 &&
		(gal_wordPartAlm_I_N[prn - 1].word7.iod_a == gal_wordPartAlm_I_N[prn - 1].word8.iod_a) &&
		(gal_wordPartAlm_I_N[prn - 1].word7.iod_a == gal_wordPartAlm_I_N[prn - 1].word9.iod_a) &&
		(gal_wordPartAlm_I_N[prn - 1].word7.iod_a == gal_wordPartAlm_I_N[prn - 1].word10.iod_a) &&
		(gal_wordPartAlm_I_N[prn - 1].word7.t0_a == gal_wordPartAlm_I_N[prn - 1].word9.t0_a) &&
		(gal_wordPartAlm_I_N[prn - 1].word7.wn_a == gal_wordPartAlm_I_N[prn - 1].word9.wn_a) )
	{
		flag[1] = 1;
	}
	return TRUE;
}

//********************DECODE_DATA_BITS********************
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//for offline agnss files decoding
void msg_addEph_offline(uint8_t gnss_mode, void*eph){
	
	if(gnss_mode == GPS_MODE){
		GPS_EPH_INFO *gps_eph;
		gps_eph = (GPS_EPH_INFO *)eph;

		if( GPS_EPH_NUM == 0 ){
			if ( (GPS_EPH[GPS_EPH_NUM] = Sys_Malloc(sizeof(GPS_EPH_INFO))) == NULL ){
				SYS_LOGGING(OBJ_SD,LOG_ERROR,"Memory Alloc Fail %s %d",__FUNCTION__,__LINE__);
				return ;
			}

			memcpy( GPS_EPH[GPS_EPH_NUM],gps_eph, sizeof(GPS_EPH_INFO));
			//GPS_EPH[GPS_EPH_NUM] = *gps_eph;
			GPS_EPH_NUM = GPS_EPH_NUM + 1;
			SYS_LOGGING(OBJ_SD,LOG_INFO,"gps_eph updated, prn=%d, week=%d, toe=%f ",
				gps_eph->eph_status, gps_eph->subframe1.weeknum+1024, gps_eph->t_oe );
		}
		else{
			uint32_t i;
			uint8_t  flag = 0 ;

			for( i = 0; i < GPS_EPH_NUM; i++ ){
				GPS_EPH_INFO *eph_tmp;

				eph_tmp = (GPS_EPH_INFO *)GPS_EPH[i];
				if( (eph_tmp->eph_status == gps_eph->eph_status) &&
					(eph_tmp->subframe1.weeknum == gps_eph->subframe1.weeknum) &&
					(eph_tmp->t_oe == gps_eph->t_oe))
				{
					flag = 1;
				}
			}

			if( flag == 0 )
			{
				if ( (GPS_EPH[GPS_EPH_NUM] = Sys_Malloc(sizeof(GPS_EPH_INFO))) == NULL ){
					SYS_LOGGING(OBJ_SD,LOG_ERROR,"Memory Alloc Fail %s %d",__FUNCTION__,__LINE__);
					return ;
				}

				memcpy( GPS_EPH[GPS_EPH_NUM],gps_eph, sizeof(GPS_EPH_INFO));
				GPS_EPH_NUM = GPS_EPH_NUM + 1;
				SYS_LOGGING(OBJ_SD,LOG_INFO,"gps_eph updated, prn=%d, week=%d, toe=%f ",
					gps_eph->eph_status, gps_eph->subframe1.weeknum+1024, gps_eph->t_oe );
			}
		}
	}
	else if( gnss_mode == GLN_MODE ){
		GLN_EPH_INFO *glo_eph;
		glo_eph = (GLN_EPH_INFO *)eph;

		//add a eph
		if( GLO_EPH_NUM == 0 ){
			if ( (GLO_EPH[GLO_EPH_NUM] = Sys_Malloc(sizeof(GLN_EPH_INFO))) == NULL ){
				SYS_LOGGING(OBJ_SD,LOG_ERROR,"Memory Alloc Fail %s %d",__FUNCTION__,__LINE__);
				return ;
			}

			memcpy( GLO_EPH[GLO_EPH_NUM],glo_eph, sizeof(GLN_EPH_INFO));
			GLO_EPH_NUM = GLO_EPH_NUM + 1;
			SYS_LOGGING(OBJ_SD,LOG_INFO,"glo_Eph updated, prn=%d, N4=%d, NT=%d, tb=%f ", 
				glo_eph->n, glo_eph->N4, glo_eph->NT ,glo_eph->tb);
		}
		else{
			uint32_t i;
			uint8_t  flag = 0 ;

			for( i = 0; i < GLO_EPH_NUM; i++ ){
				GLN_EPH_INFO *eph_tmp;

				eph_tmp = (GLN_EPH_INFO *)GLO_EPH[i];

				if( (eph_tmp->n  == glo_eph->n ) &&
					(eph_tmp->tb == glo_eph->tb) &&
					(eph_tmp->NT == glo_eph->NT) )
				{
					//check data
					if( fabs(eph_tmp->r[0] - glo_eph->r[0]) > 1.0 ){
						SYS_LOGGING(OBJ_SD,LOG_ERROR,"glo's prn and data dismatch, %s %d",__FUNCTION__,__LINE__);
					}

					flag = 1;
				}
			}

			if( flag == 0 )
			{
				if ( (GLO_EPH[GLO_EPH_NUM] = Sys_Malloc(sizeof(GLN_EPH_INFO))) == NULL ){
					SYS_LOGGING(OBJ_SD,LOG_ERROR,"Memory Alloc Fail %s %d",__FUNCTION__,__LINE__);
					return ;
				}

				memcpy( GLO_EPH[GLO_EPH_NUM],glo_eph, sizeof(GLN_EPH_INFO));
				GLO_EPH_NUM = GLO_EPH_NUM + 1;
				SYS_LOGGING(OBJ_SD,LOG_INFO,"glo_Eph updated, prn=%d, N4=%d, NT=%d, tb=%f ", 
					glo_eph->n, glo_eph->N4, glo_eph->NT ,glo_eph->tb);
			}
		}
	}
	else{
		printf("not supported sys,%s %d\n",__FUNCTION__,__LINE__);
	}
}

void ex_data_agnss_gpsEph( uint8_t prn, GpsNaviModel_t gps_eph, GPS_EPH_INFO *gps_eph_t ){
	uint8_t ura;


	gps_eph_t->eph_status   = prn;
	gps_eph_t->eph_source   = EPH_SOURCE_AGNSS;
	gps_eph_t->IODE         = gps_eph.IODE;              /* As in ICD-200  */
	gps_eph_t->fit_interval = gps_eph.fit_interval;      /* As in ICD-200 */

	gps_eph_t->C_rs         = (float)(gps_eph.C_rs*P2_05);              /* Meters */
	gps_eph_t->C_rc         = (float)(gps_eph.C_rc*P2_05);              /* Meters  */
	gps_eph_t->C_uc         = (float)(gps_eph.C_uc*P2_29);              /* Radians */
	gps_eph_t->C_us         = (float)(gps_eph.C_us*P2_29);              /* Radians */
	gps_eph_t->C_ic         = (float)(gps_eph.C_ic*P2_29);              /* Radians  */
	gps_eph_t->C_is         = (float)(gps_eph.C_is*P2_29);              /* Radians*/
	gps_eph_t->t_oe         = (float)(gps_eph.t_oe*16);              /* Seconds */
	gps_eph_t->delta_n      = gps_eph.delta_n*P2_43*PI;           /* Radians/sec */
	gps_eph_t->M_0          = gps_eph.M_0*P2_31*PI;               /* Radians  */
	gps_eph_t->e            = gps_eph.e*P2_33;                 /* Dimensionless */
	gps_eph_t->sqrt_A       = gps_eph.sqrt_A*P2_19;            /* Meters**-1/2  */
	gps_eph_t->OMEGA_0      = gps_eph.OMEGA_0*P2_31*PI;           /* Radians   */
	gps_eph_t->i_0          = gps_eph.i_0*P2_31*PI;               /* Radians */
	gps_eph_t->omega        = gps_eph.omega*P2_31*PI;             /* Radians */
	gps_eph_t->OMEGADOT     = gps_eph.OMEGADOT*P2_43*PI;          /* Radians */
	gps_eph_t->IDOT         = gps_eph.IDOT*P2_43*PI;              /* Radians */
	/* ---The following items are derived from broadcast eph block---   */
	gps_eph_t->CDTR         = 0;               /* Meters */
	gps_eph_t->Axe          = 0;               /* Meters*/
	gps_eph_t->Axis         = 0;               /* Meters*/
	gps_eph_t->n            = 0;               /* radians/sec */
	gps_eph_t->r1me2        = 0;               /* Dimensionless*/
	gps_eph_t->OMEGA_n      = 0;               /* Radians*/
	gps_eph_t->ODOT_n       = 0;               /* Radians */
	gps_eph_t->sinEk        = 0;

	gps_eph_t->subframe1.weeknum    = gps_eph.weeknum%1024;        /* GPS week number of applicability*/
	gps_eph_t->subframe1.codeL2     = gps_eph.codeL2;         /* Code on L2 flag */
	gps_eph_t->subframe1.L2Pdata    = gps_eph.L2Pdata;        /* L2-P data flag */
	gps_eph_t->subframe1.udre       = gps_eph.URA;            /* SF1 raw accuracy factor */
	gps_eph_t->subframe1.SV_health  = gps_eph.SV_health;      /* SV health byte */
	gps_eph_t->subframe1.IODC       = gps_eph.IODC;           /* IODC -- 10 LSBs*/
	gps_eph_t->subframe1.T_GD       = (float)(gps_eph.T_GD*P2_31);           /* Group delay time factor; seconds */
	gps_eph_t->subframe1.t_oc       = (float)(gps_eph.t_oc*16);           /* Time of this block's applicability;*/
	/* GPS time of week; seconds                */
	gps_eph_t->subframe1.a_f2       = (float)(gps_eph.a_f2*P2_55);           /* SV clock coef2; sec/sec^2*/
	gps_eph_t->subframe1.a_f1       = (float)(gps_eph.a_f1*P2_43);           /* SV clock coef1; sec/sec */
	gps_eph_t->subframe1.a_f0       = (float)(gps_eph.a_f0*P2_31);           /* SV clock coef0; sec*/

	ura = gps_eph.URA;
	if( ura < 6 ) gps_eph_t->subframe1.SVacc  = (float)(pow( 2,(1.0 + ura/2) ));     /* SV accuracy; meters*/
	else          gps_eph_t->subframe1.SVacc  = (float)(pow( 2,(ura - 2) ));       /* SV accuracy; meters*/
}

void ex_data_agnss_gloEph( uint8_t prn, GlonassNaviModel_t gln_eph, GLN_EPH_INFO *glo_eph_t ){

	glo_eph_t->eph_status = 0;              //??    
	glo_eph_t->eph_source = EPH_SOURCE_AGNSS;              /*0-broadcast 1-agps;2-EE */
	glo_eph_t->EphDecFlag = 0;
	glo_eph_t->Health     = 0;


	glo_eph_t->NT         = 0;
	glo_eph_t->NA         = 0;
	glo_eph_t->N4         = 0;

	glo_eph_t->M          = gln_eph.M;                     /*00: GLONASS, 01:GLONASS-M1 */
	glo_eph_t->P1         = gln_eph.P1;                     /*00:0 min;01:30 min; 10:45 min;11:60 min; */
	glo_eph_t->P2         = gln_eph.P2;                 
	glo_eph_t->P3         = 0;
	glo_eph_t->P4         = 0;
	glo_eph_t->P          = 0;
	glo_eph_t->Bn         = gln_eph.Bn;
	glo_eph_t->ln1        = 0;
	glo_eph_t->ln2        = 0;
	glo_eph_t->FT         = gln_eph.FT;
	glo_eph_t->n          = prn;
	glo_eph_t->En         = gln_eph.En;

	/* satellite acceleration */
	glo_eph_t->a[0]       = gln_eph.lsx*P2_30*1000;
	glo_eph_t->a[1]       = gln_eph.lsy*P2_30*1000;
	glo_eph_t->a[2]       = gln_eph.lsz*P2_30*1000;
	/* satellite velocity  */
	glo_eph_t->v[0]       = gln_eph.vx*P2_20*1000; 
	glo_eph_t->v[1]       = gln_eph.vy*P2_20*1000;
	glo_eph_t->v[2]       = gln_eph.vz*P2_20*1000;
	/* satellite position */
	glo_eph_t->r[0]       = gln_eph.x*P2_11*1000;
	glo_eph_t->r[1]       = gln_eph.y*P2_11*1000;
	glo_eph_t->r[2]       = gln_eph.z*P2_11*1000;

	glo_eph_t->tk         = 0;
	glo_eph_t->tb         = gln_eph.gloSec*15*60;  //unit:s

	glo_eph_t->gaman_tb   = gln_eph.gamma*P2_40;
	glo_eph_t->taun_tb    = gln_eph.tauN*P2_30;
	glo_eph_t->delta_taun = gln_eph.deltaTau*P2_30;

	glo_eph_t->tauc       = gln_eph.utc_offset*P2_31;
	glo_eph_t->tauGPS     = gln_eph.tauGPS*P2_30;
}

uint8_t   gnss_sd_agnssDec_logFile(FILE *fid){
	//local paras
	char line[40] = "\0";
	char *tar1 = "Gps";
	char *tar2 = "glo";
	char *pos  = NULL; 
	GlonassNaviModel_t glo_eph;
	GpsNaviModel_t gps_eph;
	GLN_EPH_INFO glo_eph_t;
	GPS_EPH_INFO gps_eph_t;


	//*****************
	if( fid == NULL ){
		SYS_LOGGING(OBJ_SD,LOG_WARNING,"file open failed,%s %d",__FUNCTION__,__LINE__);
		return FALSE;
	}
	
	//read data
	while(!feof(fid)){
		uint8_t prn = 0;
		fgets( line, 40, fid );

		if( strstr(line,tar1 ) != NULL ){
		    if( (pos = strstr( line, ":" )) != NULL ){
				memset( &gps_eph, 0, sizeof(GpsNaviModel_t) );
				prn = atoi( ++pos );
			}
			else{ continue; }

			//last paras
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ gps_eph.codeL2 = atoi( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ gps_eph.URA = atoi( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ gps_eph.SV_health = atoi( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ gps_eph.L2Pdata = atoi( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ gps_eph.fit_interval = atoi( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ gps_eph.IODE = atoi( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ gps_eph.weeknum = atoi( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ gps_eph.IODC = atoi( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ gps_eph.SVacc = (float)atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ gps_eph.T_GD = (float)atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ gps_eph.t_oc = (float)atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ gps_eph.a_f2 = (float)atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ gps_eph.a_f1 = (float)atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ gps_eph.a_f0 = (float)atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ gps_eph.C_uc = (float)atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ gps_eph.C_us = (float)atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ gps_eph.t_oe = (float)atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ gps_eph.C_ic = (float)atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ gps_eph.C_is = (float)atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ gps_eph.C_rc = (float)atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ gps_eph.C_rs = (float)atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ gps_eph.delta_n = atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ gps_eph.M_0 = atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ gps_eph.e = atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ gps_eph.sqrt_A = atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ gps_eph.OMEGA_0 = atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ gps_eph.i_0 = atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ gps_eph.omega = atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ gps_eph.OMEGADOT = atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ gps_eph.IDOT = atof( ++pos );}else{continue;}

			//exchange data
			memset( &gps_eph_t, 0, sizeof(GPS_EPH_INFO) );
			ex_data_agnss_gpsEph( prn, gps_eph, &gps_eph_t );

			//add a eph
			msg_addEph_offline( GPS_MODE, (void*)&gps_eph_t );

			if( gnss_Sd_Nm_AddEph( GPS_MODE, prn, &gps_eph_t) == FALSE )
				continue;
		}
		else if( strstr(line,tar2 ) != NULL ){
			if( (pos = strstr( line, ":" )) != NULL ){
				memset( &glo_eph, 0, sizeof(GlonassNaviModel_t) );
				prn = atoi( ++pos );
			}
			else{ continue; }

			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ glo_eph.FT = atoi( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ glo_eph.M = atoi( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ glo_eph.Bn = atoi( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ glo_eph.utc_offset = atoi( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ glo_eph.En = atoi( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ glo_eph.P1 = atoi( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ glo_eph.P2 = atoi( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ glo_eph.deltaTau = atoi( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ glo_eph.gamma = (float)atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ glo_eph.freqno = (float)atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ glo_eph.lsx = (float)atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ glo_eph.lsy = (float)atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ glo_eph.lsz = (float)atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ glo_eph.tauN = atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ glo_eph.gloSec = atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ glo_eph.x = atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ glo_eph.y = atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ glo_eph.z = atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ glo_eph.vx = atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ glo_eph.vy = atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ glo_eph.vz = atof( ++pos );}else{continue;}
			fgets( line, 40, fid ); if( (pos = strstr( line, ":" )) != NULL ){ glo_eph.tauGPS = atof( ++pos );}else{continue;}

			//exchange data
			memset( &glo_eph_t, 0, sizeof(GLN_EPH_INFO) );
			ex_data_agnss_gloEph( prn, glo_eph, &glo_eph_t );

			//add a eph
            msg_addEph_offline( GLN_MODE, (void*)&glo_eph_t );

			if( gnss_Sd_Nm_AddEph( GLN_MODE, prn, &glo_eph_t) == FALSE )
				continue;
		}		       
	}

	fclose(fid);
	return TRUE;
}

void *get_eph_offline( uint8_t GNSS_MODE, uint8_t prn, double tor ){
    //local paras
	int pos = -10;
	double dt0 = 604800;
	double dt;
	int i;

	//*****************
	if( GNSS_MODE == GPS_MODE ){

		if( GPS_EPH_NUM < 1 )
			return NULL;

		for( i = 0; i < (int)GPS_EPH_NUM; i++ ){
			GPS_EPH_INFO *eph;

			eph = (GPS_EPH_INFO *)GPS_EPH[i];
			if( eph->eph_status == prn && eph->subframe1.SV_health == 0 ){
				//cal dt
				dt = tor - eph->t_oe;
				if( dt > 302400 )       dt = dt - 604800;
				else if( dt < -302400 ) dt = dt + 604800;

				if( (abs(dt) < abs(dt0)) && (abs(dt) < 7200) ){
					dt0 = dt;
					pos = i;
				}
			}
		}

		//chcek res
		if( pos >= 0 ){
			return GPS_EPH[pos];
		}
	}
	else if( GNSS_MODE == GLN_MODE ){
		if( GLO_EPH_NUM < 1 )
			return NULL;

		for( i = 0; i < (int)GLO_EPH_NUM; i++ ){
			uint8_t ln,Bn;
			GLN_EPH_INFO *eph;

			eph = (GLN_EPH_INFO *)GLO_EPH[i];
			ln = eph->ln1;
			Bn = eph->Bn & 0x04;
			if( eph->n == prn && ln == 0 && Bn == 0 ){
				//cal dt
				dt = tor - eph->tb;
				if( dt > (double)(SECS_IN_DAY/2) )       dt = dt - SECS_IN_DAY;
				else if( dt < -(double)(SECS_IN_DAY/2) ) dt = dt + SECS_IN_DAY;

				if( (abs(dt) < abs(dt0)) && (abs(dt) < 3600) ){
					dt0 = dt;
					pos = i;
				}
			}
		}

		//chcek res
		if( pos >= 0 ){
			return GLO_EPH[pos];
		}
	}
	else{
		SYS_LOGGING(OBJ_SD,LOG_WARNING,"Not supported sys, %s %d",__FUNCTION__,__LINE__ );
	}

	return NULL;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//for offline rawbits files decoding
uint8_t msg_bits_prepare_logFile( FILE *fid, int32_t *header, uint8_t *dataBits )
{
	//inner vars
	char *flag1 = "size:";
	char *flag2 = "data is:";
	char line[128] = "\0";

	//inputs check
	if( fid == NULL ) return FALSE;

	//init outputs
	dataBits[0] = (uint8_t)('\0');

	//***********************************
	while (!feof(fid)){
		char *token = NULL;
		int32_t cnt = 0;
		int i,j;
		int32_t head[6] = {0};

		//read first line;
		fgets( line, 128, fid );
		if( strstr(line, flag1) == NULL )
			continue;

		//decode the first line
		token = strtok( line, "," );
		while( token != NULL ){
			char *tar = NULL;

			if( (tar = strstr( token, ":" )) != NULL )
				head[cnt] = atoi( ++tar );
				
			token = strtok( NULL, "," );
			//add vars;
			cnt++;
		}

		//read the second line
		fgets( line, 128, fid );
		if( strstr(line, flag2) == NULL )
			continue;

		//decode the first line
		token = strstr( line, ":" );
		if( token == NULL  )
		    continue;

		//move to the first valid bit
		token++;

		//modify data bits by sys
		//first GPS sys
		if( head[1] >= 1 && head[1] <= 32 ){
			//sysc check
			if( token[0] != '2' || token[1] != '2' )
				continue;

			//decode token to uint8_t 
			for( i = 0; i < 10; i++ ){
				uint32_t iWord = 0;
				char *endStr = NULL;
			    char word[8] = "\0";

				memcpy( word, &token[i*8], 8 );
				iWord = (uint32_t)strtol( word, &endStr, 16 );
				iWord = iWord << 2;
				dataBits[i*3]     = (uint8_t)((iWord >> 24) & 0xFF);
				dataBits[i*3 + 1] = (uint8_t)((iWord >> 16) & 0xFF);
				dataBits[i*3 + 2] = (uint8_t)((iWord >> 8 ) & 0xFF);
			}

			memcpy( header, head, sizeof(int32_t)*6 );
			break;
		}
		else if( head[1] >= 65 && head[1] <= 88 ){
			//decode token to uint8_t,include four strings and resort the databits
			for( i = 0; i < 5; i++ ){
				for( j = 0; j < 10; j++){
					char *endStr = NULL;
					char word[2] = "\0";

					memcpy( word, &token[(i+1)*20 - (j+1)*2], 2 );
					dataBits[i*10 + j] = (uint8_t)strtol( word, &endStr, 16 );

				}
			}

			memcpy( header, head, sizeof(int32_t)*6 );
			break;
		}
	}

	return TRUE;
}

uint8_t gnss_sd_msgDec_logFile( FILE *fid ){
	
	//fid2 = fopen("D:\\PHONE_TEST\\1013_1\\res.txt","w"); 

	if( fid == NULL ){
		printf("file open failed\n");
		return FALSE;
	}


	while(!feof(fid)){
		int32_t header[6] = {0};
		uint8_t  dataBits[50] = "\0";
		int32_t prn, subfrID;

		if( msg_bits_prepare_logFile( fid, header, dataBits ) == FALSE ){
			SYS_LOGGING(OBJ_SD,LOG_ERROR,"get msg from target file failed!, %s %d",__FUNCTION__,__LINE__);
			break;
		}

		//check  prn and subframeID
		prn = header[1];
		subfrID = header[5];

		//************GPS_PART*******************
		if( prn >= 1 && prn <= 32 && subfrID >= 1 && subfrID <= 5 ){
			int32_t flag[3] = {0}; //data update flag

			if( msg_decode_gps( dataBits, header, gps_subfrPart123, gps_subfrPart45, flag ) == FALSE )
				continue;

			//check the data update
			//gps eph update
			if( flag[0] == 1 ){
				GPS_EPH_INFO gps_eph;
				//load the valid eph data and clear the cache structure
				memset( &gps_eph, 0, sizeof(GPS_EPH_INFO) );
				if( exchange_gps_eph( &gps_subfrPart123[prn - 1], &gps_eph ) == FALSE )
					continue;

				//text
				//SYS_LOGGING(OBJ_SD,LOG_INFO,"gps_eph updated, prn=%d, week=%d, toe=%f ",prn, gps_eph.subframe1.weeknum+1024, gps_eph.t_oe );

//#ifdef SAVE_DATE
//				save_gps_eph( prn, gps_eph );
//#endif

				gps_eph.eph_status = prn;
				msg_addEph_offline( GPS_MODE, (void*)&gps_eph );

				//upload decoded eph
				if( gnss_Sd_Nm_AddEph( GPS_MODE, prn, &gps_eph) == FALSE )
					continue;

			}

			//gps alm update
			if( flag[1] == 1 ){
				int32_t i;
				GPS_ALM_INFO gps_alm[MAX_GPS_PRN];
				//load the valid alm data and clear the cache structure
				memset( gps_alm, 0, sizeof(GPS_ALM_INFO)*MAX_GPS_PRN );
				if( exchange_gps_alm( &gps_subfrPart45[prn - 1], gps_alm ) == FALSE )
					continue;

				//upload decoded alm one by one
				for( i = 0; i < MAX_GPS_PRN; i++ ){
					if( gps_alm[i].sqrt_A == 0 )
						continue;

					//text
					//SYS_LOGGING(OBJ_SD,LOG_INFO,"gps_alm updated, prn=%d, wna=%d, toa=%f ",i+1,gps_alm[i].wn_oa + 256*7,gps_alm[i].t_oa);
//#ifdef SAVE_DATE
//					save_gps_alm( prn, gps_alm[i] );
//#endif
					//add alm 
					gnss_Sd_Nm_AddAlm( GPS_MODE, (uint8_t)(i+1), &gps_alm[i] );
				}
			}

			//gps iono update
			if( flag[2] == 1 ){
				ION_INFO      gps_iono;
				gpsUtcModel_t gps_utc;

				//load the valid iono data and clear the cache structure
				memset( &gps_iono, 0, sizeof(ION_INFO) );
				memset( &gps_utc, 0, sizeof(gpsUtcModel_t) );
				if( exchange_gps_ionoAndUtc( &gps_subfrPart45[prn - 1], &gps_iono, &gps_utc ) == FALSE )
					continue;

				//text
				//SYS_LOGGING(OBJ_SD,LOG_INFO,"gps_iono updated, prn=%d, a0=%f, b0=%f ", prn,gps_iono.alpha_0,gps_iono.beta_0);

				//upload decoded iono
				gnss_Sd_Nm_AddIono( GPS_MODE, &gps_iono);
			}
		}

		//***********GLONASS_PART****************
		if( prn >= 65 && prn <= 88 ){
			int32_t flag[2] = {0}; //data update flag
			stringPart_Eph_t glo_strPart_eph;
			stringPart_Alm_t glo_strPart_alm;

			header[1] = header[1] - 64;

			//init para
			memset( &glo_strPart_eph, 0, sizeof(stringPart_Eph_t));
			memset( &glo_strPart_alm, 0, sizeof(stringPart_Alm_t));

			prn = prn - 64; //remark prn
			if( msg_decode_glo( dataBits, header, &glo_strPart_eph, &glo_strPart_alm, &glo_timeRef[prn - 1], flag ) == FALSE )
				continue;
		    
			//check the data update
			//glo eph update
			if( flag[0] == 1 ){
			    GLN_EPH_INFO glo_eph;

				memset( &glo_eph, 0, sizeof(GLN_EPH_INFO) );
				if( exchange_glo_eph( glo_strPart_eph, &glo_eph ) == FALSE )
					continue;

				//text
                //SYS_LOGGING(OBJ_SD,LOG_INFO,"glo_Eph updated, prn=%d, N4=%d, NT=%d, tb=%f ", prn, glo_eph.N4, glo_eph.NT ,glo_eph.tb);
//#ifdef SAVE_DATE
//				save_glo_eph( prn, glo_eph );
//#endif

				msg_addEph_offline( GLN_MODE,(void*)&glo_eph );

				//upload decoded glo_eph
				if( gnss_Sd_Nm_AddEph( GLN_MODE, prn, &glo_eph) == FALSE )
					continue;

				//catch N4 and N_A  for GLO_ALM
				glo_timeRef[prn - 1].flagN = 1;
				glo_timeRef[prn - 1].N4 = glo_eph.N4;
				glo_timeRef[prn - 1].N_A = glo_eph.NA;
			}

			//glo alm update
			if( flag[1] > 0 ){
				GLN_ALM_INFO glo_alm;
				uint8_t tar_prn;

				tar_prn = flag[1];
				memset( &glo_alm, 0, sizeof(GLN_ALM_INFO) );
				if( exchange_glo_alm( glo_strPart_alm, glo_timeRef[prn - 1], &glo_alm ) == FALSE )
					continue;

				//text
				//SYS_LOGGING(OBJ_SD,LOG_INFO,"glo_alm updated, prn = %d, NA = %d",tar_prn, glo_alm.N_A );
				//upload alm data
				gnss_Sd_Nm_AddAlm( GLN_MODE, tar_prn, &glo_alm );
			}

		}
	}
	
	fclose(fid);
	return TRUE;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//for online data stream decoding & for SPRD interface use
uint8_t msg_bits_prepare( uint8_t gnssMode, AGGnss_NavigationMessage_t *msg, int32_t *header, uint8_t *dataBits )
{
	//inner vars
	int i,j;
	uint8_t  bits[50] = "\0";

	//inputs check
	if( dataBits == NULL ){
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs err!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}

	//***********************************
	//modify data bits by sys
	//first GPS sys
	if( gnssMode == GPS_MODE ){
		//decode token to uint8_t 
		for( i = 0; i < 10; i++ ){
			uint32_t iWord = 0;
			uint32_t u32a,u32b,u32c,u32d;
			//char *endStr = NULL;
			//char word[8] = "\0";

			//*****for string*********
			//memcpy( word, &msg->data[i*8], 8 );
			//iWord = (uint32_t)strtol( word, &endStr, 16 );
			
			//for binary bits
			//memcpy( &iWord, &msg->data[i*4], 4 );
			u32a = (uint32_t)( msg->data[i*4+0] << 24 );
			u32b = (uint32_t)( msg->data[i*4+1] << 16 );
			u32c = (uint32_t)( msg->data[i*4+2] << 8  );
			u32d = (uint32_t)( msg->data[i*4+3] << 0  );
			iWord = (uint32_t)( u32a | u32b | u32c | u32d );
			//**********

			iWord = iWord << 2;
			bits[i*3]     = (uint8_t)((iWord >> 24) & 0xFF);
			bits[i*3 + 1] = (uint8_t)((iWord >> 16) & 0xFF);
			bits[i*3 + 2] = (uint8_t)((iWord >> 8 ) & 0xFF);
		}
	}
	else if( gnssMode == GLN_MODE ){
		//decode token to uint8_t,include four strings and resort the databits
		for( i = 0; i < 5; i++ ){
			for( j = 0; j < 10; j++){
				//char *endStr = NULL;
				//char word[2] = "\0";

				//*****for string*********
				//memcpy( word, &msg->data[(i+1)*20 - (j+1)*2], 2 );
				//bits[i*10 + j] = (uint8_t)strtol( word, &endStr, 16 );
				//*******

				//for binary bits
				memcpy( &bits[i*10 + j], &msg->data[(i+1)*10 - (j+1)], 1 );
			}
		}
	}
	else if( gnssMode == BDS_MODE ){
		for( i = 0; i < 10; i++ ){
			uint32_t iWord = 0;
			uint32_t u32a,u32b,u32c,u32d;
			uint8_t  temp;
			int cnt;

			//for binary bits
			//memcpy( &iWord, &msg->data[i*4], 4 );
			u32a = (uint32_t)( msg->data[i*4+0] << 24 );
			u32b = (uint32_t)( msg->data[i*4+1] << 16 );
			u32c = (uint32_t)( msg->data[i*4+2] << 8  );
			u32d = (uint32_t)( msg->data[i*4+3] << 0  );
			iWord = (uint32_t)( u32a | u32b | u32c | u32d );
			iWord = iWord << 2;

			if( i == 0 ){
				bits[0] = (uint8_t)((iWord >> 24) & 0xFF);
				bits[1] = (uint8_t)((iWord >> 16) & 0xFF);
				bits[2] = (uint8_t)((iWord >> 8 ) & 0xFF);
				bits[3] = (uint8_t)((iWord) & 0xC0);
			}
			if( i == 1 ){
				temp = (uint8_t)((iWord >> 26) & 0x3F);
				bits[3] = (uint8_t)( bits[3] | temp ); 
				bits[4] = (uint8_t)((iWord >> 18) & 0xFF);
				bits[5] = (uint8_t)((iWord >> 10 ) & 0xFF);
			}
			if( i == 2 || i == 6 ){
				if( i < 4 ) cnt = 5;
				else        cnt = 16;

				bits[cnt + 1 ] = (uint8_t)((iWord >> 24) & 0xFF);
				bits[cnt + 2 ] = (uint8_t)((iWord >> 16) & 0xFF);
				bits[cnt + 3 ] = (uint8_t)((iWord >> 8) & 0xFC);
			}
			if( i == 3 || i == 7 ){
				if( i < 5 ) cnt = 7;
				else        cnt = 18;

				temp = (uint8_t)((iWord >> 30) & 0x03);
				bits[cnt + 1] = (uint8_t)( bits[cnt + 1] | temp ); 
				bits[cnt + 2] = (uint8_t)((iWord >> 22) & 0xFF);
				bits[cnt + 3] = (uint8_t)((iWord >> 14) & 0xFF);
				bits[cnt + 4] = (uint8_t)((iWord >> 6) & 0xF0);
			}
			if( i == 4 || i == 8 ){
				if( i < 6 ) cnt = 10;
				else        cnt = 21;

				temp = (uint8_t)((iWord >> 28) & 0x0F);
				bits[cnt + 1] = (uint8_t)( bits[cnt + 1] | temp ); 
				bits[cnt + 2] = (uint8_t)((iWord >> 20) & 0xFF);
				bits[cnt + 3] = (uint8_t)((iWord >> 12) & 0xFF);
				bits[cnt + 4] = (uint8_t)((iWord >> 4 ) & 0xC0);
			}
			if( i == 5 || i == 9 ){
				if( i < 7 ) cnt = 13;
				else        cnt = 24;

				temp = (uint8_t)((iWord >> 26) & 0x3F);
				bits[cnt + 1] = (uint8_t)( bits[cnt + 1] | temp ); 
				bits[cnt + 2] = (uint8_t)((iWord >> 18) & 0xFF);
				bits[cnt + 3] = (uint8_t)((iWord >> 10) & 0xFF);
			}
		}
	}

	//copy the data
	memcpy( dataBits, bits, 50);

	//fill the header
	header[0] = (int32_t)(msg->size);
	header[1] = (int32_t)(msg->svid);
	header[2] = (int32_t)(msg->type);
	header[3] = (int32_t)(msg->status);
	header[4] = (int32_t)(msg->message_id);
	header[5] = (int32_t)(msg->submessage_id);
	header[6] = (int32_t)(msg->data_length);

	return TRUE;
}

uint8_t gnss_sd_msgDec(const void *rawBits)
{

	uint8_t  i;
	//local paras
	int32_t header[7] = {0};
	uint8_t  dataBits[50] = "\0";
	int32_t prn, subfrID;
	uint16_t sys_type;
	AGGnss_NavigationMessage_t msg;

	//inputs check
	if( rawBits == NULL ){
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs err!,%s %d\n",__FUNCTION__,__LINE__);
		return FALSE;
	}

	//**********************
	//get the current bits
	memcpy( &msg, rawBits, sizeof(AGGnss_NavigationMessage_t) );

	sys_type = msg.type;
	prn = msg.svid;
	subfrID = msg.submessage_id;

	//************GPS_PART*******************
	if( sys_type == AGGNSS_NAVIGATION_MESSAGE_TYPE_GPS_L1CA ){
		int32_t flag[3] = {0}; //data update flag

		//*********************
		//prn check and modify 
		if( prn >= MIN_GPS_PRN &&  prn <= MAX_GPS_PRN ){
			prn = prn;
		}
		else if( prn >= MIN_QZSS_PRN && prn <= MAX_QZSS_PRN ){
			prn = prn - MIN_QZSS_PRN + N_GPS_SVS + 1;  //change qzss prn to beginning with 33
		}
		else{
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"Not supported PRN in GPS,prn=%d, %s %d", prn, __FUNCTION__, __LINE__ );
			return FALSE;
		}
		msg.svid = prn;

		//subframe check
		if( (subfrID < 1) || (subfrID > 5) ){
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"subfrID check failed!,%s,%d\n",__FUNCTION__,__LINE__);
			return FALSE;
		}

		//prepare the bitStream
		if( msg_bits_prepare(GPS_MODE, &msg, header, dataBits) == FALSE )
			return FALSE;

		//decode the bits
		if( msg_decode_gps( dataBits, header, gps_subfrPart123, gps_subfrPart45, flag ) == FALSE )
			return FALSE;

		//check the data update
		//gps eph update
		if( flag[0] == 1 ){
			GPS_EPH_INFO gps_eph;
			//load the valid eph data and clear the cache structure
			memset( &gps_eph, 0, sizeof(GPS_EPH_INFO) );
			if( exchange_gps_eph( &gps_subfrPart123[prn - 1], &gps_eph ) == FALSE )
				return FALSE;

			//text
			SYS_LOGGING(OBJ_SD,LOG_INFO,"GPS_EPH decoded, prn=%d, week=%d, toe=%f", 
				prn, gps_eph.subframe1.weeknum+1024, gps_eph.t_oe );

			//upload decoded eph
			//change QZSS prn beginning with 193
			if( prn >= 33 && prn <= 37 ){
				prn = prn + MIN_QZSS_PRN - (N_GPS_SVS + 1);
			}

			if( gnss_Sd_Nm_AddEph( GPS_MODE, prn, &gps_eph) == FALSE )
				return FALSE;
		}

		//gps alm update
		if( flag[1] == 1 ){
			int32_t i;
			GPS_ALM_INFO gps_alm[MAX_GPS_PRN];
			//load the valid alm data and clear the cache structure
			memset( gps_alm, 0, sizeof(GPS_ALM_INFO)*MAX_GPS_PRN );
			if( exchange_gps_alm( &gps_subfrPart45[prn - 1], gps_alm ) == FALSE )
				return FALSE;

			//upload decoded alm one by one
			for( i = 0; i < MAX_GPS_PRN; i++ ){
				if( gps_alm[i].sqrt_A == 0 )
					continue;

				//text
				SYS_LOGGING(OBJ_SD,LOG_INFO,"GPS_ALM decoded, prn=%d, wna=%d, toa=%f ",
					i+1,gps_alm[i].wn_oa + 256*7,gps_alm[i].t_oa);

				//add alm 
				gnss_Sd_Nm_AddAlm( GPS_MODE, (uint8_t)(i+1), &gps_alm[i] );
			}
		}

		//gps iono update
		if( flag[2] == 1 ){
			ION_INFO       gps_iono;
			gpsUtcModel_t  gps_utc;

			//load the valid iono data and clear the cache structure
			memset( &gps_iono, 0, sizeof(ION_INFO) );
			memset( &gps_utc, 0, sizeof(gpsUtcModel_t) );
			if( exchange_gps_ionoAndUtc( &gps_subfrPart45[prn - 1], &gps_iono, &gps_utc ) == FALSE )
				return FALSE;

			//text
			SYS_LOGGING(OBJ_SD,LOG_INFO,"GPS_ION & UTC decoded, prn=%d, a0=%f, b0=%f ", 
				prn,gps_iono.alpha_0,gps_iono.beta_0);

			//upload decoded iono
			gnss_Sd_Nm_AddIono( GPS_MODE, &gps_iono);
			//upload decoded iono
			gnss_Sd_Nm_AddUtc( GPS_MODE, (void *)&gps_utc );
		}
	}
	else if( sys_type == AGGNSS_NAVIGATION_MESSAGE_TYPE_GLO_L1CA ){    
		//***********GLONASS_PART****************

		int32_t flag[2] = {0}; //data update flag
		stringPart_Eph_t glo_strPart_eph;
		stringPart_Alm_t glo_strPart_alm;

		//prn check
		if( prn < MIN_GLN_PRN || prn > MAX_GLN_PRN ){
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"Not supported in GLONASS, prn = %d, %s %d", prn, __FUNCTION__, __LINE__ );
			return FALSE;
		}

		//prepare the bitStream
		if( msg_bits_prepare( GLN_MODE, &msg, header, dataBits ) == FALSE )
			return FALSE;

		//init para
		memset( &glo_strPart_eph, 0, sizeof(stringPart_Eph_t));
		memset( &glo_strPart_alm, 0, sizeof(stringPart_Alm_t));

		if( msg_decode_glo( dataBits, header, &glo_strPart_eph, &glo_strPart_alm, &glo_timeRef[prn - 1], flag ) == FALSE )
			return FALSE;
		    
		//check the data update
		//glo eph update
		if( flag[0] == 1 ){
			GLN_EPH_INFO glo_eph;

			memset( &glo_eph, 0, sizeof(GLN_EPH_INFO) );
			if( exchange_glo_eph( glo_strPart_eph, &glo_eph ) == FALSE )
				return FALSE;

			//text
			SYS_LOGGING(OBJ_SD,LOG_INFO,"GlN_EPH decoded, prn=%d, N4=%d, NT=%d, tb=%f ", 
				prn, glo_eph.N4, glo_eph.NT ,glo_eph.tb);

			//upload decoded glo_eph
			if( gnss_Sd_Nm_AddEph( GLN_MODE, prn, &glo_eph) == FALSE )
				return FALSE;

			//catch N4 and N_A  for GLO_ALM
			glo_timeRef[prn - 1].flagN = 1;
			glo_timeRef[prn - 1].N4 = glo_eph.N4;
			glo_timeRef[prn - 1].N_A = glo_eph.NA;
		}

		//glo alm update
		if( flag[1] > 0 ){
			GLN_ALM_INFO glo_alm;
			uint8_t tar_prn;

			tar_prn = flag[1];
			memset( &glo_alm, 0, sizeof(GLN_ALM_INFO) );
			if( exchange_glo_alm( glo_strPart_alm, glo_timeRef[prn - 1], &glo_alm ) == FALSE )
				return FALSE;

			//text
			SYS_LOGGING(OBJ_SD,LOG_INFO,"GLN_ALM decoded, prn=%d, NA=%d", tar_prn, glo_alm.N_A);
			//upload alm data
			gnss_Sd_Nm_AddAlm( GLN_MODE, tar_prn, &glo_alm );
		}
	}
	else if( msg.type == AGGNSS_NAVIGATION_MESSAGE_TYPE_BDS_D1  ){
        //***********BDS_D1_PART****************
		int32_t flag[4] = {0}; //data update flag

		//***************
		//check data length
		if( msg.data_length < AODN_NAV_DATABITS_LEN_BDS ){
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"nav databits length check failed(BDS:40)! len=%llu,%s,%d",
				msg.data_length,__FUNCTION__,__LINE__);
			return FALSE;
		}

		//prn check
		if( prn < 6 || prn > 35 ){
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"Not supported PRN in BDS_D1, prn = %d, %s %d", prn, __FUNCTION__, __LINE__ );
			return FALSE;
		}

		//subframe ID check
		if( (msg.submessage_id < 1) || (msg.submessage_id > 5) ){
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"gps subfrID check failed! subfrID=%d,%s,%d",msg.submessage_id,__FUNCTION__,__LINE__);
			return FALSE;
		}

		//SYS_LOGGING(OBJ_SD,LOG_WARNING,"reveive D1 nav bits: prn = %d,sf = %d",msg.svid,msg.submessage_id);
		//prepare the bitStream
		if( msg_bits_prepare(BDS_MODE, &msg, header, dataBits) == FALSE )
			return FALSE;

		//decode the bits
		if( msg_decode_bds_N( dataBits, header, bds_subfrPartEph_D1_N,
			bds_subfrPartAlm_D1_N, bds_subfrPartEph_D2_N, bds_subfrPartAlm_D2_N, flag ) == FALSE )
			return FALSE;

		//check the data update
		//IONO DATA
		if( flag[2] > 0 ){
			ION_INFO bds_iono;

			memset( &bds_iono, 0, sizeof(ION_INFO) );
			if( exchange_bds_iono_N( AGGNSS_NAVIGATION_MESSAGE_TYPE_BDS_D1, 
				&bds_subfrPartEph_D1_N[prn - 6], NULL, &bds_iono ) == FALSE )
				return FALSE;

			//text
			SYS_LOGGING(OBJ_SD,LOG_INFO,"BDS_ION decoded, prn=%d, a0=%f, b0=%f ", 
				prn,bds_iono.alpha_0,bds_iono.beta_0);

			//upload decoded iono
			gnss_Sd_Nm_AddIono( BDS_MODE, &bds_iono);
		}

		//UTC MODEL
		if( flag[3] > 1 ){
			bdsUtcModel_t bds_utc;

			memset( &bds_utc, 0, sizeof(bdsUtcModel_t) );
			if( exchange_bds_utc_N( AGGNSS_NAVIGATION_MESSAGE_TYPE_BDS_D1, 
				&bds_subfrPartAlm_D1_N[prn - 6], NULL, &bds_utc ) == FALSE )
				return FALSE;

			//text
			SYS_LOGGING(OBJ_SD,LOG_INFO,"BDS_UTC decoded, prn=%d ", prn );

			//upload decoded iono
			gnss_Sd_Nm_AddUtc( BDS_MODE, (void *)&bds_utc);
		}

		//EPH DATA
		if( flag[0] > 0 ){
			BDS_EPH_INFO bds_eph;

			memset( &bds_eph, 0, sizeof(BDS_EPH_INFO) );
			if( exchange_bds_eph_N( (uint8_t)prn, &bds_subfrPartEph_D1_N[prn - 6], NULL, &bds_eph ) == FALSE )
				return FALSE;

			//text
			SYS_LOGGING(OBJ_SD,LOG_INFO,"BDS_EPH decoded, prn=%d, wn=%d, toe=%d ", 
				prn, bds_eph.WN, bds_eph.toe);

			//upload decoded glo_eph
			if( gnss_Sd_Nm_AddEph( BDS_MODE, (uint8_t)prn, &bds_eph) == FALSE )
				return FALSE;
		}

		//ALM DATA
		if( flag[1] > 0 ){
			for( i = 0; i < 30; i++ ){
				BDS_ALM_INFO bds_alm;

				memset( &bds_alm, 0, sizeof(BDS_ALM_INFO) );

				//check alm's existence for each sate
				if( i < 24 ){
					if( bds_subfrPartAlm_D1_N[prn - 6].subfr4_PageT24[i].pnum < 1 )
						continue;
				}else{
					if( bds_subfrPartAlm_D1_N[prn - 6].subfr5_PageT6[i - 24].pnum < 1 )
						continue;
				}

				if( exchange_bds_alm_N( AGGNSS_NAVIGATION_MESSAGE_TYPE_BDS_D1, i, 
					&bds_subfrPartAlm_D1_N[prn - 6], NULL, &bds_alm ) == FALSE )
					continue;

				SYS_LOGGING(OBJ_SD,LOG_INFO,"BDS_ALM decoded, prn=%d, wn=%d, toe=%d ", 
					(i+1), bds_alm.WNa, bds_alm.toa);

				//add alm 
				gnss_Sd_Nm_AddAlm( BDS_MODE, (uint8_t)(i+1), &bds_alm );

				//delete cache
				memset( &bds_subfrPartAlm_D1_N[prn - 6].subfr4_PageT24, 0, sizeof( subfr45_page1_D1_BDS_t )*24 );
				memset( &bds_subfrPartAlm_D1_N[prn - 6].subfr5_PageT6, 0, sizeof( subfr45_page1_D1_BDS_t )*6 );
				memset( &bds_subfrPartAlm_D1_N[prn - 6].subfr5_page8, 0, sizeof( subfr5_page8_D1_BDS_t ) );
			}
		}
	}
	else if( msg.type == AGGNSS_NAVIGATION_MESSAGE_TYPE_BDS_D2 ){
		//***********BDS_D2_PART****************
		int32_t flag[4] = {0}; //data update flag

		//******************
		//check data length
		if( msg.data_length < AODN_NAV_DATABITS_LEN_BDS ){
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"nav databits length check failed(BDS:40)! len=%d,%s,%d",
				msg.data_length,__FUNCTION__,__LINE__);
			return FALSE;
		}

		//prn check
		if( prn < 1 || prn > 5 ){
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"Not supported PRN in BDS_D1, prn = %d, %s %d", prn, __FUNCTION__, __LINE__ );
			return FALSE;
		}

		//subframe ID check
		if( (msg.submessage_id < 1) || (msg.submessage_id > 5) ){
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"gps subfrID check failed! subfrID=%d,%s,%d",msg.submessage_id,__FUNCTION__,__LINE__);
			return FALSE;
		}

		//SYS_LOGGING(OBJ_SD,LOG_WARNING,"reveive D2 nav bits: prn = %d,messageId = %d,sf = %d",msg.svid,msg.message_id,msg.submessage_id);

		//prepare the bitStream
		if( msg_bits_prepare(BDS_MODE, &msg, header, dataBits) == FALSE )
			return FALSE;

		//decode the bits
		if( msg_decode_bds_N( dataBits, header, bds_subfrPartEph_D1_N,
			bds_subfrPartAlm_D1_N, bds_subfrPartEph_D2_N, bds_subfrPartAlm_D2_N, flag ) == FALSE )
			return FALSE;

		//check the data update
		//IONO DATA
		if( flag[2] > 0 ){
			ION_INFO bds_iono;

			memset( &bds_iono, 0, sizeof(ION_INFO) );
			if( exchange_bds_iono_N( AGGNSS_NAVIGATION_MESSAGE_TYPE_BDS_D2, 
				NULL, &bds_subfrPartEph_D2_N[prn - 1], &bds_iono ) == FALSE )
				return FALSE;

			//text
			SYS_LOGGING(OBJ_SD,LOG_INFO,"BDS_ION decoded, prn=%d, a0=%f, b0=%f ", 
				prn,bds_iono.alpha_0,bds_iono.beta_0);

			//upload decoded iono
			gnss_Sd_Nm_AddIono( BDS_MODE, &bds_iono);
		}

		//UTC MODEL
		if( flag[3] > 0 ){
			bdsUtcModel_t bds_utc;

			memset( &bds_utc, 0, sizeof(bdsUtcModel_t) );
			if( exchange_bds_utc_N( AGGNSS_NAVIGATION_MESSAGE_TYPE_BDS_D2, 
				NULL, &bds_subfrPartAlm_D2_N[prn - 1], &bds_utc ) == FALSE )
				return FALSE;

			//text
			SYS_LOGGING(OBJ_SD,LOG_INFO,"BDS_UTC decoded, prn=%d ", prn );

			//upload decoded iono
			gnss_Sd_Nm_AddUtc( BDS_MODE, (void *)&bds_utc); 
		}

		//EPH DATA
		if( flag[0] > 0 ){
			BDS_EPH_INFO bds_eph;

			memset( &bds_eph, 0, sizeof(BDS_EPH_INFO) );
			if( exchange_bds_eph_N( (uint8_t)prn, NULL, &bds_subfrPartEph_D2_N[prn - 1], &bds_eph ) == FALSE )
				return FALSE;

			//text
			SYS_LOGGING(OBJ_SD,LOG_INFO,"BDS_EPH decoded, prn=%d, wn=%d, toe=%d ", 
				prn, bds_eph.WN, bds_eph.toe);

			//upload decoded glo_eph
			if( gnss_Sd_Nm_AddEph( BDS_MODE, (uint8_t)prn, &bds_eph) == FALSE )
				return FALSE;
		}

		//ALM DATA
		if( flag[1] > 0 ){
			for( i = 0; i < 30; i++ ){
				BDS_ALM_INFO bds_alm;

				memset( &bds_alm, 0, sizeof(BDS_ALM_INFO) );
				
				//check alm's existence for each sate
			    if( bds_subfrPartAlm_D2_N[prn - 1].pageT30[i].pnum < 1 )
					continue;

				if( exchange_bds_alm_N( AGGNSS_NAVIGATION_MESSAGE_TYPE_BDS_D2, i, 
					NULL, &bds_subfrPartAlm_D2_N[prn - 1], &bds_alm ) == FALSE )
					return FALSE;

				SYS_LOGGING(OBJ_SD,LOG_INFO,"BDS_ALM decoded, prn=%d, wn=%d, toe=%d ", 
					(i+1), bds_alm.WNa, bds_alm.toa);

				//add alm 
				gnss_Sd_Nm_AddAlm( BDS_MODE, (uint8_t)(i+1), &bds_alm );

				//delete cache
				memset( &bds_subfrPartAlm_D2_N[prn - 1].pageT30, 0, sizeof( subfr5_page3760_D2_BDS_t )*30 );
				memset( &bds_subfrPartAlm_D2_N[prn - 1].page36, 0, sizeof( subfr5_page36_D2_BDS_t ) );
			}
		}
	}

	return TRUE;
}

//for online data stream decoding & for Qualcomm interface use
uint8_t msg_bits_prepare_N( uint8_t gnssMode, AGGnss_NavigationMessage_t *msg, int32_t *header, uint8_t *dataBits )
{
	//inner vars
	int32_t i;
	uint8_t  bits[AODN_NAV_DATABITS_LEN_MAX] = "\0";

	//inputs check
	if( dataBits == NULL ){
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs err!,%s,%d",__FUNCTION__,__LINE__);
		return FALSE;
	}

	//***********************************
	//modify data bits by sys
	//first GPS sys
	if( gnssMode == GPS_MODE ){
		for( i = 0; i < 10; i++ ){
			uint32_t iWord = 0;
			uint32_t u32a,u32b,u32c,u32d;
			
			//for binary bits
			//memcpy( &iWord, &msg->data[i*4], 4 );
			u32a = (uint32_t)( msg->data[i*4+0] << 24 );
			u32b = (uint32_t)( msg->data[i*4+1] << 16 );
			u32c = (uint32_t)( msg->data[i*4+2] << 8  );
			u32d = (uint32_t)( msg->data[i*4+3] << 0  );
			iWord = (uint32_t)( u32a | u32b | u32c | u32d );
			iWord = iWord << 2;

			bits[i*3]     = (uint8_t)((iWord >> 24) & 0xFF);
			bits[i*3 + 1] = (uint8_t)((iWord >> 16) & 0xFF);
			bits[i*3 + 2] = (uint8_t)((iWord >> 8 ) & 0xFF);
		}
	}
	else if( gnssMode == GLN_MODE ){
		//decode token to uint8_t, resort the databits
		for( i = 0; i < AODN_NAV_DATABITS_LEN_GLN; i++ ){
			bits[i] = msg->data[i];
		}
	}
	else if( gnssMode == BDS_MODE ){
		for( i = 0; i < 10; i++ ){
			uint32_t iWord = 0;
			uint32_t u32a,u32b,u32c,u32d;
			uint8_t  temp;
			int cnt;

			//for binary bits
			//memcpy( &iWord, &msg->data[i*4], 4 );
			u32a = (uint32_t)( msg->data[i*4+0] << 24 );
			u32b = (uint32_t)( msg->data[i*4+1] << 16 );
			u32c = (uint32_t)( msg->data[i*4+2] << 8  );
			u32d = (uint32_t)( msg->data[i*4+3] << 0  );
			iWord = (uint32_t)( u32a | u32b | u32c | u32d );
			iWord = iWord << 2;

			if( i == 0 ){
				bits[0] = (uint8_t)((iWord >> 24) & 0xFF);
				bits[1] = (uint8_t)((iWord >> 16) & 0xFF);
				bits[2] = (uint8_t)((iWord >> 8 ) & 0xFF);
				bits[3] = (uint8_t)((iWord) & 0xC0);
			}
			if( i == 1 ){
				temp = (uint8_t)((iWord >> 26) & 0x3F);
				bits[3] = (uint8_t)( bits[3] | temp ); 
				bits[4] = (uint8_t)((iWord >> 18) & 0xFF);
				bits[5] = (uint8_t)((iWord >> 10 ) & 0xFF);
			}
			if( i == 2 || i == 6 ){
				if( i < 4 ) cnt = 5;
				else        cnt = 16;

				bits[cnt + 1 ] = (uint8_t)((iWord >> 24) & 0xFF);
				bits[cnt + 2 ] = (uint8_t)((iWord >> 16) & 0xFF);
				bits[cnt + 3 ] = (uint8_t)((iWord >> 8) & 0xFC);
			}
			if( i == 3 || i == 7 ){
				if( i < 5 ) cnt = 7;
				else        cnt = 18;

				temp = (uint8_t)((iWord >> 30) & 0x03);
				bits[cnt + 1] = (uint8_t)( bits[cnt + 1] | temp ); 
				bits[cnt + 2] = (uint8_t)((iWord >> 22) & 0xFF);
				bits[cnt + 3] = (uint8_t)((iWord >> 14) & 0xFF);
				bits[cnt + 4] = (uint8_t)((iWord >> 6) & 0xF0);
			}
			if( i == 4 || i == 8 ){
				if( i < 6 ) cnt = 10;
				else        cnt = 21;

				temp = (uint8_t)((iWord >> 28) & 0x0F);
				bits[cnt + 1] = (uint8_t)( bits[cnt + 1] | temp ); 
				bits[cnt + 2] = (uint8_t)((iWord >> 20) & 0xFF);
				bits[cnt + 3] = (uint8_t)((iWord >> 12) & 0xFF);
				bits[cnt + 4] = (uint8_t)((iWord >> 4 ) & 0xC0);
			}
			if( i == 5 || i == 9 ){
				if( i < 7 ) cnt = 13;
				else        cnt = 24;

				temp = (uint8_t)((iWord >> 26) & 0x3F);
				bits[cnt + 1] = (uint8_t)( bits[cnt + 1] | temp ); 
				bits[cnt + 2] = (uint8_t)((iWord >> 18) & 0xFF);
				bits[cnt + 3] = (uint8_t)((iWord >> 10) & 0xFF);
			}
		}
	}
	else if( gnssMode == GAL_MODE ){
		//decode token to uint8_t, resort the databits
		for( i = 0; i < (int32_t)(msg->data_length); i++ ){
			bits[i] = msg->data[i];
		}
	}

	//copy the data
	memcpy( dataBits, bits, 40);

	//fill the header
	header[0] = (int32_t)(msg->size);
	header[1] = (int32_t)(msg->svid);
	header[2] = (int32_t)(msg->type);
	header[3] = (int32_t)(msg->status);
	header[4] = (int32_t)(msg->message_id);
	header[5] = (int32_t)(msg->submessage_id);
	header[6] = (int32_t)(msg->data_length);

	return TRUE;
}

uint8_t gnss_sd_msgDec_N(const void *rawBits){
	//local paras
	int32_t header[7] = {0};
	uint8_t  dataBits[AODN_NAV_DATABITS_LEN_MAX] = "\0";
	int16_t prn;
	uint8_t  i;
	AGGnss_NavigationMessage_t msg;

	//inputs check
	if( rawBits == NULL ){
		SYS_LOGGING(OBJ_SD,LOG_ERROR,"inputs err!,%s %d\n",__FUNCTION__,__LINE__);
		return FALSE;
	}

	//**********************
	//get the current bits
	memcpy( &msg, rawBits, sizeof(AGGnss_NavigationMessage_t) );

	//check msg status
	if( msg.status == 0 ){
		SYS_LOGGING(OBJ_SD,LOG_ERROR," the status of received nav-bits is 0!,%s %d\n",__FUNCTION__,__LINE__);
		return FALSE;
	}

	//decoding
	prn = msg.svid;
	if( msg.type == AGGNSS_NAVIGATION_MESSAGE_TYPE_GPS_L1CA ){
		//************GPS_PART*******************
		int32_t flag[4] = {0}; //data update flag
		
		//*********************
		//check data length
		if( msg.data_length < AODN_NAV_DATABITS_LEN_GPS ){
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"nav databits length check failed(GPS:40)! len=%d,%s,%d",
				msg.data_length,__FUNCTION__,__LINE__);
			return FALSE;
		}

		//prn check and modify 
		if( prn >= MIN_GPS_PRN &&  prn <= MAX_GPS_PRN ){
			prn = prn;
		}
		else if( prn >= MIN_QZSS_PRN && prn <= MAX_QZSS_PRN ){
			prn = prn - MIN_QZSS_PRN + N_GPS_SVS + 1;  //change qzss prn to beginning with 33
		}
		else{
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"Not supported PRN in GPS,prn=%d, %s %d", prn, __FUNCTION__, __LINE__ );
			return FALSE;
		}
		msg.svid = prn;

		//subframe ID check
		if( (msg.submessage_id < 1) || (msg.submessage_id > 5) ){
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"gps subfrID check failed! subfrID=%d,%s,%d",msg.submessage_id,__FUNCTION__,__LINE__);
			return FALSE;
		}

		//prepare the bitStream
		if( msg_bits_prepare_N(GPS_MODE, &msg, header, dataBits) == FALSE )
			return FALSE;

		//decode the bits
		if( msg_decode_gps( dataBits, header, gps_subfrPart123, gps_subfrPart45, flag ) == FALSE )
			return FALSE;

		//check the data update
		//gps eph update
		if( flag[0] == 1 ){
			GPS_EPH_INFO gps_eph;
			//load the valid eph data and clear the cache structure
			memset( &gps_eph, 0, sizeof(GPS_EPH_INFO) );
			if( exchange_gps_eph( &gps_subfrPart123[prn - 1], &gps_eph ) == FALSE )
				return FALSE;

			//text
			SYS_LOGGING(OBJ_SD,LOG_INFO,"GPS_EPH decoded, prn=%d, week=%d, toe=%f", 
				prn, gps_eph.subframe1.weeknum+1024, gps_eph.t_oe );

			//upload decoded eph
			//change QZSS prn beginning with 193
			if( prn >= 33 && prn <= 37 ){
				prn = prn + MIN_QZSS_PRN - (N_GPS_SVS + 1);
			}

			if( gnss_Sd_Nm_AddEph( GPS_MODE, (uint8_t)prn, &gps_eph) == FALSE )
				return FALSE;
		}

		//gps alm update
		if( flag[1] == 1 ){
			int32_t i;
			GPS_ALM_INFO gps_alm[MAX_GPS_PRN];
			//load the valid alm data and clear the cache structure
			memset( gps_alm, 0, sizeof(GPS_ALM_INFO)*MAX_GPS_PRN );
			if( exchange_gps_alm( &gps_subfrPart45[prn - 1], gps_alm ) == FALSE )
				return FALSE;

			//upload decoded alm one by one
			for( i = 0; i < MAX_GPS_PRN; i++ ){
				if( gps_alm[i].sqrt_A == 0 )
					continue;

				//text
				SYS_LOGGING(OBJ_SD,LOG_INFO,"GPS_ALM decoded, prn=%d, wna=%d, toa=%f ",
					i+1,gps_alm[i].wn_oa + 256*7,gps_alm[i].t_oa);

				//add alm 
				gnss_Sd_Nm_AddAlm( GPS_MODE, (uint8_t)(i+1), &gps_alm[i] );
			}
		}

		//gps iono update
		if( flag[2] == 1 ){
			ION_INFO      gps_iono;
			gpsUtcModel_t gps_utc;

			//load the valid iono data and clear the cache structure
			memset( &gps_iono, 0, sizeof(ION_INFO) );
			memset( &gps_utc, 0, sizeof(gpsUtcModel_t) );
			if( exchange_gps_ionoAndUtc( &gps_subfrPart45[prn - 1], &gps_iono, &gps_utc ) == FALSE )
				return FALSE;

			//text
			SYS_LOGGING(OBJ_SD,LOG_INFO,"GPS_ION AND UTC decoded, prn=%d, a0=%f, b0=%f ", 
				prn,gps_iono.alpha_0,gps_iono.beta_0);

			//upload decoded iono
			gnss_Sd_Nm_AddIono( GPS_MODE, &gps_iono );

			//upload decoded utc
			gnss_Sd_Nm_AddUtc( GPS_MODE, (void *)&gps_utc );
		}
	}
	else if( msg.type == AGGNSS_NAVIGATION_MESSAGE_TYPE_GLO_L1CA ){    
		//***********GLONASS_PART****************
		int32_t flag[3] = {0}; //data update flag

		//************************
		//check data length
		if( msg.data_length < AODN_NAV_DATABITS_LEN_GLN ){
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"nav databits length check failed(GLN:11)! len=%d,%s,%d",
				msg.data_length,__FUNCTION__,__LINE__);
			return FALSE;
		}

		//prn check
		if( prn < MIN_GLN_PRN || prn > MAX_GLN_PRN ){
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"Not supported PRN in GLONASS, prn = %d, %s %d", prn, __FUNCTION__, __LINE__ );
			return FALSE;
		}

		//string num check
		if( (msg.submessage_id < 1) || (msg.submessage_id > 15) || msg.message_id < 1 || msg.message_id > 5 ){
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"glonass stringID or subframeID check failed! msg.message_id=%d, stringID=%d,%s,%d",
				msg.message_id,msg.submessage_id,__FUNCTION__,__LINE__);
			return FALSE;
		}

		//prepare the bitStream
		if( msg_bits_prepare_N( GLN_MODE, &msg, header, dataBits ) == FALSE )
			return FALSE;

		if( msg_decode_glo_N( dataBits, header, glo_strPartEph_N, glo_strPartAlm_N, flag ) == FALSE )
			return FALSE;
		    
		//check the data update
		//glo eph update
		if( flag[0] > 0 ){
			GLN_EPH_INFO glo_eph;

			memset( &glo_eph, 0, sizeof(GLN_EPH_INFO) );
			if( exchange_glo_eph_N( &glo_strPartEph_N[prn - 1], &glo_eph ) == FALSE )
				return FALSE;

			//text
			SYS_LOGGING(OBJ_SD,LOG_INFO,"GlN_EPH decoded, prn=%d, N4=%d, NT=%d, tb=%f ", 
				prn, glo_eph.N4, glo_eph.NT ,glo_eph.tb);

			//upload decoded glo_eph
			if( gnss_Sd_Nm_AddEph( GLN_MODE, (uint8_t)prn, &glo_eph) == FALSE )
				return FALSE;
		}

		//glo alm update
		if( flag[1] > 0 ){
			GLN_ALM_INFO glo_alm;
			uint8_t tar_pos, tar_prn;

			tar_pos = flag[1] - 1; //begin with 0;
			tar_prn = glo_strPartAlm_N[prn - 1].str6[tar_pos].nA;
			memset( &glo_alm, 0, sizeof(GLN_ALM_INFO) );
			if( exchange_glo_alm_N( &glo_strPartAlm_N[prn - 1], tar_pos, &glo_alm ) == FALSE )
				return FALSE;

			//text
			SYS_LOGGING(OBJ_SD,LOG_INFO,"GlN_ALM decoded, prn=%d, NA=%d", tar_prn, glo_alm.N_A);
			//upload alm data
			gnss_Sd_Nm_AddAlm( GLN_MODE, tar_prn, &glo_alm );

			//tauGPS
			if( abs( glo_strPartAlm_N[prn - 1].str5.tauGPS ) < 100 ){
				p_gnssTimeSysParam->tauGPS[GLN_MODE] = glo_strPartAlm_N[prn - 1].str5.tauGPS*P2_30;
				SYS_LOGGING(OBJ_SD,LOG_INFO,"GLN and GPS time diff : prn:%02d %16.12f",prn,p_gnssTimeSysParam->tauGPS[GLN_MODE]);
			}

		}

		//glo utc update
		if( flag[2] > 0 ){
			glnUtcModel_t gln_utc;

			memset( &gln_utc, 0, sizeof(glnUtcModel_t) );
			if( exchange_glo_utcModel_N( &glo_strPartAlm_N[prn - 1], &gln_utc ) == FALSE )
				return FALSE;

			//text
			SYS_LOGGING(OBJ_SD,LOG_INFO,"GlN_UTC decoded, prn=%d ", prn);

			//upload decoded glo_UTC
			gnss_Sd_Nm_AddUtc( GLN_MODE, (void *)&gln_utc );
		}
	}
	else if( msg.type == AGGNSS_NAVIGATION_MESSAGE_TYPE_BDS_D1  ){
        //***********BDS_D1_PART****************
		int32_t flag[4] = {0}; //data update flag

		//***************
		//check data length
		if( msg.data_length < AODN_NAV_DATABITS_LEN_BDS ){
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"nav databits length check failed(BDS:40)! len=%d,%s,%d",
				msg.data_length,__FUNCTION__,__LINE__);
			return FALSE;
		}

		//prn check
		if( prn < 6 || prn > 35 ){
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"Not supported PRN in BDS_D1, prn = %d, %s %d", prn, __FUNCTION__, __LINE__ );
			return FALSE;
		}

		//subframe ID check
		if( (msg.submessage_id < 1) || (msg.submessage_id > 5) ){
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"gps subfrID check failed! subfrID=%d,%s,%d",msg.submessage_id,__FUNCTION__,__LINE__);
			return FALSE;
		}

		//prepare the bitStream
		if( msg_bits_prepare_N(BDS_MODE, &msg, header, dataBits) == FALSE )
			return FALSE;

		//decode the bits
		if( msg_decode_bds_N( dataBits, header, bds_subfrPartEph_D1_N,
			bds_subfrPartAlm_D1_N, bds_subfrPartEph_D2_N, bds_subfrPartAlm_D2_N, flag ) == FALSE )
			return FALSE;

		//check the data update
		//IONO DATA
		if( flag[2] > 0 ){
			ION_INFO bds_iono;

			memset( &bds_iono, 0, sizeof(ION_INFO) );
			if( exchange_bds_iono_N( AGGNSS_NAVIGATION_MESSAGE_TYPE_BDS_D1, 
				&bds_subfrPartEph_D1_N[prn - 6], NULL, &bds_iono ) == FALSE )
				return FALSE;

			//text
			SYS_LOGGING(OBJ_SD,LOG_INFO,"BDS_ION decoded, prn=%d, a0=%f, b0=%f ", 
				prn,bds_iono.alpha_0,bds_iono.beta_0);

			//upload decoded iono
			gnss_Sd_Nm_AddIono( BDS_MODE, &bds_iono);
		}

		//UTC MODEL
		if( flag[3] > 1 ){
			bdsUtcModel_t bds_utc;

			memset( &bds_utc, 0, sizeof(bdsUtcModel_t) );
			if( exchange_bds_utc_N( AGGNSS_NAVIGATION_MESSAGE_TYPE_BDS_D1, 
				&bds_subfrPartAlm_D1_N[prn - 6], NULL, &bds_utc ) == FALSE )
				return FALSE;

			//text
			SYS_LOGGING(OBJ_SD,LOG_INFO,"BDS_UTC decoded, prn=%d ", prn );

			//upload decoded iono
			gnss_Sd_Nm_AddUtc( BDS_MODE, (void *)&bds_utc);
		}

		//EPH DATA
		if( flag[0] > 0 ){
			BDS_EPH_INFO bds_eph;

			memset( &bds_eph, 0, sizeof(BDS_EPH_INFO) );
			if( exchange_bds_eph_N( (uint8_t)prn, &bds_subfrPartEph_D1_N[prn - 6], NULL, &bds_eph ) == FALSE )
				return FALSE;

			//text
			SYS_LOGGING(OBJ_SD,LOG_INFO,"BDS_EPH decoded, prn=%d, wn=%d, toe=%d ", 
				prn, bds_eph.WN, bds_eph.toe);

			//upload decoded glo_eph
			if( gnss_Sd_Nm_AddEph( BDS_MODE, (uint8_t)prn, &bds_eph) == FALSE )
				return FALSE;
		}

		//ALM DATA
		if( flag[1] > 0 ){
			for( i = 0; i < 30; i++ ){
				BDS_ALM_INFO bds_alm;

				memset( &bds_alm, 0, sizeof(BDS_ALM_INFO) );

				//check alm's existence for each sate
				if( i < 24 ){
					if( bds_subfrPartAlm_D1_N[prn - 6].subfr4_PageT24[i].pnum < 1 )
						continue;
				}else{
					if( bds_subfrPartAlm_D1_N[prn - 6].subfr5_PageT6[i - 24].pnum < 1 )
						continue;
				}

				if( exchange_bds_alm_N( AGGNSS_NAVIGATION_MESSAGE_TYPE_BDS_D1, i, 
					&bds_subfrPartAlm_D1_N[prn - 6], NULL, &bds_alm ) == FALSE )
					continue;

				SYS_LOGGING(OBJ_SD,LOG_INFO,"BDS_ALM decoded, prn=%d, wn=%d, toe=%d ", 
					(i+1), bds_alm.WNa, bds_alm.toa);

				//add alm 
				gnss_Sd_Nm_AddAlm( BDS_MODE, (uint8_t)(i+1), &bds_alm );

				//delete cache
				memset( &bds_subfrPartAlm_D1_N[prn - 6].subfr4_PageT24, 0, sizeof( subfr45_page1_D1_BDS_t )*24 );
				memset( &bds_subfrPartAlm_D1_N[prn - 6].subfr5_PageT6, 0, sizeof( subfr45_page1_D1_BDS_t )*6 );
				memset( &bds_subfrPartAlm_D1_N[prn - 6].subfr5_page8, 0, sizeof( subfr5_page8_D1_BDS_t ) );
			}
		}
	}
	else if( msg.type == AGGNSS_NAVIGATION_MESSAGE_TYPE_BDS_D2 ){
		//***********BDS_D2_PART****************
		int32_t flag[4] = {0}; //data update flag

		//******************
		//check data length
		if( msg.data_length < AODN_NAV_DATABITS_LEN_BDS ){
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"nav databits length check failed(BDS:40)! len=%d,%s,%d",
				msg.data_length,__FUNCTION__,__LINE__);
			return FALSE;
		}

		//prn check
		if( prn < 1 || prn > 5 ){
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"Not supported PRN in BDS_D1, prn = %d, %s %d", prn, __FUNCTION__, __LINE__ );
			return FALSE;
		}

		//subframe ID check
		if( (msg.submessage_id < 1) || (msg.submessage_id > 5) ){
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"gps subfrID check failed! subfrID=%d,%s,%d",msg.submessage_id,__FUNCTION__,__LINE__);
			return FALSE;
		}

		//prepare the bitStream
		if( msg_bits_prepare_N(BDS_MODE, &msg, header, dataBits) == FALSE )
			return FALSE;

		//decode the bits
		if( msg_decode_bds_N( dataBits, header, bds_subfrPartEph_D1_N,
			bds_subfrPartAlm_D1_N, bds_subfrPartEph_D2_N, bds_subfrPartAlm_D2_N, flag ) == FALSE )
			return FALSE;

		//check the data update
		//IONO DATA
		if( flag[2] > 0 ){
			ION_INFO bds_iono;

			memset( &bds_iono, 0, sizeof(ION_INFO) );
			if( exchange_bds_iono_N( AGGNSS_NAVIGATION_MESSAGE_TYPE_BDS_D2, 
				NULL, &bds_subfrPartEph_D2_N[prn - 1], &bds_iono ) == FALSE )
				return FALSE;

			//text
			SYS_LOGGING(OBJ_SD,LOG_INFO,"BDS_ION decoded, prn=%d, a0=%f, b0=%f ", 
				prn,bds_iono.alpha_0,bds_iono.beta_0);

			//upload decoded iono
			gnss_Sd_Nm_AddIono( BDS_MODE, &bds_iono);
		}

		//UTC MODEL
		if( flag[3] > 0 ){
			bdsUtcModel_t bds_utc;

			memset( &bds_utc, 0, sizeof(bdsUtcModel_t) );
			if( exchange_bds_utc_N( AGGNSS_NAVIGATION_MESSAGE_TYPE_BDS_D2, 
				NULL, &bds_subfrPartAlm_D2_N[prn - 1], &bds_utc ) == FALSE )
				return FALSE;

			//text
			SYS_LOGGING(OBJ_SD,LOG_INFO,"BDS_UTC decoded, prn=%d ", prn );

			//upload decoded iono
			gnss_Sd_Nm_AddUtc( BDS_MODE, (void *)&bds_utc); 
		}

		//EPH DATA
		if( flag[0] > 0 ){
			BDS_EPH_INFO bds_eph;

			memset( &bds_eph, 0, sizeof(BDS_EPH_INFO) );
			if( exchange_bds_eph_N( (uint8_t)prn, NULL, &bds_subfrPartEph_D2_N[prn - 1], &bds_eph ) == FALSE )
				return FALSE;

			//text
			SYS_LOGGING(OBJ_SD,LOG_INFO,"BDS_EPH decoded, prn=%d, wn=%d, toe=%d ", 
				prn, bds_eph.WN, bds_eph.toe);

			//upload decoded glo_eph
			if( gnss_Sd_Nm_AddEph( BDS_MODE, (uint8_t)prn, &bds_eph) == FALSE )
				return FALSE;
		}

		//ALM DATA
		if( flag[1] > 0 ){
			for( i = 0; i < 30; i++ ){
				BDS_ALM_INFO bds_alm;

				memset( &bds_alm, 0, sizeof(BDS_ALM_INFO) );
				
				//check alm's existence for each sate
			    if( bds_subfrPartAlm_D2_N[prn - 1].pageT30[i].pnum < 1 )
					continue;

				if( exchange_bds_alm_N( AGGNSS_NAVIGATION_MESSAGE_TYPE_BDS_D2, i, 
					NULL, &bds_subfrPartAlm_D2_N[prn - 1], &bds_alm ) == FALSE )
					return FALSE;

				SYS_LOGGING(OBJ_SD,LOG_INFO,"BDS_ALM decoded, prn=%d, wn=%d, toe=%d ", 
					(i+1), bds_alm.WNa, bds_alm.toa);

				//add alm 
				gnss_Sd_Nm_AddAlm( BDS_MODE, (uint8_t)(i+1), &bds_alm );

				//delete cache
				memset( &bds_subfrPartAlm_D2_N[prn - 1].pageT30, 0, sizeof( subfr5_page3760_D2_BDS_t )*30 );
				memset( &bds_subfrPartAlm_D2_N[prn - 1].page36, 0, sizeof( subfr5_page36_D2_BDS_t ) );
			}
		}
	}
	else if( msg.type == AGGNSS_NAVIGATION_MESSAGE_TYPE_GAL_F ){
#if 0	//************GALILEO_PART*******************
		int32_t flag[4]    = {0}; //data update flag
		uint8_t  nav_type   = 0;   //0 = F/NAV; 1 = I/NAV

		//check data length
		if( msg.data_length < AODN_NAV_DATABITS_LEN_GAL_F ){
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"Nav databits length check failed(GAL/F:30)! len=%d,%s,%d",
				msg.data_length,__FUNCTION__,__LINE__);
			return FALSE;
		}

		//prn check and modify 
		if( prn < MIN_GAL_PRN &&  prn > MAX_GAL_PRN ){
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"Not supported PRN in GALILEO,prn=%d, %s %d", prn, __FUNCTION__, __LINE__ );
			return FALSE;
		}

		//subframe ID check
		if( (msg.message_id < 1) || (msg.message_id > 12) ){
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"Galileo subfrID check failed! subfrID=%d,%s,%d",msg.message_id,__FUNCTION__,__LINE__);
			return FALSE;
		}

		//page ID check
		if( (msg.submessage_id < 1) || (msg.submessage_id > 6) ){
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"Galileo pageID check failed! pageID=%d,%s,%d",msg.submessage_id,__FUNCTION__,__LINE__);
			return FALSE;
		}

		//prepare the bitStream
		if( msg_bits_prepare_N(GAL_MODE, &msg, header, dataBits) == FALSE )
			return FALSE;

		//decode the bits
		if( msg_decode_gal_F_N( dataBits, header, gal_pagePartEph_F_N, gal_pagePartAlm_F_N, flag ) == FALSE )
			return FALSE;

		//check the data update
		//IONO DATA
		if( flag[2] > 0 ){
			GAL_IONO_INFO   gal_iono;
			
			memset( &gal_iono, 0, sizeof(GAL_IONO_INFO) );
			if( exchange_gal_iono_N( nav_type, gal_pagePartEph_F_N[prn - 1], gal_wordPartEph_I_N[prn - 1], &gal_iono ) == FALSE )
				return FALSE;

			//upload decoded gal_iono
			gnss_Sd_Nm_AddIono_gal( GAL_MODE, gal_iono);
		}

		//UTC model
		if( flag[3] > 1 ){
			galUtcModel_t  gal_utc;

			memset( &gal_utc, 0, sizeof(galUtcModel_t) );
			if( exchange_gal_utc_N( nav_type, gal_pagePartEph_F_N[prn - 1], gal_wordPartEph_I_N[prn - 1], &gal_utc ) == FALSE )
				return FALSE;

			//upload decoded utc
			gnss_Sd_Nm_AddUtc( GAL_MODE, (void *)&gal_utc );
		}

		//EPH DATA
		if( flag[0] > 0 ){
			GAL_EPH_INFO gal_eph;

			memset( &gal_eph, 0, sizeof(GAL_EPH_INFO) );
			if( exchange_gal_eph_N( nav_type, prn, &gal_pagePartEph_F_N[prn - 1], &gal_wordPartEph_I_N[prn - 1], &gal_eph ) == FALSE )
				return FALSE;

			//text
			SYS_LOGGING(OBJ_SD,LOG_INFO,"GAL_EPH decoded, prn=%d, wn=%d, toe=%d ", 
				prn, gal_eph.WN, gal_eph.toe);

			//upload decoded glo_eph
			if( gnss_Sd_Nm_AddEph( GAL_MODE, (uint8_t)prn, &gal_eph) == FALSE )
				return FALSE;
		}

		//ALM DATA
		if( flag[1] > 1 ){
			GAL_ALM_INFO gal_alm[3];

			memset( gal_alm, 0, sizeof(GAL_ALM_INFO)*3 );
			if( exchange_gal_alm_N( nav_type, &gal_pagePartAlm_F_N[prn - 1], &gal_wordPartAlm_I_N[prn - 1], gal_alm ) == FALSE )
				return FALSE;

			for( i = 0; i < 3; i++ ){
				if( gal_alm[i].svID > 0 )
					gnss_Sd_Nm_AddAlm( GAL_MODE, gal_alm[i].svID, &gal_alm[i] );
			}
		}
#else
		SYS_LOGGING(OBJ_SD,LOG_ERROR," Not supported BDS(NAV/F) sys!,%s %d\n",__FUNCTION__,__LINE__);
		return TRUE;
#endif
	}
	else if( msg.type == AGGNSS_NAVIGATION_MESSAGE_TYPE_GAL_I ){
#if 0
		//************GALILEO_PART*******************
		int32_t flag[4]       = {0}; //data update flag
		uint8_t  flag_pageMelt = 0;
		uint8_t  dataBits_T[AODN_NAV_DATABITS_LEN_MAX] = "\0";
		uint8_t  nav_type   = 1;  //0 = F/NAV; 1 = I/NAV
		//check data length
		if( msg.data_length < AODN_NAV_DATABITS_LEN_GAL_I ){
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"Nav databits length check failed(GAL/I:29)! len=%d,%s,%d",
				msg.data_length,__FUNCTION__,__LINE__);
			return FALSE;
		}

		//prn check and modify 
		if( prn < MIN_GAL_PRN &&  prn > MAX_GAL_PRN ){
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"Not supported PRN in GALILEO,prn=%d, %s %d", prn, __FUNCTION__, __LINE__ );
			return FALSE;
		}

		//subframe ID check
		if( (msg.message_id < 1) || (msg.message_id > 24) ){
			SYS_LOGGING(OBJ_SD,LOG_WARNING,"Galileo subfrID check failed! subfrID=%d,%s,%d",msg.message_id,__FUNCTION__,__LINE__);
			return FALSE;
		}

		//prepare the bitStream
		if( msg_bits_prepare_N(GAL_MODE, &msg, header, dataBits) == FALSE )
			return FALSE;

		//page melt
		flag_pageMelt = 0;
		if( msg_pageMelt_gal_I_N( dataBits, &gal_page_all_I_N[prn - 1],
			dataBits_T, &flag_pageMelt ) == FALSE )
			return FALSE;

		//decode the bits
		if( flag_pageMelt != 1 )
			return TRUE;

		if( msg_decode_gal_I_N( dataBits_T, header, gal_wordPartEph_I_N, 
			gal_wordPartAlm_I_N, flag ) == FALSE )
			return FALSE;

		//check update
		//IONO data
		if( flag[2] > 0 ){
			GAL_IONO_INFO   gal_iono;

			memset( &gal_iono, 0, sizeof(GAL_IONO_INFO) );
			if( exchange_gal_iono_N( nav_type, gal_pagePartEph_F_N[prn - 1], gal_wordPartEph_I_N[prn - 1], &gal_iono ) == FALSE )
				return FALSE;

			//upload decoded gal_iono
			gnss_Sd_Nm_AddIono_gal( GAL_MODE, gal_iono);
		}

		//UTC model
		if( flag[3] > 0 ){
			galUtcModel_t  gal_utc;

			memset( &gal_utc, 0, sizeof(galUtcModel_t) );
			if( exchange_gal_utc_N( nav_type, gal_pagePartEph_F_N[prn - 1],gal_wordPartEph_I_N[prn - 1], &gal_utc ) == FALSE )
				return FALSE;

			//upload decoded utc
			gnss_Sd_Nm_AddUtc( GAL_MODE, (void *)&gal_utc );
		} 

		//EPH data
		if( flag[0] > 0 ){
			GAL_EPH_INFO gal_eph;

			memset( &gal_eph, 0, sizeof(GAL_EPH_INFO) );
			if( exchange_gal_eph_N( nav_type, prn, &gal_pagePartEph_F_N[prn - 1], &gal_wordPartEph_I_N[prn - 1], &gal_eph ) == FALSE )
				return FALSE;

			//text
			SYS_LOGGING(OBJ_SD,LOG_INFO,"GAL_EPH decoded, prn=%d, wn=%d, toe=%d ", 
				prn, gal_eph.WN, gal_eph.toe);

			//upload decoded glo_eph
			if( gnss_Sd_Nm_AddEph( GAL_MODE, (uint8_t)prn, &gal_eph) == FALSE )
				return FALSE;
		}
		
		//ALM DATA
		if( flag[1] > 1 ){
			GAL_ALM_INFO gal_alm[3];

			memset( gal_alm, 0, sizeof(GAL_ALM_INFO)*3 );
			if( exchange_gal_alm_N( nav_type, &gal_pagePartAlm_F_N[prn - 1], &gal_wordPartAlm_I_N[prn - 1], gal_alm ) == FALSE )
				return FALSE;

			for( i = 0; i < 3; i++ ){
				if( gal_alm[i].svID > 0 )
					gnss_Sd_Nm_AddAlm( GAL_MODE, gal_alm[i].svID, &gal_alm[i] );
			}
		}
#else
		SYS_LOGGING(OBJ_SD,LOG_ERROR," Not supported BDS(NAV/I) sys!,%s %d\n",__FUNCTION__,__LINE__);
		return TRUE;
#endif
	}
	else{
		SYS_LOGGING(OBJ_SD,LOG_ERROR," Not supported sys!,%s %d\n",__FUNCTION__,__LINE__);
		return FALSE;
	}

	return TRUE;
}

#endif