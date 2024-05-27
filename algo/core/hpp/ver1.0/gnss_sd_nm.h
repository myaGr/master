#ifndef __GNSS__SD__NM__H__
#define __GNSS__SD__NM__H__

#include "gnss_types.h"
#include "gnss.h"

#include "gnss_def.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RE_GLO                6378136.0        /* radius of earth (m)            ref [2] */
  //#define MU_GPS                3.9860050E14     /* gravitational constant         ref [1] */
  //#define MU_GLO                3.9860044E14     /* gravitational constant         ref [2] */
  //#define MU_GAL                3.986004418E14   /* earth gravitational constant   ref [7] */
  //#define MU_BDS                3.986004418E14   /* earth gravitational constant   ref [9] */
#define J2_GLO                1.0826257E-3     /* 2nd zonal harmonic of geopot   ref [2] */

//#define OMGE_GLO              7.292115E-5      /* earth angular velocity (rad/s) ref [2] */
//#define OMGE_GAL              7.2921151467E-5  /* earth angular velocity (rad/s) ref [7] */
//#define OMGE_CMP              7.292115E-5      /* earth angular velocity (rad/s) ref [9] */
#define GEOROTATE             -0.087266462599716

#define FRM_NULL              0
#define FRM_ALM               1
#define FRM_EPH               2

#define EPH_STATUS_NONE       0
#define EPH_STATUS_INVALID    1
#define EPH_STATUS_VALID      2
#define EPH_STATUS_EXPIRE     3

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
  typedef struct
  {
    int16_t     weeknum;        /* GPS week number of applicability         */
    uint8_t      codeL2;         /* Code on L2 flag                          */
    uint8_t      L2Pdata;        /* L2-P data flag                           */
    uint8_t      udre;           /* SF1 raw accuracy factor                  */
    uint8_t      SV_health;      /* SV health byte                           */
    uint16_t     IODC;           /* IODC -- 10 LSBs                          */
    float     T_GD;           /* Group delay time factor; seconds         */
    float     t_oc;           /* Time of this block's applicability;      */
    /* GPS time of week; seconds                */
    float     a_f2;           /* SV clock coef2; sec/sec^2                */
    float     a_f1;           /* SV clock coef1; sec/sec                  */
    float     a_f0;           /* SV clock coef0; sec                      */
    float     SVacc;          /* SV accuracy; meters                      */

  } Gps_Sf1_Param;
  /*----------------------------------------------------------------------*\
  | Ephemeris info
  |
  | Refer to ICD-200 for the description of each element.
  |
  | The first portion of this structure is the data sent down from the
  | satellite.  The last few elements are values needed in computing
  | satellite position/velocity that are computed based on what is
  | broadcast by the SV but remain constant.
  | They are computed as follows:
  |  dp->r1me2 = sqrt(1.0 - (dp->e | dp->e));
  |  dp->Axis = dp->sqrt_A | dp->sqrt_A;
  |  dp->n = ((WGS84_SQRT_U) / (dp->Axis | dp->sqrt_A)) +
  |          dp->delta_n;
  |  dp->ODOT_n = (double) dp->OMEGADOT - WGS84_OMEGDOTE;
  |  dp->OMEGA_n = dp->OMEGA_0 - (WGS84_OMEGDOTE | dp->t_oe);
  |  dp->Axe = dp->Axis*dp->e;
  |  dp->CDTR = (float)(dp->sqrt_A | dp->e) | (float)(-4.442807633e-10);
  |
  | Also note that when importing ephemeris block into NAV using
  | st_eph_sf1_put(), it's assumed that the whole structure including
  | the derived elements are valid.
  \*----------------------------------------------------------------------*/
  typedef struct
  {
    uint8_t         eph_status;
    uint8_t         eph_source;
    uint8_t         IODE;              /* As in ICD-200                          */
    uint8_t         fit_interval;      /* As in ICD-200                          */

    float        C_rs;              /* Meters                                 */
    float        C_rc;              /* Meters                                 */
    float        C_uc;              /* Radians                                */
    float        C_us;              /* Radians                                */
    float        C_ic;              /* Radians                                */
    float        C_is;              /* Radians                                */
    float        t_oe;              /* Seconds                                */
    double        delta_n;           /* Radians/sec                            */
    double        M_0;               /* Radians                                */
    double        e;                 /* Dimensionless                          */
    double        sqrt_A;            /* Meters**-1/2                           */
    double        OMEGA_0;           /* Radians                                */
    double        i_0;               /* Radians                                */
    double        omega;             /* Radians                                */
    double        OMEGADOT;          /* Radians                                */
    double        IDOT;              /* Radians                                */
    /* ---The following items are derived from broadcast eph block---       */
    float        CDTR;              /* Meters                                 */
    float        Axe;               /* Meters                                 */
    double        Axis;              /* Meters                                 */
    double        n;                 /* radians/sec                            */
    double        r1me2;             /* Dimensionless                          */
    double        OMEGA_n;           /* Radians                                */
    double        ODOT_n;            /* Radians                                */
    double        sinEk;

    Gps_Sf1_Param subframe1;
  } GPS_EPH_INFO;

  typedef struct
  {
    uint8_t       eph_status;
    uint8_t       eph_source;              /*0-broadcast 1-agps;2-EE */
    uint8_t       EphDecFlag;
    uint8_t       Health;


    uint16_t      NT;
    uint16_t      NA;
    uint16_t      N4;

    uint32_t      M : 2;                      /*00: GLONASS, 01:GLONASS-M1 */
    uint32_t      P1 : 2;                     /*00:0 min;01:30 min; 10:45 min;11:60 min; */
    uint32_t      P2 : 1;
    uint32_t      P3 : 1;
    uint32_t      P4 : 1;
    uint32_t      P : 2;
    uint32_t      Bn : 3;
    uint32_t      ln1 : 1;
    uint32_t      ln2 : 1;
    uint32_t      FT : 4;
    uint32_t      n : 5;
    uint32_t      En : 5;

    double      a[3];                      /* satellite acceleration */
    double      v[3];                      /* satellite velocity  */
    double      r[3];                      /* satellite position */
    double      tk;
    double      tb;

    double      gaman_tb;
    double      taun_tb;
    double      delta_taun;

    double      tauc;
    double      tauGPS;
  } GLN_EPH_INFO;

  typedef struct
  {
    uint8_t      prn;                     //svid
    uint8_t      eph_status;              // the status of ephem  (0: invalid    1:valid)
    uint8_t      eph_source;              // BRDC, AGPS, EE
    uint16_t     EphDecFlag;             //flag for EPH decode status.    Eph decode ready(D1:0x7    D2: 0x3ff )

    uint8_t      SatH1;                   //0: healthy    1: unhealthy
    uint8_t      IODC;
    uint8_t      IODE;
    uint8_t      URAI;
    int16_t     WN;
    int32_t     toc;                    // Reference time of clock data parameter set. (17 bit, 2exp+3)
    int32_t     toe;                    // Reference time of ephemeris data set.

    double     af0;                    // Clock correction polynomial coefficient. (24* bit, 2exp-33)
    float     af1;                    // Clock correction polynomial coefficient. (22* bit, 2exp-50)
    float     af2;                    // Clock correction polynomial coefficient. (11* bit, 2exp-66)

    float     TGD1;                   // Totol Group Delay, NOTE: NanoSeconds as unit here
    float     TGD2;                   // Totol Group Delay, NOTE: NanoSeconds as unit here
    float     cuc;                    // Cosine harmonic correction to orbital radius.
    float     cus;                    // Sine harmonic corr to argument of latitude.
    float     cic;                    // Cosine harmonic corr to inclination.
    float     cis;                    // Sine harmonic corr to inclination.
    float     crc;                    // Cosine harmonic correction to orbital radius.
    float     crs;                    // Sine harmonic correction to orbital radius.

    double     delta_n;                // Mean motion delta from computed value. (16* bit, 2exp-43)
    double     M0;                     // Mean anomaly at TOE.
    double     ecc;                    // Eccentricity.
    double     sqrta;                  // Square root of semi-major axis.
    double     OMEGA_0;                // Argument of perigee at TOE.
    double     OMEGA_Dot;              // Rate of right ascension.
    double     idot;                   // Rate of inclination.
    double     i0;
    double     w;

    /* following variables are derived from base EPH paras decode from BRDC */
    double     sinEk;              // stored for calculating relativistic correction

  } BDS_EPH_INFO;

  typedef struct
  {
    uint8_t      prn;                     // svid
    uint8_t      eph_status;              // the status of ephem  (0: invalid    1:valid)
    uint8_t      eph_source;              // BRDC, AGPS, EE

    int16_t     WN;
    uint16_t     IOD;
    uint8_t      svHealth;
    int8_t      E1BDVS;  /* 1bit, Data Validity Status */
    int8_t      E1BSHS;  /* 2bit, Signal Health Status */
    int8_t      E5aDVS;  /* 1bit, Data Validity Status */
    int8_t      E5aSHS;  /* 2bit, Signal Health Status */
    int8_t      E5bDVS;  /* 1bit, Data Validity Status */
    int8_t      E5bSHS;  /* 2bit, Signal Health Status */

    int32_t     toe;
    float     cuc;                    // Cosine harmonic correction to orbital radius.
    float     cus;                    // Sine harmonic corr to argument of latitude.
    float     cic;                    // Cosine harmonic corr to inclination.
    float     cis;                    // Sine harmonic corr to inclination.
    float     crc;                    // Cosine harmonic correction to orbital radius.
    float     crs;                    // Sine harmonic correction to orbital radius.
    double     sqrta;                  // Square root of semi-major axis.
    double     M0;
    double     delta_n;
    double     OMEGA_0;                // Argument of perigee at TOE.
    double     OMEGA_Dot;              // Rate of right ascension.
    double     idot;                   // Rate of inclination.
    double     i0;
    double     w;
    double     ecc;

    int32_t     toc;
    double     af0;
    float     af1;
    float     af2;
    double     bgd_e5a;  //E5a-E1
    double     bgd_e5b;  //E5b-E1
    uint8_t      sisa;
    uint8_t      navtype;  //0:INAV  1:FNAV

    /* ---The following items are derived from broadcast eph block---   */
    float        CDTR;              /* Meters                                 */
    float        Axe;               /* Meters                                 */
    double        Axis;              /* Meters                                 */
    double        n;                 /* radians/sec                            */
    double        r1me2;             /* Dimensionless                          */
    double        OMEGA_n;           /* Radians                                */
    double        ODOT_n;            /* Radians                                */
    double        sinEk;

  } GAL_EPH_INFO;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////

  /*----------------------------------------------------------------------*\
  | Almanac info
  |
  | Refer to ICD-200 for the description of each element.
  |
  | The first portion of this structure is the data sent down from the
  | satellite.  The last few elements are values needed in computing
  | satellite position/velocity that are computed based on what is
  | broadcast by the SV but remain constant.
  | They are computed as follows:
  |  dp->Axis = dp->sqrt_A | dp->sqrt_A;
  |  dp->n = WGS84_SQRT_U / (dp->Axis | dp->sqrt_A);
  |  dp->OMEGA_n = dp->OMEGA_0 - (WGS84_OMEGDOTE | dp->t_oa);
  |  dp->ODOT_n = dp->OMEGADOT - WGS84_OMEGDOTE;
  |  dp->r1me2 = sqrt((float)((float)1.0 - dp->e | dp->e));
  |  dp->Axen = dp->Axis | dp->e* dp->n;
  |
  | Also note that when importing almanac block into NAV using
  | st_alm_put(), it's assumed that the whole structure including
  | the derived elements are valid.
  \*----------------------------------------------------------------------*/
  typedef struct
  {
    uint8_t        alm_status;       /* valid flag: true/false */
    uint8_t        t_oa_raw;         /* Raw time of almanac; LSB 2^12 sec      */
    uint8_t        SV_health;        /* As in ICD-200                          */
    float       e;                /* dimensionless                          */
    float       t_oa;             /* Almanac time of week in seconds        */
    float       i_0;              /* Radians                                */
    float       OMEGADOT;         /* Radians                                */
    float       sqrt_A;           /* Meters^0.5                             */
    float       OMEGA_0;          /* Radians                                */
    float       omega;            /* Radians                                */
    float       M_0;              /* Radians                                */
    float       a_f0;             /* Seconds                                */
    float       a_f1;             /* Seconds/Seconds                        */
    /* ---The following items are derived from broadcast eph block---     */
    float       Axis;             /* Meters                                 */
    float       n;                /* Radians/Seconds                        */
    float       OMEGA_n;          /* Radians                                */
    float       ODOT_n;           /* Radians                                */
    float       r1me2;            /* Dimensionless                          */
    float       Axen;             /* Meters                                 */
    float       t_zc;             /* time-z count of alm collection         */
    int16_t       weeknum;          /* Week number of alm collection          */
    int16_t       wn_oa;            /* Week number of alm block applicability */
    uint8_t        alm_src;          /* Source of alm block; used internally   */
    uint8_t        coll_sv;          /* SV the alm block was collected from    */

  } GPS_ALM_INFO;

  typedef struct
  {
    uint8_t       alm_status;   //valid flag
    uint8_t       AlmFlag;       //collect status flag
    uint8_t       Cn;
    uint8_t       MnA;
    uint8_t       nA;
    uint8_t       HnA;
    uint8_t       ln;
    uint8_t       KP;

    uint16_t      N_A;
    uint16_t      N4;

    double      TaunA;
    double      LamdanA;
    double      DeltainA;
    double      EpsilonnA;

    double      wnA;
    double      tLamdanA;
    double      DeltaTnA;
    double      DeltaTdotnA;

    double      B1;
    double      B2;
  } GLN_ALM_INFO;

  typedef struct
  {
    uint8_t       prn;
    uint8_t       alm_status;         // 0:invalid  1:valid
    uint8_t       alm_source;         // 1: decode almanac;  2: default almanac; 3: decode almanac and default WNa/no WNa
    uint8_t       AlmFlag;            //flag for Alm collect status.

    uint16_t      WNa;                //8 bits, range:0-255
    uint32_t      toa;                //sf4, page 1-24; sf5, page 1-6
    uint32_t      toa1;               //sf5, page8
    float      a1;
    float      a0;
    float      ecc;
    double      sqrta;
    double      omega0;
    double      deltai;
    double      omegaDot;
    double      w;
    double      M0;
  } BDS_ALM_INFO;

  typedef struct {
    uint8_t   alm_status;         // 0:invalid  1:valid
    uint8_t   alm_source;         // 1: decode almanac;  2: default almanac; 3: decode almanac and default WNa/no WNa
    uint8_t   svID;
    double  sqrt_A;
    double  e;
    double  sigma_i;
    double  omega_0;
    double  omega_dot;
    double  w;
    double  m0;
    double  af0;
    double  af1;

    uint8_t   e5a_hs;
    uint8_t   e5b_hs;
    uint8_t   e1b_hs;

    uint8_t   IOD_a;
    double  t_0a;
    uint8_t   wn_a;
  }GAL_ALM_INFO;

  ///////////////////////////////////////////////////////////////////////////
  // UTC model
  typedef struct
  {
    double     utcA1;
    double     utcA0;
    int32_t     utcTot;
    int32_t     utcWNt;
    int32_t     utcDeltaTls;
    int32_t     utcWNlsf;
    int32_t     utcDN;
    int32_t     utcDeltaTlsf;
    uint8_t      have_utc;
  } gpsUtcModel_t;

  typedef struct
  {
    int32_t     nA;
    int32_t     tauC;
    double     b1;
    double     b2;
    uint8_t      kp;
    uint8_t      have_utc;
  } glnUtcModel_t;

  typedef struct
  {
    double     utcA0;
    double     utcA1;
    int32_t     utcDeltaTls;
    int32_t     utcWNlsf;
    int32_t     utcDN;
    int32_t     utcDeltaTlsf;
    uint8_t      have_utc;
  } bdsUtcModel_t;

  typedef struct {
    double  a0;
    double  a1;
    double  deltT_ls;
    double  t_0t;
    uint8_t   wn_0t;
    uint8_t   wn_lsf;
    uint8_t   dn;
    int8_t   deltT_lsf;

    uint8_t   have_utc;
  } galUtcModel_t;

  //////////////////////////////////////////////////////////////////////////
  //IONO model
  typedef struct {
    double a_i0;
    double a_i1;
    double a_i2;

    uint8_t  sf1;
    uint8_t  sf2;
    uint8_t  sf3;
    uint8_t  sf4;
    uint8_t  sf5;

    uint8_t  have_iono;
  }GAL_IONO_INFO;


  ////////////////////////////////////////////////////////////////////////////////////
  typedef struct
  {
    void* almData[MAX_PRN_ALL_MODE];
    void* ephData[MAX_PRN_ALL_MODE];
    void* ephDataBack[MAX_PRN_ALL_MODE];
    ION_INFO         ionoData[GNSS_MAX_MODE];
    GAL_IONO_INFO    ionoData_gal;
    gpsUtcModel_t    gpsUtcModel;
    glnUtcModel_t    glnUtcModel;
    bdsUtcModel_t    bdsUtcModel;
    galUtcModel_t    galUtcModel;
    int8_t               gloChannel[MAX_GLN_PRN];

    //bad sv list
    uint8_t               bdsBadSvList[MAX_BDS_PRN];
    uint16_t              week_decoded[GNSS_MAX_MODE];   /*last decoded week number*/
  } gnss_nm_t;


  // NM related functions
  int32_t gnss_sv_Idx(uint32_t gnssMode, uint32_t prn);
  int32_t gnss_Idx_sv(int32_t idx, int8_t* gnssMode);
  void gnss_Sd_Nm_Del(void);
  uint8_t gnss_Sd_Nm_AddEph(uint8_t gnssMode, uint8_t sv_id, void* src);
  void gnss_Sd_Nm_AddIono(uint8_t gnssMode, ION_INFO* iono);
  void gnss_Sd_Nm_AddIono_gal(uint8_t gnssMode, GAL_IONO_INFO iono);
  void gnss_Sd_Nm_AddUtc(uint8_t gnssMode, void* src);
  void gnss_Sd_Nm_AddAlm(uint8_t gnssMode, uint8_t sv_id, void* src);
  ION_INFO* gnss_Sd_Nm_GetIono(uint8_t gnssMode);
  void* gnss_Sd_Nm_GetEph(uint8_t gnssMode, uint8_t sv_id);
  int8_t gnss_Sd_Nm_GetGlnChanN(int32_t prn);
  void gnss_Sd_Nm_SetGlnChanN(int32_t prn, int8_t fcn);
  uint8_t gnss_Sd_Nm_Check_NoEph(uint8_t gnssMode, uint8_t sv_id);
  void* gnss_Sd_Nm_GetAlm(uint8_t gnssMode, uint8_t sv_id);
  void gnss_Sd_Nm_RmSv(uint8_t gnssMode, uint8_t sv_id);
  void gnss_Sd_Nm_SaveWeek(uint8_t eph_source, uint8_t gnssMode, uint16_t weeknum, GNSS_TIME* pTime);
  void gnss_Sd_Nm_SaveNT(uint8_t eph_source, uint8_t gnssMode, uint16_t NT, uint16_t N4, GNSS_TIME* pTime);
  void gnss_Sd_Nm_AddChlNo(uint8_t gnssMode, int8_t* gln_channelNo, uint8_t length);
  void gnss_sd_nm_init_glo_chn(void);
#ifdef __cplusplus
}
#endif


#endif
