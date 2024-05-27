#ifndef __DPI_SDK_H__
#define __DPI_SDK_H__



#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NTRIP_IP_MAX_LEN		(32)
#define NTRIP_USER_MAX_LEN		(32)
#define NTRIP_PWD_MAX_LEN		(32)
#define NTRIP_MOUNT_MAX_LEN		(32)		


#ifndef DY_NFREQ
#define DY_NFREQ       5                   /* number of carrier frequencies */
#endif

#ifndef DY_NEXOBS
#define DY_NEXOBS      0                   /* number of extended mObs codes */
#endif

#define DY_MAXSTA      255

#ifndef DY_MAXOBS
#define DY_MAXOBS      96                  /* max number of mObs in an epoch */
#endif

#ifndef DY_MAXSAT
#define DY_MAXSAT      181                  /* max number of mObs in an epoch */
#endif


#ifndef DY_NSATGLO
#define DY_NSATGLO      0                  /* max number of mObs in an epoch */
#endif


#ifndef DY_MAXOBSBUF
#define DY_MAXOBSBUF    20                  /* max number of mObs in an epoch */
#endif

#define DY_MAXCODE     68                  /* max number of mObs code */

#define DY_NSYS        3                  /* max number of mObs code */



#ifndef DY_SYS
#define DY_SYS_NONE    0x00                /* navigation system: none */
#define DY_SYS_GPS     0x01                /* navigation system: GPS */
#define DY_SYS_SBS     0x02                /* navigation system: SBAS */
#define DY_SYS_GLO     0x04                /* navigation system: GLONASS */
#define DY_SYS_GAL     0x08                /* navigation system: Galileo */
#define DY_SYS_QZS     0x10                /* navigation system: QZSS */
#define DY_SYS_CMP     0x20                /* navigation system: BeiDou */
#define DY_SYS_IRN     0x40                /* navigation system: IRNS */
#define DY_SYS_LEO     0x80                /* navigation system: LEO */
#define DY_SYS_ALL     0xFF                /* navigation system: all */
#endif

/** 
 * @enum E_SOLUTION_MODE
 * @brief 解算模式
 */
typedef enum E_SOLUTION_MODE
{
	E_SOLQ_NONE   = 0,  /* solution mStatus: no solution */
	E_SOLQ_FIX    = 1,  /* solution mStatus: fix */
 	E_SOLQ_FLOAT  = 2,  /* solution mStatus: float */
 	E_SOLQ_SBAS   = 3,  /* solution mStatus: SBAS */
 	E_SOLQ_DGPS   = 4,  /* solution mStatus: DGPS/DGNSS */
 	E_SOLQ_SINGLE = 5,  /* solution mStatus: single */
 	E_SOLQ_PPP    = 6,  /* solution mStatus: PPP */
 	E_SOLQ_DR     = 7,  /* solution mStatus: dead reconing */
 	E_MAXSOLQ     = 7,  /* max number of solution mStatus */
}E_SOLUTION_MODE;

/**
 * @struct DefaultConfig_t
 * @brief SPE默认配置
 */
typedef struct DefaultConfig_t{
	char base_ip[NTRIP_IP_MAX_LEN];       //基准站IP
	int  base_port;                       //基准站端口
	char base_user[NTRIP_USER_MAX_LEN];   //基准站用户名
	char base_pwd[NTRIP_PWD_MAX_LEN];     //基准站密码
	char base_mount[NTRIP_MOUNT_MAX_LEN]; //基准站挂载点
}DefaultConfig_t;

/**
 * @struct Gtime_t
 * @brief  time描述
 */
typedef struct {        /* time struct */
    time_t time;        /* time (s) expressed by standard time_t */
    double sec;         /* fraction of second under 1 s */
} Gtime_t;

/**
 * @struct DySatLos_t
 * @brief  Los数据
 */
/*------------usage of the Los Correction-------------------*/
//LosCorr_phase=OrtClk+PahseCorr+Stec+Phw+Trp*/
//LosCorr_phase=OrtClk+CodeCorr-Stec+Trp*/
typedef struct DySatLos_t {   /* Comprehensive error */
    Gtime_t time;             /* receiver sampling time (GPST) */
    uint16_t sat;             /* satellite/receiver number */
    int iode;
    int sys;
    int prn;
    uint16_t code[DY_NFREQ + DY_NEXOBS];
    double OrtClk;                              /* unit:m*/
    double PahseCorr[DY_NFREQ + DY_NEXOBS];     /* unit:m*/
    double CodeCorr[DY_NFREQ + DY_NEXOBS];      /* unit:m*/
    double Stec[DY_NFREQ + DY_NEXOBS];          /* Slant phase ionosphric correction, unit:m*/
    double Phw[DY_NFREQ + DY_NEXOBS];           /* unit:m*/
    double Trp;                                 /* Slant Tropospheric correction, unit:m*/
    double Stec_std[DY_NFREQ + DY_NEXOBS];      /* standard deviation of ionosphric correction, unit:m*/
    Gtime_t SsrTime; 
} DySatLos_t;
/*------------usage of the Los Correction-------------------*/

/**
 * @struct DyEph_t
 * @brief  GPS/BDS/GAL/QZS星历数据
 */
typedef struct {        /* GPS/QZS/GAL broadcast ephemeris type */
    int sat;            /* satellite number */
    int iode,iodc;      /* IODE,IODC */
    int sva;            /* SV accuracy (URA index) */
    int svh;            /* SV health (0:ok) */
    int week;           /* GPS/QZS: gps week, GAL: galileo week */
    int code;           /* GPS/QZS: code on L2 */
                        /* GAL: data source defined as rinex 3.03 */
                        /* BDS: data source (0:unknown,1:B1I,2:B1Q,3:B2I,4:B2Q,5:B3I,6:B3Q) */
    int flag;           /* GPS/QZS: L2 P data flag */
                        /* BDS: mNav type (0:unknown,1:IGSO/MEO,2:GEO) */
    Gtime_t toe,toc,ttr; /* Toe,Toc,T_trans */
                        /* SV orbit parameters */
    double A,e,i0,OMG0,omg,M0,deln,OMGd,idot;
    double crc,crs,cuc,cus,cic,cis;
    double toes;        /* Toe (s) in week */
    double fit;         /* fit interval (h) */
    double f0,f1,f2;    /* SV mClock parameters (af0,af1,af2) */
    double tgd[6];      /* group delay parameters */
                        /* GPS/QZS:tgd[0]=TGD */
                        /* GAL:tgd[0]=BGD_E1E5a,tgd[1]=BGD_E1E5b */
                        /* CMP:tgd[0]=TGD_B1I ,tgd[1]=TGD_B2I/B2b,tgd[2]=TGD_B1Cp */
                        /*     tgd[3]=TGD_B2ap,tgd[4]=ISC_B1Cd   ,tgd[5]=ISC_B2ad */
    double Adot,ndot;   /* Adot,ndot for CNAV */
} DyEph_t;

/**
 * @struct DyGeph_t
 * @brief  GLO星历数据
 */
typedef struct {        /* GLONASS broadcast ephemeris type */
    int sat;            /* satellite number */
    int iode;           /* IODE (0-6 bit of tb field) */
    int frq;            /* satellite frequency number */
    int svh,sva,age;    /* satellite health, accuracy, age of operation */
    Gtime_t toe;        /* epoch of epherides (gpst) */
    Gtime_t tof;        /* message frame time (gpst) */
    double pos[3];      /* satellite position (ecef) (m) */
    double vel[3];      /* satellite velocity (ecef) (m/s) */
    double acc[3];      /* satellite acceleration (ecef) (m/s^2) */
    double taun,gamn;   /* SV mClock bias (s)/relative freq bias */
    double dtaun;       /* delay between L1 and L2 (s) */
} DyGeph_t;

/**
 * @struct DyObsd_t
 * @brief  卫星观测数据描述
 */
typedef struct {        /* observation data record */
    Gtime_t time;       /* receiver sampling time (GPST) */
    uint16_t sat,rcv;    /* satellite/receiver number */
    uint16_t SNR[DY_NFREQ+DY_NEXOBS]; /* signal strength (0.001 dBHz) */
    uint16_t  LLI[DY_NFREQ+DY_NEXOBS]; /* loss of lock indicator */
    uint16_t code[DY_NFREQ+DY_NEXOBS]; /* code indicator (CODE_???) */
    double L[DY_NFREQ+DY_NEXOBS]; /* observation data carrier-phase (cycle) */
    double P[DY_NFREQ+DY_NEXOBS]; /* observation data pseudorange (m) */
    float  D[DY_NFREQ+DY_NEXOBS]; /* observation data doppler frequency (Hz) */
} DyObsd_t;

typedef struct {        /* observation data */
    uint16_t n,nmax;    /* number of obervation data/allocated */
    DyObsd_t *data;     /* observation data records */
} DyObs_t;


/**
 * spe初始化
 * @param DefaultConfig_t *config [in] 默认配置参数
 * @return true, 成功
 *         false,失败
**/
bool DySdkInit(DefaultConfig_t *config);


/**
 * 获取los数据
 * @param  Obs_t*   obs   輸入   观测移动数据
 * @param  DyNav_t* nav   输入   基站差分数据
 * @param  Los_t*   los   输出   los数据
 * @return true    成功 
 *         false   失败
**/
bool DyGetLos(DyObs_t *obs, DyEph_t* eph, DyGeph_t *geph, DySatLos_t *los);

/**
 * 更新经纬度
 * @param  longitude  输入   lon  
 * @param  latitude   输入   lat
 * @return null
**/
void DySetGGA(double longitude, double latitude);


/**
 * @param char *sys   [in] 输入系统类型，比如：GPS/BDS/GAL
 * @param char *fre   [in] 输入频点比如：L1C/L2C
 * @return true, 成功
 *         false,失败
**/
int SetFrePoint(char *sys, char *fre);



/**
 * spe退出
 * @param null
 * @return null
**/
void DySdkExit(void);




#ifdef __cplusplus
}
#endif

#endif