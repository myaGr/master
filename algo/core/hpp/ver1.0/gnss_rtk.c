/************************************************************
* Copyrights(C) ASG Corporation.
* All rights Reserved
* File: gnss_rtk.c
* Description: 
* Version: 1.0.0
* Author: none
* Date: 05/08
************************************************************/
#include "macro.h"
#include "rtklib.h"
#include "gnss_sys_api.h"
#include "gnss_sd_nm.h"
#include "gnss_tm.h"
#include "gnss_rtk.h"
#include "gnss_config.h"
#include "gnss_mode.h"
#include "gnss_math.h"
#include "gnss_pe.h"
#include "gnss_sd.h"
#include "gnss_cfd.h"

#undef MODULE_TAG
#define MODULE_TAG OBJ_RTK

#ifdef AG_GNSS_RTK_FUNCTION_IMPL

#ifndef PLAYBACK_MODE
#define STATFILE    "rtk_realtime.status"  /* solution status file, rtk_%Y%m%d%h%M.status */
#define TRACEFILE   "rtk_realtime.debug" /* debug trace file, rtk_%Y%m%d%h%M.debug */
#define SOLOUTFILE  "rtk_realtime.pos"   /* solution output file, rtk_%Y%m%d%h%M.pos */
#else
#define STATFILE    "rtk_postproc.status"  /* solution status file */
#define TRACEFILE   "rtk_postproc.debug" /* debug trace file */
#define SOLOUTFILE  "rtk_postproc.pos"   /* solution output file */
#endif

#define VALIDPOSNORM  1.0E4
#define DIFFPOSNORM   0.1
#define DIFFSPPVRS    2500.0       /* SPP-VRS */
#define DIFFSPPREF    50.0         /* SPP-REFPOS */
#define TDTGAP        5.0          /* TD time gap */            

/*for confidence*/
#define RTKFLTHERR683		0.8013
#define RTKFLTHERR955		1.7232
#define RTKFLTHERR997		9.3938
#define RTKFLTUERR683		1.4151
#define RTKFLTUERR955		3.2967
#define RTKFLTUERR997		12.4537

//obs_t refobs[MAXOBSBUF];  //ref obs buffer
rtk_t rtkctrl;            //rtk control and results
obs_t rtkobs;             //ref+rov obs
obs_t rtkobspre;          //rov obs
nav_t rtknav;             //nav data
pe_rtk_data_t rtcm3data;  //rtcm3 data from NRTK svr

int   rtkinitflag=0;        //0: not init yet  1: inited
int   first_static_period = 0;

#ifdef PLAYBACK_MODE
static prcopt_t prcopt;                 /* processing options */
static solopt_t solopt;                 /* solution options */
static filopt_t filopt  ={""};          /* file options */
#endif
/* out file variables ----------------------------------------------------------*/
#ifdef PLAYBACK_MODE
static int soloutflag=1;           /* rtk solution output level (0:off) */
static FILE *fp_solout=NULL;       /* rtk output file pointer */
static char file_solout[1024]="";  /* rtk output file original path */
static gtime_t time_solout={0};    /* rtk output file time */
#endif

/* debug rinex output file */
#ifdef PLAYBACK_MODE
#define RINEXOBSROV  "rtk_rov."
#define RINEXOBSREF  "rtk_ref."
#define RINEXOBSNAV  "rtk_nav."
static FILE *fp_rnx_rov=NULL;
static FILE *fp_rnx_ref=NULL;
static FILE *fp_rnx_nav=NULL;
static rnxopt_t rnx_opt;
#endif

/* extern global data */
extern PE_MODES  peMode;
extern Kf_t*      gpz_kf_data;
extern USER_PVT*  gpz_user_pvt;
extern Gnss_Cfg_t  g_pe_cfg;
extern Sd_t Sd_data;
extern PeStateMonitor_t        peState;
extern proc_ctrl_t g_proc;

static int gnss_rtk_asgobsd2rtkobs(asg_obs_t* asg_obs, const meas_blk_t* pMeas,  GNSS_TIME* pTime, obs_t* obsdata);

/* get sum of '1' in binary format marklist for specific LSB -----------------------------------------------------
* note: 
* args   : 
* return :
* author : Kun Xu
*-----------------------------------------------------------------------------*/
extern uint8_t sumofbit1(uint64_t marklist, uint8_t lsbwide)
{
    uint8_t i;
    uint8_t cnt = 0;

    for(i = 0; i < lsbwide; i++)
    {
        if ((marklist >> i) & 1)  cnt++;
    }

    return cnt;
}
/* matrix transpose -----------------------------------------------------
* note: 
* args   : 
* return :
* author : Kun Xu
*-----------------------------------------------------------------------------*/
extern void mattrans(double *H,int row,int col,double *T)
{
    int i,j;
    
    for (i=0;i<row;i++)
    {
        for (j=0;j<col;j++)
        {
            T[j+col*i]=H[i+row*j];
        }
    }
}
/* delete one column from matrix -----------------------------------------------------
* note: row>=1,col>=2
* args   : 
* return :
* author : Kun Xu
*-----------------------------------------------------------------------------*/
extern int matdelcol(double *H,int row,int col,int colinx)
{
    int i;
    double *head;

    if (row<1||col<2||colinx<1||colinx>col) return -1;

    for (i=colinx-1;i<col-1;i++)
    {
        head=H+i*row;
        memcpy(head,head+row,sizeof(double)*row);
    }
    memset(H+(col-1)*row,0,sizeof(double)*row);

    return col-1;
}
/* delete one row from matrix -----------------------------------------------------
* note: row>=2,col>=1
* args   : 
* return :
* author : Kun Xu
*-----------------------------------------------------------------------------*/
extern int matdelrow(double *H,int row,int col,int rowinx)
{
    double *T;

    if (row<2||col<1||rowinx<1||rowinx>row) return -1;

    if ((T=mat(row,col))==NULL) return -1;

    mattrans(H,row,col,T);

    if (matdelcol(T, col, row, rowinx) < 0)
    {
        Sys_Free(T);
        return -1;
    }

    mattrans(T,col,row-1,H);

    Sys_Free(T);

    return row-1;
}
/* get process option configure -------------------------------------------------------------*/
extern prcopt_t* gnss_rtk_get_prcopt()
{
    /*if (!rtkinitflag)
    {
        return NULL;
    }*/
#ifdef PLAYBACK_MODE
    return &prcopt;
#else
    return NULL;
#endif
}
/* get solution option configure -------------------------------------------------------------*/
extern solopt_t* gnss_rtk_get_solopt()
{
    /*if (!rtkinitflag)
    {
        return NULL;
    }*/
#ifdef PLAYBACK_MODE
    return &solopt;
#else
    return NULL;
#endif
}
/* update refpos opt in rtkctrl -------------------------------------------------------------*/
extern void gnss_rtk_set_refposopt(int refposopt)
{
    rtkctrl.opt.refpos=refposopt;
}

/* set IODE for satpos  -------------------------------------------------------------*/
extern void gnss_rtk_set_refeph_iode(int sat,int iode)
{
    if (rtkctrl.ssat[sat - 1])
    {
        rtkctrl.ssat[sat - 1]->slp_ref.iode <<= 16;
        rtkctrl.ssat[sat - 1]->slp_ref.iode |= iode & 0xFFFF;
    }
}

/* set TOE for satpos  -------------------------------------------------------------*/
extern void gnss_rtk_set_refeph_toe(int sat, float toe)
{
    if (NULL != rtkctrl.ssat[sat - 1])
    {
        rtkctrl.ssat[sat - 1]->slp_ref.toe[1] = rtkctrl.ssat[sat - 1]->slp_ref.toe[0];
        rtkctrl.ssat[sat - 1]->slp_ref.toe[0] = toe;
    }
}

/* update pos in opt in rtkctrl -------------------------------------------------------------*/
extern void gnss_rtk_ud_optpos(double rb[3],double ru[3])
{
    int i;

    if (!rb)
    {
        GLOGE("%s: NULL rb", __FUNCTION__);
        return;
    }
    if (asg_norm(rb, 3)<VALIDPOSNORM)
    {
        GLOGE("%s: invalid rb in ref rinex head. rb=[%.3f %.3f %.3f]", __FUNCTION__, rb[0], rb[1], rb[2]);
        return;
    }
    for (i=0;i<3;i++)
    {
        rtkctrl.opt.rb[i]=rb[i];
        if (ru) rtkctrl.opt.ru[i] = ru[i];
    }

#ifdef ENABLE_PPK_MODE
    /* back up */
    for (i=0;i<3;i++)
    {
        rtcm3data.rb[1][i] = rtcm3data.rb[0][i];
        rtcm3data.rb[0][i] = rb[i];
    }
    rtcm3data.base_switch = 1;
    rtcm3data.info_mask |= PE_RTCM_INFO_REFPOS;
    GLOGI("%s: ref pos update: rb= %.4f %.4f %.4f", __FUNCTION__, rb[0], rb[1], rb[2]);

#endif
}
/* open solution output file -------------------------------------------------------------*/
#ifdef PLAYBACK_MODE
static int gnss_rtk_open_solout(const char *file, int level)
{
    gtime_t time=utc2gpst(timeget());
    char path[1024]={0};
    if (!file) return 0;

    gnss_util_trace(3,"gnss_rtk_open_solout: file=%s level=%d\n",file,level);

    if (level<=0) return 0;

    reppath(file,path,time,"","");

    if (!(fp_solout=fopen(path,"w"))) 
    {
        gnss_util_trace(1,"gnss_rtk_open_solout: file open error path=%s\n",path);
        return 0;
    }
    strncpy(file_solout,file,sizeof(file_solout)-1);
    time_solout=time;
    soloutflag=level;
    return 1;
}
/* close solution output file ------------------------------------------------*/
static void gnss_rtk_close_solout(void)
{
    gnss_util_trace(3,"gnss_rtk_close_solout:\n");

    if (fp_solout) fclose(fp_solout);
    fp_solout=NULL;
    file_solout[0]='\0';
    soloutflag=0;
}
/* output reference position -------------------------------------------------*/
static void gnss_rtk_outrefpos(FILE *fp, const double *r, const solopt_t *opt)
{
    double pos[3],dms1[3],dms2[3];
    const char *sep=opt->sep;

    gnss_util_trace(3,"gnss_rtk_outrefpos :\n");

    if (opt->posf==SOLF_LLH||opt->posf==SOLF_ENU) 
    {
        ecef2pos(r,pos);
        if (opt->degf) 
        {
            deg2dms(pos[0]*R2D,dms1,5);
            deg2dms(pos[1]*R2D,dms2,5);
            fprintf(fp,"%3.0f%s%02.0f%s%08.5f%s%4.0f%s%02.0f%s%08.5f%s%10.4f",
                dms1[0],sep,dms1[1],sep,dms1[2],sep,dms2[0],sep,dms2[1],
                sep,dms2[2],sep,pos[2]);
        }
        else 
        {
            fprintf(fp,"%13.9f%s%14.9f%s%10.4f",pos[0]*R2D,sep,pos[1]*R2D,
                sep,pos[2]);
        }
    }
    else if (opt->posf==SOLF_XYZ) 
    {
        fprintf(fp,"%14.4f%s%14.4f%s%14.4f",r[0],sep,r[1],sep,r[2]);
    }
}
/* write output header -------------------------------------------------------------*/
static void gnss_rtk_outheader(FILE *fp, const prcopt_t *popt, const solopt_t *sopt)
{
#if 0
    gnss_util_trace(3,"gnss_rtk_outheader:\n");

    if (sopt->posf==SOLF_NMEA||sopt->posf==SOLF_STAT) {
        return;
    }
    if (sopt->outhead) 
    {
        fprintf(fp,"%s program   : %s\n",COMMENTH,sopt->prog);
    }
    if (sopt->outopt) 
    {
        outprcopt(fp,popt);
    }
    if (popt->refpos==POSOPT_POS&&PMODE_DGPS<=popt->mode&&popt->mode<=PMODE_FIXED&&popt->mode!=PMODE_MOVEB) 
    {
        fprintf(fp,"%s ref pos   :",COMMENTH);
        gnss_rtk_outrefpos(fp,popt->rb,sopt);
        fprintf(fp,"\n");
    }
    if (sopt->outhead||sopt->outopt) fprintf(fp,"%s\n",COMMENTH);

    outsolhead(fp,sopt);
#endif
}
/* open rinex output file -------------------------------------------------------------*/
static int gnss_rtk_open_rinex(const char *path)
{
    double ep[6]={0.0};
    char rnxfile[1024]={0};
    int year2=0;

    gnss_time2epoch(timeget(),ep);
    year2=((int)ep[0])%100;
    memset(rnxfile,0,sizeof(rnxfile));
    sprintf(rnxfile, "%s%c%s%2dO", path, FILEPATHSEP, RINEXOBSROV,year2);
    if (!(fp_rnx_rov=fopen(rnxfile,"w"))) 
    {
        GLOGE("%s: file open error path=%s\n",__FUNCTION__,rnxfile);
        return 0;
    }
    memset(rnxfile,0,sizeof(rnxfile));
    sprintf(rnxfile, "%s%c%s%2dO", path, FILEPATHSEP, RINEXOBSREF,year2);
    if (!(fp_rnx_ref=fopen(rnxfile,"w"))) 
    {
        GLOGE("%s: file open error path=%s\n",__FUNCTION__,rnxfile);
        return 0;
    }
    /*memset(rnxfile,0,sizeof(rnxfile));
    sprintf(rnxfile, "%s%c%s%2dn", path, FILEPATHSEP, RINEXOBSNAV,year2);
    if (!(fp_rnx_nav=fopen(rnxfile,"w"))) 
    {
        GLOGE("%s: file open error path=%s\n",__FUNCTION__,rnxfile);
        return 0;
    }*/
    return 1;
}
/* close rinex output file -------------------------------------------------------------*/
static int gnss_rtk_close_rinex()
{
    if (fp_rnx_rov) 
    {
        fclose(fp_rnx_rov);
        fp_rnx_rov=NULL;
    }
    
    if (fp_rnx_ref) 
    {
        fclose(fp_rnx_ref);
        fp_rnx_ref=NULL;
    }
    
    if (fp_rnx_nav) 
    {
        fclose(fp_rnx_nav);
        fp_rnx_nav=NULL;
    }
    return 1;
}
/* init rinex option block -------------------------------------------------------------*/
static void gnss_rtk_rnxopt_init()
{
    int i,j;
    rnx_opt.tint=1;
    rnx_opt.rnxver=3.02;
    rnx_opt.navsys=SYS_GPS|SYS_CMP|SYS_GLO|SYS_GAL|SYS_QZS;
    rnx_opt.obstype=OBSTYPE_PR|OBSTYPE_CP|OBSTYPE_DOP|OBSTYPE_SNR;
    rnx_opt.freqtype=FREQTYPE_ALL;//FREQTYPE_L1|FREQTYPE_L2;
    for (i=0;i<6;i++) for (j=0;j<64;j++) rnx_opt.mask[i][j]='1';
    
    /* GPS */
    rnx_opt.nobs[0]=12;
    strncpy(rnx_opt.tobs[0][0],"C1C",4);
    strncpy(rnx_opt.tobs[0][1],"L1C",4);
    strncpy(rnx_opt.tobs[0][2],"D1C",4);
    strncpy(rnx_opt.tobs[0][3],"S1C",4);
    strncpy(rnx_opt.tobs[0][4],"C2W",4);
    strncpy(rnx_opt.tobs[0][5],"L2W",4);
    strncpy(rnx_opt.tobs[0][6],"D2W",4);
    strncpy(rnx_opt.tobs[0][7],"S2W",4);
    strncpy(rnx_opt.tobs[0][8], "C5Q",4);
    strncpy(rnx_opt.tobs[0][9], "L5Q",4);
    strncpy(rnx_opt.tobs[0][10],"D5Q",4);
    strncpy(rnx_opt.tobs[0][11],"S5Q",4);
    /*strncpy(rnx_opt.tobs[0][8], "C2P",4);
    strncpy(rnx_opt.tobs[0][9], "L2P",4);
    strncpy(rnx_opt.tobs[0][10],"D2P",4);
    strncpy(rnx_opt.tobs[0][11],"S2P",4);*/
    

    /* GLN */
    rnx_opt.nobs[1]=8;
    strncpy(rnx_opt.tobs[1][0],"C1C",4);
    strncpy(rnx_opt.tobs[1][1],"L1C",4);
    strncpy(rnx_opt.tobs[1][2],"D1C",4);
    strncpy(rnx_opt.tobs[1][3],"S1C",4);
    strncpy(rnx_opt.tobs[1][4],"C2P",4);
    strncpy(rnx_opt.tobs[1][5],"L2P",4);
    strncpy(rnx_opt.tobs[1][6],"D2P",4);
    strncpy(rnx_opt.tobs[1][7],"S2P",4);

    /* GAL */
    rnx_opt.nobs[2]=12;
    strncpy(rnx_opt.tobs[2][0],"C1C",4);
    strncpy(rnx_opt.tobs[2][1],"L1C",4);
    strncpy(rnx_opt.tobs[2][2],"D1C",4);
    strncpy(rnx_opt.tobs[2][3],"S1C",4);
    strncpy(rnx_opt.tobs[2][4],"C7I",4);
    strncpy(rnx_opt.tobs[2][5],"L7I",4);
    strncpy(rnx_opt.tobs[2][6],"D7I",4);
    strncpy(rnx_opt.tobs[2][7],"S7I",4);
    strncpy(rnx_opt.tobs[2][8],"C5I",4);
    strncpy(rnx_opt.tobs[2][9],"L5I",4);
    strncpy(rnx_opt.tobs[2][10],"D5I",4);
    strncpy(rnx_opt.tobs[2][11],"S5I",4);

    /* QZS */
    rnx_opt.nobs[3]=12;
    strncpy(rnx_opt.tobs[3][0],"C1C",4);
    strncpy(rnx_opt.tobs[3][1],"L1C",4);
    strncpy(rnx_opt.tobs[3][2],"D1C",4);
    strncpy(rnx_opt.tobs[3][3],"S1C",4);
    strncpy(rnx_opt.tobs[3][4],"C2X",4);
    strncpy(rnx_opt.tobs[3][5],"L2X",4);
    strncpy(rnx_opt.tobs[3][6],"D2X",4);
    strncpy(rnx_opt.tobs[3][7],"S2X",4);
    strncpy(rnx_opt.tobs[3][8],"C5Q",4);
    strncpy(rnx_opt.tobs[3][9],"L5Q",4);
    strncpy(rnx_opt.tobs[3][10],"D5Q",4);
    strncpy(rnx_opt.tobs[3][11],"S5Q",4);
    
    /* BDS */
    rnx_opt.nobs[5]=12;
    strncpy(rnx_opt.tobs[5][0],"C1I",4);
    strncpy(rnx_opt.tobs[5][1],"L1I",4);
    strncpy(rnx_opt.tobs[5][2],"D1I",4);
    strncpy(rnx_opt.tobs[5][3],"S1I",4);
    strncpy(rnx_opt.tobs[5][4],"C7I",4);
    strncpy(rnx_opt.tobs[5][5],"L7I",4);
    strncpy(rnx_opt.tobs[5][6],"D7I",4);
    strncpy(rnx_opt.tobs[5][7],"S7I",4);
    strncpy(rnx_opt.tobs[5][8], "C6I",4);
    strncpy(rnx_opt.tobs[5][9], "L6I",4);
    strncpy(rnx_opt.tobs[5][10],"D6I",4);
    strncpy(rnx_opt.tobs[5][11],"S6I",4);
    
}
#endif

static void gnss_rtk_outer_cfg(const rtk_algo_cfg_t *rtk_cfg, prcopt_t *prc_opt, solopt_t *sol_opt)
{
    if (rtk_cfg == NULL || prc_opt == NULL)
    {
        GLOGI("null rtk config params!");
        return;
    }

    switch (rtk_cfg->pos_mode)
    {
    case RTK_POS_MODE_SINGLE:
        prc_opt->mode = PMODE_SINGLE; break;
    case RTK_POS_MODE_DGPS:
        prc_opt->mode = PMODE_DGPS; break;
    case RTK_POS_MODE_KINEMATIC:
        prc_opt->mode = PMODE_KINEMA; break;
    case RTK_POS_MODE_STATIC:
        prc_opt->mode = PMODE_STATIC; break;
    case RTK_POS_MODE_MOVINGBASE:
        prc_opt->mode = PMODE_MOVEB; break;
    case RTK_POS_MODE_FIXED:
        prc_opt->mode = PMODE_FIXED; break;
    case RTK_POS_MODE_PPP_KINE:
        prc_opt->mode = PMODE_PPP_KINEMA; break;
    case RTK_POS_MODE_PPP_STATIC:
        prc_opt->mode = PMODE_PPP_STATIC; break;
    case RTK_POS_MODE_PPP_FIXED:
        prc_opt->mode = PMODE_PPP_FIXED; break;
    default:
        break;
    }

    prc_opt->elmin = rtk_cfg->elevation_mask*D2R;
    prc_opt->navsys = 0x0;
    if (rtk_cfg->nav_sys_gps) prc_opt->navsys |= SYS_GPS;
    if (rtk_cfg->nav_sys_sbas) prc_opt->navsys |= SYS_SBS;
    if (rtk_cfg->nav_sys_glonass) prc_opt->navsys |= SYS_GLO;
    if (rtk_cfg->nav_sys_galileo) prc_opt->navsys |= SYS_GAL;
    if (rtk_cfg->nav_sys_qzss) prc_opt->navsys |= SYS_QZS;
    if (rtk_cfg->nav_sys_compass) prc_opt->navsys |= SYS_CMP;
    
    switch (rtk_cfg->ar_mode)
    {
    case RTK_ARMODE_OFF:
        prc_opt->modear = ARMODE_OFF; break;
    case RTK_ARMODE_CONTINUOUS:
        prc_opt->modear = ARMODE_CONT; break;
    case RTK_ARMODE_INSTANTANEOUS:
        prc_opt->modear = ARMODE_INST; break;
    case RTK_ARMODE_FIX_HOLD:
        prc_opt->modear = ARMODE_FIXHOLD; break;
    default:
        break;
    }

    prc_opt->nf = NFREQ;
    /* number of frequency (4:L1+L5) */
    if (rtk_cfg->freq_combine == 4) 
    {
        prc_opt->nf = 3;
        prc_opt->freqopt = 1;
    }
    prc_opt->soltype = rtk_cfg->process_type;
    prc_opt->bdsmodear = rtk_cfg->bds_armode == 0 ? 0 : 1;
    prc_opt->glomodear = rtk_cfg->glo_armode;
    prc_opt->thresar[0] = rtk_cfg->ar_threshold;
    prc_opt->minlock = rtk_cfg->ar_lock_cnt;
    prc_opt->elmaskar = rtk_cfg->ar_elevation_mask*D2R;
    prc_opt->minfix = rtk_cfg->ar_min_fix;
    prc_opt->elmaskhold = rtk_cfg->elevation_mask_hold*D2R;
    prc_opt->maxout = rtk_cfg->ar_out_cnt;
    prc_opt->maxtdiff = rtk_cfg->max_age;
    prc_opt->maxinno = rtk_cfg->rej_thres_innovation;
    prc_opt->eratio[0] = rtk_cfg->err_ratio_1;
    prc_opt->eratio[1] = rtk_cfg->err_ratio_2;
    prc_opt->err[1] = rtk_cfg->err_phase_factor_a;
    prc_opt->err[2] = rtk_cfg->err_phase_factor_b;
    prc_opt->err[3] = rtk_cfg->err_phase_factor_c;
    prc_opt->err[4] = rtk_cfg->err_phase_doppler_freq;
    prc_opt->std[0] = rtk_cfg->init_state_std_bias;
    prc_opt->prn[0] = rtk_cfg->proc_noise_std_bias;

    //sol_opt->trace = rtk_cfg->debug_level;
}

/* initialize rtk server -------------------------------------------------------
* initialize rtk server
* args   : const char *fconf    I  configure file name
* return : status (0:error,1:ok)
* author : none
*-----------------------------------------------------------------------------*/
extern int gnss_rtk_init(const rtk_algo_cfg_t* rtk_cfg)
{
  int i, j, fbias = 0;
  Gnss_Test_Cfg_t* g_cfg = NULL;
  prcopt_t* prcopt = NULL;

  if (rtkinitflag)
  {
    return 1;
  }

  if (rtk_cfg == NULL)
  {
    GLOGI("null rtk config params!");
    return 1;
  }

  rtknav.eph = (eph_t*)OS_MALLOC(sizeof(eph_t) * MAXSAT * 2);
  if (NULL == rtknav.eph)
  {
    return 0;
  }
  memset(rtknav.eph, 0, sizeof(eph_t) * MAXSAT * 2);

  if (NSATGLO > 0)
  {
    /*if (!(rtknav.geph=(geph_t *)Sys_Malloc(sizeof(geph_t)*NSATGLO*2)))
    {
        Sys_Free(rtknav.eph);    rtknav.eph =NULL;
        return 0;
    }
    memset(rtknav.geph,0,sizeof(geph_t)*NSATGLO*2);*/
  }
  if (NSATSBS > 0)
  {
    /*if (!(rtknav.seph=(seph_t *)Sys_Malloc(sizeof(seph_t)*NSATSBS*2)))
    {
        Sys_Free(rtknav.eph);    rtknav.eph =NULL;
        Sys_Free(rtknav.geph);   rtknav.geph =NULL;
        return 0;
    }
    memset(rtknav.seph,0,sizeof(seph_t)*NSATSBS*2);*/
  }

  rtknav.n = rtknav.nmax = MAXSAT * 2; //rtknav.n =0;
  //rtknav.ng=rtknav.ngmax=NSATGLO*2; //rtknav.ng=0;
  //rtknav.ns=rtknav.nsmax=NSATSBS*2; //rtknav.ns=0;
  //set default wavelength for non-GLONASS systems
  for (i = 0; i < MAXSAT; i++) for (j = 0; j < NFREQ; j++) {
    //fbias = 0;
#ifdef FREQ_L1L5
    //if (NFREQ == 2)
    {
      fbias = (j == 0) ? 0 : 1;
    }
#endif
    rtknav.lam[i][j] = satwavelen(i + 1, j + fbias, NULL);
  }

  for (i = 0; i < MAXOBSBUF; i++)
  {
    rtcm3data.obs[i].data = (obsd_t*)OS_MALLOC(sizeof(obsd_t) * MAXOBS);
    if (NULL == rtcm3data.obs[i].data )
    {
      return 0;
    }
    memset(rtcm3data.obs[i].data, 0, sizeof(obsd_t) * MAXOBS);
    rtcm3data.obs[i].nmax = MAXOBS;
    rtcm3data.obs[i].n = 0;
  }

  rtcm3data.is_cur_obs_coor_consist = 1;
  rtkobs.data = (obsd_t*)OS_MALLOC(sizeof(obsd_t) * MAXOBS * 2);
  if (NULL == rtkobs.data)
  {
    return 0;
  }
  memset(rtkobs.data, 0, sizeof(obsd_t) * MAXOBS * 2);
  rtkobs.nmax = MAXOBS * 2;
  rtkobs.n = 0;

  rtkobspre.data = (obsd_t*)OS_MALLOC(sizeof(obsd_t) * MAXOBS);
  if (NULL == rtkobspre.data)
  {
    return 0;
  }
  memset(rtkobspre.data, 0, sizeof(obsd_t) * MAXOBS);
  rtkobspre.nmax = MAXOBS;
  rtkobspre.n = 0;

  /* load options file */
  prcopt = (prcopt_t*)OS_MALLOC(sizeof(prcopt_t));
  memset(prcopt, 0, sizeof(prcopt_t));
  resetsysopts_lite(prcopt);

  getsysopts_lite(prcopt);
  /* overwrite cfg items with outer configuration method. */
  gnss_rtk_outer_cfg(rtk_cfg, prcopt, NULL);

  /* rtk control init */
  ag_rtk_init(&rtkctrl, prcopt);

  Sys_Free(prcopt);
  rtkinitflag = 1;

  return 1;
}

/* free rtk server -------------------------------------------------------------
* free rtk server
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_free()
{
  int i = 0;

  if (!rtkinitflag)
  {
    return;
  }

  if (rtknav.eph) {
    Sys_Free(rtknav.eph);
    rtknav.eph = NULL;
  }
  //if (rtknav.geph) Sys_Free(rtknav.geph);   rtknav.geph=NULL;
  //if (rtknav.seph) Sys_Free(rtknav.seph);   rtknav.seph=NULL;

  for (i = 0; i < MAXOBSBUF; i++)
  {
    Sys_Free(rtcm3data.obs[i].data);
    rtcm3data.obs[i].data = NULL;
  }

  Sys_Free(rtkobs.data);
  rtkobs.data = NULL;
  Sys_Free(rtkobspre.data);
  rtkobspre.data = NULL;

  for (i = 1; i <= MAXSAT; i++)
  {
    if (rtkctrl.ssat[i - 1] != NULL)
    {
      Sys_Free(rtkctrl.ssat[i - 1]);
      rtkctrl.ssat[i - 1] = NULL;
    }
  }

  ag_rtk_free(&rtkctrl);
  rtkinitflag = 0;
}

/* gnss system convert -------------------------------------------------------------
* gnss system convert
* args   : prn==0:ignore prn
* return : none
* author : none
*-----------------------------------------------------------------------------*/
extern int gnss_rtk_sys_conv(int gnss, int prn, int mode2sys)
{
    if (mode2sys)
    {
        switch (gnss)
        {
        case GPS_MODE:
            if (prn==0) return SYS_GPS; //ignore prn
            else if (prn>=MIN_GPS_PRN&&prn<=MAX_GPS_PRN)   return SYS_GPS;
            else if (prn>=MIN_QZSS_PRN&&prn<=MAX_QZSS_PRN) return SYS_QZS;
            else if (prn>=MIN_WAAS_PRN&&prn<=MAX_WAAS_PRN) return SYS_SBS;
            else return SYS_NONE;
        case GLN_MODE: return SYS_GLO;
        case BDS_MODE: return SYS_CMP;
        case GAL_MODE: return SYS_GAL;
        default:       return SYS_NONE;
        }
    }
    else
    {
        switch (gnss)
        {
        case SYS_SBS: 
        case SYS_QZS: 
        case SYS_GPS: return GPS_MODE;
        case SYS_GLO: return GLN_MODE;
        case SYS_CMP: return BDS_MODE;
        case SYS_GAL: return GAL_MODE;
        default:      return GPS_MODE;
        }
    }
}
/* gnss_rtk_crossWeek_check -------------------------------------------------------------
* cross week check
* args   : none
* return : none
* author : 
*-----------------------------------------------------------------------------*/
void gnss_rtk_crossWeek_check(int sys, GNSS_TIME* pTime, eph_t *peph ,float  toe)
{
    uint8_t     gnssMode;
    double    dt = 0;

    if ((sys & (SYS_GPS|SYS_CMP|SYS_GAL)) == 0)
    {
        return;
    }
    gnssMode = GNSS_SYS2MODE(sys);
    if (pTime->rcvr_time[gnssMode] > 0 && toe <= 7200 && toe >= 0)
    {
        dt = toe - pTime->rcvr_time[gnssMode];
        if (dt < -SECS_IN_WEEK / 2.0)
        {
            dt += (double)SECS_IN_WEEK;
            if (dt < 7200)
            {
                peph->week += 1;
            }
        }
    }
}
/* convert eph -------------------------------------------------------------
* convert eph to nav_t
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_nav_conv(GNSS_TIME* pTime, int sys,int prn, void *praweph, void *prtkeph)
{
    eph_t   *peph=NULL;
    int     sat=0;
    float     toeTmp;
    
    if (!pTime||!praweph||!prtkeph)
    {
        return;
    }

    if ((sys==SYS_GPS||sys==SYS_QZS)&&(pTime->init?pTime->week[GPS_MODE]:pTime->week_save[GPS_MODE])==0)
    {
        GLOGI("gnss_rtk_nav_conv gps init %d week %d %d fail", pTime->init, pTime->week[GPS_MODE], pTime->week_save[GPS_MODE]);
        return;
    }

    if (sys==SYS_CMP&&(pTime->init?pTime->week[BDS_MODE]:pTime->week_save[BDS_MODE])==0)
    {
        GLOGI("gnss_rtk_nav_conv cmp init %d week %d %d fail", pTime->init, pTime->week[BDS_MODE], pTime->week_save[BDS_MODE]);
        return;
    }

    if (sys==SYS_GAL&&(pTime->init?pTime->week[GAL_MODE]:pTime->week_save[GAL_MODE])==0)
    {
        GLOGI("gnss_rtk_nav_conv gal init %d week %d %d fail", pTime->init, pTime->week[GAL_MODE], pTime->week_save[GAL_MODE]);
        return;
    }
    
    sat=satno(sys,prn);
    if (sat <= 0) return;

    switch (sys)
    {
    case SYS_GPS:
    case SYS_QZS:
        {
            GPS_EPH_INFO *praw=(GPS_EPH_INFO *)praweph;
            peph=(eph_t*)prtkeph;
            peph->sat=sat;
            peph->iode=praw->IODE;
            peph->iodc=praw->subframe1.IODC;
            peph->sva=praw->subframe1.udre;
            peph->svh=praw->subframe1.SV_health;
            peph->week=pTime->init?pTime->week[GPS_MODE]:pTime->week_save[GPS_MODE];
            peph->code=praw->subframe1.codeL2;
            peph->flag=praw->subframe1.L2Pdata;

            //cross week check	
            gnss_rtk_crossWeek_check(sys,pTime,peph,praw->t_oe);
            peph->toe= gnss_gpst2time(peph->week,praw->t_oe);  //abs timestamp
            peph->toc= gnss_gpst2time(peph->week,praw->subframe1.t_oc);
            peph->A=praw->sqrt_A*praw->sqrt_A;
            peph->e=praw->e;
            peph->i0=praw->i_0;
            peph->OMG0=praw->OMEGA_0;
            peph->omg=praw->omega;
            peph->M0=praw->M_0;
            peph->deln=praw->delta_n;
            peph->OMGd=praw->OMEGADOT;
            peph->idot=praw->IDOT;
            peph->crc=praw->C_rc;
            peph->crs=praw->C_rs;
            peph->cuc=praw->C_uc;
            peph->cus=praw->C_us;
            peph->cic=praw->C_ic;
            peph->cis=praw->C_is;
            peph->toes=praw->t_oe;  //real toe
            peph->fit=praw->fit_interval;
            peph->f0=praw->subframe1.a_f0;
            peph->f1=praw->subframe1.a_f1;
            peph->f2=praw->subframe1.a_f2;
            peph->tgd[0]=praw->subframe1.T_GD;
        }
        break;
    case SYS_GLO:
        {
            int N4=0,NT=0;
            GLN_EPH_INFO *praw=(GLN_EPH_INFO*)praweph;
            geph_t *pglo=(geph_t*)prtkeph;
            N4=pTime->init?pTime->N4:pTime->N4_save;
            if (praw->eph_source == EPH_SOURCE_AGNSS_L)
            {
                NT=praw->NT;
            }
            else
            {
                NT=pTime->init?pTime->NT:pTime->NT_save;
            }
            if (N4==0||NT==0)
            {
                return;
            }
            
            pglo->sat=sat;
            pglo->iode=((int)(praw->tb/900))&0x7F;
            pglo->frq=gnss_Sd_Nm_GetGlnChanN(prn);
            pglo->svh=(praw->Bn>>2)&0x1;
            
            pglo->toe=utc2gpst(gln2utc(NT,N4,praw->tb)); //gpst
            memcpy(pglo->pos,praw->r,sizeof(double)*3);
            memcpy(pglo->vel,praw->v,sizeof(double)*3);
            memcpy(pglo->acc,praw->a,sizeof(double)*3);
            pglo->taun=praw->taun_tb;
            pglo->gamn=praw->gaman_tb;
            pglo->dtaun=praw->delta_taun;
        }
        break;
    case SYS_CMP:
        {
            BDS_EPH_INFO *praw=(BDS_EPH_INFO *)praweph;
            peph=(eph_t*)prtkeph;
            peph->sat=sat;
            peph->iode=praw->IODE;
            peph->iodc=praw->IODC;
            peph->sva=praw->URAI;
            peph->svh=praw->SatH1;
            peph->week=pTime->init?pTime->week[BDS_MODE]:pTime->week_save[BDS_MODE];

            //cross week check
            toeTmp = (float)praw->toe;
            gnss_rtk_crossWeek_check(sys, pTime, peph, toeTmp);
            peph->toe=bdt2gpst(bdt2time(peph->week,(double)praw->toe));
            peph->toc=bdt2gpst(bdt2time(peph->week,(double)praw->toc));
            peph->A=praw->sqrta*praw->sqrta;
            peph->e=praw->ecc;
            peph->i0=praw->i0;
            peph->OMG0=praw->OMEGA_0;
            peph->omg=praw->w;
            peph->M0=praw->M0;
            peph->deln=praw->delta_n;
            peph->OMGd=praw->OMEGA_Dot;
            peph->idot=praw->idot;
            peph->crc=praw->crc;
            peph->crs=praw->crs;
            peph->cuc=praw->cuc;
            peph->cus=praw->cus;
            peph->cic=praw->cic;
            peph->cis=praw->cis;
            peph->toes=praw->toe;  //real toe
            peph->f0=praw->af0;
            peph->f1=praw->af1;
            peph->f2=praw->af2;
            peph->tgd[0]=praw->TGD1*1e-9;
            peph->tgd[1]=praw->TGD2*1e-9;
        }
        break;
    case SYS_GAL:
        {
            GAL_EPH_INFO *praw=(GAL_EPH_INFO *)praweph;
            peph=(eph_t*)prtkeph;
            peph->sat=sat;
            peph->iode=praw->IOD;
            peph->iodc=praw->IOD;
            peph->sva=praw->sisa;
            peph->svh=praw->svHealth;
            peph->week=pTime->init?pTime->week[GAL_MODE]:pTime->week_save[GAL_MODE];
            peph->code=praw->eph_source;
            peph->flag=praw->navtype;

            //cross week check	
            gnss_rtk_crossWeek_check(sys,pTime,peph,(float)praw->toe);
            peph->toe=gst2time(peph->week,praw->toe); //abs timestamp
            peph->toc=gst2time(peph->week,praw->toc);
            peph->A=praw->sqrta*praw->sqrta;
            peph->e=praw->ecc;
            peph->i0=praw->i0;
            peph->OMG0=praw->OMEGA_0;
            peph->omg=praw->w;
            peph->M0=praw->M0;
            peph->deln=praw->delta_n;
            peph->OMGd=praw->OMEGA_Dot;
            peph->idot=praw->idot;
            peph->crc=praw->crc;
            peph->crs=praw->crs;
            peph->cuc=praw->cuc;
            peph->cus=praw->cus;
            peph->cic=praw->cic;
            peph->cis=praw->cis;
            peph->toes=praw->toe;  //real toe
            //peph->fit=praw->fit_interval;
            peph->f0=praw->af0;
            peph->f1=praw->af1;
            peph->f2=praw->af2;
            peph->tgd[0]=praw->bgd_e5a;
            peph->tgd[1]=praw->bgd_e5b;
        }
        break;
    default:
        break;
    }
    
}

/* update carrier length -------------------------------------------------------------
* update carrier length
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
static void gnss_rtk_ud_wavelen(nav_t *nav)
{
    int i,j;
    for (i=0;i<MAXSAT;i++) for (j=0;j<NFREQ;j++) {
        nav->lam[i][j]=satwavelen(i+1,j,nav);
    }
}
/* update nav info -------------------------------------------------------------
* update nav info
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_nav_update(int gnssmode,int prn, void *pnew, void *pbak)
{
    int sat=0,sys=0,j;
    void *rtknew=NULL, *rtkbak=NULL;
    GNSS_TIME* pTime=NULL;

    if (!rtkinitflag)
    {
        return;
    }
    pTime = gnss_tm_get_time();
    if (!pTime) return;

    sys=GNSS_MODE2SYS(gnssmode,prn);
    sat=satno(sys,prn);
    if (!sat) return;

    if (sys&(SYS_GPS|SYS_CMP|SYS_GAL|SYS_QZS))
    {
        rtknew=rtknav.eph+sat-1;
        rtkbak=rtknav.eph+sat-1+MAXSAT;
    }
    else if (sys==SYS_GLO)
    {
        //rtknew=rtknav.geph+prn-1;
        //rtkbak=rtknav.geph+prn-1+MAXPRNGLO;
    }
    else
    {
        return;
    }
    gnss_rtk_nav_conv(pTime,sys,prn, pnew, rtknew);
    gnss_rtk_nav_conv(pTime,sys,prn, pbak, rtkbak);

    //gnss_rtk_ud_wavelen(&rtknav);
    //only update glonass wavelength
    if (sys==SYS_GLO)
    {
        for (j=0;j<NFREQ;j++)
        {
            rtknav.lam[sat - 1][j] = satwavelen(sat, j, &rtknav);
        }
    }
}
/* convert obs -------------------------------------------------------------
* convert obs
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
extern int gnss_rtk_obs_conv(GNSS_TIME* pTime,gnss_meas_t *pmeas,obsd_t *pobs,int rcv)
{
    sat_data_t*    sp;

    if (!pTime||!pmeas||!pobs||pTime->init==FALSE)
    {
        return 0;
    }
    if ((pmeas->status&0xF)==0) //marked invalid meas in PE 
    {
        return 0;
    }
    memset(pobs,0,sizeof(obsd_t));
#ifdef USED_IN_MC262M
    if (g_pe_cfg.chipType == MTK)
    {
        pobs->time=gpst2time(pTime->week[GPS_MODE],(double)pmeas->received_sv_time_in_ns/1.0E9);  //pTime->GPSTime
    }
    else
#endif
    {
        pobs->time= gnss_gpst2time(pTime->week[GPS_MODE],pTime->rcvr_time[GPS_MODE]);  //pTime->GPSTime
    }
    
    pobs->sys = GNSS_MODE2SYS(pmeas->gnssMode, pmeas->prn);
    pobs->prn = pmeas->prn;
    pobs->sat=satno(pobs->sys, pobs->prn);
    if (pobs->sat==0) {
        return 0;
    }
    pobs->rcv=rcv;
    pobs->SNR[0]=(unsigned char)(pmeas->cno*4); //unit:0.25 dBHz
    pobs->LLI[0]=0;  //pmeas->cycleSlipCount>pmeas->lastCycleSlipCount&&pmeas->lastCycleSlipCount>0
    if ((g_pe_cfg.chipType == UBLOX || g_pe_cfg.chipType == QCOM) && pmeas->cycleSlipCount<=3)
    {
        pobs->LLI[0]=pmeas->cycleSlipCount;
    }
    else if (g_pe_cfg.chipType == MTK && pmeas->cycleSlipCount>0)
    {
        pobs->LLI[0]=1;  //mark as slip
    }
    pobs->code[0]=obs2code((pmeas->gnssMode==BDS_MODE)?"1I":"1C",NULL);
    pobs->L[0]=pmeas->carrierPhase;
    pobs->P[0]=pmeas->pseudoRange_raw>0.0? pmeas->pseudoRange_raw:0.0;   //pmeas->pseudoRange;
    
    if (fabs(pmeas->pseudoRangeRate_raw) > 1e-4)
    {
        if (pmeas->gnssMode == GPS_MODE)
        {
            pobs->D[0]= (float)(-pmeas->pseudoRangeRate_raw/GPS_L1_WAVELENGTH);
        }
        else if (pmeas->gnssMode == BDS_MODE)
        {
            pobs->D[0]= (float)(-pmeas->pseudoRangeRate_raw/BDS_L1_WAVELENGTH);
        }
        else if (pmeas->gnssMode == GLN_MODE)
        {
            pobs->D[0]= (float)(-pmeas->pseudoRangeRate_raw/GLN_L1_WAVELENGTH(gnss_Sd_Nm_GetGlnChanN(pmeas->prn)));
        }
        else
        {
            pobs->D[0] = 0.0;
        }
    }
    else
    {
        pobs->D[0]=0.0;
    }

    //get more pvt info
    if (rtkctrl.ssat[pobs->sat - 1] == NULL)
    {
        GLOGI("obs convert failed: sat=%d", pobs->sat);
        return 0;
    }
    else
    {
        sp = gnss_sd_get_sv_data(pmeas->gnssMode, pmeas->prn, pmeas->freq_index);

        rtkctrl.ssat[pobs->sat-1]->usectrl.rqflag[0]=pmeas->status;
        rtkctrl.ssat[pobs->sat-1]->usectrl.pr_diff = pmeas->post_prdiff==0.0?pmeas->pr_diff:pmeas->post_prdiff;
        rtkctrl.ssat[pobs->sat-1]->usectrl.dr_diff = pmeas->post_drdiff==0.0?pmeas->dr_diff:pmeas->post_drdiff;
        rtkctrl.ssat[pobs->sat-1]->usectrl.pr_qoschck[0] = pmeas->prWeightChck;
        rtkctrl.ssat[pobs->sat-1]->usectrl.dr_qoschck[0] = pmeas->drWeightChck;
        if (sp != NULL)
        {
            rtkctrl.ssat[pobs->sat - 1]->azel[0] = (double)sp->d_cos.az;
            rtkctrl.ssat[pobs->sat - 1]->azel[1] = (double)sp->d_cos.elev;
        }
        rtkctrl.ssat[pobs->sat-1]->snr[0]=pobs->SNR[0];
        rtkctrl.ssat[pobs->sat-1]->vs=pmeas->status&PE_MEAS_VALID_PR;
    }

    return 1;
}
/* clean obsl1 buffer -------------------------------------------------------------
* clean obsl1 buffer
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
#if 0
extern void gnss_rtk_obsl1_clean(hod_l1 *phod, int rcv, int rsvnum)
{
    int i;
    obs_l1 obsl1={{0}};
    phod->obsn[rcv-1]=rsvnum;
    for (i=rsvnum;i<MAXOBSBUF;i++)
    {
        phod->obs[rcv-1][i]=obsl1;
    }
    phod->obs[rcv-1][0].L[4]= 0.0;
    phod->obs[rcv-1][0].L[5]= 0.0;

    for (i = 1; i < MAXOBSBUF; i++)
    {
        phod->obs[rcv-1][i].L[4]= 0.0;
    }

}
#endif
/* save obs info -------------------------------------------------------------
* save obs info
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
#if 0
extern void gnss_rtk_obsl1_save(int f, obsd_t *pobs, int cycslipcnt, int rcv)
{
    ssat_t *psat=NULL;
    obs_l1 *pobsl1=NULL;
    hod_l1 *phod = NULL;
    int i,j,reverse=IS_PPK_REVERSE;
    double dt=1.0;
    if (!pobs||pobs->rcv!=rcv||f>=NFREQ)
    {
        return;
    }
    psat=rtkctrl.ssat[pobs->sat-1];
    if (psat == NULL)
    {
        return;
    }
    
    phod = &psat->hod[f];
    if (phod->obsn[rcv-1]>0&&
        ((!reverse&&(dt=timediff(pobs->time, phod->obs[rcv-1][0].time))<=0.0)||
        (  reverse&&(dt=timediff(pobs->time, phod->obs[rcv-1][0].time))>=0.0)))
    {
        return;
    }
    //if (fabs(pobs->L[f])<0.001)  //cp invalid
    //{
    //	gnss_rtk_obsl1_clean(phod,rcv,0);
    //	return;
    //}
    for (i=MAXOBSBUF-1;i>0;i--)
    {
        phod->obs[rcv-1][i]= phod->obs[rcv-1][i-1];
    }
    pobsl1=&phod->obs[rcv-1][0];
    pobsl1->time=pobs->time;
    pobsl1->cn0=pobs->SNR[f]/4;
    pobsl1->L[0]=pobs->L[f];
    pobsl1->P=pobs->P[f];
    pobsl1->D=pobs->D[f];
    if (rcv==1)
    {
        pobsl1->llicnt=cycslipcnt;
    }
    else
    {
        pobsl1->llicnt=pobs->LLI[f];
    }
    phod->obsn[rcv-1]++;

    if (pobs->L[f]==0.0) //cp invalid
    {
        return;
    }
    /* update high order diff */
    if (phod->obsn[rcv-1]==5)
    {
        for (i=1;i<5;i++) //order
        {
            for (j=0;j<MAXOBSBUF-i;j++) //buffer
            {
                phod->obs[rcv-1][j].L[i]= phod->obs[rcv-1][j].L[i-1]- phod->obs[rcv-1][j+1].L[i-1];
                if (i==1)
                {
                    dt=timediff(phod->obs[rcv-1][j].time, phod->obs[rcv-1][j+1].time);
                    phod->obs[rcv-1][j].L[i]/=dt; //cycle/s
                }
                
            }
        }

    }
    else if (phod->obsn[rcv-1]>5)
    {
        for (i=1;i<=5;i++)
        {
            phod->obs[rcv-1][0].L[i]= phod->obs[rcv-1][0].L[i-1]- phod->obs[rcv-1][1].L[i-1];
            if (i==1)
            {
                dt=timediff(phod->obs[rcv-1][0].time, phod->obs[rcv-1][1].time);
                phod->obs[rcv-1][0].L[i]/=dt; //cycle/s
            }
            
        }
    }

}
#endif
/* update F mat -------------------------------------------------------------
* 
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_InitF(double T)
{
    uint8_t          i, reverse=IS_PPK_REVERSE, recal=0;
    double         h1, h2, h3,a;

    a = 0.2;
    if (reverse) a = -a;
#ifndef RTK_PVMODE
    /* initialize A matrix */
    if (!reverse)
    {
        if (fabs(T-1.000)<0.001)
        {
            h1 = h1_1s;
            h2 = h2_1s;
            h3 = h3_1s;
        }
        else if (fabs(T-0.200)<0.001)
        {
            h1 = h1_200ms;
            h2 = h2_200ms;
            h3 = h3_200ms;
        }
        else if (fabs(T-0.100)<0.001)
        {
            h1 = h1_100ms;
            h2 = h2_100ms;
            h3 = h3_100ms;
        }
        else
        {
            recal = 1;
        }
    }
    else //reverse
    {
        if (fabs(T+1.000)<0.001)
        {
            h1 = h1_1s_revs;
            h2 = h2_1s_revs;
            h3 = h3_1s_revs;
        }
        else if (fabs(T+0.200)<0.001)
        {
            h1 = h1_200ms_revs;
            h2 = h2_200ms_revs;
            h3 = h3_200ms_revs;
        }
        else if (fabs(T+0.100)<0.001)
        {
            h1 = h1_100ms_revs;
            h2 = h2_100ms_revs;
            h3 = h3_100ms_revs;
        }
        else
        {
            recal = 1;
        }
    }

    if (recal)
    {
        h1 = 1 / (a*a) * (-1 + a * T + exp(-a * T));
        h2 = 1 / a * (1 - exp(-a * T));
        h3 = exp(-a * T);
    }
#else
    h1=h2=h3=0.0;
#endif
    
    for (i = 0; i < 3; i++)
    {
        rtkctrl.F[i+(i+3)*NX_PVA]=T;
        rtkctrl.F[i+(i+6)*NX_PVA]=h1;
        rtkctrl.F[(i+3)+(i+6)*NX_PVA]=h2;
        rtkctrl.F[(i+6)+(i+6)*NX_PVA]=h3;
    }

}
static void gnss_rtk_QmatSet(double Tin, double* q)
{
    double        a = 0.2;
    double        T = Tin;
    uint8_t         reverse = IS_PPK_REVERSE, recal = 0;

    Tin = fabs(T);
    if (reverse)   a = -a;
    if (fabs(Tin-1.000)<0.001)
    {
        q[0] = q11_1s;
        q[1] = q12_1s;
        q[2] = q13_1s;
        q[3] = q22_1s;
        q[4] = q23_1s;
        q[5] = q33_1s;
        if (reverse)
        {
            q[1] = -q[1];
            q[4] = -q[4];
        }
    }
    else if (fabs(Tin-0.200)<0.001)
    {
        q[0] = q11_200ms;
        q[1] = q12_200ms;
        q[2] = q13_200ms;
        q[3] = q22_200ms;
        q[4] = q23_200ms;
        q[5] = q33_200ms;
        if (reverse)
        {
            q[1] = -q[1];
            q[4] = -q[4];
        }
    }
    else if (fabs(Tin-0.100)<0.001)
    {
        q[0] = q11_100ms;
        q[1] = q12_100ms;
        q[2] = q13_100ms;
        q[3] = q22_100ms;
        q[4] = q23_100ms;
        q[5] = q33_100ms;
        if (reverse)
        {
            q[1] = -q[1];
            q[4] = -q[4];
        }
    }
    else
    {
        q[0] = 1 / pow(a,4) * (1 - exp(-2*a*T) + 2 * a * T + 2 * pow(a,3)*pow(T,3) / 3 - 2 * pow(a,2)*pow(T,2) - 4 * a * T * exp(- a * T));
        q[1] = 1 / pow(a,3) * (exp(-2*a*T) + 1 - 2 * exp(-a * T) + 2 * a * T * exp(-a * T) - 2 * a * T + pow(a,2) * pow(T,2));
        q[2] = 1 / pow(a,2) * (1 - exp(-2 * a * T) - 2 * a * T * exp(-a * T));
        q[3] = 1 / pow(a,2) * (4 * exp(-a*T) - 3 - exp(-2*a*T) + 2*a*T);
        q[4] = 1 / pow(a,1) * (exp(-2 * a * T) + 1 - 2 * exp(-a * T));
        q[5] = (1 - exp(-2*a*T));
    }
}
/* update Q mat -------------------------------------------------------------
* 
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_InitQ(double Tin,float* accSigmaInit)
{
    uint8_t             i;
    double            q11, q12, q13, q22, q23, q33;
    float            accSigma = 1;
    double            q[6]={0.0};

    if (accSigmaInit)
    {
        accSigma = *accSigmaInit;
    }

#ifndef RTK_PVMODE
    gnss_rtk_QmatSet(Tin,&q[0]);
    q11 = q[0] * accSigma;
    q12 = q[1] * accSigma;
    q13 = q[2] * accSigma;
    q22 = q[3] * accSigma;
    q23 = q[4] * accSigma;
    q33 = q[5] * accSigma;
#else
    Tin = fabs(Tin);
    if (gpz_kf_data->Qv==0.0)
    {
        gpz_kf_data->Qv=1.0;   //Qv is not init yet
    }
    if (fabs(Tin-1.000)<0.001)
    {
        q11 = gpz_kf_data->Qp + 0.3 * gpz_kf_data->Qv;
        q12 = 0.5 * gpz_kf_data->Qv;
        q22 = 1.0 * gpz_kf_data->Qv;
    }
    else
    {
        q11 = gpz_kf_data->Qp*Tin + gpz_kf_data->Qv*Tin*Tin*Tin/3.0;
        q12 = gpz_kf_data->Qv*Tin*Tin/2.0;
        q22 = gpz_kf_data->Qv*Tin;
    }
    if (IS_PPK_REVERSE) q12 = -q12;
    q13 = 0.0;
    q23 = 0.0;
    q33 = 0.0;
#endif
    
    for (i = 0; i < 3; i++)
    {
        rtkctrl.Q[i+i*NX_PVA]=q11;
        rtkctrl.Q[(i+3)+(i+3)*NX_PVA]=q22;
        rtkctrl.Q[(i)+(i+3)*NX_PVA]=q12;
        rtkctrl.Q[(i+3)+(i)*NX_PVA]=q12;

        rtkctrl.Q[(i+6)+(i+6)*NX_PVA]=q33;
        rtkctrl.Q[(i)+(i+6)*NX_PVA]=q13;
        rtkctrl.Q[(i+6)+(i)*NX_PVA]=q13;
        rtkctrl.Q[(i+3)+(i+6)*NX_PVA]=q23;
        rtkctrl.Q[(i+6)+(i+3)*NX_PVA]=q23;
    }
    //phase bias

    if (DBL_ISNT_EQUAL(rtkctrl.Q[i + i * NX_PVA], 0.0) && rtkctrl.trstCloCnt > 0)
    {
        rtkctrl.Qratio = (gpz_kf_data->Qp + 0.3 * gpz_kf_data->Qv) / rtkctrl.Q[0 + 0 * NX_PVA];
        rtkctrl.Qratio = rtkctrl.Qratio > 1.0 ? rtkctrl.Qratio : 1.0;
        rtkctrl.Qratio = rtkctrl.Qratio < 50.0 ? rtkctrl.Qratio : 50.0;
    }

    GLOGI("RTK Q:%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f, Qv=%.3f, tt=%.6f, revs=%d, Qratio=%.3f", q11, q12, q13, q22, q23, q33, gpz_kf_data->Qv, Tin, IS_PPK_REVERSE, rtkctrl.Qratio);
    
}
/* abnormal meas check -------------------------------------------------------------
*
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
extern int gnss_rtk_abnormal_meas_check(meas_blk_t* pMeas)
{
    uint32_t i;
    int gnssmode, llicnt=0, prErrCnt=0;

    for (i=0;i<pMeas->measCnt;i++)
    {
        gnssmode=pMeas->meas[i].gnssMode;
        if (gnssmode<GPS_MODE||gnssmode>=GNSS_MAX_MODE) continue;

        if (pMeas->meas[i].cycleSlipCount) llicnt++;

        if (pMeas->meas[i].pseudoRange_raw > 1.0E9 || pMeas->meas[i].pseudoRange_raw < 0.0) prErrCnt++;
    }
    if (prErrCnt > pMeas->measCnt*0.8 && llicnt > pMeas->measCnt*0.8)
    {
        return 1;
    }
    return 0;
}
extern int check_cur_vrs_valid()
{
    gtime_t time;
    uint8_t gnss_mode = 0;
    int valid = 0, i = 0, coor_consist = 1, coor_valid = 1, j = 0, isys = 0, num = 0;
    int comm_sat_num = 0, no_slip_num = 0;
    double delta_vrs_coor[3] = { 0.0 }, dist = 0.0, rcv_std = 0.0;
    double dt_clk = 0.0, dopp_corr = 0.0, tot = 0.0;
    sat_data_t* sp = NULL;
    ECEF svp;
    double diff[N_GPS_SVS] = { 0.0 }, mean = 0.0, earth_rot = 0.0;
    for (i = 0; i < 3; ++i)
    {
        if (fabs(rtcm3data.rb[0][i]) < 1.0e-2 || fabs(rtcm3data.rb[1][i]) < 1.0e-2)
        {
            coor_valid = 0;
            break;
        }
        if (fabs(rtcm3data.rb[0][i] - rtcm3data.rb[1][i]) > 1.0e-2)
        {
            coor_consist = 0;
            break;
        }
    }
    if (0 == coor_consist || 0 == coor_valid)
    {
        return 1;
    }
    for (isys = 0; isys < GNSS_MAX_MODE; ++isys)
    {
        if (GLN_MODE == isys)
        {
            continue;
        }
        num = 0;
        for (i = 0; i < rtcm3data.obs[0].n; ++i)
        {
            gnss_mode = GNSS_SYS2MODE(rtcm3data.obs[0].data[i].sys);
            if (gnss_mode != isys)
            {
                continue;
            }
            if (fabs(rtcm3data.obs[0].data[i].P[0]) < 1.0e-3)
            {
                continue;
            }
            /* transmission time by satellite clock */
            time = timeadd(rtcm3data.obs[0].data[i].time, -rtcm3data.obs[0].data[i].P[0] / CLIGHT);

            switch (gnss_mode) {
            case GPS_MODE: {tot = (time.time - SECS_IN_DAY * 3) % SECS_IN_WEEK + time.sec; break; }
            case BDS_MODE: {tot = (time.time - SECS_IN_DAY * 3 - 14) % SECS_IN_WEEK + time.sec; break; }
            case GAL_MODE: {tot = (time.time - SECS_IN_DAY * 3) % SECS_IN_WEEK + time.sec; break; }
            default: continue;
            }
            if ((sp = gnss_sd_get_sv_data(gnss_mode, rtcm3data.obs[0].data[i].prn, 0)) == NULL)
            {
                continue;
            }
            if (num >= N_GPS_SVS)
            {
                break;
            }
            dt_clk = gnss_Sd_Clk(gnss_mode, rtcm3data.obs[0].data[i].prn, tot, &dopp_corr, 0);
            tot -= dt_clk;
            dt_clk = gnss_Sd_Pos_e(gnss_mode, rtcm3data.obs[0].data[i].prn, tot, &svp, &dopp_corr, 0);
            dist = 0.0;
            for (j = 0; j < 3; ++j)
            {
                dist += (svp.p[j] - rtcm3data.rb[0][j]) * (svp.p[j] - rtcm3data.rb[0][j]);
            }
            dist = sqrt(dist);
            earth_rot = (svp.p[0]* rtcm3data.rb[0][1] - svp.p[1] * rtcm3data.rb[0][0]) * OMGE / CLIGHT;
            diff[num++] = rtcm3data.obs[0].data[i].P[0] - (dist + earth_rot - dt_clk * CLIGHT);
        }
        if (num < 4)
        {
            continue;
        }
        mean = 0.0;
        if (FALSE == gnss_median_dbl(diff, num, &mean))
        {
            continue;
        }
        rcv_std = 0.0;
        for (j = 0; j < num; ++j)
        {
            rcv_std += (diff[j]-mean)* (diff[j] - mean);
        }
        rcv_std = sqrt(rcv_std / (num - 1.0));
        if (rcv_std < 20.0)
        {
            valid = 1;
            break;
        }
    }
    return valid;
}
/* update obs info -------------------------------------------------------------
* update obs info
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
extern int gnss_rtk_obs_update(int adapt_seq_EKF, 
  GNSS_TIME* pTime, meas_blk_t* pMeas, asg_obs_t* asg_obs)
{
    int i,j,nu=0,gnssmode,reverse=IS_PPK_REVERSE/*,firstepoch = 0*/;
    obsd_t* pobs=NULL;
    double tt;

    int sat = 0; 
    int sys = 0; 
    int sat_list[MAXSAT] = { 0 };
    sat_data_t* sp = NULL;
    
    rtkobs.n = 0;
    rtkctrl.cp_pr_cnt = 0;
    if (rtkctrl.ptor==0.0 && DBL_IS_EQUAL(pTime->dt, 0.0))
    {
        rtkctrl.ptor=pMeas->tor;
        rtkctrl.kf_tor = pMeas->tor;
        GLOGI("first epoch! current tor=%.6f, reverse=%d",pMeas->tor,reverse);
        //firstepoch = 1;
        return 0;
    }

    //rover
    for (i = 0; i < MAX_MEAS_NUM*AGGNSS_MAX_FREQ_NUM; i++)
    {
        sp = Sd_data.sat_data[i];
        if (sp == NULL)
        {
            continue;
        }
        
        sys = GNSS_MODE2SYS(sp->gnssMode, sp->prn);
        sat = satno(sys, sp->prn);
        if (0 == sat) {
            continue;
        }
        sat_list[sat - 1] = sat;

        if (rtkctrl.ssat[sat - 1] == NULL)
        {
            rtkctrl.ssat[sat - 1] = (ssat_t*)Sys_Malloc(sizeof(ssat_t));
            if (rtkctrl.ssat[sat - 1] == NULL)
            {
                GLOGI("Memory alloc Fail %s %d", __FUNCTION__, __LINE__);
                continue;
            }
            memset(rtkctrl.ssat[sat - 1], 0, sizeof(ssat_t));
        }
    }

    //reference
    for (i = 0; i < rtcm3data.obs[0].n&&rtkobs.n < MAXOBS * 2; i++)
    {
        sat = satno(rtcm3data.obs[0].data[i].sys, rtcm3data.obs[0].data[i].prn);
        if (0 == sat) {
            continue;
        }
        sat_list[sat - 1] = sat;

        if (rtkctrl.ssat[sat - 1] == NULL)
        {
            rtkctrl.ssat[sat - 1] = (ssat_t*)Sys_Malloc(sizeof(ssat_t));
            if (rtkctrl.ssat[sat - 1] == NULL)
            {
                GLOGI("Memory alloc Fail %s %d", __FUNCTION__, __LINE__);
                continue;
            }
            memset(rtkctrl.ssat[sat - 1], 0, sizeof(ssat_t));
        }
    }

    rtkctrl.cp_pr_cnt = 0;
    for (i = 1; i <= MAXSAT; i++)
    {
        if ((sat_list[i - 1] == 0) && (rtkctrl.ssat[i - 1] != NULL))
        {
            Sys_Free(rtkctrl.ssat[i - 1]);
            rtkctrl.ssat[i - 1] = NULL;
        }

        if (rtkctrl.ssat[i - 1] != NULL)
        {
            rtkctrl.ssat[i-1]->present[0]=0;
            rtkctrl.ssat[i-1]->present[1]=0;
            rtkctrl.ssat[i-1]->vs=0;
            for (j = 0;j < NFREQ;j++) 
            {
                rtkctrl.ssat[i - 1]->snr[j] = 0;
                rtkctrl.ssat[i - 1]->tdcheck[j] = 0;
#ifdef USED_IN_MC262M
                rtkctrl.ssat[i - 1]->pr_std[j] = 0.0;
                rtkctrl.ssat[i - 1]->cp_std[j] = 0.0;
#endif
                rtkctrl.ssat[i - 1]->usectrl.rqflag[j] = 0;
            }
            rtkctrl.ssat[i-1]->usectrl.flag=0;
            rtkctrl.ssat[i-1]->usectrl.canberef<<=1;
            rtkctrl.ssat[i-1]->usectrl.fix_rejflag=0;
        }
    }
    
    if (pMeas->tor==0.0&&pMeas->measCnt==0)
    {
        GLOGI("no measurements, no time");
        return 0;
    }
    //tt=pMeas->tor-rtkctrl.ptor;
    tt = pMeas->tor - rtkctrl.kf_tor;

    if (!reverse) //forward
    {
        if (tt<=0.0)
        {
            if (g_pe_cfg.chipType==UBLOX&&tt<-302400.0&&pTime->dt>0.0)  //cross week
            {
                GLOGI("tor cross week: tor=%.6f,ptor=%.6f,reverse=%d", pMeas->tor, rtkctrl.ptor,reverse);
                tt = pTime->dt;
            }
            else
            {
                GLOGW("tor abnormal! tor=%.6f,ptor=%.6f,reverse=%d", pMeas->tor, rtkctrl.ptor,reverse);
                if (g_pe_cfg.chipType==UBLOX && gnss_rtk_abnormal_meas_check(pMeas))
                {
                    rtkctrl.kfstatus |= RTK_KF_RESET_AMB;
                    gnss_rtk_clean_prefix(&rtkctrl);
                    GLOGW("abnormal meas detected, to reset amb");
                }
                return 0; //tor error
            }
        }
        if (tt>5.0)
        {
            GLOGW("time outage! tt=%.3f,tor=%.6f,ptor=%.6f,reverse=%d",tt,pMeas->tor,rtkctrl.ptor,reverse);
            if (pTime->dt>=0.0)  //normal outage like tunnel
            {
                rtkctrl.ptor = pMeas->tor;
                rtkctrl.kf_tor = pMeas->tor;
            }
            rtkctrl.kfstatus=RTK_KF_RESET; //RTK_KF_RESTART
            return 0;
        }

    }
    else //reverse
    {
        if (tt>=0.0)
        {
            if (g_pe_cfg.chipType==UBLOX&&tt>302400.0&&pTime->dt<0.0)  //cross week
            {
                GLOGI("tor cross week: tor=%.6f,ptor=%.6f,reverse=%d", pMeas->tor, rtkctrl.ptor,reverse);
                tt = pTime->dt;
            }
            else
            {
                GLOGW("tor abnormal! tor=%.6f,ptor=%.6f,reverse=%d", pMeas->tor, rtkctrl.ptor,reverse);
                return 0; //tor error
            }
        }
        if (tt<-5.0)
        {
            GLOGW("time outage! tt=%.3f,tor=%.6f,ptor=%.6f,reverse=%d",tt,pMeas->tor,rtkctrl.ptor,reverse);
            if (pTime->dt<=0.0)  //normal outage like tunnel
            {
                rtkctrl.ptor = pMeas->tor;
                rtkctrl.kf_tor = pMeas->tor;
            }
            rtkctrl.kfstatus=RTK_KF_RESET; //RTK_KF_RESTART
            return 0;
        }
    }
    
    if (fabs(tt-rtkctrl.tt)>=0.05)
    {
        rtkctrl.updFQtt = tt;
    }
    rtkctrl.ptor=pMeas->tor;
    rtkctrl.tt=tt;

    if ((rtkctrl.kfstatus & RTK_KF_RUN) == 0)
    {
        rtkctrl.kf_tor = pMeas->tor;
    }

Lab_RtkObs_Updata:
    for (gnssmode=GPS_MODE;gnssmode<GNSS_MAX_MODE;gnssmode++)
    {
        rtkctrl.satcnt[gnssmode]=0;
        rtkctrl.llisatcnt[gnssmode]=0;
        rtkctrl.pvtsatcnt[gnssmode] = 0;
    }
    rtkctrl.noSkipCnt = 0;

    gnss_rtk_asgobsd2rtkobs(asg_obs, pMeas, pTime, &rtkobs);
    
        
#if 0	
        //user
    #if AGGNSS_MAX_FREQ_NUM == 1
        for (i=0;i<pMeas->measCnt;i++)
        {
            gnssmode=pMeas->meas[i].gnssMode;
            if (gnssmode<GPS_MODE||gnssmode>=GNSS_MAX_MODE)
            {
                continue;
            }
            rtkctrl.satcnt[gnssmode]++;
            if (pMeas->meas[i].cycleSlipCount)  /*|| pMeas->meas[i].carrierPhase==0.0*/
            {
                rtkctrl.llisatcnt[gnssmode]++;
            }
            if (!(GNSS_MODE2SYS(pMeas->meas[i].gnssMode,pMeas->meas[i].prn)&rtkctrl.opt.navsys))
            {
                continue;
            }
            if (rtkobs.n<(MAXOBS*2)&&gnss_rtk_obs_conv(pTime,&pMeas->meas[i],&rtkobs.data[rtkobs.n],1))
            {
                gnss_rtk_obsl1_save(0, &rtkobs.data[rtkobs.n], pMeas->meas[i].cycleSlipCount, 1);
                rtkobs.n++;
            }
        }
    #else
        if (g_pe_cfg.chipType == QCOM)
        {
            gnss_rtk_andr2obsd_qcom(pRawData, pMeas, &rtknav, pTime, &rtkobs);
        }
        else
        {
            gnss_rtk_andr2obsd(pRawData, pMeas, &rtknav, pTime, &rtkobs);
        }

    #endif

#endif
    
    sortobs(&rtkobs);
    nu=rtkobs.n;
#if 0
    if (fp_rnx_rov&&rtkobs.n>0)
    {
        outrnxobsb(fp_rnx_rov,&rnx_opt,rtkobs.data,rtkobs.n,0);
    }
#endif
    
    //reference
    if (0 == adapt_seq_EKF)
    {
        for (i = 0; i < rtcm3data.obs[0].n && rtkobs.n < MAXOBS * 2; i++)
        {
#if 0
            for (j = 0; j < AGGNSS_MAX_FREQ_NUM && j < NFREQ; j++)
            {
                gnss_rtk_obsl1_save(j, &rtcm3data.obs[0].data[i], 0, 2);
    }
#endif
            rtkobs.data[rtkobs.n++] = rtcm3data.obs[0].data[i];
        }
    }
    else
    {
        if (check_cur_vrs_valid())
        {
            rtcm3data.is_cur_obs_coor_consist = 1;
            for (i = 0; i < rtcm3data.obs[0].n && rtkobs.n < MAXOBS * 2; i++)
            {
                rtkobs.data[rtkobs.n++] = rtcm3data.obs[0].data[i];
            }
        }
        else if(MAXOBSBUF>1)
        {
            rtcm3data.is_cur_obs_coor_consist = 0;
            for (i = 0; i < rtcm3data.obs[1].n && rtkobs.n < MAXOBS * 2; i++)
            {
                rtkobs.data[rtkobs.n++] = rtcm3data.obs[1].data[i];
            }
        }
        else
        {
            rtcm3data.is_cur_obs_coor_consist = 0;
        }
    }
    
    //save present status
    for (i=0;i<rtkobs.n;i++)
    {
        pobs=&rtkobs.data[i];
        if ((pobs->rcv) < 1 || (pobs->sat) < 1)
        {
            continue;
        }
        if (rtkctrl.ssat[pobs->sat - 1] != NULL)
        {
            rtkctrl.ssat[pobs->sat - 1]->present[pobs->rcv - 1] = 1;
        }
    }
    
    GLOGW("rover count=%d, ref count=%d", nu, rtkobs.n - nu);
    
    return 1;
}
/* ref obs freq mask check -------------------------------------------------------------
*
* args   : none
* return : freq mask
* author : none
*-----------------------------------------------------------------------------*/
static uint8_t gnss_rtk_ref_freqmask(const obsd_t *obsd, int32_t obsn)
{
    uint8_t i, f, freqmask = 0;
    for (f = 0; f < NFREQ; f++)
    {
        for (i = 0; i < obsn; i++)
        {
            if (fabs(obsd[i].P[f]) > 1e-6 || fabs(obsd[i].L[f]) > 1e-6)
            {
                freqmask |= 1 << f;
                break;
            }
        }
    }

    return freqmask;
}
/* rtcm3 data process -------------------------------------------------------------
* rtcm3 data process
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_rtcm3_proc(rtcm_data_pack_t* rawData)
{
    int i,j,n=0,sat=0,staid_obs=0;
    double pos[3]={0.0},del[3]={0.0},dr[3]={0.0},*rbn=NULL;
    rtcm3_rtk_t *prawrtk=&rawData->rtcm3_data;

    if (!rtkinitflag)
    {
        return;
    }
    
    /* ref station coord */
    if (prawrtk->msg_mask&(RTCM3_MSG_1005|RTCM3_MSG_1006))
    {
        if (asg_norm(prawrtk->sta.pos,3)>VALIDPOSNORM)
        {
            rbn=&rtcm3data.rb[0][0];
            /* back up */
            for (i=0;i<3;i++) 
            {
                rtcm3data.rb[1][i]=rbn[i];
                rbn[i]=prawrtk->sta.pos[i];
            }
            /* antenna delta */
            ecef2pos(rbn,pos);
            if (prawrtk->sta.deltype) 
            {
                /* xyz */
                del[2]=prawrtk->sta.hgt;
                enu2ecef(pos,del,dr);
                for (i=0;i<3;i++) 
                {
                    rbn[i]+=prawrtk->sta.del[i]+dr[i];
                }
            }
            else 
            { 
                /* enu */
                enu2ecef(pos,prawrtk->sta.del,dr);
                for (i=0;i<3;i++) 
                {
                    rbn[i]+=dr[i];
                }
            }
            rtcm3data.info_mask|=PE_RTCM_INFO_REFPOS;
            rtcm3data.staid_rb[1]=rtcm3data.staid_rb[0];
            rtcm3data.staid_rb[0]=prawrtk->sta.antsetup&0xFFF;
            GLOGI("rtcm ref pos update: rb= %.4f %.4f %.4f, staid0=%d staid1=%d",rbn[0],rbn[1],rbn[2],rtcm3data.staid_rb[0],rtcm3data.staid_rb[1]);
        }
        else
        {
            GLOGW("%s: rtcm station postion invalid",__FUNCTION__);
        }	
    }

    /* glonass CPB info */
    if (prawrtk->msg_mask&RTCM3_MSG_1230)
    {
        //use phy_sta to store glonass cpb info for now
        rtcm3data.pe_glo_cpb.indicator = prawrtk->phy_sta.phyid & 0x1;
        rtcm3data.pe_glo_cpb.sigmask = (prawrtk->phy_sta.phyid >> 1) & 0xF;
        rtcm3data.pe_glo_cpb.cpb[0] = (float)prawrtk->phy_sta.rb[0];
        rtcm3data.pe_glo_cpb.cpb[1] = (float)prawrtk->phy_sta.rb[1];
        rtcm3data.pe_glo_cpb.cpb[2] = (float)prawrtk->phy_sta.rb[2];
        rtcm3data.pe_glo_cpb.cpb[3] = (float)(((int16_t)prawrtk->phy_sta.itrf) * 0.02);
        rtcm3data.info_mask |= PE_RTCM_INFO_GLNCPB;
        rtcm3data.glocpb_available = 1;

        GLOGI("rtcm ref glocpb update: ind=%u sigmask=%x [%.3f %.3f %.3f %.3f]", rtcm3data.pe_glo_cpb.indicator, rtcm3data.pe_glo_cpb.sigmask, 
            rtcm3data.pe_glo_cpb.cpb[0], rtcm3data.pe_glo_cpb.cpb[1], rtcm3data.pe_glo_cpb.cpb[2], rtcm3data.pe_glo_cpb.cpb[3]);
    }

    /* physical ref station coord */
    if (prawrtk->msg_mask&RTCM3_MSG_1032)
    {
        rtcm3data.phy_sta=prawrtk->phy_sta;
        rtcm3data.info_mask|=PE_RTCM_INFO_PHYSTA;
    }

    /* rcv and ant info */
    if (prawrtk->msg_mask&(RTCM3_MSG_1008|RTCM3_MSG_1033))
    {
        rtcm3data.sys_para=prawrtk->sys_para;
        rtcm3data.info_mask|=PE_RTCM_INFO_ANTINFO;
    }
    /* sys params */
    if (prawrtk->msg_mask&RTCM3_MSG_1013)
    {
        rtcm3data.info_mask|=PE_RTCM_INFO_SYSPARA;
    }
    /* obs data */
    if ((prawrtk->msg_mask&(RTCM3_MSG_1074|RTCM3_MSG_1084|RTCM3_MSG_1124|RTCM3_MSG_1094)) &&
        prawrtk->obsn>0)
    {
        for (i=MAXOBSBUF-1;i>0;i--)
        {
            //obs
            for (j=0;j<rtcm3data.obs[i].n;j++)
            {
                rtcm3data.obs[i].data[j]=rtcm3data.obs[i-1].data[j];
            }
            rtcm3data.obs[i].n=rtcm3data.obs[i-1].n;
            rtcm3data.obs[i].nmax=rtcm3data.obs[i-1].nmax;
            //obsflag
            rtcm3data.obsflag[i]=rtcm3data.obsflag[i-1];
        }
        //got the newest one
        rtcm3data.obsflag[0]=prawrtk->obsflag;
        n=0;
        for (i=0;i<prawrtk->obsn;i++) 
        {
            if (prawrtk->obsd[i].sys>0 && prawrtk->obsd[i].prn>0)
            {
                if ((sat = satno(prawrtk->obsd[i].sys, (prawrtk->obsd[i].prn)))>0)
                {
                    prawrtk->obsd[i].sat = sat;
                }
                else
                {
                    continue;
                }
            }
            else
            {
                prawrtk->obsd[i].sys=satsys(prawrtk->obsd[i].sat, &(prawrtk->obsd[i].prn));
            }
            if (rtkctrl.opt.exsats[prawrtk->obsd[i].sat-1]==1||
                !(prawrtk->obsd[i].sys&rtkctrl.opt.navsys)) continue;
            rtcm3data.obs[0].data[n]=prawrtk->obsd[i];
            rtcm3data.obs[0].data[n++].rcv=2;
        }
        rtcm3data.obs[0].n=n;
        sortobs(&rtcm3data.obs[0]);

        rtcm3data.info_mask|=PE_RTCM_INFO_REFOBS;
        staid_obs = rtcm3data.staid_obs;
        rtcm3data.staid_obs = (prawrtk->sta.antsetup >> 12)&0xFFF;
        rtcm3data.ref_freqmask = gnss_rtk_ref_freqmask(rtcm3data.obs[0].data, rtcm3data.obs[0].n);

        GLOGI("rtcm ref obs update: n=%d,staid0=%d staid1=%d",n,rtcm3data.staid_obs,staid_obs);

#if 0
        if (fp_rnx_ref&&rtcm3data.obs[0].n>0)
        {
            outrnxobsb(fp_rnx_ref,&rnx_opt,rtcm3data.obs[0].data,rtcm3data.obs[0].n,0);
        }
#endif
    }
    GLOGI("%s: message mask=0x%04x",__FUNCTION__,prawrtk->msg_mask);
}

extern void gnss_rtk_rtcm3_lite_proc(asg_rtcm_lite_t* rtcm_lite)
{
  int i, j, n = 0, sat = 0, staid_obs = 0;
  double pos[3] = { 0.0 }, del[3] = { 0.0 }, dr[3] = { 0.0 }, * rbn = NULL;
  int pos_valid = 1, cur_pre_coor_consist = 1;

  if (!rtkinitflag)
  {
    return;
  }

  if (rtcm3data.obs[0].n > 6 && rtcm_lite->obs.n <= 6)
  {
    GLOGW("rtcm sat too less");
    return;
  }

  //if (1)
  {
    if (asg_norm(rtcm_lite->sta.pos, 3) > VALIDPOSNORM)
    {
      rbn = &rtcm3data.rb[0][0];
      /* back up */
      for (i = 0; i < 3; i++)
      {
        rtcm3data.rb[1][i] = rbn[i];
        rbn[i] = rtcm_lite->sta.pos[i];
      }
      /* antenna delta */
      ecef2pos(rbn, pos);
      if (rtcm_lite->sta.deltype)
      {
        /* xyz */
        del[2] = rtcm_lite->sta.hgt;
        enu2ecef(pos, del, dr);
        for (i = 0; i < 3; i++)
        {
          rbn[i] += rtcm_lite->sta.del[i] + dr[i];
        }
      }
      else
      {
        /* enu */
        enu2ecef(pos, rtcm_lite->sta.del, dr);
        for (i = 0; i < 3; i++)
        {
          rbn[i] += dr[i];
        }
      }
      rtcm3data.info_mask |= PE_RTCM_INFO_REFPOS;
      rtcm3data.staid_rb[1] = rtcm3data.staid_rb[0];
      rtcm3data.staid_rb[0] = rtcm_lite->sta.antsetup & 0xFFF;
      GLOGI("rtcm ref pos update: rb= %.4f %.4f %.4f, staid0=%d staid1=%d", rbn[0], rbn[1], rbn[2], rtcm3data.staid_rb[0], rtcm3data.staid_rb[1]);
    }
    else
    {
      GLOGW("%s: rtcm station postion invalid", __FUNCTION__);
    }
  }
  for (i = 0; i < 3; ++i)
  {
    if (fabs(rtcm3data.rb[0][i]) < 1.0e-3 || fabs(rtcm3data.rb[1][i]) < 1.0e-3)
    {
      pos_valid = 0;
      break;
    }
  }
  if (pos_valid)
  {
    for (i = 0; i < 3; ++i)
    {
      if (fabs(rtcm3data.rb[0][i] - rtcm3data.rb[1][i]) > 1.0e-3)
      {
        cur_pre_coor_consist = 0;
        break;
      }
    }
  }
  if (rtcm3data.is_cur_obs_coor_consist
    || (0 == rtcm3data.is_cur_obs_coor_consist && 0 == cur_pre_coor_consist))
  {
    for (i = MAXOBSBUF - 1; i > 0; i--)
    {
      //obs
      for (j = 0; j < rtcm3data.obs[i].n; j++)
      {
        rtcm3data.obs[i].data[j] = rtcm3data.obs[i - 1].data[j];
      }
      rtcm3data.obs[i].n = rtcm3data.obs[i - 1].n;
      rtcm3data.obs[i].nmax = rtcm3data.obs[i - 1].nmax;
      //obsflag
      rtcm3data.obsflag[i] = rtcm3data.obsflag[i - 1];
    }
    //got the newest one
    rtcm3data.obsflag[0] = rtcm_lite->obsflag;
    n = 0;
    for (i = 0; i < rtcm_lite->obs.n; i++)
    {
      if (rtcm_lite->obs.data[i].sys > 0 && rtcm_lite->obs.data[i].prn > 0)
      {
        if (rtcm_lite->obs.data[i].sat > 0)
        {
          rtcm_lite->obs.data[i].sat = satno(rtcm_lite->obs.data[i].sys, rtcm_lite->obs.data[i].prn);
        }
        else
        {
          continue;
        }
      }
      else
      {
        rtcm_lite->obs.data[i].sys = satsys(rtcm_lite->obs.data[i].sat, &rtcm_lite->obs.data[i].prn);
      }

      if (rtkctrl.opt.exsats[rtcm_lite->obs.data[i].sat - 1] == 1 ||
        !(rtcm_lite->obs.data[i].sys & rtkctrl.opt.navsys)) {
        continue;
      }

      //if (n >= MAXOBS)
      if (n >= 35)
      {
        continue;
      }

      rtcm3data.obs[0].data[n].time.time = rtcm_lite->obs.data[i].time.time;
      rtcm3data.obs[0].data[n].time.sec = rtcm_lite->obs.data[i].time.sec;
      rtcm3data.obs[0].data[n].sat = rtcm_lite->obs.data[i].sat;
      rtcm3data.obs[0].data[n].sys = rtcm_lite->obs.data[i].sys;
      rtcm3data.obs[0].data[n].prn = rtcm_lite->obs.data[i].prn;

      for (int k = 0; k < NFREQ; k++)
      {
        rtcm3data.obs[0].data[n].SNR[k] = rtcm_lite->obs.data[i].SNR[k];
        rtcm3data.obs[0].data[n].LLI[k] = rtcm_lite->obs.data[i].LLI[k];
        rtcm3data.obs[0].data[n].code[k] = rtcm_lite->obs.data[i].code[k];
        rtcm3data.obs[0].data[n].L[k] = rtcm_lite->obs.data[i].L[k];
        rtcm3data.obs[0].data[n].P[k] = rtcm_lite->obs.data[i].P[k];
        rtcm3data.obs[0].data[n].D[k] = rtcm_lite->obs.data[i].D[k];
      }

      rtcm3data.obs[0].data[n++].rcv = 2;
    }
    rtcm3data.obs[0].n = n;
    sortobs(&rtcm3data.obs[0]);

    rtcm3data.info_mask |= PE_RTCM_INFO_REFOBS;
    staid_obs = rtcm3data.staid_obs;
    rtcm3data.staid_obs = (rtcm_lite->sta.antsetup >> 12) & 0xFFF;
    rtcm3data.ref_freqmask = gnss_rtk_ref_freqmask(rtcm3data.obs[0].data, rtcm3data.obs[0].n);

    GLOGI("rtcm ref obs update: n=%d,staid0=%d staid1=%d", n, rtcm3data.staid_obs, staid_obs);
#if 0
    if (fp_rnx_ref && rtcm3data.obs[0].n > 0)
    {
      outrnxobsb(fp_rnx_ref, &rnx_opt, rtcm3data.obs[0].data, rtcm3data.obs[0].n, 0);
    }
#endif
  }

  GLOGI("%s: message mask=0x%04x", __FUNCTION__, rtcm_lite->id_type);
}

/* rtk base sta data clean -------------------------------------------------------------
*
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_basedata_clean()
{
    int i;
    
    for (i=0;i<3;i++)
    {
        rtcm3data.rb[0][i] = 0.0;
        rtcm3data.rb[1][i] = 0.0;
    }

    for (i=0;i<MAXOBSBUF;i++)
    {
        rtcm3data.obs[i].n=0;
        rtcm3data.obsflag[i]=0;
    }

    rtcm3data.info_mask = 0;
}
/* rtk base position update -------------------------------------------------------------
* rtk base position update
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
extern int gnss_rtk_rb_update()
{
    int i,bool0=0,bool1=0,flag=0,ret=0;
    double bl[3]={0.0},prb[3]={0.0};
    double dt;

    if (rtkctrl.opt.refpos<=POSOPT_RINEX)
    {
        if (rtcm3data.base_switch)
        {
            rtkctrl.rtkevent|=RTK_BASECHANGE;
        }
        return 1;
    }

    dt = fabs(timediff(rtkobs.data[0].time, rtcm3data.obs[0].data[0].time));
    if (dt >= rtkctrl.opt.maxtdiff)
    {
        GLOGI("age abnormal!");
        return 1;
    }

    for (i=0;i<3;i++)  
    {
        rtkctrl.rbspp[i]=0.0;
        prb[i]=rtkctrl.rb[i];
    }
    /* newest ref obs not available */
    if (rtcm3data.obs[0].n<=0)
    {
        GLOGW("%s: refobs[0] has no obs!",__FUNCTION__);
        return 0;
    }
    bool0=asg_norm(rtcm3data.rb[0],3)>VALIDPOSNORM;
    bool1=asg_norm(rtcm3data.rb[1],3)>VALIDPOSNORM;
    if (bool0+bool1==0)
    {
        GLOGW("%s: have no valid ref pos!",__FUNCTION__);
        return 0;
    }
    if (bool0+bool1==1)
    {
        if (bool0)
        {
            flag=3; /* first ref pos may be wrong */
        }
        else
        {
            GLOGW("%s: ref pos abnormal,refpos1 valid but refpos0 invalid",__FUNCTION__);
            flag=1;
        }
    }
    else/* if (bool0+bool1==2)*/
    {
        /* if new and old are the same ref pos */
        for (i=0;i<3;i++)
        {
            bl[i]=rtcm3data.rb[0][i]-rtcm3data.rb[1][i];
        }
        if (asg_norm(bl,3)<DIFFPOSNORM)
        {
            flag=0;
        }
        else
        {
            flag=2;
        }
    }
    /* most common case */
    if (flag==0)
    {
        /* directly use new one */
        for (i=0;i<3;i++) 
        {
            rtkctrl.rb[i]=rtcm3data.rb[0][i];
        }
    }
    else
    {
        sol_t sol={{0}};
        char msg[128]={0};
        double norm0=0,norm1=0,*rb=NULL;
        double bl0[3]={0.0};

        for (i=0;i<3;i++)
        {
            sol.rr[i]=rtcm3data.rb[0][i];
        }
        /* get spp by ref obs */
        ret= ag_gnss_pntpos(rtcm3data.obs[0].data,rtcm3data.obs[0].n,&rtknav,
            &rtkctrl.opt,&sol,NULL,NULL,msg);
        if (!ret)
        {
            GLOGW("%s: spp by ref obs failed(%s)",__FUNCTION__,msg);
            return 0;
        }
        //save spp results
        for (i=0;i<3;i++) rtkctrl.rbspp[i]=sol.rr[i];
        
        for (i=0;i<3;i++)
        {
            bl0[i]=sol.rr[i]-rtcm3data.rb[0][i];
            bl[i] =sol.rr[i]-rtcm3data.rb[1][i];
        }
        norm0=asg_norm(bl0,3);
        norm1=asg_norm(bl,3);
        if (flag==1)
        {
            if (norm1>DIFFSPPREF)   //stored rb is expired
            {
                GLOGW("%s: the only valid refpos1 expired, norm1=%.1f",__FUNCTION__,norm1);
                /* TODO: use spp results directly? */
                return 0;
            }
            /* use the only valid old one */
            for (i=0;i<3;i++) 
            {
                rtkctrl.rb[i]=rtcm3data.rb[1][i];
            }
            GLOGW("%s: use the only valid old one, norm1=%.1f",__FUNCTION__,norm1);
        }
        else if (flag==3)
        {
            if (norm0>DIFFSPPREF)   //stored rb is expired
            {
                GLOGW("%s: the only valid refpos0 expired, norm0=%.1f",__FUNCTION__,norm0);
                /* TODO: use spp results directly? */
                return 0;
            }
            /* use the only valid new one */
            for (i=0;i<3;i++) 
            {
                rtkctrl.rb[i]=rtcm3data.rb[0][i];
            }
            GLOGW("%s: use the only valid new one, norm0=%.1f",__FUNCTION__,norm0);
        }
        else/* if (flag==2)*/
        {
            if ((norm0<DIFFSPPVRS&&norm1>DIFFSPPVRS)||(norm0<norm1&&norm0<DIFFSPPREF))
            {
                GLOGI("%s: use refpos0. norm0=%.1f,norm1=%.1f",__FUNCTION__,norm0,norm1);
                rb=&rtcm3data.rb[0][0];
            }
            else if ((norm1<DIFFSPPVRS&&norm0>DIFFSPPVRS)||(norm1<norm0&&norm1<DIFFSPPREF))
            {
                GLOGW("%s: use bak refpos1. norm0=%.1f,norm1=%.1f",__FUNCTION__,norm0,norm1);
                rb=&rtcm3data.rb[1][0];
            }
            else
            {
                GLOGW("%s: both refpos0 and refpos1 unmatch. norm0=%.1f,norm1=%.1f",__FUNCTION__,norm0,norm1);
                /* TODO: use spp results directly? */
                return 0;
            }
            for (i=0;i<3;i++) 
            {
                rtkctrl.rb[i]=rb[i];
            }
        }
    }
    //check rb change
    for (i=0;i<3;i++)
    {
        bl[i]=rtkctrl.rb[i]-prb[i];
    }
    if (asg_norm(bl,3)>=DIFFPOSNORM)
    {
        if (prb[0] == 0.0)
        {
            GLOGI("%s: first rb update", __FUNCTION__);
            return 1;
        }
        rtkctrl.rtkevent|=RTK_BASECHANGE; 
    }

    return 1;
}
/* time gap check -------------------------------------------------------------
* time gap check
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
#if 0
static int gnss_rtk_slpdet_timegap(int f,ssat_t *psat,int rcv)
{
    if (psat->hod[f].obsn[rcv-1]<=1)
    {
        return 0;
    }
    
    if (fabs(timediff(psat->hod[f].obs[rcv-1][0].time,psat->hod[f].obs[rcv-1][1].time))>5.0)
    {
        psat->slipflag[rcv-1][f]|=CYCSLIPDET_TIMEGAP;
        return CYCSLIPDET_TIMEGAP;
    }
    
    return 0;
}
#else
static int gnss_rtk_slpdet_timegap(int f, ssat_t *psat, obsd_t obs, int rcv)
{
    int i = 0;
    obsd_t obspre = {0};

    for (i = 0; i < rtkobspre.n; i++)
    {
        if (rtkobspre.data[i].sat == obs.sat)
        {
            memcpy(&obspre, &rtkobspre.data[i], sizeof(obsd_t));
            break;
        }
    }

    if (i == rtkobspre.n)
    {
        return 0;
    }

    if (fabs(timediff(obs.time, obspre.time)) >= 3.0 && g_pe_cfg.meas_rate == MEAS_RATE_1HZ && psat->azel[1] < 55.0 * D2R)
    {
        psat->slipflag[rcv - 1][f] |= CYCSLIPDET_LOCKSAT;
    }

    if ((fabs(timediff(obs.time, obspre.time)) > 1.5 && g_pe_cfg.meas_rate == MEAS_RATE_1HZ)
        || (fabs(timediff(obs.time, obspre.time)) > 0.75 && g_pe_cfg.meas_rate == MEAS_RATE_2HZ)
        || (fabs(timediff(obs.time, obspre.time)) > 0.3 && g_pe_cfg.meas_rate == MEAS_RATE_5HZ)
        || (fabs(timediff(obs.time, obspre.time)) > 0.15 && g_pe_cfg.meas_rate == MEAS_RATE_10HZ))
    {
        psat->slipflag[rcv - 1][f] |= CYCSLIPDET_TIMEGAP;
        return CYCSLIPDET_TIMEGAP;
    }

    return 0;
}
#endif
/* lli cnt check -------------------------------------------------------------
* lli cnt check
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
#if 0
static int gnss_rtk_slpdet_lli(int f,ssat_t *psat,int rcv)
{
    if (psat->hod[f].obsn[rcv-1]==0)
    {
        return 0;
    }
    
    if (rcv==1)
    {
        if (psat->hod[f].obs[rcv-1][0].llicnt>0)
        {
            psat->slipflag[rcv-1][f]|=CYCSLIPDET_LLI;
            if (g_pe_cfg.chipType!=MTK&&(psat->hod[f].obs[rcv-1][0].llicnt&0x3)==0x2) //QCOM and UBLOX
            {
                psat->slipflag[rcv-1][f]|=CYCSLIPDET_LLI_HC;
                return CYCSLIPDET_LLI_HC;
            }
            return CYCSLIPDET_LLI;
        }
        //TODO: do more according to llicnt value
    }
    else if (rcv==2)
    {
        if (psat->hod[f].obs[rcv-1][0].llicnt)
        {
            psat->slipflag[rcv-1][f]|=CYCSLIPDET_LLI;
            return CYCSLIPDET_LLI;
        }
        
    }
    
    return 0;
}
#else
static int gnss_rtk_slpdet_lli(int f, ssat_t *psat, obsd_t obs, int rcv)
{
    if (rcv == 1)
    {
        if (obs.LLI[f] > 0)
        {
            psat->slipflag[rcv - 1][f] |= CYCSLIPDET_LLI;
            if (g_pe_cfg.chipType != MTK && (obs.LLI[f] & 0x3) == 0x2) //QCOM and UBLOX
            {
                psat->slipflag[rcv - 1][f] |= CYCSLIPDET_LLI_HC;
                return CYCSLIPDET_LLI_HC;
            }
            return CYCSLIPDET_LLI;
        }
        //TODO: do more according to llicnt value
    }
    else if (rcv == 2)
    {
        if (obs.LLI[f] > 0)
        {
            psat->slipflag[rcv - 1][f] |= CYCSLIPDET_LLI;
            return CYCSLIPDET_LLI;
        }
    }

    return 0;
}
#endif
/* low power check -------------------------------------------------------------
* low power check
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
#if 0
static int gnss_rtk_slpdet_cn0(int f,ssat_t *psat,int rcv)
{
    if (psat->hod[f].obsn[rcv-1]==0)
    {
        return 0;
    }
    if (psat->hod[f].obs[rcv-1][0].cn0<35)
    {
        psat->slipflag[rcv-1][f]|=CYCSLIPDET_CN0;
        return CYCSLIPDET_CN0;
    }
    //TODO: do more as per average cn0 of latest epoch
    return 0;
}
#else
static int gnss_rtk_slpdet_cn0(int f, ssat_t *psat, obsd_t obs, int rcv)
{
    if (obs.SNR[f] < 35*4)
    {
        psat->slipflag[rcv - 1][f] |= CYCSLIPDET_CN0;
        return CYCSLIPDET_CN0;
    }
    //TODO: do more as per average cn0 of latest epoch
    return 0;
}
#endif
/* doppler check -------------------------------------------------------------
* doppler check
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
#if 0
static int gnss_rtk_slpdet_doppler(int f,ssat_t *psat,int rcv)
{
    if (psat->hod[f].obsn[rcv-1]==0)
    {
        return 0;
    }
    if (fabs(psat->hod[f].obs[rcv-1][0].D)<1E-3)
    {
        psat->slipflag[rcv-1][f]|=CYCSLIPDET_DOPPLER;
        return CYCSLIPDET_DOPPLER;
    }
    return 0;
}
#else
static int gnss_rtk_slpdet_doppler(int f, ssat_t *psat, obsd_t obs, int rcv)
{
    if (fabs(obs.D[f]) < 1E-3)
    {
        psat->slipflag[rcv - 1][f] |= CYCSLIPDET_DOPPLER;
        return CYCSLIPDET_DOPPLER;
    }
    return 0;
}
#endif
/* carrier phase check -------------------------------------------------------------
* carrier phase check
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
#if 0
static int gnss_rtk_slpdet_carrier(int f,ssat_t *psat,int rcv)
{
    if (psat->hod[f].obsn[rcv-1]==0)
    {
        return 0;
    }
    if (psat->hod[f].obs[rcv-1][0].L[0]==0.0)
    {
        psat->slipflag[rcv-1][f]|=CYCSLIPDET_CARRIER;
        return CYCSLIPDET_CARRIER;
    }
    return 0;
}
#else
static int gnss_rtk_slpdet_carrier(int f, ssat_t *psat, obsd_t obs, int rcv)
{
    if (obs.L[f] == 0.0)
    {
        psat->slipflag[rcv - 1][f] |= CYCSLIPDET_CARRIER;
        return CYCSLIPDET_CARRIER;
    }
    return 0;
}
#endif
/* high order difference -------------------------------------------------------------
* high order difference
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
#if 0
static int gnss_rtk_slpdet_hodiff(int f,ssat_t *psat,int rcv)
{
    int slipflag=0;
    float thres4th=4.0,thres5th=8.0;
    hod_l1 *phod = &psat->hod[f];
    if (phod->obsn[rcv-1]<5)
    {
        return 0;
    }
    if (phod->obs[rcv-1][0].L[0]==0.0)
    {
        psat->slipflag[rcv-1][f]|=CYCSLIPDET_HODIFF;
        return CYCSLIPDET_HODIFF;
    }
    //dynamic status
    if (peMode.staticData.staticFlag==0)
    {
        if (peMode.staticData.lsVel<10.0)
        {
            thres4th=5.0;thres5th=10.0;
        }
        else if (peMode.staticData.lsVel<15.0)
        {
            thres4th=8.0;thres5th=12.0;
        }
        else
        {
            thres4th=10.0;thres5th=15.0;
        }
    }
    
    if (phod->obsn[rcv-1]==5)
    {
        if (fabs(phod->obs[rcv-1][0].L[4])>thres4th)
        {
            slipflag=1;
        }
    }
    else
    {
        if (fabs(phod->obs[rcv-1][0].L[4])>thres4th||fabs(phod->obs[rcv-1][0].L[5])>thres5th)
        {
            slipflag=1;
        }
    }

    if (slipflag)
    {
        psat->slipflag[rcv-1][f]|=CYCSLIPDET_HODIFF;
        return CYCSLIPDET_HODIFF;
    }
    
    return 0;
}
#endif
static int gnss_rtk_slpdet_cpdr_calc(int f, ssat_t *psat, obsd_t obs, int rcv, int sat, double *cpdr)
{
    int           i = 0;
    double        dt = 0.0, thres = 5.0;
    GNSS_TIME*    pTime = gnss_tm_get_time();
    uint8_t            gnssMode = GNSS_SYS2MODE(obs.sys);
    obsd_t        obspre = {0};

    if (rcv == 2)
    {
        return 0;
    }

    if (obs.LLI[f] != 0 && obs.LLI[f] != 2)
    {
        return 0;
    }

    if (DBL_IS_EQUAL(rtknav.lam[sat - 1][f], 0.0))
    {
        return 0;
    }

    for (i = 0; i < rtkobspre.n; i++)
    {
        if (rtkobspre.data[i].sat == obs.sat)
        {
            memcpy(&obspre, &rtkobspre.data[i], sizeof(obsd_t));
            break;
        }
    }

    if (i == rtkobspre.n)
    {
        return 0;
    }

    if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100 || g_pe_cfg.sub_chipType == SUB_CHIPTYPE_HD9310)
    {
        dt = timediff(obs.time, obspre.time) - pTime->msec_correction[gnssMode] * 0.001;
    }
    else if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_QCOM_855)
    {
        dt = timediff(obs.time, obspre.time);
    }
    else
    {
        return 0;
    }

    if (dt > 3.5 || dt < 0.05 || obs.L[f] == 0.0 || obspre.L[f] == 0.0
        || (obs.D[f] == 0.0 && obspre.D[f] == 0.0)
        || ((psat->usectrl.rqflag[f] & PE_MEAS_VALID_DR) == 0))
    {
        return 0;
    }

    if (obs.D[f] != 0.0 && obspre.D[f] != 0.0)
    {
        cpdr[0] = obs.L[f] - obspre.L[f] - ((double)obs.D[f] + obspre.D[f]) / 2.0 * dt / rtknav.lam[sat - 1][f];
    }
    else if (obs.D[f] == 0.0)
    {
        cpdr[0] = obs.L[f] - obspre.L[f] - obspre.D[f] * dt / rtknav.lam[sat - 1][f];
    }
    else if (obspre.D[f] == 0.0)
    {
        cpdr[0] = obs.L[f] - obspre.L[f] - obs.D[f] * dt / rtknav.lam[sat - 1][f];
    }

    cpdr[0] = cpdr[0] * rtknav.lam[sat - 1][f];

    return 0;
}
static int gnss_rtk_slpdet_cpdr_det(rtk_t *rtk, double *cpdr)
{
    int             i, j, k, sat, rcv, ref_index = -1, dr_chck_flag = TRUE, res_chck_flag = TRUE, cnt = 0;
    ssat_t*         psat = NULL;
    double          ref_cpdr = 0.0, dt = 0.0, thres = 0.0;
    double          cpdr_res[MAXOBS*NFREQ] = { 0.0 }, cpdr_res_median = 0.0;
    unsigned char   ref_cn0 = 0;
    int loopMax = AGGNSS_MAX_FREQ_NUM < NFREQ ? AGGNSS_MAX_FREQ_NUM : NFREQ;
    obsd_t          obs = {0};
    obsd_t          obspre = {0};

LAB_SLCT_REF:
    ref_cpdr = 0.0; ref_cn0 = 0;
    for (i = 0; i < rtkobs.n; i++)
    {
        sat = rtkobs.data[i].sat;
        if (sat < 1 || sat > MAXSAT)
        {
          continue;
        }
        rcv = rtkobs.data[i].rcv;
        psat = rtk->ssat[sat - 1];
        if (rcv == 2)
        {
          continue;
        }
        if (NULL == psat)
        {
          continue;
        }

        memcpy(&obs, &rtkobs.data[i], sizeof(obsd_t));

        for (j = 0; j < loopMax; j++)
        {
            psat->slipflag[rcv - 1][j] &= ~CYCSLIPDET_CPDR;
            
            if (DBL_IS_EQUAL(cpdr[(i+j*MAXOBS)*2+0], 0.0)) continue;
            if (psat->usectrl.dr_qoschck[j] != 0 && dr_chck_flag == TRUE) continue;
            if (ref_index == (i + j * rtkobs.n) && res_chck_flag == FALSE) continue;
            if (obs.SNR[j] > ref_cn0)
            {
                ref_cn0 = obs.SNR[j];
                ref_cpdr = cpdr[(i+j*MAXOBS)*2+0];
                if (res_chck_flag == TRUE) ref_index = i + j * rtkobs.n;
            }
        }
    }

    if (dr_chck_flag == TRUE && ref_cn0 == 0)
    {
        dr_chck_flag = FALSE;
        goto LAB_SLCT_REF;
    }

    if (ref_cn0 == 0)
    {
        return 0;
    }

    for (i = 0; i < rtkobs.n; i++)
    {
        sat = rtkobs.data[i].sat;
        if (sat < 1 || sat > MAXSAT)
        {
          continue;
        }
        rcv = rtkobs.data[i].rcv;
        psat = rtk->ssat[sat - 1];
        if (rcv == 2)
        {
          continue;
        }
        if (NULL == psat)
        {
          continue;
        }

        for (j = 0; j < loopMax; j++)
        {
            if (DBL_IS_EQUAL(cpdr[(i+j*MAXOBS)*2+0], 0.0)) continue;

            cpdr_res[cnt++] = cpdr[(i+j*MAXOBS)*2+0] - ref_cpdr;
        }
    }

    cpdr_res_median = 0.0;
    if (cnt > 2)
    {
        gnss_median_dbl(cpdr_res, cnt, &cpdr_res_median);
    }

    GLOGI("cpdr ref_cpdr: %.3f, cpdr_res_median: %.3f", ref_cpdr, cpdr_res_median);

    if (fabs(cpdr_res_median) > 0.3 && res_chck_flag == TRUE)
    {
        cnt = 0;
        res_chck_flag = FALSE;
        dr_chck_flag = TRUE;
        goto LAB_SLCT_REF;
    }
    else
    {
        if (fabs(cpdr_res_median) > 0.3)
        {
            //ref_cpdr = 0.0;
            return 0;
        }
        for (i = 0; i < rtkobs.n; i++)
        {
            sat = rtkobs.data[i].sat;
            if (sat < 1 || sat > MAXSAT)
            {
              continue;
            }
            rcv = rtkobs.data[i].rcv;
            psat = rtk->ssat[sat - 1];
            if (rcv == 2)
            {
              continue;
            }
            if (NULL == psat)
            {
              continue;
            }

            memcpy(&obs, &rtkobs.data[i], sizeof(obsd_t));

            for (k = 0; k < rtkobspre.n; k++)
            {
                if (rtkobspre.data[k].sat == obs.sat)
                {
                    memcpy(&obspre, &rtkobspre.data[k], sizeof(obsd_t));
                    break;
                }
            }

            if (k == rtkobspre.n)
            {
                continue;
            }

            for (j = 0; j < loopMax; j++)
            {
                if (DBL_IS_EQUAL(cpdr[(i+j*MAXOBS)*2+0], 0.0)) continue;
                if (DBL_IS_EQUAL(rtknav.lam[sat - 1][j], 0.0)) continue;

                cpdr[(i+j*MAXOBS)*2+1] = (cpdr[(i+j*MAXOBS)*2+0] - ref_cpdr) / rtknav.lam[sat - 1][j];

                dt = timediff(obs.time, obspre.time);
                thres = psat->usectrl.dr_qoschck[j] ? 5.0 : 4.0;
                if (round(dt) > 1.5)
                {
                    thres = thres * pow(1.2, round(dt) - 1);
                }

                if (fabs(cpdr[(i+j*MAXOBS)*2+1]) > thres)
                {
                    psat->slipflag[rcv - 1][j] |= CYCSLIPDET_CPDR;
                }
            }
        }
    }

    return 0;
}
/* gf detect -------------------------------------------------------------
*
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_slpdet_gf(obsd_t *obs,ssat_t *psat)
{
    slpdet_lc_t* lc = NULL;
    double *lam=rtknav.lam[obs->sat-1], gf=0.0,dgf=0.0;
    int i=1,slip=0;

    if (rtkctrl.opt.nf <= 1 || NULL == psat) {
        return;
    }
    lc = &psat->slp_lc[obs->rcv - 1];

    if (lam[0] == 0.0 || lam[i] == 0.0 || obs->L[0] == 0.0 || obs->L[i] == 0.0) {
        return;
    }

    gf = lam[0]*obs->L[0]-lam[i]*obs->L[i];

    if (lc->gf_zd == 0.0 || fabs(timediff(obs->time, lc->time)) >= 3.0)
    {
        goto END_ZD_GF_DET;
    }

    dgf = gf - lc->gf_zd;
    if (fabs(dgf) > 0.054)
    {
        slip = 1;
        psat->slipflag[obs->rcv-1][0] |= CYCSLIPDET_ZD_GF;
    }
    lc->lc_flag |= LC_SLIP_CHECK_GF;

END_ZD_GF_DET:
    lc->gf_zd = gf;
    lc->time = obs->time;

    GLOGD("  gf%1d:%3d %1d %2d   dgf=%10.3f gf=%10.3f", 5, obs->sat, obs->rcv, slip, dgf, gf);
}
/* mw detect -------------------------------------------------------------
*
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_slpdet_mw(obsd_t *obs,ssat_t *psat)
{
    slpdet_lc_t* lc = NULL;
    double *lam=rtknav.lam[obs->sat-1],mw_m,nw,dNw=0.0,_Nw=0.0,_sigma2,sigma=0.0;
    int i=1,slip=0;

    if (rtkctrl.opt.nf <= 1 || NULL == psat)
    {
        return;
    }
    lc = &psat->slp_lc[obs->rcv - 1];

    if (   lam[0] == 0.0 ||    lam[i] == 0.0 || 
        obs->L[0] == 0.0 || obs->L[i] == 0.0 ||
        obs->P[0] == 0.0 || obs->P[i] == 0.0) 
    {
        return;
    }

    mw_m = -lam[0]*lam[i]*(obs->L[0]-obs->L[i])/(lam[i]-lam[0])+
        (lam[i]*obs->P[0]+lam[0]*obs->P[i])/(lam[i]+lam[0]);
    nw = mw_m*(lam[i]-lam[0])/(lam[0]*lam[i]);

    if (lc->mw_nw == 0.0 && lc->mw_cnt == 0)
    {
        lc->mw_nw = nw;
        lc->mw_cnt = 1;
        lc->mw_sigma2 = 0.0;
        
        goto END_ZD_MW_DET;
    }
    lc->mw_cnt++;
    dNw = nw - lc->mw_nw;
    _Nw = lc->mw_nw + dNw/lc->mw_cnt;
    _sigma2 = lc->mw_sigma2 + (dNw*dNw - lc->mw_sigma2)/lc->mw_cnt;

    sigma = sqrt(lc->mw_sigma2);
    if (lc->mw_cnt == 2)
    {
        if (fabs(dNw) >= 10.0)
        {
            slip = 1;
            psat->slipflag[obs->rcv-1][0] |= CYCSLIPDET_ZD_MW;
            /* clean mw info */
            lc->mw_nw = nw;
            lc->mw_cnt = 1;
            lc->mw_sigma2 = 0.0;
        }
    }
    else if (lc->mw_cnt>2)
    {
        if (sigma > 5.0 || (fabs(dNw) >= 4 * sigma && fabs(dNw) > 1.0))
        {
            slip = 1;
            psat->slipflag[obs->rcv-1][0] |= CYCSLIPDET_ZD_MW;
            /* clean mw info */
            lc->mw_nw = nw;
            lc->mw_cnt = 1;
            lc->mw_sigma2 = 0.0;
        }
    }

    lc->lc_flag |= LC_SLIP_CHECK_MW;
    lc->mw_dNw = (float)dNw;
    if (!slip)
    {
        lc->mw_nw = _Nw;
        lc->mw_sigma2 = _sigma2;
    }

END_ZD_MW_DET:
    GLOGD("  mw%1d:%3d %1d %2d   dNw=%10.3f nw=%10.3f _Nw=%12.3f sigma=%6.3f cnt=%3d", 5, obs->sat, obs->rcv, slip, dNw, nw, _Nw, sigma, lc->mw_cnt);
}
/* cycle slip status check -------------------------------------------------------------
* cycle slip check
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
#if 0
extern void gnss_rtk_slpdet_statcheck(int sat,ssat_t *psat,int rcv)
{
    int8_t tmgap[NFREQ] = { 0 }, lli[NFREQ] = { 0 }, cn0[NFREQ] = { 0 }, dop[NFREQ] = { 0 }, hod[NFREQ] = { 0 }, caph[NFREQ] = { 0 };
    int j;

    for (j = 0; j < AGGNSS_MAX_FREQ_NUM && j < NFREQ; j++)
    {
        tmgap[j]=(psat->slipflag[rcv-1][j]&CYCSLIPDET_TIMEGAP)>0;
        lli[j]=(psat->slipflag[rcv-1][j]&CYCSLIPDET_LLI)>0;
        cn0[j]=(psat->slipflag[rcv-1][j]&CYCSLIPDET_CN0)>0;
        dop[j]=(psat->slipflag[rcv-1][j]&CYCSLIPDET_DOPPLER)>0;
        hod[j]=(psat->slipflag[rcv-1][j]&CYCSLIPDET_HODIFF)>0;
        caph[j]=(psat->slipflag[rcv-1][j]&CYCSLIPDET_CARRIER)>0;

        if (psat->hod[j].obsn[rcv-1]<2) tmgap[j]=-1;
        if (psat->hod[j].obsn[rcv-1]<5) hod[j]=-1;

        if (caph[j])
        {
            psat->hod[j].obs[rcv-1][0].L[4]=0.0;
            psat->hod[j].obs[rcv-1][0].L[5]=0.0;
        }
    }
    if (NFREQ < 2) return;
    if (rcv==1) //rover
    {
        GLOGI("slp-p:%3d %d %5d %9.3f %9.3f %3d %1d%1d%1d%1d%2d %d %2d %5.1f %5d %9.3f %9.3f %3d %1d%1d%1d%1d%2d %d %2d  %x %x  %d %d",
            sat,rcv,psat->hod[0].obsn[rcv-1],psat->hod[0].obs[rcv-1][0].L[4],psat->hod[0].obs[rcv-1][0].L[5],
            tmgap[0],lli[0],cn0[0],dop[0],caph[0],hod[0],psat->hod[0].obs[rcv-1][0].llicnt,psat->hod[0].obs[rcv-1][0].cn0,psat->azel[1]*R2D,
            psat->hod[1].obsn[rcv-1],psat->hod[1].obs[rcv-1][0].L[4],psat->hod[1].obs[rcv-1][0].L[5],
            tmgap[1],lli[1],cn0[1],dop[1],caph[1],hod[1],psat->hod[1].obs[rcv-1][0].llicnt,psat->hod[1].obs[rcv-1][0].cn0,
            psat->trkstate[0],psat->trkstate[1], psat->lockms[0][0]>=psat->lockms[1][0], psat->lockms[0][1]>=psat->lockms[1][1]);
    }
    else        //refer
    {
        GLOGI("slp-p:%3d %d %5d %10.3f %9.3f %2d %1d%1d%1d%1d%2d %d %2d       %5d %10.3f %9.3f %2d %1d%1d%1d%1d%2d %d %2d",
            sat,rcv,psat->hod[0].obsn[rcv-1],psat->hod[0].obs[rcv-1][0].L[4],psat->hod[0].obs[rcv-1][0].L[5],
            tmgap[0],lli[0],cn0[0],dop[0],caph[0],hod[0],psat->hod[0].obs[rcv-1][0].llicnt,psat->hod[0].obs[rcv-1][0].cn0,
            psat->hod[1].obsn[rcv-1],psat->hod[1].obs[rcv-1][0].L[4],psat->hod[1].obs[rcv-1][0].L[5],
            tmgap[1],lli[1],cn0[1],dop[1],caph[1],hod[1],psat->hod[1].obs[rcv-1][0].llicnt,psat->hod[1].obs[rcv-1][0].cn0);
    }
    
}
#else
extern void gnss_rtk_slpdet_statcheck(int sat, ssat_t *psat, obsd_t obs, int rcv, double *out_cpdr)
{
    int8_t tmgap[NFREQ] = { 0 }, lli[NFREQ] = { 0 }, cn0[NFREQ] = { 0 }, dop[NFREQ] = { 0 }, /*hod[NFREQ] = { 0 },*/ caph[NFREQ] = { 0 }, cpdr[NFREQ] = { 0 };
    int j;
    int loopMax = AGGNSS_MAX_FREQ_NUM < NFREQ ? AGGNSS_MAX_FREQ_NUM : NFREQ;
    for (j = 0; j < loopMax; j++)
    {
        tmgap[j] = (psat->slipflag[rcv - 1][j] & CYCSLIPDET_TIMEGAP) > 0;
        lli[j] = (psat->slipflag[rcv - 1][j] & CYCSLIPDET_LLI) > 0;
        cn0[j] = (psat->slipflag[rcv - 1][j] & CYCSLIPDET_CN0) > 0;
        dop[j] = (psat->slipflag[rcv - 1][j] & CYCSLIPDET_DOPPLER) > 0;
        //hod[j] = (psat->slipflag[rcv - 1][j] & CYCSLIPDET_HODIFF) > 0;
        caph[j] = (psat->slipflag[rcv - 1][j] & CYCSLIPDET_CARRIER) > 0;
        cpdr[j] = (psat->slipflag[rcv - 1][j] & CYCSLIPDET_CPDR) > 0;

        //if (psat->hod[j].obsn[rcv - 1] < 2) tmgap[j] = -1;
        //if (psat->hod[j].obsn[rcv - 1] < 5) hod[j] = -1;

        //if (caph[j])
        //{
        //	psat->hod[j].obs[rcv - 1][0].L[4] = 0.0;
        //	psat->hod[j].obs[rcv - 1][0].L[5] = 0.0;
        //}
    }
    if (NFREQ < 2) {
      return;
    }

    if (rcv == 1) //rover
    {
        GLOGD("slp-p:%3d %d, %3d %1d%1d%1d%1d, cpdr %d %6.3f %6.3f, %d %2d %5.1f, %3d %1d%1d%1d%1d, cpdr %d %6.3f %6.3f, %d %2d",
            sat, rcv,
            tmgap[0], lli[0], cn0[0], dop[0], caph[0],
            cpdr[0], out_cpdr[1], out_cpdr[0],
            obs.LLI[0], obs.SNR[0]/4, psat->azel[1] * R2D,
            tmgap[1], lli[1], cn0[1], dop[1], caph[1],
            cpdr[1], out_cpdr[MAXOBS*2+1], out_cpdr[MAXOBS*2],
            obs.LLI[1], obs.SNR[1]/4);
    }
    //else        //refer
    //{
    //	GLOGI("slp-p:%3d %d %5d %10.3f %9.3f %2d %1d%1d%1d%1d%2d %d %2d       %5d %10.3f %9.3f %2d %1d%1d%1d%1d%2d %d %2d",
    //		sat, rcv, psat->hod[0].obsn[rcv - 1], psat->hod[0].obs[rcv - 1][0].L[4], psat->hod[0].obs[rcv - 1][0].L[5],
    //		tmgap[0], lli[0], cn0[0], dop[0], caph[0], hod[0], psat->hod[0].obs[rcv - 1][0].llicnt, psat->hod[0].obs[rcv - 1][0].cn0,
    //		psat->hod[1].obsn[rcv - 1], psat->hod[1].obs[rcv - 1][0].L[4], psat->hod[1].obs[rcv - 1][0].L[5],
    //		tmgap[1], lli[1], cn0[1], dop[1], caph[1], hod[1], psat->hod[1].obs[rcv - 1][0].llicnt, psat->hod[1].obs[rcv - 1][0].cn0);
    //}
}
#endif
/* cycle slip detect roughly -------------------------------------------------------------
* cycle slip detect
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_slip_detect_pre(rtk_t *rtk)
{
    int i,j,sat,rcv;
    ssat_t *psat=NULL;
    double *cpdr = zeros(MAXOBS*NFREQ,2);
    int loopMax = AGGNSS_MAX_FREQ_NUM < NFREQ ? AGGNSS_MAX_FREQ_NUM : NFREQ;

    for (i=0;i<rtkobs.n;i++)
    {
        sat=rtkobs.data[i].sat;
        rcv=rtkobs.data[i].rcv;
        psat=rtk->ssat[sat-1];
        if (psat == NULL || sat < 1 || sat > MAXSAT)
        { 
            continue;
        }
        
        for (j = 0; j < loopMax; j++)
        {
            //psat->pslipflag[rcv-1][j]=psat->slipflag[rcv-1][j];
            psat->slipflag[rcv-1][j]=0; //clear flag

#if 0
            if (!psat->hod[j].obsn[rcv-1])
            {
                psat->slipflag[rcv-1][j]=CYCSLIPDET_LLI;  //not saved in obs_l1
                continue;
            }
            /* time gap */
            gnss_rtk_slpdet_timegap(j,psat,rcv);
            /* LLI */
            gnss_rtk_slpdet_lli(j,psat,rcv);
            /* CN0 */
            gnss_rtk_slpdet_cn0(j,psat,rcv);
            /* Doppler */
            gnss_rtk_slpdet_doppler(j,psat,rcv);
            /* Carrier */
            gnss_rtk_slpdet_carrier(j,psat,rcv);
            /* HOdiff */
            gnss_rtk_slpdet_hodiff(j,psat,rcv);
#else
            /* time gap */
            gnss_rtk_slpdet_timegap(j, psat, rtkobs.data[i], rcv);
            /* LLI */
            gnss_rtk_slpdet_lli(j, psat, rtkobs.data[i], rcv);
            /* CN0 */
            gnss_rtk_slpdet_cn0(j, psat, rtkobs.data[i], rcv);
            /* Doppler */
            gnss_rtk_slpdet_doppler(j, psat, rtkobs.data[i], rcv);
            /* Carrier */
            gnss_rtk_slpdet_carrier(j, psat, rtkobs.data[i], rcv);
            /* CpDrConsistency */
            gnss_rtk_slpdet_cpdr_calc(j, psat, rtkobs.data[i], rcv, sat, cpdr+(i+j*MAXOBS)*2);
#endif
        }

        /* LC detect */
        psat->slp_lc[rcv-1].lc_flag = 0; //clean check flag
        gnss_rtk_slpdet_gf(&rtkobs.data[i],psat);
        gnss_rtk_slpdet_mw(&rtkobs.data[i],psat);
    }

    gnss_rtk_slpdet_cpdr_det(rtk, cpdr);

    for (i = 0; i < rtkobs.n; i++)
    {
        sat = rtkobs.data[i].sat;
        rcv = rtkobs.data[i].rcv;
        psat = rtk->ssat[sat - 1];
        if (psat == NULL || sat < 1 || sat > MAXSAT) continue;

        /* check status */
        gnss_rtk_slpdet_statcheck(sat, psat, rtkobs.data[i], rcv, cpdr+i*2);
    }

    Sys_Free(cpdr);
}
extern void gnss_rtk_memcpy_rtkobs()
{
    int i;
    rtkobspre.n = rtkobs.n - rtcm3data.obs[0].n;
    if (rtkobspre.n > 0 && rtkobspre.n <= MAXOBS)
    {
        for (i = 0; i < rtkobspre.n; i++)
        {
            if (rtkobs.data[i].rcv == 1)
            {
                memcpy(&rtkobspre.data[i], &rtkobs.data[i], sizeof(obsd_t));
            }
        }
    }
}
/* obsl1 clean check -------------------------------------------------------------
* obsl1 clean check
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
#if 0
extern void gnss_rtk_obsl1_clean_check(rtk_t *rtk)
{
    int i,j,sat,rcv,flag,rsvnum,slip,quality;
    ssat_t *psat=NULL;
    hod_l1 *phod = NULL;

    for (i=0;i<rtkobs.n;i++)
    {
        sat=rtkobs.data[i].sat;
        rcv=rtkobs.data[i].rcv;
        psat=rtk->ssat[sat-1];
        if (psat == NULL)
        {
            continue;
        }
        
        for (j = 0; j < AGGNSS_MAX_FREQ_NUM && j < NFREQ; j++)
        {
            phod = &psat->hod[j];
            if (!phod->obsn[rcv-1])
            {
                continue;
            }
            if (rtkobs.data[i].L[0]==0.0)  //cp invalid
            {
                gnss_rtk_obsl1_clean(phod,rcv,0);
                continue;
            }
            slip=psat->slipflag[rcv-1][j];
            flag=0;
            rsvnum=0;
            //clear obs buffer, or set flag to clear them
            quality=((slip&CYCSLIPDET_LLI)>0)+((slip&CYCSLIPDET_CN0)>0)+((slip&CYCSLIPDET_DOPPLER)>0);
            if (phod->obsn[rcv-1]<5)
            {
                if (slip&CYCSLIPDET_TIMEGAP)
                {
                    flag=1;
                    if (quality<=1) //current epoch ok
                    {
                        rsvnum=1;
                    }
                }
                else if (quality==3)//current epoch not ok
                {
                    flag=1;
                }
            }
            else if (slip&CYCSLIPDET_HODIFF)
            {
                flag=1;
                if ((psat->tdcheck[j]&TD_CHECK_EST)&&(slip&CYCSLIPDET_TDEST)==0)//td ok
                {
                    rsvnum=2;
                }
                else if (quality<=1)//current epoch ok
                {
                    rsvnum=1;
                }
            }
            else if (slip&(CYCSLIPDET_CPDR| CYCSLIPDET_TDEST| CYCSLIPDET_TDRES))
            {
                flag = 1;
                if (quality <= 1) //current epoch ok
                {
                    rsvnum = 1;
                }
            }
            else if ((slip&CYCSLIPDET_LLI)&&((slip&CYCSLIPDET_LLI_HC)==0))
            {
                flag = 1;
                if (quality <= 1) //current epoch ok
                {
                    rsvnum = 1;
                }
            }

            if (flag)
            {
                gnss_rtk_obsl1_clean(phod,rcv,rsvnum);
            }
        }
        
    }
}
#endif
/* test navi system (m=0:gps/qzs/sbs,1:glo,2:gal,3:bds) ----------------------*/
static int gnss_rtk_test_sys(int sys, int m)
{
    switch (sys) {
    case SYS_GPS: return m==0;
    case SYS_QZS: return (NSYSDD == 4 ? m == 0 : m == 4);
    case SYS_SBS: return m==0;
    case SYS_GLO: return m==1;
    case SYS_GAL: return m==2;
    case SYS_CMP: return m==3;
    }
    return 0;
}
int32_t TD_DEBUG(int32_t index,int32_t lowlimit,int32_t uplimit, int32_t line)
{
    if (index < lowlimit || index >= uplimit)
    {
        GLOGI("TD_DEBUG: %d: index=%d, lowlimit=%d, uplimit=%d",line, index, lowlimit, uplimit);
        return 1;
    }
    return 0;
}
/* td cycle slip detect -------------------------------------------------------------
* td cycle slip detect
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_slip_detect_td(rtk_t *rtk,const nav_t *nav,const obsd_t *obs,int nu,int *sat,int *iu,int *ir,int ns,double *y,double *e,double *azel, int n_all,double *crr)
{
    double dt1, dt2, ele = 0.0, cury, prey, mean = 0.0, std = 0.0, lami = 0.0;
    int i, j, m, k = 0, nsat, iuref = -1, irref = -1, refs = 0, isys, f = 0, nf = (rtk->opt.ionoopt == IONOOPT_IFLC) ? 1 : rtk->opt.nf, refsat[NSYSDD] = { 0 }, ntd, info, rejflag;
    int cn0 = 0, reff = 0, refm = 0, ref_flag = 0, tdhcflag = 0;
    unsigned char satp[MAXOBS] = { 0 }, iup[MAXOBS] = { 0 }, irp[MAXOBS] = { 0 };
    double *L = NULL, *H = NULL, dx[3] = { 0 }, Q[9] = { 0 }, offset, pdop = 0.0;
    double median, medianout, *datay = NULL, *dataabs = NULL, K1 = 10.0, lower_para = 1.0;
#ifdef PLAYBACK_MODE
    char rejsat[1024] = { 0 }, *pstr = &rejsat[0];
    char str[1024] = { '\0' }, *p = &str[0];
    int reject = 0;
#endif
    int ntdnew = 0;
    uint8_t *satv = NULL, *freq = NULL, bad_tdlist = 0, staticflag = 0;
    gtime_t ctu = obs[0].time, ctr = obs[nu].time;

    /* check last epoch info */
    dt1 = timediff(ctu, rtk->pt[0]);
    dt2 = timediff(ctu, rtk->pt[1]);
    if (fabs(dt1) > TDTGAP || fabs(dt2) > rtk->opt.maxtdiff)
    {
        GLOGI("slpdet-td:time gap too large(dt1=%.3f dt2=%.3f), ns=%d", dt1, dt2, ns);
        goto lab_tdinfo;
    }
    GLOGI("slpdet-td: dt1=%.6f dt2=%.6f ns=%d MAXSAT=%d f=%d,nf=%d,n_all=%d", dt1, dt2, ns, MAXSAT, f, nf, n_all);
    //find common sat
    for (i = 0, j = 0; i < ns&&j < rtk->nsat; i++, j++)
    {
        if (sat[i] < rtk->vsat[j]) j--;
        else if (sat[i] > rtk->vsat[j]) i--;
        else
        {
            satp[k] = sat[i];
            iup[k] = iu[i];
            irp[k++] = ir[i];
        }
    }

    if (k < 2)
    {
        GLOGI("slpdet-td:td common sat too less(%d), can not form td", k);
        goto lab_tdinfo;
    }

    satv = (uint8_t *)Sys_Calloc(1, sizeof(uint8_t)*nf*k); freq = (uint8_t *)Sys_Calloc(1, sizeof(uint8_t)*nf*k);
    if (satv == NULL || freq == NULL)
    {
        if (satv) Sys_Free(satv);
        if (freq) Sys_Free(freq);
        goto lab_tdinfo;
    }

    L = mat(nf*k, 1); H = mat(3, nf*k);
    datay = mat(nf*k, 1); dataabs = mat(nf*k, 1);

    //goto change refsat when refsat may have cycle slip 
LAB_REF_CHANGE:
    cn0 = 0; ele = 0.0; refs = 0; reff = 0; refm = 0; iuref = 0; irref = 0;
    //select refsat,all freq and all system only select one refsat
    for (f = 0; f < nf; f++) /* for each freq */
    {
        for (m = 0; m < NSYSDD; m++) /* m=0:gps/qzs/sbs,1:glo,2:gal,3:bds */
        {
            for (i = 0; i < k; i++)
            {
                isys = satsys(satp[i], NULL);
                if (!gnss_rtk_test_sys(isys, m)) continue;
                if (TD_DEBUG(f + iup[i] * nf * 2, 0, nf * 2 * n_all, __LINE__)) continue;
                if (TD_DEBUG(f + irp[i] * nf * 2, 0, nf * 2 * n_all, __LINE__)) continue;
                if (y[f + iup[i] * nf * 2] == 0.0 || y[f + irp[i] * nf * 2] == 0.0)
                {
                    continue;
                }
                if (NULL == rtk->ssat[satp[i] - 1])
                {
                    continue;
                }
                if (rtk->ssat[satp[i] - 1]->td[0].y[f] == 0.0 || rtk->ssat[satp[i] - 1]->td[1].y[f] == 0.0)
                {
                    continue;
                }
                if (TD_DEBUG(satp[i] - 1, 0, MAXSAT, __LINE__)) continue;

                if (rtk->ssat[satp[i] - 1]->slipflag[0][f] & (CYCSLIPDET_LLI | CYCSLIPDET_HODIFF | CYCSLIPDET_CPDR))
                {
                    //the satellite can not be refsat when other methods detected cycle slip for rover observation
                    continue;
                }
                if (rtk->ssat[satp[i] - 1]->slipflag[1][f] & (CYCSLIPDET_LLI | CYCSLIPDET_HODIFF | CYCSLIPDET_CPDR | CYCSLIPDET_DDSAT))
                {
                    //the satellite can not be refsat when other methods detected cycle slip for base observation
                    continue;
                }
                //refsat may have cycleslip 
                if (rtk->ssat[satp[i] - 1]->slipflag[0][f] & CYCSLIPDET_TDRES) continue;

                if ((rtk->ssat[satp[i] - 1]->usectrl.rqflag[f] & PE_MEAS_VALID_PR) == 0) continue;
                //find ref sat base on cn0 and ele
                if ((azel[1 + iup[i] * 2] > ele || azel[1 + iup[i] * 2] * R2D > 30.0)
                    && (rtk->ssat[satp[i] - 1]->snr[f]/4) > cn0)
                {
                    cn0 = (rtk->ssat[satp[i] - 1]->snr[f]/4);
                    ele = azel[1 + iup[i] * 2];
                    refs = satp[i]; reff = f; refm = m;
                    iuref = iup[i];
                    irref = irp[i];
                }
            }
        }
    }

    if (refs == 0)
    {
        GLOGI("td: no refsat available");

        if (L) Sys_Free(L);
        if (H) Sys_Free(H);
        if (satv) Sys_Free(satv);
        if (freq) Sys_Free(freq);
        if (datay) Sys_Free(datay);
        if (dataabs) Sys_Free(dataabs);

        goto lab_tdinfo;
    }

    ntd = 0;
    //make td obs
    for (f = 0; f < nf; f++) /* for each freq */
        for (m = 0; m < NSYSDD; m++) /* m=0:gps/qzs/sbs,1:glo,2:gal,3:bds */
        {
            for (i = 0; i < k; i++)
            {
                if (satp[i] == refs)
                {
                    //ref sat continue
                    //there is a problem when refsat has multi frequence obs
                    //other frequenc will not be detected besides reff
                    continue;
                }
                isys = satsys(satp[i], NULL);
                if (!gnss_rtk_test_sys(isys, m)) continue;
                if (TD_DEBUG(f + iup[i] * nf * 2, 0, nf * 2 * n_all, __LINE__)) continue;
                if (TD_DEBUG(f + irp[i] * nf * 2, 0, nf * 2 * n_all, __LINE__)) continue;
                if (y[f + iup[i] * nf * 2] == 0.0 || y[f + irp[i] * nf * 2] == 0.0)
                {
                    continue;
                }
                if (TD_DEBUG(satp[i] - 1, 0, MAXSAT, __LINE__)) continue;
                if (NULL == rtk->ssat[satp[i] - 1])
                {
                    continue;
                }
                if (rtk->ssat[satp[i] - 1]->td[0].y[f] == 0.0 || rtk->ssat[satp[i] - 1]->td[1].y[f] == 0.0)
                {
                    //rtk->ssat[satp[i]-1].slipflag[0][f]|=CYCSLIPDET_TDRES; //last epoch not usable
                    continue;
                }
                /* double-differenced residual */
                if (TD_DEBUG(reff + iuref * nf * 2, 0, nf * 2 * n_all, __LINE__)) continue;
                if (TD_DEBUG(reff + irref * nf * 2, 0, nf * 2 * n_all, __LINE__)) continue;
                if (TD_DEBUG(refs - 1, 0, MAXSAT, __LINE__)) continue;
                offset = 0.0;
                if ((NULL != rtk->ssat[refs - 1])&&(rtk->rtkevent&RTK_BASECHANGE) && (rtk->ssat[refs - 1]->tdcheck[reff] & DD_CHECK_REFSAT) && (rtk->ssat[satp[i] - 1]->tdcheck[f] & DD_CHECK_REFSAT))
                {
                    offset = rtk->ssat[refs - 1]->slp_ref.slip[reff] * rtknav.lam[refs - 1][reff]
                        - rtk->ssat[satp[i] - 1]->slp_ref.slip[f] * rtknav.lam[satp[i] - 1][f];
                }
                if (NULL != rtk->ssat[refs - 1])
                {
                    cury = (y[reff + iuref * nf * 2] - y[reff + irref * nf * 2]) - (y[f + iup[i] * nf * 2] - y[f + irp[i] * nf * 2]) + offset;
                    prey = (rtk->ssat[refs - 1]->td[0].y[reff] - rtk->ssat[refs - 1]->td[1].y[reff]) - (rtk->ssat[satp[i] - 1]->td[0].y[f] - rtk->ssat[satp[i] - 1]->td[1].y[f]);
                    rtk->ssat[satp[i] - 1]->tdy[f] = cury - prey;
                    rtk->ssat[satp[i] - 1]->tdcheck[f] |= TD_CHECK_RES;
                }

                //TD-RES check
                rejflag = 0;
                if ((NULL != rtk->ssat[refs - 1])&& fabs(rtk->ssat[satp[i] - 1]->tdy[f]) > 10.0) //TODO: take ptdy,rb valid,cn0,lli,dop into account
                {
                    rejflag = 1;
                    rtk->ssat[satp[i] - 1]->slipflag[0][f] |= CYCSLIPDET_TDRES;
                }
                else if (rtk->ssat[satp[i] - 1]->slipflag[0][f] & (CYCSLIPDET_HODIFF | CYCSLIPDET_CPDR))
                {
                    rejflag = 1;
                }
                else if (rtk->ssat[satp[i] - 1]->slipflag[1][f] & (CYCSLIPDET_HODIFF | CYCSLIPDET_CPDR | CYCSLIPDET_DDSAT))
                {
                    rejflag = 1;
                }
                else if ((rtk->ssat[satp[i] - 1]->slipflag[0][f] & CYCSLIPDET_LLI) &&
                    ((rtk->ssat[satp[i] - 1]->slipflag[0][f] & CYCSLIPDET_LLI_HC) == 0))
                {
                    //LLI==2 will be used in td slip detection
                    rejflag = 1;
                }
                else if ((rtk->ssat[satp[i] - 1]->slipflag[1][f] & CYCSLIPDET_LLI) &&
                    ((rtk->ssat[satp[i] - 1]->slipflag[1][f] & CYCSLIPDET_LLI_HC) == 0))
                {
                    //LLI==2 will be used in td slip detection
                    rejflag = 1;
                }

                //est prepare
                if (TD_DEBUG(j + iuref * 3, 0, 3 * n_all, __LINE__)) continue;
                if (TD_DEBUG(j + iup[i] * 3, 0, 3 * n_all, __LINE__)) continue;
                if ((NULL != rtk->ssat[refs - 1]) && rejflag == 0 && L&&H&&ntd < nf*k)
                {
                    L[ntd] = rtk->ssat[satp[i] - 1]->tdy[f];
                    for (j = 0; j < 3; j++)
                    {
                        H[j + ntd * 3] = -(e[j + iuref * 3] - e[j + iup[i] * 3]);
                    }
                    satv[ntd] = satp[i]; //save valid sat list
                    freq[ntd] = f;
                    ntd++;
                }
            }
        }

    if (peMode.staticData.staticFlag == 1 && peMode.staticData.lsVel < 0.1&&peMode.staticData.deltaDoplVar < 0.1) staticflag = 1;

    if (ntd>0)
    {	
        //static mode,coordinate error compensation
        //previous epoch xyz are same as current for static mode,eliminate geometric distance error
        if (staticflag==1)
        {
            for (i = 0; i < 3; i++) dx[i] = crr[i] - rtk->prr[i];
#ifndef ASM_MATMUL
            matmul("TN", ntd, 1, 3, 1.0, H, dx, 1.0, L);
#else
            Fast_Rtk_matmul("TN", ntd, 1, 3, 1.0, H, dx, 1.0, L);
#endif
        }
        else {
            if (rtk->shift.feedback_flag && (!rtk->shift.last_feedback_flag))
            {
                /*	when rtk solutions feedback pvt,the previous coordinate for td is pvt solutions
                    however,the current coordinate for td is based on feedback rtk solutions
                    so the coordinate is not consistent for td slip check
                    there adjust the previous pvt to previous feedback rtk solutions	*/
                for (i = 0; i < 3; i++) dx[i] = rtk->shift.feedback_rr[i] - rtk->prr[i];
#ifndef ASM_MATMUL
                matmul("TN", ntd, 1, 3, 1.0, H, dx, 1.0, L);
#else
                Fast_Rtk_matmul("TN", ntd, 1, 3, 1.0, H, dx, 1.0, L);
#endif
                GLOGI("eliminate geometric distance error for rtk feedback to PE");
            }
            else if (g_pe_cfg.meas_rate == MEAS_RATE_2HZ || g_pe_cfg.meas_rate == MEAS_RATE_5HZ || g_pe_cfg.meas_rate == MEAS_RATE_10HZ)
            {
                if (((g_proc.flag & 0xFC01) > 0) && ((g_proc.last_flag & 0xFC01) == 0) && (g_proc.last_flag > 0))
                {
                    for (i = 0; i < 3; i++) dx[i] = crr[i] - g_proc.pos[i];
#ifndef ASM_MATMUL
                    matmul("TN", ntd, 1, 3, 1.0, H, dx, 1.0, L);
#else
                    Fast_Rtk_matmul("TN", ntd, 1, 3, 1.0, H, dx, 1.0, L);
#endif
                    GLOGI("eliminate geometric distance error for extrapolation and calculation position exchange");
                }
            }
        }

        if (datay&&dataabs)
        {
            for (i = 0; i < ntd; i++)
            {
                datay[i] = fabs(L[i]);
            }
            gnss_math_dstd(datay, ntd, &mean, &std);
            gnss_median_dbl(datay, ntd, &median);
            gnss_MAD_DBL(datay, dataabs, ntd, &medianout);

            GLOGI("refsat info:sat=%d,m=%d,f=%d,ntd=%d,ele=%.2f,cn0=%d, mean=%.3f, std=%.3f,median=%.3f,medianout=%.3f",
                refs, refm, reff, ntd, ele*R2D, cn0, mean, std, median, medianout);

            //ref check
            if (ntd > 1 && ((median > 0.5 &&std < 0.2) || (std >= 0.5 && median >= 0.5 && medianout >= 0.1)
                || (mean > 0.4&&median > 0.45 &&std < 0.35)
                || (staticflag == 1 && std<0.02&&mean > 0.08)))
            {
                GLOGI("refsat may have cycleslip,tor=%f,sat=%d,m=%d,f=%d", rtk->ptor, refs, refm, reff);
                if (NULL != rtk->ssat[refs - 1])
                {
                    rtk->ssat[refs - 1]->slipflag[0][reff] |= CYCSLIPDET_TDRES;
                }	
                ref_flag++;
                if (ref_flag <= 2) //change ref,
                {
                    ntd = 0;
                    goto LAB_REF_CHANGE;
                }
            }
            //sign refsat TD_CHECK_RES,this will be used in lc_fusion
            if (NULL != rtk->ssat[refs - 1])
            {
                rtk->ssat[refs - 1]->tdcheck[reff] |= TD_CHECK_RES;
            }

            //printf td info
            for (i = 0; i < ntd; i++)
            {
                if (NULL == rtk->ssat[satv[i] - 1])
                {
                    continue;
                }
                GLOGD("slpdet-td:%d %3d-%3d %16.3f %5.1f %3d [%2d]", freq[i], refs, satv[i],
                    L[i], rtk->ssat[satv[i] - 1]->azel[1]*R2D, rtk->ssat[satv[i] - 1]->snr[freq[i]]/4, i+1);
            }
            
            GLOGD("slpdet-td: mean=%.3f std=%.3f median=%.3f dis_median=%.3f", mean, std, median, medianout);
            if ((rtk->closkycnt & 0xFFF) == 0) lower_para = 3.0;
            if (std < 0.5 || median < 0.5 || medianout < 0.1) //gross error detection
            {
                ntdnew = 0;
                for (i = 0; i < ntd; i++)
                {
                    if (datay[i] > 1.0 || (datay[i] > 0.1&&median<0.1&&dataabs[i]>K1*(medianout / 0.6745) / lower_para))
                    {
                        if (NULL != rtk->ssat[satv[i] - 1])
                        {
                            rtk->ssat[satv[i] - 1]->slipflag[0][freq[i]] |= CYCSLIPDET_TDRES;
#ifdef PLAYBACK_MODE
                            pstr += sprintf(pstr, "%u-%2u ", freq[i], satv[i]);
#endif
                            continue;
                        }
                    }

                    //static mode
                    if (staticflag == 1 && datay[i] > 0.2&&median < 0.1&&mean < 0.2&&std < 0.2)
                    {
                        if (NULL != rtk->ssat[satv[i] - 1])
                        {
                            rtk->ssat[satv[i] - 1]->slipflag[0][freq[i]] |= CYCSLIPDET_TDRES;
#ifdef PLAYBACK_MODE
                            pstr += sprintf(pstr, "%u-%2u ", freq[i], satv[i]);
#endif
                            continue;
                        }
                    }

                    //adjust L, H
                    L[ntdnew] = L[i];
                    for (j = 0; j < 3; j++)
                    {
                        H[j + ntdnew * 3] = H[j + i * 3];
                    }
                    satv[ntdnew] = satv[i];
                    freq[ntdnew] = freq[i];
                    ntdnew++;
                }

                ntd = ntdnew;
#ifdef PLAYBACK_MODE
                *pstr = '\0';
                GLOGI("td res reject sat: %s", rejsat);
#endif
            }
            else
            {
                GLOGI("abnormal td res list detected, rbflag=%d, basechanged=%d", rtk->rbflag, (rtk->rtkevent&RTK_BASECHANGE) > 0);
                if (rtk->rbflag == 1 && !(rtk->rtkevent&RTK_BASECHANGE) && std > 1.0 && median > 1.0 && medianout > 1.0)
                {
                    bad_tdlist = 1;
                }

                //some special strategy when ntd is not enough for lsq detect cycle slip 
                if (ntd <= 3)
                {
                    for (i = 0; i < ntd; i++)
                    {
                        if (NULL == rtk->ssat[satv[i] - 1])
                        {
                            continue;
                        }
                        if (datay[i] > 1.0 && (rtk->ssat[satv[i] - 1]->slipflag[0][freq[i]] & CYCSLIPDET_LLI_HC)
                            && (rtk->ssat[satv[i] - 1]->usectrl.rqflag[freq[i]] & PE_MEAS_VALID_DR) == 0
                            && (rtk->ssat[satv[i] - 1]->snr[freq[i]]/4) < 35)
                        {
                            rtk->ssat[satv[i] - 1]->slipflag[0][freq[i]] |= CYCSLIPDET_TDRES;
                            GLOGI("special td res reject sat: %u", satv[i]);
                        }
                    }
                }

            }

        }
    }

    if (ntd > 3)
    {
        //do estimate
        if ((info = lsq(H, L, 3, ntd, dx, Q)) || ((pdop = gnss_rtk_getpdop_prefix(H, 3, ntd)) > 6.0))
        {
            GLOGI("slpdet-td:lsq error,info=%d,ntd=%d pdop=%.3f", info, ntd, pdop);
        }
        else
        {
            for (i = 0; i < 3; i++) rtk->td_dx[i] = crr[i] - rtk->prr[i] + dx[i];
            //post-res
#ifndef ASM_MATMUL
            matmul("TN", ntd, 1, 3, 1.0, H, dx, -1.0, L);
#else
            Fast_Rtk_matmul("TN", ntd, 1, 3, 1.0, H, dx, -1.0, L);
#endif
            //sigma0
            rtk->tdsigma = sqrt(dot(L, L, ntd) / (ntd - 3.0));
            GLOGI("ddx [%8.3f %8.3f %8.3f] sigma=%.3f pdop=%.3f ntd=%d dX [%8.3f %8.3f %8.3f]", dx[0], dx[1], dx[2], rtk->tdsigma, pdop, ntd, rtk->td_dx[0], rtk->td_dx[1], rtk->td_dx[2]);
            if (rtk->tdsigma > 2.0 && bad_tdlist && fabs(rtk->tt) > 2.0 && (rtk->kfstatus&RTK_KF_RUN) && rtk->kfcnt > 10)
            {
                rtk->kfstatus |= RTK_KF_RESET_AMB;
                gnss_rtk_clean_prefix(rtk);
                GLOGI("abnormal td est res, reset amb");
            }
            //sign refsat,this will be used in lc_fusion
            if (NULL != rtk->ssat[refs - 1])
            {
                rtk->ssat[refs - 1]->tdcheck[reff] |= TD_CHECK_EST;
            }
            //post-res report
#ifdef PLAYBACK_MODE
            p += sprintf(p, "td-pres:");
#endif
            for (i = 0; i < ntd; i++)
            {
#ifdef PLAYBACK_MODE
                reject = 0;
#endif
                if (NULL == rtk->ssat[satv[i] - 1])
                {
                    continue;
                }
                rtk->ssat[satv[i] - 1]->tdcheck[freq[i]] |= TD_CHECK_EST;
                rtk->ssat[satv[i] - 1]->tdres[freq[i]] = L[i];
                if (fabs(L[i]) > rtk->tdsigma&&fabs(L[i]) > 0.1)
                {
#ifdef PLAYBACK_MODE
                    reject = 1;
#endif
                    rtk->ssat[satv[i] - 1]->slipflag[0][freq[i]] |= CYCSLIPDET_TDEST;
                }
#ifdef PLAYBACK_MODE
                p += sprintf(p, "%c[%u-%2u]%7.3f", reject ? '*' : ' ', freq[i], satv[i], L[i]);
                if ((i + 1) % 9 == 0)
                {
                    *p = '\0';
                    GLOGI("%s", str);
                    if (i < ntd - 1)
                    {
                        p = &str[0];
                        p += sprintf(p, "td-pres:");
                    }
                }
#endif
            }
#ifdef PLAYBACK_MODE
            if (ntd % 9)
            {
                *p = '\0'; GLOGI("%s", str);
            }
#endif
            ntdnew = 0;
            for (i = 0; i < ntd; i++)
            {
                if (NULL == rtk->ssat[satv[i] - 1])
                {
                    continue;
                }
                if((rtk->ssat[satv[i] - 1]->slipflag[0][freq[i]] & CYCSLIPDET_TDEST) != 0) continue;
                L[ntdnew] = L[i];
                ntdnew++;
            }
            gnss_MAD_DBL(L, dataabs, ntdnew, &medianout);
            j = ntdnew; ntdnew = 0;
            for (i = 0; i < j; i++)
            {
                if (dataabs[i] > 8 * (medianout / 0.6745)) continue;
                L[ntdnew] = L[i];
                ntdnew++;
            }
            for (i = 0; i < ntdnew; i++)
            {
                datay[i] = fabs(L[i]);
            }
            gnss_median_dbl(datay, ntdnew, &median);

            if (ntdnew > 5 && median < 0.02)
            {
                for (i = 0; i < ntd; i++)
                {
                    if (NULL == rtk->ssat[satv[i] - 1])
                    {
                        continue;
                    }
                    tdhcflag = 0;
                    lami = nav->lam[satv[i] - 1][freq[i]];
                    if (lami > 0 && (fabs(fabs(rtk->ssat[satv[i] - 1]->tdres[freq[i]] / lami) - 0.5) < 0.1))
                    {
                        tdhcflag = 1;
                        rtk->ssat[satv[i] - 1]->usectrl.halfflag |= FREQ_MASK(freq[i]);
                    }
                    GLOGD("td-pres-hc: median %.3f, reject %.3f, sat %03d freq %d cycle %.3f rejflag %d", 
                        median, 8 * (medianout / 0.6745), satv[i], freq[i], fabs(rtk->ssat[satv[i] - 1]->tdres[freq[i]] / lami), tdhcflag);
                }
            }
        }

    }
    if (L) Sys_Free(L);
    if (H) Sys_Free(H);
    if (satv) Sys_Free(satv);
    if (freq) Sys_Free(freq);
    if (datay) Sys_Free(datay);
    if (dataabs) Sys_Free(dataabs);

lab_tdinfo:
    /* ref info update */
    for (i = nu; i < n_all; i++)
    {
        if (NULL == rtk->ssat[obs[i].sat - 1])
        {
            continue;
        }
        for (f = 0; f < nf && f < NFREQ; f++)
        {
            rtk->ssat[obs[i].sat - 1]->td[1].y[f] = y[f + i * nf * 2];
            rtk->ssat[obs[i].sat - 1]->td[1].y[f+nf] = y[f + nf + i * nf * 2];
        }
        for (k = 0; k < 3; k++)
        {
            rtk->ssat[obs[i].sat - 1]->e[k] = e[k + i * 3];
        }
        //GLOGW("tdtd check all tor %.3f sat %03d y %.16f ctr %d", rtk->ptor, obs[i].sat, y[f + i * nf * 2], ctr.time);
    }

    /* td info update */
    nsat = 0;
    for (i = 0; i < ns; i++)
    {
        if (NULL == rtk->ssat[sat[i] - 1])
        {
            continue;
        }
        for (f = 0; f < nf && f < NFREQ; f++)
        {
            rtk->ssat[sat[i] - 1]->td[0].y[f] = y[f + iu[i] * nf * 2];
        }
        rtk->vsat[nsat++] = sat[i];
    }
    rtk->nsat = nsat;
    rtk->pt[0] = ctu;
    rtk->pt[1] = ctr;
    //save prr
    for (i = 0; i < 3; i++) rtk->prr[i] = crr[i];
}
static void gnss_rtk_slip_base_ref(rtk_t* rtk, const unsigned char* satp, const double* y, int comsat, const unsigned char* iup,
    const nav_t* nav, int nf, const double* azel)
{
    int i = 0, j = 0, k = 0;
    int f = 0, m = 0, refs = 0, isys = 0, iuref = 0;
    double residual = 0.0, lami = 0.0, lamj = 0.0, ele = 0.0, prey = 0.0, cury = 0.0;
    double *res = NULL, mean = 0.0, std = 0.0;
    float toe = 0.0, toe_l = 0.0;

    if (comsat <= 0) return;
    res = zeros(comsat, 1);
    if (NULL == res) return;

    for (f = 0; f < nf; f++) /* for each freq */
    {
        for (m = 0; m < NSYSDD; m++) /* m=0:gps/qzs/sbs,1:glo,2:gal,3:bds */
        {
            for (j = 0; j < comsat; j++)
            {
                ele = 0.0; refs = 0;
                for (i = 0; i < comsat; i++)
                {
                    isys = satsys(satp[i], NULL);
                    if (!gnss_rtk_test_sys(isys, m)) continue;

                    if (y[f + iup[i] * nf * 2] == 0.0) //cur ref
                    {
                        continue;
                    }
                    if (NULL == rtk->ssat[satp[i] - 1])
                    {
                        continue;
                    }
                    if (rtk->ssat[satp[i] - 1]->td[1].y[f] == 0.0) //last ref
                    {
                        continue;
                    }
                    //screen out large pr diff which may be introduced by wrong EPH
                    if ((rtk->ssat[satp[i] - 1]->usectrl.rqflag[0] & PE_MEAS_VALID_PR) == 0 && rtk->ssat[satp[i] - 1]->usectrl.pr_diff > 800)
                    {
                        continue;
                    }
                    if ((rtk->ssat[satp[i] - 1]->slipflag[1][f] & CYCSLIPDET_LLI))
                    {
                        continue;
                    }
                    if ((rtk->ssat[satp[i] - 1]->slipflag[1][f] & CYCSLIPDET_DDSAT))
                    {
                        continue;
                    }
                    if (nav->lam[satp[i] - 1][f] <= 0.0)
                    {
                        continue;
                    }
                    //find ref sat
                    if (azel[1 + iup[i] * 2] > ele)
                    {
                        ele = azel[1 + iup[i] * 2];  //should do as sys
                        refs = satp[i];
                        iuref = iup[i];
                    }
                }
                if (refs == 0 || NULL== rtk->ssat[refs - 1])
                {
                    break;
                }

                cury = y[f + iuref * nf * 2] - rtk->ssat[refs - 1]->td[1].y[f]; //ref
                lami = nav->lam[refs - 1][f];
                cury = cury / lami;

                k = 0;
                for (i = 0; i < comsat; i++)
                {
                    if (satp[i] == refs)
                    {
                        continue;
                    }
                    isys = satsys(satp[i], NULL);
                    if (!gnss_rtk_test_sys(isys, m)) continue;

                    if (y[f + iup[i] * nf * 2] == 0.0)
                    {
                        continue;
                    }
                    if (NULL == rtk->ssat[satp[i] - 1])
                    {
                        continue;
                    }
                    if (rtk->ssat[satp[i] - 1]->td[1].y[f] == 0.0)
                    {
                        continue;
                    }
                    lamj = nav->lam[satp[i] - 1][f];
                    if (lamj <= 0.0) continue;
                    /* residual */
                    prey = y[f + iup[i] * nf * 2] - rtk->ssat[satp[i] - 1]->td[1].y[f]; //non-ref
                    prey = prey / lamj;
                    residual = cury - prey;

                    toe = rtk->ssat[satp[i] - 1]->slp_ref.toe[0];
                    toe_l = rtk->ssat[satp[i] - 1]->slp_ref.toe[1];
                    if (FLT_IS_EQUAL(toe, toe_l))
                    {
                        res[k++] = residual;
                    }
                }

                toe = rtk->ssat[refs - 1]->slp_ref.toe[0];
                toe_l = rtk->ssat[refs - 1]->slp_ref.toe[1];
                gnss_math_dstd(res, k, &mean, &std);
                if ((fabs(mean) > 0.5 && std < 0.1 && k > 1) && FLT_IS_EQUAL(toe, toe_l))
                {
                    /*this sat should not be selected by reference sat*/
                    rtk->ssat[refs - 1]->slipflag[1][f] |= CYCSLIPDET_DDSAT;
                    GLOGI("slip detect base refsat slip sat=%d f=%d nsat=%d mean=%.3f std=%.3f", refs, f, k, mean, std);
                }
                else
                {
                    break;
                }
            }
        }
    }

    Sys_Free(res);
}
/* ref cycle slip detect -------------------------------------------------------------
* ref cycle slip detect
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_slpdet_ref(rtk_t *rtk, const nav_t *nav, int nu, int nr, const double *y, const double *e, const double *azel)
{
    double dt,ele = 0.0, cury, prey, residual,lami,lamj,dt1,dt2,thres=0.5;
    int i, j, m, k = 0, iuref = -1, refs = 0, isys, f = 0, nf = (rtk->opt.ionoopt == IONOOPT_IFLC) ? 1 : rtk->opt.nf, rejflag;
    float toe_l, toe, reftoe_l, reftoe;
    int rejfreq=0,rejall=0,cntfreq=0,cntall=0;
    unsigned char satp[MAXOBS] = { 0 }, iup[MAXOBS] = { 0 };
    obsd_t *refl = NULL, *refc = NULL;

    if(nr==0) return;

    dt1 = timediff(rtcm3data.obs[0].data[0].time, rtk->pt[1]);
    dt2 = timediff(rtcm3data.obs[1].data[0].time, rtk->pt[1]);
    dt = dt1 - dt2;  //dt = timediff(rtcm3data.obs[0].data[0].time, rtcm3data.obs[1].data[0].time);
    if (fabs(dt1) < 0.001 || fabs(dt2) > rtk->opt.maxtdiff || fabs(dt) > rtk->opt.maxtdiff || nr != rtcm3data.obs[0].n)
    {
        GLOGD("slpdet-ref: quit detect. dt=%.3f dt1=%.3f dt2=%.3f, nr=%d/%d", dt, dt1, dt2, rtcm3data.obs[0].n, nr);
        return;
    }
    GLOGD("slpdet-ref: dt=%.3f dt1=%.3f dt2=%.3f, nr=%d/%d", dt, dt1, dt2, rtcm3data.obs[0].n, nr);

    refc = rtcm3data.obs[0].data;
    refl = rtcm3data.obs[1].data;

    //find common sat
    for (i = 0, j = 0;i<rtcm3data.obs[0].n&&j<rtcm3data.obs[1].n;i++, j++)
    {
        if (refc[i].sat<refl[j].sat) j--;
        else if (refc[i].sat>refl[j].sat) i--;
        else
        {
            satp[k] = refc[i].sat;
            iup[k] = nu + i;  
            k++;
        }
    }

    if (k<2)
    {
        GLOGI("slpdet-ref: common sat too less(%d)", k);
        return;
    }

    gnss_rtk_slip_base_ref(rtk, satp, y, k, iup, nav, nf, azel);
    //make td obs
    for (f = 0; f < nf; f++) /* for each freq */
    {
      for (m = 0; m < NSYSDD; m++) /* m=0:gps/qzs/sbs,1:glo,2:gal,3:bds */
      {
        ele = 0.0; refs = 0;
        for (i = 0; i < k; i++)
        {
          isys = satsys(satp[i], NULL);
          if (!gnss_rtk_test_sys(isys, m)) continue;

          if (y[f + iup[i] * nf * 2] == 0.0) //cur ref
          {
            continue;
          }
          if (NULL == rtk->ssat[satp[i] - 1])
          {
            continue;
          }
          if (rtk->ssat[satp[i] - 1]->td[1].y[f] == 0.0) //last ref
          {
            continue;
          }

          //screen out large pr diff which may be introduced by wrong EPH
          if ((rtk->ssat[satp[i] - 1]->usectrl.rqflag[0] & PE_MEAS_VALID_PR) == 0 && rtk->ssat[satp[i] - 1]->usectrl.pr_diff > 800)
          {
            continue;
          }
          if ((rtk->ssat[satp[i] - 1]->slipflag[1][f] & CYCSLIPDET_LLI))
          {
            continue;
          }
          if ((rtk->ssat[satp[i] - 1]->slipflag[1][f] & CYCSLIPDET_DDSAT))
          {
            continue;
          }
          if (nav->lam[satp[i] - 1][f] <= 0.0)
          {
            continue;
          }
          toe = rtk->ssat[satp[i] - 1]->slp_ref.toe[0];
          toe_l = rtk->ssat[satp[i] - 1]->slp_ref.toe[1];
          if (FLT_ISNT_EQUAL(toe, toe_l))
          {
            continue;
          }
          //find ref sat
          if (azel[1 + iup[i] * 2] > ele)
          {
            ele = azel[1 + iup[i] * 2];  //should do as sys
            refs = satp[i];
            iuref = iup[i];
          }
        }
        if (refs == 0 || NULL == rtk->ssat[refs - 1])
        {
          continue;
        }
        rejfreq = 0; cntfreq = 0;

        cury = y[f + iuref * nf * 2] - rtk->ssat[refs - 1]->td[1].y[f]; //ref
        lami = nav->lam[refs - 1][f];
        cury = cury / lami;
        reftoe = rtk->ssat[refs - 1]->slp_ref.toe[0];
        reftoe_l = rtk->ssat[refs - 1]->slp_ref.toe[1];

        for (i = 0; i < k; i++)
        {
          if (satp[i] == refs)
          {
            continue;
          }
          isys = satsys(satp[i], NULL);
          if (!gnss_rtk_test_sys(isys, m)) continue;

          if (y[f + iup[i] * nf * 2] == 0.0)
          {
            continue;
          }
          if (NULL == rtk->ssat[satp[i] - 1])
          {
            continue;
          }
          if (rtk->ssat[satp[i] - 1]->td[1].y[f] == 0.0)
          {
            continue;
          }
          lamj = nav->lam[satp[i] - 1][f];
          if (lamj <= 0.0) continue;
          /* residual */
          prey = y[f + iup[i] * nf * 2] - rtk->ssat[satp[i] - 1]->td[1].y[f]; //non-ref
          prey = prey / lamj;
          residual = cury - prey;

          cntfreq++; cntall++;

          toe = rtk->ssat[satp[i] - 1]->slp_ref.toe[0];
          toe_l = rtk->ssat[satp[i] - 1]->slp_ref.toe[1];
          thres = azel[1 + iup[i] * 2] * R2D < 20.0 ? 1.25 : 0.5;
          if (dt > 30.0)thres = thres * 2;
          rejflag = 0;
          if (fabs(residual) > thres && FLT_IS_EQUAL(toe, toe_l))
          {
            rejfreq++; rejall++;
            rejflag = 1;
            rtk->ssat[satp[i] - 1]->slipflag[1][f] |= CYCSLIPDET_DDSAT;
          }

          rtk->ssat[satp[i] - 1]->slp_ref.slip[f] = (float)prey;
          rtk->ssat[satp[i] - 1]->tdcheck[f] |= DD_CHECK_REFSAT;

          if (rtk->rtkevent & RTK_BASECHANGE)
          {
            gnss_rtk_slip_offset(rtk, satp[i], f, prey);
            gnss_rtk_prefix_offset(rtk, satp[i], isys, f, residual);
          }

          GLOGD("slpdet-ref:%d %3d-%3d %9.3f %9.4f %8s %5.1f iu=%2d %6.0f %6.0f %d", f, refs, satp[i], prey,
            residual, rejflag ? "refslip" : "OK", azel[1 + iup[i] * 2] * R2D, iup[i], toe_l, toe, toe_l != toe);
        }
        //cntall++;

        rejflag = 0;
        if (rejfreq > cntfreq * 0.5 && fabs(cury) > 1.0 && FLT_IS_EQUAL(reftoe, reftoe_l))
        {
          //rejall++;
          rejflag = 1;
          rtk->ssat[refs - 1]->slipflag[1][f] |= CYCSLIPDET_DDSAT;
        }
        rtk->ssat[refs - 1]->slp_ref.slip[f] = (float)cury;
        rtk->ssat[refs - 1]->tdcheck[f] |= DD_CHECK_REFSAT;
        if (rtk->rtkevent & RTK_BASECHANGE)
        {
          gnss_rtk_slip_offset(rtk, refs, f, cury);
        }
        GLOGD("slpdet-ref:%d %3d     %9.3f     %2d/%2d %8s %5.1f ir=%2d %6.0f %6.0f %d", f, refs, cury, rejfreq, cntfreq,
          rejflag ? "refslip" : "OK", azel[1 + iuref * 2] * R2D, iuref, reftoe_l, reftoe, reftoe_l != reftoe);
      }
    }
    if (/*(rtk->shift.fixlist & 0x2) && (rtk->closkycnt & 0xF) == 0 && cntall > 5 &&
            rejall>cntall*0.8&&*/(rtk->rtkevent & RTK_BASECHANGE) && (rtk->kfstatus & RTK_KF_RUN))
    {
      rtk->kfstatus &= ~RTK_KF_RESET_AMB;
      GLOGI("do ref amb offset. rejall=%d cntall=%d kfstatus=%u", rejall, cntall, rtk->kfstatus);
    }
    return;
}
/* get rtk solution -------------------------------------------------------------
*
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
extern sol_t gnss_rtk_get_sol()
{
    return rtkctrl.sol;
}

/* calculate pdop for prefix sol -------------------------------------------------------------
*args   : double *A        I   transpose of(weighted) design matrix(n x m)
*         int n, m         I   number of parameters and measurements(n <= m)
* return : pdop
* author : none
*-----------------------------------------------------------------------------*/
extern double gnss_rtk_getpdop_prefix(double *H, int n, int m)
{
    int i, info;
    double pdop = 0.0, *HH;
    if (m < n) return pdop;

    HH = mat(n, n);
    if (HH == NULL) return pdop;
#ifndef ASM_MATMUL
    matmul("NT", n, n, m, 1.0, H, H, 0.0, HH);  /* Q=A*A' */
#else
    Fast_Rtk_matmul("NT", n, n, m, 1.0, H, H, 0.0, HH);  /* Q=A*A' */
#endif

    if (!(info = matinvsm(HH, n)))
    {
        for (i = 0; i < n; i++)
        {
            pdop += HH[i + i * n];
        }
        pdop = pdop > 0.0 ? sqrt(pdop) : 0.0;
    }

    Sys_Free(HH);

    return pdop;
}

/* gnss_rtk_firstStaticheck -------------------------------------------------------------
*if exit the first location, first_static_period=0,else first_static_period=1
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
static void gnss_rtk_firstStaticheck()
{
    ddres_t *resupdate = &rtkctrl.dd_res[DDRES_UPDATE];
    GLOGI("first_static_period = %d", first_static_period);
    if (first_static_period == 0) return;

    if ((peMode.staticData.historyStatic & 0xF) == 0x0 && peMode.staticData.lsVel > 0.2 &&
        (rtkctrl.kfstatus & RTK_KF_RUN) && rtkctrl.kfcnt > 10)
    {
        first_static_period = 0;
    }
    //else if ((g_pe_cfg.applyScenario != APPLY_SCENE_WATCH) && (rtkctrl.qcpasscnt & 0xF) == 0xF &&
    //	resupdate->np > 7 && resupdate->cntp > 4 && resupdate->rejp < 5 && rtkctrl.sol.qflag == 3)
    //{
    //	first_static_period = 0;
    //}
    else if ((rtkctrl.closkycnt & 0xF) == 0 && rtkctrl.llisatcnt[GPS_MODE] < 5 &&
        resupdate->np > 10 && resupdate->cntp > 10 && resupdate->rejp < 5 && rtkctrl.sol.qflag == 3)
    {
        first_static_period = 0;
    }
}
/* rtk position engine -------------------------------------------------------------
* rtk position engine
* args   : none
* return : none
* author : 
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_process(meas_blk_t* pMeas, asg_obs_t* asg_obs)
{
  GNSS_TIME* pTime = gnss_tm_get_time();

  rtkctrl.unc.prediff.preposf = 0;

  if (!rtkinitflag)
  {
    //GLOGE("RTK Fail: RTK Module not init yet!");
    GLOGE("RTK fail for not init!");
    return;
  }
  rtkctrl.sol.stat = SOLQ_NONE;

  if (!pTime || pTime->init == FALSE)
  {
    GLOGE("RTK Fail: Time not init yet!");
    return;
  }

  if (rtcm3data.obs[0].n < 6)
  {
    GLOGW("Ref Sat Count Less:%d", rtcm3data.obs[0].n);
    return;
  }

  if (rtkctrl.opt.mode == PMODE_SINGLE)
  {
    if (g_pe_cfg.ppk_mode) {
      gnss_rtk_get_init_rr(&rtkctrl.sol, gnss_gpst2time(pTime->week[GPS_MODE], pTime->rcvr_time[GPS_MODE]));
    }
    return;
  }

  rtkctrl.Qratio = 1.0;
  rtkctrl.rtkevent = 0;
  rtkctrl.kfextpol = 0;
  rtkctrl.shift.sollist <<= 1;
  if ((g_proc.flag & 0xFC01) > 0)
  {
    rtkctrl.shift.sppsollist <<= 1;
    rtkctrl.shift.dis2dList1 <<= 1;
    rtkctrl.shift.dis2dList2 <<= 1;
  }
  rtkctrl.nrefs = 0;
  rtkctrl.shift.fixlist <<= 1;
  rtkctrl.shift.flag &= ~SHIFT_FLAG_NO_FEEDBACK;
  rtkctrl.wl.flag <<= 1;

  if (!gnss_rtk_obs_update(rtkctrl.opt.adaptSeqEKF, pTime, pMeas, asg_obs))
  {
    return;
  }

  rtkctrl.rbflag = gnss_rtk_rb_update();

  gnss_rtk_slip_detect_pre(&rtkctrl);

  gnss_rtk_memcpy_rtkobs();

  ag_rtk_position(&rtkctrl, rtkobs.data, rtkobs.n, &rtknav);
#if 0
  gnss_rtk_obsl1_clean_check(&rtkctrl);
#endif
  gnss_rtk_fill_pvt(&rtkctrl, gpz_user_pvt);

  if (rtcm3data.base_switch) {
    rtcm3data.base_switch = 0;
  }

  //	if (rtkctrl.sol.stat!=SOLQ_NONE&&fp_solout) 
  //	{
  //		/* write solution */
  //		ag_rtk_solution_out(fp_solout,&rtkctrl.sol,rtkctrl.rb,&solopt);
  //	}
  gnss_rtk_firstStaticheck();
}

/* get rover init pos -------------------------------------------------------------
* get rover pos
* args   : none
* return : none
* author : 
*-----------------------------------------------------------------------------*/
extern int gnss_rtk_get_init_rr(sol_t *sol,gtime_t time)
{
    int j;
    double threskf=5.0,thresls=5.0;
    KF_PVT_INFO *kfpvt=gpz_kf_data->kf_Pvt_Info;
    PosBiasDet_PrDiff* p_PrDiffDec = &(peState.prDiffPosBiasDect);
    if (!kfpvt||kfpvt->kfFixStatus!=FIX_STATUS_NEW)
    {
        sol->stat=SOLQ_NONE;
        sol->time=time;
        return 0;
    }
    
    sol->type=0;
    sol->time=time;
    sol->dtr[0]=kfpvt->clkBias[GPS_MODE]/CLIGHT; /* receiver clock bias (s) */
    sol->dtr[1]=kfpvt->clkBias[GLN_MODE]/CLIGHT; /* glo-gps time offset (s) */
    sol->dtr[2]=kfpvt->clkBias[GAL_MODE]/CLIGHT; /* gal-gps time offset (s) */
    sol->dtr[3]=kfpvt->clkBias[BDS_MODE]/CLIGHT; /* bds-gps time offset (s) */
    if ((g_proc.flag & 0xFC01) > 0)
    {
        for (j = 0; j < 3; j++) sol->rr[j] = kfpvt->ecefPos[j];
    }
    else
    {
        for (j = 0; j < 3; j++) sol->rr[j] = gpz_user_pvt->ecef.pos[j] - kfpvt->ecefVel[j] * g_proc.dt;
    }
    for (j=3;j<6;j++) sol->rr[j]=kfpvt->ecefVel[j-3];
    
    for (j=0;j<6;j++) sol->qr[j]= kfpvt->ecefPosPmat[j];

    sol->ns= gpz_kf_data->kf_ctrl.prNum - gpz_kf_data->kf_ctrl.prRejNum;
    sol->age=sol->ratio=sol->thres=0.0;
    sol->stat=SOLQ_SINGLE;

    /* pos,pr quality check */
    sol->qflag=0;
    if (g_pe_cfg.automobile==0)
    {
        threskf=10.0;
        thresls=10.0;

        if ((peMode.userContext.context == USER_CONTEXT_VEHICLE) && ((gpz_kf_data->kf_Pvt_Info->kfHeadingVel < 1) ||
                                                                     ((sumofbit1(peMode.staticData.historyStaticLong, 16) > 0) && (gpz_kf_data->kf_Pvt_Info->kfHeadingVel < 3.0))))
        {
            //if (p_PrDiffDec != NULL)
            {
                if ((p_PrDiffDec->smoothPrDiffStd > 6.0) || (((double)gpz_kf_data->kf_ctrl.prDewNum + gpz_kf_data->kf_ctrl.prRejNum) / (gpz_kf_data->kf_ctrl.prNum) > 0.4)) return 1;

                for (j = 0; j < MAX_SMOOTH_NUM - 1; j++)
                {
                    if (j < MAX_SMOOTH_NUM - 3) continue;
                    if (p_PrDiffDec->prDiffBack[j] > 6.5) return 1;
                }
            }
        }
    }
    if ((gpz_kf_data->posRes > 0.0 && gpz_kf_data->posRes < threskf) || (gpz_kf_data->posResStd > 0.0 && gpz_kf_data->posResStd < threskf))
    {
        sol->qflag|=(QFLAG_PVT_KFPOS_GOOD|QFLAG_PVT_PRRES_GOOD);
    }
    else if (gpz_kf_data->lsBlk && gpz_kf_data->lsBlk->ls_Pvt_Info->valid &&
             gpz_kf_data->lsBlk->pos_res > 0.0 && gpz_kf_data->lsBlk->pos_res < thresls)
    {
        sol->qflag|=QFLAG_PVT_PRRES_GOOD;
    }
    
    return 1;
}
/* open sky detect -------------------------------------------------------------
* open sky detect
* args   : none
* return : none
* author : 
*-----------------------------------------------------------------------------*/
extern int gnss_rtk_opensky_check(rtk_t *rtk)
{
    int scene=SCE_SEMISKY,llisat=0,gnss,nollicnt[GNSS_MAX_MODE]={0};
    uint32_t avgcn0=35,syscnt=0,nsat_cls=10,nsat_opn=10,nsat_nolli=7,satcnt=0;
    float cno35pct=0.5;
    meas_blk_t*    pMeas=NULL;

    if (!gpz_kf_data->meas_blk)
    {
        GLOGE("opensky check: get meas block pointer as NULL");
        return 0;
    }
    /* 0.open sky scenario detect as per: cn0, cycle slip count, sat number */
    pMeas=gpz_kf_data->meas_blk;
    for (gnss=GPS_MODE;gnss<GNSS_MAX_MODE;gnss++)
    {
        llisat+=rtk->llisatcnt[gnss];
        satcnt+=rtk->satcnt[gnss];
        nollicnt[gnss]=rtk->satcnt[gnss]-rtk->llisatcnt[gnss];
        if (rtk->satcnt[gnss]>0) syscnt++;
    }
    rtk->nollicnt=satcnt-llisat;
    if (g_pe_cfg.automobile==0)
    {
        avgcn0=30;
        cno35pct=(float)0.4;
    }
    if (syscnt==1)
    {
        nsat_cls=4;
        nsat_opn=6;
    }
    else if (syscnt==2)
    {
        nsat_cls=8;
        nsat_opn=12;
    }
    else if (syscnt>=3)
    {
        nsat_cls=12;
        nsat_opn=18;
        nsat_nolli=10;
        if (g_pe_cfg.applyScenario == APPLY_SCENE_DRONE && (rtk->llisatcnt[GPS_MODE]<rtk->satcnt[GPS_MODE] * 0.3) && 
            nollicnt[GPS_MODE]>4 && rtk->nollicnt>7)
        {
            nsat_cls = 8;
            nsat_opn = 12;
        }
        else if (g_pe_cfg.applyScenario == APPLY_SCENE_AERO)
        {
            nsat_cls = 10;
            nsat_opn = 16;
        }
    }
    /*如果satcnt>=0,考虑将satcnt定义为uint32_t,否则将nsat_cls定义为int*/
    if (satcnt <= nsat_cls || rtk->satcnt[GPS_MODE]<4 ||/*(g_pe_cfg.applyScenario != APPLY_SCENE_AERO && g_pe_cfg.sub_chipType != SUB_CHIPTYPE_BRCM_47755 &&
            rtk->satcnt[BDS_MODE]>0&&rtk->satcnt[BDS_MODE]<4)||*/
        ((pMeas->maxCno < 40 || pMeas->avgCno < 30 || pMeas->Cno35Cnt[GPS_MODE] < rtk->satcnt[GPS_MODE] * 0.25) && llisat > satcnt*0.5) ||
        llisat>satcnt*0.7 || (rtk->llisatcnt[GPS_MODE] > rtk->satcnt[GPS_MODE] * 0.5&&nollicnt[GPS_MODE] <= 4 && rtk->nollicnt < nsat_nolli))
    {
        scene = SCE_CLOSESKY;
    }
    else if ((satcnt > nsat_opn && rtk->satcnt[GPS_MODE] > 5 &&
        pMeas->avgCno >= avgcn0 && pMeas->Cno35Cnt[GPS_MODE] > rtk->satcnt[GPS_MODE] * cno35pct &&
        llisat<satcnt * 0.4 && rtk->llisatcnt[GPS_MODE] < rtk->satcnt[GPS_MODE] * 0.4 && nollicnt[GPS_MODE]>4 && rtk->nollicnt>nsat_nolli)
        || (rtk->satcnt[GPS_MODE] >= 8 && pMeas->Cno35Cnt[GPS_MODE] >= 8 && rtk->llisatcnt[GPS_MODE] <= 2 && pMeas->Cno35Cnt[GPS_MODE] >= 0.75 * rtk->satcnt[GPS_MODE]))
    {
        scene = SCE_OPENSKY;
    }
    if (scene==SCE_OPENSKY&&sumofbit1(rtk->opt.navsys,32)>1&&syscnt<=1&&llisat>2)
    {
        scene=SCE_SEMISKY;
    }
    rtk->qcpasscnt<<=1;
    rtk->closkycnt<<=1;
    if (scene==SCE_OPENSKY)
    {
        rtk->qcpasscnt|=1;
    }
    else if (scene==SCE_CLOSESKY)
    {
        rtk->closkycnt|=1;
    }

    rtk->trstCloCnt = rtk->trstCloCnt > 0 ? (rtk->trstCloCnt - 1) : 0;
    if (rtk->trstCloCnt > 0 && (rtk->qcpasscnt & 0x1) == 1)
    {
        rtk->trstCloCnt = 0;
    }
    if ((rtk->closkycnt & 0x1) == 1 && sumofbit1(rtk->closkycnt, 5) <= 3 && rtk->trstCloCnt == 0/*&&rtk->kfstatus==RTK_KF_RUN*/
        && (pMeas->avgCno >= 32 && rtk->noSkipCnt < 0.3*satcnt && satcnt>=20)
        )
    {
        rtk->trstCloCnt = 10;
    }

#if defined(_WIN32)
    GLOGI("scene=%d open=0x%I64x clo=0x%I64x avg=%u max=%u cn035[%u %u %u %u] sat[%u %u %u %u] lli[%u %u %u %u] sysc=%d kfstat=%u qflag=%u trstclo=%d",
        scene, rtk->qcpasscnt, rtk->closkycnt, pMeas->avgCno, pMeas->maxCno, 
        pMeas->Cno35Cnt[GPS_MODE], pMeas->Cno35Cnt[GLN_MODE], pMeas->Cno35Cnt[GAL_MODE], pMeas->Cno35Cnt[BDS_MODE],
        rtk->satcnt[GPS_MODE], rtk->satcnt[GLN_MODE], rtk->satcnt[GAL_MODE], rtk->satcnt[BDS_MODE],
        rtk->llisatcnt[GPS_MODE], rtk->llisatcnt[GLN_MODE], rtk->llisatcnt[GAL_MODE], rtk->llisatcnt[BDS_MODE],
        syscnt, rtk->kfstatus, rtk->sol.qflag, rtk->trstCloCnt);
#else
    SYS_LOGGING(OBJ_ALGO, LOG_INFO, "scene=%d open=0x%jx clo=0x%jx avg=%u max=%u cn035[%u %u %u %u] sat[%u %u %u %u] lli[%u %u %u %u] sysc=%d kfstat=%u qflag=%u",
        scene, rtk->qcpasscnt, rtk->closkycnt, pMeas->avgCno, pMeas->maxCno, 
        pMeas->Cno35Cnt[GPS_MODE], pMeas->Cno35Cnt[GLN_MODE], pMeas->Cno35Cnt[GAL_MODE], pMeas->Cno35Cnt[BDS_MODE],
        rtk->satcnt[GPS_MODE], rtk->satcnt[GLN_MODE], rtk->satcnt[GAL_MODE], rtk->satcnt[BDS_MODE],
        rtk->llisatcnt[GPS_MODE], rtk->llisatcnt[GLN_MODE], rtk->llisatcnt[GAL_MODE], rtk->llisatcnt[BDS_MODE],
        syscnt, rtk->kfstatus, rtk->sol.qflag);
#endif

    return 1;
}
/* check if it is time to start kf -------------------------------------------------------------
* kf stauts check
* args   : none
* return : none
* author : 
*-----------------------------------------------------------------------------*/
extern int gnss_rtk_kf_start_check(rtk_t *rtk)
{
    int quickStart = FALSE;

    if (rtk->kfstatus&RTK_KF_RUN)
    {
        return 1;
    }

    quickStart = gnss_rtk_quick_start_check(rtk);

    if (g_pe_cfg.ppk_mode == 1 && peMode.staticData.staticFlag == 0 && (rtk->qcpasscnt & 0x1) == 0x1/*(rtk->closkycnt & 0x1) == 0x0*/)
    {
        return 1;
    }

    if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_BRCM_47755 && peMode.staticData.staticFlag == 0)
    {
        if ((rtk->closkycnt & 0xFFF) != 0) return 0;
        /*if (!((peMode.userContext.context == USER_CONTEXT_VEHICLE) && ((gpz_kf_data->kf_Pvt_Info->kfHeadingVel < 1) ||
            ((sumofbit1(peMode.staticData.historyStaticLong, 16) > 0) && (gpz_kf_data->kf_Pvt_Info->kfHeadingVel < 3.0)))))
        {
            if ((rtk->closkycnt & 0x3) == 0) return 1;
        }*/
    }

    /* 1.open sky consistence check */
    if ((rtk->closkycnt&0xF)!=0x0 && quickStart == FALSE)  //(rtk->qcpasscnt&0xF)!=0xF
    {
        return 0;
    }

    if ((g_pe_cfg.chipType == UBLOX && g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100)
        && (gpz_kf_data->posRes < 1.5 && gpz_kf_data->posResStd < 1.5) && rtk->nollicnt >= 10 && peMode.staticData.lsVel > 2.0)
    {
        return 1;
    }

    GLOGI("start check: static=%u his=%x clo=%x",peMode.staticData.staticFlag,peMode.staticData.historyStatic,rtk->closkycnt);

    if (g_pe_cfg.chipType != QCOM 
        && (peMode.staticData.staticFlag || sumofbit1(peMode.staticData.historyStatic, 5) >= 2) 
        && ((rtk->qcpasscnt & 0xF) != 0xF || sumofbit1(rtk->closkycnt, 32) >= 10))
    {
        if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100 && quickStart == TRUE)
        {
            GLOGI("consider quick start rtk!");
        }
        else
        {
            return 0;
        }
    }

    if (((sumofbit1(rtk->closkycnt,32)>=20||sumofbit1(rtk->closkycnt,12)>=8)&&sumofbit1(rtk->qcpasscnt,4)<2) && quickStart == FALSE)
    {
        return 0;
    }
    if (peMode.staticData.staticFlag&&rtk->kfcnt>0&&rtk->kfcnt<5)
    {
        if (rtk->kfstartwait<10)
        {
            rtk->kfstartwait++;
            GLOGI("rtk start wait. cnt=%d",rtk->kfstartwait);
            return 0;
        }
    }
    /* 2.check pos and pr res quality */
    if (rtk->sol.stat == SOLQ_NONE) {
        return 0;
    }

    if ((rtk->qcpasscnt & 0xF) == 0xF && sumofbit1(rtk->closkycnt, 32) < 10 && quickStart == TRUE)
    {
        return 1;
    }
    
    if ((rtk->kfstatus==RTK_KF_IDLE)&&
        (rtk->sol.qflag&(QFLAG_PVT_PRRES_GOOD|QFLAG_PVT_KFPOS_GOOD))==(QFLAG_PVT_PRRES_GOOD|QFLAG_PVT_KFPOS_GOOD))
    {
        rtk->kfstatus=RTK_KF_INIT;
        return 1;
    }
    if ((rtk->kfstatus&RTK_KF_RESET)&&
        (rtk->sol.qflag&(QFLAG_PVT_PRRES_GOOD|QFLAG_PVT_KFPOS_GOOD))==(QFLAG_PVT_PRRES_GOOD|QFLAG_PVT_KFPOS_GOOD))
    {
        return 1;
    }
    
    return 0;
    //return 1; /* disable it now */
}
extern int gnss_rtk_quick_start_check(rtk_t *rtk)
{
    uint32_t minAvgCno = 30;
    int quickStart = FALSE;
    int gnss = 0, syscnt = 0, pvtsatcnt = 0, rejsatcnt = 0, satcnt = 0;
    int llisat = 0/*, nollisat = 0*/;

    for (gnss = GPS_MODE; gnss < GNSS_MAX_MODE; gnss++)
    {
        pvtsatcnt += rtk->pvtsatcnt[gnss];
        satcnt += rtk->satcnt[gnss];
        llisat += rtk->llisatcnt[gnss];
        if (rtk->satcnt[gnss] > 0) syscnt++;
    }

    rejsatcnt = satcnt - pvtsatcnt;
    //nollisat = satcnt - llisat;

    if (gpz_kf_data->meas_blk && gpz_kf_data->lsBlk)
    {
        if (gpz_kf_data->meas_blk->avgCno > minAvgCno)
        {
            if (syscnt == 1 && pvtsatcnt > 5)
            {
                quickStart = TRUE;
            }
            else if (syscnt == 2 && pvtsatcnt > 8)
            {
                quickStart = TRUE;
            }
            else if (syscnt == 3 && pvtsatcnt > 12)
            {
                quickStart = TRUE;
            }
            else if (syscnt == 4 && pvtsatcnt > 15 &&
                (gpz_kf_data->lsBlk->DoP.pDOP < 1.0 || ((llisat <= 5 || rejsatcnt < 15) && gpz_kf_data->posRes < 2.0 && gpz_kf_data->lsBlk->DoP.pDOP < 1.5)))
            {
                quickStart = TRUE;
            }
        }
        if (quickStart == TRUE)
        {
            GLOGI("pvtsatcnt is available, quickStart is TRUE");
        }
    }

    if (gpz_kf_data->meas_blk && gpz_kf_data->lsBlk && quickStart == FALSE)
    {
        if (gpz_kf_data->meas_blk->avgCno > minAvgCno)
        {
            if (satcnt >= 15)
            {
                if (rtk->noSkipCnt >= 11)
                {
                    quickStart = TRUE;
                }
            }
            else if (satcnt >= 12)
            {
                if (rtk->noSkipCnt >= 9)
                {
                    quickStart = TRUE;
                }
            }
            else if (satcnt >= 9)
            {
                if (rtk->noSkipCnt >= 7)
                {
                    quickStart = TRUE;
                }
            }
        }
        if (quickStart == TRUE)
        {
            GLOGI("noSkipCnt is available, quickStart is TRUE");
        }
    }

    return quickStart;
}
/* ddres check -------------------------------------------------------------
* ddres check
* args   : none
* return : none
* author : 
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_kf_ddres_check(int soltype,rtk_t *rtk,const double *v,const int *vflg,int nv,int ns)
{
    int i/*,sat1*/,sat2,type,freq,nc=0,np=0,cntc=0,cntp=0,madc=0,madp=0,nfix=0,cp_pr_cnt=0;
    double rmsc=0.0,rmsp=0.0,stdc=0.0,stdp=0.0,rmsfixc=0.0,rmsfixp=0.0;
    unsigned char flagc=0,flagp=0;
    ddres_t *pddres=NULL;
    uint8_t flag = FALSE;
    double *vc = NULL, *vp = NULL, var_median_vc, var_median_vp,var_median_vc2=-1.0, var_median_vp2=-1.0;

    pddres = &rtk->dd_res[soltype];

    if (soltype == DDRES_SINGLE)
    {
        if ((rtk->qcpasscnt & 0x3)!=0x3 ||
            (g_pe_cfg.sub_chipType==SUB_CHIPTYPE_ST_8100 && (peMode.staticData.historyStatic&0xF)==0xF && rtcm3data.ref_freqmask>1)) {
            rtk->rtduseflag = 1;
        }
        else {
            rtk->rtduseflag = 0;
        }
    }
    cp_pr_cnt = rtk->cp_pr_cnt;
    if (cp_pr_cnt<1)
    {
        pddres->nc = pddres->np = 0;
        pddres->cntc = pddres->cntp = 0;
        pddres->ncmad = pddres->npmad = 0;
        pddres->rmsc = pddres->rmsp = 0;
        pddres->stdc = pddres->stdp = 0;
        GLOGI("ddres check: no valid ddres. type=%d nv=%d", soltype, nv);
        if (soltype == DDRES_PREDICT && rtk->rtduseflag && nv - cp_pr_cnt>0)
        {
            pddres->flag = DDRESFLAG_RTD_DRONLY;
        }
        return;
    }

    vc = mat(cp_pr_cnt, 1);vp = mat(cp_pr_cnt, 1);
    if (vc == NULL || vp == NULL) {
        goto DDRES_CHECK_QUIT;
    }
    
    for (i=0;i<cp_pr_cnt;i++)
    {
        //sat1=(vflg[i]>>16)&0xFF;
        sat2=(vflg[i]>> 8)&0xFF;
        type=(vflg[i]>> 4)&0xF;
        freq=vflg[i]&0xF;

        /* TODO: screen out the outliers using MAD? */
        if (type==0)
        {
            rmsc+=v[i]*v[i];
            vc[nc] = fabs(v[i]);
            nc++;
            if (fabs(v[i])<1.0) cntc++;
        }
        else
        {
            rmsp+=v[i]*v[i];
            vp[np] = fabs(v[i]);
            np++;
            if (fabs(v[i])<5.0) cntp++;
        }
        if (NULL == rtk->ssat[sat2 - 1])
        {
            continue;
        }
        if (soltype == DDRES_FIXED && rtk->ssat[sat2 - 1]->fix[freq < NFREQ ? freq : 0] > 1) {
            if (type == 0)
            { 
                rmsfixc += v[i] * v[i];
                nfix++;
            }
            else
                rmsfixp += v[i] * v[i];
        }

    }

    /*calculate mad info*/
    flag = gnss_median_dbl(vc, nc, &var_median_vc);
    if (flag)
    {
        var_median_vc2 = 10.0*var_median_vc / 0.6745;
        for (i = 0; i< nc; i++)
        {
            if (vc[i] <= var_median_vc2)
            {
                stdc += vc[i] * vc[i];
                madc++;
            }
        }
    }
    
    //flag = FALSE;
    flag = gnss_median_dbl(vp, np, &var_median_vp);
    if (flag)
    {
        var_median_vp2 = var_median_vp / 0.6745;
        for (i = 0;i < np;i++)
        {
            if (vp[i] <= var_median_vp2)
            {
                stdp += vp[i] * vp[i];
                madp++;
            }
        }
    }

    /*calculate fix info*/
    if (nfix > 0)
    {
        rmsfixc = sqrt(rmsfixc / nfix);
        rmsfixp = sqrt(rmsfixp / nfix);
        pddres->rmsfc = (float)rmsfixc;
        pddres->rmsfp = (float)rmsfixp;
    }

    pddres->nc=nc;
    pddres->np=np;
    pddres->cntc=cntc;
    pddres->cntp=cntp;
    pddres->ncmad = madc;
    pddres->npmad = madp;

    pddres->flag=DDRESFLAG_CHECKVALID; //mark as valid ddres check

    if(nc>0) rmsc=sqrt(rmsc/nc);
    if(np>0) rmsp=sqrt(rmsp/np);
    if (madc != 0) stdc = sqrt(stdc / madc);
    if (madp != 0) stdp = sqrt(stdp / madp);

    flagc = (rmsc<1.0 || cntc>nc / 2 || cntc > 5) && (DBL_ISNT_EQUAL(rmsc, 0.0));
    flagp = (rmsp < 5.0 || (cntp > np / 2 && cntp > 2) || (rmsp < 8.0&&cntp >= np / 2)) && (DBL_ISNT_EQUAL(rmsp, 0.0));

    pddres->rmsc = (DBL_ISNT_EQUAL(rmsc, 0.0)) ? (float)rmsc : 100.0f;
    pddres->rmsp = (DBL_ISNT_EQUAL(rmsp, 0.0)) ? (float)rmsp : 100.0f;
    pddres->stdc = (DBL_ISNT_EQUAL(stdc, 0.0)) ? (float)stdc : 100.0f;
    pddres->stdp = (DBL_ISNT_EQUAL(stdp, 0.0)) ? (float)stdp : 100.0f;

    if ((pddres->rejc >= ns / 2 || pddres->rejp >= ns / 2) && (cntc < 5 && cntp < 5)) /* (ns-np>=ns/2||ns-nc>=ns/2)&&cntc<3&&cntp<3&&nv<5 */
    {
        pddres->flag |= DDRESFLAG_REJTOOMUCH;
        if (soltype == DDRES_SINGLE)
        {
            rtk->rtduseflag = 1;
        }
        GLOGI("ddres check: reject too much. ddtype=%d ns=%d [nc=%d cntc=%d rejc=%d %.3f] [np=%d cntp=%d rejp=%d %.3f] kfstatus=%d, kfcnt=%d",
            soltype, ns, nc, cntc, pddres->rejc, rmsc, np, cntp, pddres->rejp, rmsp, rtk->kfstatus, rtk->kfcnt);
        if (vc) Sys_Free(vc);
        if (vp) Sys_Free(vp);
        return;
    }

    if (flagc) pddres->flag|=DDRESFLAG_CP_GOOD;
    if (flagp) pddres->flag|=DDRESFLAG_PR_GOOD;
    pddres->flag|=DDRESFLAG_RMSCPVALID|DDRESFLAG_RMSPRVALID;
    
    GLOGI("ddres check: type=%d ns=%d [nc=%d cntc=%d %.3f flagc=%d] [np=%d cntp=%d %.3f flagp=%d] kfstatus=%d, kfcnt=%d",
        soltype,ns,nc,cntc,rmsc,flagc,np,cntp,rmsp,flagp,rtk->kfstatus,rtk->kfcnt);

    GLOGI("ddres check: mad type=%d [varmadc=%.3f nc=%d madc=%d stdc=%.3f] [varmadp=%.3f np=%d madp=%d stdp=%.3f]", soltype,var_median_vc2, nc, madc,stdc, var_median_vp2, np, madp, stdp);
    
    if (nfix > 0)
    {
        GLOGI("ddres check: fix nfix=%d rmsc=%.3f rmsp=%.3f",nfix, rmsfixc, rmsfixp);
    }

DDRES_CHECK_QUIT:
    if (vc) Sys_Free(vc);
    if (vp) Sys_Free(vp);
}
/* kf reset check -------------------------------------------------------------
* kf stauts check
* args   : none
* return : none
* author : 
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_kf_reset_check(rtk_t *rtk,int ns)
{
    int nc=0,np=0,cntc=0,cntp=0,vflag=0;
    double rmsc=0.0,rmsp=0.0,stdc=0.0,stdp=0.0, argu = 2.0;
    unsigned char flagc=0,flagp=0;
    ddres_t *pddres=NULL;
    unsigned char kfstatus_old = rtk->kfstatus;
    //uint8_t judge = 0;

    if (rtk->kfstatus==RTK_KF_RESET)  /*||peMode.staticData.staticFlag||rtk->kfcnt<20*/
    {
        return;
    }
    pddres=&rtk->dd_res[DDRES_UPDATE];
    if (pddres->flag==0)
    {
        if (rtk->rtduseflag && rtk->dd_res[DDRES_PREDICT].flag == DDRESFLAG_RTD_DRONLY && rtk->shift.dis2D[0] < 2.0)
        {
            GLOGI("reset check: only dr update in rtd mode, not reset RTK");
            return;
        }
        rtk->kfstatus=RTK_KF_RESET;
        GLOGI("reset check: have no post-ddres, reset kf");
        return;
    }

    nc=pddres->nc;
    np=pddres->np;
    cntc=pddres->cntc;
    cntp=pddres->cntp;
    rmsc = pddres->rmsc;
    rmsp = pddres->rmsp;
    flagc = pddres->flag&DDRESFLAG_CP_GOOD;
    flagp = pddres->flag&DDRESFLAG_PR_GOOD;

    if ((sumofbit1(rtk->closkycnt,32)>24&&(rtk->closkycnt&0xFF)==0xFF&&(g_pe_cfg.meas_type_mask&MEAS_TYPE_MASK_DR)&&peMode.staticData.lsVel<3.0&&cntp<5) ||
        (g_pe_cfg.applyScenario == APPLY_SCENE_AUTOROOF && (rtk->closkycnt & 0xFFF) == 0xFFF && rtk->nollicnt < 3 && pddres->nc < 4))
    {
        rtk->kfstatus=RTK_KF_RESET;
        GLOGI("reset check: closkycnt too much, reset kf");
        goto LAB_RESET_CHECK;
        //return;
    }
    
    //judge = 1;/*disable this condition*/

    if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_BRCM_47755) argu = 4.0;

    if (((pddres->flag&DDRESFLAG_REJTOOMUCH) /*&& judge*/) ||
        ((rtk->qcpasscnt & 0xFF) == 0 && np < ns / argu && rtk->kfcnt < 4))
    {
        rtk->kfstatus=RTK_KF_RESET;
        GLOGI("reset check: too many obs rejected. nc=%d np=%d, ns=%d",nc,np,ns);
        goto LAB_RESET_CHECK;
        //return;
    }

    if (peMode.staticData.staticFlag)
    {
        if (!flagp && (gpz_kf_data->kf_ctrl.prNum - gpz_kf_data->kf_ctrl.prRejNum) > 5 && gpz_kf_data->posRes > 0.0)
        {
            if ((cntp<5 &&rmsp>6.0 && gpz_kf_data->posRes < 5.0 && (rtk->qcpasscnt & 0xF) == 0xF && rtk->kfcnt < 10) ||
                (cntp<3 &&rmsp>10.0 && gpz_kf_data->posRes < 10.0 && sumofbit1(rtk->closkycnt, 6) >= 3))
            {
                rtk->kfstatus=RTK_KF_RESET;
            }
        }
        
        if (!flagc&&flagp && rmsc > 2.0 && rtk->kfstatus == RTK_KF_RUN)
        {
            rtk->kfstatus |= RTK_KF_RESET_AMB;
        }
        vflag = -1; //invalid
        GLOGI("static reset check: spp(%u %.3f) kfstatus=%d, kfcnt=%d", gpz_kf_data->kf_ctrl.prNum - gpz_kf_data->kf_ctrl.prRejNum, gpz_kf_data->posRes, rtk->kfstatus, rtk->kfcnt);
        goto LAB_RESET_CHECK;
        //return;
    }

    //judge = 1;/*disable this condition*/
    
    //if (judge)
    {
        if (flagc && !flagp)
        {
            rtk->kfstatus = RTK_KF_RESET;
        }
        else if (!flagc&&flagp)
        {
            if (rtk->kfstatus == RTK_KF_RUN)
            {
                rtk->kfstatus |= RTK_KF_RESET_AMB;
            }
            else
            {
                rtk->kfstatus = RTK_KF_RESET;
            }
        }
        else if (!flagc && !flagp)
        {
            rtk->kfstatus = RTK_KF_RESET;
        }
        else
        {
            vflag = 1;
        }
    }

LAB_RESET_CHECK:

    if ((rtk->kfstatus&(RTK_KF_RESET | RTK_KF_RESET_AMB)) && g_pe_cfg.sub_chipType != SUB_CHIPTYPE_BRCM_47755)
    {
        if ((rtk->nbias > 5 && rtk->sol.ratio > 8.0&&rtk->nfix > 3 && sumofbit1(rtk->closkycnt, 8) < 6 && rtk->dd_res[DDRES_FIXED].rmsc<1.0) ||
            (pddres->ncmad > 5 && pddres->npmad > 5 && pddres->stdc < 0.05 && pddres->stdp < 15.0) ||
            (pddres->ncmad > 5 && pddres->npmad > 3 && pddres->stdc < 0.05 && pddres->stdp < 4.0))
        {
            GLOGI("reset check: reset back,tor=%.6f kfstatus=%d [nc=%d madc=%d stdc=%.3f] [np=%d madp=%d stdp=%.3f]", rtk->ptor,
                rtk->kfstatus, pddres->nc, pddres->ncmad, pddres->stdc, pddres->np, pddres->npmad, pddres->stdp);
            rtk->kfstatus = kfstatus_old;
        }
        else if (rtk->rtduseflag && np > 5 && rmsp < 20 && rmsp > 1e-6 && gpz_kf_data->meas_blk->avgCno > 20)
        {
            GLOGI("reset check: reset back-2,tor=%.6f kfstatus=%d",rtk->ptor, rtk->kfstatus);
            rtk->kfstatus = kfstatus_old;
        }
        else if (rtk->rtduseflag && rtk->shift.dis2D[0] < 5.0 && gpz_kf_data->meas_blk->avgCno > 20)
        {
            GLOGI("reset check: reset back-3,tor=%.6f kfstatus=%d",rtk->ptor, rtk->kfstatus);
            rtk->kfstatus = kfstatus_old;
        }
    }
    //if (rtk->ptor >  284375.0&&rtk->ptor <  284377.0) rtk->kfstatus = RTK_KF_RESET;
    GLOGI("reset check: ns=%d [nc=%d cntc=%d %.3f flagc=%d] [np=%d cntp=%d %.3f flagp=%d] kfstatus=%d, vflag=%d, kfcnt=%d",
        ns,nc,cntc,rmsc,flagc,np,cntp,rmsp,flagp,rtk->kfstatus,vflag,rtk->kfcnt);
}
/* check float status -------------------------------------------------------------
*
* args   : none
* return : none
* author : 
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_fixstatus_check(rtk_t *rtk, int stat)
{
    if (stat == SOLQ_FIX)
    {
        rtk->shift.nofixc = 0;
        rtk->shift.nofixcnt = 0;
        rtk->shift.fixlist |= 1;
        rtk->shift.flag &= ~SHIFT_FLAG_FLOAT2FIX;
    }
    else
    {
        rtk->shift.nofixc++;
        if (rtk->shift.nofixcnt < 100)
        {
            rtk->shift.nofixcnt++;
        }
        rtk->nfix = 0;
    }

    if (stat != SOLQ_FLOAT)
    {
        return;
    }
    if (rtk->shift.nofixc>10 && (rtk->qcpasscnt & 0xFFF) == 0xFFF && rtk->nollicnt>8 && (rtk->dd_res[DDRES_UPDATE].flag & 0x3) == 0x3)
    {
        GLOGI("fix check: nofixc=%d fixlist=0x%x nollic=%u", rtk->shift.nofixc, rtk->shift.fixlist, rtk->nollicnt);
        rtk->shift.flag |= SHIFT_FLAG_FLOAT2FIX;
    }
}
/*return TRUE wl fix sol is good, FALSE wl fix sol is bad*/
extern int gnss_rtk_wl_confirm(rtk_t *rtk)
{
    int result = TRUE;
    wl_t* wl = &rtk->wl;

    if (wl->adop >= 10.0)
    {
        result = FALSE;
    }
    else
    {
        if (wl->nb <= 3)
        {
            if (wl->ratio <= 10.0)
            {
                result = FALSE;
            }
        }
        else if (wl->nb <= 8)
        {
            if (wl->ratio <= 3.0)
            {
                result = FALSE;
            }
            if ((wl->ratio / wl->adop) < 1.5)
            {
                result = FALSE;
            }
        }
        else
        {
            if ((wl->ratio / wl->adop) < 1.5)
            {
                result = FALSE;
            }
        }
    }

    return result;
}

/* ag_rtk_wl_constraint -------------------------------------------------------------
* get wl or and position constraint equation
* note   : if use this condition ,the R and H matrix should large enough when deleloped
* args   : none
* return : 1 yes, 0 no
* author : none
*-----------------------------------------------------------------------------*/
extern int ag_rtk_wl_constraint(rtk_t* rtk, double* H, double *R, const int* sat, double* v, int* _nv, int f1, int f2, nav_t* nav)
{
#if 0
    int i = 0, j = 0, t = 0, u = 0, z = 0, nwl = 0, nv = *_nv, oldnv = *_nv, nx = rtk->nx, column = 0;
    int *nrov = NULL, *isat = NULL;
    double *Hi = NULL, *D = NULL, *A = NULL, *Ai = NULL, *Di = NULL, *Dii = NULL, *AD = NULL, *temp = NULL;
    double *newR = NULL, *oldR = NULL, *floatWL = NULL, *fWL = NULL;
    double tf1 = 0.0, tf2 = 0.0;
    wl_t *wl = &rtk->wl;
    prcopt_t* opt = &rtk->opt;

    if (!(wl->flag & 1))
    {
        GLOGI("WL this epoch no wl constraint for wl no fixed sol");
        return 0;
    }
    nwl = wl->nb;
    column = nv + nwl + 3;
    if (column >= CMN_OBS_NUM)
    {
        GLOGI("WL this epoch no wl constraint for too much eq.");
        return 0;
    }

    nrov = wl->nwlsys;
    isat = wl->isat;
    D = zeros(2 * nwl, nx);
    A = zeros(nwl, 2 * nwl);
    fWL = zeros(nwl, 1);
    if (NULL == D || NULL == A || NULL == fWL)
    {
        if (D)   Sys_Free(D);
        if (A)   Sys_Free(A);
        if (fWL) Sys_Free(fWL);
        return 0;
    }

    t = 0;
    z = 0;
    u = 0;
    for (i = 0; i < NSYSDD; i++)
    {
        for (j = 0; j < nrov[i]; j++)
        {
            Di = D + (z + j) * nx;
            tf1 = nav->lam[sat[isat[t]] - 1][f1];
            tf2 = nav->lam[sat[isat[t]] - 1][f2];
            fWL[u++] = 1.0 / (1.0 / tf1 - 1.0 / tf2);
            Di[rtk->ix[IB(sat[isat[t]], f1, opt)]] = -1;// tf1;
            Di[rtk->ix[IB(sat[isat[t + j + 1]], f1, opt)]] = 1;// tf1;

            Dii = D + (nwl + z + j) * nx;
            Dii[rtk->ix[IB(sat[isat[t]], f2, opt)]] = -1;// tf2;
            Dii[rtk->ix[IB(sat[isat[t + j + 1]], f2, opt)]] = 1;// tf2;
        }
        t += nrov[i];
        z += nrov[i];
        if (nrov[i] > 0) t++;
    }

    for (i = 0; i < nwl; i++)
    {
        Ai = A + i * 2 * nwl;
        Ai[i] = 1;
        Ai[i + nwl] = -1;
    }

#if 0
    gnss_util_trace(3, "wl A=\n");
    gnss_util_tracemat(3, A, 2 * nwl, nwl, 8, 3);
#endif

    AD = zeros(nwl, nx);
    if (NULL == AD)
    {
        Sys_Free(A); Sys_Free(D); Sys_Free(fWL);
        return 0;
    }

    matmul("TT", nwl, nx, 2 * nwl, 1.0, A, D, 0.0, AD);
    Sys_Free(A); Sys_Free(D);
#if 0
    temp = mat(nwl, 1);
    matmul("NN", nwl, 1, nx, 1.0, AD, rtk->x, 0.0, temp);
    gnss_util_trace(3, "wl temp=\n");
    gnss_util_tracemat(3, temp, nwl, 1, 8, 3);
    gnss_util_trace(3, "wl bias=\n");
    gnss_util_tracemat(3, rtk->wl.bias, nwl, 1, 8, 3);
    Sys_Free(temp);
#endif

    floatWL = mat(nwl, 1);

    if (NULL == floatWL)
    {
        Sys_Free(AD); Sys_Free(fWL);
        return 0;
    }

    matmul("NN", nwl, 1, nx, 1.0, AD, rtk->x, 0.0, floatWL);
    for (i = 0; i < nwl; i++)
    {
        Hi = H + nv * nx;
        for (j = 0; j < nx; j++)
        {
            Hi[j] = AD[j + i * nx];
        }

        v[nv] = wl->bias[i] - floatWL[i];
        GLOGI("WL v=%.3f", v[nv]);
        nv++;
    }
    Sys_Free(AD);
    Sys_Free(floatWL);
    Sys_Free(fWL);

    /*add position constraint*/
    for (i = 0; i < 3; i++)
    {
        Hi = H + nv * nx;
        for (j = 0; j < nx; j++) Hi[j] = 0.0;
        Hi[i] = 1.0;
        v[nv] = wl->sol[i] - rtk->x[i];
        nv++;
    }
    *_nv = nv;

    /*next to design R matrix include wl and position*/
    newR = mat(nwl, nwl);
    if (NULL == newR) return 0;
    for (i = 0; i < nwl; i++)
    {
        for (j = 0; j < nwl; j++)
        {
            newR[i + j * nwl] = i == j ? 0.0003 : 0.0;
        }
    }

#if 0
    gnss_util_trace(3, "WL newR =\n");
    gnss_util_tracemat(3, newR, nwl, nwl, 8, 6);
#endif

    temp = mat(oldnv, oldnv);
    if (NULL == temp)
    {
        Sys_Free(newR);
        return 0;
    }
    for (i = 0; i < oldnv; i++)
    {
        for (j = 0; j < oldnv; j++)
        {
            temp[i + j * oldnv] = R[i + j * oldnv];
        }
    }

    memset(R, 0, CMN_OBS_NUM*CMN_OBS_NUM * sizeof(double));
    for (i = 0; i < oldnv; i++)
    {
        for (j = 0; j < oldnv; j++)
        {
            R[i + j * column] = temp[i + j * oldnv];
        }
    }
    Sys_Free(temp);
    for (i = 0; i < nwl; i++)
    {
        for (j = 0; j < nwl; j++)
        {
            R[i + oldnv + (j + oldnv) * column] = newR[i + j * nwl];
        }
    }
    Sys_Free(newR);

    for (i = 0; i < 3; i++)
    {
        R[i + oldnv + nwl + (i + oldnv + nwl)*column] = 0.05;
    }

#if 0
    gnss_util_trace(3, "WL R =\n");
    gnss_util_tracemat(3, R, column, column, 8, 6);
#endif

#endif
    return 1;
}

/* gnss_rtk_wl_sol -------------------------------------------------------------
* get wl solution by lsq
* args   : none
* return : 1 yes, 0 no
* author : none
*-----------------------------------------------------------------------------*/
static int gnss_rtk_wl_sol(rtk_t* rtk, const double *b, const double *y, const double* e, const nav_t* nav,
    const int* iu, const int *ir, const int *sat, int f1, int f2)
{
    int nv = 0, i = 0, j = 0, t = 0, k = 0, nwl = 0, nf = NF(&rtk->opt);
    int *isat = NULL, *nrov = NULL;
    double *H = NULL, *v = NULL, Q[9] = { 0.0 }, x[3] = { 0.0 };
    double wlf = 0.0;
    wl_t *wl = &rtk->wl;

    nwl = wl->nb;
    if (nwl < 3) return 0;
    H = mat(nwl, 3);
    v = mat(nwl, 1);
    if (NULL == H || NULL == v)
    {
        if (H) Sys_Free(H);
        if (v) Sys_Free(v);
        return 0;
    }

    nrov = wl->nwlsys;
    isat = wl->isat;

    t = 0;
    nv = 0;
    for (i = 0; i < NSYSDD; i++)
    {
        for (j = 0; j < nrov[i]; j++)
        {
            if (nav->lam[sat[isat[t]] - 1][f1] <= 0.0 || nav->lam[sat[isat[t]] - 1][f2] <= 0.0) continue;
            wlf = 1 / (1 / nav->lam[sat[isat[t]] - 1][f1] - 1 / nav->lam[sat[isat[t]] - 1][f2]);
            for (k = 0; k < 3; k++) {
                H[nv * 3 + k] = wlf * ((-e[k + iu[isat[t]] * 3] + e[k + iu[isat[t + j + 1]] * 3]) / nav->lam[sat[isat[t]] - 1][f1] -
                    (-e[k + iu[isat[t]] * 3] + e[k + iu[isat[t + j + 1]] * 3]) / nav->lam[sat[isat[t]] - 1][f2]);
            }
            v[nv] = -((y[f1 + iu[isat[t]] * nf * 2] - y[f1 + ir[isat[t]] * nf * 2]) -
                (y[f1 + iu[isat[t + j + 1]] * nf * 2] - y[f1 + ir[isat[t + j + 1]] * nf * 2])) / nav->lam[sat[isat[t]] - 1][f1] +
                ((y[f2 + iu[isat[t]] * nf * 2] - y[f2 + ir[isat[t]] * nf * 2]) -
                (y[f2 + iu[isat[t + j + 1]] * nf * 2] - y[f2 + ir[isat[t + j + 1]] * nf * 2])) / nav->lam[sat[isat[t]] - 1][f2];

            v[nv] = v[nv] * wlf;
            v[nv] -= b[nv] * wlf;
            nv++;
        }
        t += j;
        if (j > 0) t++;
    }

#if 0
    gnss_util_trace(3, "wl H=\n"); gnss_util_tracemat(3, H, nwl, 3, 8, 3);
#endif
    if (nv > 3)
    {
        i = lsq(H, v, 3, nv, x, Q);
        if (0 != i)
        {
            GLOGI("WL lsq get wl fix sol error");
        }
        else
        {
            for (i = 0; i < 3; i++)
            {
                wl->sol[i] = rtk->x[i] - x[i];
            }
            wl->pdop = gnss_rtk_getpdop_prefix(H, 3, nv);
#ifndef ASM_MATMUL
            matmul("TN", nv, 1, 3, 1.0, H, x, -1.0, v);
#else
            Fast_Rtk_matmul("TN", nv, 1, 3, 1.0, H, x, -1.0, v);
#endif
            wl->sigma = sqrt(dot(v, v, nv) / (nv - 3.0));
            if (wl->pdop < 8.0)
            {
                wl->flag |= 1;
            }
            GLOGI("WL lsq get wl fix sol pdop=%.3f sigma=%.3f delta_x=%.3f %.3f %.3f", wl->pdop, wl->sigma, x[0], x[1], x[2]);
        }
    }

    Sys_Free(H);
    Sys_Free(v);

    return 0;
}


/* ag_rtk_resamb_WL -------------------------------------------------------------
* get wl amb by lambda and get wl solution by lsq
* args   : none
* return : 1 yes, 0 no
* author : none
*-----------------------------------------------------------------------------*/
extern int ag_rtk_resamb_WL(rtk_t* rtk, int f1, int f2, const int* sat, const int* iu, const int* ir, int ns, const nav_t* nav, const double *y, const double *e)
{
    int i = 0, j = 0, m = 0, z = 0, nwl = 0, sysj = 0, t = 0, nx = rtk->nx;
    int *nrov = NULL, *isat = NULL;
    double *x = NULL, *sx = NULL, *D = NULL, *A = NULL, *b = NULL;
    double *P = NULL, *PP = NULL, *AD = NULL;
    double *Di = NULL, *Dii = NULL, *Ai = NULL, *Aii = NULL;
    double refwl = 0.0, rovwl = 0.0, s[2] = { 0.0 };
    const prcopt_t* opt = &rtk->opt;
    wl_t *wl = &rtk->wl;

    if (f1 < 0 || f2 < 0) return 0;

    wl->ratio = 0.0;
    wl->nb = 0;
    wl->adop = 1000.0;
    wl->sigma = 1000.0;
    wl->pdop = 1000.0;

    isat = wl->isat;
    nrov = wl->nwlsys;
    x = rtk->x;
    memset(isat, 0, MAX_N_WL * sizeof(int));
    memset(nrov, 0, NSYSDD * sizeof(int));

    /*select sat to form wl equation*/
    nwl = 0;
    for (m = 0; m < NSYSDD; m++)
    {
        if (m == 1) continue;
        /*select reference satellite*/
        for (i = -1, j = 0; j < ns; j++)
        {
            if (NULL == rtk->ssat[sat[j] - 1])
            {
                continue;
            }
            sysj = rtk->ssat[sat[j] - 1]->sys;
            if (!gnss_rtk_test_sys(sysj, m)) continue;
            if (!rtk->ssat[sat[j] - 1]->vsat[f1] || !rtk->ssat[sat[j] - 1]->vsat[f2]) continue;
            if ((rtk->ssat[sat[j] - 1]->snr[f1] / 4) < 30 && (rtk->ssat[sat[j] - 1]->snr[f1] / 4) < (unsigned char)gpz_kf_data->meas_blk->avgCno) continue;
            if ((rtk->ssat[sat[j] - 1]->snr[f2] / 4) < 30 && (rtk->ssat[sat[j] - 1]->snr[f2] / 4) < (unsigned char)gpz_kf_data->meas_blk->avgCno) continue;
            if ((rtk->ssat[sat[j] - 1]->slip[f1] & 2) || (rtk->ssat[sat[j] - 1]->slip[f2] & 2)) continue;
            if (rtk->ix[IB(sat[j], f1, opt)] >= 0 && rtk->ix[IB(sat[j], f2, opt)] >= 0)
            {
                i = j;
                break;
            }
        }
        if (i < 0) {
            continue;
        }

        if (nwl >= MAX_N_WL) break;
        isat[nwl] = i;
        nwl++;
        if (nwl >= MAX_N_WL) break;
        /*make dobule diference WL*/
        for (t = 0, j = 0; j < ns; j++)
        {
            if (i == j) continue;
            if (NULL == rtk->ssat[sat[j] - 1])
            {
                continue;
            }
            sysj = rtk->ssat[sat[j] - 1]->sys;
            if (!gnss_rtk_test_sys(sysj, m)) continue;
            if (!rtk->ssat[sat[j] - 1]->vsat[f1] || !rtk->ssat[sat[j] - 1]->vsat[f2]) continue;
            if ((rtk->ssat[sat[j] - 1]->snr[f1] / 4) < 30 && (rtk->ssat[sat[j] - 1]->snr[f1] / 4) < (unsigned char)gpz_kf_data->meas_blk->avgCno) continue;
            if ((rtk->ssat[sat[j] - 1]->snr[f2] / 4) < 30 && (rtk->ssat[sat[j] - 1]->snr[f2] / 4) < (unsigned char)gpz_kf_data->meas_blk->avgCno) continue;
            if (rtk->ix[IB(sat[j], f1, opt)] < 0 || rtk->ix[IB(sat[j], f2, opt)] < 0) continue;
            if ((rtk->ssat[sat[j] - 1]->slip[f1] & 2) || (rtk->ssat[sat[j] - 1]->slip[f2] & 2)) continue;

            isat[nwl] = j;
            nwl++;
            t++;
            if (nwl >= MAX_N_WL) break;
#if 0
            refwl = (f1 >= 0 ? x[rtk->ix[IB(sat[i], f1, opt)]] : 0.0) - (f2 >= 0 ? x[rtk->ix[IB(sat[i], f2, opt)]] : 0.0);
            rovwl = (f1 >= 0 ? x[rtk->ix[IB(sat[j], f1, opt)]] : 0.0) - (f2 >= 0 ? x[rtk->ix[IB(sat[j], f2, opt)]] : 0.0);
            gnss_util_trace(3, "wl sat=%d-%d wl=%.3f\n", sat[i], sat[j], rovwl - refwl);
#endif
        }
        nrov[m] = t;
        GLOGI("WL m=%d %d wl obs", m, t);
        if (0 == t) nwl--;
    }

    for (i = 0; i < nwl; i++)
    {
        GLOGI("WL fixed sat %d", sat[isat[i]]);
    }

    nwl = 0;
    for (i = 0; i < NSYSDD; i++) nwl += nrov[i];
    if (nwl <= 0)
    {
        GLOGI("WL failed,stop wl, nwl=%d", nwl);
        return 0;
    }

    /*1. to design D and A matrix*/
    D = zeros(2 * nwl, nx);
    A = zeros(nwl, 2 * nwl);
    if (NULL == D || NULL == A)
    {
        if (D)   Sys_Free(D);
        if (A)   Sys_Free(A);
        return 0;
    }

    t = 0;
    z = 0;
    for (i = 0; i < NSYSDD; i++)
    {
        for (j = 0; j < nrov[i]; j++)
        {
            Di = D + (z + j) * nx;
            Di[rtk->ix[IB(sat[isat[t]], f1, opt)]] = -1;
            Di[rtk->ix[IB(sat[isat[t + j + 1]], f1, opt)]] = 1;

            Dii = D + (nwl + z + j) * nx;
            Dii[rtk->ix[IB(sat[isat[t]], f2, opt)]] = -1;
            Dii[rtk->ix[IB(sat[isat[t + j + 1]], f2, opt)]] = 1;
        }
        t += nrov[i];
        z += nrov[i];
        if (nrov[i] > 0) t++;
    }

    for (i = 0; i < nwl; i++)
    {
        Ai = A + i * 2 * nwl;
        Ai[i] = 1;
        Ai[i + nwl] = -1;
    }

#if 0
    gnss_util_trace(3, "wl D=\n"); gnss_util_tracemat(3, D, 2 * nwl, nx, 8, 3);
    gnss_util_trace(3, "wl A=\n"); gnss_util_tracemat(3, A, nwl, 2 * nwl, 8, 3);
#endif

    /*2.next to design float wl sx and its P matrix*/
    AD = zeros(nwl, nx);
    if (NULL == AD)
    {
        Sys_Free(A); Sys_Free(D);
        return 0;
    }
#ifndef ASM_MATMUL
    matmul("TT", nwl, nx, 2 * nwl, 1.0, A, D, 0.0, AD);
#else
    Fast_Rtk_matmul("TT", nwl, nx, 2 * nwl, 1.0, A, D, 0.0, AD);
#endif
    Sys_Free(A); Sys_Free(D);

    sx = mat(nwl, 1);
    PP = mat(nwl, nx);
    P = mat(nwl, nwl);
    if (NULL == sx || NULL == PP || NULL == P)
    {
        Sys_Free(AD);
        if (sx) Sys_Free(sx);
        if (PP) Sys_Free(PP);
        if (P)  Sys_Free(P);
        return 0;
    }
#ifndef ASM_MATMUL
    matmul("NN", nwl, 1, nx, 1.0, AD, x, 0.0, sx);
#else
    Fast_Rtk_matmul("NN", nwl, 1, nx, 1.0, AD, x, 0.0, sx);
#endif
    matmulsm("NN010", nwl, nx, nx, 1.0, AD, rtk->P, 0.0, PP);
#ifndef ASM_MATMUL
    matmul("NT", nwl, nwl, nx, 1.0, PP, AD, 0.0, P);
#else
    Fast_Rtk_matmul("NT", nwl, nwl, nx, 1.0, PP, AD, 0.0, P);
#endif
    Sys_Free(AD); Sys_Free(PP);
#if 0
    gnss_util_trace(3, "wl P=\n"); gnss_util_tracemat(3, P, nwl, nwl, 8, 3);
#endif

    /*3. last get wl fixed amb b by lambda and get fixed solution if confirmed*/
    b = mat(nwl, 2);
    if (NULL == b)
    {
        Sys_Free(P); Sys_Free(sx);
        return 0;
    }
    if (!(t = lambda(nwl, 2, sx, P, b, s)))
    {
        wl->ratio = s[0] > 0.0 ? (s[1] / s[0]) : 0.0;
        wl->nb = nwl;
        wl->adop = gnss_rtk_adop_get();
        for (i = 0; i < nwl; i++) wl->bias[i] = b[i];

#if 0
        gnss_util_trace(3, "wl ratio=%.3f\n", wl->ratio);
        gnss_util_trace(3, "wl(1)=\n"); gnss_util_tracemat(3, b, 1, nwl, 5, 3);
        gnss_util_trace(3, "wl(2)=\n"); gnss_util_tracemat(3, b + nwl, 1, nwl, 5, 3);
#endif

        /*WL fixed sol confirm*/
        if (TRUE == gnss_rtk_wl_confirm(rtk))
        {
            /*to get WL solution*/
            gnss_rtk_wl_sol(rtk, b, y, e, nav, iu, ir, sat, f1, f2);
        }
        Sys_Free(b);
    }
    else
    {
        GLOGI("THIS EPOCH WL FIX FAILED");
    }

    GLOGI("WL flag=%d nb=%d ratio=%.1f adop=%.3f sigma=%.3f pdop=%.3f", wl->flag & 0x1, wl->nb, wl->ratio, wl->adop,
        wl->sigma, wl->pdop);


    if (D)   Sys_Free(D);
    if (A)   Sys_Free(A);
    if (AD)  Sys_Free(AD);
    if (b)   Sys_Free(b);
    if (P)   Sys_Free(P);
    if (PP)  Sys_Free(PP);
    if (sx)  Sys_Free(sx);

    return 1;
}
/* static detect and process -------------------------------------------------------------
* TODO: need refine more
* args   : none
* return : none
* author : 
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_kf_static_check(rtk_t *rtk)
{
    int i;
    //need more strict condition for static constrain, better not use in open sky scenario
    if (!(peMode.staticData.staticFlag || (peMode.staticData.speedStaticCnt>0&&(rtk->qcpasscnt&1))) ||
        ((rtk->qcpasscnt&1)&&(peMode.staticData.lsVel>0.5||peMode.staticData.deltaDoplVar>0.3)))
    {
        if (rtk->staticsol.first==1) 
        {
            //what to do when finally get out from static mode
            if ((rtk->qcpasscnt&1)&&rtk->dd_res[DDRES_UPDATE].rmsp>2.0)
            {
                rtk->kfstatus=RTK_KF_RESET;
            }
        }
        rtk->staticsol.first=0;
        return;
    }
    if (rtk->staticsol.first==0)
    {
        rtk->staticsol.first=1;
        //what to do when first get into static mode
        if ((rtk->qcpasscnt&1)&&rtk->dd_res[DDRES_UPDATE].rmsp>3.0)
        {
            rtk->kfstatus|=RTK_KF_RESET_AMB;
        }
        for (i=0;i<3;i++)
        {
            rtk->staticsol.r[i]=rtk->sol.rr[i];
        }
        rtk->staticsol.stat=rtk->sol.stat;
    }
    else
    {
        for (i=0;i<3;i++)
        {
            rtk->sol.rr[i]=rtk->staticsol.r[i];
            rtk->x[i]=rtk->staticsol.r[i];
        }
        rtk->sol.stat=rtk->staticsol.stat;

        //TODO: compare the fix one with updated one, use updated one as new fix if it wins
    }
}
/* cycle slip determine -------------------------------------------------------------
* cycle slip determine
* args   : none
* return : none
* author : 
*-----------------------------------------------------------------------------*/
extern int gnss_rtk_slip_determine(int f,ssat_t *psat,int rcv)
{
    int16_t cd=CYCSLIPDET_CN0|CYCSLIPDET_DOPPLER;
    if (psat->slipflag[rcv - 1][f] & CYCSLIPDET_CPDR)
    {
        return 1;
    }
    if (psat->slipflag[rcv - 1][f] & CYCSLIPDET_LOCKSAT)
    {
        return 1;
    }
    if (psat->slipflag[rcv-1][f]&(CYCSLIPDET_TDRES|CYCSLIPDET_TDEST))
    {
        return 1;
    }
    if (g_pe_cfg.chipType==MTK && (psat->slipflag[rcv-1][f]&cd)==cd && psat->azel[1]*R2D<30.0)
    {
        return 1;
    }
    if (psat->slipflag[rcv-1][f]&CYCSLIPDET_LLI)
    {
        if (psat->slipflag[rcv - 1][f] & CYCSLIPDET_LLI_HC)
        {
            return 2;
        }
        return 1;
    }

    return 0;
}
/* multi-freq LC detect results fusion -------------------------------------------------------------
* lc results fusion
* args   : none
* return : none
* author : 
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_slip_lc_fusion(int sat,ssat_t *psat,int rcv)
{
    uint8_t slip1=psat->slip[0]&0x3, slip2=psat->slip[1]&0x3;
    int32_t lc_detect = (psat->slipflag[rcv-1][0] & (CYCSLIPDET_ZD_MW | CYCSLIPDET_ZD_GF)), f_slip=0;
    
    if (rtkctrl.opt.nf <= 1) return;
    if (slip1!=0 && slip2!=0) //already mark slip flag both
    {
        return;
    }

    if ((lc_detect & CYCSLIPDET_ZD_GF) == 0) //ZD GF detect normal
    {
        return;
    }

    if (lc_detect & CYCSLIPDET_ZD_MW) //both BAD
    {
        f_slip = 3;
        psat->slip[0] |= 1;
        psat->slip[1] |= 1;

        //TODO:add dt for MW valid reference?
    }
    else if(rcv==1)//only ZD GF abnormal
    {
        if ((psat->slip[1]&3)!=0 && (psat->slip[0]&3)==0 &&
            (psat->tdcheck[0]&TD_CHECK_RES) && fabs(psat->tdy[0])<1.0&&
            (psat->tdcheck[0] & TD_CHECK_EST) && fabs(psat->tdres[0]) < 0.1
            && (psat->snr[0]/4) >= 30
            && (psat->usectrl.rqflag[0] & PE_MEAS_VALID_DR))//f1 OK
        {
            f_slip = 2;
            psat->slip[1] |= 1; //f3 BAD
        }
        else if ((psat->slip[0]&3)!=0 && (psat->slip[1]&3)==0 &&
            (psat->tdcheck[1]&TD_CHECK_RES) && fabs(psat->tdy[1])<1.0&&
            (psat->tdcheck[1] & TD_CHECK_EST) && fabs(psat->tdres[1]) < 0.1
            && (psat->snr[1]/4) >= 30
            && (psat->usectrl.rqflag[1] & PE_MEAS_VALID_DR))//f3 OK
        {
            f_slip = 1;
            psat->slip[0] |= 1; //f1 BAD
        }
        else //f1 f2 BAD
        {
            f_slip = 3;
            psat->slip[0] |= 1; //f1 BAD
            psat->slip[1] |= 1; //f3 BAD
        }
    }
    else if (rcv == 2)
    {
        f_slip = 3;
        psat->slip[0] |= 1; //f1 BAD
        psat->slip[1] |= 1; //f2/f3 BAD
    }


    GLOGI("slp-fusion:%2d rcv=%d f_slip=%1d  slip[%u %u] mw=%d tdcheck[%u %u] tdy1=%8.3f tdres1=%6.3f tdy2=%8.3f tdres2=%6.3f",
        sat,rcv,f_slip,slip1,slip2,(lc_detect & CYCSLIPDET_ZD_MW)?1:0,psat->tdcheck[0],psat->tdcheck[1],
        psat->tdy[0],psat->tdres[0],psat->tdy[1],psat->tdres[1]);
}
/* check time update -------------------------------------------------------------
* 
* args   : none
* return : none
* author : 
*-----------------------------------------------------------------------------*/
extern int gnss_rtk_extpol_check(rtk_t *rtk,sol_t *solp)
{
    //int i;
    if (rtk->kfextpol)
    {
        if (rtk->rtkevent!=0&&rtk->rtkevent!=RTK_NOROVRSPP)
        {
            if (rtk->rtkevent&RTK_NOROVRSPP) //no pvt results
            {
                rtk->sol=*solp;
                rtk->sol.stat=SOLQ_DR; //use DR as extrapolation flag
                rtk->nfix=0;
            }
            return 1;
        }
    }
    return 0;
}

/* gnss_rtk_fill_measUseInfo  -------------------------------------------------------------
*
* args   : none
* return : none
* author :
*-----------------------------------------------------------------------------*/
void gnss_rtk_fill_measUseInfo(rtk_t *rtk, USER_PVT* pvt)
{
    MEAS_USE_INFO*       pMeasUseInfo = NULL;
    ddres_t*             pddres = NULL;

    if (rtk == NULL || pvt == NULL) return;

    pMeasUseInfo = &(pvt->meas_use_info);
    pddres = &(rtk->dd_res[DDRES_UPDATE]);

    pMeasUseInfo->cr_res_num = pddres->nc;
    pMeasUseInfo->prNum_rtk = pddres->np;

    pMeasUseInfo->rtdCrDewNum = rtk->dewNum_cr;
    pMeasUseInfo->rtdPrDewNum = rtk->dewNum_pr;

    pMeasUseInfo->pre_cr_res_std = rtk->dd_res[DDRES_PREDICT].stdc;
    
    /* calculare gnss codephase residual rms*/
    if (pMeasUseInfo->cr_res_num == 0)
    {
        pMeasUseInfo->usedCrResStd = -1.0;
    }
    else
    {
        pMeasUseInfo->usedCrResStd = pddres->rmsc;
    }

    if (rtk->sol.stat == SOLQ_FIX)
    {   

        if ((rtk->shift.fixlist & 0x1) == 0x1)
        {
            pMeasUseInfo->svNum_for_fix = rtk->nbias;
            pMeasUseInfo->fixed_ratio = rtk->sol.ratio;
        }
        else
        {
            pMeasUseInfo->svNum_for_fix = rtk->prefixn;
            pMeasUseInfo->fixed_ratio = (float)rtk->prefixr;
        }
    }
    else
    {
        pMeasUseInfo->svNum_for_fix = 0;
        pMeasUseInfo->fixed_ratio = 0.0;
    }
}

static void gnss_rtk_err_change(const rtk_t *rtk, USER_PVT *pvt, const float inPosVarxy, float *outPosVarXY)
{
    float posVarX, posVarY, templ, errENU[3];
    float horz_acc, up_acc;

    posVarX = pvt->posErr.lon_err;
    posVarY = pvt->posErr.lat_err;
    templ = (float)sqrtf(posVarX * posVarX + posVarY * posVarY);
    if (templ < 1e-10 || rtk->cfd.cfdflag != 1)
    {
        *outPosVarXY = 999.9f;
        pvt->accuracy = 999.9f;
        pvt->posErr.lat_err = 999.9f;
        pvt->posErr.lon_err = 999.9f;
        pvt->posErr.alt_err = 999.9f;
        pvt->posErr.h = 999.9f;
        pvt->posErr.v = 999.9f;
        pvt->posErr.p = 999.9f;
        return;
    }

    horz_acc = (float)rtk->cfd.horicfd[0];
    up_acc = (float)rtk->cfd.upcfd[0];

    errENU[0] = (float)(posVarY * horz_acc / templ);
    errENU[1] = (float)(posVarX * horz_acc / templ);
    errENU[2] = (float)up_acc;

    *outPosVarXY = (float)(inPosVarxy * (horz_acc / templ) * (horz_acc / templ));
    pvt->accuracy = (float)horz_acc;
    pvt->posErr.lat_err = errENU[0];
    pvt->posErr.lon_err = errENU[1];
    pvt->posErr.alt_err = errENU[2];
    pvt->posErr.h = (float)horz_acc;
    pvt->posErr.v = (float)up_acc;
    pvt->posErr.p = (float)sqrtf(horz_acc * horz_acc + up_acc * up_acc);

}

/* fill rtk pos fix -------------------------------------------------------------
* 
* args   : none
* return : none
* author : 
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_fill_pvt(rtk_t *rtk,USER_PVT* pvt)
{
    uint8_t              k, m;
    float             ecefPosVar[3][3],enuPosVar[3][3], ceg[3][3], cge[3][3], matTmp[3][3];
    float             posVarX, posVarY, posVarXY;
    double             dt = 0.0,dt1 = 0.0,Nh = 0.0;
    GNSS_TIME*      pTime;
    gtime_t         time;
    int             week=0;
    if (rtk->sol.stat != SOLQ_NONE)
    {
        //gnss_sd_set_diffage(rtk->sol.age);
        pvt->diff_age = rtk->sol.age;
    }

    /*$GPGGA diff_age output add protection,when ref tor advanced*/
    if (pvt->diff_age <= 0.0)
    {
        pvt->diff_age = 0.01;
    }

    if (rtk->sol.stat==SOLQ_NONE||rtk->sol.stat==SOLQ_SINGLE)
    {
        return;
    }

    pTime = gnss_tm_get_time();
    if (!pTime) return;
    
    if (rtk->sol.stat==SOLQ_FLOAT)
    {
        pvt->diff_status = DIFF_SOL_STATUS_RTK_FLOAT;  //float results
    }
    else if (rtk->sol.stat==SOLQ_FIX)
    {
        pvt->diff_status = DIFF_SOL_STATUS_RTK_FIX;  //fix results
    }
    else if (rtk->sol.stat==SOLQ_DR)
    {
        pvt->diff_status = DIFF_SOL_STATUS_RTK_PROPAGATE;  //propagated by rtk results
    }
    else if (rtk->sol.stat==SOLQ_DGPS)
    {
        pvt->diff_status = DIFF_SOL_STATUS_RTD;
        if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_BRCM_47755)
        {
            if (rtk->dd_res[DDRES_UPDATE].nc > 2)
            {
                pvt->diff_status = DIFF_SOL_STATUS_RTK_FLOAT;
                rtk->sol.stat = SOLQ_FLOAT;
            }
            else
            {
                pvt->diff_status = DIFF_SOL_STATUS_STANDALONE;
                /*no rtd for BRCM's large rtd solution */
                rtk->sol.stat = SOLQ_SINGLE;
                GLOGI("FILL PVT select single instead rtd");
                return;
            }
        }
    }
    else
    {
        return;
    }

    /* Decide fusion mode flag from diff-status above. */
    gnss_pe_fill_fusion_mode(pvt);

    /* set fixed sv number*/
    pvt->fixedSvNum = 0;
    if (rtk->sol.stat == SOLQ_FIX)
    {
        if (rtk->prefixf == 2)
        {
            pvt->fixedSvNum = rtk->prefixn;
        }
        else
        {
            pvt->fixedSvNum = rtk->nbias;
        }
    }
    
    dt1 = pTime->bias[GPS_MODE] / LIGHT_SEC;
    time=timeadd(rtk->sol.time,-dt1);
    dt = time2gpst(time,NULL);
    dt *= 10; //unit:100ms
    dt -= (int64_t)dt; //sub 100ms
    if (dt>=0.5)
    {
        dt=1.0-dt;
    }
    else
    {
        dt=-dt;
    }
    dt=dt*0.1;//s
    
    for (k = 0; k < 3; k++)
    {
        pvt->ecef.pos[k] = rtk->sol.rr[k] + rtk->sol.rr[k+3] * dt;
        pvt->ecef.vel[k] = (float)rtk->sol.rr[k+3];
    }
    
    gnssConvEcef2Lla(pvt->ecef.pos,pvt->lla.pos);
    //calculate altitude to Mean sea level
    Nh = gnss_sd_get_geoId();
    pvt->altitudeMsl = pvt->lla.pos[2] - Nh;
    
    /* get ENU velocity */
    gnssConvEcef2EnuVel(pvt->ecef.vel, pvt->lla.vel, pvt->lla.pos);

#if 1
    if (((g_pe_cfg.meas_type_mask&MEAS_TYPE_MASK_DR) == 0 && rtk->opt.dynamics) || g_pe_cfg.meas_rate > MEAS_RATE_1HZ)
    {
        /* heading velocity and direction */
        pvt->velocity = gnssClcSqrtSum_FLT(pvt->lla.vel, 2);
        if (pvt->velocity > 0.5)
        {
            pvt->heading = (float)atan2(pvt->lla.vel[0], pvt->lla.vel[1]);
            pvt->heading = (float)(pvt->heading * RAD2DEG);
            if (pvt->heading < 0.0)
            {
                pvt->heading += 360.0;
            }
        }
    }
#endif

    //set pos/vel std of rtk position fix
    gnssGetEcef2EnuMatrix(ceg, pvt->lla.pos);
    for (k = 0; k < 3; k++)
    {
        for (m = 0; m < 3; m++)
        {
            cge[k][m] = ceg[m][k];
        }
    }

    /* covert x,y,z variance to ENU */
    for (k = 0; k < 3; k++)
    {
        ecefPosVar[k][k] = rtk->sol.qr[k];
    }
    
    ecefPosVar[0][1] = ecefPosVar[1][0] = rtk->sol.qr[3];
    ecefPosVar[1][2] = ecefPosVar[2][1] = rtk->sol.qr[4];
    ecefPosVar[2][0] = ecefPosVar[0][2] =  rtk->sol.qr[5];

    gnss_Matrix_Mult(ceg, ecefPosVar, matTmp);
    gnss_Matrix_Mult(matTmp, cge, enuPosVar);

    if(enuPosVar[1][1]>0 && enuPosVar[0][0]>0 && enuPosVar[2][2]>0) 
    {
        pvt->posErr.lat_err = (float)sqrt(enuPosVar[1][1]);
        pvt->posErr.lon_err = (float)sqrt(enuPosVar[0][0]);
        pvt->posErr.alt_err = (float)sqrt(enuPosVar[2][2]);
    }
    else
    {
        pvt->posErr.lat_err = 100.0;
        pvt->posErr.lon_err = 100.0;
        pvt->posErr.alt_err = 100.0;
    }
    

    pvt->accuracy = (float)sqrtf(pvt->posErr.lat_err*pvt->posErr.lat_err + pvt->posErr.lon_err * pvt->posErr.lon_err);
    posVarXY = enuPosVar[0][1];
    if (rtk->sol.stat == SOLQ_FLOAT || rtk->sol.stat == SOLQ_FIX || rtk->sol.stat == SOLQ_DGPS)
    {
        gnss_rtk_err_change(rtk, pvt, enuPosVar[0][1], &posVarXY);
    }
    
    /*calculate horizontal elliptical uncertainty*/
    posVarX = pvt->posErr.lon_err * pvt->posErr.lon_err;
    posVarY = pvt->posErr.lat_err * pvt->posErr.lat_err;
    gnss_Pe_Cal_HorizEllipseUnc(posVarX, posVarY, posVarXY, pvt);

#if defined(PLAYBACK_MODE)
    GLOGI("stat=%d UncSemiMajor=%.6f Minor=%.6f Ori=%.6f E=%.6f N=%.6f U=%.6f", rtk->sol.stat, pvt->ellipseUncSemiMajor, pvt->ellipseUncSemiMinor,
        pvt->ellipseUncOrientation, pvt->posErr.lat_err, pvt->posErr.lon_err, pvt->posErr.alt_err);
#endif

	//set pos fix accuracy confidence
	//gnss_Pe_Pos_Confidence_Set(pvt);
	gnss_rtk_accu_upd(rtk,pvt);

    //fill rtk residual
    gnss_rtk_fill_measUseInfo(rtk, pvt);

    // fix time
    time=timeadd(rtk->sol.time,dt);
    pvt->posfix_t=time2gpst(time,&week);
    pvt->posfix_wn = (uint16_t)week;

    // Fill ECEF position
    pvt->ecef.have_position = POS_FRM_KF;
    pvt->lla.have_position = POS_FRM_KF;
    pvt->ecef.posfix_t = pvt->posfix_t;
    pvt->ecef.posfix_wn = pvt->posfix_wn;

    // Error fill
    //pvt->posErr = gpz_kf_pvt_data->kfErrEst;
    //pvt->accuracy=1.0; //fill rtk pos accuracy

    pvt->have_position = HAVE_POS_FIX3D; // HAVE_POS_APPX
    //SYS_LOGGING(OBJ_PE, LOG_INFO, "rtk_user_pvt: %14.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f%", pTime->tor, pvt->altitudeMsl, pvt->posErr.lon_err, pvt->posErr.lat_err, pvt->ellipseUncSemiMajor, pvt->ellipseUncSemiMinor, pvt->ellipseUncOrientation);
}

/* deal with rb change -------------------------------------------------------------
* 
* args   : none
* return : none
* author : 
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_rb_changed(rtk_t *rtk)
{
    //int i,j;
    if (rtk->kfstatus&RTK_KF_RUN)
    {
        rtk->kfstatus|=RTK_KF_RESET_AMB;
    }
    else
    {
        rtk->kfstatus=RTK_KF_RESET;
    }
#if 0
    //clean base obsl1
    for (i=1;i<=MAXSAT;i++)
    {
        for (j = 0; j < AGGNSS_MAX_FREQ_NUM && j < NFREQ; j++)
        {
            if (!rtk->ssat[i-1]->hod[j].obsn[1])
            {
                continue;
            }

            gnss_rtk_obsl1_clean(&rtk->ssat[i-1]->hod[j],2,1);
        }
    }
#endif
}
/* save float sol -------------------------------------------------------------
* 
* args   : none
* return : none
* author : 
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_save_floatsol(rtk_t *rtk,sol_t *sol)
{
    int i;

    for (i=0;i<3;i++) {
         sol->rr[i]=rtk->x[i];
        sol->qr[i] = (float)rtk->P[ISM(i, i)];
     }
    sol->qr[3] = (float)rtk->P[ISM(0, 1)];
    sol->qr[4] = (float)rtk->P[ISM(1, 2)];
    sol->qr[5] = (float)rtk->P[ISM(0, 2)];

    sol->stat = SOLQ_DR;
}
/* compare solutions -------------------------------------------------------------
* 
* args   : none
* return : 0:old one win, 1: new one win
* author : 
*-----------------------------------------------------------------------------*/
static int gnss_rtk_sol_compare(rtk_t *rtk,double *poso,int ddtypeo,double *posn,int ddtypen,int ns)
{
    ddres_t *preso=NULL,*presn=NULL;
    int i,ret=0;
    double delta[3]={0.0},lla[3]={0.0},enu[3]={0.0},dis=0.0,dis2D=0.0;
    
    preso=&rtk->dd_res[ddtypeo];
    presn=&rtk->dd_res[ddtypen];

    /* get distance */
    for (i=0;i<3;i++)
    {
        delta[i]=poso[i]-posn[i];
        dis+=delta[i]*delta[i];
    }
    dis=sqrt(dis);

    ecef2pos(poso,lla);
    ecef2enu(lla,delta,enu);
    
    dis2D=enu[0]*enu[0]+enu[1]*enu[1];
    dis2D=sqrt(dis2D);

    GLOGI("sol comp: ns=%d d3=%.1f d2=%.2f [ddo=%d f=0x%x rc=%.3f rp=%.3f] [ddn=%d f=0x%x rc=%.3f rp=%.3f]",
        ns,dis,dis2D,ddtypeo,preso->flag,preso->rmsc,preso->rmsp,ddtypen,presn->flag,presn->rmsc,presn->rmsp);

    if (ddtypeo==DDRES_SINGLE && (gpz_kf_data->kf_ctrl.prNum - gpz_kf_data->kf_ctrl.prRejNum) > 5 &&
        gpz_kf_data->posRes > 0.0 && gpz_kf_data->posRes < 15.0)
    {
        if (((dis2D>20.0||dis>30.0) &&presn->cntp<4 && gpz_kf_data->posRes < 10.0) || ((rtk->closkycnt & 0xf) == 0xf && presn->cntc < 3 && presn->cntp < 3 && dis2D > 1.5 && dis > 5.0))
        {
            if (ddtypen==DDRES_UPDATE)
            {
                rtk->kfstatus=RTK_KF_RESET;
                GLOGI("ddn=%d: use spp(meas=%u posres=%.3f), cntp=%u reset kf",ddtypen,gpz_kf_data->kf_ctrl.prNum-gpz_kf_data->kf_ctrl.prRejNum,gpz_kf_data->posRes,presn->cntp);
            }
            return 0;
        }
        else if ((dis2D>10.0||dis>20.0) &&presn->cntp<3 &&rtk->kfcnt<3 && gpz_kf_data->posRes < 5.0)
        {
            return 0;
        }
    }
    else if (peMode.staticData.staticFlag &&ddtypeo==DDRES_SINGLE &&
             (gpz_kf_data->kf_ctrl.prNum - gpz_kf_data->kf_ctrl.prRejNum) > 10 && gpz_kf_data->posRes > 0.0 && gpz_kf_data->posRes < 15.0)
    {
        if ((presn->cntp<3 &&rtk->kfcnt<3 &&dis2D>10.0 && gpz_kf_data->posRes < 5.0) ||
            ((rtk->qcpasscnt&0xF)==0xF && gpz_kf_data->posRes < 10.0 && dis2D > 20.0))
        {
            if (ddtypen==DDRES_UPDATE)
            {
                rtk->kfstatus=RTK_KF_RESET;
                GLOGI("ddn=%d: use spp(meas=%u posres=%.3f), cntp=%u reset kf",ddtypen,gpz_kf_data->kf_ctrl.prNum-gpz_kf_data->kf_ctrl.prRejNum,gpz_kf_data->posRes,presn->cntp);
            }
            return 0;
        }
        else if ((preso->flag&DDRESFLAG_REJTOOMUCH) &&presn->rmsp>10.0 &&rtk->kfcnt<3 &&dis2D>5.0 && gpz_kf_data->posRes < 5.0)
        {
            return 0;
        }
    }

    if (ddtypeo==DDRES_SINGLE&&ddtypen==DDRES_PREDICT)
    {
        ret=0;
        if ((preso->flag&0x3)!=0x3 && (presn->flag&0x3)==0x3 && presn->rmsp<3.0)  //DDRESFLAG_CP_GOOD|DDRESFLAG_PR_GOOD
        {
            ret=1;
        }
        else if (preso->nc<=presn->nc && preso->cntc<presn->cntc && preso->rmsc>presn->rmsc &&
            preso->np<=presn->np && preso->cntp<presn->cntp && preso->rmsp>presn->rmsp)
        {
            ret=1;
        }
    }
    else if (ddtypen==DDRES_UPDATE)
    {
        if (ddtypeo==DDRES_SINGLE &&(rtk->closkycnt&0xF)==0xF &&presn->np<5 &&dis2D>15.0 && gpz_kf_data->posRes < 15.0)
        {
            rtk->kfstatus=RTK_KF_RESET;
            GLOGI("large dis detected in closky, reset kf");
            return 0;
        }
        ret=1;
        if ((presn->flag&0x3)!=0x3 && (preso->flag&0x3)==0x3 && preso->rmsp<3.0)
        {
            ret=0;
        }
        else if ((presn->rmsp-preso->rmsp)>0.5f && presn->np<=preso->np && presn->cntp<=preso->cntp &&
            presn->rmsp>2.0 && preso->rmsp<3.0)
        {
            if (preso->rmsc<0.1 || (presn->rmsp>5.0 && (preso->flag&DDRESFLAG_PR_GOOD)))
            {
                ret=0;
            }
        }

        if (ret==0 && (presn->flag&0x3)==0x3 && (preso->flag&0x3)!=0x3)
        {
            ret=1;
        }
    }
    else if (ddtypen==DDRES_FIXED)
    {
        ret=1;  //TODO: refine as ratio, nsat, dis2D, rmsc and rmsp
        /*if (ddtypeo==DDRES_UPDATE&&presn->nc<6&&rtk->nbias<4&&dis2D>5.0&&(rtk->closkycnt&0xF)==0xF)
        {
            ret=0;
        }*/
        /*if (ddtypeo==DDRES_UPDATE && presn->cntc<=preso->cntc && presn->rmsc>preso->rmsc &&
            presn->cntp<=preso->cntp && presn->rmsp>preso->rmsp)
        {
            ret=0;
        }*/
    }

    return ret;
}
/* determine solution type -------------------------------------------------------------
* 
* args   : none
* return : solution status
* author : 
*-----------------------------------------------------------------------------*/
static const char* gnss_rtk_get_sol_name(int solstat)
{
    const char* name = NULL;
    switch (solstat)
    {
    case SOLQ_NONE:   name = SOLQ_NAME_NONE; break;
    case SOLQ_FIX:    name = SOLQ_NAME_FIX; break;
    case SOLQ_FLOAT:  name = SOLQ_NAME_FLOAT; break;
    case SOLQ_SBAS:   name = SOLQ_NAME_SBAS; break;
    case SOLQ_DGPS:   name = SOLQ_NAME_DGPS; break;
    case SOLQ_SINGLE: name = SOLQ_NAME_SINGLE; break;
    case SOLQ_PPP:    name = SOLQ_NAME_PPP; break;
    case SOLQ_DR:     name = SOLQ_NAME_DR; break;
    default: break;
    }
    return name;
}

extern int gnss_rtk_sol_determine(rtk_t *rtk,sol_t *solspp,sol_t *solpre,int stat,int ns)
{
    int i,solstat=SOLQ_SINGLE,ddtype=DDRES_SINGLE;
    double poso[3]={0.0},posn[3]={0.0};

    if (rtk->kfstatus==RTK_KF_RESET && solspp->stat==SOLQ_SINGLE)
    {
        GLOGI("sol determine: rtk kf was reset. solstat=%d",solstat);
        return solstat;
    }
    if (stat==SOLQ_DGPS && rtk->rtduseflag && rtk->shift.dis2D[0] < 5.0)
    {
        return SOLQ_DGPS;
    }

    for(i=0;i<3;i++) poso[i]=solspp->rr[i];

    if (solspp->stat!=SOLQ_SINGLE)
    {
        solstat=SOLQ_DR;
        ddtype=DDRES_PREDICT;
        for(i=0;i<3;i++) poso[i]=solpre->rr[i];
    }
    else if (rtk->dd_res[DDRES_PREDICT].flag)
    {
#ifndef ENABLE_PPK_MODE
        for(i=0;i<3;i++) posn[i]=solpre->rr[i];
        if (gnss_rtk_sol_compare(rtk,poso,ddtype,posn,DDRES_PREDICT,ns))
        {
            solstat=SOLQ_DR;
            ddtype=DDRES_PREDICT;
            for(i=0;i<3;i++) poso[i]=posn[i];
        }
#endif
    }

    if (stat==SOLQ_NONE)
    {
        GLOGI("sol determine: no meas updated sol. solstat=%d",solstat);
        return solstat;
    }

    //float or dgps status
    if (rtk->dd_res[DDRES_UPDATE].flag)
    {
        for(i=0;i<3;i++) posn[i]=rtk->x[i];
        if (gnss_rtk_sol_compare(rtk,poso,ddtype,posn,DDRES_UPDATE,ns))
        {
            solstat=(rtk->opt.mode<=PMODE_DGPS||stat==SOLQ_DGPS)?SOLQ_DGPS:SOLQ_FLOAT;
            ddtype=DDRES_UPDATE;
            for(i=0;i<3;i++) poso[i]=posn[i];
        }
    }
    if (solstat==SOLQ_SINGLE||solstat==SOLQ_DR)
    {
        GLOGI("sol determine: bad float sol. solstat=%d",solstat);
        if(g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100 || g_pe_cfg.sub_chipType == SUB_CHIPTYPE_HD9310) goto LAB_SINGLE_CHECK;
        else return solstat;
    }
    //fix status
    if (stat==SOLQ_FIX&&rtk->dd_res[DDRES_FIXED].flag)
    {
        for(i=0;i<3;i++) posn[i]=rtk->xa[i];
        if (gnss_rtk_sol_compare(rtk,poso,ddtype,posn,DDRES_FIXED,ns))
        {
            solstat=SOLQ_FIX;
        }
    }

    if (solstat == SOLQ_FLOAT || solstat == SOLQ_FIX || solstat == SOLQ_DGPS)
    {
        if (gnss_rtk_shift_check(rtk))
        {
            solstat=SOLQ_SINGLE;
            GLOGI("sol determine shift check select single instead of float or fix");
        }
    }

LAB_SINGLE_CHECK:
    if (solstat == SOLQ_SINGLE && (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100 || g_pe_cfg.sub_chipType == SUB_CHIPTYPE_HD9310))
    {
        if (rtk->kfcnt > 10 && (gnss_rtk_slctrtk_check(rtk, solspp) == 1 || gnss_rtk_slctspp_check(rtk, solspp) == 0))
        {
            //solstat = (rtk->opt.mode <= PMODE_DGPS || stat == SOLQ_DGPS) ? SOLQ_DGPS : SOLQ_FLOAT;
            solstat = (stat == SOLQ_FIX && rtk->rtduseflag == 1) ? SOLQ_DGPS : SOLQ_FLOAT;
        }
    }

    SYS_LOGGING(OBJ_ALGO, LOG_INFO, "sol determine: use rtk sol. solstat=%s",gnss_rtk_get_sol_name(solstat));
    return solstat;
}
/* check feedback to PE -------------------------------------------------------------
*
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_feedback2pe(rtk_t *rtk, int stat, int ns)
{
    uint8_t i;

    if ((g_pe_cfg.meas_type_mask & MEAS_TYPE_MASK_DR) == 0)
    {
        return;
    }
    if (stat == SOLQ_DGPS)
    {
        if (rtk->shift.flag & SHIFT_FLAG_NO_FEEDBACK)
        {
            return;
        }
        if (rtk->noSkipCnt < 5 || rtk->kfcnt < 3)
        {
            return;
        }
        if (gpz_kf_data->posRes < 1.5 && gpz_kf_data->posResStd < 1.5)
        {
            return;
        }
        if ((rtk->sol.qflag&(QFLAG_PVT_PRRES_GOOD | QFLAG_PVT_KFPOS_GOOD)) == (QFLAG_PVT_PRRES_GOOD | QFLAG_PVT_KFPOS_GOOD))
        {
            return;
        }
    }
    else if (stat == SOLQ_FLOAT)
    {
        if (rtk->shift.flag & SHIFT_FLAG_NO_FEEDBACK)
        {
            return;
        }
        if ((rtk->closkycnt & 0x7) != 0 || rtk->nollicnt < 5 || rtk->kfcnt < 3)
        {
            return;
        }
        if ((rtk->dd_res[DDRES_UPDATE].flag & 0x3) != 0x3 || rtk->dd_res[DDRES_UPDATE].rmsp>=5.0 || rtk->dd_res[DDRES_UPDATE].cntp <= 2)
        {
            return;
        }
        
    }
    else if (stat == SOLQ_FIX)
    {
        //if (g_pe_cfg.ppk_mode == 0) return;

        if (rtk->shift.flag & SHIFT_FLAG_NO_FEEDBACK)
        {
            return;
        }
        if ((rtk->closkycnt & 0x7) != 0 || rtk->nollicnt < 5 || rtk->kfcnt < 3)
        {
            return;
        }
        if (rtk->nbias < 10 || rtk->sol.ratio < 5)
        {
            return;
        }
    }
    else
    {
        return;
    }

    gnss_Kf_SetKfPosVel(rtk->sol.rr, NULL);

    rtk->shift.feedback_flag = TRUE;
    for (i = 0; i < 3; i++) rtk->shift.feedback_rr[i] = rtk->sol.rr[i];

    GLOGI("rtk feedback to PE");
}
extern int gnss_rtk_comfirmamb(double* floatx, double* fixx, double* subx)
{
    int i;
    double a=0.0, b=0.0, c=0.0;
    double ratio_a = 0.0, ratio_b = 0.0, ratio_c = 0.0;
    int is_a = 0/*, is_b = 0*/, is_c = 0, flag = 0;

    for (i = 0;i < 3;i++)
    {
        a += (floatx[i] - fixx[i])* (floatx[i] - fixx[i]);
        b += (floatx[i] - subx[i]) * (floatx[i] - subx[i]);
        c += (subx[i] - fixx[i]) * (subx[i] - fixx[i]);
    }

    if (a < 1e-3 || b < 1e-3 || c < 1e-3) return flag;

    ratio_a = b / a;
    ratio_b = c / b;
    ratio_c = c / a;

    is_a = ratio_a > 200.0 ? 1 : 0;
    //is_b = ratio_b > 200.0 ? 1 : 0;
    is_c = ratio_c > 200.0 ? 1 : 0;

    if (is_a && is_c && ratio_b<1.2)
    {
        flag = 1;
    }

    GLOGI("triangle:%f,%f,%f", ratio_a, ratio_b, ratio_c);

    return flag;
}
/* check amb status -----------------------------------------------------------
*
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
extern int gnss_rtk_amb_gap(double* floatx, double* b, int nb)
{
    int i, gap_a = 0, gap_b = 0, gap_c = 0, flag = 0;

    for (i = 0;i < nb;i++)
    {
        if (fabs(b[i] - b[i + nb]) >= 5) gap_a++;
        if (fabs(b[i] - b[i + nb]) >= 10) gap_b++;
        if (fabs(b[i] - b[i + nb]) >= 15) gap_c++;
    }

    if (gap_a > 0.5 * nb) flag = 1;
    else if (gap_b >= 0.2 * nb && gap_b > 1) flag = 1;

    GLOGI("amb_gap:%d,%d", gap_a, flag);
    return flag;
}
/* convert obsd_t to interface struct -------------------------------------------------------------
* 
* args   : none
* return : 
* author : none
*-----------------------------------------------------------------------------*/
extern int gnss_rtk_obsd2andr(const obsd_t *obs, int n, const nav_t *nav, AGGnss_Data_t *pRawData, GNSS_TIME* pTime)
{
    int i, j, nobs = 0, week = 0, sys, prn, timeset = 0, N4, NT;
    double tow = 0.0, tot=0.0, lam = 0.0, ep[6] = { 0.0 }, tod;
    int loopMax = AGGNSS_MAX_FREQ_NUM < NFREQ ? AGGNSS_MAX_FREQ_NUM : NFREQ;
    AGGnss_Measurement_t*  pSvRawData;
    GnssChannelMeas_t*    pChannelMeas;

    pRawData->size = sizeof(AGGnss_Data_t);
    pRawData->clock.size = sizeof(AGGnss_Clock_t);
    pRawData->clock.flags = 0;

    for (i = 0; i < n; i++)
    {
        if (obs[i].rcv != 1)
        {
            continue;
        }
        if (obs[i].P[0] == 0.0)
        {
            continue;
        }

        if (timeset == 0 && obs[i].time.time > 0)
        {
            tow = time2gpst(obs[i].time, &week);
            pRawData->clock.time_ns = (int64_t)(tow*1e9);
            timeset = 1;
            if (pTime&&pTime->init == FALSE)
            {
                pTime->rcvr_time[GPS_MODE] = tow;
                pTime->week_save[GPS_MODE] = week;
                gnss_time2epoch(gpst2utc(obs[i].time), ep);
                utc2gln(&NT, &N4, &tod, ep);
                pTime->N4_save = N4;
                pTime->NT_save = NT;
                pTime->rcvr_time[GLN_MODE] = tod;
            }
        }
        //measurement convert
        pSvRawData = &pRawData->measurements[nobs];
        pSvRawData->flags = 0;
        pSvRawData->size = sizeof(AGGnss_Measurement_t);

        sys = satsys(obs[i].sat, &prn);
        switch (sys)
        {
        case SYS_GPS: pSvRawData->constellation = AGGNSS_CONSTELLATION_GPS;     break;
        case SYS_SBS: pSvRawData->constellation = AGGNSS_CONSTELLATION_SBAS;    break;
        case SYS_GLO: pSvRawData->constellation = AGGNSS_CONSTELLATION_GLONASS; break;
        case SYS_QZS: pSvRawData->constellation = AGGNSS_CONSTELLATION_QZSS;    break;
        case SYS_CMP: pSvRawData->constellation = AGGNSS_CONSTELLATION_BEIDOU;  break;
        case SYS_GAL: pSvRawData->constellation = AGGNSS_CONSTELLATION_GALILEO; break;
        default:
            continue;
        }
        pSvRawData->svid = prn;
        pSvRawData->reserved1[0] = 0;
        for (j = 0; j < loopMax; j++)
        {
            pChannelMeas = &pSvRawData->channel_meas[j];
            pChannelMeas->c_n0_dbhz = obs[i].SNR[j] * 0.25;
            //TODO:

            pChannelMeas->received_sv_time_uncertainty_in_ns = 0;  //codeDetstd
            pChannelMeas->pseudorange_rate_uncertainty_mps = 0.0;  //dopplerstd

                                                                   //range
            pChannelMeas->pseudorange_m = obs[i].P[j];
            pSvRawData->flags |= AGGNSS_MEASUREMENT_HAS_PSEUDORANGE;
            pSvRawData->used_in_fix = 0;   //TODO

            if (sys == SYS_GPS || sys == SYS_QZS || sys == SYS_GAL)
            {
                tot = time2gpst(timeadd(obs[i].time, -pChannelMeas->pseudorange_m / CLIGHT), NULL);
            }
            else if (sys == SYS_CMP)
            {
                tot = time2bdt(timeadd(gpst2bdt(obs[i].time), -pChannelMeas->pseudorange_m / CLIGHT), NULL);
            }
            else if (sys == SYS_GLO)
            {
                gnss_time2epoch(timeadd(gpst2utc(obs[i].time), -pChannelMeas->pseudorange_m / CLIGHT), ep);
                utc2gln(&NT, &N4, &tod, ep);
                tot = tod;
            }

            pChannelMeas->received_sv_time_in_ns = (int64_t)(tot*1e9);
            if (sys&(SYS_GPS | SYS_QZS | SYS_CMP | SYS_GAL))
            {
                pChannelMeas->state = AGGNSS_MEASUREMENT_STATE_TOW_DECODED;
            }
            else if (sys&SYS_GLO)
            {
                pChannelMeas->state = AGGNSS_MEASUREMENT_STATE_GLO_TOD_DECODED;
            }
            pChannelMeas->multipath_indicator = 0; //TODO

                                                   //doppler
            pChannelMeas->doppler_shift_hz = obs[i].D[j];
            pSvRawData->flags |= AGGNSS_MEASUREMENT_HAS_DOPPLER_SHIFT;
            pChannelMeas->pseudorange_rate_mps = 0.0;
            lam = nav->lam[obs[i].sat - 1][j];  //L1 wavelength
            if (lam > 0.0)
            {
                pChannelMeas->pseudorange_rate_mps = -pChannelMeas->doppler_shift_hz * lam;
            }

            //carrier phase
            pChannelMeas->carrier_frequency_hz = 0;
            pChannelMeas->carrier_cycles = 0;
            pChannelMeas->carrier_phase = fabs(obs[i].L[j]) < 1.0 ? 0.0 : obs[i].L[j];
            pSvRawData->flags |= AGGNSS_MEASUREMENT_HAS_CARRIER_PHASE;
            pChannelMeas->carrier_phase_uncertainty = 0; //unit: cycle
            pChannelMeas->LLI = obs[i].LLI[j];

            pChannelMeas->accumulated_delta_range_state = AGGNSS_ADR_STATE_VALID;
            if (obs[i].LLI[j] != 0)
            {
                pChannelMeas->accumulated_delta_range_state = AGGNSS_ADR_STATE_CYCLE_SLIP;
            }
            if (lam > 0.0)
            {
                pChannelMeas->accumulated_delta_range_m = -lam * pChannelMeas->carrier_phase;
            }
            pChannelMeas->accumulated_delta_range_uncertainty_m = 0.0;
            pSvRawData->reserved1[0] |= obs[i].code[j] << (j * 8);
        }

        nobs++;
    }

    pRawData->measurement_count = nobs;

    return nobs;
}
static uint8_t gnss_rtk_default_obscode(int sys, int freq)
{
    if (freq==0) //L1
    {
        if(sys&(SYS_GPS|SYS_GLO|SYS_QZS|SYS_GAL)) return CODE_L1C;
        if(sys==SYS_CMP) return CODE_L1I;
    }
    else if (freq==1)//L2
    {
        if(sys==SYS_GPS) return CODE_L2W;
        if(sys==SYS_GLO) return CODE_L2P;
        if(sys&(SYS_CMP|SYS_GAL)) return CODE_L7I;
        if(sys==SYS_QZS) return CODE_L2X;
    }
    else if (freq==2) //L3
    {
        if(sys&(SYS_GPS|SYS_QZS)) return CODE_L5Q;
        if(sys==SYS_GAL) return CODE_L5I;
        if(sys==SYS_CMP) return CODE_L6I;
    }

    return CODE_NONE;
}
/* convert interface struct to obs_t data -------------------------------------------------------------
*
* args   : none
* return :
* author : none
*-----------------------------------------------------------------------------*/
#ifdef USED_IN_MC262M
extern int gnss_rtk_andr2obsd(const AGGnss_Data_t *pRawData,const meas_blk_t* pMeas, const nav_t *nav, GNSS_TIME* pTime, obs_t *obsdata)
{
    int i, j, k,gnssmode,freqcnt;
    double lam = 0.0;
    const AGGnss_Measurement_t*  pSvRawData;
    const GnssChannelMeas_t*    pChannelMeas;
    obsd_t *pobs = NULL;
    const gnss_meas_t *pmeas = NULL;

    obsdata->n = 0;
    //pRawData->clock.time_ns
    for (i = 0; i < pRawData->measurement_count && obsdata->n < obsdata->nmax;i++)
    {
        //measurement convert
        pSvRawData = &pRawData->measurements[i];
        pobs = &obsdata->data[obsdata->n];
        memset(pobs, 0, sizeof(obsd_t));
        
        switch (pSvRawData->constellation)
        {
        case AGGNSS_CONSTELLATION_GPS:     pobs->sys = SYS_GPS;     break;
        case AGGNSS_CONSTELLATION_SBAS:    pobs->sys = SYS_SBS;     break;
        case AGGNSS_CONSTELLATION_GLONASS: pobs->sys = SYS_GLO;     break;
        case AGGNSS_CONSTELLATION_QZSS:    pobs->sys = SYS_QZS;     break;
        case AGGNSS_CONSTELLATION_BEIDOU:  pobs->sys = SYS_CMP;     break;
        case AGGNSS_CONSTELLATION_GALILEO: pobs->sys = SYS_GAL;     break;
        default:
            continue;
        }
        gnssmode = GNSS_SYS2MODE(pobs->sys);
        if (gnssmode<GPS_MODE || gnssmode >= GNSS_MAX_MODE)
        {
            continue;
        }
        rtkctrl.satcnt[gnssmode]++;
        if ((pSvRawData->channel_meas[0].LLI&0xFF) ||
            (pSvRawData->channel_meas[0].carrier_cycles + pSvRawData->channel_meas[0].carrier_phase) == 0.0)
        {
            rtkctrl.llisatcnt[gnssmode]++;
        }
        if (!(pobs->sys & rtkctrl.opt.navsys))
        {
            continue;
        }

        pobs->prn = pSvRawData->svid;
        pobs->sat = satno(pobs->sys, pobs->prn);
        if (pobs->sat == 0) continue;
        pobs->rcv = 1;
        pobs->time = gpst2time(pTime->week[GPS_MODE], pTime->rcvr_time[GPS_MODE]);
        freqcnt = 0;
        for (j = 0; j < AGGNSS_MAX_FREQ_NUM && j < NFREQ; j++)
        {
            pChannelMeas = &pSvRawData->channel_meas[j];
            lam = nav->lam[pobs->sat - 1][j];  //wavelength
            //range
            pobs->P[j] = 0.0;
            if ((pSvRawData->flags & AGGNSS_MEASUREMENT_HAS_PSEUDORANGE) || pChannelMeas->pseudorange_m > 0.0)
            {
                pobs->P[j] = pChannelMeas->pseudorange_m;
            }
            
            //doppler
            if (pSvRawData->flags & AGGNSS_MEASUREMENT_HAS_DOPPLER_SHIFT)
            {
                pobs->D[j] = (float)(pChannelMeas->doppler_shift_hz);
            }
            else
            {
                if (lam > 0.0 && pChannelMeas->pseudorange_rate_mps != 0.0)
                {
                    pobs->D[j] = (float)(-pChannelMeas->pseudorange_rate_mps / lam);
                }
                else
                {
                    continue;
                }
            }

            //carrier phase
            if (pSvRawData->flags & AGGNSS_MEASUREMENT_HAS_CARRIER_PHASE)
            {
                pobs->L[j] = (double)pChannelMeas->carrier_cycles + pChannelMeas->carrier_phase;
                pobs->LLI[j] = pChannelMeas->LLI&0xFF;
                if (pobs->L[j] == 0.0)
                {
                    pobs->LLI[j] = 4;
                }
            }
            else
            {
                continue;
            }

            if (rtkctrl.ssat[pobs->sat - 1] != NULL)
            {
                rtkctrl.ssat[pobs->sat - 1]->snr[j] = pobs->SNR[j];
                rtkctrl.ssat[pobs->sat - 1]->trkstate[j] = (uint8_t)((pChannelMeas->LLI>>8)&0xFF);
                rtkctrl.ssat[pobs->sat - 1]->lockms[1][j] = rtkctrl.ssat[pobs->sat - 1]->lockms[0][j];
                rtkctrl.ssat[pobs->sat - 1]->lockms[0][j] = (U16)pSvRawData->reserved1[j + 1];
                rtkctrl.ssat[pobs->sat - 1]->cp_std[j] = (float)pChannelMeas->carrier_phase_uncertainty;
                rtkctrl.ssat[pobs->sat - 1]->pr_std[j] = (float)pChannelMeas->pseudorange_uncertainty_m;
            }
            pobs->SNR[j] = (uint8_t)(pChannelMeas->c_n0_dbhz * 4);
            pobs->code[j] = (uint8_t)((pSvRawData->reserved1[0] >> (j * 8)) & 0xFF);
            if(pobs->code[j]==0 && pobs->SNR[j]>0) {
                pobs->code[j]=gnss_rtk_default_obscode(pobs->sys, j);
            }

            //if (j == 0)
#if 0
            {
                gnss_rtk_obsl1_save(j, pobs, pobs->LLI[j], 1);
            }
#endif
            freqcnt++;

            for (k = 0;k < pMeas->measCnt;k++)
            {
                if ((pMeas->meas[k].gnssMode == GNSS_SYS2MODE(pobs->sys)) && (pMeas->meas[k].prn == pobs->prn) && (pMeas->meas[k].freq_index == j))
                {
                    pmeas = &pMeas->meas[k];
                    break;
                }
            }
            //get more pvt info
            if (k >= pMeas->measCnt)
            {
                if (j == 0)
                {
                    freqcnt = 0;
                    break;
                }
                continue; //do not use this sat that it has no L1 data
            }
            if (j == 0)
            {
                
                rtkctrl.ssat[pobs->sat - 1]->usectrl.pr_diff = pmeas->post_prdiff == 0.0 ? pmeas->pr_diff : pmeas->post_prdiff;
                rtkctrl.ssat[pobs->sat - 1]->usectrl.dr_diff = (g_pe_cfg.meas_type_mask&MEAS_TYPE_MASK_DR) ? (pmeas->post_drdiff == 0.0 ? pmeas->dr_diff : pmeas->post_drdiff) : 0.0;
                rtkctrl.ssat[pobs->sat - 1]->azel[0] = pmeas->sv_info.fltAz*D2R;
                rtkctrl.ssat[pobs->sat - 1]->azel[1] = pmeas->sv_info.fltElev*D2R;
                rtkctrl.ssat[pobs->sat - 1]->vs = pmeas->status&PE_MEAS_VALID_PR;
                //rtkctrl.ssat[pobs->sat - 1].usectrl.rqflag[j] = pmeas->status;
            }

            rtkctrl.ssat[pobs->sat - 1]->usectrl.pr_qoschck[j] = pmeas->prWeightChck;
            rtkctrl.ssat[pobs->sat - 1]->usectrl.dr_qoschck[j] = (g_pe_cfg.meas_type_mask&MEAS_TYPE_MASK_DR) ? pmeas->drWeightChck : 0;

            rtkctrl.ssat[pobs->sat - 1]->usectrl.rqflag[j] = pmeas->status;
        }

        if (freqcnt < 1) continue;

        obsdata->n++;
    }

    return obsdata->n;
}
#endif
static int gnss_rtk_asgobsd2rtkobs(asg_obs_t* asg_obs, const meas_blk_t* pMeas,  GNSS_TIME* pTime, obs_t* obsdata)
{
    uint32_t k;
    int i, j, gnssmode, freqcnt;
    const gnss_meas_t* pmeas = NULL;
    asg_obsd_t* src = NULL;
    obsd_t*      dst = NULL;
    sat_data_t*    sp;
    
    obsdata->n = 0;
    for (i = 0; i < asg_obs->n && obsdata->n < obsdata->nmax; i++)
    {
        //measurement convert
        src = &asg_obs->data[i];
        dst = &obsdata->data[obsdata->n];
        memset(dst, 0, sizeof(obsd_t));

        dst->sys = src->sys;
        dst->prn = src->prn;
        dst->sat = satno(dst->sys, dst->prn);
        if (dst->sat == 0) {
            continue;
        }
        
        gnssmode = GNSS_SYS2MODE(dst->sys);
        if (gnssmode<GPS_MODE || gnssmode >= GNSS_MAX_MODE)
        {
            continue;
        }
        rtkctrl.satcnt[gnssmode]++;
        if ((src->LLI[0]&0xFF) || (src->L[0] == 0.0))
        {
            rtkctrl.llisatcnt[gnssmode]++;
        }
        if (pMeas->meas[i].cycleSlipCount == 0 || pMeas->meas[i].cycleSlipCount == 2)
        {
            rtkctrl.noSkipCnt++;
        }
        if (pMeas->meas[i].status&PE_MEAS_VALID_PR)
        {
            rtkctrl.pvtsatcnt[gnssmode]++;		  //L1 pr is valid sat count	
        }
            
        dst->rcv = 1;
        dst->time = gnss_gpst2time(pTime->week[GPS_MODE], pTime->rcvr_time[GPS_MODE]);

        freqcnt = 0;		
        for (j = 0; j < NFREQ; j++)
        {	
            pmeas = NULL;
            for (k = 0;k < pMeas->measCnt;k++)
            {
                if ((pMeas->meas[k].gnssMode == GNSS_SYS2MODE(dst->sys)) && 
                    (pMeas->meas[k].prn == dst->prn))
                {
                    if ((pMeas->meas[k].freq_index == 0) && (j == 0))
                    {
                        pmeas = &pMeas->meas[k];
                        break;
                    }
#ifdef FREQ_L1L5
                    else if ((pMeas->meas[k].freq_index == 2) && (j == 1))
                    {
                        pmeas = &pMeas->meas[k];
                        break;
                    }
#else
                    else if ((pMeas->meas[k].freq_index == 1) && (j == 1))
                    {
                        pmeas = &pMeas->meas[k];
                        break;
                    }
#endif
                }
            }
            if (pmeas == NULL)
            {
                continue;
            }
            
            dst->code[j] = src->code[j];
            dst->P[j] = pmeas->pseudoRange_raw;
            dst->D[j] = src->D[j];
            dst->L[j] = pmeas->carrierPhase;
            dst->LLI[j] = src->LLI[j];
            
            if (dst->L[j] == 0.0)
            {
                dst->LLI[j] = 4;
            }	
            
            dst->SNR[j] = (uint8_t)(src->SNR[j] * 4);
#if 0
            gnss_rtk_obsl1_save(j, dst, dst->LLI[j], 1);
#endif

            if (rtkctrl.ssat[dst->sat - 1] != NULL)
            {
                rtkctrl.ssat[dst->sat - 1]->snr[j] = dst->SNR[j];
#ifdef USED_IN_MC262M
                rtkctrl.ssat[dst->sat - 1]->trkstate[j] = (uint8_t)((dst->LLI[j]>>8)&0xFF);
                rtkctrl.ssat[dst->sat - 1]->lockms[1][j] = rtkctrl.ssat[dst->sat - 1]->lockms[0][j];
                rtkctrl.ssat[dst->sat - 1]->cp_std[j] = (float)pmeas->carrierUnc;
                rtkctrl.ssat[dst->sat - 1]->pr_std[j] = 0.0;
#endif


                if (j == 0)
                {
                    sp = gnss_sd_get_sv_data(pmeas->gnssMode, pmeas->prn, pmeas->freq_index);

                    rtkctrl.ssat[dst->sat - 1]->usectrl.pr_diff = pmeas->post_prdiff == 0.0 ? pmeas->pr_diff : pmeas->post_prdiff;
                    rtkctrl.ssat[dst->sat - 1]->usectrl.dr_diff = (g_pe_cfg.meas_type_mask&MEAS_TYPE_MASK_DR) ? (pmeas->post_drdiff == 0.0 ? pmeas->dr_diff : pmeas->post_drdiff) : 0.0f;
                    if (sp != NULL)
                    {
                        rtkctrl.ssat[dst->sat - 1]->azel[0] = (double)sp->d_cos.az;
                        rtkctrl.ssat[dst->sat - 1]->azel[1] = (double)sp->d_cos.elev;
                    }
                    rtkctrl.ssat[dst->sat - 1]->vs = pmeas->status&PE_MEAS_VALID_PR;
                }

                rtkctrl.ssat[dst->sat - 1]->usectrl.pr_qoschck[j] = pmeas->prWeightChck;
                rtkctrl.ssat[dst->sat - 1]->usectrl.dr_qoschck[j] = (g_pe_cfg.meas_type_mask&MEAS_TYPE_MASK_DR) ? pmeas->drWeightChck : 0;
                rtkctrl.ssat[dst->sat - 1]->usectrl.rqflag[j] = pmeas->status;

                if ((pmeas->status & 0xF) == 0) //marked invalid meas in PE
                {
                    dst->P[j] = 0.0;
                    dst->L[j] = 0.0;
                    dst->D[j] = 0.0;
                    continue;
                }
                freqcnt++;
            }
            
        }

        if (freqcnt < 1) {
            continue;
        }

        obsdata->n++;
    }

    return obsdata->n;
}

/* convert interface struct to obs_t data for QCOM chiptype -------------------------------------------------------------
*
* args   : none
* return :
* author : none
*-----------------------------------------------------------------------------*/
extern int gnss_rtk_andr2obsd_qcom(const AGGnss_Data_t *pRawData, const meas_blk_t* pMeas, const nav_t *nav, GNSS_TIME* pTime, obs_t *obsdata)
{
    uint32_t k;
    int j, gnssmode, freqcnt;
    uint64_t i = 0;
    double lam = 0.0;
    const AGGnss_Measurement_t*  pSvRawData;
    const GnssChannelMeas_t*    pChannelMeas;
    obsd_t *pobs = NULL;
    const gnss_meas_t *pmeas = NULL;
    int loopMax = AGGNSS_MAX_FREQ_NUM < NFREQ ? AGGNSS_MAX_FREQ_NUM : NFREQ;

    obsdata->n = 0;
    //pRawData->clock.time_ns
    for (i = 0; i < pRawData->measurement_count && obsdata->n < obsdata->nmax; i++)
    {
        //measurement convert
        pSvRawData = &pRawData->measurements[i];
        pobs = &obsdata->data[obsdata->n];
        memset(pobs, 0, sizeof(obsd_t));

        switch (pSvRawData->constellation)
        {
        case AGGNSS_CONSTELLATION_GPS:     pobs->sys = SYS_GPS;     break;
        case AGGNSS_CONSTELLATION_SBAS:    pobs->sys = SYS_SBS;     break;
        case AGGNSS_CONSTELLATION_GLONASS: pobs->sys = SYS_GLO;     break;
        case AGGNSS_CONSTELLATION_QZSS:    pobs->sys = SYS_QZS;     break;
        case AGGNSS_CONSTELLATION_BEIDOU:  pobs->sys = SYS_CMP;     break;
        case AGGNSS_CONSTELLATION_GALILEO: pobs->sys = SYS_GAL;     break;
        default:
            continue;
        }
        gnssmode = GNSS_SYS2MODE(pobs->sys);
        if (gnssmode < GPS_MODE || gnssmode >= GNSS_MAX_MODE)
        {
            continue;
        }
        if (!(pobs->sys & rtkctrl.opt.navsys))
        {
            continue;
        }

        pobs->prn = pSvRawData->svid;
        pobs->sat = satno(pobs->sys, pobs->prn);
        if (pobs->sat == 0) continue;
        pobs->rcv = 1;
        pobs->time = gnss_gpst2time(pTime->week[GPS_MODE], pTime->rcvr_time[GPS_MODE]);
        freqcnt = 0;
        for (j = 0; j < loopMax; j++)
        {
            for (k = 0; k < pMeas->measCnt; k++)
            {
                if ((pMeas->meas[k].gnssMode == GNSS_SYS2MODE(pobs->sys)) && (pMeas->meas[k].prn == pobs->prn) && (pMeas->meas[k].freq_index == j))
                {
                    pmeas = &pMeas->meas[k];
                    break;
                }
            }
            //get more pvt info
            if (k >= pMeas->measCnt || NULL == pmeas)
            {
                continue;
            }
            if ((pmeas->status & 0xF) == 0) //marked invalid meas in PE
            {
                continue;
            }

            pChannelMeas = &pSvRawData->channel_meas[j];
            lam = nav->lam[pobs->sat - 1][j];  //wavelength
            if (lam<1e-4)
            {
                int fcn = (pmeas->gnssMode == GLN_MODE) ? gnss_Sd_Nm_GetGlnChanN(pmeas->prn) : 0;
                lam = ag_getwavelen(GNSS_MODE2SYS(pmeas->gnssMode, pmeas->prn), pmeas->freq_index, fcn);
            }

            //range
            pobs->P[j] = pmeas->pseudoRange_raw > 0.0 ? pmeas->pseudoRange_raw : 0.0;

            //doppler
            pobs->D[j] = 0.0;
            if (lam > 1e-4 && fabs(pmeas->pseudoRangeRate_raw) > 1e-4)
            {
                pobs->D[j] = (float)(-pmeas->pseudoRangeRate_raw / lam);
            }

            //carrier phase
            pobs->L[j] = 0.0;
            if ((pSvRawData->flags & AGGNSS_MEASUREMENT_HAS_CARRIER_PHASE))
            {
                pobs->L[j] = (double)pChannelMeas->carrier_cycles + pChannelMeas->carrier_phase;
                pobs->LLI[j] = pChannelMeas->LLI & 0xFF;
            }
            else if (fabs(pChannelMeas->accumulated_delta_range_m) > 1e-3 && pmeas->carrierPhase != 0.0)
            {
                pobs->L[j] = pmeas->carrierPhase;
                pobs->LLI[j] = pmeas->cycleSlipCount < 0 ? 0x3 : (pmeas->cycleSlipCount & 0x3);
            }

            if (pobs->L[j] == 0.0)
            {
                pobs->LLI[j] = 4;
            }

            pobs->SNR[j] = (uint8_t)(pChannelMeas->c_n0_dbhz * 4);

            if (pobs->SNR[j] > 0) {	
                pobs->code[j] = gnss_rtk_default_obscode(pobs->sys, j);
            }
#if 0
            gnss_rtk_obsl1_save(j, pobs, pobs->LLI[j], 1);
#endif
            freqcnt++;

            if (rtkctrl.ssat[pobs->sat - 1] != NULL)
            {
                rtkctrl.ssat[pobs->sat - 1]->snr[j] = pobs->SNR[j];
                if (j == 0)
                {
                    rtkctrl.ssat[pobs->sat - 1]->usectrl.pr_diff = pmeas->post_prdiff == 0.0 ? pmeas->pr_diff : pmeas->post_prdiff;
                    rtkctrl.ssat[pobs->sat - 1]->usectrl.dr_diff = (g_pe_cfg.meas_type_mask&MEAS_TYPE_MASK_DR) ? (pmeas->post_drdiff == 0.0 ? pmeas->dr_diff : pmeas->post_drdiff) : 0.0f;
                    rtkctrl.ssat[pobs->sat - 1]->azel[0] = pmeas->sv_info.fltAz*D2R;
                    rtkctrl.ssat[pobs->sat - 1]->azel[1] = pmeas->sv_info.fltElev*D2R;
                    rtkctrl.ssat[pobs->sat - 1]->vs = pmeas->status&PE_MEAS_VALID_PR;
                    //rtkctrl.ssat[pobs->sat - 1].usectrl.rqflag[j] = pmeas->status;

                    rtkctrl.satcnt[gnssmode]++;
                    if (pobs->LLI[j])
                    {
                        rtkctrl.llisatcnt[gnssmode]++;
                    }
                }

                rtkctrl.ssat[pobs->sat - 1]->usectrl.pr_qoschck[j] = pmeas->prWeightChck;
                rtkctrl.ssat[pobs->sat - 1]->usectrl.dr_qoschck[j] = (g_pe_cfg.meas_type_mask&MEAS_TYPE_MASK_DR) ? pmeas->drWeightChck : 0;
                rtkctrl.ssat[pobs->sat - 1]->usectrl.rqflag[j] = pmeas->status;
            }
            
        }

        if (freqcnt < 1) continue;

        obsdata->n++;
    }

    return obsdata->n;
}
/* convert obsd_t to local rtcm struct -------------------------------------------------------------
* 
* args   : none
* return : 
* author : none
*-----------------------------------------------------------------------------*/
extern int gnss_rtk_obsd2rtcm(const obsd_t *obs, int n, rtcm_data_pack_t *rtcmdata)
{
    int i,nobs=0,ngps=0,nglo=0,nbds=0,ngal=0,sys;
    rtcm3_rtk_t *rtcm3;

    rtcm3=&rtcmdata->rtcm3_data;
    rtcm3->msg_mask=0;
    for (i=0;i<n;i++)
    {
        if (obs[i].rcv!=2)
        {
            continue;
        }
        rtcm3->obsd[nobs]=obs[i];

        sys=satsys(obs[i].sat,NULL);
        switch (sys)
        {
        case SYS_GPS: ngps++; break;
        case SYS_GLO: nglo++; break;
        case SYS_CMP: nbds++; break;
        case SYS_GAL: ngal++; break;
        }
        nobs++;
    }

    if (ngps>0)
    {
        rtcm3->msg_mask|=RTCM3_MSG_1074;
    }
    if (nglo>0)
    {
        rtcm3->msg_mask|=RTCM3_MSG_1084;
    }
    if (nbds>0)
    {
        rtcm3->msg_mask|=RTCM3_MSG_1124;
    }
    if (ngal>0)
    {
        rtcm3->msg_mask|=RTCM3_MSG_1094;
    }

    rtcm3->obsn=nobs;
    rtcm3->obsflag=1;

    return nobs;
}
/* convert nav_t to interface eph -------------------------------------------------------------
* convert nav_t to interface eph
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_nav2andr(GNSS_TIME* pTime, int sys,int prn, void *praweph, void *prtkeph)
{

}
/* convert nav_t to PE eph -------------------------------------------------------------
* convert nav_t to PE eph
* args   : none
* return : none
* author : none
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_nav_revconv(int sys,int prn, void *praweph, void *prtkeph)
{
    eph_t *peph=NULL;
    int week=0;

    if (!praweph||!prtkeph)
    {
        return;
    }
    
    switch (sys)
    {
    case SYS_GPS:
    case SYS_QZS:
        {
            GPS_EPH_INFO *praw=(GPS_EPH_INFO *)praweph;
            peph=(eph_t*)prtkeph;
            
            praw->eph_status=peph->svh==0?EPH_STATUS_VALID:EPH_STATUS_INVALID;
            praw->eph_source=EPH_SOURCE_AGNSS_L;
            praw->IODE=peph->iode;
            praw->fit_interval=(uint8_t)peph->fit;

            praw->subframe1.IODC=peph->iodc;
            praw->subframe1.udre=peph->sva;
            praw->subframe1.SV_health=peph->svh;
            praw->subframe1.codeL2=peph->code;
            praw->subframe1.L2Pdata=peph->flag;
            praw->subframe1.t_oc=(float)time2gpst(peph->toc,&week);
            praw->subframe1.weeknum=peph->week?peph->week:week;
            praw->subframe1.a_f0=(float)peph->f0;
            praw->subframe1.a_f1=(float)peph->f1;
            praw->subframe1.a_f2=(float)peph->f2;
            praw->subframe1.T_GD=(float)peph->tgd[0];
            if(peph->sva < 6) praw->subframe1.SVacc  = (float)pow(2,(1.0 + peph->sva/2));          /* SV accuracy; meters*/
            else              praw->subframe1.SVacc  = (float)pow(2,(peph->sva - 2.0));          /* SV accuracy; meters*/

            praw->sqrt_A=sqrt(peph->A);
            praw->e=peph->e;
            praw->i_0=peph->i0;
            praw->OMEGA_0=peph->OMG0;
            praw->omega=peph->omg;
            praw->M_0=peph->M0;
            praw->delta_n=peph->deln;
            praw->OMEGADOT=peph->OMGd;
            praw->IDOT=peph->idot;
            praw->C_rc=(float)peph->crc;
            praw->C_rs=(float)peph->crs;
            praw->C_uc=(float)peph->cuc;
            praw->C_us=(float)peph->cus;
            praw->C_ic=(float)peph->cic;
            praw->C_is=(float)peph->cis;
            praw->t_oe=(float)peph->toes;  //real toe
        }
        break;
    case SYS_GLO:
        {
            int N4=0,NT=0;
            double utc[6]={0.0},tod;
            GLN_EPH_INFO *praw=(GLN_EPH_INFO*)praweph;
            geph_t *pglo=(geph_t*)prtkeph;
            
            praw->eph_status=pglo->svh==0?EPH_STATUS_VALID:EPH_STATUS_INVALID;
            praw->eph_source=EPH_SOURCE_AGNSS_L;
            praw->Health=pglo->svh;
            
            gnss_Sd_Nm_SetGlnChanN(prn, (int8_t)pglo->frq);
            praw->Bn=(pglo->svh<<2)&0x7;
            praw->n=prn;

            gnss_time2epoch(gpst2utc(pglo->toe),utc);
            utc2gln(&NT,&N4,&tod,utc);
            praw->N4=N4;
            praw->NT=NT;
            praw->tb=tod;

            memcpy(praw->r,pglo->pos,sizeof(double)*3);
            memcpy(praw->v,pglo->vel,sizeof(double)*3);
            memcpy(praw->a,pglo->acc,sizeof(double)*3);

            praw->taun_tb=pglo->taun;
            praw->gaman_tb=pglo->gamn;
            praw->delta_taun=pglo->dtaun;
        }
        break;
    case SYS_CMP:
        {
            BDS_EPH_INFO *praw=(BDS_EPH_INFO *)praweph;
            peph=(eph_t*)prtkeph;

            praw->eph_status=peph->svh==0?EPH_STATUS_VALID:EPH_STATUS_INVALID;
            praw->eph_source=EPH_SOURCE_AGNSS_L;
            praw->prn=prn;
            praw->IODE=peph->iode;
            praw->IODC=peph->iodc;
            praw->URAI=peph->sva;
            praw->SatH1=peph->svh;
            
            praw->toc=(uint32_t)time2bdt(gpst2bdt(peph->toc),&week);
            praw->WN=peph->week?peph->week:week;
            
            praw->sqrta=sqrt(peph->A);
            praw->ecc=peph->e;
            praw->i0=peph->i0;
            praw->OMEGA_0=peph->OMG0;
            praw->w=peph->omg;
            praw->M0=peph->M0;
            praw->delta_n=peph->deln;
            praw->OMEGA_Dot=peph->OMGd;
            praw->idot=peph->idot;
            praw->crc=(float)peph->crc;
            praw->crs=(float)peph->crs;
            praw->cuc=(float)peph->cuc;
            praw->cus=(float)peph->cus;
            praw->cic=(float)peph->cic;
            praw->cis=(float)peph->cis;
            praw->toe=(uint32_t)peph->toes;  //real toe
            praw->af0=peph->f0;
            praw->af1=(float)peph->f1;
            praw->af2=(float)peph->f2;
            praw->TGD1=(float)(peph->tgd[0]*1e9);
            praw->TGD2=(float)(peph->tgd[1]*1e9);
        }
        break;
    case SYS_GAL:
        {
            GAL_EPH_INFO *praw=(GAL_EPH_INFO *)praweph;
            peph=(eph_t*)prtkeph;

            praw->eph_status=peph->svh==0?EPH_STATUS_VALID:EPH_STATUS_INVALID;
            praw->eph_source=EPH_SOURCE_AGNSS_L; //praw->eph_source=peph->code;
            praw->prn = prn;
            praw->IOD=peph->iode;
            praw->sisa=peph->sva;
            praw->svHealth=peph->svh;
            
            praw->navtype=peph->flag;
            praw->toc=(int32_t)time2gst(peph->toc,&week);
            praw->WN=peph->week?peph->week:week;
            praw->af0=peph->f0;
            praw->af1=(float)peph->f1;
            praw->af2=(float)peph->f2;
            praw->bgd_e5a=peph->tgd[0];
            praw->bgd_e5b=peph->tgd[1];

            praw->sqrta=sqrt(peph->A);
            praw->ecc=peph->e;
            praw->i0=peph->i0;
            praw->OMEGA_0=peph->OMG0;
            praw->w=peph->omg;
            praw->M0=peph->M0;
            praw->delta_n=peph->deln;
            praw->OMEGA_Dot=peph->OMGd;
            praw->idot=peph->idot;
            praw->crc=(float)peph->crc;
            praw->crs=(float)peph->crs;
            praw->cuc=(float)peph->cuc;
            praw->cus=(float)peph->cus;
            praw->cic=(float)peph->cic;
            praw->cis=(float)peph->cis;
            praw->toe=(int32_t)peph->toes;  //real toe
        }
        break;
    default:
        break;
    }

}
/* nav inject to PE module -------------------------------------------------------------
* nav inject to PE module
* args   : opt 1:inject current sat list nav; opt 0:inject all sat nav
* return : none
* author : none
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_nav_inject2pe(const obsd_t *obs, int n, const nav_t *nav, int opt)
{
    int i,sys,prn,sat,cnt=(opt?n:(MAXSAT+1));
    void* gnsseph=NULL;
    void* peeph=NULL;
    GPS_EPH_INFO gpse;
    GLN_EPH_INFO gloe;
    BDS_EPH_INFO bdse;
    GAL_EPH_INFO gale;
    
    for (i=(opt?0:1);i<cnt;i++)
    {
        sat=opt?obs[i].sat:i;
        sys=satsys(sat,&prn);

        if ((sys&(SYS_GPS|SYS_GLO|SYS_CMP|SYS_QZS|SYS_GAL))==0)  //only support GRCJE now
        {
            continue;
        }

        gnsseph=selephfromnav(opt?obs[i].time:obs[0].time, sat, nav, -1);

        if (!gnsseph)
        {
            continue;
        }

        if (sys==SYS_GPS || sys==SYS_QZS)
        {
            peeph=&gpse;
            memset(peeph,0,sizeof(GPS_EPH_INFO));
        }
        else if (sys==SYS_GLO)
        {
            peeph=&gloe;
            memset(peeph,0,sizeof(GLN_EPH_INFO));
        }
        else if (sys==SYS_CMP)
        {
            peeph=&bdse;
            memset(peeph,0,sizeof(BDS_EPH_INFO));
        }
        else if (sys==SYS_GAL)
        {
            peeph=&gale;
            memset(peeph,0,sizeof(GAL_EPH_INFO));
        }
        else
        {
            continue;
        }

        gnss_rtk_nav_revconv(sys,prn,peeph,gnsseph);

        gnss_Sd_Nm_AddEph(GNSS_SYS2MODE(sys), prn, peeph);
    }

}
/* load measurements from rtcm string -------------------------------------------------------------
* 
* args   : none
* return : 
* author : none
*-----------------------------------------------------------------------------*/
extern int gnss_rtk_loadrtcm3(gtime_t time,ag_rtcm_t *_rtcm,FILE *fp,rtcm_data_pack_t *rtcmdata)
{
#if 0
    int ret;
    static int timeinit=0;
    static rtcm_data_pack_t rtcm_newer;
    static rtcm_data_pack_t rtcm_bak;
    double dt=0;
    rtcm_t* rtcm;

    if (!_rtcm || !_rtcm->rtcm) return -1;
    rtcm = _rtcm->rtcm;

    if (!timeinit)
    {
        rtcm->time=time;
        timeinit=1;
    }

    rtcmdata->rtcm3_data.msg_mask=0;
    rtcm_bak.rtcm3_data.msg_mask=0;

    if (rtcm_newer.rtcm3_data.msg_mask)
    {
        if (timediff(rtcm_newer.rtcm3_data.obsd[0].time,time)>0.0)
        {
            return 0; //not update rtcm
        }
        *rtcmdata=rtcm_newer;
        rtcm_bak=*rtcmdata;
        rtcmdata->rtcm3_data.msg_mask=0; //clear
    }
    /* read rtcm file as per specified time */
    while (1) 
    {
        ret=Gnss_Sys_InputRTCMv3File(_rtcm,fp); //input_rtcm3f(rtcm,fp);
        if (ret<-1)      break;     //end of file
        else if (ret<=0) continue;  //no msg or err msg
        else
        {
            Gnss_Sys_output_rtcm3(_rtcm, ret, rtcmdata);
            if (ret==1)
            {
                //time compare
                if (rtcmdata->rtcm3_data.obsn <= 0)  continue;

                dt=timediff(rtcm->obs.data[0].time,time);
                if (dt<=0.0)
                {
                    rtcmdata->rtcm3_data.msg_mask |= rtcm_bak.rtcm3_data.msg_mask;
                    rtcm_bak=*rtcmdata;
                    rtcmdata->rtcm3_data.msg_mask=0; //clear
                    continue;
                }
                else
                {
                    //reserve it for next update?
                    rtcm_newer=*rtcmdata;
                    if (rtcm_bak.rtcm3_data.msg_mask)
                    {
                        *rtcmdata=rtcm_bak;
                    }
                    else
                    {
                        rtcmdata->rtcm3_data.msg_mask=0;
                    }
                    break;
                }
            }
        }

    }

    return rtcmdata->rtcm3_data.msg_mask!=0;
#else
    return -1;
#endif
}
/* zero baseline analyze -------------------------------------------------------------
* 
* args   : none
* return : 
* author : none
*-----------------------------------------------------------------------------*/
extern int gnss_rtk_zerobaseline(FILE* fp, const obsd_t *obs, int n)
{
    int nu,nr,week,i,j,k=0;
    double tow,ep[6]={0.0},sdP,sdL,epr[6]={0.0};
    
    /* count rover/base station observations */
    for (nu=0;nu   <n&&obs[nu   ].rcv==1;nu++) ;
    for (nr=0;nu+nr<n&&obs[nu+nr].rcv==2;nr++) ;

    if (!nu||!nr)
    {
        return 0;
    }
    tow=time2gpst(obs[0].time,&week);
    gnss_time2epoch(obs[0].time,ep);
    gnss_time2epoch(obs[nu].time,epr);

    fprintf(fp,"week %d tow %.3f rov %d:%d:%.3f ref %d:%d:%.3f\n", week,tow,(int)(ep[3]),(int)(ep[4]),ep[5],(int)(epr[3]),(int)(epr[4]),epr[5]);

    for (i=0,j=nu;i<nu&&j<nu+nr;i++,j++) 
    {
        if      (obs[i].sat<obs[j].sat) j--;
        else if (obs[i].sat>obs[j].sat) i--;
        else if (fabs(obs[i].L[0])>1.0&&fabs(obs[j].L[0])>1.0) 
        {
            sdP=obs[i].P[0]-obs[j].P[0];
            sdL=obs[i].L[0]-obs[j].L[0];

            fprintf(fp,"%4d %20.3f %20.3f %4d %4d\n",obs[i].sat,sdP,sdL,obs[i].LLI[0],obs[j].LLI[0]);
            k++;
        }
    }

    return k;
}
/* adjust R factor -------------------------------------------------------------
* 
* args   : none
* return : 
* author : none
*-----------------------------------------------------------------------------*/
extern double gnss_rtk_adjerror(rtk_t *rtk,int isat,int nsat,double v,int f,double R)
{	
    int i=0, nf = (rtk->opt.ionoopt == IONOOPT_IFLC) ? 1 : rtk->opt.nf, ff=f%nf;
    float factor_speed =1.0;
    float factor_scene = 1.0;
    float factor_res=1.0;
    float power=1.0;
    float grad=1.0;
    ssat_t *psat = rtk->ssat[isat - 1];
    if (NULL == psat)
    {
        return (double)power;
    }
    uint8_t *rescnt = psat->usectrl.avalbl;
    int pr_qoscnt = sumofbit1(psat->usectrl.pr_qoschck[ff], 16);
    int dr_qoscnt = sumofbit1(psat->usectrl.dr_qoschck[ff], 16);
    
    if ((rtk->closkycnt&0xf)==0xf)
    {
        factor_scene=6.0;//2.0
    }

    if((g_pe_cfg.meas_type_mask&MEAS_TYPE_MASK_DR) && (peMode.staticData.lsVel<2.0||peMode.staticData.deltaDoplVar<1.0))
    {
        factor_speed=8.0;//6.0
    }

    if(peMode.staticData.lsVel>15.0&&peMode.staticData.deltaDoplVar>5.0&&(rtk->closkycnt&0xf)!=0x0)
    {
        factor_scene=1.0;
        factor_speed=1.0;
        if(f<nf)
        {
            if(v>10.0) grad=10.0;
            else if(v>5.0) grad=3.0;
            else if(v>1.0) grad=1.5;
        }
        else
        {
            if(v>6.0) grad=5.0;
            else if(v>1.0) grad=2.0;
        }
    }
    else if (f<nf) //phase
    {
        if(v>10.0) grad=10.0;
        else if(v>5.0) grad=5.0;
        else if(v>3.0) grad=3.0;
        else if(v>1.0) grad=(float)v;
    }
    else //code
    {
        if(v>1.0) grad=(float)v;
    }

    if(f<nf)//phase
    {
        i=rescnt[0];
        i<<=1;
        i|=v>1.0?1:0;
        rescnt[0]=(i&0xF);
    }
    else//code
    {
        i=rescnt[1];
        i<<=1;
        i|=v>2.0?1:0;
        rescnt[1]=(i&0xF);
    }

    if(sumofbit1(i,4)>=3)
    {
        factor_res=grad*factor_speed*factor_scene;
    }
    else if(v>1.0)
    {
        grad=(float)(grad>2.0?(grad/2.0):1.0);
        factor_res=grad*factor_speed;
    }

    if(rtk->dd_res[DDRES_SINGLE].cntc < 5 && (rtk->closkycnt & 0xFF) != 0 && (rtk->qcpasscnt & 0xF) == 0
       && nsat < 15 && gpz_kf_data->meas_blk->avgCno < 38)
    {
        if (f < nf || v > 3.0) factor_res *= 1000.0;
    }

    if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_BRCM_47755 && f < nf && (psat->slip[f] & 1) != 0 && v > 0.5) factor_res *= 10.0;

    if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_ST_8100)
    {
        if (f < nf && v < 2.5 && nsat > 15 && gpz_kf_data->meas_blk->avgCno > 37 && rtk->shift.nofixc > 6 && rtk->kfcnt > 10)
        {
            factor_res /= 10.0;
        }
    }

    if (first_static_period == 1 && f >= nf) factor_res *= 1e2;

    power=factor_res;

    if (g_pe_cfg.sub_chipType != SUB_CHIPTYPE_BRCM_47755 && f < nf && rtk->rtduseflag && 
        (rtk->qcpasscnt & 0xFF) == 0 && rtk->kfcnt < 10 /*&& R*power<1.0*/)
    {
        power *= (float)(v / R);
    }


    GLOGD("%c%d %3d v=%.1f factor[v=%.1f sc=%.1f grad=%.1f] diff[pr=%4.1f dr=%4.1f] rq_flag=%d slip=%d cn0=%2u ele=%4.1f R=%.6f pow=%.1f",
        f < nf ? 'L' : 'P', f%nf + 1, isat, v, factor_speed, factor_scene, grad, psat->usectrl.pr_diff, psat->usectrl.dr_diff,
        psat->usectrl.rqflag[ff] & 0x3, psat->slip[ff] & 0x3, psat->snr[ff] / 4, psat->azel[1] * R2D, R*power, power);

    return power;
}

extern int gnss_rtk_refsat_check(rtk_t *rtk,int *nrej,double *vrej,double *vval,int ns,int *refsat,int *nval)
{
    int i, j, t = 0, m, ntol = 0, flag[NSYSDD] = { 0 };
    double *v,mean,std;
    double *vv, median, std_dist, mean_dist;

    if(ns<=0) return 0;

    v=mat(ns,1); vv = mat(ns, 1);
    if (v == NULL || vv == NULL) {
        if (v != NULL)  Sys_Free(v);
        if (vv != NULL)  Sys_Free(vv);
        return 0;
    }

    for (m=0;m<NSYSDD;m++)
    {
        i=refsat[m];
        if(i==-1) continue;
        ntol = nrej[m] + nval[m];
        mean=0; std=0;
        
        if (ntol > ns) continue;
        if (i <= 0 || NULL == rtk->ssat[i - 1])
        {
            continue;
        }
        if (nrej[m] > 4 || (ntol > 3 && nrej[m] > ntol / 2))  //change ref sat by reject
        {
            for (j = 0;j<nrej[m];j++) v[j] = fabs(vrej[j * NSYSDD + m]);
            gnss_math_dstd(v, nrej[m], &mean, &std);

            rtk->ssat[i - 1]->usectrl.canberef |= 1;
            GLOGI("refsat_check:reject too much tor=%.6f nsrej=%2d nstotal=%2d refsat=%2d mean=%.3f std=%.3f prdiff=%.3f", 
                rtk->ptor, nrej[m], ntol, i, mean, std, rtk->ssat[i - 1]->usectrl.pr_diff);

            flag[m] = 1;
        }
        else  //change ref sat by large value
        {
            double min = 0.0;
            if (nval[m] < 1) continue;
            for (j = 0;j < nval[m];j++) v[j] = fabs(vval[j * NSYSDD + m]);
            gnss_math_dstd(v, nval[m], &mean, &std);
            min = gnss_math_min(v, nval[m]);

            gnss_MAD_DBL(v, vv, nval[m], &median);
            gnss_math_dstd(vv, nval[m], &mean_dist, &std_dist);

            if ((nval[m] > 2 && ((mean > 10 && std < 10) || (mean > 1.0 && std < 0.5) || (mean > 15 && min > 3.0))) ||
                (((nval[m] + nrej[m]) > 3) && min > 3.0 && mean > 5.0))
            {
                rtk->ssat[i - 1]->usectrl.canberef |= 1;
                flag[m] = 1;
                GLOGI("refsat_check:value too large tor=%.6f nsval=%2d nstotal=%2d refsat=%2d mean=%.3f std=%.3f prdiff=%.3f", 
                    rtk->ptor, nval[m], ntol, i, mean, std, rtk->ssat[i - 1]->usectrl.pr_diff);
            }
            else if (nval[m] > 5 && mean > 3.0 &&mean_dist < 2.0 && peMode.staticData.staticFlag
                     &&fabs(rtk->ssat[i - 1]->usectrl.pr_diff)>3.0 && gpz_kf_data->meas_blk->avgCno >= 40)
            {
                rtk->ssat[i - 1]->usectrl.canberef |= 1;
                flag[m] = 1;
                GLOGI("refsat_check-2:value too large tor=%.6f nsval=%2d nstotal=%2d refsat=%2d mean=%.3f std=%.3f prdiff=%.3f",
                    rtk->ptor, nval[m], ntol, i, mean, std, rtk->ssat[i - 1]->usectrl.pr_diff);

            }
        }
    }

    Sys_Free(v); Sys_Free(vv);

    for (m = 0, t = 0; m < NSYSDD; m++)
        t += flag[m];

    return t;
}

static double gnss_rtk_getdis2d(double *poso, double *posn, double *vctenu)
{
    int i;
    double delta[3] = { 0.0 }, lla[3] = { 0.0 }, enu[3] = { 0.0 }/*, dis = 0.0*/, dis2D = 0.0;
    for (i = 0;i < 3;i++)
    {
        delta[i] = poso[i] - posn[i];
        //dis += delta[i] * delta[i];
    }

    //dis = sqrt(dis);
    ecef2pos(poso, lla);
    ecef2enu(lla, delta, enu);
    dis2D = enu[0] * enu[0] + enu[1] * enu[1];
    dis2D = sqrt(dis2D);

    if(vctenu) ecef2enu(lla, &posn[3], vctenu);  //new enu vel

    return dis2D;
}

/*consider rtk good,return 1, otherwise return 0*/
/* select rtk check -------------------------------------------------------------
*
* args   : none
* return :
* author : none
*-----------------------------------------------------------------------------*/
extern int gnss_rtk_slctrtk_check(rtk_t *rtk, sol_t *solspp)
{
    int i;
    double pospp[3] = { 0.0 };
    double posrtk[3] = { 0.0 };
    float dis2d_rtk2pre = 0.0;
    float dis2d_spp2pre = 0.0;

    if ((rtk->shift.sollist & 0x3) == 3 && rtk->kfcnt > 1)
    {
        if (fabs(rtk->shift.prexyz[0]) < 1.0) return 0;

        for (i = 0;i < 3;i++)
        {
            pospp[i] = solspp->rr[i];
            posrtk[i] = rtk->x[i];
        }
        dis2d_rtk2pre = (float)(gnss_rtk_getdis2d(posrtk, rtk->shift.prexyz, NULL));
        dis2d_spp2pre = (float)(gnss_rtk_getdis2d(pospp, rtk->shift.prexyz, NULL));

        GLOGI("select rtk check, rtk2pre=%.1f rtkvel=%.1f spp2pre=%.1f sppvel=%.1f dt=%.1f",
            dis2d_rtk2pre, rtk->shift.rtkvel[0], dis2d_spp2pre, rtk->shift.sppkfvel[0], rtk->tt);

        if ((fabs(dis2d_rtk2pre - (rtk->shift.rtkvel[0] * rtk->tt)) < 0.5) &&
            (fabs(dis2d_spp2pre - (rtk->shift.sppkfvel[0] * rtk->tt)) > 4.0) &&
            (fabs((double)rtk->shift.rtkvel[0] - rtk->shift.sppkfvel[0]) < 1.5))
        {
            return 1;
        }
    }

    return 0;
}

/*consider spp good, return 1, otherwise return 0*/
/* select spp check -------------------------------------------------------------
*
* args   : none
* return :
* author : none
*-----------------------------------------------------------------------------*/
extern int gnss_rtk_slctspp_check(rtk_t *rtk, sol_t *solspp)
{
    int i;
    double pospp[3] = { 0.0 };
    double posrtk[3] = { 0.0 };
    float dis2d_rtk2pre = 0.0;
    float dis2d_spp2pre = 0.0;

    if (rtk->shift.dis2D[0] > 5.0 || (rtk->tt < 2.0 && ((rtk->qcpasscnt & 0x3F) == 0 || rtk->nrefs > 0 || rtk->sol.qflag != 3)))
    {
        if (fabs(rtk->shift.prexyz[0]) < 1.0) return 0;

        for (i = 0;i < 3;i++)
        {
            pospp[i] = solspp->rr[i];
            posrtk[i] = rtk->x[i];
        }
        dis2d_rtk2pre = (float)(gnss_rtk_getdis2d(posrtk, rtk->shift.prexyz, NULL));
        dis2d_spp2pre = (float)(gnss_rtk_getdis2d(pospp, rtk->shift.prexyz, NULL));

        GLOGI("select spp check, rtk2pre=%.1f rtkvel=%.1f spp2pre=%.1f sppvel=%.1f dt=%.1f",
            dis2d_rtk2pre, rtk->shift.rtkvel[0], dis2d_spp2pre, rtk->shift.sppkfvel[0], rtk->tt);

        if (((fabs(dis2d_rtk2pre - (rtk->shift.rtkvel[0] * rtk->tt)) > 5.0 &&
            fabs(dis2d_spp2pre - (rtk->shift.sppkfvel[0] * rtk->tt)) < 0.5) ||
            (fabs(dis2d_rtk2pre - (rtk->shift.rtkvel[0] * rtk->tt)) > 15.0 &&
                fabs(dis2d_spp2pre - (rtk->shift.sppkfvel[0] * rtk->tt))<3.0)) &&
            fabs((double)rtk->shift.rtkvel[0] - rtk->shift.sppkfvel[0]) < 1.5)
        {
            return 1;
        }
    }

    return 0;
}

/* get shift info -------------------------------------------------------------
*
* args   : none
* return :
* author : none
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_getshiftinfo(rtk_t *rtk, sol_t *solspp,int stat)
{
    int i;
    double poso[6] = { 0.0 }, posn[6] = { 0.0 }, vctenu[3] = { 0.0 };
    double floatkfvel = 0.0, floatheading = 0.0, dis2D = 0.0, sppposrms = 0.0, kfHeading = 0.0, discurpre=0.0;

    if (stat == SOLQ_FLOAT || stat == SOLQ_FIX || stat == SOLQ_DGPS)
    {
        for (i = 0;i < 6;i++)
        {
            poso[i] = solspp->rr[i];
            posn[i] = rtk->x[i];
        }
    }
    else { return; }//have no update sol

    rtk->shift.sollist |= 1;//have info this epoch

    for (i = 0; i < 3; i++) discurpre += (posn[i] - rtk->shift.prexyz[i])*(posn[i] - rtk->shift.prexyz[i]);
    discurpre = sqrt(discurpre);

    for (i = MAX_SOL_BUF - 1; i > 0; i--)
    {
        rtk->shift.discurpre[i] = rtk->shift.discurpre[i - 1];
    }
    rtk->shift.discurpre[0] = (float)discurpre;

    if ((g_proc.flag & 0xFC01) > 0)
    {
        rtk->shift.sppsollist |= 1;//have info this epoch

        //float info
        dis2D = gnss_rtk_getdis2d(poso, posn, vctenu);
        floatkfvel = sqrt(vctenu[0] * vctenu[0] + vctenu[1] * vctenu[1]);
        floatheading = atan2(vctenu[0], vctenu[1])*RAD2DEG;

        for (i = MAX_SOL_BUF - 1; i > 0; i--)
        {
            rtk->shift.spplsvel[i] = rtk->shift.spplsvel[i - 1];
            rtk->shift.sppkfvel[i] = rtk->shift.sppkfvel[i - 1];
            rtk->shift.sppheading[i] = rtk->shift.sppheading[i - 1];
            rtk->shift.sppposrms[i] = rtk->shift.sppposrms[i - 1];
            rtk->shift.rtkvel[i] = rtk->shift.rtkvel[i - 1];
            rtk->shift.rtkheading[i] = rtk->shift.rtkheading[i - 1];
            rtk->shift.dis2D[i] = rtk->shift.dis2D[i - 1];
            //rtk->shift.discurpre[i] = rtk->shift.discurpre[i - 1];
        }
        kfHeading = gpz_kf_data->kf_Pvt_Info->kfHeadingDir * RAD2DEG;
        rtk->shift.spplsvel[0] = peMode.staticData.lsVel;
        rtk->shift.sppkfvel[0] = gpz_kf_data->kf_Pvt_Info->kfHeadingVel;
        rtk->shift.sppheading[0] = (float)kfHeading;// >= 0.0 ? kfHeading : (kfHeading + 360.0);
        rtk->shift.sppposrms[0] = (float)gpz_kf_data->posRes;
        rtk->shift.rtkvel[0] = (float)floatkfvel;
        rtk->shift.rtkheading[0] = (float)floatheading;// >= 0.0 ? floatheading : (floatheading + 360.0);
        rtk->shift.dis2D[0] = (float)dis2D;
        //rtk->shift.discurpre[0] = (float)discurpre;
        rtk->shift.sppkfvsat = gpz_kf_data->kf_ctrl.prNum - gpz_kf_data->kf_ctrl.prRejNum;

        if (dis2D < 5.0)
            rtk->shift.dis2dList1 |= 1;
        else if (dis2D > 15.0)
            rtk->shift.dis2dList2 |= 1;
    }

    //if (MAX_SOL_BUF>=5)
    {
#if defined(_WIN32)
        GLOGI("shift_det: flag=0x%x sollist=0x%I64x kfvnum=%u hisstatic=0x%x", rtk->shift.flag, rtk->shift.sollist, rtk->shift.sppkfvsat, peMode.staticData.historyStatic);
#else
        GLOGI("shift_det: flag=0x%x sollist=0x%llx kfvnum=%u hisstatic=0x%x", rtk->shift.flag, rtk->shift.sollist, rtk->shift.sppkfvsat, peMode.staticData.historyStatic);
#endif
        GLOGI("dis2d   [0]= %8.3f [1] %8.3f [2] %8.3f [3] %8.3f [4] %8.3f", rtk->shift.dis2D[0], rtk->shift.dis2D[1], rtk->shift.dis2D[2], rtk->shift.dis2D[3], rtk->shift.dis2D[4]);
        GLOGI("spprms  [0]= %8.3f [1] %8.3f [2] %8.3f [3] %8.3f [4] %8.3f", rtk->shift.sppposrms[0], rtk->shift.sppposrms[1], rtk->shift.sppposrms[2], rtk->shift.sppposrms[3], rtk->shift.sppposrms[4]);
        GLOGI("spphead [0]= %8.3f [1] %8.3f [2] %8.3f [3] %8.3f [4] %8.3f", rtk->shift.sppheading[0], rtk->shift.sppheading[1], rtk->shift.sppheading[2], rtk->shift.sppheading[3], rtk->shift.sppheading[4]);
        GLOGI("rtkhead [0]= %8.3f [1] %8.3f [2] %8.3f [3] %8.3f [4] %8.3f", rtk->shift.rtkheading[0], rtk->shift.rtkheading[1], rtk->shift.rtkheading[2], rtk->shift.rtkheading[3], rtk->shift.rtkheading[4]);
        GLOGI("sppkfvel[0]= %8.3f [1] %8.3f [2] %8.3f [3] %8.3f [4] %8.3f", rtk->shift.sppkfvel[0], rtk->shift.sppkfvel[1], rtk->shift.sppkfvel[2], rtk->shift.sppkfvel[3], rtk->shift.sppkfvel[4]);
        GLOGI("spplsvel[0]= %8.3f [1] %8.3f [2] %8.3f [3] %8.3f [4] %8.3f", rtk->shift.spplsvel[0], rtk->shift.spplsvel[1], rtk->shift.spplsvel[2], rtk->shift.spplsvel[3], rtk->shift.spplsvel[4]);
        GLOGI("rtkvel  [0]= %8.3f [1] %8.3f [2] %8.3f [3] %8.3f [4] %8.3f", rtk->shift.rtkvel[0], rtk->shift.rtkvel[1], rtk->shift.rtkvel[2], rtk->shift.rtkvel[3], rtk->shift.rtkvel[4]);
        GLOGI("discurp [0]= %8.3f [1] %8.3f [2] %8.3f [3] %8.3f [4] %8.3f", rtk->shift.discurpre[0], rtk->shift.discurpre[1], rtk->shift.discurpre[2], rtk->shift.discurpre[3], rtk->shift.discurpre[4]);
    }
    
}


/* shift check -------------------------------------------------------------
*
* args   : none
* return :
* author : none
*-----------------------------------------------------------------------------*/
extern int gnss_rtk_shift_check(rtk_t *rtk)
{
    int i;
    int syscnt = 0, gnss = 0;
    int stat = 0;//0 float;1 spp;2 spp & reset kf
    shiftdet_t *shift = &rtk->shift;
    ddres_t *pddres = NULL;
    int n = sumofbit1(shift->sppsollist, MAX_SOL_BUF), m = sumofbit1(shift->sollist, MAX_SOL_BUF);
    float avgspprms = 0.0, avgdis2d = 0.0, avgsppheading = 0.0, avgrtkheading = 0.0, avgsppkfvel = 0.0, avgspplsvel = 0.0, avgrtkvel = 0.0;
    float stdspprms = 0.0, stddis2d = 0.0, stdsppheading = 0.0, stdrtkheading = 0.0, stdsppkfvel = 0.0, stdspplsvel = 0.0, stdrtkvel = 0.0;
    float avgheaddiff_rtkspp = 0.0, avgdevdiff_rtkls = 0.0, stdheaddiff_rtkspp = 0.0, stddevdiff_rtkls = 0.0, difdata[MAX_SOL_BUF] = { 0.0 }, difdata2[MAX_SOL_BUF] = { 0.0 };
    float heading0 = 0.0, avgdiscurpre = 0.0, stddiscurpre = 0.0;

    if (n < 1) return 0;

    if ((g_pe_cfg.meas_type_mask & MEAS_TYPE_MASK_DR)==0 && g_pe_cfg.applyScenario == APPLY_SCENE_DRONE)
    {
        return 0;
    }

    for (gnss = GPS_MODE;gnss<GNSS_MAX_MODE;gnss++)
    {
        if (rtk->satcnt[gnss]>0) syscnt++;
    }

    //consider rtk good,return 0
    pddres = &rtk->dd_res[DDRES_UPDATE];
    if ((rtk->kfcnt > 3 && (((rtk->closkycnt & 0xff) == 0x0 && sumofbit1(rtk->qcpasscnt, 8) > 2) || (rtk->qcpasscnt & 0xf) == 0xf)) ||
        (pddres->ncmad > 5 && pddres->npmad > 5 && pddres->stdc < 0.05&&pddres->stdp < 4.0 && rtk->nrefs < 1) ||
        (syscnt == 1 && (rtk->closkycnt & 0xf) == 0x0 && pddres->ncmad > 2 && pddres->npmad > 2 && pddres->stdp < 4.0) ||
        (rtk->nbias > 3 && rtk->sol.ratio > 10.0 && rtk->nrefs < 1) ||
        (rtk->nbias > 5 && rtk->sol.ratio > 3.0))
    {
        return 0;
    }

    gnss_math_fstd(shift->sppposrms, n, &avgspprms, &stdspprms);
    gnss_math_fstd(shift->dis2D, n, &avgdis2d, &stddis2d);
    gnss_math_fstd(shift->sppheading, n, &avgsppheading, &stdsppheading);
    gnss_math_fstd(shift->rtkheading, n, &avgrtkheading, &stdrtkheading);
    gnss_math_fstd(shift->sppkfvel, n, &avgsppkfvel, &stdsppkfvel);
    gnss_math_fstd(shift->spplsvel, n, &avgspplsvel, &stdspplsvel);
    gnss_math_fstd(shift->rtkvel, n, &avgrtkvel, &stdrtkvel);
    gnss_math_fstd(shift->discurpre, m, &avgdiscurpre, &stddiscurpre);

    for (i = 0;i < n;i++)
    {
        difdata[i] = shift->rtkheading[i] - shift->sppheading[i];
        difdata[i] = (float)(fabs(difdata[i]) < 180.0 ? difdata[i] : (shift->rtkheading[i] >= 0.0 ? (360.0 - difdata[i]) : (-difdata[i] - 360.0)));
        difdata2[i] = shift->rtkvel[i] - shift->spplsvel[i];
    }
    gnss_math_fstd(difdata, n, &avgheaddiff_rtkspp, &stdheaddiff_rtkspp);
    gnss_math_fstd(difdata2, n, &avgdevdiff_rtkls, &stddevdiff_rtkls);
    GLOGI("shift check n=%1d avg: spprms=%8.3f dis2d=%8.3f sppheading=%8.3f rtkheading=%8.3f sppkfdev=%8.3f spplsdev=%8.3f rtkdev=%8.3f discp=%8.3f",
        n, avgspprms, avgdis2d, avgsppheading, avgrtkheading, avgsppkfvel, avgspplsvel, avgrtkvel, avgdiscurpre);
    GLOGI("                std: spprms=%8.3f dis2d=%8.3f sppheading=%8.3f rtkheading=%8.3f sppkfdev=%8.3f spplsdev=%8.3f rtkdev=%8.3f discp=%8.3f",
        stdspprms, stddis2d, stdsppheading, stdrtkheading, stdsppkfvel, stdspplsvel, stdrtkvel,stddiscurpre);
    GLOGI("avghead_rtk-spp=%8.3f std=%8.3f avgdev_rtk-ls=%8.3f std=%8.3f dis2List1=0x%x dis2List2=0x%x",
        avgheaddiff_rtkspp, stdheaddiff_rtkspp, avgdevdiff_rtkls, stddevdiff_rtkls, shift->dis2dList1, shift->dis2dList2);

    heading0 = (float)fabs((double)shift->rtkheading[0] - shift->sppheading[0]);
    heading0 = (float)(heading0 < 180.0 ? heading0 : (360.0 - heading0));

    //consider spp bad,return 0
    //if (n > 3 && sumofbit1(peMode.staticData.historyStatic, 8) == 8 && avgspprms > 3.0 && shift->sppposrms[0] > 3.0) return 0;

    if (peMode.staticData.staticFlag || (shift->spplsvel[0] > 0.00001 && shift->spplsvel[0] < 0.1))//static model
    {
        if (rtk->kfcnt < 5 && shift->dis2D[0] > 10.0) stat = 1; //condition to be changed
        if (avgspprms < 12.0&&stdspprms < 5.0&&shift->sppposrms[0]<5.0)  //spp is good
        {
            if (n < 3) //just start rtk
            {
                if (avgdis2d > 4.0&&stddis2d < 3.0) stat = 1;
            }
            else
            {
                if (sumofbit1(shift->dis2dList1, 8) < 3 && shift->dis2D[0] > 5.0) stat = 1;
                if (sumofbit1(shift->dis2dList2, 8) > 4 && shift->dis2D[0] > 5.0) stat = 1;
                if (avgdis2d > 10.0&&stddis2d < 5.0) stat = 1;

                if (avgdis2d > 20.0&&stddis2d < 5.0&&avgspprms < 5.0&&shift->sppheading[0] < 5.0) stat = 2;
            }

        }
        if (((rtk->shift.sollist & 0x3) == 3) && (rtk->shift.discurpre[0] > 2.9)) stat = 1;
        if (m == MAX_SOL_BUF && avgdiscurpre > 1.0 && stddiscurpre < 1.0 && rtk->shift.discurpre[0] > 2.5) stat = 2;
        if (((rtk->shift.sollist & 0x3f) == 0x3f) && avgdiscurpre > 3.0 && rtk->shift.discurpre[0] > 2.5) stat = 2;
    }
    else    //dyn model
    {
        if (shift->sppposrms[0] < 20.0 && shift->dis2D[0] > 25.0 && fabs((double)shift->sppheading[0] - shift->rtkheading[0]) > 30.0 && (rtk->closkycnt&0xF)==0xF) stat = 1; //condition to be changed
        if ((avgspprms < 12.0&&stdspprms < 5.0) || shift->sppposrms[0] < 5.0)
        {
            if (shift->dis2D[0] > 30) stat = 1;
            if (n < 3)  //just start rtk
            {
                if ((shift->dis2D[0] > 10.0&&shift->sppposrms[0] < 8.0) ||
                    (shift->dis2D[0] > 7.0&&shift->sppposrms[0] < 5.0) ||
                    (heading0 > 10.0 && sumofbit1(peMode.staticData.historyStatic, 8) < 3))
                    stat = 1;

                if (shift->dis2D[0] > 30.0&&shift->sppposrms[0] < 1.5) 
                    stat = 2;
            }
            else
            {
                if ((shift->dis2D[0] > 15.0&&shift->sppposrms[0] < 5.0) ||
                    (shift->dis2D[0] > 5.0&&shift->sppposrms[0] < 1.5) ||
                    (shift->dis2D[0] > 8.0&&heading0 > 6.0&&sumofbit1(peMode.staticData.historyStatic, 8) < 3) ||
                    (shift->dis2D[0] > 8.0&&fabs((double)shift->rtkvel[0] - shift->spplsvel[0]) > 5.0) ||
                    (avgdis2d > 15.0&&shift->dis2D[0] > 20.0) ||
                    (avgdis2d > 10.0&&stddis2d < 5.0) ||
                    (heading0 > 30.0&&sumofbit1(peMode.staticData.historyStatic, 8) == 0 && shift->spplsvel[0] > 1.0&&shift->dis2D[0] > 3.0) ||
                    (fabs(avgheaddiff_rtkspp) > 8.0 && stdheaddiff_rtkspp < 5.0 && heading0 > 15.0 && sumofbit1(peMode.staticData.historyStatic, 8) < 3) ||
                    (fabs(avgdevdiff_rtkls) > 5.0&&stddevdiff_rtkls < 3.0))
                    stat = 1;

                if ((avgdis2d > 10.0&&stddis2d<3.0&&shift->dis2D[0]>10.2&&avgspprms < 3.0&&shift->sppposrms[0] < 3.0&&rtk->dd_res[DDRES_UPDATE].cntp < 7) ||
                    /*avgdis2d > 8.0 && shift->dis2D[0] > 10.0&&avgspprms < 3.0&&shift->sppposrms[0] < 5.0 ||*/
                    (avgdis2d > 5.0&&avgspprms < 5.0&&shift->sppposrms[0] < 5.0&&heading0 > 30.0&&sumofbit1(peMode.staticData.historyStatic, 8) < 3 && shift->spplsvel[0]>0.2) ||
                    (avgspplsvel > 5.0&&stdspplsvel < 3.0&&fabs(avgheaddiff_rtkspp) > 15.0&&heading0 > 15.0))
                    stat = 2;
            }
        }
    }
    if (stat == 2)
    {
        rtk->kfstatus = RTK_KF_RESET;
        GLOGI("KF reset at shift check");
    }
    if (stat > 0)
    {
        rtk->shift.flag |= SHIFT_FLAG_NO_FEEDBACK;
    }
    return stat;
}

/* gnss_rtk_amb_out -------------------------------------------------------------
*
* args   : output float or real amb to analysis
* return :
* author : none
*-----------------------------------------------------------------------------*/
extern int gnss_rtk_amb_out(rtk_t *rtk, double v, int f, int rovsat, int refsat, double lam, int ddtype)
{
#if defined(PLAYBACK_MODE)
    if (/*solopt.trace > 0 && */(ddtype == DDRES_UPDATE || ddtype == DDRES_HPREF) && NULL!= rtk->ssat[rovsat - 1])
    {
        double dot;
        int lli;
        dot = v / lam - floor(0.5 + (v / lam));
        lli = (rtk->ssat[rovsat - 1]->slip[f] & 0x3);
        if (ddtype == DDRES_HPREF)
        {
            GLOGI("ref cucl amb(sat=%3d L%d tor= %.3f sat=%3d-%3d v=%13.3f %7.3f lli=%d)",
                rovsat, f + 1, rtk->ptor, refsat, rovsat, v / lam, dot, lli);
        }
        //else if (ddtype == DDRES_UPDATE)
        //{
        //	GLOGI("float cucl amb(sat=%3d L%d tor= %.3f sat=%3d-%3d v=%13.3f %7.3f lli=%d)",
        //		rovsat, f + 1, rtk->ptor, refsat, rovsat, v / lam, dot, lli);
        //}
    }
#endif

    return 0;
}

extern void gnss_rtk_ambp_reset_check(rtk_t *rtk,int ns,int *sat)
{
    int i, f, nf = (rtk->opt.ionoopt == IONOOPT_IFLC) ? 1 : rtk->opt.nf;
    int slip = 0, flag = 0;
    ssat_t *psat=NULL;

    if (rtk->shift.adjPcnt < 100) rtk->shift.adjPcnt++;

    //enlarge ambP for one sat
    for (i = 0; i < ns; i++)
    {
        psat = rtk->ssat[sat[i] - 1];
        if (NULL == psat)
        {
            continue;
        }
        for (f = 0; f < nf; f++)
        {
            flag = 0;
            if ((psat->tdcheck[f] & TD_CHECK_EST) == 0) continue;

            if (fabs(psat->tdres[f]) > 0.1)
            {
                flag = 1;
            }
            else if (rtk->tdsigma < 0.05&&fabs(psat->tdres[f]) > 0.08)
            {
                flag = 1;
            }
            else if (rtk->tdsigma < 0.05&&fabs(psat->tdres[f]) > 0.05&&(psat->snr[f]/4) < 30)
            {
                flag = 1;
            }
            else if (rtk->tdsigma < 0.05&&fabs(psat->tdres[f]) > 0.06 && (psat->slipflag[0][f] & CYCSLIPDET_LLI_HC))
            {
                flag = 1;
            }
            else if (peMode.staticData.staticFlag == 1 && peMode.staticData.lsVel < 0.1&&peMode.staticData.deltaDoplVar < 0.1
                && fabs(psat->tdres[f]) > 0.06&&rtk->tdsigma < 0.04)
            {
                flag = 1;
            }

            if (flag == 1)
            {
                gnss_rtk_init_ambp(rtk, sat[i], f);
                GLOGI("enlarge ambp: tor=%.4f,sat=%d,f=%d,cn0=%d,tdres=%.3f", rtk->ptor, sat[i], f,
                    psat->snr[f]/4, psat->tdres[f]);
            }
        }
    }

    if (rtk->shift.nofixc > 10 && (rtk->qcpasscnt & 0xFFF) == 0xFFF && rtk->nollicnt > 8 && 
        sumofbit1(peMode.staticData.historyStatic, 8) == 0)
    {
        for (i = 0;i < ns;i++)
        {
            for (f=0;f<nf;f++)
            {
                gnss_rtk_init_ambp(rtk,sat[i],f);
            }
        }
        rtk->shift.nofixc = 0;

        GLOGI("amb reset check: tor=%.4f nofixc=%d fixlist=0x%x nollic=%u", rtk->ptor, rtk->shift.nofixc, rtk->shift.fixlist, rtk->nollicnt);
    }else if ((rtk->qcpasscnt & 0x1) && ns > 15 && gpz_kf_data->meas_blk->avgCno > 40 && rtk->shift.nofixc > 1 &&
        rtk->shift.adjPcnt > 0 && rtk->kfcnt > 10)
    {
        for (i = 0;i < ns;i++)
        {
            for (f = 0;f<nf;f++)
            {
                gnss_rtk_init_ambp(rtk, sat[i], f);
            }
        }
        rtk->shift.adjPcnt = -10;

        GLOGI("amb_P reset check: tor=%.4f nofixc=%d fixlist=0x%llx nollic=%u", rtk->ptor, rtk->shift.nofixc, rtk->shift.fixlist, rtk->nollicnt);
    }
}
#if defined(PLAYBACK_MODE)
extern void gnss_rtk_get_refdisinfo(rtk_t *rtk, double *refx, int stat)
{
    int i;
    double dis3dfix = 0.0  , dis2dfix   = 0.0;
    double dis3dfloat = 0.0, dis2dfloat = 0.0;
    wl_t* wl = &rtk->wl;

    if (stat == SOLQ_FIX)
    {
        for (i = 0;i < 3;i++) dis3dfix   += (refx[i] - rtk->xa[i])*(refx[i] - rtk->xa[i]);
        dis3dfix = sqrt(dis3dfix);
        dis2dfix = gnss_rtk_getdis2d(refx, rtk->xa, NULL);
        for (i = 0;i < 3;i++) dis3dfloat += (refx[i] - rtk->x[i] )*(refx[i] - rtk->x[i]);
        dis3dfloat = sqrt(dis3dfloat);
        dis2dfloat = gnss_rtk_getdis2d(refx, rtk->x, NULL);
        GLOGI("refdis_fix: nb=%2d ratio=%5.1f dis2dfix=%.3f dis2dfloat=%.3f dis3dfix=%.3f dis3dfloat=%.3f", 
            rtk->nbias, rtk->sol.ratio, dis2dfix, dis2dfloat,dis3dfix, dis3dfloat);
    }
    else if (stat == SOLQ_FLOAT)
    {
        for (i = 0;i < 3;i++) dis3dfloat += (refx[i] - rtk->x[i])*(refx[i] - rtk->x[i]);
        dis3dfloat = sqrt(dis3dfloat);
        dis2dfloat = gnss_rtk_getdis2d(refx, rtk->x, NULL);
        GLOGI("refdis_float: nsat=%2d dis2dfloat=%.3f,dis3dfloat=%.3f", rtk->dd_res[DDRES_UPDATE].nc, dis2dfloat, dis3dfloat);
    }

    dis3dfix = 0.0;
    dis2dfix = 0.0;
    for (i = 0; i < 3; i++) dis3dfix += (refx[i] - wl->sol[i])*(refx[i] - wl->sol[i]);
    dis3dfix = sqrt(dis3dfix);
    dis2dfix = gnss_rtk_getdis2d(refx, rtk->wl.sol, NULL);
    GLOGI("WL refdis_fix:flag=%d nb=%2d ratio=%5.1f adop=%.3f pdop=%.3f sigma=%.3f dis2dfix=%.3f dis3dfix=%.3f",
        wl->flag & 0x1, wl->nb, wl->ratio, wl->adop, wl->pdop, wl->sigma, dis2dfix, dis3dfix);
}
#endif

extern void gnss_rtk_cfd_init(rtk_t *rtk)
{
    rtk->fat.state = 0;
    rtk->cfd.cfdflag = 0;
    rtk->fat.phasen[0] = rtk->fat.phasen[1] = 0;
    rtk->fat.coden[0] = rtk->fat.coden[1] = 0;
    rtk->fat.satn[0] = rtk->fat.satn[1] = 0;
    rtk->fat.sigmap[0] = rtk->fat.sigmap[1] = 0;
    rtk->fat.sigmac[0] = rtk->fat.sigmac[1] = 0;
    rtk->cfd.horicfd[0] = rtk->cfd.horicfd[1] = rtk->cfd.horicfd[2] = 0;
    rtk->cfd.upcfd[0] = rtk->cfd.upcfd[1] = rtk->cfd.upcfd[2] = 0;
    rtk->fat.rs0 = rtk->fat.rs1 = 0;
    rtk->fat.avelock[0] = rtk->fat.avelock[1] = 0;
    rtk->fat.avecn0[0] = rtk->fat.avecn0[1] = 0;
    rtk->fat.ns = 0;
    rtk->fat.drn = 0;
    rtk->fat.sigmad = 0;
    rtk->fat.ratio = 0;
    rtk->unc.vflag = 0;
    rtk->unc.prefixf = 0;
    rtk->unc.state = 0;
}

/* prefix cfd for dji drone mode-------------------------------------------------------
*
* args   : none
* return :
* author : none
*-------------------------------------------------------------------------------------*/
static void gnss_rtk_arprefix_cfd(rtk_t *rtk, int nbias, double sigma, double *cfdhw, double *cfduw)
{
    const double overallhcfd[3] = { 0.0257,0.0909,1.4953 };
    const double overallucfd[3] = { 0.2444,0.3423,3.1252 };
    memcpy(rtk->cfd.horicfd, overallhcfd, sizeof(rtk->cfd.horicfd));
    memcpy(rtk->cfd.upcfd, overallucfd, sizeof(rtk->cfd.upcfd));
    if ((nbias <= 8 && nbias > 5) && sigma <= 0.01)
    {
        cfdhw[0] = 1.30; cfdhw[1] = 4.93; cfdhw[2] = 1.25;
        cfduw[0] = 1.04; cfduw[1] = 2.53; cfduw[2] = 0.94;
    }
    if ((nbias <= 8 && nbias > 5) && (sigma <= 0.02 && sigma > 0.01))
    {
        cfdhw[0] = 2.90; cfdhw[1] = 11.88; cfdhw[2] = 1.47;
        cfduw[0] = 1.23; cfduw[1] = 5.40; cfduw[2] = 1.66;
    }
    if ((nbias <= 8 && nbias > 5) && (sigma <= 0.03 && sigma > 0.02))
    {
        cfdhw[0] = 27.85; cfdhw[1] = 16.23; cfdhw[2] = 1.72;
        cfduw[0] = 3.16; cfduw[1] = 6.99; cfduw[2] = 1.70;
    }
    if ((nbias <= 8 && nbias > 5) && sigma > 0.03)
    {
        cfdhw[0] = 8.03; cfdhw[1] = 14.32; cfdhw[2] = 1.17;
        cfduw[0] = 2.66; cfduw[1] = 5.06; cfduw[2] = 1.12;
    }
    if ((nbias <= 11 && nbias > 8) && sigma <= 0.01)
    {
        cfdhw[0] = 1.01; cfdhw[1] = 0.48; cfdhw[2] = 0.08;
        cfduw[0] = 0.99; cfduw[1] = 0.83; cfduw[2] = 0.13;
    }
    if ((nbias <= 11 && nbias > 8) && (sigma <= 0.02 && sigma > 0.01))
    {
        cfdhw[0] = 1.27; cfdhw[1] = 3.91; cfdhw[2] = 1.49;
        cfduw[0] = 1.05; cfduw[1] = 4.49; cfduw[2] = 1.66;
    }
    if ((nbias <= 11 && nbias > 8) && (sigma <= 0.03 && sigma > 0.02))
    {
        cfdhw[0] = 17.05; cfdhw[1] = 24.42; cfdhw[2] = 1.50;
        cfduw[0] = 2.91; cfduw[1] = 12.07; cfduw[2] = 1.35;
    }
    if ((nbias <= 11 && nbias > 8) && sigma > 0.03)
    {
        cfdhw[0] = 4.86; cfdhw[1] = 5.17; cfdhw[2] = 0.90;
        cfduw[0] = 1.48; cfduw[1] = 5.02; cfduw[2] = 1.13;
    }
    if (nbias > 12)
    {
        cfdhw[0] = 0.87; cfdhw[1] = 0.70; cfdhw[2] = 0.06;
        cfduw[0] = 0.99; cfduw[1] = 0.83; cfduw[2] = 0.12;
    }
}

/* fix cfd evaluate--------------------------------------------------------------------
*
* args   : none
* return :
* author : none
*-------------------------------------------------------------------------------------*/
static void gnss_rtk_fix_cfd(rtk_t *rtk, int nb, double sigma, double ratio, double *cfdhw, double *cfduw)
{
    const double overallhcfd[3] = { 0.0213,0.2862,2.1530 };
    const double overallucfd[3] = { 0.0640,0.3252,2.5117 };
    memcpy(rtk->cfd.horicfd, overallhcfd, sizeof(rtk->cfd.horicfd));
    memcpy(rtk->cfd.upcfd, overallucfd, sizeof(rtk->cfd.upcfd));
    if (nb <= 5)
    {
        cfdhw[0] = 35.39; cfdhw[1] = 8.72; cfdhw[2] = 1.49;
        cfduw[0] = 12.01; cfduw[1] = 6.69; cfduw[2] = 2.28;
    }
    if (nb == 6 && sigma < 0.005 && ratio < 900.0)
    {
        cfdhw[0] = 3.21; cfdhw[1] = 4.53; cfdhw[2] = 1.11;
        cfduw[0] = 2.14; cfduw[1] = 2.70; cfduw[2] = 0.60;
    }
    if (nb == 6 && sigma >= 0.005 && ratio < 900.0)
    {
        cfdhw[0] = 36.61; cfdhw[1] = 6.89; cfdhw[2] = 1.16;
        cfduw[0] = 12.78; cfduw[1] = 4.81; cfduw[2] = 1.62;
    }
    if (nb == 6 && sigma < 0.004 && ratio >= 900.0)
    {
        cfdhw[0] = 1.32; cfdhw[1] = 1.86; cfdhw[2] = 1.20;
        cfduw[0] = 1.57; cfduw[1] = 1.17; cfduw[2] = 0.70;
    }
    if (nb == 6 && sigma >= 0.004 && ratio >= 900.0)
    {
        cfdhw[0] = 3.73; cfdhw[1] = 5.61; cfdhw[2] = 1.16;
        cfduw[0] = 2.19; cfduw[1] = 3.64; cfduw[2] = 1.24;
    }
    if (nb == 7 && sigma < 0.008)
    {
        cfdhw[0] = 12.11; cfdhw[1] = 3.41; cfdhw[2] = 0.82;
        cfduw[0] = 4.52; cfduw[1] = 4.52; cfduw[2] = 0.87;
    }
    if (nb == 7 && sigma >= 0.008)
    {
        cfdhw[0] = 35.35; cfdhw[1] = 6.88; cfdhw[2] = 1.22;
        cfduw[0] = 8.77; cfduw[1] = 7.73; cfduw[2] = 2.05;
    }
    if (nb == 8 && sigma < 0.008 && ratio < 400.0)
    {
        cfdhw[0] = 1.11; cfdhw[1] = 1.89; cfdhw[2] = 0.76;
        cfduw[0] = 1.57; cfduw[1] = 1.05; cfduw[2] = 0.52;
    }
    if (nb == 8 && sigma >= 0.008 && ratio < 400.0)
    {
        cfdhw[0] = 32.50; cfdhw[1] = 4.61; cfdhw[2] = 1.17;
        cfduw[0] = 5.90; cfduw[1] = 7.73; cfduw[2] = 1.06;
    }
    if (nb == 8 && sigma < 0.006 && ratio >= 400.0)
    {
        cfdhw[0] = 1.11; cfdhw[1] = 0.67; cfdhw[2] = 0.61;
        cfduw[0] = 1.29; cfduw[1] = 0.74; cfduw[2] = 0.60;
    }
    if (nb == 8 && (sigma >= 0.006 && sigma < 0.008) && ratio >= 400.0)
    {
        cfdhw[0] = 1.54; cfdhw[1] = 3.57; cfdhw[2] = 0.77;
        cfduw[0] = 1.29; cfduw[1] = 1.34; cfduw[2] = 0.33;
    }
    if (nb == 8 && sigma >= 0.008 && ratio >= 400.0)
    {
        cfdhw[0] = 2.41; cfdhw[1] = 5.79; cfdhw[2] = 0.83;
        cfduw[0] = 1.58; cfduw[1] = 2.76; cfduw[2] = 1.33;
    }
    if (nb >= 9 && sigma < 0.012)
    {
        cfdhw[0] = 0.83; cfdhw[1] = 0.15; cfdhw[2] = 0.16;
        cfduw[0] = 0.78; cfduw[1] = 0.46; cfduw[2] = 0.26;
    }
    if (nb >= 9 && (sigma >= 0.012 && sigma < 0.015))
    {
        cfdhw[0] = 2.02; cfdhw[1] = 0.79; cfdhw[2] = 0.70;
        cfduw[0] = 2.45; cfduw[1] = 0.81; cfduw[2] = 1.00;
    }
    if (nb >= 9 && sigma >= 0.015)
    {
        cfdhw[0] = 1.75; cfdhw[1] = 2.44; cfdhw[2] = 0.72;
        cfduw[0] = 2.35; cfduw[1] = 2.66; cfduw[2] = 1.00;
    }
}

static void gnss_rtk_prefix_cfd(rtk_t *rtk, int nbias, double sigma, double ratio, double *cfdhw, double *cfduw)
{
    double prefixage = timediff(rtk->sol.time, rtk->prefixt);
    int i;
    if (g_pe_cfg.rinexDataType != RINEX_DATA_TYPE_UM4B0)
    {
        for (i = 0; i < 3; i++)
        {
            rtk->cfd.horicfd[i] = rtk->cfd.phcfd[i] * (1 + prefixage / 180) + 0.2;
            rtk->cfd.upcfd[i] = rtk->cfd.pucfd[i] * (1 + prefixage / 180) + 0.3;
        }
    }
    else
    {
        //gnss_rtk_arprefix_cfd(rtk, nbias, sigma, cfdhw, cfduw);
        for (i = 0; i < 3; i++)
        {
            rtk->cfd.horicfd[i] = rtk->cfd.phcfd[i];
            rtk->cfd.upcfd[i] = rtk->cfd.pucfd[i];
        }
    }
}

/* update pvt navigation_accuracy_t -----------------------------------------------------
*
* args   : none
* return :
* author : none
*-------------------------------------------------------------------------------------*/
extern void gnss_rtk_accu_upd(rtk_t *rtk,USER_PVT* pvt)
{
	/*mapping horizontal cfd to east and north direction factor*/
	double sq2 = sqrt(2);
	if (rtk->cfd.cfdflag==1)
	{
		pvt->accu.accu1_east = (float)(rtk->cfd.horicfd[0] / sq2);
		pvt->accu.accu1_north = (float)(rtk->cfd.horicfd[0] / sq2);
		pvt->accu.accu1_up = (float)rtk->cfd.upcfd[0];
		pvt->accu.accu2_east = (float)(rtk->cfd.horicfd[1] / sq2);
		pvt->accu.accu2_north = (float)(rtk->cfd.horicfd[1] / sq2);
		pvt->accu.accu2_up = (float)rtk->cfd.upcfd[1];
		pvt->accu.accu3_east = (float)(rtk->cfd.horicfd[2] / sq2);
		pvt->accu.accu3_north = (float)(rtk->cfd.horicfd[2] / sq2);
		pvt->accu.accu3_up = (float)rtk->cfd.upcfd[2];
		pvt->accu.accu_type = 0/*fusion_mode_to_accu_indicator(pvt->pos_fusion_mode)*/;
		pvt->accu.valid=TRUE;
	}
}

#if 0
/* rtk solution confidence copy from ubx by xongpan ----------------------------
*
* args   : none
* return :
* author : none
*-----------------------------------------------------------------------------*/
extern void gnss_rtk_confidence(rtk_t *rtk, int state)
{
    /*overall staistic horizontal and up err coffidence*/
    double horicfd_flt[3] = { RTKFLTHERR683,RTKFLTHERR955,RTKFLTHERR997 };
    double upcfd_flt[3] = { RTKFLTUERR683,RTKFLTUERR955,RTKFLTUERR997 };
    const double horicfd_prf[3] = { 0.3527,1.1245,1.4299 };
    const double upcfd_prf[3] = { 0.3296,2.4983,3.7345 };
    double cfdhw[3] = { 1.0,1.0,1.0 };	/*confidence horizontal weight*/
    double cfduw[3] = { 1.0,1.0,1.0 };
    double cfdnoice = 0;
    int i = 0;
    int nbias = 0;
    double ratio = 0.0;
    double sigma = 0.0;
    double prefixage = 0.0;
    double prefixw = 1.0;		/*a prefix age cdf fator 1.0~1.3*/
    int openskyc = 0;
    int closeskyc = 0;
    int semiskyc = 0;
    int llifn = 0;		/*lli free sat num*/
    int soli = 1;
    if (state == SOLQ_NONE || (state == SOLQ_SINGLE && !rtk->prefixf))
        return;
    if (state == SOLQ_FIX) soli = 0;
    else soli = 1;

    prefixage = timediff(rtk->sol.time, rtk->prefixt);
    openskyc = sumofbit1(rtk->qcpasscnt, 16);
    closeskyc = sumofbit1(rtk->closkycnt, 16);
    llifn = (rtk->satcnt[GPS_MODE] + rtk->satcnt[BDS_MODE] - rtk->llisatcnt[GPS_MODE] - rtk->llisatcnt[BDS_MODE]);
    /*if true :semiskyc=actual semisky cnt else semiskyc=0*/
    if (rtk->qcpasscnt + rtk->closkycnt >= 0xffff)
    {
        semiskyc = sumofbit1((0xffff ^ rtk->qcpasscnt) ^ rtk->closkycnt, 16);
    }
    else
    {
        semiskyc = 0;
    }
    /*prefix*/
    if (state != SOLQ_FIX && rtk->prefixf)
    {
        nbias = rtk->prefixn;
        ratio = rtk->prefixr;
        sigma = rtk->prefixsigma0;
            gnss_rtk_prefix_cfd(rtk, nbias, sigma, ratio, cfdhw, cfduw);
    }
    else if (state == SOLQ_FIX)
    {
        nbias = rtk->fat.phasen[soli];
        ratio = rtk->sol.ratio;
        sigma = rtk->fat.sigmap[soli];
        gnss_rtk_fix_cfd(rtk, nbias, sigma, ratio, cfdhw, cfduw);
    }
    else if ((state == SOLQ_FLOAT || state == SOLQ_DGPS) && !(rtk->prefixf))
    {
        nbias = rtk->fat.satn[soli];
        ratio = rtk->sol.ratio;
        sigma = rtk->fat.sigmap[soli];
    }
    /*evaluate float cfd*/
    if ((state == SOLQ_FLOAT || state == SOLQ_DGPS) && !(rtk->prefixf))
    {
        memcpy(rtk->cfd.horicfd, horicfd_flt, sizeof(rtk->cfd.horicfd));
        memcpy(rtk->cfd.upcfd, upcfd_flt, sizeof(rtk->cfd.upcfd));
        if (nbias<5)
        {
            cfdhw[0] = 1.43; cfdhw[1] = 2.52; cfdhw[2] = 0.98;
            cfduw[0] = 0.95; cfduw[1] = 1.41; cfduw[2] = 0.98;
        }
        if ((nbias >= 5 && nbias<10) && sigma<0.01)
        {
            cfdhw[0] = 0.86; cfdhw[1] = 0.87; cfdhw[2] = 0.67;
            cfduw[0] = 0.79; cfduw[1] = 0.85; cfduw[2] = 0.47;
        }
        if ((nbias >= 5 && nbias<10) && (sigma >= 0.01&&sigma<0.02))
        {
            cfdhw[0] = 1.06; cfdhw[1] = 1.28; cfdhw[2] = 0.80;
            cfduw[0] = 1.07; cfduw[1] = 1.20; cfduw[2] = 0.82;
        }
        if ((nbias >= 5 && nbias<10) && sigma >= 0.02)
        {
            cfdhw[0] = 1.43; cfdhw[1] = 2.81; cfdhw[2] = 1.05;
            cfduw[0] = 1.83; cfduw[1] = 1.76; cfduw[2] = 1.40;
        }
        if ((nbias >= 10 && nbias<15) && sigma<0.01)
        {
            cfdhw[0] = 0.83; cfdhw[1] = 0.68; cfdhw[2] = 0.19;
            cfduw[0] = 0.93; cfduw[1] = 0.73; cfduw[2] = 0.31;
        }
        if ((nbias >= 10 && nbias<15) && (sigma >= 0.01&&sigma<0.02))
        {
            cfdhw[0] = 1.13; cfdhw[1] = 0.93; cfdhw[2] = 0.31;
            cfduw[0] = 1.16; cfduw[1] = 0.99; cfduw[2] = 0.37;
        }
        if ((nbias >= 10 && nbias<15) && sigma >= 0.02)
        {
            cfdhw[0] = 1.11; cfdhw[1] = 0.67; cfdhw[2] = 0.39;
            cfduw[0] = 0.96; cfduw[1] = 0.87; cfduw[2] = 0.54;
        }
        if (nbias >= 15)
        {
            cfdhw[0] = 0.64; cfdhw[1] = 0.52; cfdhw[2] = 0.18;
            cfduw[0] = 0.67; cfduw[1] = 0.62; cfduw[2] = 0.24;
        }
    }
    cfdnoice = fmod(sigma, 0.01);
    if (state == SOLQ_FIX || (state != SOLQ_FIX && rtk->prefixf) || state == SOLQ_FLOAT || state == SOLQ_DGPS)
    {
        int prefixflag = 0;
        if (state != SOLQ_FIX && rtk->prefixf) prefixflag = 8;
        for (i = 0; i<3; i++)
        {
            rtk->cfd.horicfd[i] = (rtk->cfd.horicfd[i] + cfdnoice) * cfdhw[i];
            rtk->cfd.upcfd[i] = (rtk->cfd.upcfd[i] + cfdnoice) * cfduw[i];
        }
        if (state == SOLQ_FIX)
        {
            memcpy(rtk->cfd.phcfd, rtk->cfd.horicfd, sizeof(rtk->cfd.phcfd));
            memcpy(rtk->cfd.pucfd, rtk->cfd.upcfd, sizeof(rtk->cfd.pucfd));
        }
        //rtk->cfd.cfdflag = 1;
        GLOGI("confidece: state %d hcdf %.4f %.4f %.4f ucdf %.4f %.4f %.4f sigma %f nb %d", prefixflag != 0? prefixflag : state, rtk->cfd.horicfd[0], rtk->cfd.horicfd[1],
            rtk->cfd.horicfd[2], rtk->cfd.upcfd[0], rtk->cfd.upcfd[1], rtk->cfd.upcfd[2],sigma,nbias);
        gnss_util_trace(3,"confidece: state %d tor %.4f hcdf %.4f %.4f %.4f ucdf %.4f %.4f %.4f\n", prefixflag != 0 ? prefixflag : state,rtk->ptor,rtk->cfd.horicfd[0],rtk->cfd.horicfd[1],
            rtk->cfd.horicfd[2],rtk->cfd.upcfd[0],rtk->cfd.upcfd[1],rtk->cfd.upcfd[2]);
    }
#if defined(PLAYBACK_MODE)
    if (gpz_kf_data->meas_blk&&gpz_kf_data->meas_blk->hpRefQflag[1]>0)
    {
        double dxyz[3] = { -gpz_kf_data->meas_blk->hpRefEcef[0] + rtk->sol.rr[0],-gpz_kf_data->meas_blk->hpRefEcef[1] + rtk->sol.rr[1],-gpz_kf_data->meas_blk->hpRefEcef[2] + rtk->sol.rr[2] };
        double denu[3] = { 0.0 }, pos[3] = { 0.0 };
        double realhorierr = 0.0, realuperr = 0.0;
        ecef2pos(gpz_kf_data->meas_blk->hpRefEcef, pos);
        ecef2enu(pos, dxyz, denu);
        realhorierr = sqrt(denu[0] * denu[0] + denu[1] * denu[1]);
        realuperr = fabs(denu[2]);
        //gnss_util_trace(3, "state %d %10.3f realerr %.4f %.4f sigmap %.4f sigmat %.4f ratio %.4f nb %d"
        //	" hcdf %.4f %.4f %.4f ucdf %.4f %.4f %.4f fage %.3f pfs %.3f ncod %d scene %d %d %d satn %d %d lli %d %d\r\n", state, rtk->ptor,
        //	realhorierr, realuperr, sigma, rtk->fat.sigmac[soli], ratio, nbias, rtk->cfd.horicfd[0], rtk->cfd.horicfd[1]
        //	, rtk->cfd.horicfd[2], rtk->cfd.upcfd[0], rtk->cfd.upcfd[1], rtk->cfd.upcfd[2], prefixage, rtk->prefixsigma0, rtk->fat.coden[soli],
        //	openskyc, closeskyc, semiskyc, rtk->satcnt[GPS_MODE], rtk->satcnt[BDS_MODE], rtk->llisatcnt[GPS_MODE], rtk->llisatcnt[BDS_MODE]);
        if ((state == SOLQ_FIX || state == SOLQ_FLOAT || state == SOLQ_DGPS))
        {
            gnss_rtk_factor(rtk, state, realhorierr, realuperr);
        }
    }
#endif
}
#endif

/* rtk solution unit power sigma -------------------------------------------------------
*
* args   : none
* return :
* author : none
*-------------------------------------------------------------------------------------*/
extern void gnss_rtk_sigma(int ddtype, rtk_t *rtk, const double *v, const double *R,
    const int *vflg, int nv, int ns)
{
    prcopt_t *opt = &rtk->opt;
    double vv = 0.0;
    int i, /*sat1,*/ sat2, type, freq;
    /*a powered v*/
    int npara = rtk->opt.dynamics == 0 ? 3 : 9;
    int vsat[MAXSAT] = { 0 };
    int soli = 1;
    int locksum = 0, lock = 0;
    int coden = 0, phasen = 0;
    double sigmac = 0.0, sigmap = 0.0;
    double avecn0 = 0.0;

    rtk->fat.ns = ns;

    if (ddtype == DDRES_FIXED)
    {
        soli = 0;
    }
    else
    {
        soli = 1;
    }

    for (i = 0; i<nv; i++) {
        //sat1 = (vflg[i] >> 16) & 0xFF;
        sat2 = (vflg[i] >> 8) & 0xFF;
        if (sat2<1 || sat2>MAXSAT)
        {
          continue;
        }
        type = (vflg[i] >> 4) & 0xF;
        freq = vflg[i] & 0xF;
        if (freq >= NFREQ)
        {
          continue;
        }
        vsat[sat2 - 1] = 1;
        if (NULL == rtk->ssat[sat2 - 1])
        {
            continue;
        }

        avecn0 += (double)rtk->ssat[sat2 - 1]->snr[freq] / 4;

        if (type == 1)
        {
            coden++;
            sigmac += v[i] * v[i] / R[i + i * nv];
        }
        else if (type == 0)
        {
            phasen++;
            sigmap += v[i] * v[i] / R[i + i * nv];
            lock = rtk->ssat[sat2 - 1]->lock[freq] + rtk->opt.minlock;
            locksum += lock > 20 ? 20 : lock;
        }
    }

    rtk->fat.avecn0[soli] = nv > 0 ? avecn0 / nv : 0;

    rtk->fat.avelock[soli] = phasen > 0 ? (double)locksum / (phasen) : 0;

    rtk->fat.phasen[soli] = phasen;
    rtk->fat.coden[soli] = coden;

    if (coden > 0)
        rtk->fat.sigmac[soli] = sqrt(sigmac / coden);
    else
        rtk->fat.sigmac[soli] = 0;

    if (phasen > 0)
        rtk->fat.sigmap[soli] = sqrt(sigmap / phasen);
    else
        rtk->fat.sigmap[soli] = 0;

    for (i = 0; i < MAXSAT; i++)
    {
        if (vsat[i]) rtk->fat.satn[soli]++;
    }

    gnss_util_trace(3, "rtks %d %10.3f sigmap %8.3f vv %8.3f sigmac %8.3f satn %d nv %d\n", soli, rtk->ptor, rtk->fat.sigmap[soli], vv, rtk->fat.sigmac[soli], rtk->fat.satn[soli], nv);
    GLOGI("rtks %d %10.3f sigmap %8.3f vv %8.3f sigmac %8.3f satn %d nv %d", soli, rtk->ptor, rtk->fat.sigmap[soli], vv, rtk->fat.sigmac[soli], rtk->fat.satn[soli], nv);
}

/* ambiguity confirm -------------------------------------------------------------
*
* args   : rtk_t
* return : int
* author : none
*-----------------------------------------------------------------------------*/
extern int gnss_rtk_fix2float_check(rtk_t *rtk, int ns, int carrier_num, int fix_carrier_num)
{
    int result = FALSE;
    double adop = 0.0, sigma0 = 0.5 * 0.5, test_value = 0.0, fix_sigma0 = 0.05 * 0.05, fix_test_value = 0.0;
    int gnss = 0;
    int chi_check_passed = FALSE;
    if ((rtk->sol.fixed_sat_num) <= 4 || carrier_num <= 3 || fix_carrier_num <= 3)
    {
        return TRUE;
    }
    if (g_pe_cfg.sub_chipType == SUB_CHIPTYPE_BRCM_47755)
    {
        return result;//disabled temporarily for BRCM
    }

    adop = gnss_rtk_adop_get();
    test_value = rtk->sol.post_sigma / sigma0;
    fix_test_value = rtk->sol.fix_post_sigma / fix_sigma0;
    if (test_value< chisqr[carrier_num-3-1] && fix_test_value < chisqr[fix_carrier_num - 3 - 1])
    {
        chi_check_passed = TRUE;
    }

    if (adop <= 0.0 || rtk->sol.post_sigma <= 0.0 || FALSE == chi_check_passed)
    {
        return TRUE;
    }

    if (rtk->nbias <= 8 && (adop > 10.0 || rtk->nrefs == 2))
    {
        result = TRUE;
    }
    else if (rtk->kfcnt < 50 && rtk->nbias < 7 && adop > 0.5)
    {
        result = TRUE;
    }
    else if (rtk->sol.qflag != 3 && (peMode.staticData.historyStatic & 0xFF) == 0xFF)
    {
        result = TRUE;
    }
    else if (rtk->nbias < 8 && (peMode.staticData.historyStatic & 0xFF) == 0xFF && rtk->dd_res[DDRES_UPDATE].np < ns / 2)
    {
        result = TRUE;
    }

    if (result == FALSE && adop > 0.0)
    {
        if (adop >= 15.0 && rtk->nbias < 10)
        {
            result = TRUE;
        }
        else
        {
            if (rtk->nbias <= 3 && rtk->sol.ratio <= 10.0)
            {
                result = TRUE;
            }
            else if (rtk->nbias <= 5 && (rtk->sol.ratio / adop) < 3.0 && rtk->dd_res[DDRES_FIXED].rmsc > 0.3)
            {
                result = TRUE;
            }
            else if (rtk->nbias <= 8 && rtk->sol.ratio <= 3.0 &&
                (adop > rtk->nbias || rtk->dd_res[DDRES_FIXED].rmsc > 0.2))
            {
                result = TRUE;
            }
            else if ((rtk->sol.ratio / adop) < 1.0 && (adop / rtk->nbias >= 1.0 ||
                (rtk->dd_res[DDRES_FIXED].rmsc > 0.2&&adop > 5.0)))
            {
                result = TRUE;
            }
            else if ((rtk->sol.ratio / adop) < 1.0 && rtk->dd_res[DDRES_FIXED].rmsc > rtk->dd_res[DDRES_UPDATE].rmsc && rtk->dd_res[DDRES_FIXED].rmsc > 0.3)
            {
                result = TRUE;
            }
            else if ((rtk->sol.ratio / adop) < 2.0 && rtk->dd_res[DDRES_FIXED].rmsc > rtk->dd_res[DDRES_UPDATE].rmsc && rtk->dd_res[DDRES_FIXED].rmsc > 0.3)
            {
                result = TRUE;
            }
            else if ((rtk->sol.ratio / adop) < 1.5 && rtk->dd_res[DDRES_FIXED].rmsc > rtk->dd_res[DDRES_UPDATE].rmsc)
            {
                result = TRUE;
            }
            else if (adop > 8.0 && rtk->sol.ratio / adop < 0.8 && rtk->nfix == 0 && rtk->dd_res[DDRES_FIXED].rmsc > 10.0 * rtk->dd_res[DDRES_UPDATE].rmsc)
            {
                result = TRUE;
            }
            else if (adop > 6.0 && rtk->sol.ratio / adop < 0.8 && rtk->nfix == 0 && rtk->comfirmamb && rtk->fix_amb_L1[0] == 0)
            {
                result = TRUE;
            }
            else if (rtk->amb_gap == 1 && adop > 5.0 && rtk->nfix == 0/* && rtk->fix_amb_L1[0] == 0*/)
            {
                result = TRUE;
            }
            else if (rtk->ambfix_flag == Part_LAMBDA_ELECN0 && adop > 5.0 && rtk->comfirmamb)
            {
                result = TRUE;
            }
        }

        if (rtk->nbias <= 5 &&rtk->dd_res[DDRES_FIXED].rmsc > 0.1&&adop > 5.0)
        {
            result = TRUE;
        }
    }

    if (result == FALSE)
    {
        if (rtk->nbias < 4)
        {
            if ((rtk->qcpasscnt & 0xF) == 0 || (rtk->closkycnt & 0x1) == 1)
            {
                if (rtk->nbias == 2)
                    result = TRUE;
                else if (adop > 50.0)
                    result = TRUE;
                else if (adop > 1.0)
                    result = TRUE;
                else if (sumofbit1(rtk->closkycnt, 16) > 8)
                    result = TRUE;
                else if ((int)(rtk->sol.qflag) != 3)
                    result = TRUE;
                else if (rtk->nrefs > 1)
                    result = TRUE;
            }
        }
        else if (rtk->nbias <=13 && (peMode.staticData.historyStatic & 0xFF) != 0)
        {
            if (rtk->rtduseflag != 0 && (rtk->shift.fixlist & 0xFF) != 0xFF && adop > 0.5)
                result = TRUE;
        }
    }

    if (result == TRUE && (rtk->amb_gap == 0 || rtk->nbias >= 11))
    {
        if (rtk->nbias > 8 && rtk->sol.ratio > 3.0&& adop < 2.0 * rtk->nbias && rtk->fix_amb_L1[0]>0 
            && rtk->dd_res[DDRES_FIXED].rmsc < rtk->nbias / 50.0)
        {
            result = FALSE;
        }
        else if (rtk->nbias >= 12 && rtk->sol.ratio > 3.0&&adop < rtk->nbias && rtk->fix_amb_L1[0] > 0)
        {
            result = FALSE;
        }
        else if (rtk->nbias >= 11 && rtk->sol.ratio > 3.0&&adop < 6&&rtk->dd_res[DDRES_FIXED].rmsc<0.4)
        {
            result = FALSE;
        }
        else if (gpz_kf_data->meas_blk->avgCno >= 38 && rtk->nbias >= 10 && rtk->sol.ratio > 3.0 && adop < 2.0 * rtk->nbias &&
            rtk->dd_res[DDRES_FIXED].rmsc < 10.0 * rtk->dd_res[DDRES_UPDATE].rmsc && rtk->fix_amb_L1[0] > 0)
        {
            result = FALSE;
        }
    }

    GLOGI("fix2float check:flag=%d,nb=%d,ns=%d,adop=%.3f,ratio=%.1f,sigma=%.3f,rmsc=%.3f",
        result, rtk->nbias, rtk->sol.ns, adop, rtk->sol.ratio, rtk->sol.post_sigma, rtk->dd_res[DDRES_FIXED].rmsc);

    return result;
}

/* rtk ubx f9 obs reweight --------------------------------------------------------------
*
* args   : none
* return :
* author : none
*-------------------------------------------------------------------------------------*/
#ifdef USED_IN_MC262M
extern double gnss_rtk_ubx_weight(rtk_t *rtk, int sat, int f, int nf, double v)
{
    double factor = 1.0;
    int stdn = 0;
    double cn0 = rtk->ssat[sat - 1]->snr[f%nf] * 0.25;
    int lockms = rtk->ssat[sat - 1]->lockms[0][f%nf];
    int lockc = rtk->ssat[sat - 1]->lock[f%nf] + rtk->opt.minlock;
    //if (lockms == 0 || rtk->ssat[sat - 1].pr_std[f%nf] < 1e-4)
    //	trace(3, "prdebug sat %d f %d v %f prstd %f lockms %d\n", sat, f, v, rtk->ssat[sat - 1].pr_std[f%nf], lockms);
    if (rtk->ssat[sat - 1]->pr_std[f%nf] < 1e-4) return factor;
    if ((f >= nf) && (sumofbit1(rtk->closkycnt, 32) > 2 || sumofbit1(rtk->qcpasscnt, 20) < 15) && v > 1)
    {
        stdn = (int)(log(rtk->ssat[sat - 1]->pr_std[f%nf] / 0.01) / log(2) + 0.5);
        if (stdn <= 9 && stdn > 5) factor = 1*stdn;
        if (stdn <= 12 && stdn > 9) factor = 2*stdn;
        if (stdn >12) factor = 10*stdn;
        if (cn0 <= 35)
        {
            factor *= (36 - cn0) * 2;
            //if (cn0>34) factor *= (36 - cn0) * 5;
            //if (cn0 <= 34 && cn0>32) factor *= (36 - cn0) * 10;
            //if (cn0 <= 32 && cn0>30) factor *= (36 - cn0) * 20;
            //if (cn0 <= 30) factor *= (36 - cn0) * 100;
        }
        if (lockms < 100 && lockms >= 0) factor *= (120 - lockms) / 20;
        //if (lockms == 0) factor *= 10000;
        gnss_util_trace(3, "ubxreweight sat %d f %d v %f factor %f prstd %f stdn %d cn0 %f lockms %d cln %d\n", sat, f, v, factor, rtk->ssat[sat - 1]->pr_std[f%nf], stdn, rtk->ssat[sat - 1]->snr[f%nf] * 0.25, lockms, sumofbit1(rtk->closkycnt, 32));
        //GLOGI("ubxreweight sat %d f %d v %f factor %f prstd %f stdn %d cn0 %f lockms %d cln %d", sat, f, v, factor, rtk->ssat[sat - 1].pr_std[f%nf], stdn, rtk->ssat[sat - 1].snr[f%nf] * 0.25, lockms, sumofbit1(rtk->closkycnt, 32));
    }
    return factor;
}
#endif

#endif
nav_t* gnss_rtk_get_nav(void)
{
    return &rtknav;
}


