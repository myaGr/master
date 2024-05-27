#ifndef __GNSS__BACK__H__
#define __GNSS__BACK__H__

#include "gnss.h"
#include "gnss_tm.h"
#include "gnss_sd_nm.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct  
{
    uint8_t           context;               // context(hand or vehicle) of last start/stop
    uint8_t           kfFixStatus;
    uint32_t          kfRunEpochCnt;         // times of KF run in last start/stop
    double          kfPosRes;
    double          kfPosResStd;
    double          kfVelRes;
    double          kfVelResStd;

    double          preciseAlt;           // accurate altitude during last start/stop, not the altitude of the last epoch
    double          towForAlt;            // TOW for accurate altitude     // 
    uint32_t          weekForAlt;           // week number for accurate altitude
    double          llaPosForAlt[3]; 
} history_Info;

#ifdef __cplusplus
}
#endif


#endif