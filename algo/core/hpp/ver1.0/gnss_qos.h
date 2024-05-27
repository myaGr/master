#ifndef __GNSS__QOS__H__
#define __GNSS__QOS__H__

#include "gnss_kf.h"
#include "gnss_pe.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CHECK_REJ_CNT               3

#define MEAS_TYPE_PR                1
#define MEAS_TYPE_DR                2

#define BAD_MEAS_CNT                3

// Define the weight adjust factor
#define ELEV_CHECK                  (1<<1)
#define MP_CHECK                    (1<<2)
#define DIFF_CHECK                  (1<<3)
#define PRDR_CHECK                  (1<<4)
#define TUNNEL_CHECK                (1<<5)
#define CLUSTER_CHECK               (1<<6)
#define MAD_CHECK                   (1<<7)
#define REACQ_CHECK                 (1<<8)
#define PRERR_CHECK                 (1<<9)
#define BADMEAS_CHECK               (1<<10)
#define DUALMEAS_CHECK              (1<<11)

	typedef struct 
	{
		uint8_t isOutlier : 1;
		uint8_t isPRCluster : 1;
		uint8_t isDRCluster : 1;
	}QoS_State_t;
	void gnss_Qos_Main(meas_blk_t* pMeas, Kf_t* p_Kf,PeStateMonitor_t* p_state);
	void gnss_Qos_PRDR_MAD(meas_blk_t* pMeas);
	void gnss_Qos_MP(meas_blk_t* pMeas);
	void gnss_Qos_avgCno(meas_blk_t* pMeas);
	void gnss_Qos_Cno_Mask(meas_blk_t* pMeas);
	void gnss_Qos_DRSmoothPR(gnss_meas_t* pSvMeas);
#ifdef __cplusplus
}
#endif


#endif
