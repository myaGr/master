/**@file        windup_correction.h
 * @brief       satellite windup correction
 * @details     phase windup effect
 * @author      houxiaowei
 * @date        2022/05/27
 * @version     V0.1
 **********************************************************************************
 * @attention
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/5/27   <td>0.1      <td>houxiaowei  <td>Init version
 * </table>
 *
 **********************************************************************************
 */

#ifndef __WINDUP_CORRECTION__
#define __WINDUP_CORRECTION__

#include "gnss_type.h"
#include "cmn_def.h"
#include "sat_attitude.h"

/**
 * @brief deinit windup correction
 */
extern void wup_deinit();

/**
 * @brief windup correction
 * @param[in] svid
 * @param[in] sys
 * @param[in] yaw
 * @param[in] gpst
 * @param[in] rs
 * @param[in] vs
 * @param[in] rr
 * @param[out] wind_up_corr
 * @return
 */
extern bool wup_getWindUp(const GpsTime_t *gpst, uint8_t svid, uint8_t sys, const double *yaw, const double rs[3],
                   const double vs[3], const double rr[3], double *wind_up_corr);

/**
 * @brief satellite windup
 * @param[in] svid
 * @param[in] sys
 * @param[in] gpst
 * @param[in] yaw
 * @param[in] rs
 * @param[in] vs
 * @param[in] sat2rec_unit
 * @param[out] d_windup_s
 * @return
 */
extern bool wup_SatWindUp(int svid, int sys, const GpsTime_t *gpst, const double *yaw, const double rs[3], const double vs[3],
                   const double sat2rec_unit[3], double d_windup_s[3]);

extern int model_phw(const GpsTime_t* time, int sat, const char *type, int opt,
        const double *rs, const double *rr, double *phw);

/**
 * @brief receiver windup
 * @param[in] sat2rec_unit
 * @param[in] blh
 * @param[out] d_windup_r
 */
extern void wup_ReceiverWindUp(const double sat2rec_unit[3], const double blh[3], double d_windup_r[3]);

#endif // !__WINDUP_CORRECTION__
