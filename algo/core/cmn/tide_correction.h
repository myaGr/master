/**@file        tide_correction.h
 * @brief       tide correction
 * @details     solid earth tide, ocean tide loading, pole tide
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

#ifndef __TIDE_CORRECTION_H__
#define __TIDE_CORRECTION_H__

#include "gnss_common.h"

typedef struct
{
  double _cosPhi;
  double _sinPhi;
  double _cosTwoPhi;
  double _cosLa;
  double _sinLa;
  double _cosTwoLa;
  double _sinTwoLa;
}LonAndLat_t;


/**
 * @brief Tidal displacement
 * @param[in] gpst
 * @param[in] rr site position (ecef) (m)
 * @param[in] opt options (or of the followings)
 *                1: solid earth tide
 *                2: ocean tide loading
 *                4: pole tide
 *                8: elimate permanent deformation
 * @param[in] erp earth rotation parameters (NULL: not used)
 * @param[in] odisp ocean loading parameters  (NULL: not used)
 *                  odisp[0+i*6]: consituent i amplitude radial(m)
 *                  odisp[1+i*6]: consituent i amplitude west  (m)
 *                  odisp[2+i*6]: consituent i amplitude south (m)
 *                  odisp[3+i*6]: consituent i phase radial  (deg)
 *                  odisp[4+i*6]: consituent i phase west    (deg)
 *                  odisp[5+i*6]: consituent i phase south   (deg)
 *                  (i=0:M2,1:S2,2:N2,3:K2,4:K1,5:O1,6:P1,7:Q1,
 *                                   8:Mf,9:Mm,10:Ssa)
 * @param[out] dr displacement by earth tides (ecef) (m)
 * @return none
 * notes  : see ref [1], [2] chap 7
 *          see ref [4] 5.2.1, 5.2.2, 5.2.3
 *          ver.2.4.0 does not use ocean loading and pole tide corrections
 */
void tide_Displacement(const GpsTime_t* gpst, const double* rr, gnss_ssrErrorModelMask opt,
  const gnss_Erp_t* erp, const double* odisp, float* dr);


/**
 * @brief Solid tide
 * @param[in] gpst
 * @param[in] recpos
 * @param[in] xsun
 * @param[in] xmoon
 * @param[out] corr
 * @return true or false
 */
bool tide_getSolidTide(const GpsTime_t* gpst, const double recpos[3], const double xsun[3],
  const double xmoon[3], double corr[3]);


/* option correction, from rtklib */

/**
 * @brief get earth rotation parameter values
 * @param[in] erp earth rotation parameters
 * @param[in] gpst
 * @param[out] erpv erp values {xp,yp,ut1_utc,lod} (rad,rad,s,s/d)
 * @return status (1:ok,0:error)
 */
BOOL tide_GetErp(const gnss_Erp_t* erp, const GpsTime_t* gpst, double* erpv);


void tide_SolarLunar(const double* eu, const double* rp, double GMp, const double* pos, double* dr);

void tide_Solid(const double* rsun, const double* rmoon, const double* pos, const double* E, double gmst, int opt, double* dr);

void tide_Oceanload(const UtcTime_t* tut, const double* odisp, double* denu);

void tide_Pole(const UtcTime_t* tut, const double* pos, const double* erpv, double* denu);

#endif // __TIDE_CORRECTION_H__