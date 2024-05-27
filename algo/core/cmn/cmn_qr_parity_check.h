/**@file        cmn_qr_parity_check.h
 * @brief       QR parity check
 * @version     V0.1
 **********************************************************************************
 * @note
 * @par History:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/09/22  <td>0.1      <td>chenjinhe   <td>Init version
 * </table>
 *
 **********************************************************************************
 */
#ifndef __CMN_QR_PARITY_CHECK_H__
#define __CMN_QR_PARITY_CHECK_H__
#include "cmn_def.h"
#include "cmn_utils.h"
BEGIN_DECL

typedef uint8_t QR_check_status;
#define QR_CHECK_UNKNOW            (0x0)
#define QR_CHECK_OBS_Insufficient  (0x1)
#define QR_CHECK_IDETIFY_OBS_FAIL  (0x2)
#define QR_CHECK_IDETIFY_OBS_SUCC  (0x3)
#define QR_CHECK_OBS_NORMAL        (0x4)
/**
 * @brief using the method of QR parity check to detect gross error observation according to the post residual
 * @param[in]  d_sigma0 is the prior sigma of chi2-test
 * @param[in]  pz_H is the design matrix
 * @param[in]  pz_P is the weight matrix
 * @param[in]  pz_L is the omc (Observation Minus Calculation)
 * @param[out] pq_grossIndex is the index of gross observation
 * @param[out] pz_deltaX is the delta value of solution
 * @return     the status of QR check
 */
QR_check_status cmn_qrParityCheckAcoordingToPostRes(double d_sigma0, const matrix_t* pz_H, const matrix_t* pz_P,
    const matrix_t* pz_L, int32_t* pq_grossIndex, matrix_t* pz_deltaX);
END_DECL
#endif
