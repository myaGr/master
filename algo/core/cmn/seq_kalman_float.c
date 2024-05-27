#include "seq_kalman_float.h"
#include "gnss_type.h"
/**
 * @brief add the non-zero element of design matrix to the sequential kalman filter
 * @param[in]  w_curL is the paramter index in the kalman filter, there will only push non-zero elements to the filter
 * @param[in]  f_curH is the element in design matrix
 * @param[out] pz_seqVar is the struct used by sequential kalman filter,the type is float
 * @return     void
 */
extern void hpp_seqAddHfloat(uint16_t w_curL, float f_curH, SeqKalmanFloatVar_t* pz_seqVar)
{
    uint16_t w_i = 0;
    if (NULL != pz_seqVar && w_curL < (pz_seqVar->w_paraNum))
    {
        for (w_i = 0; w_i < (pz_seqVar->w_n); ++w_i)
        {
            if ((pz_seqVar->pw_L[w_i]) == w_curL)
            {
                break;
            }
        }
        if (w_i == (pz_seqVar->w_n) && (pz_seqVar->w_n) < (pz_seqVar->w_paraNum))
        {
            pz_seqVar->pf_h[pz_seqVar->w_n] = f_curH;
            pz_seqVar->pw_L[pz_seqVar->w_n] = w_curL;
            ++(pz_seqVar->w_n);
        }
    }
    return;
}

/**
 * @brief clean the variable of sequential kalman filter
 * @param[out] pz_seqVar is the struct used by sequential kalman filter,the type is float
 * @return     void
 */
extern void hpp_seqCleanHfloat(SeqKalmanFloatVar_t* pz_seqVar)
{
    uint16_t w_i = 0;
    if (NULL != pz_seqVar)
    {
        for (w_i = 0; w_i < (pz_seqVar->w_paraNum); ++w_i)
        {
            pz_seqVar->pf_h[w_i] = 0.0;
            pz_seqVar->pw_L[w_i] = 0;
        }
        pz_seqVar->w_n = 0;
        pz_seqVar->f_Z = 0.0;
        pz_seqVar->f_R = 0.0;
        pz_seqVar->f_pZ = 0.0;
        pz_seqVar->f_zL = 0.0;
        pz_seqVar->f_rL = 0.0;
        pz_seqVar->f_res = 0.0;
    }
    return;
}
/**
 * @brief add the observation information(include OMC and variance) to the sequential kalman filter
 * @param[in]  f_curOmc is observation minus calculatiion value
 * @param[in]  f_curR is variance of observation
 * @param[out] pz_seqVar is the struct used by sequential kalman filter,the type is float
 * @return     void
 */
extern void hpp_seqSetOmcFloat(float f_curOmc, float f_curR, SeqKalmanFloatVar_t* pz_seqVar)
{
    if (NULL != pz_seqVar)
    {
        pz_seqVar->f_Z = f_curOmc;
        pz_seqVar->f_R = f_curR;
    }
    return;
}

/**
 * @brief add predict step
 * @param[out] pf_deltaX is delta X relative to parameter init value
 * @param[in]  pf_Pk is covariance of parameter in the filter
 * @param[out] pz_seqVar is the struct used by sequential kalman filter,the type is float
 * @return     void
 */
extern void hpp_seqPredictStepFloat(float* pf_deltaX, float* pf_Pk, SeqKalmanFloatVar_t* pz_seqVar)
{
    uint16_t w_n = pz_seqVar->w_paraNum;
    uint16_t w_m = pz_seqVar->w_n;
    uint16_t w_j = 0;
    uint16_t w_k = 0;
    //H(k)*P(k)*H(k)'
    for (w_j = 0; w_j < w_n; ++w_j)
    {
        pz_seqVar->pf_hP[w_j] = 0.0;
        for (w_k = 0; w_k < w_m; ++w_k)
        {
            pz_seqVar->pf_hP[w_j] += pz_seqVar->pf_h[w_k] * pf_Pk[IUTM(pz_seqVar->pw_L[w_k], w_j)];
        }
    }
    //Pz=H(k)*P(k)_*H(k)'+R(k)
    pz_seqVar->f_pZ = pz_seqVar->f_R;
    for (w_k = 0; w_k < w_m; ++w_k)
    {
        (pz_seqVar->f_pZ) += pz_seqVar->pf_hP[pz_seqVar->pw_L[w_k]] * pz_seqVar->pf_h[w_k];
    }
    //z_=z-Hx
    pz_seqVar->f_zL = 0.0;
    for (w_k = 0; w_k < w_m; ++w_k)
    {
        (pz_seqVar->f_zL) += pz_seqVar->pf_h[w_k] * pf_deltaX[pz_seqVar->pw_L[w_k]];
    }
    pz_seqVar->f_res = pz_seqVar->f_Z - (pz_seqVar->f_zL);
    pz_seqVar->f_rL = ((pz_seqVar->f_res) * (pz_seqVar->f_res)) / (pz_seqVar->f_pZ);
    return;
}
/**
 * @brief using the observation to do measure update the kalman filter
 * @param[out] pf_xk is the value of parameter X in the kalman filter
 * @param[out] pf_deltaX is delta X relative to parameter init value
 * @param[in]  pf_Pk is covariance of parameter in the filter
 * @param[out] pz_seqVar is the struct used by sequential kalman filter,the type is float
 * @param[in]  f_outlierThres is rejected threshold used innovation
 * @return     if the return value is equal to 0,the observation had been rejected by innovation.
 */
extern uint16_t hpp_seqMeasUpdateFloat(float* pf_xk, float* pf_deltaX, float* pf_Pk,
    SeqKalmanFloatVar_t* pz_seqVar, float f_outlierThres)
{
    uint16_t w_isContinued = 1;
    uint16_t w_n = pz_seqVar->w_paraNum;
    uint16_t w_i = 0;
    uint16_t w_j = 0;
    float f_Pz_ = 1.0f / (pz_seqVar->f_pZ);
    float f_innov = 0.0;
    float f_temp = 0.0;
    if (f_outlierThres > 0.0 && (pz_seqVar->f_rL) > (f_outlierThres * f_outlierThres))
    {
        w_isContinued = 0;
    }
    if (1 == w_isContinued)
    {
        //K=P(k)_*H(k)'*inv(Pz)=(H(k)*P(k)_)'*inv(Pz)
        for (w_i = 0; w_i < w_n; ++w_i)
        {
            pz_seqVar->pf_k[w_i] = pz_seqVar->pf_hP[w_i] * f_Pz_;
        }
        //P(k)=P(k)-K*H(k)*P(k)_
        for (w_i = 0; w_i < w_n; ++w_i)
        {
            for (w_j = 0; w_j <= w_i; ++w_j)
            {
                pf_Pk[IUTM(w_i, w_j)] -= pz_seqVar->pf_k[w_i] * pz_seqVar->pf_hP[w_j];
            }
        }
        f_innov = pz_seqVar->f_Z - pz_seqVar->f_zL;
        //x=x+K*(z-z_)
        for (w_i = 0; w_i < w_n; ++w_i)
        {
            f_temp = pz_seqVar->pf_k[w_i] * f_innov;
            pf_deltaX[w_i] += f_temp;
            pf_xk[w_i] += f_temp;
        }
    }
    return w_isContinued;
}