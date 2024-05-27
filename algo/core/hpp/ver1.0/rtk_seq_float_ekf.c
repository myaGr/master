#include "rtk_seq_float_ekf.h"
#include <stdlib.h>
#include <string.h>
#include "gnss_sys_api.h"
#include "gnss_rtk.h"

static float* _rtk_seq_h_f = NULL;
static float* _rtk_seq_hp_f = NULL;
static float* _rtk_seq_k_f = NULL;
static int* _rtk_seq_L_f = NULL;
static int _rtk_seq_para_num_f = 0;
static int _rtk_seq_n_f = 0;
static float _rtk_seq_z_f = 0.0;
static float _rtk_seq_r_f = 0.0;
static float _rtk_seq_pz_f = 0.0;
static float _rtk_seq_z_l_f = 0.0;
static float _rtk_seq_r_l_f = 0.0;
static float _rtk_seq_res_f = 0.0;

extern void reset_seq_ekf_f()
{
    if (_rtk_seq_para_num_f > 0)
    {
        memset(_rtk_seq_h_f, 0, sizeof(float) * _rtk_seq_para_num_f);
        memset(_rtk_seq_hp_f, 0, sizeof(float) * _rtk_seq_para_num_f);
        memset(_rtk_seq_k_f, 0, sizeof(float) * _rtk_seq_para_num_f);
        memset(_rtk_seq_L_f, 0, sizeof(int) * _rtk_seq_para_num_f);
    }
    _rtk_seq_para_num_f = 0;
    _rtk_seq_n_f = 0;
    _rtk_seq_z_f = 0.0;
    _rtk_seq_r_f = 0.0;
    _rtk_seq_pz_f = 0.0;
    _rtk_seq_z_l_f = 0.0;
    _rtk_seq_r_l_f = 0.0;
    _rtk_seq_res_f = 0.0;
}
void initH_f(int state_num)
{
    if (state_num > 0)
    {
        if (state_num > _rtk_seq_para_num_f)
        {
            Sys_Free(_rtk_seq_h_f);
            Sys_Free(_rtk_seq_hp_f);
            Sys_Free(_rtk_seq_k_f);
            Sys_Free(_rtk_seq_L_f);
            _rtk_seq_h_f = (float*)Sys_Malloc(sizeof(float) * state_num);
            _rtk_seq_hp_f = (float*)Sys_Malloc(sizeof(float) * state_num);
            _rtk_seq_k_f = (float*)Sys_Malloc(sizeof(float) * state_num);
            _rtk_seq_L_f = (int*)Sys_Malloc(sizeof(int) * state_num);
        }
        memset(_rtk_seq_h_f, 0, sizeof(float) * state_num);
        memset(_rtk_seq_hp_f, 0, sizeof(float) * state_num);
        memset(_rtk_seq_k_f, 0, sizeof(float) * state_num);
        memset(_rtk_seq_L_f, 0, sizeof(int) * state_num);
        _rtk_seq_para_num_f = state_num;
    }
    return;
}
extern void freeSeqMatrix_f()
{
    Sys_Free(_rtk_seq_h_f);
    Sys_Free(_rtk_seq_hp_f);
    Sys_Free(_rtk_seq_k_f);
    Sys_Free(_rtk_seq_L_f);
    _rtk_seq_para_num_f = 0;
    return;
}
extern void addH_f(int cur_L, float cur_h)
{
    int i = 0;
    if (cur_L >= 0 && cur_L < _rtk_seq_para_num_f)
    {
        for (i = 0; i < _rtk_seq_n_f; ++i)
        {
            if (_rtk_seq_L_f[i] == cur_L)
            {
                break;
            }
        }
        if (i == _rtk_seq_n_f && _rtk_seq_n_f < _rtk_seq_para_num_f)
        {
            _rtk_seq_h_f[_rtk_seq_n_f] = cur_h;
            _rtk_seq_L_f[_rtk_seq_n_f] = cur_L;
            ++_rtk_seq_n_f;
        }
    }
}
extern void clearN_f()
{
    _rtk_seq_n_f = 0;
    return;
}
extern void setOMC_f(float cur_omc, float cur_r)
{
    _rtk_seq_z_f = cur_omc;
    _rtk_seq_r_f = cur_r;
    return;
}
extern void predictStep_f(float* delta_x, float* Pk)
{
    int n = _rtk_seq_para_num_f;
    int m = _rtk_seq_n_f;
    //H(k)*P(k)*H(k)'
    int j = 0, k = 0;
    for (j = 0; j < n; ++j)
    {
        _rtk_seq_hp_f[j] = 0.0;
        for (k = 0; k < m; ++k)
        {
            if (j <= _rtk_seq_L_f[k])
            {
                _rtk_seq_hp_f[j] += _rtk_seq_h_f[k] * Pk[_rtk_seq_L_f[k] * _rtk_seq_para_num_f + j];
            }
            else
            {
                _rtk_seq_hp_f[j] += _rtk_seq_h_f[k] * Pk[j * _rtk_seq_para_num_f + _rtk_seq_L_f[k]];
            }
        }
    }
    //Pz=H(k)*P(k)_*H(k)'+R(k)
    _rtk_seq_pz_f = _rtk_seq_r_f;
    for (k = 0; k < m; ++k)
    {
        _rtk_seq_pz_f += _rtk_seq_hp_f[_rtk_seq_L_f[k]] * _rtk_seq_h_f[k];
    }
    //z_=z-Hx
    _rtk_seq_z_l_f = 0.0;
    for (k = 0; k < m; ++k)
    {
        _rtk_seq_z_l_f += _rtk_seq_h_f[k] * delta_x[_rtk_seq_L_f[k]];
    }
    _rtk_seq_res_f = _rtk_seq_z_f - _rtk_seq_z_l_f;
    _rtk_seq_r_l_f = (_rtk_seq_res_f * _rtk_seq_res_f) / _rtk_seq_pz_f;
}
extern float getInnoRes_f()
{
    return _rtk_seq_res_f;
}
extern int measUpdate_f(float* xk, float* delta_x, float* Pk, float outlier_thres)
{
    int is_continued = 1, n = _rtk_seq_para_num_f, i = 0, j = 0;
    float Pz_ = 1.0f / _rtk_seq_pz_f, innov = 0.0, temp = 0.0;
    if (outlier_thres > 0.0 && _rtk_seq_r_l_f > (outlier_thres * outlier_thres))
    {
        is_continued = 0;
    }
    if (1 == is_continued)
    {
        //K=P(k)_*H(k)'*inv(Pz)=(H(k)*P(k)_)'*inv(Pz)
        for (i = 0; i < n; ++i)
        {
            _rtk_seq_k_f[i] = _rtk_seq_hp_f[i] * Pz_;
        }
        //P(k)=P(k)-K*H(k)*P(k)_
        for (i = 0; i < n; ++i)
        {
            for (j = 0; j <= i; ++j)
            {
                Pk[i * _rtk_seq_para_num_f + j] -= _rtk_seq_k_f[i] * _rtk_seq_hp_f[j];
            }
        }
        innov = _rtk_seq_z_f - _rtk_seq_z_l_f;
        //x=x+K*(z-z_)
        for (i = 0; i < n; ++i)
        {
            temp = _rtk_seq_k_f[i] * innov;
            delta_x[i] += temp;
            xk[i] += temp;
        }
    }
    return is_continued;
}