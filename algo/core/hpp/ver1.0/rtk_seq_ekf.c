#include "rtk_seq_ekf.h"
#include <stdlib.h>
#include <string.h>
#include "gnss_sys_api.h"
#include "gnss_rtk.h"

static double* _rtk_seq_h = NULL;
static double* _rtk_seq_hp = NULL;
static double* _rtk_seq_k = NULL;
static int* _rtk_seq_L = NULL;
static int _rtk_seq_para_num = 0;
static int _rtk_seq_n = 0;
static double _rtk_seq_z = 0.0;
static double _rtk_seq_r = 0.0;
static double _rtk_seq_pz = 0.0;
static double _rtk_seq_z_l = 0.0;
static double _rtk_seq_r_l = 0.0;
static double _rtk_seq_res = 0.0;

extern void reset_seq_ekf()
{
    if (_rtk_seq_para_num > 0)
    {
        memset(_rtk_seq_h, 0, sizeof(double) * _rtk_seq_para_num);
        memset(_rtk_seq_hp, 0, sizeof(double) * _rtk_seq_para_num);
        memset(_rtk_seq_k, 0, sizeof(double) * _rtk_seq_para_num);
        memset(_rtk_seq_L, 0, sizeof(int) * _rtk_seq_para_num);
    }
    _rtk_seq_para_num = 0;
    _rtk_seq_n = 0;
    _rtk_seq_z = 0.0;
    _rtk_seq_r = 0.0;
    _rtk_seq_pz = 0.0;
    _rtk_seq_z_l = 0.0;
    _rtk_seq_r_l = 0.0;
    _rtk_seq_res = 0.0;
}
void initH(int state_num)
{
    if (state_num > 0)
    {
        if (state_num > _rtk_seq_para_num)
        {
            Sys_Free(_rtk_seq_h);
            Sys_Free(_rtk_seq_hp);
            Sys_Free(_rtk_seq_k);
            Sys_Free(_rtk_seq_L);
            _rtk_seq_h = (double*)Sys_Malloc(sizeof(double) * state_num);
            _rtk_seq_hp = (double*)Sys_Malloc(sizeof(double) * state_num);
            _rtk_seq_k = (double*)Sys_Malloc(sizeof(double) * state_num);
            _rtk_seq_L = (int*)Sys_Malloc(sizeof(int) * state_num);
        }
        memset(_rtk_seq_h, 0, sizeof(double) * state_num);
        memset(_rtk_seq_hp, 0, sizeof(double) * state_num);
        memset(_rtk_seq_k, 0, sizeof(double) * state_num);
        memset(_rtk_seq_L, 0, sizeof(int) * state_num);
        _rtk_seq_para_num = state_num;
    }
    return;
}
extern void freeSeqMatrix()
{
    Sys_Free(_rtk_seq_h);
    Sys_Free(_rtk_seq_hp);
    Sys_Free(_rtk_seq_k);
    Sys_Free(_rtk_seq_L);
    _rtk_seq_para_num = 0;
    return;
}
extern void addH(int cur_L, double cur_h)
{
    int i = 0;
    if (cur_L >= 0 && cur_L < _rtk_seq_para_num)
    {
        for (i = 0; i < _rtk_seq_n; ++i)
        {
            if (_rtk_seq_L[i] == cur_L)
            {
                break;
            }
        }
        if (i == _rtk_seq_n && _rtk_seq_n < _rtk_seq_para_num)
        {
            _rtk_seq_h[_rtk_seq_n] = cur_h;
            _rtk_seq_L[_rtk_seq_n] = cur_L;
            ++_rtk_seq_n;
        }
    }
}
extern void clearN()
{
    _rtk_seq_n = 0;
    return;
}
extern void setOMC(double cur_omc, double cur_r)
{
    _rtk_seq_z = cur_omc;
    _rtk_seq_r = cur_r;
    return;
}
extern void predictStep(double* delta_x, double* Pk)
{
    int n = _rtk_seq_para_num;
    int m = _rtk_seq_n;
    //H(k)*P(k)*H(k)'
    int j = 0, k = 0;
    for (j = 0; j < n; ++j)
    {
        _rtk_seq_hp[j] = 0.0;
        for (k = 0; k < m; ++k)
        {
            if (j <= _rtk_seq_L[k])
            {
                _rtk_seq_hp[j] += _rtk_seq_h[k] * Pk[_rtk_seq_L[k]* _rtk_seq_para_num+j];
            }
            else
            {
                _rtk_seq_hp[j] += _rtk_seq_h[k] * Pk[j * _rtk_seq_para_num + _rtk_seq_L[k]];
            }
        }
    }
    //Pz=H(k)*P(k)_*H(k)'+R(k)
    _rtk_seq_pz = _rtk_seq_r;
    for (k = 0; k < m; ++k)
    {
        _rtk_seq_pz += _rtk_seq_hp[_rtk_seq_L[k]] * _rtk_seq_h[k];
    }
    //z_=z-Hx
    _rtk_seq_z_l = 0.0;
    for (k = 0; k < m; ++k)
    {
        _rtk_seq_z_l += _rtk_seq_h[k] * delta_x[_rtk_seq_L[k]];
    }
    _rtk_seq_res = _rtk_seq_z - _rtk_seq_z_l;
    _rtk_seq_r_l = (_rtk_seq_res * _rtk_seq_res) / _rtk_seq_pz;
}
extern double getInnoRes()
{
    return _rtk_seq_res;
}
extern int measUpdate(double* xk, double* delta_x, double* Pk, double outlier_thres)
{
    int is_continued = 1, n = _rtk_seq_para_num, i = 0, j = 0;
    double Pz_ = 1.0 / _rtk_seq_pz, innov = 0.0, temp = 0.0;
    if (outlier_thres > 0.0 && _rtk_seq_r_l > (outlier_thres * outlier_thres))
    {
        is_continued = 0;
    }
    if (1 == is_continued)
    {
        //K=P(k)_*H(k)'*inv(Pz)=(H(k)*P(k)_)'*inv(Pz)
        for (i = 0; i < n; ++i)
        {
            _rtk_seq_k[i] = _rtk_seq_hp[i] * Pz_;
        }
        //P(k)=P(k)-K*H(k)*P(k)_
        for (i = 0; i < n; ++i)
        {
            for (j = 0; j <= i; ++j)
            {
                Pk[i* _rtk_seq_para_num+j] -= _rtk_seq_k[i] * _rtk_seq_hp[j];
            }
        }
        innov = _rtk_seq_z - _rtk_seq_z_l;
        //x=x+K*(z-z_)
        for (i = 0; i < n; ++i)
        {
            temp = _rtk_seq_k[i] * innov;
            delta_x[i] += temp;
            xk[i] += temp;
        }
    }
    return is_continued;
}