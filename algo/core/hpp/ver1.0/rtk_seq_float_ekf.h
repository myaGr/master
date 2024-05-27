#pragma once
#ifndef RTK_SEQ_FLOAT_EKF_H
#define RTK_SEQ_FLOAT_EKF_H
extern void initH_f(int state_num);
extern void freeSeqMatrix_f();
extern void addH_f(int cur_L, float cur_h);
extern void clearN_f();
extern void setOMC_f(float cur_omc, float cur_r);
extern void reset_seq_ekf_f();
extern void predictStep_f(float* delta_x, float* Pk);
extern float getInnoRes_f();
//if the return value is equal to 0,the observation had been rejected by innovation.
extern int measUpdate_f(float* xk, float* delta_x, float* Pk, float outlier_thres);
#endif