#pragma once
#ifndef RTK_SEQ_EKF_H
#define RTK_SEQ_EKF_H
extern void initH(int state_num);
extern void freeSeqMatrix();
extern void addH(int cur_L, double cur_h);
extern void clearN();
extern void setOMC(double cur_omc, double cur_r);
extern void reset_seq_ekf();
extern void predictStep(double* delta_x, double* Pk);
extern double getInnoRes();
//if the return value is equal to 0,the observation had been rejected by innovation.
extern int measUpdate(double* xk, double* delta_x, double* Pk, double outlier_thres);
#endif