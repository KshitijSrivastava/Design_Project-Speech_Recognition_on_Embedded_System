/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: mfcc.h
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 22-Mar-2018 18:45:34
 */

#ifndef MFCC_H
#define MFCC_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "mfcc_types.h"

/* Function Declarations */
extern void mfcc(double speech[4000], double fs, double Tw, double Ts, double
                 alpha, const double R[2], double M, double N, double L,
                 emxArray_real_T *CC, emxArray_real_T *FBE, emxArray_real_T
                 *frames);

#endif

/*
 * File trailer for mfcc.h
 *
 * [EOF]
 */
