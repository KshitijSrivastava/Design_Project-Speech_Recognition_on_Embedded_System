/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_mfcc_api.h
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 22-Mar-2018 18:45:34
 */

#ifndef _CODER_MFCC_API_H
#define _CODER_MFCC_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_mfcc_api.h"

/* Type Definitions */
#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_real_T*/

#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T

typedef struct emxArray_real_T emxArray_real_T;

#endif                                 /*typedef_emxArray_real_T*/

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void mfcc(real_T speech[4000], real_T fs, real_T Tw, real_T Ts, real_T
                 alpha, real_T R[2], real_T M, real_T N, real_T L,
                 emxArray_real_T *CC, emxArray_real_T *FBE, emxArray_real_T
                 *frames);
extern void mfcc_api(const mxArray *prhs[9], const mxArray *plhs[3]);
extern void mfcc_atexit(void);
extern void mfcc_initialize(void);
extern void mfcc_terminate(void);
extern void mfcc_xil_terminate(void);

#endif

/*
 * File trailer for _coder_mfcc_api.h
 *
 * [EOF]
 */
