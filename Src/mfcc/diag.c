/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: diag.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 22-Mar-2018 18:45:34
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "mfcc.h"
#include "diag.h"
#include "mfcc_emxutil.h"

/* Function Definitions */

/*
 * Arguments    : const emxArray_real_T *v
 *                emxArray_real_T *d
 * Return Type  : void
 */
void diag(const emxArray_real_T *v, emxArray_real_T *d)
{
  int unnamed_idx_0;
  int unnamed_idx_1;
  int i2;
  unnamed_idx_0 = v->size[1];
  unnamed_idx_1 = v->size[1];
  i2 = d->size[0] * d->size[1];
  d->size[0] = unnamed_idx_0;
  d->size[1] = unnamed_idx_1;
  emxEnsureCapacity((emxArray__common *)d, i2, (int)sizeof(double));
  unnamed_idx_0 *= unnamed_idx_1;
  for (i2 = 0; i2 < unnamed_idx_0; i2++) {
    d->data[i2] = 0.0;
  }

  for (unnamed_idx_0 = 0; unnamed_idx_0 + 1 <= v->size[1]; unnamed_idx_0++) {
    d->data[unnamed_idx_0 + d->size[0] * unnamed_idx_0] = v->data[unnamed_idx_0];
  }
}

/*
 * File trailer for diag.c
 *
 * [EOF]
 */
