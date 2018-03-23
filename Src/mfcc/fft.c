/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: fft.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 22-Mar-2018 18:45:34
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "mfcc.h"
#include "fft.h"
#include "mfcc_emxutil.h"

/* Function Declarations */
static void bluestein(const emxArray_real_T *x, int xoffInit, int nfft, int
                      nRows, const emxArray_real_T *costab, const
                      emxArray_real_T *sintab, const emxArray_real_T *costabinv,
                      const emxArray_real_T *sintabinv, const emxArray_creal_T
                      *wwc, emxArray_creal_T *y);
static void generate_twiddle_tables(int nRows, boolean_T useRadix2,
  emxArray_real_T *costab, emxArray_real_T *sintab, emxArray_real_T *sintabinv);
static void r2br_r2dit_trig_impl(const emxArray_creal_T *x, int unsigned_nRows,
  const emxArray_real_T *costab, const emxArray_real_T *sintab, emxArray_creal_T
  *y);

/* Function Definitions */

/*
 * Arguments    : const emxArray_real_T *x
 *                int xoffInit
 *                int nfft
 *                int nRows
 *                const emxArray_real_T *costab
 *                const emxArray_real_T *sintab
 *                const emxArray_real_T *costabinv
 *                const emxArray_real_T *sintabinv
 *                const emxArray_creal_T *wwc
 *                emxArray_creal_T *y
 * Return Type  : void
 */
static void bluestein(const emxArray_real_T *x, int xoffInit, int nfft, int
                      nRows, const emxArray_real_T *costab, const
                      emxArray_real_T *sintab, const emxArray_real_T *costabinv,
                      const emxArray_real_T *sintabinv, const emxArray_creal_T
                      *wwc, emxArray_creal_T *y)
{
  int minNrowsNx;
  int ix;
  int xidx;
  double r;
  double twid_im;
  emxArray_creal_T *fy;
  int istart;
  emxArray_creal_T *fv;
  int nRowsD2;
  int nRowsD4;
  int i;
  boolean_T tst;
  double temp_re;
  double temp_im;
  int j;
  double fv_re;
  double fv_im;
  int ihi;
  double wwc_im;
  double b_fv_re;
  if (nRows <= x->size[0]) {
    minNrowsNx = nRows;
  } else {
    minNrowsNx = x->size[0];
  }

  ix = y->size[0];
  y->size[0] = nRows;
  emxEnsureCapacity((emxArray__common *)y, ix, (int)sizeof(creal_T));
  xidx = x->size[0];
  if (nRows > xidx) {
    xidx = y->size[0];
    ix = y->size[0];
    y->size[0] = xidx;
    emxEnsureCapacity((emxArray__common *)y, ix, (int)sizeof(creal_T));
    for (ix = 0; ix < xidx; ix++) {
      y->data[ix].re = 0.0;
      y->data[ix].im = 0.0;
    }
  }

  xidx = xoffInit;
  for (ix = 0; ix + 1 <= minNrowsNx; ix++) {
    r = wwc->data[(nRows + ix) - 1].re;
    twid_im = wwc->data[(nRows + ix) - 1].im;
    y->data[ix].re = r * x->data[xidx];
    y->data[ix].im = twid_im * -x->data[xidx];
    xidx++;
  }

  while (minNrowsNx + 1 <= nRows) {
    y->data[minNrowsNx].re = 0.0;
    y->data[minNrowsNx].im = 0.0;
    minNrowsNx++;
  }

  emxInit_creal_T1(&fy, 1);
  r2br_r2dit_trig_impl(y, nfft, costab, sintab, fy);
  if (wwc->size[0] <= nfft) {
    istart = wwc->size[0];
  } else {
    istart = nfft;
  }

  emxInit_creal_T1(&fv, 1);
  nRowsD2 = nfft / 2;
  nRowsD4 = nRowsD2 / 2;
  ix = fv->size[0];
  fv->size[0] = nfft;
  emxEnsureCapacity((emxArray__common *)fv, ix, (int)sizeof(creal_T));
  if (nfft > wwc->size[0]) {
    xidx = fv->size[0];
    ix = fv->size[0];
    fv->size[0] = xidx;
    emxEnsureCapacity((emxArray__common *)fv, ix, (int)sizeof(creal_T));
    for (ix = 0; ix < xidx; ix++) {
      fv->data[ix].re = 0.0;
      fv->data[ix].im = 0.0;
    }
  }

  ix = 0;
  minNrowsNx = 0;
  xidx = 0;
  for (i = 1; i < istart; i++) {
    fv->data[xidx] = wwc->data[ix];
    xidx = nfft;
    tst = true;
    while (tst) {
      xidx >>= 1;
      minNrowsNx ^= xidx;
      tst = ((minNrowsNx & xidx) == 0);
    }

    xidx = minNrowsNx;
    ix++;
  }

  fv->data[xidx] = wwc->data[ix];
  if (nfft > 1) {
    for (i = 0; i <= nfft - 2; i += 2) {
      temp_re = fv->data[i + 1].re;
      temp_im = fv->data[i + 1].im;
      fv->data[i + 1].re = fv->data[i].re - fv->data[i + 1].re;
      fv->data[i + 1].im = fv->data[i].im - fv->data[i + 1].im;
      fv->data[i].re += temp_re;
      fv->data[i].im += temp_im;
    }
  }

  xidx = 2;
  minNrowsNx = 4;
  ix = 1 + ((nRowsD4 - 1) << 2);
  while (nRowsD4 > 0) {
    for (i = 0; i < ix; i += minNrowsNx) {
      temp_re = fv->data[i + xidx].re;
      temp_im = fv->data[i + xidx].im;
      fv->data[i + xidx].re = fv->data[i].re - temp_re;
      fv->data[i + xidx].im = fv->data[i].im - temp_im;
      fv->data[i].re += temp_re;
      fv->data[i].im += temp_im;
    }

    istart = 1;
    for (j = nRowsD4; j < nRowsD2; j += nRowsD4) {
      r = costab->data[j];
      twid_im = sintab->data[j];
      i = istart;
      ihi = istart + ix;
      while (i < ihi) {
        temp_re = r * fv->data[i + xidx].re - twid_im * fv->data[i + xidx].im;
        temp_im = r * fv->data[i + xidx].im + twid_im * fv->data[i + xidx].re;
        fv->data[i + xidx].re = fv->data[i].re - temp_re;
        fv->data[i + xidx].im = fv->data[i].im - temp_im;
        fv->data[i].re += temp_re;
        fv->data[i].im += temp_im;
        i += minNrowsNx;
      }

      istart++;
    }

    nRowsD4 /= 2;
    xidx = minNrowsNx;
    minNrowsNx <<= 1;
    ix -= xidx;
  }

  ix = fy->size[0];
  emxEnsureCapacity((emxArray__common *)fy, ix, (int)sizeof(creal_T));
  xidx = fy->size[0];
  for (ix = 0; ix < xidx; ix++) {
    r = fy->data[ix].re;
    twid_im = fy->data[ix].im;
    fv_re = fv->data[ix].re;
    fv_im = fv->data[ix].im;
    fy->data[ix].re = r * fv_re - twid_im * fv_im;
    fy->data[ix].im = r * fv_im + twid_im * fv_re;
  }

  if (fy->size[0] <= nfft) {
    istart = fy->size[0];
  } else {
    istart = nfft;
  }

  nRowsD2 = nfft / 2;
  nRowsD4 = nRowsD2 / 2;
  ix = fv->size[0];
  fv->size[0] = nfft;
  emxEnsureCapacity((emxArray__common *)fv, ix, (int)sizeof(creal_T));
  if (nfft > fy->size[0]) {
    xidx = fv->size[0];
    ix = fv->size[0];
    fv->size[0] = xidx;
    emxEnsureCapacity((emxArray__common *)fv, ix, (int)sizeof(creal_T));
    for (ix = 0; ix < xidx; ix++) {
      fv->data[ix].re = 0.0;
      fv->data[ix].im = 0.0;
    }
  }

  ix = 0;
  minNrowsNx = 0;
  xidx = 0;
  for (i = 1; i < istart; i++) {
    fv->data[xidx] = fy->data[ix];
    xidx = nfft;
    tst = true;
    while (tst) {
      xidx >>= 1;
      minNrowsNx ^= xidx;
      tst = ((minNrowsNx & xidx) == 0);
    }

    xidx = minNrowsNx;
    ix++;
  }

  fv->data[xidx] = fy->data[ix];
  emxFree_creal_T(&fy);
  if (nfft > 1) {
    for (i = 0; i <= nfft - 2; i += 2) {
      temp_re = fv->data[i + 1].re;
      temp_im = fv->data[i + 1].im;
      fv->data[i + 1].re = fv->data[i].re - fv->data[i + 1].re;
      fv->data[i + 1].im = fv->data[i].im - fv->data[i + 1].im;
      fv->data[i].re += temp_re;
      fv->data[i].im += temp_im;
    }
  }

  xidx = 2;
  minNrowsNx = 4;
  ix = 1 + ((nRowsD4 - 1) << 2);
  while (nRowsD4 > 0) {
    for (i = 0; i < ix; i += minNrowsNx) {
      temp_re = fv->data[i + xidx].re;
      temp_im = fv->data[i + xidx].im;
      fv->data[i + xidx].re = fv->data[i].re - temp_re;
      fv->data[i + xidx].im = fv->data[i].im - temp_im;
      fv->data[i].re += temp_re;
      fv->data[i].im += temp_im;
    }

    istart = 1;
    for (j = nRowsD4; j < nRowsD2; j += nRowsD4) {
      r = costabinv->data[j];
      twid_im = sintabinv->data[j];
      i = istart;
      ihi = istart + ix;
      while (i < ihi) {
        temp_re = r * fv->data[i + xidx].re - twid_im * fv->data[i + xidx].im;
        temp_im = r * fv->data[i + xidx].im + twid_im * fv->data[i + xidx].re;
        fv->data[i + xidx].re = fv->data[i].re - temp_re;
        fv->data[i + xidx].im = fv->data[i].im - temp_im;
        fv->data[i].re += temp_re;
        fv->data[i].im += temp_im;
        i += minNrowsNx;
      }

      istart++;
    }

    nRowsD4 /= 2;
    xidx = minNrowsNx;
    minNrowsNx <<= 1;
    ix -= xidx;
  }

  if (fv->size[0] > 1) {
    r = 1.0 / (double)fv->size[0];
    ix = fv->size[0];
    emxEnsureCapacity((emxArray__common *)fv, ix, (int)sizeof(creal_T));
    xidx = fv->size[0];
    for (ix = 0; ix < xidx; ix++) {
      fv->data[ix].re *= r;
      fv->data[ix].im *= r;
    }
  }

  xidx = 0;
  for (ix = nRows - 1; ix + 1 <= wwc->size[0]; ix++) {
    r = wwc->data[ix].re;
    fv_re = fv->data[ix].re;
    twid_im = wwc->data[ix].im;
    fv_im = fv->data[ix].im;
    temp_re = wwc->data[ix].re;
    temp_im = fv->data[ix].im;
    wwc_im = wwc->data[ix].im;
    b_fv_re = fv->data[ix].re;
    y->data[xidx].re = r * fv_re + twid_im * fv_im;
    y->data[xidx].im = temp_re * temp_im - wwc_im * b_fv_re;
    xidx++;
  }

  emxFree_creal_T(&fv);
}

/*
 * Arguments    : int nRows
 *                boolean_T useRadix2
 *                emxArray_real_T *costab
 *                emxArray_real_T *sintab
 *                emxArray_real_T *sintabinv
 * Return Type  : void
 */
static void generate_twiddle_tables(int nRows, boolean_T useRadix2,
  emxArray_real_T *costab, emxArray_real_T *sintab, emxArray_real_T *sintabinv)
{
  emxArray_real_T *costab1q;
  double e;
  int nRowsD4;
  int nd2;
  int k;
  int n2;
  emxInit_real_T(&costab1q, 2);
  e = 6.2831853071795862 / (double)nRows;
  nRowsD4 = nRows / 2 / 2;
  nd2 = costab1q->size[0] * costab1q->size[1];
  costab1q->size[0] = 1;
  costab1q->size[1] = nRowsD4 + 1;
  emxEnsureCapacity((emxArray__common *)costab1q, nd2, (int)sizeof(double));
  costab1q->data[0] = 1.0;
  nd2 = nRowsD4 / 2;
  for (k = 1; k <= nd2; k++) {
    costab1q->data[k] = cos(e * (double)k);
  }

  for (k = nd2 + 1; k < nRowsD4; k++) {
    costab1q->data[k] = sin(e * (double)(nRowsD4 - k));
  }

  costab1q->data[nRowsD4] = 0.0;
  if (!useRadix2) {
    nRowsD4 = costab1q->size[1] - 1;
    n2 = (costab1q->size[1] - 1) << 1;
    nd2 = costab->size[0] * costab->size[1];
    costab->size[0] = 1;
    costab->size[1] = n2 + 1;
    emxEnsureCapacity((emxArray__common *)costab, nd2, (int)sizeof(double));
    nd2 = sintab->size[0] * sintab->size[1];
    sintab->size[0] = 1;
    sintab->size[1] = n2 + 1;
    emxEnsureCapacity((emxArray__common *)sintab, nd2, (int)sizeof(double));
    costab->data[0] = 1.0;
    sintab->data[0] = 0.0;
    nd2 = sintabinv->size[0] * sintabinv->size[1];
    sintabinv->size[0] = 1;
    sintabinv->size[1] = n2 + 1;
    emxEnsureCapacity((emxArray__common *)sintabinv, nd2, (int)sizeof(double));
    for (k = 1; k <= nRowsD4; k++) {
      sintabinv->data[k] = costab1q->data[nRowsD4 - k];
    }

    for (k = costab1q->size[1]; k <= n2; k++) {
      sintabinv->data[k] = costab1q->data[k - nRowsD4];
    }

    for (k = 1; k <= nRowsD4; k++) {
      costab->data[k] = costab1q->data[k];
      sintab->data[k] = -costab1q->data[nRowsD4 - k];
    }

    for (k = costab1q->size[1]; k <= n2; k++) {
      costab->data[k] = -costab1q->data[n2 - k];
      sintab->data[k] = -costab1q->data[k - nRowsD4];
    }
  } else {
    nRowsD4 = costab1q->size[1] - 1;
    n2 = (costab1q->size[1] - 1) << 1;
    nd2 = costab->size[0] * costab->size[1];
    costab->size[0] = 1;
    costab->size[1] = n2 + 1;
    emxEnsureCapacity((emxArray__common *)costab, nd2, (int)sizeof(double));
    nd2 = sintab->size[0] * sintab->size[1];
    sintab->size[0] = 1;
    sintab->size[1] = n2 + 1;
    emxEnsureCapacity((emxArray__common *)sintab, nd2, (int)sizeof(double));
    costab->data[0] = 1.0;
    sintab->data[0] = 0.0;
    for (k = 1; k <= nRowsD4; k++) {
      costab->data[k] = costab1q->data[k];
      sintab->data[k] = -costab1q->data[nRowsD4 - k];
    }

    for (k = costab1q->size[1]; k <= n2; k++) {
      costab->data[k] = -costab1q->data[n2 - k];
      sintab->data[k] = -costab1q->data[k - nRowsD4];
    }

    nd2 = sintabinv->size[0] * sintabinv->size[1];
    sintabinv->size[0] = 1;
    sintabinv->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)sintabinv, nd2, (int)sizeof(double));
  }

  emxFree_real_T(&costab1q);
}

/*
 * Arguments    : const emxArray_creal_T *x
 *                int unsigned_nRows
 *                const emxArray_real_T *costab
 *                const emxArray_real_T *sintab
 *                emxArray_creal_T *y
 * Return Type  : void
 */
static void r2br_r2dit_trig_impl(const emxArray_creal_T *x, int unsigned_nRows,
  const emxArray_real_T *costab, const emxArray_real_T *sintab, emxArray_creal_T
  *y)
{
  int j;
  int nRowsD2;
  int nRowsD4;
  int iy;
  int iDelta;
  int ix;
  int ju;
  int i;
  boolean_T tst;
  double temp_re;
  double temp_im;
  double twid_re;
  double twid_im;
  int ihi;
  if (x->size[0] <= unsigned_nRows) {
    j = x->size[0];
  } else {
    j = unsigned_nRows;
  }

  nRowsD2 = unsigned_nRows / 2;
  nRowsD4 = nRowsD2 / 2;
  iy = y->size[0];
  y->size[0] = unsigned_nRows;
  emxEnsureCapacity((emxArray__common *)y, iy, (int)sizeof(creal_T));
  iy = x->size[0];
  if (unsigned_nRows > iy) {
    iDelta = y->size[0];
    iy = y->size[0];
    y->size[0] = iDelta;
    emxEnsureCapacity((emxArray__common *)y, iy, (int)sizeof(creal_T));
    for (iy = 0; iy < iDelta; iy++) {
      y->data[iy].re = 0.0;
      y->data[iy].im = 0.0;
    }
  }

  ix = 0;
  ju = 0;
  iy = 0;
  for (i = 1; i < j; i++) {
    y->data[iy] = x->data[ix];
    iDelta = unsigned_nRows;
    tst = true;
    while (tst) {
      iDelta >>= 1;
      ju ^= iDelta;
      tst = ((ju & iDelta) == 0);
    }

    iy = ju;
    ix++;
  }

  y->data[iy] = x->data[ix];
  if (unsigned_nRows > 1) {
    for (i = 0; i <= unsigned_nRows - 2; i += 2) {
      temp_re = y->data[i + 1].re;
      temp_im = y->data[i + 1].im;
      y->data[i + 1].re = y->data[i].re - y->data[i + 1].re;
      y->data[i + 1].im = y->data[i].im - y->data[i + 1].im;
      y->data[i].re += temp_re;
      y->data[i].im += temp_im;
    }
  }

  iDelta = 2;
  iy = 4;
  ix = 1 + ((nRowsD4 - 1) << 2);
  while (nRowsD4 > 0) {
    for (i = 0; i < ix; i += iy) {
      temp_re = y->data[i + iDelta].re;
      temp_im = y->data[i + iDelta].im;
      y->data[i + iDelta].re = y->data[i].re - temp_re;
      y->data[i + iDelta].im = y->data[i].im - temp_im;
      y->data[i].re += temp_re;
      y->data[i].im += temp_im;
    }

    ju = 1;
    for (j = nRowsD4; j < nRowsD2; j += nRowsD4) {
      twid_re = costab->data[j];
      twid_im = sintab->data[j];
      i = ju;
      ihi = ju + ix;
      while (i < ihi) {
        temp_re = twid_re * y->data[i + iDelta].re - twid_im * y->data[i +
          iDelta].im;
        temp_im = twid_re * y->data[i + iDelta].im + twid_im * y->data[i +
          iDelta].re;
        y->data[i + iDelta].re = y->data[i].re - temp_re;
        y->data[i + iDelta].im = y->data[i].im - temp_im;
        y->data[i].re += temp_re;
        y->data[i].im += temp_im;
        i += iy;
      }

      ju++;
    }

    nRowsD4 /= 2;
    iDelta = iy;
    iy <<= 1;
    ix -= iDelta;
  }
}

/*
 * Arguments    : const emxArray_real_T *x
 *                double varargin_1
 *                emxArray_creal_T *y
 * Return Type  : void
 */
void fft(const emxArray_real_T *x, double varargin_1, emxArray_creal_T *y)
{
  emxArray_real_T *costab;
  emxArray_real_T *sintab;
  emxArray_real_T *sintabinv;
  emxArray_creal_T *wwc;
  emxArray_creal_T *rwork;
  boolean_T useRadix2;
  int sz[2];
  int ihi;
  int istart;
  int nn1m1;
  int pmax;
  int pmin;
  unsigned int sx[2];
  boolean_T exitg1;
  int pow2p;
  int k;
  int nRowsD2;
  double nt_im;
  int nRowsD4;
  double nt_re;
  int i;
  double temp_re;
  double temp_im;
  emxInit_real_T(&costab, 2);
  emxInit_real_T(&sintab, 2);
  emxInit_real_T(&sintabinv, 2);
  emxInit_creal_T1(&wwc, 1);
  emxInit_creal_T1(&rwork, 1);
  if ((x->size[0] == 0) || (x->size[1] == 0) || ((int)varargin_1 == 0)) {
    for (istart = 0; istart < 2; istart++) {
      sz[istart] = x->size[istart];
    }

    istart = y->size[0] * y->size[1];
    y->size[0] = (int)varargin_1;
    y->size[1] = sz[1];
    emxEnsureCapacity((emxArray__common *)y, istart, (int)sizeof(creal_T));
    if ((int)varargin_1 > x->size[0]) {
      istart = y->size[0] * y->size[1];
      emxEnsureCapacity((emxArray__common *)y, istart, (int)sizeof(creal_T));
      nn1m1 = y->size[1];
      for (istart = 0; istart < nn1m1; istart++) {
        pmax = y->size[0];
        for (pmin = 0; pmin < pmax; pmin++) {
          y->data[pmin + y->size[0] * istart].re = 0.0;
          y->data[pmin + y->size[0] * istart].im = 0.0;
        }
      }
    }
  } else {
    useRadix2 = ((!((int)varargin_1 <= 0)) && (((int)varargin_1 & ((int)
      varargin_1 - 1)) == 0));
    ihi = 1;
    if (useRadix2) {
      nn1m1 = (int)varargin_1;
    } else {
      if ((int)varargin_1 > 0) {
        nn1m1 = ((int)varargin_1 + (int)varargin_1) - 1;
        pmax = 31;
        if (nn1m1 <= 1) {
          pmax = 0;
        } else {
          pmin = 0;
          exitg1 = false;
          while ((!exitg1) && (pmax - pmin > 1)) {
            istart = (pmin + pmax) >> 1;
            pow2p = 1 << istart;
            if (pow2p == nn1m1) {
              pmax = istart;
              exitg1 = true;
            } else if (pow2p > nn1m1) {
              pmax = istart;
            } else {
              pmin = istart;
            }
          }
        }

        ihi = 1 << pmax;
      }

      nn1m1 = ihi;
    }

    generate_twiddle_tables(nn1m1, useRadix2, costab, sintab, sintabinv);
    if (useRadix2) {
      for (istart = 0; istart < 2; istart++) {
        sx[istart] = (unsigned int)x->size[istart];
      }

      for (istart = 0; istart < 2; istart++) {
        sz[istart] = x->size[istart];
      }

      istart = y->size[0] * y->size[1];
      y->size[0] = (int)varargin_1;
      y->size[1] = sz[1];
      emxEnsureCapacity((emxArray__common *)y, istart, (int)sizeof(creal_T));
      if ((int)varargin_1 > x->size[0]) {
        istart = y->size[0] * y->size[1];
        emxEnsureCapacity((emxArray__common *)y, istart, (int)sizeof(creal_T));
        nn1m1 = y->size[1];
        for (istart = 0; istart < nn1m1; istart++) {
          pmax = y->size[0];
          for (pmin = 0; pmin < pmax; pmin++) {
            y->data[pmin + y->size[0] * istart].re = 0.0;
            y->data[pmin + y->size[0] * istart].im = 0.0;
          }
        }
      }

      for (k = 0; k + 1 <= (int)sx[1]; k++) {
        if (x->size[0] <= (int)varargin_1) {
          pow2p = x->size[0];
        } else {
          pow2p = (int)varargin_1;
        }

        nRowsD2 = (int)varargin_1 / 2;
        nRowsD4 = nRowsD2 / 2;
        istart = rwork->size[0];
        rwork->size[0] = (int)varargin_1;
        emxEnsureCapacity((emxArray__common *)rwork, istart, (int)sizeof(creal_T));
        nn1m1 = x->size[0];
        if ((int)varargin_1 > nn1m1) {
          nn1m1 = rwork->size[0];
          istart = rwork->size[0];
          rwork->size[0] = nn1m1;
          emxEnsureCapacity((emxArray__common *)rwork, istart, (int)sizeof
                            (creal_T));
          for (istart = 0; istart < nn1m1; istart++) {
            rwork->data[istart].re = 0.0;
            rwork->data[istart].im = 0.0;
          }
        }

        pmax = k * x->size[0];
        pmin = 0;
        nn1m1 = 0;
        for (i = 1; i < pow2p; i++) {
          rwork->data[nn1m1].re = x->data[pmax];
          rwork->data[nn1m1].im = 0.0;
          nn1m1 = (int)varargin_1;
          useRadix2 = true;
          while (useRadix2) {
            nn1m1 >>= 1;
            pmin ^= nn1m1;
            useRadix2 = ((pmin & nn1m1) == 0);
          }

          nn1m1 = pmin;
          pmax++;
        }

        rwork->data[nn1m1].re = x->data[pmax];
        rwork->data[nn1m1].im = 0.0;
        if ((int)varargin_1 > 1) {
          for (i = 0; i <= (int)varargin_1 - 2; i += 2) {
            temp_re = rwork->data[i + 1].re;
            temp_im = rwork->data[i + 1].im;
            rwork->data[i + 1].re = rwork->data[i].re - rwork->data[i + 1].re;
            rwork->data[i + 1].im = rwork->data[i].im - rwork->data[i + 1].im;
            rwork->data[i].re += temp_re;
            rwork->data[i].im += temp_im;
          }
        }

        nn1m1 = 2;
        pmax = 4;
        pmin = 1 + ((nRowsD4 - 1) << 2);
        while (nRowsD4 > 0) {
          for (i = 0; i < pmin; i += pmax) {
            temp_re = rwork->data[i + nn1m1].re;
            temp_im = rwork->data[i + nn1m1].im;
            rwork->data[i + nn1m1].re = rwork->data[i].re - temp_re;
            rwork->data[i + nn1m1].im = rwork->data[i].im - temp_im;
            rwork->data[i].re += temp_re;
            rwork->data[i].im += temp_im;
          }

          istart = 1;
          for (pow2p = nRowsD4; pow2p < nRowsD2; pow2p += nRowsD4) {
            nt_re = costab->data[pow2p];
            nt_im = sintab->data[pow2p];
            i = istart;
            ihi = istart + pmin;
            while (i < ihi) {
              temp_re = nt_re * rwork->data[i + nn1m1].re - nt_im * rwork->
                data[i + nn1m1].im;
              temp_im = nt_re * rwork->data[i + nn1m1].im + nt_im * rwork->
                data[i + nn1m1].re;
              rwork->data[i + nn1m1].re = rwork->data[i].re - temp_re;
              rwork->data[i + nn1m1].im = rwork->data[i].im - temp_im;
              rwork->data[i].re += temp_re;
              rwork->data[i].im += temp_im;
              i += pmax;
            }

            istart++;
          }

          nRowsD4 /= 2;
          nn1m1 = pmax;
          pmax <<= 1;
          pmin -= nn1m1;
        }

        for (nn1m1 = 0; nn1m1 + 1 <= (int)varargin_1; nn1m1++) {
          y->data[nn1m1 + y->size[0] * k] = rwork->data[nn1m1];
        }
      }
    } else {
      for (istart = 0; istart < 2; istart++) {
        sx[istart] = (unsigned int)x->size[istart];
      }

      pow2p = ((int)varargin_1 + (int)varargin_1) - 1;
      istart = wwc->size[0];
      wwc->size[0] = pow2p;
      emxEnsureCapacity((emxArray__common *)wwc, istart, (int)sizeof(creal_T));
      nn1m1 = (int)varargin_1;
      pmax = 0;
      wwc->data[(int)varargin_1 - 1].re = 1.0;
      wwc->data[(int)varargin_1 - 1].im = 0.0;
      pmin = (int)varargin_1 << 1;
      for (k = 1; k < (int)varargin_1; k++) {
        istart = (k << 1) - 1;
        if (pmin - pmax <= istart) {
          pmax += istart - pmin;
        } else {
          pmax += istart;
        }

        nt_im = -3.1415926535897931 * (double)pmax / (double)(int)varargin_1;
        if (nt_im == 0.0) {
          nt_re = 1.0;
          nt_im = 0.0;
        } else {
          nt_re = cos(nt_im);
          nt_im = sin(nt_im);
        }

        wwc->data[nn1m1 - 2].re = nt_re;
        wwc->data[nn1m1 - 2].im = -nt_im;
        nn1m1--;
      }

      nn1m1 = 0;
      for (k = pow2p - 1; k >= (int)varargin_1; k--) {
        wwc->data[k] = wwc->data[nn1m1];
        nn1m1++;
      }

      for (istart = 0; istart < 2; istart++) {
        sz[istart] = x->size[istart];
      }

      istart = y->size[0] * y->size[1];
      y->size[0] = (int)varargin_1;
      y->size[1] = sz[1];
      emxEnsureCapacity((emxArray__common *)y, istart, (int)sizeof(creal_T));
      if ((int)varargin_1 > x->size[0]) {
        istart = y->size[0] * y->size[1];
        emxEnsureCapacity((emxArray__common *)y, istart, (int)sizeof(creal_T));
        nn1m1 = y->size[1];
        for (istart = 0; istart < nn1m1; istart++) {
          pmax = y->size[0];
          for (pmin = 0; pmin < pmax; pmin++) {
            y->data[pmin + y->size[0] * istart].re = 0.0;
            y->data[pmin + y->size[0] * istart].im = 0.0;
          }
        }
      }

      for (k = 0; k + 1 <= (int)sx[1]; k++) {
        bluestein(x, k * x->size[0], ihi, (int)varargin_1, costab, sintab,
                  costab, sintabinv, wwc, rwork);
        for (nn1m1 = 0; nn1m1 + 1 <= (int)varargin_1; nn1m1++) {
          y->data[nn1m1 + y->size[0] * k] = rwork->data[nn1m1];
        }
      }
    }
  }

  emxFree_creal_T(&rwork);
  emxFree_creal_T(&wwc);
  emxFree_real_T(&sintabinv);
  emxFree_real_T(&sintab);
  emxFree_real_T(&costab);
}

/*
 * File trailer for fft.c
 *
 * [EOF]
 */
