/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: trifbank.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 22-Mar-2018 18:45:34
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "mfcc.h"
#include "trifbank.h"
#include "mfcc_emxutil.h"

/* Function Definitions */

/*
 * TRIFBANK Triangular filterbank.
 *
 *    [H,F,C]=TRIFBANK(M,K,R,FS,H2W,W2H) returns matrix of M triangular filters
 *    (one per row), each K coefficients long along with a K coefficient long
 *    frequency vector F and M+2 coefficient long cutoff frequency vector C.
 *    The triangular filters are between limits given in R (Hz) and are
 *    uniformly spaced on a warped scale defined by forward (H2W) and backward
 *    (W2H) warping functions.
 *
 *    Inputs
 *            M is the number of filters, i.e., number of rows of H
 *
 *            K is the length of frequency response of each filter
 *              i.e., number of columns of H
 *
 *            R is a two element vector that specifies frequency limits (Hz),
 *              i.e., R = [ low_frequency high_frequency ];
 *
 *            FS is the sampling frequency (Hz)
 *
 *            H2W is a Hertz scale to warped scale function handle
 *
 *            W2H is a wared scale to Hertz scale function handle
 *
 *    Outputs
 *            H is a M by K triangular filterbank matrix (one filter per row)
 *
 *            F is a frequency vector (Hz) of 1xK dimension
 *
 *            C is a vector of filter cutoff frequencies (Hz),
 *              note that C(2:end) also represents filter center frequencies,
 *              and the dimension of C is 1x(M+2)
 *
 *    Example
 *            fs = 16000;               % sampling frequency (Hz)
 *            nfft = 2^12;              % fft size (number of frequency bins)
 *            K = nfft/2+1;             % length of each filter
 *            M = 23;                   % number of filters
 *
 *            hz2mel = @(hz)(1127*log(1+hz/700)); % Hertz to mel warping function
 *            mel2hz = @(mel)(700*exp(mel/1127)-700); % mel to Hertz warping function
 *
 *            % Design mel filterbank of M filters each K coefficients long,
 *            % filters are uniformly spaced on the mel scale between 0 and Fs/2 Hz
 *            [ H1, freq ] = trifbank( M, K, [0 fs/2], fs, hz2mel, mel2hz );
 *
 *            % Design mel filterbank of M filters each K coefficients long,
 *            % filters are uniformly spaced on the mel scale between 300 and 3750 Hz
 *            [ H2, freq ] = trifbank( M, K, [300 3750], fs, hz2mel, mel2hz );
 *
 *            % Design mel filterbank of 18 filters each K coefficients long,
 *            % filters are uniformly spaced on the Hertz scale between 4 and 6 kHz
 *            [ H3, freq ] = trifbank( 18, K, [4 6]*1E3, fs, @(h)(h), @(h)(h) );
 *
 *             hfig = figure('Position', [25 100 800 600], 'PaperPositionMode', ...
 *                               'auto', 'Visible', 'on', 'color', 'w'); hold on;
 *            subplot( 3,1,1 );
 *            plot( freq, H1 );
 *            xlabel( 'Frequency (Hz)' ); ylabel( 'Weight' ); set( gca, 'box', 'off' );
 *
 *            subplot( 3,1,2 );
 *            plot( freq, H2 );
 *            xlabel( 'Frequency (Hz)' ); ylabel( 'Weight' ); set( gca, 'box', 'off' );
 *
 *            subplot( 3,1,3 );
 *            plot( freq, H3 );
 *            xlabel( 'Frequency (Hz)' ); ylabel( 'Weight' ); set( gca, 'box', 'off' );
 *
 *    Reference
 *            [1] Huang, X., Acero, A., Hon, H., 2001. Spoken Language Processing:
 *                A guide to theory, algorithm, and system development.
 *                Prentice Hall, Upper Saddle River, NJ, USA (pp. 314-315).
 * Arguments    : double M
 *                double K
 *                const double R[2]
 *                double fs
 *                emxArray_real_T *H
 * Return Type  : void
 */
void trifbank(double M, double K, const double R[2], double fs, emxArray_real_T *
              H)
{
  emxArray_real_T *f;
  double f_max;
  int y;
  emxArray_real_T *b_y;
  int k;
  double c_y;
  emxArray_real_T *c;
  int m;
  emxArray_boolean_T *r0;
  emxArray_boolean_T *r1;
  emxArray_int32_T *r2;
  emxArray_int32_T *r3;
  int end;
  emxInit_real_T(&f, 2);

  /*    Author  Kamil Wojcicki, UTD, June 2011 */
  /*     if( nargin~= 6 ), help trifbank; return; end; % very lite input validation */
  /*  filter coefficients start at this frequency (Hz) */
  /*  lower cutoff frequency (Hz) for the filterbank  */
  /*  upper cutoff frequency (Hz) for the filterbank  */
  f_max = 0.5 * fs;

  /*  filter coefficients end at this frequency (Hz) */
  y = f->size[0] * f->size[1];
  f->size[0] = 1;
  f->size[1] = (int)floor(K);
  emxEnsureCapacity((emxArray__common *)f, y, (int)sizeof(double));
  f->data[f->size[1] - 1] = f_max;
  if (f->size[1] >= 2) {
    f->data[0] = 0.0;
    if (f->size[1] >= 3) {
      if ((f_max < 0.0) && (fabs(f_max) > 8.9884656743115785E+307)) {
        f_max /= (double)f->size[1] - 1.0;
        y = f->size[1];
        for (k = 0; k <= y - 3; k++) {
          f->data[1 + k] = f_max * (1.0 + (double)k);
        }
      } else {
        f_max /= (double)f->size[1] - 1.0;
        y = f->size[1];
        for (k = 0; k <= y - 3; k++) {
          f->data[1 + k] = (1.0 + (double)k) * f_max;
        }
      }
    }
  }

  /*  frequency range (Hz), size 1xK */
  /*  filter cutoff frequencies (Hz) for all filters, size 1x(M+2) */
  f_max = 1127.0 * log(1.0 + R[0] / 700.0);
  emxInit_real_T(&b_y, 2);
  if (rtIsNaN(M + 1.0)) {
    y = b_y->size[0] * b_y->size[1];
    b_y->size[0] = 1;
    b_y->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)b_y, y, (int)sizeof(double));
    b_y->data[0] = rtNaN;
  } else if (M + 1.0 < 0.0) {
    y = b_y->size[0] * b_y->size[1];
    b_y->size[0] = 1;
    b_y->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)b_y, y, (int)sizeof(double));
  } else if (rtIsInf(M + 1.0) && (0.0 == M + 1.0)) {
    y = b_y->size[0] * b_y->size[1];
    b_y->size[0] = 1;
    b_y->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)b_y, y, (int)sizeof(double));
    b_y->data[0] = rtNaN;
  } else {
    y = b_y->size[0] * b_y->size[1];
    b_y->size[0] = 1;
    b_y->size[1] = (int)floor(M + 1.0) + 1;
    emxEnsureCapacity((emxArray__common *)b_y, y, (int)sizeof(double));
    k = (int)floor(M + 1.0);
    for (y = 0; y <= k; y++) {
      b_y->data[b_y->size[0] * y] = y;
    }
  }

  c_y = (1127.0 * log(1.0 + R[1] / 700.0) - 1127.0 * log(1.0 + R[0] / 700.0)) /
    (M + 1.0);
  y = b_y->size[0] * b_y->size[1];
  b_y->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)b_y, y, (int)sizeof(double));
  k = b_y->size[0];
  y = b_y->size[1];
  k *= y;
  for (y = 0; y < k; y++) {
    b_y->data[y] = (f_max + b_y->data[y] * c_y) / 1127.0;
  }

  emxInit_real_T(&c, 2);
  y = c->size[0] * c->size[1];
  c->size[0] = 1;
  c->size[1] = b_y->size[1];
  emxEnsureCapacity((emxArray__common *)c, y, (int)sizeof(double));
  k = b_y->size[0] * b_y->size[1];
  for (y = 0; y < k; y++) {
    c->data[y] = b_y->data[y];
  }

  for (k = 0; k + 1 <= b_y->size[1]; k++) {
    c->data[k] = exp(c->data[k]);
  }

  emxFree_real_T(&b_y);
  y = c->size[0] * c->size[1];
  c->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)c, y, (int)sizeof(double));
  k = c->size[0];
  y = c->size[1];
  k *= y;
  for (y = 0; y < k; y++) {
    c->data[y] = 700.0 * c->data[y] - 700.0;
  }

  y = H->size[0] * H->size[1];
  H->size[0] = (int)M;
  H->size[1] = (int)K;
  emxEnsureCapacity((emxArray__common *)H, y, (int)sizeof(double));
  k = (int)M * (int)K;
  for (y = 0; y < k; y++) {
    H->data[y] = 0.0;
  }

  /*  zero otherwise */
  m = 0;
  emxInit_boolean_T(&r0, 2);
  emxInit_boolean_T(&r1, 2);
  emxInit_int32_T(&r2, 2);
  emxInit_int32_T(&r3, 2);
  while (m <= (int)M - 1) {
    /*  implements Eq. (6.140) on page 314 of [1]  */
    /*  k = f>=c(m)&f<=c(m+1); % up-slope */
    /*  H(m,k) = 2*(f(k)-c(m)) / ((c(m+2)-c(m))*(c(m+1)-c(m))); */
    /*  k = f>=c(m+1)&f<=c(m+2); % down-slope */
    /*  H(m,k) = 2*(c(m+2)-f(k)) / ((c(m+2)-c(m))*(c(m+2)-c(m+1))); */
    /*  implements Eq. (6.141) on page 315 of [1] */
    y = r0->size[0] * r0->size[1];
    r0->size[0] = 1;
    r0->size[1] = f->size[1];
    emxEnsureCapacity((emxArray__common *)r0, y, (int)sizeof(boolean_T));
    f_max = c->data[m];
    k = f->size[0] * f->size[1];
    for (y = 0; y < k; y++) {
      r0->data[y] = (f->data[y] >= f_max);
    }

    y = r1->size[0] * r1->size[1];
    r1->size[0] = 1;
    r1->size[1] = f->size[1];
    emxEnsureCapacity((emxArray__common *)r1, y, (int)sizeof(boolean_T));
    f_max = c->data[(int)((1.0 + (double)m) + 1.0) - 1];
    k = f->size[0] * f->size[1];
    for (y = 0; y < k; y++) {
      r1->data[y] = (f->data[y] <= f_max);
    }

    /*  up-slope */
    end = r0->size[1] - 1;
    k = 0;
    for (y = 0; y <= end; y++) {
      if (r0->data[y] && r1->data[y]) {
        k++;
      }
    }

    y = r2->size[0] * r2->size[1];
    r2->size[0] = 1;
    r2->size[1] = k;
    emxEnsureCapacity((emxArray__common *)r2, y, (int)sizeof(int));
    k = 0;
    for (y = 0; y <= end; y++) {
      if (r0->data[y] && r1->data[y]) {
        r2->data[k] = y + 1;
        k++;
      }
    }

    c_y = c->data[(int)((1.0 + (double)m) + 1.0) - 1] - c->data[m];
    end = r0->size[1] - 1;
    k = 0;
    for (y = 0; y <= end; y++) {
      if (r0->data[y] && r1->data[y]) {
        k++;
      }
    }

    y = r3->size[0] * r3->size[1];
    r3->size[0] = 1;
    r3->size[1] = k;
    emxEnsureCapacity((emxArray__common *)r3, y, (int)sizeof(int));
    k = 0;
    for (y = 0; y <= end; y++) {
      if (r0->data[y] && r1->data[y]) {
        r3->data[k] = y + 1;
        k++;
      }
    }

    f_max = c->data[m];
    k = r2->size[1];
    for (y = 0; y < k; y++) {
      H->data[m + H->size[0] * (r3->data[r3->size[0] * y] - 1)] = (f->data
        [r2->data[r2->size[0] * y] - 1] - f_max) / c_y;
    }

    y = r0->size[0] * r0->size[1];
    r0->size[0] = 1;
    r0->size[1] = f->size[1];
    emxEnsureCapacity((emxArray__common *)r0, y, (int)sizeof(boolean_T));
    f_max = c->data[(int)((1.0 + (double)m) + 1.0) - 1];
    k = f->size[0] * f->size[1];
    for (y = 0; y < k; y++) {
      r0->data[y] = (f->data[y] >= f_max);
    }

    y = r1->size[0] * r1->size[1];
    r1->size[0] = 1;
    r1->size[1] = f->size[1];
    emxEnsureCapacity((emxArray__common *)r1, y, (int)sizeof(boolean_T));
    f_max = c->data[(int)((1.0 + (double)m) + 2.0) - 1];
    k = f->size[0] * f->size[1];
    for (y = 0; y < k; y++) {
      r1->data[y] = (f->data[y] <= f_max);
    }

    /*  down-slope */
    end = r0->size[1] - 1;
    k = 0;
    for (y = 0; y <= end; y++) {
      if (r0->data[y] && r1->data[y]) {
        k++;
      }
    }

    y = r2->size[0] * r2->size[1];
    r2->size[0] = 1;
    r2->size[1] = k;
    emxEnsureCapacity((emxArray__common *)r2, y, (int)sizeof(int));
    k = 0;
    for (y = 0; y <= end; y++) {
      if (r0->data[y] && r1->data[y]) {
        r2->data[k] = y + 1;
        k++;
      }
    }

    c_y = c->data[(int)((1.0 + (double)m) + 2.0) - 1] - c->data[(int)((1.0 +
      (double)m) + 1.0) - 1];
    end = r0->size[1] - 1;
    k = 0;
    for (y = 0; y <= end; y++) {
      if (r0->data[y] && r1->data[y]) {
        k++;
      }
    }

    y = r3->size[0] * r3->size[1];
    r3->size[0] = 1;
    r3->size[1] = k;
    emxEnsureCapacity((emxArray__common *)r3, y, (int)sizeof(int));
    k = 0;
    for (y = 0; y <= end; y++) {
      if (r0->data[y] && r1->data[y]) {
        r3->data[k] = y + 1;
        k++;
      }
    }

    f_max = c->data[(int)((1.0 + (double)m) + 2.0) - 1];
    k = r2->size[1];
    for (y = 0; y < k; y++) {
      H->data[m + H->size[0] * (r3->data[r3->size[0] * y] - 1)] = (f_max -
        f->data[r2->data[r2->size[0] * y] - 1]) / c_y;
    }

    m++;
  }

  emxFree_int32_T(&r3);
  emxFree_int32_T(&r2);
  emxFree_boolean_T(&r1);
  emxFree_boolean_T(&r0);
  emxFree_real_T(&c);
  emxFree_real_T(&f);

  /*  H = H./repmat(max(H,[],2),1,K);  % normalize to unit height (inherently done) */
  /*  H = H./repmat(trapz(f,H,2),1,K); % normalize to unit area  */
  /*  EOF */
}

/*
 * File trailer for trifbank.c
 *
 * [EOF]
 */
