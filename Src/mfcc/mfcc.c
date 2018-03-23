/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: mfcc.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 22-Mar-2018 18:45:34
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "mfcc.h"
#include "mfcc_emxutil.h"
#include "diag.h"
#include "log.h"
#include "trifbank.h"
#include "fft.h"

/* Function Declarations */
static void __anon_fcn(double N, double L, emxArray_real_T *varargout_1);
static double rt_hypotd_snf(double u0, double u1);
static double rt_powd_snf(double u0, double u1);
static double rt_roundd_snf(double u);

/* Function Definitions */

/*
 * Arguments    : double N
 *                double L
 *                emxArray_real_T *varargout_1
 * Return Type  : void
 */
static void __anon_fcn(double N, double L, emxArray_real_T *varargout_1)
{
  emxArray_real_T *y;
  int b_y;
  int loop_ub;
  double a;
  emxInit_real_T(&y, 2);
  if (rtIsNaN(N - 1.0)) {
    b_y = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)y, b_y, (int)sizeof(double));
    y->data[0] = rtNaN;
  } else if (N - 1.0 < 0.0) {
    b_y = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)y, b_y, (int)sizeof(double));
  } else if (rtIsInf(N - 1.0) && (0.0 == N - 1.0)) {
    b_y = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)y, b_y, (int)sizeof(double));
    y->data[0] = rtNaN;
  } else {
    b_y = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = (int)floor(N - 1.0) + 1;
    emxEnsureCapacity((emxArray__common *)y, b_y, (int)sizeof(double));
    loop_ub = (int)floor(N - 1.0);
    for (b_y = 0; b_y <= loop_ub; b_y++) {
      y->data[y->size[0] * b_y] = b_y;
    }
  }

  b_y = y->size[0] * y->size[1];
  y->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)y, b_y, (int)sizeof(double));
  b_y = y->size[0];
  loop_ub = y->size[1];
  loop_ub *= b_y;
  for (b_y = 0; b_y < loop_ub; b_y++) {
    y->data[b_y] = 3.1415926535897931 * y->data[b_y] / L;
  }

  b_y = varargout_1->size[0] * varargout_1->size[1];
  varargout_1->size[0] = 1;
  varargout_1->size[1] = y->size[1];
  emxEnsureCapacity((emxArray__common *)varargout_1, b_y, (int)sizeof(double));
  loop_ub = y->size[0] * y->size[1];
  for (b_y = 0; b_y < loop_ub; b_y++) {
    varargout_1->data[b_y] = y->data[b_y];
  }

  for (b_y = 0; b_y + 1 <= y->size[1]; b_y++) {
    varargout_1->data[b_y] = sin(varargout_1->data[b_y]);
  }

  emxFree_real_T(&y);
  a = 0.5 * L;
  b_y = varargout_1->size[0] * varargout_1->size[1];
  varargout_1->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)varargout_1, b_y, (int)sizeof(double));
  loop_ub = varargout_1->size[0];
  b_y = varargout_1->size[1];
  loop_ub *= b_y;
  for (b_y = 0; b_y < loop_ub; b_y++) {
    varargout_1->data[b_y] = 1.0 + a * varargout_1->data[b_y];
  }
}

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_hypotd_snf(double u0, double u1)
{
  double y;
  double a;
  double b;
  a = fabs(u0);
  b = fabs(u1);
  if (a < b) {
    a /= b;
    y = b * sqrt(a * a + 1.0);
  } else if (a > b) {
    b /= a;
    y = a * sqrt(b * b + 1.0);
  } else if (rtIsNaN(b)) {
    y = b;
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_powd_snf(double u0, double u1)
{
  double y;
  double d0;
  double d1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    d0 = fabs(u0);
    d1 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d0 == 1.0) {
        y = rtNaN;
      } else if (d0 > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

/*
 * Arguments    : double u
 * Return Type  : double
 */
static double rt_roundd_snf(double u)
{
  double y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

/*
 * MFCC Mel frequency cepstral coefficient feature extraction.
 *
 *    MFCC(S,FS,TW,TS,ALPHA,WINDOW,R,M,N,L) returns mel frequency
 *    cepstral coefficients (MFCCs) computed from speech signal given
 *    in vector S and sampled at FS (Hz). The speech signal is first
 *    preemphasised using a first order FIR filter with preemphasis
 *    coefficient ALPHA. The preemphasised speech signal is subjected
 *    to the short-time Fourier transform analysis with frame durations
 *    of TW (ms), frame shifts of TS (ms) and analysis window function
 *    given as a function handle in WINDOW. This is followed by magnitude
 *    spectrum computation followed by filterbank design with M triangular
 *    filters uniformly spaced on the mel scale between lower and upper
 *    frequency limits given in R (Hz). The filterbank is applied to
 *    the magnitude spectrum values to produce filterbank energies (FBEs)
 *    (M per frame). Log-compressed FBEs are then decorrelated using the
 *    discrete cosine transform to produce cepstral coefficients. Final
 *    step applies sinusoidal lifter to produce liftered MFCCs that
 *    closely match those produced by HTK [1].
 *
 *    [CC,FBE,FRAMES]=MFCC(...) also returns FBEs and windowed frames,
 *    with feature vectors and frames as columns.
 *
 *    This framework is based on Dan Ellis' rastamat routines [2]. The
 *    emphasis is placed on closely matching MFCCs produced by HTK [1]
 *    (refer to p.337 of [1] for HTK's defaults) with simplicity and
 *    compactness as main considerations, but at a cost of reduced
 *    flexibility. This routine is meant to be easy to extend, and as
 *    a starting point for work with cepstral coefficients in MATLAB.
 *    The triangular filterbank equations are given in [3].
 *
 *    Inputs
 *            S is the input speech signal (as vector)
 *
 *            FS is the sampling frequency (Hz)
 *
 *            TW is the analysis frame duration (ms)
 *
 *            TS is the analysis frame shift (ms)
 *
 *            ALPHA is the preemphasis coefficient
 *
 *
 *            R is the frequency range (Hz) for filterbank analysis
 *
 *            M is the number of filterbank channels
 *
 *            N is the number of cepstral coefficients
 *              (including the 0th coefficient)
 *
 *            L is the liftering parameter
 *
 *    Outputs
 *            CC is a matrix of mel frequency cepstral coefficients
 *               (MFCCs) with feature vectors as columns
 *
 *            FBE is a matrix of filterbank energies
 *                with feature vectors as columns
 *
 *            FRAMES is a matrix of windowed frames
 *                   (one frame per column)
 *
 *    Example
 *            Tw = 25;           % analysis frame duration (ms)
 *            Ts = 10;           % analysis frame shift (ms)
 *            alpha = 0.97;      % preemphasis coefficient
 *            R = [ 300 3700 ];  % frequency range to consider
 *            M = 20;            % number of filterbank channels
 *            C = 13;            % number of cepstral coefficients
 *            L = 22;            % cepstral sine lifter parameter
 *
 *            % hamming window (see Eq. (5.2) on p.73 of [1])
 *            hamming = @(N)(0.54-0.46*cos(2*pi*[0:N-1].'/(N-1)));
 *
 *            % Read speech samples, sampling rate and precision from file
 *            [ speech, fs, nbits ] = wavread( 'sp10.wav' );
 *
 *            % Feature extraction (feature vectors as columns)
 *            [ MFCCs, FBEs, frames ] = ...
 *                            mfcc( speech, fs, Tw, Ts, alpha, hamming, R, M, C, L );
 *
 *            % Plot cepstrum over time
 *            figure('Position', [30 100 800 200], 'PaperPositionMode', 'auto', ...
 *                   'color', 'w', 'PaperOrientation', 'landscape', 'Visible', 'on' );
 *
 *            imagesc( [1:size(MFCCs,2)], [0:C-1], MFCCs );
 *            axis( 'xy' );
 *            xlabel( 'Frame index' );
 *            ylabel( 'Cepstrum index' );
 *            title( 'Mel frequency cepstrum' );
 *
 *    References
 *
 *            [1] Young, S., Evermann, G., Gales, M., Hain, T., Kershaw, D.,
 *                Liu, X., Moore, G., Odell, J., Ollason, D., Povey, D.,
 *                Valtchev, V., Woodland, P., 2006. The HTK Book (for HTK
 *                Version 3.4.1). Engineering Department, Cambridge University.
 *                (see also: http://htk.eng.cam.ac.uk)
 *
 *            [2] Ellis, D., 2005. Reproducing the feature outputs of
 *                common programs using Matlab and melfcc.m. url:
 *                http://labrosa.ee.columbia.edu/matlab/rastamat/mfccs.html
 *
 *            [3] Huang, X., Acero, A., Hon, H., 2001. Spoken Language
 *                Processing: A guide to theory, algorithm, and system
 *                development. Prentice Hall, Upper Saddle River, NJ,
 *                USA (pp. 314-315).
 *
 *    See also EXAMPLE, COMPARE, FRAMES2VEC, TRIFBANK.
 * Arguments    : double speech[4000]
 *                double fs
 *                double Tw
 *                double Ts
 *                double alpha
 *                const double R[2]
 *                double M
 *                double N
 *                double L
 *                emxArray_real_T *CC
 *                emxArray_real_T *FBE
 *                emxArray_real_T *frames
 * Return Type  : void
 */
void mfcc(double speech[4000], double fs, double Tw, double Ts, double alpha,
          const double R[2], double M, double N, double L, emxArray_real_T *CC,
          emxArray_real_T *FBE, emxArray_real_T *frames)
{
  double y[4000];
  int k;
  int ixstart;
  double absn;
  int nx;
  boolean_T exitg1;
  int i0;
  double Nw;
  double Ns;
  double f;
  int br;
  emxArray_real_T *mat1;
  emxArray_real_T *b_y;
  emxArray_real_T *mat2;
  int ar;
  double b[2];
  double b_M;
  emxArray_real_T *indf;
  emxArray_real_T *inds;
  emxArray_creal_T *x;
  int i1;
  emxArray_real_T *MAG;
  emxArray_real_T *H;
  emxArray_real_T *b_b;
  int m;
  int ic;
  int ib;
  int ia;
  emxArray_real_T *b_mat1;

  /*    Author: Kamil Wojcicki, September 2011 */
  /*     %% PRELIMINARIES  */
  /*     % Ensure correct number of inputs */
  /*     if( nargin~= 9 ), help mfcc; return; end;  */
  /*  Explode samples to the range of 16 bit shorts */
  for (k = 0; k < 4000; k++) {
    y[k] = fabs(speech[k]);
  }

  ixstart = 1;
  absn = y[0];
  if (rtIsNaN(y[0])) {
    nx = 2;
    exitg1 = false;
    while ((!exitg1) && (nx < 4001)) {
      ixstart = nx;
      if (!rtIsNaN(y[nx - 1])) {
        absn = y[nx - 1];
        exitg1 = true;
      } else {
        nx++;
      }
    }
  }

  if (ixstart < 4000) {
    while (ixstart + 1 < 4001) {
      if (y[ixstart] > absn) {
        absn = y[ixstart];
      }

      ixstart++;
    }
  }

  if (absn <= 1.0) {
    for (i0 = 0; i0 < 4000; i0++) {
      speech[i0] *= 32768.0;
    }
  }

  Nw = rt_roundd_snf(0.001 * Tw * fs);

  /*  frame duration (samples) */
  Ns = rt_roundd_snf(0.001 * Ts * fs);

  /*  frame shift (samples) */
  absn = fabs(Nw);
  if ((!rtIsInf(absn)) && (!rtIsNaN(absn))) {
    f = frexp(absn, &br);
    absn = br;
    if (f == 0.5) {
      absn = (double)br - 1.0;
    }
  }

  emxInit_real_T(&mat1, 2);
  absn = rt_powd_snf(2.0, absn);

  /*  length of FFT analysis  */
  f = absn / 2.0;

  /*  length of the unique part of the FFT  */
  /*     %% HANDY INLINE FUNCTION HANDLES */
  /*  Forward and backward mel frequency warping (see Eq. (5.13) on p.76 of [1])  */
  /*  Note that base 10 is used in [1], while base e is used here and in HTK code */
  /*  Hertz to mel warping function */
  /*  mel to Hertz warping function */
  /*  Type III DCT matrix routine (see Eq. (5.14) on p.77 of [1]) */
  i0 = mat1->size[0] * mat1->size[1];
  mat1->size[0] = (int)N;
  mat1->size[1] = (int)M;
  emxEnsureCapacity((emxArray__common *)mat1, i0, (int)sizeof(double));
  nx = 0;
  emxInit_real_T(&b_y, 2);
  while (nx <= (int)M - 1) {
    if (N - 1.0 < 0.0) {
      i0 = b_y->size[0] * b_y->size[1];
      b_y->size[0] = 1;
      b_y->size[1] = 0;
      emxEnsureCapacity((emxArray__common *)b_y, i0, (int)sizeof(double));
    } else if (rtIsInf(N - 1.0) && (0.0 == N - 1.0)) {
      i0 = b_y->size[0] * b_y->size[1];
      b_y->size[0] = 1;
      b_y->size[1] = 1;
      emxEnsureCapacity((emxArray__common *)b_y, i0, (int)sizeof(double));
      b_y->data[0] = rtNaN;
    } else {
      i0 = b_y->size[0] * b_y->size[1];
      b_y->size[0] = 1;
      b_y->size[1] = (int)floor(N - 1.0) + 1;
      emxEnsureCapacity((emxArray__common *)b_y, i0, (int)sizeof(double));
      ar = (int)floor(N - 1.0);
      for (i0 = 0; i0 <= ar; i0++) {
        b_y->data[b_y->size[0] * i0] = i0;
      }
    }

    ar = b_y->size[1];
    for (i0 = 0; i0 < ar; i0++) {
      mat1->data[i0 + mat1->size[0] * nx] = b_y->data[b_y->size[0] * i0];
    }

    nx++;
  }

  emxInit_real_T(&mat2, 2);
  i0 = mat2->size[0] * mat2->size[1];
  mat2->size[0] = (int)N;
  mat2->size[1] = (int)M;
  emxEnsureCapacity((emxArray__common *)mat2, i0, (int)sizeof(double));
  for (nx = 0; nx < (int)N; nx++) {
    if (M < 1.0) {
      i0 = b_y->size[0] * b_y->size[1];
      b_y->size[0] = 1;
      b_y->size[1] = 0;
      emxEnsureCapacity((emxArray__common *)b_y, i0, (int)sizeof(double));
    } else if (rtIsInf(M) && (1.0 == M)) {
      i0 = b_y->size[0] * b_y->size[1];
      b_y->size[0] = 1;
      b_y->size[1] = 1;
      emxEnsureCapacity((emxArray__common *)b_y, i0, (int)sizeof(double));
      b_y->data[0] = rtNaN;
    } else {
      i0 = b_y->size[0] * b_y->size[1];
      b_y->size[0] = 1;
      b_y->size[1] = (int)floor(M - 1.0) + 1;
      emxEnsureCapacity((emxArray__common *)b_y, i0, (int)sizeof(double));
      ar = (int)floor(M - 1.0);
      for (i0 = 0; i0 <= ar; i0++) {
        b_y->data[b_y->size[0] * i0] = 1.0 + (double)i0;
      }
    }

    ar = b_y->size[1];
    for (i0 = 0; i0 < ar; i0++) {
      mat2->data[nx + mat2->size[0] * i0] = 3.1415926535897931 * (b_y->data
        [b_y->size[0] * i0] - 0.5) / M;
    }
  }

  /*  Cepstral lifter routine (see Eq. (5.12) on p.75 of [1]) */
  /*     %% FEATURE EXTRACTION  */
  /*  Preemphasis filtering (see Eq. (5.1) on p.73 of [1]) */
  b[0] = 1.0;
  b[1] = -alpha;
  for (i0 = 0; i0 < 4000; i0++) {
    y[i0] = speech[i0];
    speech[i0] = 0.0;
  }

  for (k = 0; k < 2; k++) {
    for (ixstart = k; ixstart + 1 < 4001; ixstart++) {
      speech[ixstart] += b[k] * y[ixstart - k];
    }
  }

  /*  fvtool( [1 -alpha], 1 ); */
  /*  Framing and windowing (frames as columns) */
  /*  VEC2FRAMES Splits signal into overlapped frames using indexing. */
  /*   */
  /*    B=vec2frames(A,M,N) creates a matrix B whose columns consist of  */
  /*    segments of length M, taken at every N samples along input vector A. */
  /*  */
  /*    [B,R]=vec2frames(A,M,N,D,W,P) creates a matrix B whose columns  */
  /*    or rows, as specified by D, consist of segments of length M, taken  */
  /*    at every N samples along the input vector A and windowed using the */
  /*    analysis window specified by W. The division of A into frames is  */
  /*    achieved using indexes returned in R as follows: B=A(R); */
  /*  */
  /*    Summary */
  /*  */
  /*            A is an input vector */
  /*  */
  /*            M is a frame length (in samples) */
  /*  */
  /*            N is a frame shift (in samples) */
  /*  */
  /*            D specifies if the frames in B are rows or columns, */
  /*              i.e., D = 'rows' or 'cols', respectively */
  /*  */
  /*            W is an optional analysis window function to be applied to  */
  /*              each frame, given as a function handle, e.g., W = @hanning */
  /*              or as a vector of window samples, e.g., W = hanning( M ) */
  /*  */
  /*            P specifies if last frame should be padded to full length, */
  /*              or simply discarded, i.e., P = true or false, respectively */
  /*     */
  /*            B is the output matrix of frames */
  /*  */
  /*            R is a matrix of indexes used for framing, such that division  */
  /*              of A into frames is achieved as follows: B=A(R); */
  /*  */
  /*    Examples */
  /*  */
  /*            % divide the input vector into seven-sample-long frames with a shift */
  /*            % of three samples and return frames as columns of the output matrix */
  /*            % (note that the last sample of the input vector is discarded) */
  /*            vec2frames( [1:20], 7, 3 ) */
  /*  */
  /*            % divide the input vector into seven-sample-long frames with a shift */
  /*            % of three samples and return frames as rows of the output matrix */
  /*            % (note that the last sample of the input vector is discarded) */
  /*            vec2frames( [1:20], 7, 3, 'rows' ) */
  /*  */
  /*            % divide the input vector into seven-sample-long frames with a shift */
  /*            % of three samples, pad the last frame with zeros so that no samples */
  /*            % are discarded and return frames as rows of the output matrix */
  /*            vec2frames( [1:20], 7, 3, 'rows', [], true ) */
  /*  */
  /*            % divide the input vector into seven-sample-long frames with a shift */
  /*            % of three samples, pad the last frame with white Gaussian noise */
  /*            % of variance (1E-5)^2 so that no samples are discarded and  */
  /*            % return frames as rows of the output matrix */
  /*            vec2frames( [1:20], 7, 3, 'rows', false, { 'noise', 1E-5 } ) */
  /*  */
  /*            % divide the input vector into seven-sample-long frames with a shift */
  /*            % of three samples, pad the last frame with zeros so that no samples  */
  /*            % are discarded, apply the Hanning analysis window to each frame and */
  /*            % return frames as columns of the output matrix */
  /*            vec2frames( [1:20], 7, 3, 'cols', @hanning, 0 ) */
  /*   */
  /*    See also FRAMES2VEC, DEMO */
  /*    Author: Kamil Wojcicki, UTD, July 2011 */
  /*  usage information */
  /*  default settings  */
  /*  input validation */
  /*  ensure column vector */
  /*  length of the input vector */
  b_M = floor((4000.0 - Nw) / Ns + 1.0);

  /*  number of frames  */
  /*  perform signal padding to enable exact division of signal samples into frames  */
  /*  (note that if padding is disabled, some samples may be discarded) */
  /*  figure out if the input vector can be divided into frames exactly */
  /*  see if padding is actually needed */
  if (4000.0 - ((b_M - 1.0) * Ns + Nw) > 0.0) {
    /*  how much padding will be needed to complete the last frame? */
    /*  pad with zeros */
    b_M--;

    /*  increment the frame count */
    b_M++;
  }

  /*  compute index matrix  */
  /*  for frames as columns */
  emxInit_real_T(&indf, 2);
  if (rtIsNaN(b_M - 1.0)) {
    i0 = indf->size[0] * indf->size[1];
    indf->size[0] = 1;
    indf->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)indf, i0, (int)sizeof(double));
    indf->data[0] = rtNaN;
  } else if (b_M - 1.0 < 0.0) {
    i0 = indf->size[0] * indf->size[1];
    indf->size[0] = 1;
    indf->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)indf, i0, (int)sizeof(double));
  } else if (rtIsInf(b_M - 1.0) && (0.0 == b_M - 1.0)) {
    i0 = indf->size[0] * indf->size[1];
    indf->size[0] = 1;
    indf->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)indf, i0, (int)sizeof(double));
    indf->data[0] = rtNaN;
  } else {
    i0 = indf->size[0] * indf->size[1];
    indf->size[0] = 1;
    indf->size[1] = (int)(b_M - 1.0) + 1;
    emxEnsureCapacity((emxArray__common *)indf, i0, (int)sizeof(double));
    ar = (int)(b_M - 1.0);
    for (i0 = 0; i0 <= ar; i0++) {
      indf->data[indf->size[0] * i0] = i0;
    }
  }

  i0 = indf->size[0] * indf->size[1];
  indf->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)indf, i0, (int)sizeof(double));
  ixstart = indf->size[0];
  nx = indf->size[1];
  ar = ixstart * nx;
  for (i0 = 0; i0 < ar; i0++) {
    indf->data[i0] *= Ns;
  }

  /*  indexes for frames       */
  if (rtIsNaN(Nw)) {
    i0 = b_y->size[0] * b_y->size[1];
    b_y->size[0] = 1;
    b_y->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)b_y, i0, (int)sizeof(double));
    b_y->data[0] = rtNaN;
  } else if (Nw < 1.0) {
    i0 = b_y->size[0] * b_y->size[1];
    b_y->size[0] = 1;
    b_y->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)b_y, i0, (int)sizeof(double));
  } else if (rtIsInf(Nw) && (1.0 == Nw)) {
    i0 = b_y->size[0] * b_y->size[1];
    b_y->size[0] = 1;
    b_y->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)b_y, i0, (int)sizeof(double));
    b_y->data[0] = rtNaN;
  } else {
    i0 = b_y->size[0] * b_y->size[1];
    b_y->size[0] = 1;
    b_y->size[1] = (int)floor(Nw - 1.0) + 1;
    emxEnsureCapacity((emxArray__common *)b_y, i0, (int)sizeof(double));
    ar = (int)floor(Nw - 1.0);
    for (i0 = 0; i0 <= ar; i0++) {
      b_y->data[b_y->size[0] * i0] = 1.0 + (double)i0;
    }
  }

  emxInit_real_T1(&inds, 1);
  i0 = inds->size[0];
  inds->size[0] = b_y->size[1];
  emxEnsureCapacity((emxArray__common *)inds, i0, (int)sizeof(double));
  ar = b_y->size[1];
  for (i0 = 0; i0 < ar; i0++) {
    inds->data[i0] = b_y->data[b_y->size[0] * i0];
  }

  emxFree_real_T(&b_y);

  /*  indexes for samples */
  /*  combined framing indexes */
  /*  divide the input signal into frames using indexing */
  ar = indf->size[1];
  i0 = frames->size[0] * frames->size[1];
  frames->size[0] = (int)Nw;
  frames->size[1] = ar;
  emxEnsureCapacity((emxArray__common *)frames, i0, (int)sizeof(double));
  for (i0 = 0; i0 < ar; i0++) {
    br = (int)Nw;
    for (i1 = 0; i1 < br; i1++) {
      frames->data[i1 + frames->size[0] * i0] = speech[(int)(indf->data
        [indf->size[0] * i0] + inds->data[i1]) - 1];
    }
  }

  emxFree_real_T(&inds);
  emxInit_creal_T(&x, 2);

  /*     % return if custom analysis windowing was not requested */
  /*  EOF  */
  /*  Magnitude spectrum computation (as column vectors) */
  fft(frames, absn, x);
  for (i0 = 0; i0 < 2; i0++) {
    b[i0] = x->size[i0];
  }

  emxInit_real_T(&MAG, 2);
  i0 = MAG->size[0] * MAG->size[1];
  MAG->size[0] = (int)b[0];
  MAG->size[1] = (int)b[1];
  emxEnsureCapacity((emxArray__common *)MAG, i0, (int)sizeof(double));
  ixstart = x->size[0] * x->size[1];
  for (k = 0; k + 1 <= ixstart; k++) {
    MAG->data[k] = rt_hypotd_snf(x->data[k].re, x->data[k].im);
  }

  emxFree_creal_T(&x);
  emxInit_real_T(&H, 2);
  emxInit_real_T(&b_b, 2);

  /*  Triangular filterbank with uniformly spaced filters on mel scale */
  trifbank(M, f + 1.0, R, fs, H);

  /*  size of H is M x K  */
  /*  Filterbank application to unique part of the magnitude spectrum */
  ar = (int)(absn / 2.0 + 1.0);
  br = MAG->size[1];
  i0 = b_b->size[0] * b_b->size[1];
  b_b->size[0] = ar;
  b_b->size[1] = br;
  emxEnsureCapacity((emxArray__common *)b_b, i0, (int)sizeof(double));
  for (i0 = 0; i0 < br; i0++) {
    for (i1 = 0; i1 < ar; i1++) {
      b_b->data[i1 + b_b->size[0] * i0] = MAG->data[i1 + MAG->size[0] * i0];
    }
  }

  if ((H->size[1] == 1) || ((int)(f + 1.0) == 1)) {
    i0 = FBE->size[0] * FBE->size[1];
    FBE->size[0] = H->size[0];
    FBE->size[1] = b_b->size[1];
    emxEnsureCapacity((emxArray__common *)FBE, i0, (int)sizeof(double));
    ar = H->size[0];
    for (i0 = 0; i0 < ar; i0++) {
      br = b_b->size[1];
      for (i1 = 0; i1 < br; i1++) {
        FBE->data[i0 + FBE->size[0] * i1] = 0.0;
        nx = H->size[1];
        for (ixstart = 0; ixstart < nx; ixstart++) {
          FBE->data[i0 + FBE->size[0] * i1] += H->data[i0 + H->size[0] * ixstart]
            * b_b->data[ixstart + b_b->size[0] * i1];
        }
      }
    }
  } else {
    k = H->size[1];
    i0 = MAG->size[1];
    b[0] = (unsigned int)H->size[0];
    i1 = FBE->size[0] * FBE->size[1];
    FBE->size[0] = (int)b[0];
    FBE->size[1] = i0;
    emxEnsureCapacity((emxArray__common *)FBE, i1, (int)sizeof(double));
    m = H->size[0];
    i0 = FBE->size[0] * FBE->size[1];
    emxEnsureCapacity((emxArray__common *)FBE, i0, (int)sizeof(double));
    ar = FBE->size[1];
    for (i0 = 0; i0 < ar; i0++) {
      br = FBE->size[0];
      for (i1 = 0; i1 < br; i1++) {
        FBE->data[i1 + FBE->size[0] * i0] = 0.0;
      }
    }

    if (H->size[0] == 0) {
    } else {
      i0 = MAG->size[1];
      if (i0 == 0) {
      } else {
        i0 = MAG->size[1] - 1;
        nx = H->size[0] * i0;
        ixstart = 0;
        while ((m > 0) && (ixstart <= nx)) {
          i0 = ixstart + m;
          for (ic = ixstart; ic + 1 <= i0; ic++) {
            FBE->data[ic] = 0.0;
          }

          ixstart += m;
        }

        br = 0;
        ixstart = 0;
        while ((m > 0) && (ixstart <= nx)) {
          ar = 0;
          i0 = br + k;
          for (ib = br; ib + 1 <= i0; ib++) {
            if (MAG->data[ib % (int)(f + 1.0) + MAG->size[0] * (ib / (int)(f +
                  1.0))] != 0.0) {
              ia = ar;
              i1 = ixstart + m;
              for (ic = ixstart; ic + 1 <= i1; ic++) {
                ia++;
                FBE->data[ic] += b_b->data[ib] * H->data[ia - 1];
              }
            }

            ar += m;
          }

          br += k;
          ixstart += m;
        }
      }
    }
  }

  emxFree_real_T(&H);
  emxFree_real_T(&MAG);

  /*  FBE( FBE<1.0 ) = 1.0; % apply mel floor */
  /*  DCT matrix computation */
  absn = sqrt(2.0 / M);
  i0 = mat1->size[0] * mat1->size[1];
  emxEnsureCapacity((emxArray__common *)mat1, i0, (int)sizeof(double));
  nx = mat1->size[0];
  ixstart = mat1->size[1];
  ar = nx * ixstart;
  for (i0 = 0; i0 < ar; i0++) {
    mat1->data[i0] *= mat2->data[i0];
  }

  emxFree_real_T(&mat2);
  nx = mat1->size[0] * mat1->size[1];
  for (k = 0; k + 1 <= nx; k++) {
    mat1->data[k] = cos(mat1->data[k]);
  }

  i0 = mat1->size[0] * mat1->size[1];
  emxEnsureCapacity((emxArray__common *)mat1, i0, (int)sizeof(double));
  nx = mat1->size[0];
  ixstart = mat1->size[1];
  ar = nx * ixstart;
  for (i0 = 0; i0 < ar; i0++) {
    mat1->data[i0] *= absn;
  }

  /*  Conversion of logFBEs to cepstral coefficients through DCT */
  i0 = b_b->size[0] * b_b->size[1];
  b_b->size[0] = FBE->size[0];
  b_b->size[1] = FBE->size[1];
  emxEnsureCapacity((emxArray__common *)b_b, i0, (int)sizeof(double));
  ar = FBE->size[0] * FBE->size[1];
  for (i0 = 0; i0 < ar; i0++) {
    b_b->data[i0] = FBE->data[i0];
  }

  b_log(b_b);
  if ((mat1->size[1] == 1) || (b_b->size[0] == 1)) {
    i0 = CC->size[0] * CC->size[1];
    CC->size[0] = mat1->size[0];
    CC->size[1] = b_b->size[1];
    emxEnsureCapacity((emxArray__common *)CC, i0, (int)sizeof(double));
    ar = mat1->size[0];
    for (i0 = 0; i0 < ar; i0++) {
      br = b_b->size[1];
      for (i1 = 0; i1 < br; i1++) {
        CC->data[i0 + CC->size[0] * i1] = 0.0;
        nx = mat1->size[1];
        for (ixstart = 0; ixstart < nx; ixstart++) {
          CC->data[i0 + CC->size[0] * i1] += mat1->data[i0 + mat1->size[0] *
            ixstart] * b_b->data[ixstart + b_b->size[0] * i1];
        }
      }
    }
  } else {
    k = mat1->size[1];
    b[0] = (unsigned int)mat1->size[0];
    b[1] = (unsigned int)b_b->size[1];
    i0 = CC->size[0] * CC->size[1];
    CC->size[0] = (int)b[0];
    CC->size[1] = (int)b[1];
    emxEnsureCapacity((emxArray__common *)CC, i0, (int)sizeof(double));
    m = mat1->size[0];
    i0 = CC->size[0] * CC->size[1];
    emxEnsureCapacity((emxArray__common *)CC, i0, (int)sizeof(double));
    ar = CC->size[1];
    for (i0 = 0; i0 < ar; i0++) {
      br = CC->size[0];
      for (i1 = 0; i1 < br; i1++) {
        CC->data[i1 + CC->size[0] * i0] = 0.0;
      }
    }

    if ((mat1->size[0] == 0) || (b_b->size[1] == 0)) {
    } else {
      nx = mat1->size[0] * (b_b->size[1] - 1);
      ixstart = 0;
      while ((m > 0) && (ixstart <= nx)) {
        i0 = ixstart + m;
        for (ic = ixstart; ic + 1 <= i0; ic++) {
          CC->data[ic] = 0.0;
        }

        ixstart += m;
      }

      br = 0;
      ixstart = 0;
      while ((m > 0) && (ixstart <= nx)) {
        ar = 0;
        i0 = br + k;
        for (ib = br; ib + 1 <= i0; ib++) {
          if (b_b->data[ib] != 0.0) {
            ia = ar;
            i1 = ixstart + m;
            for (ic = ixstart; ic + 1 <= i1; ic++) {
              ia++;
              CC->data[ic] += b_b->data[ib] * mat1->data[ia - 1];
            }
          }

          ar += m;
        }

        br += k;
        ixstart += m;
      }
    }
  }

  /*  Cepstral lifter computation */
  __anon_fcn(N, L, indf);

  /*  Cepstral liftering gives liftered cepstral coefficients */
  diag(indf, mat1);
  i0 = b_b->size[0] * b_b->size[1];
  b_b->size[0] = CC->size[0];
  b_b->size[1] = CC->size[1];
  emxEnsureCapacity((emxArray__common *)b_b, i0, (int)sizeof(double));
  ar = CC->size[0] * CC->size[1];
  emxFree_real_T(&indf);
  for (i0 = 0; i0 < ar; i0++) {
    b_b->data[i0] = CC->data[i0];
  }

  emxInit_real_T(&b_mat1, 2);
  if ((mat1->size[1] == 1) || (CC->size[0] == 1)) {
    i0 = b_mat1->size[0] * b_mat1->size[1];
    b_mat1->size[0] = mat1->size[0];
    b_mat1->size[1] = CC->size[1];
    emxEnsureCapacity((emxArray__common *)b_mat1, i0, (int)sizeof(double));
    ar = mat1->size[0];
    for (i0 = 0; i0 < ar; i0++) {
      br = CC->size[1];
      for (i1 = 0; i1 < br; i1++) {
        b_mat1->data[i0 + b_mat1->size[0] * i1] = 0.0;
        nx = mat1->size[1];
        for (ixstart = 0; ixstart < nx; ixstart++) {
          b_mat1->data[i0 + b_mat1->size[0] * i1] += mat1->data[i0 + mat1->size
            [0] * ixstart] * CC->data[ixstart + CC->size[0] * i1];
        }
      }
    }

    i0 = CC->size[0] * CC->size[1];
    CC->size[0] = b_mat1->size[0];
    CC->size[1] = b_mat1->size[1];
    emxEnsureCapacity((emxArray__common *)CC, i0, (int)sizeof(double));
    ar = b_mat1->size[1];
    for (i0 = 0; i0 < ar; i0++) {
      br = b_mat1->size[0];
      for (i1 = 0; i1 < br; i1++) {
        CC->data[i1 + CC->size[0] * i0] = b_mat1->data[i1 + b_mat1->size[0] * i0];
      }
    }
  } else {
    k = mat1->size[1];
    b[0] = (unsigned int)mat1->size[0];
    b[1] = (unsigned int)CC->size[1];
    i0 = CC->size[0] * CC->size[1];
    CC->size[0] = (int)b[0];
    CC->size[1] = (int)b[1];
    emxEnsureCapacity((emxArray__common *)CC, i0, (int)sizeof(double));
    m = mat1->size[0];
    i0 = CC->size[0] * CC->size[1];
    emxEnsureCapacity((emxArray__common *)CC, i0, (int)sizeof(double));
    ar = CC->size[1];
    for (i0 = 0; i0 < ar; i0++) {
      br = CC->size[0];
      for (i1 = 0; i1 < br; i1++) {
        CC->data[i1 + CC->size[0] * i0] = 0.0;
      }
    }

    if ((mat1->size[0] == 0) || (b_b->size[1] == 0)) {
    } else {
      nx = mat1->size[0] * (b_b->size[1] - 1);
      ixstart = 0;
      while ((m > 0) && (ixstart <= nx)) {
        i0 = ixstart + m;
        for (ic = ixstart; ic + 1 <= i0; ic++) {
          CC->data[ic] = 0.0;
        }

        ixstart += m;
      }

      br = 0;
      ixstart = 0;
      while ((m > 0) && (ixstart <= nx)) {
        ar = 0;
        i0 = br + k;
        for (ib = br; ib + 1 <= i0; ib++) {
          if (b_b->data[ib] != 0.0) {
            ia = ar;
            i1 = ixstart + m;
            for (ic = ixstart; ic + 1 <= i1; ic++) {
              ia++;
              CC->data[ic] += b_b->data[ib] * mat1->data[ia - 1];
            }
          }

          ar += m;
        }

        br += k;
        ixstart += m;
      }
    }
  }

  emxFree_real_T(&b_mat1);
  emxFree_real_T(&b_b);
  emxFree_real_T(&mat1);

  /*  ~ HTK's MFCCs */
  /*  EOF */
}

/*
 * File trailer for mfcc.c
 *
 * [EOF]
 */
