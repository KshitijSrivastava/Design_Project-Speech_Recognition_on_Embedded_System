#include "main.h"
#include "stm32f4xx_hal.h"
#include "mfccFunc.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "constants.h"

#define q	9		/* for 2^3 points */
#define N	(1<<q)		/* N-point FFT, iFFT */

typedef struct {
 float r;
 float i;
} complex_float;

void fft(complex_float *v ,int n ,complex_float *tmp );

static const float pi = 3.1415926;


int mfccFunc(float*speech,float*mfcc){ // speech[4000] , mfcc[24*13]


	for(int f = 0; f<24; f++){//f<24
		float frame[512];

		for(int k = 0; k < 512; k++){
			if(k < 320){
				int index = k+f*160;
				frame[k] = speech[index]*hamming[k];
			}else{
				frame[k] = 0;
			}
		}

		complex_float in[512], out[512];

		for(int i = 0; i < 512; i++){
			
				in[i].r = frame[i];
				in[i].i = 0;
			
				out[i].r = 0;
				out[i].i = 0;
		}

		fft( in, N, out);


		//FTT DONE!
		for(int o = 0; o < 512; o++)
		{
			float a = in[o].r;
			float b = in[o].i;
			a = a*a;
			b = b*b;
			float c = a+b;
			frame[o] = c; //MAG
		}


		//Calculate FBE
		float FBE[25];
		for(int i = 0; i < 25; i++){
			float tempMult = 0;
			for(int j = 0; j < 257; j++){
				tempMult = tempMult + frame[j]*H[i][j];
			}
			if(tempMult < 1){
				tempMult = 1;
			}
			FBE[i] = tempMult;
		}

		//Calculate CC
		float CC[13];
		for(int i = 0; i < 13; i++){
			//CC = DCT * log(FBE); 13x1 result
			float tempMult = 0;
			for(int j = 0; j < 25; j++){
					float a = logf(FBE[j]);
					tempMult = tempMult + a*DCT[i][j];
			}
			CC[i] = tempMult*CC_Weights[i];
			mfcc[f*13+i] = CC[i];
		}




	}


	return 0;
}


/* 
   fft(v,N):
   [0] If N==1 then return.
   [1] For k = 0 to N/2-1, let ve[k] = v[2*k]
   [2] Compute fft(ve, N/2);
   [3] For k = 0 to N/2-1, let vo[k] = v[2*k+1]
   [4] Compute fft(vo, N/2);
   [5] For m = 0 to N/2-1, do [6] through [9]
   [6]   Let w.r = cos(2*PI*m/N)
   [7]   Let w.i = -sin(2*PI*m/N)
   [8]   Let v[m] = ve[m] + w*vo[m]
   [9]   Let v[m+N/2] = ve[m] - w*vo[m]
 */
void fft(complex_float *v ,int n ,complex_float *tmp )
{
  if(n>1) {			/* otherwise, do nothing and return */
    int k,m;    complex_float z, w, *vo, *ve;
    ve = tmp; vo = tmp+n/2;
    for(k=0; k<n/2; k++) {
      ve[k] = v[2*k];
      vo[k] = v[2*k+1];
    }
    fft( ve, n/2, v );		/* FFT on even-indexed elements of v[] */
    fft( vo, n/2, v );		/* FFT on odd-indexed elements of v[] */
    for(m=0; m<n/2; m++) {
      w.r = cos(2*pi*m/(double)n);
      w.i = -sin(2*pi*m/(double)n);
      z.r = w.r*vo[m].r - w.i*vo[m].i;	/* r(w*vo[m]) */
      z.i = w.r*vo[m].i + w.i*vo[m].r;	/* Im(w*vo[m]) */
      v[  m  ].r = ve[m].r + z.r;
      v[  m  ].i = ve[m].i + z.i;
      v[m+n/2].r = ve[m].r - z.r;
      v[m+n/2].i = ve[m].i - z.i;
    }
  }
return;
}
 
