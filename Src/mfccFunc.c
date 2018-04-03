#include "main.h"
#include "stm32f4xx_hal.h"
#include "mfccFunc.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "arm_math.h"
#include "arm_const_structs.h"
#include "constants.h"


typedef struct {
 float r;
 float i;
} complex_float;

static const float pi = 3.1415926;

//float mfcc[24][13];

void filter (float*buffer)
{

	for(int i = 0; i < 4000; i++){
		//buffer[i] = 32768*(buffer[i]);
		buffer[i] = 32768*buffer[i];
	}
	for(int i = 1; i < 4000; i++){
		//buffer[i] = 32768*(buffer[i]);
		buffer[i] = buffer[i] - 0.95*buffer[i-1];
	}
}

int mfccFunc(float*speech,float*mfcc){ // speech[4000] , mfcc[24][13]
//Pre_emph filter
filter(speech);
printf("filter: %f\n",speech[3]);
	printf("filter: %f\n",speech[4]);
	printf("filter: %f\n",speech[5]);
	printf("filter: %f\n",speech[6]);


float input[2*512];
float frame[512];

for(int f = 0; f<24; f++){//f=23
	printf("loop: %d\n",f);
	for(int k = 0; k < 2*512; k++){
		if(k < 320){
		int index = k+f*160;
		input[k] = speech[index]*hamming[k];
		}else{
			input[k] = 0;
		}
//		printf("speech: %f\n",speech[index]);
//		printf("hamming: %f\n",hamming[k]);
//		printf("S * F: %f\n",speech[index]*hamming[k]);
//		printf("input: %f\n",input[k]);
	}
	printf("inputH: %f\n",input[4]);
	printf("inputH: %f\n",input[5]);
	printf("inputH: %f\n",input[6]);
//	complex_float in[512], out[512];
//	for(int i = 0; i < 512; i++){
//		if(i<320){
//			in[i].r = frame[i];
//		}else{
//			in[i].r = 0;
//		}
//		in[i].i = 0;
//	}
	
	//FFT here

	arm_cfft_f32(&arm_cfft_sR_f32_len512,input,0,1);
	printf("inputCFFT: %f\n",input[4]);
	printf("inputCFFT: %f\n",input[5]);
	printf("inputCFFT: %f\n",input[6]);
	arm_cmplx_mag_f32(input,frame,512);

	if(f == 0){
		UART_Transmit_F(frame,512);
	}
//	for(int o = 0; o < 512; o++)
//	{
//		float a = in[o].r;
//		float b = in[o].i;
//		a = a*a;
//		b = b*b;
//		float c = a+b;
//		frame[o] = sqrt(c); //MAG
//	}
//	printf("in[5].r: %f\n",in[5].r);
//	printf("in[5].i: %f\n",in[5].i);
//	printf("in[10].r: %f\n",in[10].r);
//	printf("in[10].i: %f\n",in[10].i);

	printf("frame: %f\n",frame[3]);
	printf("frame: %f\n",frame[4]);
	printf("frame: %f\n",frame[5]);
	printf("frame: %f\n",frame[6]);
	//Calculate FBE
	float FBE[25];
	for(int i = 0; i < 25; i++){
		//FBE = H * frame[0:256]; 25x1 result
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
		//mfcc[f+i] = 22; //temp input for testing...
	}



//	free(FBE);
//	free(CC);
}

//free(frame);
//free(hamming);
return 0;
}

