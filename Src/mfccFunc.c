#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "kiss_fft.h"
#include "constants.h"


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
//malloc Hamming array

float* hamming = malloc(320*sizeof(float));
for(int i = 0; i < 320;i++){
	hamming[i] = 0.54 - 0.46*cos(2*pi*i/319);
}
	printf("HERE: %f\n",speech[6]);
float* frame = malloc(512*sizeof(float));
for(int f = 0; f<23; f++){//f=23
	printf("loop: %d\n",f);
	for(int k = 0; k < 320; k++){
		frame[k] = speech[k+f*160]*hamming[k];
	}
	kiss_fft_cpx in[512], out[512];

	for(int i = 0; i < 512; i++){
		if(i<320){
			in[i].r = frame[i];
		}else{
			in[i].r = 0;
		}
		in[i].i = 0;
	}
	kiss_fft_cfg cfg;

 	if ((cfg = kiss_fft_alloc(512, 0/*is_inverse_fft*/, NULL, NULL)) != NULL)
 	{
    	kiss_fft(cfg, in, out);
    	free(cfg);
  	}else
  	{
    return 1;
	}

	for(int o = 0; o < 512; o++)
	{
		float a = out[o].r;
		float b = out[o].i;
		a = a*a;
		b = b*b;
		float c = a+b;
		frame[o] = sqrt(c); //MAG
	}
printf("frame: %f\n",frame[3]);
	printf("frame: %f\n",frame[4]);
	printf("frame: %f\n",frame[5]);
	printf("frame: %f\n",frame[6]);
	//Calculate FBE
	float* FBE = malloc(25*sizeof(float));
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
	float* CC = malloc(13*sizeof(float));
	for(int i = 0; i < 13; i++){
		//CC = DCT * log(FBE); 13x1 result
		float tempMult = 0;
		for(int j = 0; j < 25; j++){
			float a = logf(FBE[j]);
			tempMult = tempMult + a*DCT[i][j];
		}
		CC[i] = tempMult*CC_Weights[i];
		//mfcc[f][i] = H[i];
		mfcc[f+i] = 22; //temp input for testing...
	}



	free(FBE);
	free(CC);
}

free(frame);
free(hamming);
return 0;
}


