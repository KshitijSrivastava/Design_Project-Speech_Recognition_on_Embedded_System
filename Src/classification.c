#include "main.h"
#include "stm32f4xx_hal.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "classification_const.h"



int classification(float*mfcc,float*results){
	
	
	//printf("array a: %f\n", a[30]);
	double n1[10], a1[10], n2[10];
	double max = 0, sum = 0;
	
	//preprocessing done on input vector
	for( int i=0; i<312; i++) {
		mfcc[i] = mfcc[i] - offset[i];
		mfcc[i] = mfcc[i] * gain[i];
		mfcc[i] = mfcc[i] - 1;
		
		//input layer calculation - intermediate n1 result
		for( int j=0; j<10; j++) {
			n1[j] += a[j][i] * mfcc[i]; 	
		}
	}
		
	//input layer calculation n1, a1
	for( int i=0; i<10; i++) {
		n1[i] += b1[i];
		a1[i] = -2/(1 + exp(-2*n1[i])) -1;
	}
		printf("HARDFAULT 1)");
	//layer 2 calculation
	for( int i=0; i<10; i++) 
	{
		for(int j=0; j<10; j++)
		{
			n2[i] += ( b_a[i][j] * a1[j] );
		}
		n2[i] += b2[i];
		
		//Find max
		if(n2[i] > max)
			max = n2[i];
	}
	printf("HARDFAULT 2)");
	
	//Post processing
	for( int i=0; i<10; i++)
	{
			n2[i] = n2[i] - max;
			n2[i] = exp(n2[i]);
		
			sum += n2[i];
	}
	
	if (sum==0)
		sum = 1;
	
	//Classification
	for( int i=0; i<10; i++) {
		results[i] = n2[i]/sum;
	}
	
	return 1;	
}
