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
	float n1[10], a1[10], n2[10];
	float max = 0, sum = 0;
	
	
	for( int j=0; j<10; j++) {
		for( int i=0; i<312; i++){
			if(j == 0){
					mfcc[i] = mfcc[i] - offset[i];
					mfcc[i] = mfcc[i] * gain[i];
					mfcc[i] = mfcc[i] - 1;
			}
			n1[j] += a[j][i] * mfcc[i];
		}
	}
		
	//input layer calculation n1, a1
	for( int i=0; i<10; i++) {
		n1[i] += b1[i];
		printf("n1[%d]: %f\n",i,n1[i]);
		
		printf("exp(-2*n1[%d])= %f\n",i, exp(-2*n1[i]));
		
		a1[i] = 2/(1 + exp(-2*n1[i])) -1;
		printf("al[%d]: %f\n",i,a1[i]);
	}
	
	
		printf("HARDFAULT 1)");
	//layer 2 calculation
	
	
	for( int i=0; i<10; i++) 
	{
		for(int j=0; j<10; j++)
		{
			printf("b_a[%d][%d] * a1[%d] : %f   +   ",i,j,i,b_a[i][j] * a1[j]);
			n2[i] += ( b_a[i][j] * a1[j] );
		}
		printf("\n");
		printf("n2[%d]: %f\n",i,n2[i]);
		printf("b2[%d]: %f\n",i,b2[i]);
		n2[i] += b2[i];
		printf("n2[%d]: %f\n",i,n2[i]);
		//Find max
		if(n2[i] > max)
			max = n2[i];
	}
	
	printf("MAX: %f\n", max);
	
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
