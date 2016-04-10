#include "mex.h"

#define _USE_MATH_DEFINES 
#include <math.h>
#include <stdio.h>
#include <omp.h>


double spawn_threads(double x[],int cols)
{
	double sum=0;
	int count;
    
    #pragma omp parallel shared(x, cols) private(count) reduction(+: sum) 
	for(count = 0;count<cols;count++)
	{        
	     sum += sin(x[count]*x[count]*x[count])/cos(x[count]+1.0);;
	}
	
	return sum;
}

void *PrintHello(void *threadid)
{
   long tid = 0;
   //tid = (long)threadid;
   //printf("Hello World! Thread ID %i\n", tid);
   pthread_exit(NULL);
}



void mexFunction(int nlhs,mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
	double sum; 	
	 /* Check for proper number of arguments. */
 	if(nrhs!=1) {
    	mexErrMsgTxt("One input required.");
  	} else if(nlhs>1) {
    	mexErrMsgTxt("Too many output arguments.");
  	}
	
	/*Find number of rows and columns in input data*/
	int rows = mxGetM(prhs[0]);
	int cols = mxGetN(prhs[0]);		

	/*I'm only going to consider row matrices in this code so ensure that rows isn't more than 1*/
	if(rows>1){
	mexErrMsgTxt("Input data has too many Rows.  Just the one please");
	}
		
	/*Get pointer to input data*/
	double* x = mxGetPr(prhs[0]);

    sum = spawn_threads(x,cols);
   
	
	/*create space for output*/
 	 plhs[0] = mxCreateDoubleMatrix(1,1, mxREAL);

	/*Get pointer to output array*/
	double* y = mxGetPr(plhs[0]);

	/*Return sum to output array*/
	y[0]=sum;
   
}

