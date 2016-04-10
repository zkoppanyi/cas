#include "mex.h"
#include "matrix.h"

#define _USE_MATH_DEFINES 
#include <math.h> 
#include <string.h>

#include "util.h"

#include "Bin.h" 
#include "COG.h" 
#include "CASAbstractMatrix.h" 
#include "CASMatrix.h" 
#include "CASShallowMatrix.h" 
#include "CASAbstractMeasure.h" 
#include "CASMeasureEntropy.h" 
#include "CASMeasureVolume.h" 

#include "CASReconstructionCV3DoF.h" 
#include "CASReconstructionCA2DoF.h" 
#include "CASReconstructionCA3DoF.h" 
#include "CASReconstructionGeneral.h" 
#include "CASReconstructionFactory.h" 

#define SETTINGS_FIELD_VERBOSE        "verbose"
#define SETTINGS_BIN_SIZE             "bin_size"
#define SETTINGS_MODEL                "model"
#define SETTINGS_METRIC               "metric"

#define RET_BINS             0
#define RET_MODEL_CLOUD      1

using namespace cas;

/* The gateway function */
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[])
{

    /*
     * Default parameters
     */
    int    verbose    = 1;
    double bin_size   = 100;  
    const char*  model = "general"; 
    const char*  metric = NULL;
    
   /* Check for proper number of input and output arguments */    
    if (nrhs < 3) {
        mexErrMsgIdAndTxt( "MATLAB:mexgetarray:minrhs",
                "Three inputs are required!");
    } 
    
    /*
     * Cloud: X,Y,Z,T
     */
    plhs[RET_MODEL_CLOUD] = mxDuplicateArray(prhs[0]);    
    CASShallowMatrix cloud_mat(mxGetPr(plhs[RET_MODEL_CLOUD]), mxGetM(plhs[RET_MODEL_CLOUD]), mxGetN(plhs[RET_MODEL_CLOUD]));    
    CASPointCloud cloud(cloud_mat);
    
    if (cloud.Points.GetM() < 4) {
        mexErrMsgIdAndTxt( "MATLAB:mexgetarray:minrhs",
                "Cloud matrix has to contain at least 4 columns: X Y Z T!");
    } 
    
    const mxArray* tv = prhs[1];
    CASMatrix params(mxGetPr(tv), mxGetM(tv), mxGetN(tv));
    
    if (params.GetN()  < 1) {
        mexErrMsgIdAndTxt( "MATLAB:mexgetarray:minrhs",
                "Velocities matrix is empty!");
    } 
    
    if (!mxIsStruct(prhs[2]))
    {
        mexErrMsgTxt("The second input parameter has to be settings structure!");
        return;
    }
    const mxArray* settings = prhs[2];
            
    /*
     * VERBOSE = 0 - no results on the output
     * VERBOSE = 1 - put results to the output
     */
    GET_FROM_SETTINGS(SETTINGS_FIELD_VERBOSE, verbose, mxGetScalar, int);
            
    /*
     * BIN_SIZE = 1
     */
    GET_FROM_SETTINGS(SETTINGS_BIN_SIZE, bin_size, mxGetScalar, double);

    /*
     * MODEL = 'general'
     */
    GET_FROM_SETTINGS(SETTINGS_MODEL, model, mxArrayToString, char*);    
    
    /*
     * METRIC = 'volume'
     */
    GET_FROM_SETTINGS(SETTINGS_METRIC, metric, mxArrayToString, char*);   
        
    CASAbstractMeasure* measure = NULL;    
    if (metric != NULL)
    {
        if (strcmp(metric, "entropy") == 0)
            measure = new CASMeasureEntropy(bin_size);    
        if (strcmp(metric, "volume") == 0)
            measure = new CASMeasureVolume(bin_size);
    }
    
    // Display settings
    if (verbose)
    {
        mexPrintf("Settings\n");
        mexPrintf("--------------------\n");
        mexPrintf("Model         :  %s\n", model);
        mexPrintf("Metric        :  %s\n", measure == NULL ? "none" : metric);
        mexPrintf("Verbose       :  %s\n", verbose == 1 ? "TRUE" : "FALSE");
        mexPrintf("Cloud size    :  %i\n", cloud.GetPointNumber());
        mexPrintf("Bin size      :  %.2f\n", bin_size);
        mexPrintf("Velocity vector: %s\n", params.ToString());
        mexPrintf("\n");
    }
    
    char err_msg[50];
    CASAbstractReconstruction* reconstruction = CASReconstructionFactory::Create(model, params.GetM(), err_msg);
    if (reconstruction == NULL)
    {
        mexErrMsgIdAndTxt( "MATLAB:mexgetarray:minrhs", err_msg);      
    }
    
    reconstruction->Reconstruct(cloud, params, true, measure);
    
    double mvalue = 0;
    if (measure != NULL)
        mvalue = measure->Calculate();
    
    if (verbose)
        mexPrintf("Energy: %.2f\n", mvalue);
        
    plhs[RET_BINS] = mxCreateDoubleScalar(mvalue);    
    
    if (reconstruction != NULL)
        delete reconstruction;     
    if (measure != NULL)
        delete measure;
}
