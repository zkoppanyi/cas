#include "mex.h"

#include "CASMeasureBinNum.h" 
#include "CASMeasureEntropy.h" 
#include "CASShallowMatrix.h" 
#include "CASPointCloud.h" 

using namespace cas;

/* The gateway function */
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[])
{

    if (nrhs < 4) {
        mexErrMsgIdAndTxt( "MATLAB:mexgetarray:minrhs",
                "4 inputs are required!");
    } 
        
    CASShallowMatrix cloud_mat(mxGetPr(prhs[0]), mxGetM(prhs[0]), mxGetN(prhs[0]));    
    CASPointCloud cloud(cloud_mat);
    
    double bin_size = mxGetScalar(prhs[1]);    
    double mode = mxGetScalar(prhs[2]);
    double measure_type = mxGetScalar(prhs[3]);
    
    mexPrintf("Cloud size: %i\n", cloud.GetPointNumber());

    CASAbstractBinMeasure* measure = new CASMeasureEntropy(bin_size);
    if (measure_type == 1)
    {
        measure = new CASMeasureBinNum(bin_size);
    }
    
    cloud.CalculateMeasure(measure);
    double value = measure->Calculate();
    int nbins = measure->bins.size();
    
    mexPrintf("Measure: %.3f\n", value);
    mexPrintf("Bin numbers: %i\n", nbins);
    
    if(mode == 0)
    {
        // Return just with measure value
        plhs[0] = mxCreateDoubleMatrix((mwSize)(1), (mwSize)1, mxREAL);
        double* ret= mxGetPr(plhs[0]);
        ret[0] = value;
        return;
    }
    else
    {
        // Populate return matrix
        plhs[0] = mxCreateDoubleMatrix((mwSize)(nbins), (mwSize)4, mxREAL);
        double* ret = mxGetPr(plhs[0]);

        int i = 0;
        for(Bins::iterator iter = measure->bins.begin(); iter != measure->bins.end(); ++iter)
        {
            Bin k =  iter->first;
            int v = iter->second;
            
            GET(ret, i, 0, nbins) = k.x*bin_size;
            GET(ret, i, 1, nbins) = k.y*bin_size;
            GET(ret, i, 2, nbins) = k.z*bin_size;
            GET(ret, i, 3, nbins) = v;
            i++;
        }
        
    }
    
    delete measure;
            
}
