// Complied with Matlab 2016, TDM-GCC-64 (MinGW64, GCC 5.1.0), OpenMP

#include "mex.h"

#include "CASEntropyBasedMinimization.h"
#include "CASMeasureVolume.h"
#include "CASMeasureEntropy.h"

#define BIGNUM 1000000

#define SETTINGS_FIELD_VERBOSE        "verbose"
#define SETTINGS_FIELD_SOLVER         "solver"
#define SETTINGS_MODEL                "model"
#define SETTINGS_METRIC               "metric"
#define SETTINGS_BIN_SIZE             "bin_size"
#define SETTINGS_N_NEIGHB             "n_neighb"
#define SETTINGS_MAX_ITER             "max_iter"

#define SETTINGS_MODEL_CLOUD          "model_cloud"


#define INPUT_CLOUD          0
#define INPUT_PARAMS         1
#define INPUT_BAUNDS         2
#define INPUT_SETTINGS       3

#define RET_PARAMS           0
#define RET_BINS             1
#define RET_MODEL_CLOUD      2
#define RET_ITER             3

using namespace cas;

class Logger : public CASEntropyBasedMinimizationLogger
{
    double* ret_iter;
    bool verbose;
    int max_iter;
    
    public:

        Logger(double* ret_iter, bool verbose, int max_iter)
        {
            this->ret_iter = ret_iter;
            this->verbose = verbose;
            this->max_iter;
        }

        void IterationLog(const CASMatrix& params, const double ebest, const int iter_num)
        {     
            // Iteration output
            if(verbose)
                mexPrintf("%i. iteration (%.4f)\n%s\n", iter_num, ebest, params.ToString());    
            for (int i=0; i<params.GetM(); i++)
                GET(ret_iter, iter_num, i, max_iter) = params(0,i); 
        }
};
    
/* The gateway function */
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[])
{
    
    /* Check for proper number of input and output arguments */    
    if (nrhs < 3) 
    {
        mexErrMsgIdAndTxt( "MATLAB:mexgetarray:minrhs", "Three inputs are required!");
    }     

    /*
     * Default parameters
     */
    int verbose       = 1;
    double bin_size   = 1;
    double start_bin_size   = 1;
    const char* model = "CV2DoF";
    const char* metric = "volume";
    int n_neighb = 10;
    int max_iter = 10;      
    
    //bool has_model_cloud = false;    
    //int n_model_cloud = 1;        
    //mxArray* model_cloud_arr  = mxCreateDoubleMatrix((mwSize)(1), (mwSize)nh, mxREAL);
    //double* model_cloud = NULL;
       
    /*
     * Cloud: X,Y,Z,T 
     */
    plhs[RET_MODEL_CLOUD] = mxDuplicateArray(prhs[INPUT_CLOUD]);
    CASShallowMatrix cloud_mat(mxGetPr(plhs[RET_MODEL_CLOUD]), mxGetM(plhs[RET_MODEL_CLOUD]), mxGetN(plhs[RET_MODEL_CLOUD]));    
    CASPointCloud cloud(cloud_mat); 
    
    /*
     * Parameters: VX, VY,...
     */
    CASMatrix params(mxGetPr(prhs[INPUT_PARAMS]), mxGetM(prhs[INPUT_PARAMS]), mxGetN(prhs[INPUT_PARAMS]));

    /*
     * Parameters: VX_BAUND, VY_BAUND,...
     */
    CASMatrix params_baund(mxGetPr(prhs[INPUT_BAUNDS]), mxGetM(prhs[INPUT_BAUNDS]), mxGetN(prhs[INPUT_BAUNDS]));
    
    /*
     * Settings
     */
    if (!mxIsStruct(prhs[INPUT_SETTINGS]))
    {
        mexErrMsgTxt("The third input parameter has to be settings structure!");
        return;
    }
    const mxArray* settings = prhs[INPUT_SETTINGS];
            
    /*
     * VERBOSE = 0 - no results on the output
     * VERBOSE = 1 - put results to the output
     */
    GET_FROM_SETTINGS(SETTINGS_FIELD_VERBOSE, verbose, mxGetScalar, int);
    
        
    /*
     * BIN_SIZE = 1
     */
    GET_FROM_SETTINGS(SETTINGS_BIN_SIZE, bin_size, mxGetScalar, double);
    start_bin_size = bin_size;
    
    /*
     * MODEL = ''
     */
    GET_FROM_SETTINGS(SETTINGS_MODEL, model, mxArrayToString, char*);
    
    
    /*
     * N_NEIGHB = 100
     */
    GET_FROM_SETTINGS(SETTINGS_N_NEIGHB, n_neighb, mxGetScalar, int);
    
    /*
     * MAX_ITER = 50
     */
    GET_FROM_SETTINGS(SETTINGS_MAX_ITER, max_iter, mxGetScalar, int);
    
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
    
        
    /*
     * MODEL_CLOUD
     */
    /*GET_FROM_SETTINGS(SETTINGS_MODEL_CLOUD, model_cloud_arr, (void*), mxArray*);
    n_model_cloud = mxGetM(model_cloud_arr);
    if (n_model_cloud != 1)
    {
        model_cloud = mxGetPr(model_cloud_arr);
        has_model_cloud = true;
    }*/

    // Display settings
    if (verbose)
    {
        mexPrintf("Settings\n");
        mexPrintf("--------------------\n");
        mexPrintf("verbose         :  %s\n", verbose == 1 ? "TRUE" : "FALSE");
        mexPrintf("model           :  %s\n", model); 
        mexPrintf("metric          :  %s\n", measure == NULL ? "none" : metric);        
        mexPrintf("bin_size        :  %.2f\n", bin_size);
        mexPrintf("n_neighb        :  %i\n", n_neighb);
        mexPrintf("max_iter        :  %i\n", max_iter);
        mexPrintf("Parameters      : %s\n", params.ToString());
        mexPrintf("Par. baunds     : %s\n", params_baund.ToString());
        //mexPrintf("has_model_cloud :  %s\n", has_model_cloud == true ? "TRUE" : "FALSE");     
        mexPrintf("\n");
    }
    
//     // Calculate initial bins
//     if (has_model_cloud)
//     {
//         calculateBins(model_cloud, n_model_cloud, bin_size, bins_init);
//         
//         if(verbose)
//         {
//             mexPrintf("model cloud size        :  %i\n", n_model_cloud);
//             mexPrintf("model cloud bin size    :  %i\n", bins_init.size());
//         }
//     }
    
    char msg[100];
    CASAbstractReconstruction* reconstruction = CASReconstructionFactory::Create(model, params.GetM(), msg);
    if (reconstruction == NULL)
    {
        mexErrMsgIdAndTxt( "MATLAB:mexgetarray:minrhs", msg);      
    }   
    
    plhs[RET_ITER] = mxCreateDoubleMatrix((mwSize)(max_iter), (mwSize)params.GetM(), mxREAL);
    double* ret_iter = mxGetPr(plhs[RET_ITER]);
    Logger* logger = new Logger(ret_iter, verbose, max_iter);
    
    // Run the minimization algorithm
    CASMatrix best_params = CASEntropyBasedMinimization::Run(cloud, reconstruction, measure, params, params_baund, bin_size, n_neighb, max_iter, logger);
                
    // Update with the best 
    reconstruction->Reconstruct(cloud, best_params, true, measure);
    double energy = measure->Calculate();
    
    // Save estimated parameters    
    plhs[RET_PARAMS] = mxCreateDoubleMatrix((mwSize)(1), (mwSize)best_params.GetM(), mxREAL);
    double* ret_params = mxGetPr(plhs[RET_PARAMS]);
    for (int i=0; i<best_params.GetM(); i++)
            ret_params[i] = best_params(0,i);     
    
    // Energy
    plhs[RET_BINS] = mxCreateDoubleScalar(energy);
    
    if (measure != NULL)
        delete measure;
    if (reconstruction != NULL)
        delete reconstruction;
    if (logger != NULL)
        delete logger;
            
}
