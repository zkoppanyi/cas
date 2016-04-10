#ifndef CASENTROPYBASEDMIN_H
#define CASENTROPYBASEDMIN_H

#include <omp.h>
#include "util.h"
#include "Bin.h"
#include "COG.h"

#include "CASMatrix.h"
#include "CASPointCloud.h"
#include "CASAbstractReconstruction.h"
#include "CASReconstructionFactory.h"
#include "CASAbstractMeasure.h"

namespace cas
{
    class CASEntropyBasedMinimizationLogger
    {
        public:
            virtual void IterationLog(const CASMatrix& params, const double ebest, const int iter_num) {};
    };
    
    class CASEntropyBasedMinimization
    {
        public:            
            CASEntropyBasedMinimization();
            static CASMatrix Run(CASPointCloud& cloud, CASAbstractReconstruction* reconstruction, CASAbstractMeasure* measure, 
                                            const CASMatrix& params, const CASMatrix& params_baund, double bin_size = 1, int n_neighb = 100, int max_iter = 100,
                                            CASEntropyBasedMinimizationLogger* logger = NULL);
            
        private: 
            inline static void neighbourhood_calculation(int n_neighb, CASPointCloud& cloud, CASAbstractReconstruction* reconstruction, CASAbstractMeasure* measure,
                                                  const CASMatrix& rad_params, const CASMatrix& best_params, CASMatrix& best_cand_params, double& ebest_cand);
            

    };
        
            
    inline void CASEntropyBasedMinimization::neighbourhood_calculation(int n_neighb, CASPointCloud& cloud, CASAbstractReconstruction* reconstruction, CASAbstractMeasure* measure,
                                                                     const CASMatrix& rad_params, const CASMatrix& best_params, CASMatrix& best_cand_params, double& ebest_cand)
    {
        CASMatrix cand_params(rad_params.GetN(), rad_params.GetM());

        //#pragma omp parallel for shared(cloud, vx_rad, vy_rad, vz_rad, ax_rad, ay_rad, az_rad, h_rad, best_vx, best_vy, best_vz, best_ax, best_ay, best_az, best_h, best_cand_vx, best_cand_vx, best_cand_vz, best_cand_ax, best_cand_ay, best_cand_az, best_cand_h, ebest_cand) private(k) 
        //#pragma omp parallel for 
        for (int k = 0; k < n_neighb; k++)
        {
            Bins  bins;   
            COG pCOG;

            for (int i=0; i<cand_params.GetM(); i++)
                cand_params(0,i) = best_params(0,i) + ((rand() / (double)RAND_MAX) - 0.5) * rad_params(0,i) * 2;

            measure->Reset();
            reconstruction->Reconstruct(cloud, cand_params, false, measure);
            double enew = measure->Calculate();

            if ( (ebest_cand > enew) | (exp(-(enew-ebest_cand)) > (rand() / (double)RAND_MAX)) )
            //if ( (ebest_cand > enew) )
            {
                ebest_cand = enew;
                for (int i=0;i<cand_params.GetM();i++)
                    best_cand_params(0,i) = cand_params(0,i);
            }              
        }
    }
    
    CASMatrix CASEntropyBasedMinimization::Run(CASPointCloud& cloud, CASAbstractReconstruction* reconstruction, CASAbstractMeasure* measure, 
                                            const CASMatrix& params, const CASMatrix& params_baund, double bin_size, int n_neighb, int max_iter,
                                            CASEntropyBasedMinimizationLogger* logger)
    {
        int nbins;
        Bins bins_init;
        COG pCOG;    

        CASMatrix best_params(params);
        double ebest = BIGNUM;
        int iter_num = 0;

        double temp = BIGNUM;
        while ( (iter_num < max_iter) && (temp > 0.001))
        {
            // Linear cooling
            // double temp = 1 - pow( ((double)iter_num/(double)max_iter), 2) ;

            // Geometric cooling
            //double alpha = 0.92;
            double alpha = 0.92;
            temp = pow(alpha, (double)iter_num);

            // Exponential cooling
            //double alpha = 0.85 ;
            //double temp = alpha / (log(1+(double)iter_num));

            // Logarthmic cooling
            //double alpha = 1;
           // double temp = exp(-alpha * pow((double)iter_num, 1/((double)max_iter)));

            CASMatrix rad_params(best_params.GetN(), best_params.GetM());
            for (int i=0; i<rad_params.GetM(); i++)
                rad_params(0,i) = params_baund(0,i)*temp;

            double ebest_cand = ebest;
            CASMatrix best_cand_params(best_params.GetN(), best_params.GetM());

            neighbourhood_calculation(n_neighb, cloud, reconstruction, measure, rad_params, best_params, best_cand_params, ebest_cand);        

            if (ebest_cand < ebest)
            {
                for (int i=0; i<best_params.GetM(); i++)
                    best_params(0,i) = best_cand_params(0,i);
                ebest = ebest_cand;
            };

            // Logger
            if (logger!= NULL)
                logger->IterationLog(best_params, ebest, iter_num);
            
            iter_num++;
        }
        
        return best_params;
    }
    
    
}

#endif