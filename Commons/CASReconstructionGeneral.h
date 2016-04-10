#ifndef CASRECONSTRUCTIONGENERAL_H
#define CASRECONSTRUCTIONGENERAL_H

#include "CASAbstractReconstruction.h"

namespace cas
{
    class CASReconstructionGeneral : public CASAbstractReconstruction
    {
        public:        
            inline void Reconstruct(CASPointCloud& cloud, CASAbstractMatrix& params, bool isModifyCloud, CASAbstractMeasure* measure=NULL, COG* pcog=NULL);
    };     
    
    inline void CASReconstructionGeneral::Reconstruct(CASPointCloud& cloud, CASAbstractMatrix& params, bool isModifyCloud, CASAbstractMeasure* measure, COG* pcog)
    {
        double start_t = params(0,0); 
        double end_t   = params(params.GetN()-1,0);
        size_t ncloud  = cloud.GetPointNumber();
        size_t ntv  = params.GetN();
            
        for (int i = 0; i < ncloud; i++)
        {
            // Calculate integral of the velocity for getting displacment
            double sumdx = 0, sumdy = 0, sumdz = 0, step = 0;
            double curr_t = cloud.Points(i , 3);          

            if (ntv == 1) // CV model
            {
                sumdx = curr_t*params(0,1);
                sumdy = curr_t*params(0,2);
                sumdz = curr_t*params(0,3);
            } 
            else        // General model
            {                    

              if (curr_t < start_t) 
              {
                double dtrest = curr_t - start_t;
                sumdx = params(0,1)*dtrest;
                sumdy = params(0,2)*dtrest;
                sumdz = params(0,3)*dtrest;
                //mexPrintf("%.2f, %.2f\n", dtrest, curr_t);
              }

              for (int j = 1; j < ntv; j++)
              {              
                    if (curr_t < params(j-1, 0)) break;     
                    //mexPrintf("%i, %.2f, %.2f, %.2f\n", j, sumdx, GET(tv, j, 0, ntv), curr_t);

                    if (curr_t >= params(j, 0))
                    {
                        double dvx = (params(j-1,1) + params(j,1))/2;
                        double dvy = (params(j-1,2) + params(j,2))/2;
                        double dvz = (params(j-1,3) + params(j,3))/2;
                        step = (params(j,0) - params(j-1,0));                    
                        sumdx += step*dvx;
                        sumdy += step*dvy;
                        sumdz += step*dvz;
                        //mexPrintf("%i, %.2f, %.2f, %.2f\n", j,  GET(tv, j-1, 0, ntv), GET(tv, j, 0, ntv), curr_t);
                    }
                    else 
                    {
                        double wstart_t = params(j-1,0);
                        double wend_t = params(j,0);
                        double wstep = wend_t - wstart_t;
                        double dvxv = (curr_t - wend_t)/(wstep)*(params(j-1,1) - params(j,1)) + params(j-1,1);
                        double dvyv = (curr_t - wend_t)/(wstep)*(params(j-1,2) - params(j,2)) + params(j-1,2);
                        double dvzv = (curr_t - wend_t)/(wstep)*(params(j-1,3) - params(j,3)) + params(j-1,3);
                        sumdx += (dvxv + params(j-1,1))/2 * (curr_t-wstart_t);
                        sumdy += (dvyv + params(j-1,2))/2 * (curr_t-wstart_t);
                        sumdz += (dvzv + params(j-1,3))/2 * (curr_t-wstart_t);
                        //mexPrintf("%i, %.2f, %.2f, %.2f, part\n", j,  GET(tv, j-1, 0, ntv), GET(tv, j, 0, ntv), curr_t);                            
                    }
              }

              if (curr_t > end_t)
              {
                 double dtrest = curr_t - end_t;
                 sumdx += params(ntv-1,1)*dtrest;
                 sumdy += params(ntv-2,1)*dtrest;
                 sumdz += params(ntv-3,1)*dtrest;
              }
            }
            
            double x = cloud.Points(i,0) + sumdx;
            double y = cloud.Points(i,1) + sumdy;
            double z = cloud.Points(i,2) + sumdz;
            
            if (measure != NULL)
            {
               measure->AddPoint(x,y,z);
            }

            if (isModifyCloud)
            {
                cloud.Points(i,0) = x;
                cloud.Points(i,1) = y;
                cloud.Points(i,2) = z;                 
            }
        }   
    }
}

#endif