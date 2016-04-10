#ifndef MEXBINS_H
#define MEXBINS_H

#define _USE_MATH_DEFINES 
#include <math.h>
#include <map>

#include "CASMatrix.h"
#include "Bin.h"
#include "COG.h"

#include "util.h"

namespace cas
{
    inline COG calculateCOG(double* cloud, size_t ncloud)
    {
        COG res;
        for (int i = 0; i < ncloud; i++)
        {           
            res.x += GET(cloud, i , 0, ncloud);
            res.y += GET(cloud, i , 1, ncloud);
            res.z += GET(cloud, i , 2, ncloud);
        }

        res.x = res.x / ncloud;
        res.y = res.y / ncloud;
        res.z = res.z / ncloud;

        return res;
    }

    inline double calculateEntropy(std::map<Bin,int> bins, int ncloud)
    {    
            // Calcaulte entropy            
             double enew = 0;
             for(std::map<Bin,int>::iterator iter = bins.begin(); iter != bins.end(); ++iter)
             {
                 int v = iter->second;
                 enew += ((double)v/(double)ncloud) * log(((double)v/(double)ncloud));
             }
             return -enew * 100;
    }

    inline void calculateReconstructionCA3DoF(double* cloud, int ncloud, 
                                              double vx, double vy,  double vz, double ax,  double ay, double az, 
                                              double bin_size, std::map<Bin,int>& bins, COG& pcog,
                                              bool isModifyCloud=false,
                                              bool isAngleCorrection=false, double cx=0,  double cy=0)
    {

        pcog.x = 0; pcog.y = 0; pcog.z = 0;    
        int i;
        double t, x, y, z, x2, y2, angle, bin_x, bin_y, bin_z;

        for (i = 0; i < ncloud; i++)
        {           
            t = GET(cloud, i , 3, ncloud);           
            x = GET(cloud, i , 0, ncloud) + vx*t + (ax/2*t*t);
            y = GET(cloud, i , 1, ncloud) + vy*t + (ay/2*t*t);        
            z = GET(cloud, i , 2, ncloud) + vz*t + (az/2*t*t);    

            // Rotate the point
            if (isAngleCorrection)
            {
                x2 = x - cx;
                y2 = y - cy;
                angle = -atan2(vy*t + (ay/2*t*t), vx*t + (ax/2*t*t));
                x =  cos(angle)*x2 - sin(angle)*y2;
                y =  sin(angle)*x2 + cos(angle)*y2;
                x +=  cx;
                y +=  cy;
            }

            pcog.x += x;
            pcog.y += y;
            pcog.z += z;

            if(isModifyCloud)
            {
                GET(cloud, i , 0, ncloud) = x;
                GET(cloud, i , 1, ncloud) = y;        
                GET(cloud, i , 2, ncloud) = z;  
            }

            bin_x = (x >= 0) ? (int)(x / bin_size) : (int)(x / bin_size)-1;
            bin_y = (y >= 0) ? (int)(y / bin_size) : (int)(y / bin_size)-1;
            bin_z = (z >= 0) ? (int)(z / bin_size) : (int)(z / bin_size)-1;                   
            bins[Bin(bin_x, bin_y, bin_z)]++;             
        }

        pcog.x = pcog.x / ncloud;
        pcog.y = pcog.y / ncloud;
        pcog.z = pcog.z / ncloud;
    }

    inline void calculateReconstructionGeneralTrapez(double* tv, size_t ntv, double* cloud, size_t ncloud)
    {
        double start_t = GET(tv, 0 , 0, ncloud); 
        double end_t = GET(tv, ntv-1 , 0, ncloud);

        for (int i = 0; i < ncloud; i++)
        {
            // Calculate integral of the velocity for getting displacment
            double sumdx = 0, sumdy = 0, sumdz = 0, step = 0;
            double curr_t = GET(cloud, i , 3, ncloud);          

            if (ntv == 1) // CV model
            {
                sumdx = curr_t*GET(tv, 0, 1, ntv);
                sumdy = curr_t*GET(tv, 0, 2, ntv);
                sumdz = curr_t*GET(tv, 0, 3, ntv);
            } 
            else        // General model
            {                    

              if (curr_t < start_t) 
              {
                double dtrest = curr_t - start_t;
                sumdx = GET(tv, 0, 1, ntv)*dtrest;
                sumdy = GET(tv, 0, 2, ntv)*dtrest;
                sumdz = GET(tv, 0, 3, ntv)*dtrest;
                //mexPrintf("%.2f, %.2f\n", dtrest, curr_t);
              }

              for (int j = 1; j < ntv; j++)
              {              
                    if (curr_t < GET(tv, j-1, 0, ntv)) break;     
                    //mexPrintf("%i, %.2f, %.2f, %.2f\n", j, sumdx, GET(tv, j, 0, ntv), curr_t);

                    if (curr_t >= GET(tv, j, 0, ntv))
                    {
                        double dvx = (GET(tv, j-1, 1, ntv) + GET(tv, j, 1, ntv))/2;
                        double dvy = (GET(tv, j-1, 2, ntv) + GET(tv, j, 2, ntv))/2;
                        double dvz = (GET(tv, j-1, 3, ntv) + GET(tv, j, 3, ntv))/2;
                        step = GET(tv, j, 0, ntv) - GET(tv, j-1, 0, ntv);                    
                        sumdx += step*dvx;
                        sumdy += step*dvy;
                        sumdz += step*dvz;
                        //mexPrintf("%i, %.2f, %.2f, %.2f\n", j,  GET(tv, j-1, 0, ntv), GET(tv, j, 0, ntv), curr_t);
                    }
                    else 
                    {
                        double wstart_t = GET(tv, j-1, 0, ntv);
                        double wend_t = GET(tv, j, 0, ntv);
                        double wstep = wend_t - wstart_t;
                        double dvxv = (curr_t - wend_t)/(wstep)*(GET(tv, j, 1, ntv) - GET(tv, j-1, 1, ntv)) + GET(tv, j-1, 1, ntv);
                        double dvyv = (curr_t - wend_t)/(wstep)*(GET(tv, j, 2, ntv) - GET(tv, j-1, 2, ntv)) + GET(tv, j-1, 2, ntv);
                        double dvzv = (curr_t - wend_t)/(wstep)*(GET(tv, j, 3, ntv) - GET(tv, j-1, 3, ntv)) + GET(tv, j-1, 3, ntv);
                        sumdx += (dvxv + GET(tv, j-1, 1, ntv))/2 * (curr_t-wstart_t);
                        sumdy += (dvyv + GET(tv, j-1, 2, ntv))/2 * (curr_t-wstart_t);
                        sumdz += (dvzv + GET(tv, j-1, 3, ntv))/2 * (curr_t-wstart_t);
                        //mexPrintf("%i, %.2f, %.2f, %.2f, part\n", j,  GET(tv, j-1, 0, ntv), GET(tv, j, 0, ntv), curr_t);                            
                    }
               }

                if (curr_t > end_t)
                {
                    double dtrest = curr_t - end_t;
                    sumdx += GET(tv, ntv-1, 1, ntv)*dtrest;
                    sumdy += GET(tv, ntv-1, 2, ntv)*dtrest;
                    sumdz += GET(tv, ntv-1, 3, ntv)*dtrest;
                }
            }      

            double prev = GET(cloud, i , 0, ncloud);
            GET(cloud, i , 0, ncloud) += sumdx;
            GET(cloud, i , 1, ncloud) += sumdy;       
            GET(cloud, i , 2, ncloud) += sumdz;  

            //mexPrintf("%.2f %.2f  %.2f %.2f %.2f\n", prev, GET(cloud, i , 0, ncloud), sumdx, step, curr_t);
        }
    }
}


#endif
