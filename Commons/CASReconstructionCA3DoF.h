#ifndef CASRECONSTRUCTIONCA3DOF_H
#define CASRECONSTRUCTIONCA3DOF_H

#include "util.h"
#include "CASAbstractMatrix.h"
#include "CASPointCloud.h"
#include "CASAbstractReconstruction.h"

namespace cas
{
    class CASReconstructionCA3DoF : public CASAbstractReconstruction
    {
        public:        
            inline void Reconstruct(CASPointCloud& cloud, CASAbstractMatrix& params, bool isModifyCloud, CASAbstractMeasure* measure=NULL, COG* pcog=NULL);
            
            inline void Reconstruct(CASPointCloud& cloud, double vx, double vy,  double vz, double ax,  double ay, double az, double cx, double cy, 
                                                                        bool isAngleCorrection, bool isModifyCloud, CASAbstractMeasure* measure=NULL, COG* pcog=NULL);
    };    
    
    inline void CASReconstructionCA3DoF::Reconstruct(CASPointCloud& cloud, CASAbstractMatrix& params, bool isModifyCloud, CASAbstractMeasure* measure, COG* pcog)
    {
       if (params.GetM() >= 8) // use angle correction if the rotation center is given
       {
            Reconstruct(cloud, params(0,0), params(0,1), params(0,2), params(0,3),  params(0,4), params(0,5), params(0,6), params(0,7), true, isModifyCloud, measure, pcog);
       }
       else
       {
            Reconstruct(cloud, params(0,0), params(0,1), params(0,2), params(0,3),  params(0,4), params(0,5), 0, 0, false, isModifyCloud, measure, pcog);           
       }
        
    }
    
    inline void CASReconstructionCA3DoF::Reconstruct(CASPointCloud& cloud, double vx, double vy,  double vz, double ax,  double ay, double az, double cx, double cy, bool isAngleCorrection,
                                                                        bool isModifyCloud, CASAbstractMeasure* measure, COG* pcog)
    {             
        double ncloud = cloud.GetPointNumber();
        
        if (pcog != NULL)
        {
            pcog->x = 0; pcog->y = 0; pcog->z = 0;    
        }
        
        int i;
        double t, x, y, z, x2, y2, angle, bin_x, bin_y, bin_z;

        for (i = 0; i < ncloud; i++)
        {           
            t = cloud.Points(i, 3);           
            x = cloud.Points(i, 0) + vx*t + (ax/2*t*t);
            y = cloud.Points(i, 1) + vy*t + (ay/2*t*t);        
            z = cloud.Points(i, 2) + vz*t + (az/2*t*t);    

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

            if (measure != NULL)
                measure->AddPoint(x,y,z);

            if (pcog != NULL)
            {
                pcog->x += x;
                pcog->y += y;
                pcog->z += z;
            }

            if(isModifyCloud)
            {
                cloud.Points(i, 0) = x;
                cloud.Points(i, 1) = y;        
                cloud.Points(i, 2) = z;  
            }          
        }

        if (pcog != NULL)
        {
            pcog->x = pcog->x / ncloud;
            pcog->y = pcog->y / ncloud;
            pcog->z = pcog->z / ncloud;
        }
    }
    

     
}

#endif