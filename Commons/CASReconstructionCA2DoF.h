#ifndef CASRECONSTRUCTIONCA2DOF_H
#define CASRECONSTRUCTIONCA2DOF_H

#include "CASReconstructionCA3DoF.h"

namespace cas
{
    class CASReconstructionCA2DoF : public CASReconstructionCA3DoF
    {
        public:        
            inline void Reconstruct(CASPointCloud& cloud, CASAbstractMatrix& params, bool isModifyCloud, CASAbstractMeasure* measure=NULL, COG* pcog=NULL);
            inline void Reconstruct(CASPointCloud& cloud, double vx, double vy,  double ax,  double ay, double cx, double cy, bool isAngleCorrection,
                                                                        bool isModifyCloud, CASAbstractMeasure* measure=NULL, COG* pcog=NULL);
    };     
    
    inline void CASReconstructionCA2DoF::Reconstruct(CASPointCloud& cloud, CASAbstractMatrix& params, bool isModifyCloud, CASAbstractMeasure* measure, COG* pcog)
    {
       if (params.GetM() >= 6) // use angle correction if the rotation center is given
       {
            Reconstruct(cloud, params(0,0), params(0,1), params(0,2), params(0,3), params(0,4), params(0,5), true, isModifyCloud, measure, pcog);
       }
       else
       {           
            Reconstruct(cloud, params(0,0), params(0,1), params(0,2), params(0,3), 0, 0, false, isModifyCloud, measure, pcog);
       }
    }
    
    inline void CASReconstructionCA2DoF::Reconstruct(CASPointCloud& cloud, double vx, double vy,  double ax,  double ay, double cx, double cy, bool isAngleCorrection,
                                                                        bool isModifyCloud, CASAbstractMeasure* measure, COG* pcog)                                                                        
    {
        CASReconstructionCA3DoF::Reconstruct(cloud, vx, vy, 0, ax, ay, 0, cx, cy, isAngleCorrection, isModifyCloud, measure, pcog);
    }
        
}

#endif