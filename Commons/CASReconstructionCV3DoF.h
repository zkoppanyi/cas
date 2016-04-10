#ifndef CASRECONSTRUCTIONCV3DOF_H
#define CASRECONSTRUCTIONCV3DOF_H

#include "CASReconstructionCA3DoF.h"

namespace cas
{
    class CASReconstructionCV3DoF : public CASReconstructionCA3DoF
    {
        public:        
            inline void Reconstruct(CASPointCloud& cloud, CASAbstractMatrix& params, bool isModifyCloud, CASAbstractMeasure* measure=NULL, COG* pcog=NULL);
            inline void Reconstruct(CASPointCloud& cloud, double vx, double vy,  double vz, bool isModifyCloud, CASAbstractMeasure* measure=NULL, COG* pcog=NULL);
    };     
    
    inline void CASReconstructionCV3DoF::Reconstruct(CASPointCloud& cloud, CASAbstractMatrix& params, bool isModifyCloud, CASAbstractMeasure* measure, COG* pcog)
    {
        Reconstruct(cloud, params(0,0), params(0,1), params(0,2), isModifyCloud, measure, pcog);
    }
    
    inline void CASReconstructionCV3DoF::Reconstruct(CASPointCloud& cloud, double vx, double vy,  double vz, bool isModifyCloud, CASAbstractMeasure* measure, COG* pcog)                                                                        
    {
        CASReconstructionCA3DoF::Reconstruct(cloud, vx, vy, vz, 0, 0, 0, 0, 0, false, isModifyCloud, measure, pcog);
    }
        
}

#endif