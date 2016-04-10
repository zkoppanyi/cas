#ifndef CASRECONSTRUCTIONCV2DOF_H
#define CASRECONSTRUCTIONCV2DOF_H

#include "CASReconstructionCA3DoF.h"

namespace cas
{
    class CASReconstructionCV2DoF : public CASReconstructionCA3DoF
    {
        public:        
            inline void Reconstruct(CASPointCloud& cloud, CASAbstractMatrix& params, bool isModifyCloud, CASAbstractMeasure* measure=NULL, COG* pcog=NULL);
            inline void Reconstruct(CASPointCloud& cloud, double vx, double vy, bool isModifyCloud, CASAbstractMeasure* measure=NULL, COG* pcog=NULL);
    };     
    
    inline void CASReconstructionCV2DoF::Reconstruct(CASPointCloud& cloud, CASAbstractMatrix& params, bool isModifyCloud, CASAbstractMeasure* measure, COG* pcog)
    {
        Reconstruct(cloud, params(0,0), params(0,1), isModifyCloud, measure, pcog);
    }
    
    inline void CASReconstructionCV2DoF::Reconstruct(CASPointCloud& cloud, double vx, double vy, bool isModifyCloud, CASAbstractMeasure* measure, COG* pcog)                                                                        
    {
        CASReconstructionCA3DoF::Reconstruct(cloud, vx, vy, 0, 0, 0, 0, 0, 0, false, isModifyCloud, measure, pcog);
    }
        
}

#endif