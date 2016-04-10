#ifndef CASABSTRACTRECONSTRUCTION_H
#define CASABSTRACTRECONSTRUCTION_H

#include "CASAbstractMatrix.h"
#include "CASAbstractMeasure.h"
#include "CASPointCloud.h"
#include "COG.h"
#include "Bin.h"

namespace cas
{
    class CASReconstructionResult
    {
        public:
            COG cog;           
            Bins bins;      
            CASReconstructionResult() {};
    };
        
    class CASAbstractReconstruction
    {
        public:        
            virtual void Reconstruct(CASPointCloud& cloud, CASAbstractMatrix& params, bool isModifyCloud, CASAbstractMeasure* measure = NULL, COG* pcog=NULL);
    };  
        
}

#endif