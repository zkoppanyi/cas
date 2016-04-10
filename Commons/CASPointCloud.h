#ifndef CASPOINTCLOUD_H
#define CASPOINTCLOUD_H

#include "CASAbstractMatrix.h"
#include "CASAbstractMeasure.h"

namespace cas
{
    class CASPointCloud
    {
        public:        
            CASPointCloud(CASAbstractMatrix& points);
            
            inline size_t GetPointNumber();
            inline void CalculateMeasure(CASAbstractMeasure* measure);
            
            CASAbstractMatrix& Points;            
    };     
        
    CASPointCloud::CASPointCloud(CASAbstractMatrix& points) : Points(points)
    {
        
    }
    
    inline size_t CASPointCloud::GetPointNumber()
    {
        return Points.GetN();
    }
    
    inline void CASPointCloud::CalculateMeasure(CASAbstractMeasure* measure)
    {
        for(int ci = 0; ci < GetPointNumber(); ci++)
        {
            measure->AddPoint(Points(ci,0), Points(ci,1), Points(ci,2));
        }       
    }

}

#endif