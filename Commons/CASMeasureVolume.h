#ifndef CASMEASUREBINNUM_H
#define CASMEASUREBINNUM_H

#include "CASAbstractBinMeasure.h"

namespace cas
{
    class CASMeasureVolume : public CASAbstractBinMeasure
    {
        public:
            CASMeasureVolume(double bin_size);
            CASMeasureVolume(const CASMeasureVolume& obj);
            void Reset();
            inline void AddPoint(double x, double y, double z);
            double Calculate();
            
        private:
            int npts;
    };      
    
    CASMeasureVolume::CASMeasureVolume(const CASMeasureVolume& obj) : CASAbstractBinMeasure(obj)
    {
        
    }
    
    CASMeasureVolume::CASMeasureVolume(double bin_size)
    {
        bins.clear();
        this->bin_size = bin_size;
        npts = 0;
    }
    
    void CASMeasureVolume::Reset()
    {
        bins.clear();
        npts = 0;
    }
            
    inline void CASMeasureVolume::AddPoint(double x, double y, double z)
    {
        int bin_x = (x >= 0) ? (int)(x / bin_size) : (int)(x / bin_size)-1;
        int bin_y = (y >= 0) ? (int)(y / bin_size) : (int)(y / bin_size)-1;
        int bin_z = (z >= 0) ? (int)(z / bin_size) : (int)(z / bin_size)-1;                   
        bins[Bin(bin_x, bin_y, bin_z)]++;   
        npts++;
    }
    
    double CASMeasureVolume::Calculate()
    {
         return (double)bins.size();      
    }
    
}

#endif