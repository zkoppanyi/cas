#ifndef CASMEASUREENTROPY_H
#define CASMEASUREENTROPY_H

#include "util.h"
#include "CASAbstractBinMeasure.h"

namespace cas
{
    class CASMeasureEntropy : public CASAbstractBinMeasure
    {
        public:
            CASMeasureEntropy(double bin_size);
            CASMeasureEntropy(const CASMeasureEntropy& obj);
            void Reset();
            inline void AddPoint(double x, double y, double z);
            double Calculate();
            
        private:
            int npts;
    };      
    
    CASMeasureEntropy::CASMeasureEntropy(double bin_size) 
    {
        bins.clear();
        this->bin_size = bin_size;
        npts = 0;
    }
    
    CASMeasureEntropy::CASMeasureEntropy(const CASMeasureEntropy& obj) : CASAbstractBinMeasure(obj)
    {
        
    }
    
    void CASMeasureEntropy::Reset()
    {
        bins.clear();
        npts = 0;
    }
            
    inline void CASMeasureEntropy::AddPoint(double x, double y, double z)
    {
        int bin_x = (x >= 0) ? (int)(x / bin_size) : (int)(x / bin_size)-1;
        int bin_y = (y >= 0) ? (int)(y / bin_size) : (int)(y / bin_size)-1;
        int bin_z = (z >= 0) ? (int)(z / bin_size) : (int)(z / bin_size)-1;                   
        bins[Bin(bin_x, bin_y, bin_z)]++;   
        npts++;
    }
    
    double CASMeasureEntropy::Calculate()
    {
         double enew = 0;
         for(Bins::iterator iter = bins.begin(); iter != bins.end(); ++iter)
         {
             int v = iter->second;
             enew += ((double)v/(double)npts) * log(((double)v/(double)npts));
         }
         return -enew * 100;         
    }
    
}

#endif