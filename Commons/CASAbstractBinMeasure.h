#ifndef CASABSTRACTBINMEASURE_H
#define CASABSTRACTBINMEASURE_H

#include "CASAbstractMeasure.h"
#include "Bin.h"

namespace cas
{
    class CASAbstractBinMeasure : public CASAbstractMeasure
    {
        public:        
            CASAbstractBinMeasure();
            CASAbstractBinMeasure(const CASAbstractBinMeasure &obj);
            Bins bins;
            double bin_size;
    };     

    CASAbstractBinMeasure::CASAbstractBinMeasure()
    {
        bins.clear();
        this->bin_size = 100;
    }
    
    CASAbstractBinMeasure::CASAbstractBinMeasure(const CASAbstractBinMeasure &obj)
    {
        this->bin_size = obj.bin_size;
        this->bins = obj.bins;
    }

}

#endif