#ifndef CASABSTRACTMATRIX_H
#define CASABSTRACTMATRIX_H

namespace cas
{
    class CASAbstractMatrix
    {
        public:
            
            size_t GetN() const;
            size_t GetM() const;
            
            virtual double& operator()(size_t i, size_t j) {};
            virtual double operator()(size_t i, size_t j) const {};
            virtual char* ToString() const {};
            
        protected:
            size_t n;
            size_t m;
    };     
        
    inline size_t CASAbstractMatrix::GetN() const
    {
        return n;
    }

    inline size_t CASAbstractMatrix::GetM() const
    {
        return m;
    }
}

#endif