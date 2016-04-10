#ifndef CASSHALLOWMATRIX_H
#define CASSHALLOWMATRIX_H

#include "util.h"
#include "CASAbstractMatrix.h"

namespace cas
{
    class CASShallowMatrix : public CASAbstractMatrix
    {
        public:            
            
            CASShallowMatrix(double* ptr_array, size_t n, size_t m);          
            CASShallowMatrix(const CASShallowMatrix& obj);
            ConvertFromMatlabMatrix(double* ptr_mat_array, size_t n, size_t m);            
            
            char* ToString() const;        
            
            double& operator()(size_t i, size_t j);
            double operator()(size_t i, size_t j) const;
            
            double* ptr_array = NULL;
            
        protected:
            CASShallowMatrix();
    };    
    
    CASShallowMatrix::CASShallowMatrix() {} 
        
    CASShallowMatrix::CASShallowMatrix(const CASShallowMatrix& obj)
    {
        this->n = obj.GetN();
        this->m = obj.GetM();
        ptr_array = new double[n*m];
        ptr_array = obj.ptr_array;
    }
            
    CASShallowMatrix::CASShallowMatrix(double* ptr_mat_array, size_t n, size_t m) 
    {
        this->n = n;
        this->m = m;
        this->ptr_array = ptr_mat_array;
    }        
        
    inline double& CASShallowMatrix::operator()(size_t i, size_t j)
    {
        return ptr_array[i + j * n];
    }

    inline double CASShallowMatrix::operator()(size_t i, size_t j) const
    {
        return ptr_array[i + j * n];
    }
    
    char* CASShallowMatrix::ToString() const
    {      
        char* smatrix = new char[n*m*50];
        char buffer[20];
        sprintf (buffer, "Matrix (%i, %i)\n", n, m);
        strcpy(smatrix, buffer);
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < m; j++)
            {
                 sprintf (buffer, "%.4f, ", (*this)(i,j));
                 strcat(smatrix, buffer);
            }
            
            strcat(smatrix, "\n");
        }
        
        return smatrix;
    }
    

}

#endif