#ifndef CASMATRIX_H
#define CASMATRIX_H

#include <string.h>
#include "util.h"
#include "CASShallowMatrix.h"

namespace cas
{
    class CASMatrix : public CASShallowMatrix
    {
        public:
            CASMatrix(size_t n, size_t m);
            CASMatrix(double* ptr_array, size_t n, size_t m);
            CASMatrix(size_t n, size_t m, double value);
            CASMatrix(double* ptr_mat_array, size_t n, size_t m, size_t limit_n);
            CASMatrix(const CASMatrix& obj);
            ~CASMatrix();
    };    
    
    CASMatrix::CASMatrix(double* ptr_mat_array, size_t n, size_t m) 
    {
        this->n = n;
        this->m = m;
        ptr_array = new double[n*m];        
        memcpy(ptr_array, ptr_mat_array, sizeof(double)*n*m); 
    }
    
    CASMatrix::CASMatrix(const CASMatrix& obj)
    {
        this->n = obj.GetN();
        this->m = obj.GetM();
        ptr_array = new double[n*m];
        memcpy(ptr_array, obj.ptr_array, sizeof(double)*n*m);
    }

                
    CASMatrix::CASMatrix(double* ptr_mat_array, size_t n, size_t m, size_t limit_n) 
    {
        this->n = n;
        this->m = m;
        ptr_array = new double[n*m];        
        memcpy(ptr_array, ptr_mat_array, sizeof(double)*limit_n); 
    }
    
    
    CASMatrix::CASMatrix(size_t n, size_t m)
    {
        this->n = n;
        this->m = m;
        ptr_array = new double[n*m];
        for(int i = 0; i<n*m; i++)
            ptr_array[i] = 0;
    }
    
    CASMatrix::CASMatrix(size_t n, size_t m, double value)
    {
        this->n = n;
        this->m = m;
        ptr_array = new double[n*m];
        for(int i = 0; i<n*m; i++)
            ptr_array[i] = value;
    }

    
    CASMatrix::~CASMatrix()
    {
        if (ptr_array != NULL)
        {
            delete ptr_array;
            ptr_array = NULL;
        }
    }

}

#endif