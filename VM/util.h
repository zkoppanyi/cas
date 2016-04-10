#ifndef UTIL_H
#define UTIL_H

#define BIGNUM 1000000

#define GET(A,i,j, nrows) A[i + j * nrows]

#define GET_FROM_SETTINGS(FIELDNAME, VARIABLE, CONVERTER, TYPE)  \
    if (mxGetFieldNumber(settings, FIELDNAME) > -1)   \
    {                                                  \
        VARIABLE = (TYPE)CONVERTER(mxGetField(settings, (mwIndex)0, FIELDNAME));\
    }
            

#define MYASSERT(VAL1, VAL2)       \
    if (VAL1 != VAL2)             \
    {                              \
        mexPrintf("Val1: %i Val2: %i \n", VAL1, VAL2); \
        mexErrMsgTxt("ASSERT! Problem with the code\n"); \
        return;                    \
    }
    
#endif