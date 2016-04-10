#ifndef UTIL_H
#define UTIL_H

#define _USE_MATH_DEFINES 
#include <math.h>

#include <stdio.h>
#include <string.h>

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

namespace cas 
{    
    // from MinGW
    inline void *memcpy (void *d, const void *s, size_t n)
    {
      const char *sc, *se;
      char *dc;

      dc = (char *) d;
      sc = (const char *) s;
      se = sc + n;

      if (se != sc)
        {
          do {
           *dc++ = *sc++;
          } while (sc < se);
        }
      return d;
    }
}
    
#endif