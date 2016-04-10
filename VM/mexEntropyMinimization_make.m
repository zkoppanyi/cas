dos('set OMP_NUM_THREADS=3');

mex -I"../Commons/" mexEntropyMinimization.cpp 
%mex mexEntropyMinimizationSGD.cpp 
%mex mexEntropyMinimization.cpp CFLAGS="$CFLAGS -fopenmp" LDFLAGS="$LDFLAGS -fopenmp"
 