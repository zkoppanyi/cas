#ifndef BIN_H
#define BIN_H

#include <map>

namespace cas
{
    
    struct Bin 
    {
        int x; int y; int z;
        Bin(int x, int y, int z) : x(x), y(y), z(z) {};

        bool operator<(const Bin& other) const
        {
           if (x != other.x)
               return (x < other.x);

           if (y != other.y)
               return (y < other.y);

           return (z < other.z);
        };
    };
    
    typedef std::map<Bin,int> Bins;
}

#endif