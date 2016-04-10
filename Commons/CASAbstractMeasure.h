#ifndef CASABSTRACTMEASURE_H
#define CASABSTRACTMEASURE_H

namespace cas
{
    class CASAbstractMeasure
    {
        public:
            virtual void Reset() {};
            virtual void AddPoint(double x, double y, double z) {};
            virtual double Calculate() {return 0;};
    };       

}

#endif