#ifndef IIR_filter_h
#define IIR_filter_h

#include "Arduino.h"

class IIR_filter
{
  public:
    IIR_filter();
    double IIR_SOS(float input_data, double coef[7]);
  private:
  double _xkv[3];
  double _ykv[3];

   
};

#endif