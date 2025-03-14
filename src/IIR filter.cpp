

#include "Arduino.h"
#include "IIR filter.h"

IIR_filter::IIR_filter()
{

}

double IIR_filter::IIR_SOS(float input_data, double coef[7])
{
  
  // {b0,b1,b2,a0,a1,a2,gain} we ignore a0 as its 1 in direct form 1
  _xkv[0] = _xkv[1]; _xkv[1] = _xkv[2]; // shift past input samples in input buffer _xkv
  _xkv[2] = input_data * coef[6];
  _ykv[0] = _ykv[1]; _ykv[1] = _ykv[2]; // shift past output samples in output buffer _ykv
  _ykv[2] =   (coef[2] * _xkv[0] + coef[1] * _xkv[1] + coef[0]* _xkv[2])
             - (coef[5] * _ykv[0] + coef[4] * _ykv[1]);
  double yk = _ykv[2];
  return yk;
}

