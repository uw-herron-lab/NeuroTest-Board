#include "IIRFilter.h"

IIR_filter::IIR_filter()
{
  for (int i = 0; i < 3; i++)
  {
    _x_history[i] = 0.0;
    _y_history[i] = 0.0;
  }
}

double IIR_filter::IIR_SOS(float input_data, BP_coeffs coef)
{
  // {b0,b1,b2,a0,a1,a2,gain} we ignore a0 as its 1 in direct form 1
  _x_history[0] = _x_history[1];
  _x_history[1] = _x_history[2]; // shift past input samples in input buffer _x_history
  _x_history[2] = input_data * coef.gain;
  _y_history[0] = _y_history[1];
  _y_history[1] = _y_history[2]; // shift past output samples in output buffer _y_history
  _y_history[2] = (coef.b2 * _x_history[0] + coef.b1 * _x_history[1] + coef.b0 * _x_history[2]) - (coef.a2 * _y_history[0] + coef.a1 * _y_history[1]);
  double yk = _y_history[2];
  return yk;
}
