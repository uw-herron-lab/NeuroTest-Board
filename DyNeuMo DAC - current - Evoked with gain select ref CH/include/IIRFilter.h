#ifndef IIR_filter_h
#define IIR_filter_h

#include "Arduino.h"
#include "constants.h"

class IIR_filter
{
public:
  IIR_filter();

  /**
   * @brief  Applies a second order section iir filter to an input data sample
   *
   * TODO: write long
   *
   * @param  input_data:  Current input sample to filter
   * @param  coef:        Coefficients of a BP filter
   *
   * @return The first value in the y history buffer
   */
  double IIR_SOS(float input_data, BP_coeffs coef);

private:
  double _x_history[3];
  double _y_history[3];
};
#endif