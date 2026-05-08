#ifndef _DATATYPES_H
#define _DATATYPES_H

struct BP_coeffs
{
  double b0, b1, b2; // numerator (feedforward) coefficients
  double a0, a1, a2; // denominator (feedback) coefficients
  double gain;
};

struct BP_filter
{
  BP_coeffs stages[2]; // Stages for the filter 
};

/**
 * @brief Enum for burst modes
 */
enum burst_mode
{
 // TODO: Fill this out
};

/**
 * @brief Calibration values for the DAC
 *
 * In the linear regression model b1 slope and b0 is the y-intercept
 */
struct cal_values
{
  float b0 = 0;
  float b1 = 1;
  float sig_scaler = 5;
  bool cal_finish = false;
  bool cal_start = false;
};

/**
 * @brief Analog data for a channel
 *
 * Analog data for a channel
 */
struct channel_data
{
  float raw;
  float notched;
  float band_pass;
  float filtered_20Hz;
  float stim;
};

/**
 * @brief Data for the reference electrode
 *
 * Data for the reference electrode
 */
struct ref_electrode_data
{
  float band_pass;
  float stim;
  float stim_env;
  float stim_abs;
  int stim_detect;
};

/**
 * @brief Combined all data in for the ADC
 *
 * Uses two channel_data structs to represent the two analog streams coming in.
 * Additionally has a ref_electrode_data struct and a flag for if an evoked potential is detected
 */
struct adc_data
{
  channel_data ch[2];
  ref_electrode_data electrode;
  bool evoked_detect;
};

#endif