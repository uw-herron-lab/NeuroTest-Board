#ifndef _SENSING_H
#define _SENSING_H

#include "Arduino.h"
#include "constants.h"
#include "IIRFilter.h"
#include <SPI.h>
// #include "IIRFilter.h"

class Sensing
{
public:
    Sensing();

    /**
     * @brief  Calibrates the output
     *
     * Uses a simple linear regression model to generate calibration values off of the analog data in
     *
     * @param  data_:  Analog data
     * @param  C:      Calibration values
     */
    void cal(adc_data &data_, cal_values &C);

    /** TODO:
     * @brief 
     */
    void save_cal();

    /** TODO:
     * @brief 
     */
    void load_cal();

    /**
     * @brief  Reads the ADC
     *
     * Filters the analog data and populates the values to the adc_data struct
     *
     * @param  data_:       Analog data
     * @param  stim_level:  Used for evoked potential
     * @param  burst_mode:  Different stimulation bursts  NOTE: Change to enum that is descriptive
     * @param  evoked_t:    Time window for capture evoked potentials
     * @param  volt_ref:    Voltage reference level for the ADC
     * @param  filt_v:      Band-pass filter settings
     * @param  state:       @todo 
     */
    void adc_read(adc_data &data_, int stim_level, int burst_mode, int evoked_t, int volt_ref, struct BP_filter filt_v, int state); 

    /** TODO:
     * @brief 
     * 
     * write long
     * 
     * @param srate
     * @param low_bound
     * @param high_bound
     * @param type
     * 
     */
    void calc_coeffs(float srate, float low_bound, float high_bound);

private:
    IIR_filter _CH0_IIR_Notch;
    IIR_filter _CH1_IIR_Notch;

    IIR_filter _CH0_STIM_BP1;
    IIR_filter _CH0_STIM_BP2;

    IIR_filter _CH0_STIM_ENV;
    IIR_filter _CH1_STIM_ENV;

    IIR_filter _CH0_BETA_BP1;
    IIR_filter _CH0_BETA_BP2;

    IIR_filter _CH0_BP1;
    IIR_filter _CH0_BP2;

    IIR_filter _CH1_STIM_BP1;
    IIR_filter _CH1_STIM_BP2;

    IIR_filter _CH1_BP1;
    IIR_filter _CH1_BP2;
};

#endif