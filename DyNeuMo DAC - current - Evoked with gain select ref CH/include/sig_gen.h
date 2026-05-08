#ifndef _SIG_GEN_H
#define _SIG_GEN_H

#include "Arduino.h"
#include "FS.h"
#include "constants.h"
#include "sensing.h"
#include <SPI.h>
#include <Wire.h>
// #include "IIRFilter.h"

class Sig_gen
{
public:
    Sig_gen();

    /**
     * @brief  Write the same value to both DACs
     *
     * Write the same value to both DACs
     *
     * @param  out:  Output
     */
    void write_both_DAC(uint16_t out);

    /**
     * @brief  Write a value to a specific DAC
     *
     * Write a value to a specific DAC
     *
     * @param  DAC:  Dac to write to
     * @param  out:  Output
     */
    void set_output(uint16_t DAC, uint16_t out);

    /**
     * @brief  Generates a signal and writes to the DAC
     *
     * Generates a signal and writes to the DAC
     *
     * @param  t_us:        Timestep of the timer in microseconds
     * @param  burst_mode:  Different stimulation bursts  NOTE: Change to enum that is descriptive
     * @param  freq:        Frequency of the generated signal
     * @param  burst_ON_t:  Duration the burst is active
     * @param  burst_OFF_t: Duration the burst is inactive
     * @param  amplitude:   Peak amplitude
     * @param  gain_m:      Gain multiplier
     * @param  data_:       Different stimulation bursts
     *
     * @return The output state of the signal
     */
    u_int8_t signal_gen(int t_us, int burst_mode, int freq, int burst_ON_t,
                        int burst_OFF_t, float amplitude, float gain_m, adc_data &data_);

    /**
     * @brief  Plays a signal from a file
     *
     * Reads a csv file from the SD card and plays the file
     * if the first two values are the sampling rate and the number of data points to replay
     *
     * @param  playback:            Signal data to replay
     * @param  idx:                 Current index in the playback file
     * @param  b0:                  Y-intercept (from sense::cal())
     * @param  b1:                  Slope (from sense::cal())
     * @param  reset_file_index:    If non-zero, resets the file read index
     * @param  gain_m:              Gain multiplier
     * @param  event_state:         Unused flag  NOTE: Remove
     *
     * @return Flag for if a file is being replayed
     */
    byte play_file_data(File playback, u_int32_t idx, float b0, float b1,
                        u_int8_t reset_file_index, float gain_m, u_int8_t *event_state);

private:
};

#endif