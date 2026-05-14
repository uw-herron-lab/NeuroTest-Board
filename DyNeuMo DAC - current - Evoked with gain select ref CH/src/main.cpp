#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <math.h>
#include <IIRFilter.h>
#include <TsyDMASPI.h>
#include "complex.h"
#include "datatypes.h"
#include "sensing.h"
#include "sig_gen.h"

Sig_gen generator;
Sensing sense;

// TODO: Find a better way to handle this
// time in microseconds, used in counters
static volatile int t_us = DEFAULT_t_us;

///////////////////////////////////// FOR YASH

/** TODO:
 * Steps for debug, documentation, and refinement:
 * 1. Extract classes [done]
 * 2. Extract methods [mostly done]
 * 3. Document [first pass done]
 * 4. Add additional functionality
 *    - Pretty sure the bandpass filters have been made and sent but reintegrate with changes
 *    - Add functionality to load and save calibration settings
 */

/////////////////////////////////////

// TODO: Do something about these global vars
File myFile;
// myTimer is the interrupt timer
IntervalTimer myTimer;
// ZERO_LEVEL is the mid level voltage of the DAC voltage generator which = 0V.
// static uint16_t ZERO_LEVEL = 32768;
// adc_data_ready this is used for the ADC data ready interrupt which is set to 1kHz
static volatile bool adc_data_ready = false;
// timer interrupt flag
volatile static bool t_counter_en = false;
volatile uint16_t us_time_counter = 0;
volatile uint16_t sense_time_counter = 0;
// this filter_ call seems to just be a default call as other filter_ calls have variables in them so im unsure what this one does exactly
void filter_(double, double, double, BP_filter &); // Function prototype

// ADC ready interrupt function
void adc_flag()
{
  adc_data_ready = true;
}

// Timer interrupt flag function sets flag to true every 1.6mS the time can be changed
void time_counter()
{
  t_counter_en = true;
  us_time_counter = us_time_counter + t_us;
  sense_time_counter = sense_time_counter + t_us;
}

void set_bounds(uint16_t &value, uint16_t lower, uint16_t upper)
{
  if (value > upper)
  {
    value = upper;
  }
  else if (value < lower)
  {
    value = lower;
  }
}

void setup()
{

  SD.begin(BUILTIN_SDCARD);
  Serial.setTimeout(1);

  pinMode(CLK_EN, OUTPUT);
  digitalWrite(CLK_EN, HIGH);
  pinMode(adc1_MUX, OUTPUT);
  pinMode(adc1_EN, OUTPUT);
  pinMode(adc2_EN, OUTPUT);
  pinMode(adc2_MUX, OUTPUT);
  digitalWrite(adc1_EN, LOW);
  digitalWrite(adc2_EN, LOW);
  digitalWrite(adc1_MUX, HIGH);
  digitalWrite(adc2_MUX, HIGH);
  pinMode(adc_CS, OUTPUT);
  pinMode(STIM_ADC, OUTPUT);
  pinMode(DAC_LATCH, OUTPUT);
  digitalWrite(DAC_LATCH, LOW);
  digitalWrite(adc_CS, HIGH);
  digitalWrite(STIM_ADC, HIGH);
  pinMode(dac_out_EN, OUTPUT);
  digitalWrite(dac_out_EN, HIGH);
  pinMode(STIM_OUT, OUTPUT);
  digitalWrite(STIM_OUT, HIGH);
  pinMode(DAC1_CS, OUTPUT);
  pinMode(DAC2_CS, OUTPUT);
  digitalWrite(DAC1_CS, HIGH);
  digitalWrite(DAC2_CS, HIGH);
  digitalWrite(adc_RST, LOW);

  delayMicroseconds(1000);
  digitalWrite(adc_RST, HIGH);
  delayMicroseconds(1000);

  attachInterrupt(adc_RDY, adc_flag, FALLING);

  Serial.begin(115200);

  u_int16_t address = 4;
  u_int16_t cmd_READ_REG = 1;
  u_int16_t cmd = cmd_READ_REG | (address << 7 | 0);
  u_int16_t cmd_write_ = CMD_write | (address << 7 | 0);

  u_int16_t ID_[8];

  SPI.begin();
  generator.set_output(DAC1_CS, ZERO_LEVEL);
  generator.set_output(DAC2_CS, ZERO_LEVEL);
  delay(100);
  SPI1.setMISO(39);
  SPI1.setSCK(27);
  SPI1.setMOSI(26);
  SPI1.begin();

  // set freq to 2khz
  cmd_write_ = 0x6000 | (3 << 7 | 0);
  SPI1.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE1));
  digitalWrite(adc_CS, LOW);
  delayMicroseconds(1);
  ID_[0] = SPI1.transfer16(cmd_write_);
  SPI1.transfer(0x00);

  ID_[1] = SPI1.transfer16(0x0308);
  SPI1.transfer(0x00);
  ID_[2] = SPI1.transfer16(0x0000);
  SPI1.transfer(0x00);
  ID_[4] = SPI1.transfer16(0x0000);
  SPI1.transfer(0x00);
  ID_[5] = SPI1.transfer16(0x0000);
  SPI1.transfer(0x00);
  ID_[6] = SPI1.transfer16(0x0000);
  SPI1.transfer(0x00);

  delayMicroseconds(1);

  digitalWrite(adc_CS, HIGH);
  SPI1.endTransaction();

  // DC block setup
  cmd_write_ = 0x6000 | (8 << 7 | 0);
  SPI1.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE1));
  digitalWrite(adc_CS, LOW);
  delayMicroseconds(1);
  ID_[0] = SPI1.transfer16(cmd_write_);
  SPI1.transfer(0x00);

  ID_[1] = SPI1.transfer16(0x0A);
  SPI1.transfer(0x00);
  ID_[2] = SPI1.transfer16(0x0000);
  SPI1.transfer(0x00);
  ID_[4] = SPI1.transfer16(0x0000);
  SPI1.transfer(0x00);
  ID_[5] = SPI1.transfer16(0x0000);
  SPI1.transfer(0x00);
  ID_[6] = SPI1.transfer16(0x0000);
  SPI1.transfer(0x00);

  delayMicroseconds(1);
  digitalWrite(adc_CS, HIGH);
  SPI1.endTransaction();
  // gain set gain for the two channels,
  // ch0 gain 128 range 9.37mV peak
  // ch1 gain 8 range 150mV peak
  cmd_write_ = 0x6000 | (4 << 7 | 0);
  SPI1.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE1));

  digitalWrite(adc_CS, LOW);
  delayMicroseconds(1);

  ID_[0] = SPI1.transfer16(cmd_write_);
  SPI1.transfer(0x00);

  ID_[1] = SPI1.transfer16(0x33); // 0x37 for gain of 128 and gain of 8
  SPI1.transfer(0x00);

  ID_[2] = SPI1.transfer16(0x0000);
  SPI1.transfer(0x00);

  ID_[4] = SPI1.transfer16(0x0000);
  SPI1.transfer(0x00);

  ID_[5] = SPI1.transfer16(0x0000);
  SPI1.transfer(0x00);

  ID_[6] = SPI1.transfer16(0x0000);
  SPI1.transfer(0x00);

  delayMicroseconds(1);
  digitalWrite(adc_CS, HIGH);
  SPI1.endTransaction();

  generator.set_output(DAC1_CS, ZERO_LEVEL);
  generator.set_output(DAC2_CS, ZERO_LEVEL);
  myTimer.priority(1);
  // set timer interrupt to 625Hz the sampling freq of the data in the file
  myTimer.begin(time_counter, t_us);
}


void loop()
{
  static int sine_freq = DEFAULT_SINE_FREQUENCY;
  static bool bootup = false; // Run the filter once, should move to startup

  float output_b = 0;
  static u_int32_t idx = 0;
  u_int16_t output_b_code = 0;
  static byte play_data = 0;
  static bool file_open = false;

  static int16_t star_count = DEFAULT_STAR_COUNT;

  static long t_ = 0;
  String comm_;

  static cal_values CAL;
  static struct BP_filter filt;
  static struct adc_data ADC_D;
  static u_int16_t output_uV = DEFAULT_OUTPUT_uV;

  static int CAL_START = 0;  // NOTE: This can be replaced by a bool
  static int burst_mode = 1; // This should be an enum
  static int burst_mode_ON = 1;
  static int burst_mode_OFF = 5;
  static u_int8_t sig_output_state = 0;
  static u_int8_t reset_file_index = 1;
  static u_int16_t stim_level = DEFAULT_STIM_LEVEL;

  static u_int16_t gain_ = DEFAULT_GAIN;
  static float gain_m = gain_ / 100.0;

  static int voltage_ref = 0; // NOTE: This can be replaced by a bool
  static u_int16_t evoked_time = DEFAULT_EVOKED_TIME;
  if (bootup == false) // This should probably be done once in set up?
  {
    bootup = true;
    filter_(2000, 18, 22, filt);
  }

  if (t_counter_en == true)
  {
    t_counter_en = false;
    if (CAL_START == 0)
    {
      if (play_data == 1)
      {
        float CAL_uV = (output_uV - CAL.b0) / CAL.b1;

        sig_output_state = generator.signal_gen(t_us, burst_mode, sine_freq, burst_mode_ON, burst_mode_OFF, CAL_uV, gain_m, ADC_D);
      }
      else if (play_data == 2)
      {
        play_data = generator.play_file_data(myFile, idx, CAL.b1, CAL.b0, reset_file_index, gain_m, &sig_output_state);
        if (reset_file_index == 1)
        {
          reset_file_index = 0;
        }
      }
    }
    else if (CAL_START == 1)
    {
      output_b = sin(sine_freq * PI * 2 * t_ / TIME_STEP) * 1 * CAL.sig_scaler;
      output_b_code = u_int16_t(output_b + ZERO_LEVEL);
      t_ = t_ + t_us;
      generator.write_both_DAC(output_b_code);
    }
  }

  if (adc_data_ready == true)
  {
    adc_data_ready = false;

    sense.adc_read(ADC_D, stim_level, burst_mode, evoked_time, voltage_ref, filt, sig_output_state);

    if (CAL_START == 1)
    {
      sense.cal(ADC_D, CAL);
      if (CAL.cal_finish == true)
      {
        CAL_START = 0;
        CAL.cal_finish = false;
        CAL.cal_start = false;
        CAL.sig_scaler = 5;
        play_data = 0;
        generator.write_both_DAC(ZERO_LEVEL);
      }
    }
  }

  if (Serial.available())
  {

    comm_ = Serial.readStringUntil('#');
    String end_;
    if (comm_ == "Freq")
    {

      sine_freq = Serial.parseInt();
      double f0 = sine_freq - 2;
      double f1 = sine_freq + 2;
      if (sine_freq <= 0)
      {
        sine_freq = 1;
        f0 = 0.2;
        f1 = 2;
      }
      else if (sine_freq >= 1000)
      {
        sine_freq = 1000;
      }
      else if (sine_freq > 1 && sine_freq < 4)
      {
        f0 = sine_freq - 1;
        f1 = sine_freq + 1;
      }
      else if (sine_freq == 1)
      {
        f0 = 0.2;
        f1 = 2;
      }

      Serial.flush();
      filter_(2000, f0, f1, filt);

      Serial.println("test");
    }

    else if (comm_ == "CAL")
    {

      CAL_START = 1;
      CAL.cal_start = true;
      CAL.cal_finish = false;
      Serial.flush();
      t_us = DEFAULT_t_us;
      myTimer.update(t_us);
    }
    else if (comm_ == "REFE")
    {
      int reNOTCH_FREQUENCY_MAINSh = Serial.parseInt();
      if (reNOTCH_FREQUENCY_MAINSh == 0)
      {
        voltage_ref = 0;
      }
      else if (reNOTCH_FREQUENCY_MAINSh == 1)
      {
        voltage_ref = 1;
      }
      else
      {
        voltage_ref = 0;
      }
    }
    else if (comm_ == "GAIN")
    {
      gain_ = Serial.parseInt();

      set_bounds(gain_, MIN_GAIN, MAX_GAIN);

      gain_m = gain_ / 100.0;
    }
    else if (comm_ == "PlayB")
    {
      output_uV = Serial.parseInt();
      burst_mode = Serial.parseInt();
      burst_mode_ON = Serial.parseInt();
      burst_mode_OFF = Serial.parseInt();

      set_bounds(output_uV, MIN_OUTPUT_uV, MAX_OUTPUT_uV);

      if (burst_mode > 3)
      {
        burst_mode = 3;
      }
      else if (burst_mode < 1)
      {
        burst_mode = 1;
      }
      if (burst_mode_ON < 1)
      {
        burst_mode_ON = 1;
      }
      else if (burst_mode_ON > 300)
      {
        burst_mode_ON = 300;
      }
      if (burst_mode_OFF < 1)
      {
        burst_mode_OFF = 1;
      }
      else if (burst_mode_OFF > 300)
      {
        burst_mode_OFF = 300;
      }

      t_us = DEFAULT_t_us;

      play_data = 1;

      myTimer.update(t_us);

      Serial.flush();
    }
    else if (comm_ == "PlayF")
    {

      String file_name = Serial.readStringUntil(';');
      u_int16_t file_name_len = file_name.length() + 1;
      char file_[file_name_len];
      file_name.toCharArray(file_, file_name_len);

      if (SD.exists(file_))
      {
        myFile = SD.open(file_, FILE_READ);

        myFile.seek(0);
        file_open = true;
        float Fs_DAC = myFile.parseFloat();
        float idx_ = myFile.parseFloat();

        idx = u_int32_t(idx_);
        t_us = int(1.0e6 / Fs_DAC);

        if (t_us < 500)
        {
          t_us = 500;
        }
        else if (t_us > 2000)
        {
          t_us = 2000;
        }
        myTimer.update(t_us);

        play_data = 2;
      }
      else
      {
        Serial.println("Error: file does not exist");
      }

      Serial.flush();
    }
    else if (comm_ == "Stop")
    {

      play_data = 0;
      myFile.seek(0);
      myFile.close();
      reset_file_index = 1;
      t_us = DEFAULT_t_us;
      generator.write_both_DAC(ZERO_LEVEL);
      myTimer.update(t_us);
      burst_mode = 1;
      Serial.flush();
    }
    else if (comm_ == "PlayEv")
    {
      output_uV = Serial.parseInt();
      stim_level = Serial.parseInt();
      int evoked_mode = Serial.parseInt(); // This should be replaced with an enum
      evoked_time = Serial.parseInt();
      if (evoked_mode == 1)
      {
        burst_mode = 5; // No High freq sin
      }
      else if (evoked_mode == 2)
      {
        burst_mode = 6; // No High freq cos
      }
      else if (evoked_mode == 3)
      {
        burst_mode = 4; // HF sin
      }
      else
      {
        burst_mode = 5;
      }

      set_bounds(output_uV, MIN_OUTPUT_uV, MAX_OUTPUT_uV);

      set_bounds(stim_level, MIN_STIM_LEVEL, MAX_STIM_LEVEL);

      set_bounds(evoked_time, MIN_EVOKED_TIME, MAX_EVOKED_TIME);

      t_us = 500;

      play_data = 1; // Signal to firmware that a timeseries is ready to be played

      myTimer.update(t_us); // Sets sampling rate

      Serial.flush();
    }
  }
}

void filter_(double fs, double f1, double f2, BP_filter &filt_v)
{

  double f0 = sqrt(f1 * f2);
  double Q = f0 / (f2 - f1);
  double x2 = 1.0 / (sqrt(2) * Q);
  Complex x3(-x2, x2);
  Complex x4(-4, -1 / pow(Q, 2));
  Complex x5(0, 0);
  Complex x6(0, 0);
  x5 = x4.c_sqrt();
  x6 = x3 + x5;
  Complex x7(0.5, 0);
  Complex x8(0, 0);
  x8 = x6 * x7;
  Complex x10(0, 0);
  x10 = x3 - x5;
  Complex x11(0, 0);
  x11 = x7 * x10;
  double x8_abs = sqrt(pow(x8.real(), 2) + pow(x8.imag(), 2));
  double x11_abs = sqrt(pow(x11.real(), 2) + pow(x11.imag(), 2));
  double qk8 = (-1.0 * x8_abs) / (2 * x8.real());
  double qk11 = (-1.0 * x11_abs) / (2 * x11.real());
  double w0k_8 = x8_abs * 2 * PI * f0 / fs;
  double w0k_11 = x11_abs * 2 * PI * f0 / fs;
  double w01 = w0k_11;
  double q1 = qk11;
  double alpha1 = sin(w01 / (2 * q1));
  double tsettling1 = 8 * q1 / (w01 * fs);
  double w02 = w0k_8;
  double q2 = qk8;
  double alpha2 = sin(w02 / (2 * q2));
  double tsettling2 = 8 * q2 / (w02 * fs);

  double stage1_a0 = 1 + alpha1;
  filt_v.stages[0].a0 = stage1_a0 / stage1_a0;
  filt_v.stages[0].a1 = (-2 * cos(w01)) / stage1_a0;
  filt_v.stages[0].a2 = (1 - alpha1) / stage1_a0;
  double stage1_b0 = (alpha1);
  filt_v.stages[0].b0 = stage1_b0 / stage1_b0;
  filt_v.stages[0].b1 = 0;
  filt_v.stages[0].b2 = -alpha1 / stage1_b0;
  filt_v.stages[0].gain = (q1 / Q) * (stage1_b0 / stage1_a0);

  double stage2_a0 = 1 + alpha2;
  filt_v.stages[1].a0 = stage2_a0 / stage2_a0;
  filt_v.stages[1].a1 = (-2 * cos(w02)) / stage2_a0;
  filt_v.stages[1].a2 = (1 - alpha2) / stage2_a0;
  double stage2_b0 = alpha2;
  filt_v.stages[1].b0 = stage2_b0 / stage2_b0;
  filt_v.stages[1].b1 = 0;
  filt_v.stages[1].b2 = -alpha2 / stage2_b0;
  filt_v.stages[1].gain = (q2 / Q) * (stage2_b0 / stage2_a0);
}