#include <Arduino.h>
#include "ADS131M04.h"
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <math.h>
#include <IIR filter.h>
#include <TsyDMASPI.h>
#include <Filters.h>
#include <Filters/Notch.hpp>
#include "complex.h"




ADS131M04 adc;

static u_int64_t file_len = 0;


// Sampling frequency
const double f_s = 2000; // Hz
// Notch frequency (-∞ dB)
const double f_c = 50; // Hz
const double f_c2 = 20; // Hz
const double f_n= 2 * f_c / f_s;
// Normalized notch frequency
const double f_n2= 2 * f_c2 / f_s;

// Very simple Finite Impulse Response notch filter
auto filter1 = simpleNotchFIR(f_n);     // fundamental
auto remove_sig = simpleNotchFIR( f_n2); // second harmonic

#define adc_CS 38
#define adc_RDY 24
#define adc_RST 37
#define CLK_EN 29
#define ADC_SDI 26
#define ADC_SDO 39
#define ADC_SCK 29
#define adc1_MUX 30
#define adc2_MUX 32
#define adc2_EN 31
#define adc1_EN 28
#define adc_CS 38
#define dac_out_EN 2
#define DAC1_CS 10
#define DAC2_CS 36
#define CMD_write 0x4000
#define DAC_LATCH 21
#define STIM_ADC 20
#define STIM_OUT 0
#define ADC_SH 24

#define CMD_write 0x4000

struct cal_values{
  float b0 = 0;
  float b1 = 1;
  float sig_scaler = 50;
  bool cal_finish = false;
  bool cal_start = false;
};

struct BP_filter_CH0{
  double a0,a1,a2,b0,b1,b2,g1;
  double a10,a11,a12,b10,b11,b12,g11;
  double f0, f1, fs;
};
struct BP_filter_stim_CH0{
  double a0,a1,a2,b0,b1,b2,g1;
  double a10,a11,a12,b10,b11,b12,g11;
  double f0, f1, fs;
};

struct BP_filter_CH1{
  double a0,a1,a2,b0,b1,b2,g1;
  double a10,a11,a12,b10,b11,b12,g11;
  double f0, f1, fs;
};
struct BP_filter_stim_CH1{
  double a0,a1,a2,b0,b1,b2,g1;
  double a10,a11,a12,b10,b11,b12,g11;
  double f0, f1, fs;
};

struct adc_data{
  double CH0, CH1, CH0_N, CH1_N, CH0_BP, CH0_20Hz, CH0_STIM;
  int CH0_STIM_detect;
  bool evoked_detect;
};

static volatile int t_us = 1000;
static float DAC_scaler = 1.3;
static uint16_t zero_level = 32768;

static volatile byte pin_trigger = 0;
static volatile unsigned int adc_us_counter = 0;
static volatile unsigned int stim_us_counter = 0;
static volatile unsigned int print_us_counter = 0;
static volatile int adc_clock = 0;
static volatile int stim_state = 0;
static volatile int stim_per_state = 0;

static int freq = 1e6/125;
static int sampling_f = 10;
static int sampling_freq = ((((freq/8) + 9)/10) * 10);
static int freq_us = sampling_freq*8 ;//(((1e6/freq) + 4)/5) * 5;
static int adc_sampling_freq = 250;// sampling_freq;//freq_us/10;
static int first_pulse_width = 100;
static int gap_pulse_width = 50;
static int second_pulse_width = 100;
static uint16_t ch2_zero_level = 32768;
static uint16_t b_first_pulse_amp = 2000;
static uint16_t b_first_pulse_amp_max = 2000;
static uint16_t b_second_pulse_amp = 2000;
static uint16_t current_counter = 0;
static uint16_t stim_freq_us = 1e6/125;
static uint16_t adc_sample_freq_us = 1600; //4kHz
int baseline = 32768;
// SOS IIR filters for CH0 and CH1
// Notch filters used to remove 50Hz noise
// STIM BP1 and BP2 are 4th order butterworth filters for stim freq between 100-450Hz. 
// IIR_ENV is a lowpass filter with 1Hz cut off freq works as a smoothing filter
IIR_filter CH0_IIR_Notch ;
IIR_filter CH1_IIR_Notch ;
IIR_filter STIM_IIR_BP1 ;
IIR_filter STIM_IIR_BP2 ;
IIR_filter STIM_IIR_ENV ;

IIR_filter BETA_IIR_BP1 ;
IIR_filter BETA_IIR_BP2 ;

IIR_filter IIR_BP1 ;
IIR_filter IIR_BP2 ;

// 2khz FS coef

double notch1_coef[7] = {1,-1.975376681190275540700440615182742476463, 0.999999999999999777955395074968691915274,1, -1.969081089945081020076145250641275197268, 0.993625933418024409604640823090448975563,0.996812966709012093780017949029570445418 };
double notch2_coef[7] = {1,-1.975376681190275540700440615182742476463, 0.999999999999999777955395074968691915274,1, -1.969081089945081020076145250641275197268, 0.993625933418024409604640823090448975563,0.996812966709012093780017949029570445418 };
double STIM_BP1_coef[7] = {1,0,-1,1,1.778195445536177032863633939996361732483  ,0.800972851152864984136670045700157061219   ,0.836525143984312324896279733366100117564 };
double STIM_BP2_coef[7] = {1,0,-1,1, -1.516060791454891365503954148152843117714, 0.614018610636670070235254570434335619211 ,0.836525143984312324896279733366100117564 };
double STIM_LP_coef[7] = {1,2,1,1, -1.991114292201653590552723471773788332939, 0.99115359586893536647522751081851311028  ,0.000009825916820482034428198107878227319 };
double BETA_BP1_coef[7] = {1,0,-1,1, -1.986045333399571344301648423424921929836, 0.990528706755016141727310241549275815487  ,0.006255414086945538382211751127215393353 };

double BETA_BP2_coef[7] =  {1,0,-1,1, -1.98840402948143424310956106637604534626 , 0.991778879213337072329181864915881305933  ,0.006255414086945538382211751127215393353 };

File myFile;
// myTimer is the interrupt timer
IntervalTimer myTimer; 
// ch2_zero_level is the mid level voltage of the DAC voltage generator which = 0V.
//static uint16_t ch2_zero_level = 32768;
// adc_data_ready this is used for the ADC data ready interrupt which is set to 1kHz
static volatile bool adc_data_ready = false;
// timer interrupt flag
volatile static bool t_counter_en = false;
volatile uint16_t us_time_counter = 0;
volatile uint16_t sense_time_counter = 0;
void filter_(double,double, double, BP_filter_CH0&);
// cal function

void cal(float data_20Hz, cal_values& C)
{
  static float cal_x[4];
  static float cal_y[4];
  static float cal_xy[4];
  static float cal_xx[4];
  static float sum_x,sum_y,sum_xy,sum_xx;
  static bool cal_stop = false;
  static u_int16_t cal_counter = 0;
  static u_int16_t settling_t = 10000;
  static u_int16_t cal_t = 0;
  float sqV;
  static float sumV;
  float RMS_val = 0;

  if (C.cal_finish == false && C.cal_start == true)
  {


    if (settling_t>0)
    {
      settling_t--;
      
    }
    else if (cal_t < 10000)
    {
      sqV = data_20Hz * data_20Hz;
      sumV += sqV;
      cal_t++;
      

    }
    else if (cal_t >= 10000  && cal_counter < 4)
    {
      
      RMS_val = sqrt(sumV/10000);
      //Serial.print("RMS: ");Serial.println(RMS_val);
      cal_y[cal_counter] = RMS_val;
      cal_x[cal_counter] = C.sig_scaler;
      cal_xy[cal_counter] = RMS_val * C.sig_scaler;
      cal_xx[cal_counter] = C.sig_scaler*C.sig_scaler;
      sum_x +=cal_x[cal_counter];
      sum_y +=cal_y[cal_counter];
      sum_xy +=cal_xy[cal_counter];
      sum_xx +=cal_xx[cal_counter];
      C.sig_scaler = C.sig_scaler + 100;
      
      cal_counter++;
      cal_t = 0;
      settling_t = 10000;
      sumV = 0;
      cal_t = 0;
      
    }
    else if (cal_counter >= 4)
    {
      cal_t = 0;
      settling_t = 10000;
      sumV = 0;
      cal_counter = 0;
      cal_stop = true;
      /*
      sum_x =0;
      sum_y =0;
      sum_xy =0;
      sum_xx =0;
      */
      C.b1 = ((4.0*sum_xy)-(sum_x*sum_y))/((4.0*sum_xx)-((sum_x*sum_x)));
      C.b0 = (sum_y-(C.b1*sum_x))/4.0;
      //Serial.print("B0: ");Serial.print(C.b0);Serial.print(" B1: ");Serial.println(C.b1);
      C.cal_finish = true;
      C.cal_start = false;
      
      //cal_stop = false;
      sum_x =0;
      sum_y =0;
      sum_xy =0;
      sum_xx =0;
    }

  }

}
//ADC ready interrupt function
void adc_flag()
{
  adc_data_ready = true;
}
// this function rounds the voltage to the nears uint16_t 
uint16_t roundUp(uint16_t numToRound, uint16_t multiple)
{
    if (multiple == 0)
        return numToRound;

    uint16_t remainder = numToRound % multiple;
    if (remainder == 0)
        return numToRound;

    return numToRound + multiple - remainder;
}
// this function generates the uint16_t value needed for the DAC votlage gen
uint16_t DAC_code (float  ch_amp)

{
  float dac_code = (((ch_amp)/2.75)*32768.0)+ch2_zero_level;
  uint16_t ch_dac_code = round(dac_code);
  ch_dac_code = roundUp(ch_dac_code,4);
  return ch_dac_code;

}
void stim_handler()
{

  //stim_mux_select ( stim2_mux_cs, 1);

  noInterrupts();

  adc_us_counter = adc_us_counter + 10;
  stim_us_counter = stim_us_counter + 10;
  if (adc_us_counter >=adc_sample_freq_us)
  {
    adc_us_counter = 0;
  }
  if (stim_us_counter >=stim_freq_us)
  {
    stim_us_counter = 0;
  }
  interrupts();



}


uint16_t DAC_code (uint16_t  ch_amp)

{
  float dac_code = (((ch_amp/1.0e6)*499.0)/2.75)*32768.0;
  uint16_t ch_dac_code = round(dac_code);
  ch_dac_code = roundUp(ch_dac_code,4);
  return ch_dac_code;

}
// this function writes the DAC 1
void setOutputA(uint16_t pulse_a_amp)
{
  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
  digitalWriteFast(DAC1_CS, LOW);
  digitalWriteFast(DAC1_CS, LOW);
  digitalWriteFast(DAC1_CS, LOW);
  SPI.transfer16(pulse_a_amp);
  digitalWrite(DAC1_CS, HIGH);
  SPI.endTransaction();
}

// this function writes the DAC 2
void setOutputB(uint16_t pulse_b_amp)
{
  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
  digitalWriteFast(DAC2_CS, LOW);
  digitalWriteFast(DAC2_CS, LOW);
  digitalWriteFast(DAC2_CS, LOW);
  SPI.transfer16(pulse_b_amp);
  digitalWrite(DAC2_CS, HIGH);
  SPI.endTransaction();
}

// Timer interrupt flag function sets flag to true every 1.6mS the time can be changed
void time_counter()
{
  t_counter_en = true;
  us_time_counter = us_time_counter+100;
  sense_time_counter = sense_time_counter + 100;

}

float adc_read(adc_data& data_, int stim_level,int burst_mode, float volt_ref, struct BP_filter_CH0 filt_v, u_int8_t state_ = 0, byte play_data = 0)
{
  float scalfacarray[5] = {1,1,0.713,0.125,0.062};
  float stim_[5] = {0,0.5,1,2,3};
  float scaler_ = 1;
  static u_int16_t adc_counter = 0;
  static u_int32_t stim_counter = 0;

  SPI1.beginTransaction(SPISettings(15000000, MSBFIRST, SPI_MODE1));

  uint8_t x = 0;
  uint8_t x2 = 0;
  uint8_t x3 = 0;
  uint32_t aux = 0;
  uint32_t aux2 = 0;
  int32_t ch0 = 0;
  int32_t ch1 = 0;
  uint16_t status = 0;
  //adcOutput res;
  //noInterrupts();
  noInterrupts();
  digitalWrite(adc_CS, LOW);
  
  delayMicroseconds(1);
  long t = micros();
  x = SPI1.transfer(0x00);
  x2 = SPI1.transfer(0x00);
  SPI1.transfer(0x00);

  status = ((x << 8) | x2);

  x = SPI1.transfer(0x00);
  x2 = SPI1.transfer(0x00);
  x3 = SPI1.transfer(0x00);

  aux = (((x << 16) | (x2 << 8) | x3) & 0x00FFFFFF);
  
  if (aux > 0x007FFFFF)
  {
    ch0 = (((~aux)&0x00FFFFFF) + 1) * -1;
  }
  else{
    ch0 = aux;
  }
  

  

  x = SPI1.transfer(0x00);
  x2 = SPI1.transfer(0x00);
  x3 = SPI1.transfer(0x00);

  aux2 = (((x << 16) | (x2 << 8) | x3) & 0x00FFFFFF);
  if (aux2 > 0x007FFFFF)
  {
    ch1 = ((~(aux2)&0x00FFFFFF) + 1) * -1;
  }
  else
  {
    ch1 = aux2;
  }

  
 SPI1.transfer(0x00);
 SPI1.transfer(0x00);
 //SPI1.transfer(0x00);
 //t = micros() -t;
  //delayMicroseconds(1);
  digitalWrite(adc_CS, HIGH);
  //long tt = micros();
      //res = adc.readADC();
      
  SPI1.endTransaction();
  
  interrupts(); 
  static double CH0 = 0;
  static double CH1 = 0;
  double CH0_Notch ;
  double CH1_ ;
  double CH1_Notch;
  double CH0_HF_STIM ;
  double CH0_HF_STIM_ENV;
  double CH0_beta ;

  CH0 =  ch0*(0.150/(pow(2,23)))*1e6;
  CH1 =  ch1*(0.150 /(pow(2,23)))*1e6;
  

      CH0_Notch = ((CH0_IIR_Notch.IIR_SOS(CH0,notch1_coef)));//filter1 (CH0);float(filter1(CH0));
      CH1_Notch = ((CH1_IIR_Notch.IIR_SOS(CH1,notch1_coef)));//remove_sig (CH1);// //remove_sig (CH1);//((CH1_IIR_Notch.IIR_SOS(CH1,notch1_coef)));//sos_filter_HP2(CH1);
      CH1_ = CH1 - (volt_ref*1e6);
      CH0_HF_STIM = (STIM_IIR_BP2.IIR_SOS(STIM_IIR_BP1.IIR_SOS((CH0_Notch ),STIM_BP1_coef),STIM_BP2_coef));
      double CH0_STIM_ABS = abs(CH0_HF_STIM);
      CH0_HF_STIM_ENV= STIM_IIR_ENV.IIR_SOS(CH0_STIM_ABS ,STIM_LP_coef);
      CH0_beta = BETA_IIR_BP2.IIR_SOS(BETA_IIR_BP1.IIR_SOS((CH0_Notch),BETA_BP1_coef),BETA_BP2_coef);
      double BP_coff1[7] ={filt_v.b0,filt_v.b1,filt_v.b2,filt_v.a0,filt_v.a1,filt_v.a2,filt_v.g1} ;
      double BP_coff2[7] ={filt_v.b10,filt_v.b11,filt_v.b12,filt_v.a10,filt_v.a11,filt_v.a12,filt_v.g11} ;
      double CH0_BP = IIR_BP2.IIR_SOS(IIR_BP1.IIR_SOS(CH0_Notch,BP_coff1),BP_coff2); 
      data_.CH0 = CH0 ;
      data_.CH1 = CH1;
      data_.CH0_N = CH0_Notch;
      data_.CH1_N = CH1_Notch;
      data_.CH0_20Hz= CH0_beta;
      data_.CH0_BP = CH0_BP;
      data_.CH0_STIM = CH0_STIM_ABS;
      if (CH0_STIM_ABS > stim_level && data_.CH0_STIM_detect == 0 && (burst_mode >= 4 || burst_mode <= 6))
      {
        data_.evoked_detect = true;
        data_.CH0_STIM_detect = 1;
      }
      
      //Serial.println(data_.CH0_STIM_detect );;
      

      //t = micros()-t;
      
      Serial.print(CH0); Serial.print(",");
      Serial.print(CH0_Notch); Serial.print(",");
      Serial.print(CH1); Serial.print(",");
      Serial.print(CH1_Notch); Serial.print(",");
      Serial.print(CH0_HF_STIM);Serial.print(",");
      Serial.print(CH0_HF_STIM_ENV);Serial.print(",");
      Serial.print(CH0_beta );Serial.print(",");
      Serial.print(CH0_BP );Serial.print(",");
      Serial.print(state_);Serial.print(",");
      Serial.print(play_data);
      Serial.println();  
      
    
    
    if (CH0_HF_STIM_ENV < 120)
    {
      scaler_ = scalfacarray[0];
    }
    else if (CH0_HF_STIM_ENV >= 120 & CH0_HF_STIM_ENV < 350)
    {
      scaler_ = scalfacarray[1];
    }
    else if (CH0_HF_STIM_ENV >= 350& CH0_HF_STIM_ENV < 700)
    {
      scaler_ = scalfacarray[2];
    }
    else if (CH0_HF_STIM_ENV >= 700 & CH0_HF_STIM_ENV < 1000)
    {
      scaler_ = scalfacarray[3];
    }
    else if (CH0_HF_STIM_ENV >= 1000)
    {
      scaler_ = scalfacarray[4];
    }     
     
     
     return CH0_beta;
    
  
}
int roundStim_frq(uint16_t stim_freq, uint16_t adc_us_clock)
{
    uint16_t round_val = 0.0;
    uint16_t stim_freq_us = 1e6/stim_freq;
    if (adc_us_clock == 0)
    {
      round_val = stim_freq_us;
    }

    uint16_t remainder = stim_freq_us % adc_us_clock;
    if (remainder == 0)
    {
      round_val = stim_freq_us;
    }


    else if(remainder < (adc_us_clock/ 2))
    {
      round_val = stim_freq_us - remainder;
    }


    else if (remainder >= (adc_us_clock / 2))
    {
      round_val = stim_freq_us + adc_us_clock - remainder;
    }

    return round_val;
}

byte Play_file_data(u_int32_t idx, float b1, float b0, u_int8_t reset_file_index,float gain_m, u_int8_t* event_state)
{

  static u_int32_t idx_counter = 0;
  float output_b = 0;
  float check_state = 0;
  
  u_int16_t output_b_code = 0;
  static bool start_ = false;
  static byte play_data = 0;
  static bool file_open = false;

  static int16_t star_count = 6250*2;
    if (reset_file_index == 1)
    {
      star_count = 6250*2;
      idx_counter = 0;
      start_ = false;
    }
  
    if (start_ == false)
    {
        play_data = 2;
        if (star_count > 0)
        {
          output_b = 1;
          output_b_code = ch2_zero_level;//DAC_code(output_b);
          setOutputB(output_b_code );
          setOutputA(output_b_code );
          star_count --;
        }
        else if (star_count <= 0)
        {
          start_ = true;
          star_count = 6250;
          idx_counter = 0;
          //Serial.println("start");
        }
      }
      else if (myFile.available() && idx_counter < idx && start_ == true)
      { 
        //scale the signal fro mthe CSV file and limit it to a max of 2.75 volts, this is the maximum voltage of the DAC before the divider
        output_b = myFile.parseFloat(); 
        //check_state = myFile.parseFloat(); 
        //Serial.print(output_b); Serial.print(',');
       // Serial.println(check_state);
        if (check_state == 0)
        {
          *event_state = 0;
        }
        else{
          *event_state = 1;
        }
        float CAL_uV = (output_b-b0)/b1*gain_m;
        // ref voltage if we would like to do template subtraction 
        //voltage_gen = u_int16_t(output_b+ch2_zero_level) ;//* 10.0/255.0 ;//(255-div)/255 * 2.75V = (255-245)/255)* 2.75V = 107.8mV peak to peak
        // convert the signal to u_int16 for the DAC
        output_b_code = u_int16_t(CAL_uV+ch2_zero_level);//DAC_code(output_b);
        //Serial.println(output_b);
        noInterrupts();
        // write dat point to both channels
        setOutputB(output_b_code );
        setOutputA(output_b_code );
        interrupts();
        //Serial.println(output_b);
        idx_counter ++;
      }
      else if (idx_counter >=idx)
      {
        idx_counter = 0;
        //myFile.seek(0);
        myFile.close();
        file_open = false;
        start_ = false;
        play_data = 0;
        idx_counter = 0;
        //myFile.seek(0);
        //myFile.close();
        file_open = false;
        start_ = false;
        setOutputB(ch2_zero_level);
        setOutputA(ch2_zero_level);
        //Serial.println("stop");
        //Serial.println("stop");
      }
    
    
    return play_data;
      
}


u_int8_t Signal_Gen(int burst_mode, int freq, int burst_ON_t , int burst_OFF_t , float amplitude,float gain_m, adc_data& data_)
{
  float output_b ;
  static long t_;
  u_int16_t output_b_code;
  static u_int32_t burst_ON_counter;
  static u_int32_t burst_OFF_counter;
  static u_int32_t burst_ON_rand;
  static u_int32_t burst_OFF_rand;
  static u_int8_t state_ = 0;
  static u_int16_t evoked_time_counter = 0;
  static bool evoked_state = false;
  static u_int16_t evoked_time_limit ;//= int(1000/freq);
  static byte evoke_phase = 0;
  float evoked_h_freq = 100;
  if (state_ == 0)
  {
    t_ = 0;
  }
  if (burst_mode == 1) // signal always ON
  {
      state_ = 1;
      output_b = sin(freq* PI * 2 * t_/1000000)*gain_m*amplitude;//*scale_sine;//DAC_scaler;
      output_b_code = u_int16_t(output_b+ch2_zero_level);
      //Serial.println(output_b );
      t_ = t_ +t_us;
      //Serial.println(output_b_code );
      noInterrupts();
      setOutputA(output_b_code );
      setOutputB(output_b_code );
      interrupts();

  }
  else if (burst_mode == 2)// burst fixed duration in seconds
  {
    if (burst_ON_counter < (burst_ON_t*1000) )
    {
      state_ = 1;
      output_b = sin(freq* PI * 2 * t_/1000000)*gain_m*amplitude;//*scale_sine;//DAC_scaler;
      output_b_code = u_int16_t(output_b+ch2_zero_level);
      //Serial.println(output_b );
      t_ = t_ +t_us;
      //Serial.println(output_b_code );
      noInterrupts();
      setOutputA(output_b_code );
      setOutputB(output_b_code );
      interrupts();
      burst_ON_counter++;
    }
    else if (burst_ON_counter >= (burst_ON_t*1000) && burst_OFF_counter < (burst_OFF_t*1000) )
    {
      state_ = 0;
      output_b_code = ch2_zero_level;
      //Serial.println(output_b );
      t_ = t_ +t_us;
      //Serial.println(output_b_code );
      noInterrupts();
      setOutputA(output_b_code );
      setOutputB(output_b_code );
      interrupts();
      burst_OFF_counter++;
    }
    else if (burst_ON_counter >= (burst_ON_t*1000 && burst_OFF_counter >= (burst_OFF_t*1000)) )
    {
      state_ = 0;
      burst_OFF_counter = 0;
      burst_ON_counter = 0;

    }
  }
  else if (burst_mode == 3)// random burst  in seconds
  {
    if (burst_ON_counter == 0 && burst_OFF_counter == 0)
    {
      state_ = 0;
      burst_OFF_rand = random(6,60) * 1000;
      burst_ON_rand = random(1,3)*1000;
      burst_ON_counter++;
    }
    else if (burst_ON_counter < (burst_ON_rand) )
    {
      
      state_ = 1;
      output_b = sin(freq* PI * 2 * t_/1000000)*gain_m*amplitude;//*scale_sine;//DAC_scaler;
      output_b_code = u_int16_t(output_b+ch2_zero_level);
      //Serial.println(output_b );
      t_ = t_ +t_us;
      //Serial.println(output_b_code );
      noInterrupts();
      setOutputA(output_b_code );
      setOutputB(output_b_code );
      interrupts();
      burst_ON_counter++;
    }
    else if (burst_ON_counter >= (burst_ON_rand ) && burst_OFF_counter < (burst_OFF_rand ) )
    {
      state_ = 0;
      output_b_code = ch2_zero_level;
      //Serial.println(output_b );
      t_ = t_ +t_us;
      //Serial.println(output_b_code );
      noInterrupts();
      setOutputA(output_b_code );
      setOutputB(output_b_code );
      interrupts();
      burst_OFF_counter++;
    }
    else if (burst_ON_counter >= (burst_ON_rand) && burst_OFF_counter >= (burst_OFF_rand) )
    {
      state_ = 0;
      burst_OFF_counter = 0;
      burst_ON_counter = 0;

    }
  }
  else if (burst_mode == 4)
  {
    
    //Serial.println(evoked_time_limit);
    if (data_.evoked_detect == true && evoke_phase == 0)
    {
      evoked_time_counter = 0;
      if (freq < 5)
      {
        evoked_h_freq = freq*20;
        evoked_time_limit = int(1000/(evoked_h_freq))*0.5;
      }
      else
      {
        evoked_h_freq = 100;
        evoked_time_limit = int(1000/(100))*0.5;
      }
      
      evoke_phase = 1;
      t_ = 0;
    }
    if (data_.evoked_detect == true && evoke_phase == 1)
    {
      if (evoked_time_counter < evoked_time_limit)
      {
        state_ = 1;
        output_b = sin(evoked_h_freq * PI * 2 * t_/1000000)*-2*amplitude*gain_m;//*scale_sine;//DAC_scaler;
        output_b_code = u_int16_t(output_b+ch2_zero_level);
        //Serial.println(output_b );
        t_ = t_ +t_us;
        //Serial.println(output_b_code );
        noInterrupts();
        setOutputA(output_b_code );
        setOutputB(output_b_code );
        interrupts();
        
        evoked_time_counter ++;
      }
      else 
      {
        evoke_phase = 2;
        evoked_time_counter = 0;
        t_=0;
        evoked_time_limit = int(1000/freq)*2;
      }

    }
    
    else if (data_.evoked_detect == true && evoke_phase == 2)
    {
      
      if (evoked_time_counter < evoked_time_limit*0.5)
      {
        output_b = sin(freq* PI * 2 * t_/1000000)*1*amplitude*gain_m;//*scale_sine;//DAC_scaler;
        output_b_code = u_int16_t(output_b+ch2_zero_level);
        //Serial.println(output_b );
        t_ = t_ +t_us;
        //Serial.println(output_b_code );
        noInterrupts();
        setOutputA(output_b_code );
        setOutputB(output_b_code );
        interrupts();
        evoked_time_counter ++;
      }
      
      else if (evoked_time_counter >= evoked_time_limit*0.5 && evoked_time_counter < evoked_time_limit)
      {
        output_b = sin(freq* PI * 2 * t_/1000000)*0.25*amplitude*gain_m;//*scale_sine;//DAC_scaler;
        output_b_code = u_int16_t(output_b+ch2_zero_level);
        //Serial.println(output_b );
        t_ = t_ +t_us;
        //Serial.println(output_b_code );
        noInterrupts();
        setOutputA(output_b_code );
        setOutputB(output_b_code );
        interrupts();
        evoked_time_counter ++;
      }
      else if (evoked_time_counter >= evoked_time_limit*1 && evoked_time_counter < evoked_time_limit * 1.5)
      {
        output_b = sin(freq* PI * 2 * t_/1000000)*0.05*amplitude*gain_m;//*scale_sine;//DAC_scaler;
        output_b_code = u_int16_t(output_b+ch2_zero_level);
        //Serial.println(output_b );
        t_ = t_ +t_us;
        //Serial.println(output_b_code );
        noInterrupts();
        setOutputA(output_b_code );
        setOutputB(output_b_code );
        interrupts();
        evoked_time_counter ++;
      }
      
      else 
      {
        evoke_phase = 0;
        evoked_time_counter = 0;
        t_=0;
        data_.evoked_detect = false;
        data_.CH0_STIM_detect = 0;
        state_ = 0;
        //evoked_time_limit = int(1000/freq);
      

      }
    }
  }
  else if (burst_mode == 5)
  {
    
    //Serial.println(evoked_time_limit);
    if (data_.evoked_detect == true && evoke_phase == 0)
    {
      evoked_time_counter = 0;
      evoke_phase = 1;
      t_ = 0;
      evoked_time_limit = int(1000/freq)*2;
    }
    if (data_.evoked_detect == true && evoke_phase == 1)
    {
      state_ = 1;
      if (evoked_time_counter < evoked_time_limit*0.5)
      {
        output_b = sin(freq* PI * 2 * t_/1000000)*1*amplitude*gain_m;//*scale_sine;//DAC_scaler;
        output_b_code = u_int16_t(output_b+ch2_zero_level);
        //Serial.println(output_b );
        t_ = t_ +t_us;
        //Serial.println(output_b_code );
        noInterrupts();
        setOutputA(output_b_code );
        setOutputB(output_b_code );
        interrupts();
        evoked_time_counter ++;
      }
      
      else if (evoked_time_counter >= evoked_time_limit*0.5 && evoked_time_counter < evoked_time_limit)
      {
        output_b = sin(freq* PI * 2 * t_/1000000)*0.25*amplitude*gain_m;//*scale_sine;//DAC_scaler;
        output_b_code = u_int16_t(output_b+ch2_zero_level);
        //Serial.println(output_b );
        t_ = t_ +t_us;
        //Serial.println(output_b_code );
        noInterrupts();
        setOutputA(output_b_code );
        setOutputB(output_b_code );
        interrupts();
        evoked_time_counter ++;
      }
      else if (evoked_time_counter >= evoked_time_limit*1 && evoked_time_counter < evoked_time_limit * 1.5)
      {
        output_b = sin(freq* PI * 2 * t_/1000000)*0.05*amplitude*gain_m;//*scale_sine;//DAC_scaler;
        output_b_code = u_int16_t(output_b+ch2_zero_level);
        //Serial.println(output_b );
        t_ = t_ +t_us;
        //Serial.println(output_b_code );
        noInterrupts();
        setOutputA(output_b_code );
        setOutputB(output_b_code );
        interrupts();
        evoked_time_counter ++;
      }
      
      else 
      {
        evoke_phase = 0;
        evoked_time_counter = 0;
        t_=0;
        data_.evoked_detect = false;
        data_.CH0_STIM_detect = 0;
        state_ = 0;
        //evoked_time_limit = int(1000/freq);

      }
    }
  }
  else if (burst_mode == 6)
  {
    
    //Serial.println(evoked_time_limit);
    if (data_.evoked_detect == true && evoke_phase == 0)
    {
      evoked_time_counter = 0;
      evoke_phase = 1;
      t_ = 0;
      evoked_time_limit = int(1000/freq)*2;
    }
    if (data_.evoked_detect == true && evoke_phase == 1)
    {
      state_ = 1;
      if (evoked_time_counter < evoked_time_limit*0.75)
      {
        output_b = cos(freq* PI * 2 * t_/1000000)*-1*amplitude*gain_m;//*scale_sine;//DAC_scaler;
        output_b_code = u_int16_t(output_b+ch2_zero_level);
        //Serial.println(output_b );
        t_ = t_ +t_us;
        //Serial.println(output_b_code );
        noInterrupts();
        setOutputA(output_b_code );
        setOutputB(output_b_code );
        interrupts();
        evoked_time_counter ++;
      }
      
      else if (evoked_time_counter >= evoked_time_limit*0.75 && evoked_time_counter < evoked_time_limit*1.25)
      {
        output_b = cos(freq* PI * 2 * t_/1000000)*-0.25*amplitude*gain_m;//*scale_sine;//DAC_scaler;
        output_b_code = u_int16_t(output_b+ch2_zero_level);
        //Serial.println(output_b );
        t_ = t_ +t_us;
        //Serial.println(output_b_code );
        noInterrupts();
        setOutputA(output_b_code );
        setOutputB(output_b_code );
        interrupts();
        evoked_time_counter ++;
      }
      else if (evoked_time_counter >= evoked_time_limit*1.25 && evoked_time_counter < evoked_time_limit * 1.75)
      {
        output_b = cos(freq* PI * 2 * t_/1000000)*-0.05*amplitude*gain_m;//*scale_sine;//DAC_scaler;
        output_b_code = u_int16_t(output_b+ch2_zero_level);
        //Serial.println(output_b );
        t_ = t_ +t_us;
        //Serial.println(output_b_code );
        noInterrupts();
        setOutputA(output_b_code );
        setOutputB(output_b_code );
        interrupts();
        evoked_time_counter ++;
      }
      
      else 
      {
        evoke_phase = 0;
        evoked_time_counter = 0;
        t_=0;
        data_.evoked_detect = false;
        data_.CH0_STIM_detect = 0;
        state_ = 0;
        //evoked_time_limit = int(1000/freq);

      }
    }
  }
  return state_;
}

void setup()
{

  
  SD.begin(BUILTIN_SDCARD);
  Serial.setTimeout(1);
  

  pinMode(CLK_EN,OUTPUT);
  digitalWrite(CLK_EN,HIGH);
 
  pinMode(adc1_MUX, OUTPUT);
  pinMode(adc1_EN,OUTPUT);
  pinMode(adc2_EN,OUTPUT);
  pinMode(adc2_MUX,OUTPUT);
  digitalWrite(adc1_EN,LOW);
  digitalWrite(adc2_EN,LOW);
  digitalWrite(adc1_MUX,HIGH);
  digitalWrite(adc2_MUX,HIGH);
  pinMode(adc_CS,OUTPUT);
  pinMode(STIM_ADC,OUTPUT);
  pinMode(DAC_LATCH,OUTPUT);
  digitalWrite(DAC_LATCH,LOW);
  digitalWrite(adc_CS,HIGH);  
  digitalWrite(STIM_ADC,HIGH);
  pinMode(dac_out_EN,OUTPUT);
  digitalWrite(dac_out_EN,HIGH);
  pinMode(STIM_OUT,OUTPUT);
  digitalWrite(STIM_OUT,HIGH);
  pinMode(DAC1_CS,OUTPUT);
  pinMode(DAC2_CS,OUTPUT);
  //pinMode(18,INPUT_PULLUP);
  digitalWrite(DAC1_CS,HIGH);
  digitalWrite(DAC2_CS,HIGH);

// pinMode(ADC_SH,OUTPUT);
  //pinMode(18,INPUT_PULLUP);
 // digitalWrite(ADC_SH,LOW);
  
  digitalWrite(adc_RST,LOW);

  
  
  delayMicroseconds(1000);
  digitalWrite(adc_RST,HIGH);
  delayMicroseconds(1000);
  attachInterrupt(adc_RDY,adc_flag,FALLING);

  
  
  Serial.begin(115200);
  


 
  
  u_int16_t address = 4;
  u_int16_t cmd_READ_REG = 1;
  u_int16_t cmd = cmd_READ_REG | (address << 7 | 0);
  u_int16_t cmd_write_ =CMD_write | (address << 7 | 0);
  
  u_int16_t ID_[8];

  SPI.begin();
  setOutputB(ch2_zero_level);
  setOutputA(ch2_zero_level);
  delay(100);
  SPI1.setMISO(39); SPI1.setSCK(27);SPI1.setMOSI(26);
  SPI1.begin();

    // set freq to 2khz
  cmd_write_ =0x6000| (3 << 7 | 0);
  SPI1.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE1));
  digitalWrite(adc_CS,LOW);
  delayMicroseconds(1);
  ID_[0]  = SPI1.transfer16(cmd_write_);
  SPI1.transfer(0x00);

  ID_[1]  = SPI1.transfer16(0x0308);
  SPI1.transfer(0x00);
  ID_[2]  =SPI1.transfer16(0x0000);
  SPI1.transfer(0x00);
  ID_[4]  = SPI1.transfer16(0x0000);
  SPI1.transfer(0x00);
  ID_[5]=SPI1.transfer16(0x0000);
  SPI1.transfer(0x00);
  ID_[6] = SPI1.transfer16(0x0000);
  SPI1.transfer(0x00);

  delayMicroseconds(1);
    
  digitalWrite(adc_CS,HIGH);
  SPI1.endTransaction();

  

  // DC block setup
  cmd_write_ =0x6000| (8 << 7 | 0);
  SPI1.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE1));
  digitalWrite(adc_CS,LOW);
  delayMicroseconds(1);
  ID_[0]  = SPI1.transfer16(cmd_write_);
  SPI1.transfer(0x00);

  ID_[1]  = SPI1.transfer16(0x0A);
  SPI1.transfer(0x00);
  ID_[2]  =SPI1.transfer16(0x0000);
  SPI1.transfer(0x00);
  ID_[4]  = SPI1.transfer16(0x0000);
  SPI1.transfer(0x00);
  ID_[5]=SPI1.transfer16(0x0000);
  SPI1.transfer(0x00);
  ID_[6] = SPI1.transfer16(0x0000);
  SPI1.transfer(0x00);

  delayMicroseconds(1);
  digitalWrite(adc_CS,HIGH);
  SPI1.endTransaction();
  // gain set gain for the two channels,
  // ch0 gain 128 range 9.37mV peak
  // ch1 gain 8 range 150mV peak 
  cmd_write_ =0x6000| (4 << 7 | 0);
  SPI1.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE1));
  
  digitalWrite(adc_CS,LOW);
  delayMicroseconds(1);
  
  ID_[0]  = SPI1.transfer16(cmd_write_);
  SPI1.transfer(0x00);

  ID_[1]  = SPI1.transfer16(0x33); //0x37 for gain of 128 and gain of 8
  SPI1.transfer(0x00);

  ID_[2]  =SPI1.transfer16(0x0000);
  SPI1.transfer(0x00);

  ID_[4]  = SPI1.transfer16(0x0000);
  SPI1.transfer(0x00);

  ID_[5]=SPI1.transfer16(0x0000);
  SPI1.transfer(0x00);

  ID_[6] = SPI1.transfer16(0x0000);
  SPI1.transfer(0x00);
  
  delayMicroseconds(1);
  digitalWrite(adc_CS,HIGH);
  SPI1.endTransaction();

  
 

  //SPI1.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE1));
   setOutputA( zero_level );
   setOutputB( zero_level );
   //myFile = SD.open("Cortec data.csv", FILE_READ);//Cortec data
   myTimer.priority(1);
 // set timer interrupt to 625Hz the sampling freq of the data in the file
 
 myTimer.begin(time_counter, t_us);
  
}
/*
// Read and print ADC data
void adc_read(float volt_ref)
{
  
 

     Serial.print(CH0); Serial.print(",");
     Serial.print(CH0_Notch); Serial.print(",");
     Serial.print(CH1); Serial.print(",");
     Serial.print(CH1_Notch); Serial.print(",");
     Serial.print(CH0_HF_STIM);Serial.print(",");
     Serial.print(CH0_HF_STIM_ENV);
     Serial.println();     
    
  
}*/

void loop()
{
  static int sine_freq = 20;
  static bool bootup = false;
  
  
  static u_int32_t idx_counter = 0;
  float output_b = 0;
  static u_int32_t idx = 0;
  u_int16_t output_b_code = 0;
  static bool start_ = false;
  static byte play_data = 0;
  static bool file_open = false;

  static int16_t star_count = 6250*2;

  static float voltage_gen = 0;
  static long  t_ = 0;
  String comm_;

 static uint16_t zero_crossing_output = 0;
  static int stim_status= 0;
  static float adc_print = 0;
  static float ADC_read1 = 0;
  static float ADC_read_1khz = 0;
  static int adc_read_done = 0;
  static long counter = 0;
  static int stim_setup = 0;
  static uint8_t stim_start_stop = 1;
  static uint8_t  stim_ON = 0;
  static uint8_t  adc_ON = 0;
  static uint8_t  close_loop_ON = 0;
  float adc_iir_data0;
  float adc_iir_data1;
  float adc_iir_data2;
  float adc_iir_data_hp;
  static uint32_t sample_counter = 0;
  static float ADC1_total = 0;
  static uint8_t  adc_counter = 0;
  static long us_timer = 0;
  static long current_us_timer = 0;
  static uint8_t us_timer_bit = 0;
  static uint8_t system_setup = 0;
  static uint16_t stim_freq_Hz = 125;
  adc_sample_freq_us = 800;

  static uint16_t stim_pulse_width = 100;
  static uint16_t stim_gap_width =20;
  static uint16_t stim_sensing_offset_us = 100;
  static uint16_t adc_first_sample = 10; // micro seconds for the adc first sample
  static uint16_t g_stim_pulse = stim_pulse_width + stim_sensing_offset_us;
  static uint16_t n_stim_pulse = g_stim_pulse + stim_gap_width;
  static uint16_t stp_stim_pulse = n_stim_pulse + stim_pulse_width;
  static uint16_t stim_stp_sense_str = stp_stim_pulse + 50;
  static uint16_t adc_read_flag = 0;
  static uint16_t adc_hold_time = stim_stp_sense_str +50;//600;//
  static uint8_t adc_hold_time_en = 1; // number os samples to hold
  static uint8_t adc_SH_type = 2; // 1 = true sample hold, 2 = old mux sample hold
  static float phase_freq_hz = 2; // phase lock freq
  static float phase_degree = 90; // phase lock freq
  static float upper_limit = 20;
  static float lower_limit = -1;
  static float stim_trigger_limit = 55;
  static bool stim_burst_om = false;
  static uint16_t stim_pulse_counter = 0;
  static uint16_t stim_pulse_counter_off = 0;
  static uint16_t set_no_stim_burst_pulses = 300; // change with power change ie (5 for slow wave, 2 for beta)
  static uint16_t set_no_stim_burst_pulses_off = 300; // change with power change ie (5 for slow wave, 2 for beta)
  float slow_wave_power = 0;
  float beta_power = 0;
  float slow_wave_data = 0;
  float beta_data = 0;
  // beta zero corssing
  static float phase_freq_hz_beta = 30; // phase lock freq
  static float phase_degree_beta = 180; // phase lock freq
  static float upper_limit_beta = 10;
  static float lower_limit_beta = -1;
  static float stim_trigger_limit_beta = 15;
  static byte power_select = 0; // 0 = slow wave, 1 = beta signal
  static bool software_SH = false;
  static uint16_t current_ramp = 0;
  static float scale_sine = 1;
  static cal_values CAL;
  static struct BP_filter_CH0 filt;
  static struct adc_data ADC_D;
  static int output_uV = 50;
  
  static int CAL_START = 0;
  static int  burst_mode = 1;
  static int  burst_mode_ON = 1;
  static int  burst_mode_OFF = 5;
  static u_int8_t sig_output_state = 0;
  static u_int8_t reset_file_index = 1;
  static int stim_level = 3000;
  static int gain_ = 100;
  static float gain_m = gain_/100.0;
if (bootup == false)
  {
    bootup = true;
    filter_(2000,18,22,filt);
  }

if (t_counter_en == true)
  {
    t_counter_en = false;
    if (CAL_START== 0)
    {
      if (play_data == 1)
      {
        float CAL_uV = (output_uV-CAL.b0)/CAL.b1;
       
       sig_output_state = Signal_Gen(burst_mode,sine_freq,burst_mode_ON,burst_mode_OFF,CAL_uV,gain_m, ADC_D);
        
      }
      else if (play_data == 2)
      {
        //float CAL_uV = (output_uV-CAL.b0)/CAL.b1;
        play_data = Play_file_data(idx,CAL.b1,CAL.b0,reset_file_index,gain_m,&sig_output_state);
        if (reset_file_index == 1)
        {
          reset_file_index = 0;
        }
      }
    }
    else if (CAL_START == 1)
    {
        output_b = sin(sine_freq* PI * 2 * t_/1000000)*1*CAL.sig_scaler;//*scale_sine;//DAC_scaler;
        output_b_code = u_int16_t(output_b+ch2_zero_level);
        voltage_gen = output_b_code;
        
        t_ = t_ +t_us;
        
        noInterrupts();
        setOutputA(output_b_code );
        setOutputB(output_b_code );
        interrupts();
      
    }
    
  }

  if (adc_data_ready == true)
  {
    adc_data_ready = false;
    
    scale_sine = adc_read(ADC_D,stim_level,burst_mode, voltage_gen,filt,sig_output_state,play_data);
    //Serial.println(scale_sine);
    if (CAL_START == 1)
    {
      cal(scale_sine,CAL);
      if (CAL.cal_finish == true)
      {
        CAL_START = 0;
        CAL.cal_finish = false;
        CAL.cal_start = false;
        CAL.sig_scaler = 50;
        play_data = 0;
        noInterrupts();
        setOutputA(ch2_zero_level );
        setOutputB(ch2_zero_level);
        interrupts();
      }
    }

  }

  if (Serial.available())
    {

      comm_ = Serial.readStringUntil('#');
      String end_ ;
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
        else if (sine_freq >= 250)
        {
          sine_freq = 250;
        }
        else if (sine_freq > 1 && sine_freq < 4)
        {
            f0 = sine_freq-1;
            f1 = sine_freq + 1;
        }
        else if (sine_freq == 1)
        {
            f0 = 0.2;
            f1 = 2;
        }
        
        remove_sig = simpleNotchFIR( sine_freq *2 /f_s); // second harmonic
        Serial.flush();
        filter_(2000,f0,f1, filt);
      }
      
    else if (comm_ == "CAL")
    {
      CAL_START = 1;
      CAL.cal_start = true;
      CAL.cal_finish = false;
      //sine_freq = Serial.parseInt();
      Serial.flush();
      t_us = 1000;
      myTimer.update(t_us);
      //Serial.println("cal");
    }
    else if (comm_ == "GAIN")
    {
      gain_ = Serial.parseInt();
      if (gain_ < 1)
      {
        gain_ = 1;
      }
      else if (gain_ > 1000)
      {
        gain_ = 1000;
      }
      gain_m = gain_/100.0;
      //Serial.println("cal");
    }
    else if (comm_ == "PlayB")
    {
      output_uV = Serial.parseInt();
      burst_mode = Serial.parseInt();
      burst_mode_ON = Serial.parseInt();
      burst_mode_OFF = Serial.parseInt();
      

      if (output_uV < 1)
      {
        output_uV = 1;
      }
      else if (output_uV > 2000)
      {
        output_uV = 2000;
      }
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
      else if(burst_mode_ON > 300)
      {
        burst_mode_ON = 300;
      }
      if (burst_mode_OFF < 1)
      {
        burst_mode_OFF = 1;
      }
      else if(burst_mode_OFF > 300)
      {
        burst_mode_OFF= 300;
      }
      
      t_us = 1000;
      

      play_data = 1;
      
      
      myTimer.update(t_us);
      
      //end_ = Serial.readStringUntil(;);
      
      Serial.flush();
     //Serial.println("play");
    }
    else if (comm_ == "PlayF")
    {
      
      String file_name = Serial.readStringUntil(';');
      u_int16_t file_name_len = file_name.length()+1;
      char file_ [file_name_len ];
      file_name.toCharArray(file_,file_name_len);
      
      if (SD.exists(file_))
      {
        myFile = SD.open(file_, FILE_READ);
          
        myFile.seek(0);
        file_open = true;
        float Fs_DAC = myFile.parseFloat();//myFile.parseFloat();
        float idx_ = myFile.parseFloat();//myFile.parseFloat();
        
        //Serial.print(idx_); Serial.print(',');
        //Serial.println(Fs_DAC);
        idx = u_int32_t( idx_);
        t_us = int(1.0e6/Fs_DAC);
        
        if (t_us < 500)
        {
          t_us = 500;
        }
        else if (t_us > 2000)
        {
          t_us = 2000;
        }
        myTimer.update(t_us);
        //end_ = Serial.readStringUntil(;);
        play_data = 2;
        
      }
      else
      {
        Serial.println("Error: file does not exist");
      }
      
      
      Serial.flush();
     //Serial.println("play");
    }
    else if (comm_ == "Stop")
    {

      play_data = 0;
      myFile.seek(0);
      myFile.close();
      reset_file_index = 1;
      t_us = 1000;
      noInterrupts();
      setOutputA(ch2_zero_level );
      setOutputB(ch2_zero_level);
      interrupts();
      myTimer.update(t_us);
      burst_mode = 1;
      Serial.flush();

    }
    else if (comm_ == "PlayEv")
    {
      output_uV = Serial.parseInt();
      stim_level = Serial.parseInt();
      int evoked_mode = Serial.parseInt();
      if (evoked_mode == 1)
      {
        burst_mode = 5; // No High freq sin
      }
      else if(evoked_mode == 2)
      {
        burst_mode = 6; // No High freq cos
      }
      else if(evoked_mode == 3)
      {
        burst_mode = 4; // HF sin
      }
      else
      {
        burst_mode = 5;
      }
      if (output_uV < 1)
      {
        output_uV = 1;
      }
      else if (output_uV > 2000)
      {
        output_uV = 2000;
      }
      if (stim_level < 0)
      {
        stim_level = 1;
      }
      else if (stim_level > 3000)
      {
        stim_level = 3000;
      }
      t_us = 500;
      
      play_data = 1;
      
      
      myTimer.update(t_us);
      
      //end_ = Serial.readStringUntil(;);
      
      Serial.flush();

    }
    }
    }

  void filter_(double fs, double f1,double f2, BP_filter_CH0& filt_v) 
  {

    double f0 = sqrt(f1*f2);
    double Q = f0/(f2-f1);
    double x2 = 1.0/(sqrt(2)*Q);
    Complex x3(-x2,x2);
    Complex x4(-4,-1/pow(Q,2));
    Complex x5 (0,0);
    Complex x6 (0,0);
    x5 = x4.c_sqrt();
    x6 = x3+x5;
    Complex x7 (0.5,0);
    Complex x8 (0,0);
    x8 = x6 * x7;
    Complex x10 (0,0);
    x10 = x3-x5;
    Complex x11 (0,0);
    x11 = x7*x10;
    double x8_abs = sqrt(pow(x8.real(),2)+pow(x8.imag(),2));
    double x11_abs = sqrt(pow(x11.real(),2)+pow(x11.imag(),2));
    double qk8 = (-1.0*x8_abs)/(2*x8.real());
    double qk11 = (-1.0*x11_abs)/(2*x11.real());
    double w0k_8 = x8_abs*2*PI*f0/fs;
    double w0k_11 = x11_abs*2*PI*f0/fs;
    double w01 = w0k_11;
    double q1 = qk11 ;
    double alpha1 = sin(w01/(2*q1));
    double tsettling1 = 8*q1/(w01*fs);
    double w02 = w0k_8;
    double q2 = qk8 ;
    double alpha2 =sin(w02/(2*q2));
    double tsettling2 = 8*q2/(w02*fs);

    double stage1_a0 = 1+alpha1 ;
    filt_v.a0 = stage1_a0 /stage1_a0 ;
    filt_v.a1 = (-2*cos(w01))/stage1_a0 ;
    filt_v.a2 = (1 - alpha1)/stage1_a0 ;
    double stage1_b0 = (alpha1);
    filt_v.b0 = stage1_b0 /stage1_b0 ;
    filt_v.b1 = 0;
    filt_v.b2 = -alpha1/stage1_b0 ;
    filt_v.g1 = (q1/Q)*(stage1_b0 /stage1_a0 );
    
    double stage2_a0 = 1+alpha2;
    filt_v.a10 = stage2_a0/stage2_a0;
    filt_v.a11 = (-2*cos(w02))/stage2_a0;
    filt_v.a12= (1 - alpha2)/stage2_a0;
    double stage2_b0 = alpha2;
    filt_v.b10 = stage2_b0/stage2_b0;
    filt_v.b11 = 0;
    filt_v.b12 = -alpha2/stage2_b0;
    filt_v.g11  = (q2/Q)*(stage2_b0 /stage2_a0 );
}
  

 