#ifndef _CONSTANTS_H
#define _CONSTANTS_H

#include <math.h>
#include <stdint.h>
#include "datatypes.h"

////////////////////////////////////////////////////////
// IMPORTANT SETTING
////////////////////////////////////////////////////////

#define NOTCH_FREQUENCY_MAINS 60 // !CHANGE ME: Change this to the frequency (in Hz) for mains power in your country

#if NOTCH_FREQUENCY_MAINS == 50
static const BP_coeffs notch_mains_coef = {1, -1.975376681190275540700440615182742476463, 0.999999999999999777955395074968691915274,
                                           1, -1.969081089945081020076145250641275197268, 0.993625933418024409604640823090448975563,
                                           0.996812966709012093780017949029570445418};

#elif NOTCH_FREQUENCY_MAINS == 60
static const BP_coeffs notch_mains_coef = {1, -1.964584196283818640083040918398182839155, 1,
                                           1, -1.958431581772660523554918654554057866335, 0.993736471541615262736968361423350870609,
                                           0.996868235770807631368484180711675435305};
                                           
#endif

////////////////////////////////////////////////////////
// Pin address on board
////////////////////////////////////////////////////////

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
#define dac_out_EN 2
#define DAC1_CS 10
#define DAC2_CS 36
#define CMD_write 0x4000
#define DAC_LATCH 21
#define STIM_ADC 20
#define STIM_OUT 0
#define ADC_SH 24

////////////////////////////////////////////////////////
// Constants
////////////////////////////////////////////////////////

// Sensing/ ADC / Input:
static const float ADC_SAMPLING_FREQUENCY = 2000;

static const float NOTCH_FREQUENCY_20Hz = 20;

static const float NORM_NOTCH_FREQUENCY_MAINS = 2.0 * NOTCH_FREQUENCY_MAINS / ADC_SAMPLING_FREQUENCY;
static const float NORM_NOTCH_FREQUENCY_20Hz = 2.0 * NOTCH_FREQUENCY_20Hz / ADC_SAMPLING_FREQUENCY;

static const float SCALE_FACTOR_ARRAY[5] = {1, 1, 0.713, 0.125, 0.062};

static const uint16_t DEFAULT_STAR_COUNT = 12500;

static const uint16_t CAL_SETTLING = 10000;

static const uint16_t CAL_POINTS = 4;

static const float BIT_VOLT_SCALAR = (0.150 / (pow(2, 23))) * 1e6;

// Sig_gen / DAC / Output:
static const uint16_t ZERO_LEVEL = 32768;  // Mid level voltage of the DAC voltage generator which = 0V.
static const uint32_t TIME_STEP = 1000000; // One second (1e6 uS)

static const uint16_t DEFAULT_SINE_FREQUENCY = 20;
static const uint16_t MIN_SINE_FREQUENCY = 1;
static const uint16_t MAX_SINE_FREQUENCY = 1000;

static const uint16_t DEFAULT_OUTPUT_uV = 50;
static const uint16_t MIN_OUTPUT_uV = 1;
static const uint16_t MAX_OUTPUT_uV = 2000;

static const uint16_t DEFAULT_GAIN = 100;
static const uint16_t MIN_GAIN = 1;
static const uint16_t MAX_GAIN = 1000;

static const uint16_t DEFAULT_STIM_LEVEL = 3000;
static const uint16_t MIN_STIM_LEVEL = 1;
static const uint16_t MAX_STIM_LEVEL = 3000;

static const uint16_t DEFAULT_EVOKED_TIME = 0;
static const uint16_t MIN_EVOKED_TIME = 0;
static const uint16_t MAX_EVOKED_TIME = 4000;

static const uint16_t DEFAULT_t_us = 100;

// SPI:
static const uint32_t CLOCK_IN = 15000000;

// Coefficients:
static const BP_coeffs STIM_BP1_coef = {1, 0, -1,
                                        1, 1.778195445536177032863633939996361732483, 0.800972851152864984136670045700157061219,
                                        0.836525143984312324896279733366100117564};

static const BP_coeffs STIM_BP2_coef = {1, 0, -1,
                                        1, -1.516060791454891365503954148152843117714, 0.614018610636670070235254570434335619211,
                                        0.836525143984312324896279733366100117564};

static const BP_coeffs STIM_LP_coef = {1, 2, 1,
                                       1, -1.991114292201653590552723471773788332939, 0.99115359586893536647522751081851311028,
                                       0.000009825916820482034428198107878227319};

static const BP_coeffs BETA_BP1_coef = {1, 0, -1,
                                        1, -1.986045333399571344301648423424921929836, 0.990528706755016141727310241549275815487,
                                        0.006255414086945538382211751127215393353};

static const BP_coeffs BETA_BP2_coef = {1, 0, -1,
                                        1, -1.98840402948143424310956106637604534626, 0.991778879213337072329181864915881305933,
                                        0.006255414086945538382211751127215393353};

// Ranged BP Filters:
static const BP_coeffs FULL_BP_coef = {1, 0, -1,
                                       1, -1.509963529706781715233887553040403872728, 0.511505636461403678794113147887401282787,
                                       0.244247181769298188358519041685212869197}; // 1-200

static const BP_coeffs THETA_BP_coef = {1, 0, -1,
                                        1, -1.987198078733593664679801804595626890659, 0.98751192990729486087531086013768799603,
                                        0.006244035046352603389452351478894343018}; // 4-8

static const BP_coeffs ALPHA_BP_coef = {1, 0, -1,
                                        1, -1.986570425946955209539623865566682070494, 0.98751192990732150622790186389465816319,
                                        0.006244035046339239079793426157038993551}; // 8-12

static const BP_coeffs BETA_BP_coef = {1, 0, -1,
                                       1, -1.941536755810836289981580193853005766869, 0.944991987857986659449238686647731810808,
                                       0.027504006071006645989251993000834772829}; // 12-30

static const BP_coeffs LOW_GAMMA_BP_coef = {1, 0, -1,
                                            1, -1.924722152984461143176986297476105391979, 0.939062505817492065851581628521671518683,
                                            0.030468747091253921971398810342179785948}; // 30-50

static const BP_coeffs MID_GAMMA_BP_coef = {1, 0, -1,
                                            1, -1.872325526560577824142228564596734941006, 0.909929988177737736343431151908589527011,
                                            0.045035005911131069378239288880649837665}; // 50-80

static const BP_coeffs HIGH_GAMMA_BP_coef = {1, 0, -1,
                                             1, -1.547151183331659574093919218285009264946, 0.679599298224526693346092542924452573061,
                                             0.160200350887736597815802497279946692288}; // 80-200

#endif