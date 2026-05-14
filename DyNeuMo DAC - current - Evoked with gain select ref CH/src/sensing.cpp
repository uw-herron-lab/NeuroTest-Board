#include "sensing.h"

Sensing::Sensing() {}

void Sensing::cal(adc_data &data_, cal_values &C)
{
    static float scaler_values[CAL_POINTS];
    static float rms_measurements[CAL_POINTS];
    static float product_scaler_rms[CAL_POINTS];
    static float squared_scaler_values[CAL_POINTS];

    static float sum_scaler_values = 0, sum_rms_measurements = 0,
                 sum_product_scaler_rms = 0, sum_squared_scaler_values = 0;
    static float sumV = 0;

    static bool cal_stop = false;
    static u_int16_t cal_counter = 0;
    static u_int16_t settling_t = CAL_SETTLING;
    static u_int16_t cal_t = 0;

    if (C.cal_finish == false && C.cal_start == true)
    {

        if (settling_t > 0)
        {
            settling_t--;
        }
        else if (cal_t < CAL_SETTLING)
        {
            float sqV = data_.electrode.band_pass * data_.electrode.band_pass;
            sumV += sqV;
            cal_t++;
        }
        else if (cal_t >= CAL_SETTLING && cal_counter < 4)
        {

            float RMS_val = sqrt(sumV / CAL_SETTLING);
            // Serial.print("RMS: ");Serial.println(RMS_val);
            rms_measurements[cal_counter] = RMS_val;
            scaler_values[cal_counter] = C.sig_scaler;
            product_scaler_rms[cal_counter] = RMS_val * C.sig_scaler;
            squared_scaler_values[cal_counter] = C.sig_scaler * C.sig_scaler;
            sum_scaler_values += scaler_values[cal_counter];
            sum_rms_measurements += rms_measurements[cal_counter];
            sum_product_scaler_rms += product_scaler_rms[cal_counter];
            sum_squared_scaler_values += squared_scaler_values[cal_counter];
            C.sig_scaler = C.sig_scaler * 4;

            cal_counter++;
            cal_t = 0;
            settling_t = CAL_SETTLING;
            sumV = 0;
            cal_t = 0;
        }
        else if (cal_counter >= 4)
        {
            cal_t = 0;
            settling_t = CAL_SETTLING;
            sumV = 0;
            cal_counter = 0;
            cal_stop = true;

            C.b1 = ((4.0 * sum_product_scaler_rms) - (sum_scaler_values * sum_rms_measurements)) / ((4.0 * sum_squared_scaler_values) - ((sum_scaler_values * sum_scaler_values)));
            C.b0 = (sum_rms_measurements - (C.b1 * sum_scaler_values)) / 4.0;
            C.cal_finish = true;
            C.cal_start = false;

            sum_scaler_values = 0;
            sum_rms_measurements = 0;
            sum_product_scaler_rms = 0;
            sum_squared_scaler_values = 0;
        }
    }
}

void Sensing::adc_read(adc_data &data_, int stim_level, int burst_mode, int evoked_t, int volt_ref, BP_filter filt_v, int state)
{
    SPI1.beginTransaction(SPISettings(CLOCK_IN, MSBFIRST, SPI_MODE1));

    uint8_t x = 0;
    uint8_t x2 = 0;
    uint8_t x3 = 0;
    uint32_t aux = 0;
    uint32_t aux2 = 0;
    int32_t ch0 = 0;
    int32_t ch1 = 0;

    noInterrupts();
    digitalWrite(adc_CS, LOW);
    delayMicroseconds(1);

    x = SPI1.transfer(0x00);
    x2 = SPI1.transfer(0x00);
    SPI1.transfer(0x00);

    x = SPI1.transfer(0x00);
    x2 = SPI1.transfer(0x00);
    x3 = SPI1.transfer(0x00);

    aux = (((x << 16) | (x2 << 8) | x3) & 0x00FFFFFF);

    if (aux > 0x007FFFFF)
    {
        ch0 = (((~aux) & 0x00FFFFFF) + 1) * -1;
    }
    else
    {
        ch0 = aux;
    }

    x = SPI1.transfer(0x00);
    x2 = SPI1.transfer(0x00);
    x3 = SPI1.transfer(0x00);

    aux2 = (((x << 16) | (x2 << 8) | x3) & 0x00FFFFFF);
    if (aux2 > 0x007FFFFF)
    {
        ch1 = ((~(aux2) & 0x00FFFFFF) + 1) * -1;
    }
    else
    {
        ch1 = aux2;
    }

    SPI1.transfer(0x00);
    SPI1.transfer(0x00);
    digitalWrite(adc_CS, HIGH);
    SPI1.endTransaction();

    interrupts();
    static double CH0 = 0;
    static double CH1 = 0;
    double CH0_Notch;
    double CH1_Notch;
    double CH0_HF_STIM;
    double CH0_HF_STIM_ENV;
    double CH0_beta;
    static int evoked_response_time = 0;
    CH0 = ch0 * BIT_VOLT_SCALAR;
    CH1 = ch1 * BIT_VOLT_SCALAR;

    CH0_Notch = ((_CH0_IIR_Notch.IIR_SOS(CH0, notch_mains_coef)));
    CH1_Notch = ((_CH1_IIR_Notch.IIR_SOS(CH1, notch_mains_coef)));
    CH0_HF_STIM = (_CH0_STIM_BP2.IIR_SOS(_CH0_STIM_BP1.IIR_SOS((CH0_Notch), STIM_BP1_coef), STIM_BP2_coef));
    double CH0_STIM_ABS = abs(CH0_HF_STIM);
    CH0_HF_STIM_ENV = _CH0_STIM_ENV.IIR_SOS(CH0_STIM_ABS, STIM_LP_coef);
    CH0_beta = _CH0_BETA_BP2.IIR_SOS(_CH0_BETA_BP1.IIR_SOS((CH0_Notch), BETA_BP1_coef), BETA_BP2_coef);

    double CH0_BP = _CH0_BP2.IIR_SOS(_CH0_BP1.IIR_SOS(CH0_Notch, filt_v.stages[0]), filt_v.stages[1]);
    double CH1_BP = _CH1_BP2.IIR_SOS(_CH1_BP1.IIR_SOS(CH1_Notch, filt_v.stages[0]), filt_v.stages[1]);
    double CH1_HF_STIM = (_CH1_STIM_BP2.IIR_SOS(_CH1_STIM_BP1.IIR_SOS((CH1_Notch), STIM_BP1_coef), STIM_BP2_coef));
    double CH1_STIM_ABS = abs(CH1_HF_STIM);
    double CH1_HF_STIM_ENV = _CH1_STIM_ENV.IIR_SOS(CH1_STIM_ABS, STIM_LP_coef);

    data_.ch[0].raw = CH0;
    data_.ch[1].raw = CH1;

    data_.ch[0].notched = CH0_Notch;
    data_.ch[1].notched = CH1_Notch;

    data_.ch[0].filtered_20Hz = CH0_beta;

    data_.ch[0].band_pass = CH0_BP;
    data_.ch[1].band_pass = CH1_BP;

    data_.ch[0].stim = CH0_HF_STIM;
    data_.ch[1].stim = CH1_HF_STIM;

    if (volt_ref == 0)
    {
        data_.electrode.band_pass = CH0_BP;
        data_.electrode.stim = CH0_HF_STIM;
        data_.electrode.stim_env = CH0_HF_STIM_ENV;
        data_.electrode.stim_abs = CH0_STIM_ABS;
    }
    else if (volt_ref == 1)
    {
        data_.electrode.band_pass = CH1_BP;
        data_.electrode.stim = CH1_HF_STIM;
        data_.electrode.stim_env = CH1_HF_STIM_ENV;
        data_.electrode.stim_abs = CH1_STIM_ABS;
    }

    if (data_.electrode.stim_abs > stim_level && data_.electrode.stim_detect == 0 && (burst_mode >= 4 || burst_mode <= 6)) // Evan: the last portion of this if statement will always evaluate to True ans should either be removed or changed to an &&
    {
        data_.electrode.stim_detect = 1;
    }
    else if (burst_mode < 4)
    {
        data_.evoked_detect = false;
        data_.electrode.stim_detect = 0;
        evoked_response_time = 0;
    }

    if (!data_.evoked_detect && data_.electrode.stim_detect == 1 && (burst_mode >= 4 || burst_mode <= 6)) // Evan: the last portion of this if statement will always evaluate to True ans should either be removed or changed to an &&
    {
        if (evoked_t == 0)
        {
            data_.evoked_detect = true;
            evoked_response_time = 0;
        }
        else if (evoked_t > 0)
        {
            if (evoked_response_time >= evoked_t)
            {
                data_.evoked_detect = true;
                data_.electrode.stim_detect = 1;
                evoked_response_time = 0;
                Serial.print("D ");
                Serial.println(evoked_response_time);
            }
            else if (evoked_response_time < evoked_t)
            {
                evoked_response_time++;
            }
        }
    }

    Serial.printf("O%f,%f,%f,%f,%f,%f,%f,%i\n",
                  CH0, CH0_Notch, CH1, CH1_Notch,
                  data_.electrode.stim, data_.electrode.stim_env, data_.electrode.band_pass, state);
}
