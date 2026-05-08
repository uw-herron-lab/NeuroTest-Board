#include "sig_gen.h"

Sig_gen::Sig_gen() {}

void Sig_gen::set_output(uint16_t DAC, uint16_t out)
{
    SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
    digitalWriteFast(DAC, LOW);
    digitalWriteFast(DAC, LOW);
    digitalWriteFast(DAC, LOW);
    SPI.transfer16(out);
    digitalWrite(DAC, HIGH);
    SPI.endTransaction();
}

void Sig_gen::write_both_DAC(uint16_t out)
{
    noInterrupts();
    set_output(DAC1_CS, out);
    set_output(DAC2_CS, out);
    interrupts();
}

u_int8_t Sig_gen::signal_gen(int t_us, int burst_mode, int freq, int burst_ON_t, int burst_OFF_t, float amplitude, float gain_m, adc_data &data_)
{
    float output_b;
    static long t_;
    u_int16_t output_b_code;
    static u_int32_t burst_ON_counter;
    static u_int32_t burst_OFF_counter;
    static u_int32_t burst_ON_rand;
    static u_int32_t burst_OFF_rand;
    static u_int8_t state_ = 0;
    static u_int16_t evoked_time_counter = 0;
    static u_int16_t evoked_time_limit;
    static byte evoke_phase = 0;
    float evoked_h_freq = 100;
    if (state_ == 0)
    {
        t_ = 0;
    }
    if (burst_mode == 1) // signal always ON
    {
        state_ = 1;
        output_b = sin(freq * PI * 2 * t_ / TIME_STEP) * gain_m * amplitude;
        output_b_code = u_int16_t(output_b + ZERO_LEVEL);
        t_ = t_ + t_us;
        write_both_DAC(output_b_code);
    }
    else if (burst_mode == 2) // burst fixed duration in seconds
    {
        if (burst_ON_counter < (burst_ON_t * 1000))
        {
            state_ = 1;
            output_b = sin(freq * PI * 2 * t_ / TIME_STEP) * gain_m * amplitude;
            output_b_code = u_int16_t(output_b + ZERO_LEVEL);
            t_ = t_ + t_us;
            write_both_DAC(output_b_code);
            burst_ON_counter++;
        }
        else if (burst_ON_counter >= (burst_ON_t * 1000) && burst_OFF_counter < (burst_OFF_t * 1000))
        {
            state_ = 0;
            output_b_code = ZERO_LEVEL;
            t_ = t_ + t_us;
            write_both_DAC(output_b_code);
            burst_OFF_counter++;
        }
        else if (burst_ON_counter >= (burst_ON_t * 1000 && burst_OFF_counter >= (burst_OFF_t * 1000)))
        {
            state_ = 0;
            burst_OFF_counter = 0;
            burst_ON_counter = 0;
        }
    }
    else if (burst_mode == 3) // random burst  in seconds
    {
        if (burst_ON_counter == 0 && burst_OFF_counter == 0)
        {
            state_ = 0;
            burst_OFF_rand = random(6, 60) * 1000;
            burst_ON_rand = random(1, 3) * 1000;
            burst_ON_counter++;
        }
        else if (burst_ON_counter < (burst_ON_rand))
        {

            state_ = 1;
            output_b = sin(freq * PI * 2 * t_ / TIME_STEP) * gain_m * amplitude;
            output_b_code = u_int16_t(output_b + ZERO_LEVEL);
            t_ = t_ + t_us;
            write_both_DAC(output_b_code);
            burst_ON_counter++;
        }
        else if (burst_ON_counter >= (burst_ON_rand) && burst_OFF_counter < (burst_OFF_rand))
        {
            state_ = 0;
            output_b_code = ZERO_LEVEL;
            t_ = t_ + t_us;
            write_both_DAC(output_b_code);
            burst_OFF_counter++;
        }
        else if (burst_ON_counter >= (burst_ON_rand) && burst_OFF_counter >= (burst_OFF_rand))
        {
            state_ = 0;
            burst_OFF_counter = 0;
            burst_ON_counter = 0;
        }
    }
    else if (burst_mode == 4) // This might be the evoked potential stuff?
    {

        // Serial.println(evoked_time_limit);
        if (data_.evoked_detect == true && evoke_phase == 0)
        {
            evoked_time_counter = 0;
            if (freq < 5)
            {
                evoked_h_freq = freq * 20;
                evoked_time_limit = int(1000 / (evoked_h_freq)) * 0.5;
            }
            else
            {
                evoked_h_freq = 100;
                evoked_time_limit = int(1000 / (100)) * 0.5;
            }

            evoke_phase = 1;
            t_ = 0;
        }
        if (data_.evoked_detect == true && evoke_phase == 1)
        {
            if (evoked_time_counter < evoked_time_limit)
            {
                state_ = 1;
                output_b = sin(evoked_h_freq * PI * 2 * t_ / TIME_STEP) * -2 * amplitude * gain_m;
                output_b_code = u_int16_t(output_b + ZERO_LEVEL);
                t_ = t_ + t_us;
                write_both_DAC(output_b_code);
                evoked_time_counter++;
            }
            else
            {
                evoke_phase = 2;
                evoked_time_counter = 0;
                t_ = 0;
                evoked_time_limit = int(1000 / freq) * 2;
            }
        }

        else if (data_.evoked_detect == true && evoke_phase == 2)
        {

            if (evoked_time_counter < evoked_time_limit * 0.5)
            {
                output_b = sin(freq * PI * 2 * t_ / TIME_STEP) * 1 * amplitude * gain_m;
                output_b_code = u_int16_t(output_b + ZERO_LEVEL);
                t_ = t_ + t_us;
                write_both_DAC(output_b_code);
                evoked_time_counter++;
            }

            else if (evoked_time_counter >= evoked_time_limit * 0.5 && evoked_time_counter < evoked_time_limit)
            {
                output_b = sin(freq * PI * 2 * t_ / TIME_STEP) * 0.25 * amplitude * gain_m;
                output_b_code = u_int16_t(output_b + ZERO_LEVEL);
                t_ = t_ + t_us;
                write_both_DAC(output_b_code);
                evoked_time_counter++;
            }
            else if (evoked_time_counter >= evoked_time_limit * 1 && evoked_time_counter < evoked_time_limit * 1.5)
            {
                output_b = sin(freq * PI * 2 * t_ / TIME_STEP) * 0.05 * amplitude * gain_m;
                output_b_code = u_int16_t(output_b + ZERO_LEVEL);
                t_ = t_ + t_us;
                write_both_DAC(output_b_code);
                evoked_time_counter++;
            }

            else
            {
                evoke_phase = 0;
                evoked_time_counter = 0;
                t_ = 0;
                data_.evoked_detect = false;
                data_.electrode.stim_detect = 0;
                state_ = 0;
            }
        }
    }
    else if (burst_mode == 5) // Evan: with the current settings, the evoked potential stuff enters here
    {

        if (data_.evoked_detect == true && evoke_phase == 0)
        {
            evoked_time_counter = 0;
            evoke_phase = 1;
            t_ = 0;
            evoked_time_limit = int(1000 / freq) * 2; // Evan: for the current settings (freq=20), this would be 1000 samples
        }
        if (data_.evoked_detect == true && evoke_phase == 1)
        {
            state_ = 1;
            if (evoked_time_counter < evoked_time_limit * 0.5)
            {
                output_b = sin(freq * PI * 2 * t_ / TIME_STEP) * 1 * amplitude * gain_m;
                output_b_code = u_int16_t(output_b + ZERO_LEVEL);
                t_ = t_ + t_us;
                write_both_DAC(output_b_code);
                evoked_time_counter++;
            }

            else if (evoked_time_counter >= evoked_time_limit * 0.5 && evoked_time_counter < evoked_time_limit)
            {
                output_b = sin(freq * PI * 2 * t_ / TIME_STEP) * 0.25 * amplitude * gain_m;
                output_b_code = u_int16_t(output_b + ZERO_LEVEL);
                t_ = t_ + t_us;
                write_both_DAC(output_b_code);
                evoked_time_counter++;
            }
            else if (evoked_time_counter >= evoked_time_limit * 1 && evoked_time_counter < evoked_time_limit * 1.5)
            {
                output_b = sin(freq * PI * 2 * t_ / TIME_STEP) * 0.05 * amplitude * gain_m;
                output_b_code = u_int16_t(output_b + ZERO_LEVEL);
                t_ = t_ + t_us;
                write_both_DAC(output_b_code);
                evoked_time_counter++;
            }

            else
            {
                evoke_phase = 0;
                evoked_time_counter = 0;
                t_ = 0;
                data_.evoked_detect = false;
                data_.electrode.stim_detect = 0;
                state_ = 0;
            }
        }
    }
    else if (burst_mode == 6)
    {
        if (data_.evoked_detect == true && evoke_phase == 0)
        {
            evoked_time_counter = 0;
            evoke_phase = 1;
            t_ = 0;
            evoked_time_limit = int(10000 / freq) * 2;
        }
        if (data_.evoked_detect == true && evoke_phase == 1)
        {
            state_ = 1;
            if (evoked_time_counter < evoked_time_limit * 0.75)
            {
                output_b = cos(freq * PI * 2 * t_ / TIME_STEP) * -1 * amplitude * gain_m;
                output_b_code = u_int16_t(output_b + ZERO_LEVEL);
                t_ = t_ + t_us;
                write_both_DAC(output_b_code);
                evoked_time_counter++;
            }

            else if (evoked_time_counter >= evoked_time_limit * 0.75 && evoked_time_counter < evoked_time_limit * 1.25)
            {
                output_b = cos(freq * PI * 2 * t_ / TIME_STEP) * -0.25 * amplitude * gain_m;
                output_b_code = u_int16_t(output_b + ZERO_LEVEL);
                t_ = t_ + t_us;
                write_both_DAC(output_b_code);
                evoked_time_counter++;
            }
            else if (evoked_time_counter >= evoked_time_limit * 1.25 && evoked_time_counter < evoked_time_limit * 1.75)
            {
                output_b = cos(freq * PI * 2 * t_ / TIME_STEP) * -0.05 * amplitude * gain_m; //*scale_sine;//DAC_scaler;
                output_b_code = u_int16_t(output_b + ZERO_LEVEL);
                t_ = t_ + t_us;
                write_both_DAC(output_b_code);
                evoked_time_counter++;
            }

            else
            {
                evoke_phase = 0;
                evoked_time_counter = 0;
                t_ = 0;
                data_.evoked_detect = false;
                data_.electrode.stim_detect = 0;
                state_ = 0;
            }
        }
    }
    return state_;
}

byte Sig_gen::play_file_data(File playback, u_int32_t idx, float b1, float b0, u_int8_t reset_file_index, float gain_m, u_int8_t *event_state)
{
    static u_int32_t idx_counter = 0;
    float output_b = 0;
    float check_state = 0;

    u_int16_t output_b_code = 0;
    static bool start_ = false;
    static byte play_data = 0;

    static int16_t star_count = DEFAULT_STAR_COUNT;
    if (reset_file_index == 1)
    {
        star_count = DEFAULT_STAR_COUNT;
        idx_counter = 0;
        start_ = false;
    }

    if (start_ == false)
    {
        play_data = 2;
        if (star_count > 0)
        {
            output_b = 1;
            output_b_code = ZERO_LEVEL;
            write_both_DAC(output_b_code);
            star_count--;
        }
        else if (star_count <= 0)
        {
            start_ = true;
            star_count = (DEFAULT_STAR_COUNT / 2);
            idx_counter = 0;
        }
    }
    else if (playback.available() && idx_counter < idx && start_ == true)
    {
        output_b = playback.parseFloat();

        if (check_state == 0)
        {
            *event_state = 0;
        }
        else
        {
            *event_state = 1;
        }

        float CAL_uV = (output_b - b0) / b1 * gain_m;

        output_b_code = u_int16_t(CAL_uV + ZERO_LEVEL);
        write_both_DAC(output_b_code);
        idx_counter++;
    }
    else if (idx_counter >= idx)
    {
        playback.close();

        idx_counter = 0;
        start_ = false;
        play_data = 0;
        idx_counter = 0;
        start_ = false;

        set_output(DAC1_CS, ZERO_LEVEL);
        set_output(DAC2_CS, ZERO_LEVEL);
    }

    return play_data;
}
