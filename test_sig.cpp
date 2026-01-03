#include "test_sig.h"
#include <stdio.h>
#include <thread>

TestSignal::TestSignal(float fs, int sat_id, float freq, int advance, float dfreq, float dT)
    : fs(fs),
    dfreq(dfreq),
    dT(dT),
    dco(fs),
    ca(sat_id),
    dist(RAND_MEAN, RAND_STD)
{
    dco.set_frequency(freq);
    chip_index = 0;
    buffer_index = 0;
    samples_per_chip = fs/F_CHIP;
    if(samples_per_chip*F_CHIP != (int)fs){
        printf("Sampling frequency is not a multiple of 1.023MHz\n");
        throw;
    }
    samples_per_buffer = fs / BUFFER_RATE;
    for(int a=0;a<advance;a++)
        ca.advance();
    freq_index = 0;
    samples_per_ramp = fs*dT;
    ramp_up = true;
}

std::complex<float> TestSignal::evaluate(void)
{
    if(buffer_index==0){
        // save the target time
        target_time = std::chrono::steady_clock::now();
        target_time += std::chrono::milliseconds{1000/BUFFER_RATE};
    }
    if(++buffer_index==samples_per_buffer){
        buffer_index = 0;
        // wait until the next buffer interval
        std::this_thread::sleep_until(target_time);
    }
    std::complex<float> x;
    if(chip_index==0){
        chip_value = (ca.advance())?1.0f:-1.0f;
    }
    if(samples_per_ramp){
        float t = (float)freq_index/samples_per_ramp;
        float f;
        if(ramp_up){
            f = freq + dfreq*t;
        }else{
            f = freq + dfreq*(1.0f-t);
        }
        dco.set_frequency(f);
        if(++freq_index==samples_per_ramp){
            freq_index = 0;
            if(ramp_up){
                ramp_up = false;
            }else{
                ramp_up = true;
            }
        }
    }
    x = chip_value*0.00015f;
    x *= dco.evaluate();
    x += std::complex<float>(dist(gen), dist(gen));

    if(++chip_index==samples_per_chip){
        chip_index = 0;
    }
    return x;
}
