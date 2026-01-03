#include "satellite.h"
#include "gps.h"
#include "constants.h"
#include <stdio.h>
#include <cmath>

#define COSTAS_FREQ_MAX (1.0/8.0/0.001)
#define COSTAS_PHASE_MAX (M_PI/4.0)
#define COSTAS_FREQ_FACTOR 0.0005
#define COSTAS_PHASE_FACTOR 0.01
#define PLL_ERROR_STATS_SIZE 5

Satellite::Satellite(GPSRx &gpsrx, int sat, int fs, float freq)
    :gpsrx(gpsrx), sat(sat), dco(fs), f_offset_avg(5),
    pll_error_stats(PLL_ERROR_STATS_SIZE)
{
    printf("Satellite::Satellite sat:%d fs:%d freq:%f\n",
           sat+1, fs, freq);
    int samples_per_chip = fs/F_CHIP;
    samples_per_period = samples_per_chip*N_PERIOD;
    rx_buff.reset(new std::complex<float>[samples_per_period]);
    rx_buff_fft.reset(new std::complex<float>[samples_per_period]);
    prod_fft.reset(new std::complex<float>[samples_per_period]);
    corr.reset(new std::complex<float>[samples_per_period]);
    rx_plan = fftwf_plan_dft_1d(
        samples_per_period,
        reinterpret_cast<fftwf_complex*>(rx_buff.get()),
        reinterpret_cast<fftwf_complex*>(rx_buff_fft.get()),
        FFTW_FORWARD, FFTW_ESTIMATE);
    corr_plan = fftwf_plan_dft_1d(
        samples_per_period,
        reinterpret_cast<fftwf_complex*>(prod_fft.get()),
        reinterpret_cast<fftwf_complex*>(corr.get()),
        FFTW_BACKWARD, FFTW_ESTIMATE);

    dco.set_frequency(-freq);
    buffer_index = 0;
    offset = 0;
    offset_range_max = samples_per_chip/4;
    if(offset_range_max==0)
        offset_range_max = 1;
    n_valid_offsets = 0;
    n_invalid_offsets = 0;
    phase_reset = true;
    dphase_avg = 0.0;
    n_dphase = 0;
    preamble_buff = 0;
    n_valid_iq = 0;
    n_invalid_iq = 0;
    bit_acquire_reset = true;
    bit_sign = 1.0f;
    first_subframe_processed = false;
    rxstate = RXSTATE_OFFSET_ACQUIRE;
    sat_thread = std::thread(&Satellite::thread_func, this);
}

Satellite::~Satellite(){
    fftwf_destroy_plan(rx_plan);
    fftwf_destroy_plan(corr_plan);
    sat_thread.join();
    printf("Satellite::~Satellite satellite:%d\n", sat+1);
}

bool Satellite::is_active(void)
{
    return (rxstate != RXSTATE_SIGNAL_LOST);
}


void Satellite::send_ssiq(std::shared_ptr<SSIQ> ssiq)
{
    queue.push(ssiq);
}

void Satellite::thread_func(void)
{
    //printf("Satellite::thread_func Starting.\n");
    while(true){
        std::shared_ptr<SSIQ> ssiq = queue.pop();
        //printf("Satellite::thread_func received a ssiq. N_samples:%d\n", ssiq->N_samples);
        sample_index = ssiq->sample_index;
        for(int i=0;i<ssiq->N_samples;i++){
            x_in = ssiq->iq[i];
            evaluate();
            if(rxstate == RXSTATE_SIGNAL_LOST)
                return;
            sample_index++;
        }
    }
}

void Satellite::evaluate(void)
{
    std::complex<float> x = x_in*dco.evaluate();
    // offset compensation
    if(buffer_index==0){
        if(offset>0){
            // skip this sample
            offset--;
            return;
        }else if(offset<0){
            // copy samples from the previous buffer
            for(;offset<0;offset++,buffer_index++){
                rx_buff[buffer_index] = rx_buff[samples_per_period+offset];
            }
        }
    }
    rx_buff[buffer_index] = x;
    if(++buffer_index == samples_per_period){
        period();
        buffer_index = 0;
    }
}

void Satellite::period(void)
{
    fftwf_execute(rx_plan);
    std::complex<float> *rx_fft = rx_buff_fft.get();
    std::complex<float> *prn_fft = gpsrx.prns.prn_fft(sat);
    std::complex<float> *prod = prod_fft.get();
    for(int i=0;i<samples_per_period;i++){
        *(prod++) = *(rx_fft++) * *(prn_fft++);
    }
    fftwf_execute(corr_plan);
    float abs_max = 0.0f;
    std::complex<float> *c = corr.get();
    for(int i=0;i<samples_per_period;i++,c++){
        float abs = std::abs(*c);
        if(abs > abs_max){
            abs_max = abs;
            offset_max = i;
        }
    }
    offset = offset_max;
    if(offset > samples_per_period/2){
        offset -= samples_per_period;
    }
    bool offset_valid = (offset>=-offset_range_max)&&(offset<=offset_range_max);
    if(rxstate == RXSTATE_OFFSET_ACQUIRE){
        if(offset_valid){
            n_valid_offsets++;
            n_invalid_offsets = 0;
        }else{
            n_valid_offsets=0;
            n_invalid_offsets++;
        }
        if(n_valid_offsets==10){
            printf("Offset Acquired. Satellite:%2d\n", sat+1);
            rxstate = RXSTATE_FREQUENCY_ACQUIRE;
        }
        if(n_invalid_offsets==10){
            printf("Offset couldn't be acquired. Satellite:%2d\n", sat+1);
            rxstate = RXSTATE_SIGNAL_LOST;
        }
    }else if(rxstate > RXSTATE_OFFSET_ACQUIRE){
        if(!offset_valid){
            if(++n_invalid_offsets==10){
                printf("Offset lost: Satellite:%2d offset:%d\n", sat+1, offset);
                rxstate = RXSTATE_SIGNAL_LOST;
            }else{
                offset = 0;
                offset_max = 0;
            }
            return;
        }else{
            n_invalid_offsets = 0;
        }
        if(rxstate == RXSTATE_FREQUENCY_ACQUIRE){
            frequency();
        }else{
            phase();
        }
    }
}

double Satellite::fix_angle_range(double angle)
{
    if(angle > M_PI){
        angle -= 2.0*M_PI;
    }else if(angle < -M_PI){
        angle += 2.0*M_PI;
    }
    return angle;
}

void Satellite::frequency(void)
{
    float phase = arg(corr[offset_max]);
    if(phase_reset){
        phase_reset = false;
    }else{
        float dphase = fix_angle_range(phase - phase_last);
        if(std::abs(dphase)<M_PI/2){
            dphase_avg += dphase;
            if(++n_dphase==20){
                float f_offset = -dphase_avg/2.0/M_PI/0.001/n_dphase;
                float avg = f_offset_avg.evaluate(f_offset);
                dco.add_frequency(f_offset);
                if(std::abs(avg)<0.02f){
                    rxstate = RXSTATE_PLL_ACQUIRE;
                    printf("Frequency acquired. Satellite:%2d freq:%f\n",
                           sat+1, -dco.get_frequency());
                }
                n_dphase = 0;
                dphase_avg = 0.0;
            }
        }else{
            // adjust the frequency with the current dphase_avg
            if(n_dphase){
                float f_offset = -dphase_avg/2.0/M_PI/0.001/n_dphase;
                dco.add_frequency(f_offset);
            }
            n_dphase = 0;
            dphase_avg = 0.0;
        }
    }
    phase_last = phase;
}

void Satellite::phase(void)
{
    std::complex<float> iq = corr[offset_max];
    //
    // costas loop
    //
    // normalized error
    float mag2 = std::norm(iq);
    float error;
    if(mag2 == 0.0f){
        error = 0.0f;
    }else{
        error = 2.0*real(iq)*imag(iq)/mag2;
    }
    //
    // error = 1  phase error = 45degrees
    // error =-1  phase error =-45degrees
    // dtheta = 2*pi*df*dT
    // df = dtheta/(2*pi*dT)
    //    = pi/4/(2*pi*dT)
    //    1/8/dT

    if(!pll_error_stats.values(error))
        return;
    float mean;
    float sigma;
    float phi = arg(iq);
    float iq_sign = (iq.real()>=0.0f)?1.0f:-1.0f;
    if(phi>M_PI/2){
        phi = phi - M_PI;
    }else if(phi<-M_PI/2){
        phi = phi + M_PI;
    }
    bool valid_iq = abs(phi)<M_PI/4.0f;
    pll_error_stats.stats(mean, sigma);
    // if(rxstate == RXSTATE_PLL_ACQUIRE)
    //     printf("arg:% 3.1f phi:% 3.1f sign:%d valid_iq:%d freq:% 4.2f\n",
    //            arg(iq)*180/M_PI, phi*180/M_PI, phase_positive, valid_iq, -dco.get_frequency());
    //printf("freq:%f\n", -dco.get_frequency());
    float dfreq = -mean*COSTAS_FREQ_MAX*COSTAS_FREQ_FACTOR;
    float dtheta = -mean*COSTAS_PHASE_MAX*COSTAS_PHASE_FACTOR;
    dco.add_frequency(dfreq);
    dco.advance_phase(dtheta);
    if(rxstate==RXSTATE_PLL_ACQUIRE){
        if(valid_iq){
            if(++n_valid_iq == 200){
                printf("PLL locked. satellite:%d\n", sat+1);
                rxstate = RXSTATE_BIT_ACQUIRE;
            }
        }else{
            n_valid_iq = 0;
        }
    }else{
        if(!valid_iq){
            if(++n_invalid_iq==10){
                printf("Invalid iq points. satellite:%d\n", sat+1);
                rxstate = RXSTATE_SIGNAL_LOST;
            }
        }else{
            n_invalid_iq=0;
        }
        if(rxstate==RXSTATE_BIT_ACQUIRE){
            if(bit_acquire_reset){
                bit_acquire_reset = false;
            }else{
                if(iq_sign!=iq_sign_last){
                    printf("Bit transition acquired. satellite:%d\n", sat+1);
                    n_periods = 0;
                    rxstate = RXSTATE_PREAMBLE_ACQUIRE;
                    sign_total = 0.0f;
                }
            }
            iq_sign_last = iq_sign;
        }else{
            sign_total += iq_sign;
            if(++n_periods == 20){
                sign_total /= 20;
                float sign_total_abs = abs(sign_total);
                if(sign_total_abs>0.0f){
                    int b = ((sign_total*bit_sign)>0.0f)?1:0;
                    register_bit(b);
                }else{
                    printf("Sign total was zero. satellite:%d\n", sat+1);
                    rxstate = RXSTATE_BIT_ACQUIRE;
                    bit_acquire_reset = true;
                    first_subframe_processed = false;
                }
                n_periods = 0;
                sign_total = 0.0f;
            }
        }
    }
}

void Satellite::register_bit(int b)
{
    //printf("Bit:%b\n", b);
    if(rxstate == RXSTATE_PREAMBLE_ACQUIRE){
        preamble_buff <<= 1;
        preamble_buff |= b;
        preamble_buff &= WORD_MASK;
        int polarity;
        if(lnav.tlm_test(preamble_buff, polarity)){
            bit_sign *= (polarity)?-1.0f:1.0f;
            subframe_bit_count = 31;
            printf("Preamble found. Satellite:%2d polarity:%d\n", sat+1, polarity);
            if(polarity){
                preamble_buff = (~preamble_buff)&WORD_MASK;
            }
            lnav.subframe_raw[0] = preamble_buff;
            rxstate = RXSTATE_SUBFRAME_ACQUIRE;
        }
    }else if(rxstate == RXSTATE_SUBFRAME_ACQUIRE){
        lnav.subframe_set_bit(b, subframe_bit_count);
        subframe_bit_count++;
        if(subframe_bit_count>BITS_PER_SUBFRAME){
            printf("Received a subframe.\n");
            if(lnav.subframe_decode(subframe, sample_index)){
                printf("Decoded a subframe. subframe:%d\n", subframe);
                if(subframe == 1){
                    first_subframe_processed = true;
                }else if(subframe==5 && first_subframe_processed){
                    int page;
                    lnav.frame_decode(page);
                    printf("Decoded a frame. page:%d\n", page);
                    SatelliteFix fix = lnav.calculate_position(subframe);
                    gpsrx.triangulator.send_add_message(sat, fix);
                }
            }else{
                rxstate = RXSTATE_SIGNAL_LOST;
            }
            subframe_bit_count = 1;
        }
    }
}
