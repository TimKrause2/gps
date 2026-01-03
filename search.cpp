#include "search.h"
#include "dco.h"
#include "gps.h"
#include <stdio.h>

Search::Search(GPSRx &gpsrx, int fs)
    :fs(fs), gpsrx(gpsrx)
{
    int samples_per_chip = fs/F_CHIP;
    samples_per_period = samples_per_chip * N_PERIOD;
    samples_per_trigger = fs*SEC_PER_TRIGGER;
    buff_size = samples_per_period*N_EPOCHS;
    rx_index = 0;
    trigger_index = 0;
    receiving = false;
    scanning = false;
    scan_done = false;

    rx.reset(new std::complex<float>[buff_size]);
    rx_conv.reset(new std::complex<float>[buff_size]);
    for(int i=0;i<N_EPOCHS;i++){
        rx_conv_fft[i].reset(new std::complex<float>[samples_per_period]);
    }
    freq_prod.reset(new std::complex<float>[samples_per_period]);
    corr.reset(new std::complex<float>[samples_per_period]);
    corr_acc.reset(new float[samples_per_period]);
    ratios.reset(new float[N_FREQ*N_SATELLITES]);

    std::complex<float> *rx_p = rx_conv.get();
    for(int e=0;e<N_EPOCHS;e++, rx_p+=samples_per_period){
        plan_rx[e] = fftwf_plan_dft_1d(
            samples_per_period,
            reinterpret_cast<fftwf_complex*>(rx_p),
            reinterpret_cast<fftwf_complex*>(rx_conv_fft[e].get()),
            FFTW_FORWARD, FFTW_ESTIMATE);
    }
    plan_corr = fftwf_plan_dft_1d(
        samples_per_period,
        reinterpret_cast<fftwf_complex*>(freq_prod.get()),
        reinterpret_cast<fftwf_complex*>(corr.get()),
        FFTW_BACKWARD, FFTW_ESTIMATE);

}

void Search::convert_rx(float f)
{
    DCO dco(fs);
    dco.set_frequency(-f);
    std::complex<float> *src = rx.get();
    std::complex<float> *dst = rx_conv.get();
    for(int i=0;i<buff_size;i++){
        *(dst++) = *(src++) * dco.evaluate();
    }
    for(int e=0;e<N_EPOCHS;e++){
        fftwf_execute(plan_rx[e]);
    }
}

void Search::scan(void)
{
    printf("Search::scan Starting scan.\n");
    float *ratio_p = ratios.get();

    float freq = -F_RANGE;
    for(int f=0;f<N_FREQ;f++, freq += F_DELTA){
        //printf("Search::scan freq:%f\n", freq);
        convert_rx(freq);
        for(int s=0;s<N_SATELLITES;s++){
            float *c_acc_p = corr_acc.get();
            for(int i=0;i<samples_per_period;i++){
                *(c_acc_p++) = 0.0f;
            }
            for(int epoch=0;epoch<N_EPOCHS;epoch++){
                std::complex<float> *prn_p = gpsrx.prns.prn_fft(s);
                std::complex<float> *rx_p = rx_conv_fft[epoch].get();
                std::complex<float> *prod_p = freq_prod.get();
                for(int i=0;i<samples_per_period;i++){
                    *(prod_p++) = *(prn_p++) * *(rx_p++);
                }
                fftwf_execute(plan_corr);
                c_acc_p = corr_acc.get();
                std::complex<float> *corr_p = corr.get();
                for(int i=0;i<samples_per_period;i++){
                    *(c_acc_p++) += std::abs(*(corr_p++));
                }
            }
            float abs_max = 0.0f;
            float abs_avg = 0.0f;
            c_acc_p = corr_acc.get();
            for(int i=0;i<samples_per_period;i++, c_acc_p++){
                if(*c_acc_p > abs_max){
                    abs_max = *c_acc_p;
                }
                abs_avg += *c_acc_p;
            }
            abs_avg /= samples_per_period;
            *(ratio_p++) = abs_max / abs_avg;
        }
    }
    results();
    scan_done = true;
}

void Search::results(void)
{
    found.clear();
    for(int s=0;s<N_SATELLITES;s++){
        float *ratio_p = &ratios[s];
        float freq = -F_RANGE;
        float ratio_max=0.0f;
        float freq_max;
        for(int f=0;f<N_FREQ;f++, freq+=F_DELTA, ratio_p+=N_SATELLITES){
            if(*ratio_p > ratio_max){
                ratio_max = *ratio_p;
                freq_max = freq;
            }
        }
        //printf("Search::results Satellite:%2d ratio_max:%7f\n", s+1, ratio_max);
        if(ratio_max>=3.0f){
            found.push_back(SearchResult(s, ratio_max, freq_max));
        }
    }
    for(auto &result:found){
        printf("Satellite:%2d ratio:%7.2f freq:%7.2f\n",
               result.sat+1, result.ratio, result.freq);
    }
}

void Search::start_scan(void)
{
    if(scanning)
        return;
    scan_done = false;
    scanning = true;
    scan_thread = std::thread(&Search::scan, this);
}

void Search::evaluate(std::complex<float> x)
{
    if(trigger_index == 0){
        receiving = true;
        rx_index = 0;
    }
    if(++trigger_index == samples_per_trigger){
        trigger_index = 0;
    }
    if(receiving){
        rx[rx_index] = x;
        if(++rx_index == buff_size){
            receiving = false;
            start_scan();
        }
    }
    if(scanning){
        if(scan_done){
            scan_thread.join();
            scanning = false;
            gpsrx.select_satellites();
        }
    }
}
