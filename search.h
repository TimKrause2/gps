#pragma once

#include "constants.h"

#include <thread>
#include <memory>
#include <complex>
#include <list>
#include <fftw3.h>

#define N_EPOCHS 10
#define F_RANGE 7000
#define F_DELTA 50
#define N_FREQ ((F_RANGE*2/F_DELTA)+1)
#define SEC_PER_TRIGGER 10

struct GPSRx;

struct SearchResult
{
    int sat;
    float ratio;
    float freq;
    SearchResult(int sat, float ratio, float freq):
        sat(sat), ratio(ratio), freq(freq){}
};

struct Search
{
    int fs;
    GPSRx &gpsrx;
    int rx_index;
    int trigger_index;
    int samples_per_period;
    int samples_per_trigger;
    int buff_size;
    bool receiving;
    bool scanning;
    bool scan_done;

    std::thread scan_thread;

    std::unique_ptr<std::complex<float>[]> rx;
    std::unique_ptr<std::complex<float>[]> rx_conv;
    std::unique_ptr<std::complex<float>[]> rx_conv_fft[N_EPOCHS];
    std::unique_ptr<std::complex<float>[]> freq_prod;
    std::unique_ptr<std::complex<float>[]> corr;
    std::unique_ptr<float[]> corr_acc;
    std::unique_ptr<float[]> ratios;

    std::list<SearchResult> found;

    fftwf_plan plan_rx[N_EPOCHS];
    fftwf_plan plan_corr;

    void convert_rx(float f);
    void scan(void);
    void results(void);
    void start_scan(void);

public:
    Search(GPSRx &gpsrx, int fs);
    void evaluate(std::complex<float> x);
};
