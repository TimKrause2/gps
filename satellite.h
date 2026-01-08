#pragma once

/*
 * Satellite tracking pipeline
 * 1) DCO
 * 2) offset compensation
 * 3) vectorize
 *      a) fft forward on the converted signal
 *      b) multiply frequency domain buffers
 *      c) fft reverse on the product to yield the correlation
 *      d) peek and offset of peek
 *      e) offset feedback to stage 2
 *      if(offset_acquired)
 *          f) phase discriminator and frequency feedback to stage 1
 *
 *
 *
 *
 *
 *
 *
 */

#include "dco.h"
#include "queue.h"
#include "ssiq.h"
#include "moving_avg.h"
#include "lnav.h"
#include <thread>
#include <fftw3.h>

struct GPSRx;

enum RxState
{
    RXSTATE_SIGNAL_LOST,
    RXSTATE_OFFSET_ACQUIRE,
    RXSTATE_FREQUENCY_ACQUIRE,
    RXSTATE_PLL_ACQUIRE,
    RXSTATE_BIT_ACQUIRE,
    RXSTATE_PREAMBLE_ACQUIRE,
    RXSTATE_SUBFRAME_ACQUIRE
};

struct Satellite
{
    GPSRx &gpsrx;
    int sat;
    long sample_index;
    int samples_per_period;
    int buffer_index;
    int offset;
    int offset_max;
    int offset_range_max;
    int n_valid_offsets;
    int n_invalid_offsets;
    int n_dphase;
    bool phase_reset;
    float phase_last;
    float dphase_avg;
    int n_periods;
    int n_valid_iq;
    int n_invalid_iq;
    bool bit_acquire_reset;
    float iq_sign_last;
    float sign_total;
    float bit_sign;
    int preamble_buff;
    int subframe_bit_count;
    int subframe;
    bool first_subframe_processed;
    MovingAvg  f_offset_avg;
    MovingStats pll_error_stats;
    RxState rxstate;
    std::complex<float> x_in;
    std::unique_ptr<std::complex<float>[]> rx_buff;
    std::unique_ptr<std::complex<float>[]> rx_buff_fft;
    std::unique_ptr<std::complex<float>[]> prod_fft;
    std::unique_ptr<std::complex<float>[]> corr;
    std::unique_ptr<std::complex<float>[]> sensor_iq;
    int sensor_iq_index;
    fftwf_plan rx_plan;
    fftwf_plan corr_plan;
    std::thread sat_thread;
    ThreadQueue<std::shared_ptr<SSIQ>> queue;
    DCO dco;
    LNAV lnav;
public:
    Satellite(GPSRx &gpsrx, int sat, int fs, float freq);
    ~Satellite();
    bool is_active(void);
    void send_ssiq(std::shared_ptr<SSIQ> ssiq);
    void thread_func(void);
    void evaluate(void); // expects x_in to be set
    void period(void);
    double fix_angle_range(double angle);
    void frequency(void);
    void phase(void);
    void register_transition(int t);
    void register_bit(int b);
    void sensor_iq_evaluate(std::complex<float> x);
};


