#include "lfsr.h"
#include "dco.h"
#include <random>
#include <complex>
#include <chrono>

#define RAND_MEAN 0.0
#define RAND_STD  0.001
#define F_CHIP 1023000
#define BUFFER_RATE 10

class TestSignal
{
    float fs;
    int   sat_id;
    DCO   dco;
    CA    ca;
    int   chip_index;
    int   samples_per_chip;
    float chip_value;
    int   samples_per_buffer;
    int   buffer_index;
    float freq;
    float dfreq;
    float dT;
    int freq_index;
    int samples_per_ramp;
    bool ramp_up;
    std::mt19937 gen;
    std::normal_distribution<float> dist;
    std::chrono::steady_clock::time_point target_time;

public:
    TestSignal(float fs, int sat_id, float freq=0.0f, int advance=0, float dfreq=0.0f, float dT=0.0f);
    std::complex<float> evaluate(void);
};
