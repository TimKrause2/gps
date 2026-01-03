#pragma once

#include <complex>
#include <memory>

struct SSIQ // sample stamped iq data
{
    long sample_index;
    int  N_samples;
    std::unique_ptr<std::complex<float>[]> iq;
    SSIQ(long sample_index, int N_samples)
        :sample_index(sample_index),
        N_samples(N_samples),
        iq(new std::complex<float>[N_samples]){}
};

