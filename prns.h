#pragma once

#include "constants.h"
#include <complex>
#include <fftw3.h>
#include <memory>

struct PRNS
{
    std::unique_ptr<std::complex<float>[]> prns_fft[N_SATELLITES];
    PRNS(int fs);
    std::complex<float> *prn_fft(int s);
};
