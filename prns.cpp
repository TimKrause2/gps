#include "prns.h"
#include "lfsr.h"
#include <cmath>

PRNS::PRNS(int fs)
{
    int periods_per_sample = fs/F_CHIP;
    int samples_per_period = periods_per_sample*N_PERIOD;
    fftwf_plan plan;

    std::unique_ptr<std::complex<float>[]> prn(new std::complex<float>[samples_per_period]);
    for(int s=0;s<N_SATELLITES;s++){
        prns_fft[s].reset(new std::complex<float>[samples_per_period]);
        std::unique_ptr<CA> ca(new CA(s+1));
        int i=0;
        for(int c=0;c<N_PERIOD;c++){
            float x = (ca->advance())?1.0f:-1.0f;
            for(int j=0;j<periods_per_sample;j++){
                prn[i++] = x;
            }
        }
        plan = fftwf_plan_dft_1d(samples_per_period,
                                 reinterpret_cast<fftwf_complex*>(prn.get()),
                                 reinterpret_cast<fftwf_complex*>(prns_fft[s].get()),
                                 FFTW_FORWARD, FFTW_ESTIMATE);
        fftwf_execute(plan);
        fftwf_destroy_plan(plan);
        for(i=0;i<samples_per_period;i++){
            prns_fft[s][i] = std::conj(prns_fft[s][i]);
        }
    }
}

std::complex<float>* PRNS::prn_fft(int s)
{
    return prns_fft[s].get();
}
