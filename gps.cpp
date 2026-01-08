#include "gps.h"
#include <stdio.h>

GPSRx::GPSRx(int fs)
    :fs(fs), prns(fs), triangulator(fs)
{
    int samples_per_chip = fs/F_CHIP;
    if(samples_per_chip*F_CHIP != fs){
        printf("Sample rate is not an integer multiple of the chip rate, 1.023e6.\n");
        throw;
    }
    samples_per_buffer = fs/F_BUFFER;
    buffer_index = 0;
    sample_index = 0;
    search.reset(new Search(*this, fs));
    sHost.reset(new SensorsHost);
}

void GPSRx::evaluate(std::complex<float> x){
    if(buffer_index==0){
        ssiq.reset(new SSIQ(sample_index, samples_per_buffer));
    }
    ssiq->iq[buffer_index] = x;
    if(++buffer_index==samples_per_buffer){
        buffer_index = 0;
        // send this buffer to active satellites
        // and destroy inactive satellites
        std::list<std::unique_ptr<Satellite>>::iterator s_it = satellites.begin();
        for(;s_it!=satellites.end();){
            if((*s_it)->is_active()){
                (*s_it)->send_ssiq(ssiq);
                s_it++;
            }else{
                triangulator.send_del_message((*s_it)->sat);
                s_it = satellites.erase(s_it);
            }
        }
    }
    search->evaluate(x);
    sample_index++;
}

void GPSRx::select_satellites(void)
{
    // add new satellites to the list ignore satellites already there
    std::list<SearchResult>::iterator found_sat;
    for(found_sat = search->found.begin(); found_sat != search->found.end(); found_sat++){
        // if(found_sat->sat != 32-1)
        //     continue;
        // insert this satellite into the satellites list if it isn't already
        bool match = false;
        for(auto &sat : satellites){
            if(sat->sat == found_sat->sat){
                match = true;
                break;
            }
        }
        if(!match){
            printf("Adding satellite %d to the list.\n", found_sat->sat+1);
            satellites.push_front(
                std::unique_ptr<Satellite>(new Satellite(*this,
                                                         found_sat->sat,
                                                         fs,
                                                         found_sat->freq)));
        }
    }
}



#define FS 1023000*24
#include "test_sig.h"
#include <fstream>
#include <iostream>
#include <assert.h>

int main(void)
{
    GPSRx gpsrx(FS);
    TestSignal t_sig(FS, 32, 0.0f, N_PERIOD - 10, 40.0f, 20.0f);

    // while(true){
    //     gpsrx.evaluate(t_sig.evaluate());
    // }



    std::ifstream file("iq.fifo");
    if(!file.is_open()){
        std::cout << "couldn't open fifo." << std::endl;
        return 1;
    }

    while(true){
        std::complex<float> x;
        file.read(reinterpret_cast<char*>(&x), sizeof(x));
        if(file.fail()){
            std::cout << "failed to read file." << std::endl;
            return 1;
        }
        assert(!std::isnan(x.real()));
        assert(!std::isnan(x.imag()));

        gpsrx.evaluate(x);
    }

}

