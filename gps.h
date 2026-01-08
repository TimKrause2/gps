#pragma once

#include "prns.h"
#include "ssiq.h"
#include "satellite.h"
#include "search.h"
#include "triangulate.h"
#include "sensorshost.h"
#include <list>

struct GPSRx
{
    int fs;
    int samples_per_buffer;
    int buffer_index;
    long sample_index;
    std::shared_ptr<SSIQ> ssiq;
    PRNS prns;
    Triangulator triangulator;
    std::unique_ptr<Search> search;
    std::unique_ptr<SensorsHost> sHost;
    std::list<std::unique_ptr<Satellite>> satellites;
public:
    GPSRx(int fs);

    void evaluate(std::complex<float> x);
    void select_satellites(void);
};
