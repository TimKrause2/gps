#ifndef SENSORS_H
#define SENSORS_H

#include <sm/vec>
#include <sm/vvec>
#include <mplot/Visual.h>
#include <mplot/GraphVisual.h>
#include <complex>
#include <thread>
#include "queue.h"
#include "timer.h"
#include "sensorshost.h"

#define N_SATELLITES 32

class Constellation
{
    mplot::Visual<> &v;
    int N_points;
    int index;
    mplot::GraphVisual<float> *gvp;
    sm::vvec<float> abscissa;
    sm::vvec<float> ordinate;
    std::unique_ptr<std::complex<float>[]> data;
public:
    Constellation(mplot::Visual<> &v, int sat, int N_points, sm::vec<float> trans);
    ~Constellation();
    void data_point(std::complex<float> x);
    void update(void);
};

class Frequency
{

};

struct SensorSlot
{
    Constellation constellation;
    SensorSlot(mplot::Visual<> &v, int sat, int N_points, sm::vec<float> trans):
        constellation(v, sat, N_points, trans){}
};

class Sensors
{
    mplot::Visual<> v;
    int sat_slot[N_SATELLITES];
    std::unique_ptr<SensorSlot> slots[N_SATELLITES];
public:
    void add_sat(SensorMsgAdd *msg);
    void del_sat(SensorMsgDel *msg);
    void sat_data(SensorMsgData *msg);
    void render(void);
    bool readyToFinish(void);
    Sensors(int width, int height, std::string &title);
};

#endif // SENSORS_H
