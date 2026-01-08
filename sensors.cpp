#include "sensors.h"
#include <cmath>

#define SENSOR_SEPERATION 2.0f
#define SENSOR_Z  0.0f
#define CONSTELLATION_N_POINTS 500

Constellation::Constellation(mplot::Visual<> &v, int sat, int N_points, sm::vec<float> trans):
    v(v),
    N_points(N_points)
{
    abscissa.linspace(0.0f, 0.0f, N_points);
    ordinate.linspace(0.0f, 0.0f, N_points);
    data.reset(new std::complex<float>[N_points]);
    std::string name = std::format("Constellation for satellite {}", sat+1);
    auto gv = std::make_unique<mplot::GraphVisual<float>>(trans);
    v.bindmodel(gv);
    gv->setlimits(-1,1,-1,1);
    mplot::DatasetStyle ds(mplot::stylepolicy::markers);
    ds.datalabel = name;
    ds.markersize = 0.06f;
    ds.markerstyle = mplot::markerstyle::hexagon;
    ds.markercolour = mplot::colour::blue;
    gv->setdata(abscissa, ordinate, ds);
    gv->finalize();
    gvp = v.addVisualModel(gv);
    index = 0;
    for(int i=0;i<N_points;i++){
        data[i] = 0.0f;
    }
}

Constellation::~Constellation()
{
    v.removeVisualModel(gvp);
}

void Constellation::data_point(std::complex<float> x)
{
    data[index] = x;
    if(++index==N_points){
        index = 0;
    }
}

void Constellation::update(void)
{
    // determine the maximum absolute magnitude
    float max_abs = 0.0f;
    for(int i=0;i<N_points;i++){
        float data_abs = std::abs(data[i]);
        if(data_abs > max_abs){
            max_abs = data_abs;
        }
    }
    if(max_abs==0.0f){
        max_abs = 1.0f;
    }

    // scale and fill the graph vectors
    for(int i=0;i<N_points;i++){
        abscissa[i] = data[i].real()/max_abs;
        ordinate[i] = data[i].imag()/max_abs;
    }

    gvp->update(abscissa, ordinate, 0);
}



Sensors::Sensors(int width, int height, std::string &title):
    v(width, height, title)
{
    for(int s=0;s<N_SATELLITES;s++){
        sat_slot[s] = -1;
    }
}

void Sensors::add_sat(SensorMsgAdd *msg)
{
    // test for already allocated
    if(sat_slot[msg->sat]>=0)
        return;

    // find the next available slot
    int slot;
    for(slot=0;slot<N_SATELLITES;slot++){
        if(!slots[slot])
            break;
    }

    // allocate the sensors for the slot
    float trans_x = slot*SENSOR_SEPERATION;
    float trans_y = 0.0f;
    float trans_z = SENSOR_Z;
    slots[slot].reset(new SensorSlot(v,
                                     msg->sat,
                                     CONSTELLATION_N_POINTS,
                                     sm::vec<float>{trans_x, trans_y, trans_z}));
    sat_slot[msg->sat] = slot;
}

void Sensors::del_sat(SensorMsgDel *msg)
{
    // check for already deallocated
    if(sat_slot[msg->sat]<0)
        return;

    int slot = sat_slot[msg->sat];
    // deallocate this slot
    slots[slot].reset(nullptr);
    // indicate that this slot is available
    sat_slot[slot] = -1;
}

void Sensors::sat_data(SensorMsgData *msg)
{
    // check for a valid slot
    int slot = sat_slot[msg->sat];
    if(slot<0)
        return;

    for(int i=0;i<msg->N_data;i++){
        slots[slot]->constellation.data_point(msg->iq[i]);
    }
    slots[slot]->constellation.update();
}

void Sensors::render(void)
{
    v.render();
}

bool Sensors::readyToFinish(void)
{
    return v.readyToFinish();
}

