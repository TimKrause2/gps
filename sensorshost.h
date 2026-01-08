#ifndef SENSORSHOST_H
#define SENSORSHOST_H

#include <memory>
#include "queue.h"
#include "timer.h"

enum SensorMsgType
{
    SMT_ADD,
    SMT_DEL,
    SMT_DATA,
    SMT_RENDER
};

struct SensorMsg
{
    SensorMsgType type;
    SensorMsg(SensorMsgType type):
        type(type) {}
};

struct SensorMsgSat : SensorMsg
{
    int sat;
    SensorMsgSat(SensorMsgType type, int sat):
        SensorMsg(type), sat(sat){}
};


#define SENSOR_SEPERATION 2.0f
#define SENSOR_Z  0.0f
#define CONSTELLATION_N_POINTS 500


struct SensorMsgAdd : SensorMsgSat
{
    SensorMsgAdd(int sat):
        SensorMsgSat(SMT_ADD, sat){}
};

struct SensorMsgDel : SensorMsgSat
{
    SensorMsgDel(int sat):
        SensorMsgSat(SMT_DEL, sat){}
};

struct SensorMsgData : SensorMsgSat
{
    int N_data;
    std::unique_ptr<std::complex<float>[]> iq;
    SensorMsgData(int sat,
                  int N_data,
                  std::unique_ptr<std::complex<float>[]> iq)
        : SensorMsgSat(SMT_DATA, sat),
        N_data(N_data), iq(std::move(iq))
    {
    }
};

struct SensorMsgRender : SensorMsg
{
    SensorMsgRender():
        SensorMsg(SMT_RENDER){}
};

class Sensors;

class SensorsHost
{
    std::unique_ptr<Sensors> sensors;
    ThreadQueue<std::unique_ptr<SensorMsg>> queue;
    std::thread thread;
    Timer timer;
    void timer_cb(void);
    void thread_func(void);
public:
    SensorsHost();
    ~SensorsHost();
    void send_add_sat(int sat);
    void send_del_sat(int sat);
    void send_sat_data(int sat, int N_data, std::unique_ptr<std::complex<float>[]> iq);
    void send_render(void);
};

#endif // SENSORSHOST_H
