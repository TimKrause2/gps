#include "sensorshost.h"
#include "sensors.h"

#define WINDOW_WIDTH 1000
#define WINDOW_HEIGHT 800
#define WINDOW_TITLE "GPS Sensors"

SensorsHost::SensorsHost()
{
    timer.set_callback(std::bind(&SensorsHost::timer_cb, this));
    timer.create();
    timer.set_time(1.0/60.0, 0.5);

    thread = std::thread(&SensorsHost::thread_func, this);
}

SensorsHost::~SensorsHost()
{
    timer.set_time(0.0, 0.0);
    queue.stop();
    thread.join();
}

void SensorsHost::timer_cb(void)
{
    send_render();
}

void SensorsHost::thread_func(void)
{
    // allocate the Sensors
    std::string title = WINDOW_TITLE;
    sensors.reset(new Sensors(WINDOW_WIDTH, WINDOW_HEIGHT, title));

    while(sensors->readyToFinish() == false){
        std::unique_ptr<SensorMsg> msg = queue.pop();
        if(!msg)
            break;
        switch(msg->type){
        case SMT_ADD:
            sensors->add_sat(static_cast<SensorMsgAdd*>(msg.get()));
            break;
        case SMT_DEL:
            sensors->del_sat(static_cast<SensorMsgDel*>(msg.get()));
            break;
        case SMT_DATA:
            sensors->sat_data(static_cast<SensorMsgData*>(msg.get()));
            break;
        case SMT_RENDER:
            sensors->render();
            break;
        }
    }
}

void SensorsHost::send_add_sat(int sat)
{
    queue.push(std::make_unique<SensorMsgAdd>(sat));
}

void SensorsHost::send_del_sat(int sat)
{
    queue.push(std::make_unique<SensorMsgDel>(sat));
}

void SensorsHost::send_sat_data(int sat, int N_data, std::unique_ptr<std::complex<float>[]> iq)
{
    queue.push(std::make_unique<SensorMsgData>(sat, N_data, std::move(iq)));
}

void SensorsHost::send_render(void)
{
    queue.push(std::make_unique<SensorMsgRender>());
}
