#pragma once

#include <memory>
#include <cmath>

struct MovingAvg
{
    int N_points;
    std::unique_ptr<float[]> data;
    int index;
    int divisor;
    MovingAvg(int N_points):
        N_points(N_points),
        data(new float[N_points])
    {
        for(int i=0;i<N_points;i++)
            data[i] = 0.0f;
        index = 0;
    }
    float evaluate(float x){
        data[index] = x;
        index++;
        if(index == N_points)
            index = 0;
        float avg = 0.0f;
        for(int i=0;i<N_points;i++){
            avg += data[i];
        }
        avg /= N_points;
        return avg;
    }
};

struct MovingStats
{
    int N_points;
    std::unique_ptr<float[]> data;
    int index;
    bool full;

    MovingStats(int N_points):
        N_points(N_points),
        data(new float[N_points])
    {
        for(int i=0;i<N_points;i++)
            data[i] = 0.0f;
        index = 0;
        full = false;
    }

    bool values(float x){
        data[index] = x;
        if(++index == N_points){
            index = 0;
            full = true;
        }
        return full;
    }

    void stats(float &mean, float &sigma){
        mean = 0.0f;
        for(int i=0;i<N_points;i++){
            mean += data[i];
        }
        mean /= N_points;
        sigma = 0.0f;
        for(int i=0;i<N_points;i++){
            float d = data[i] - mean;
            sigma += d*d;
        }
        sigma /= N_points;
        sigma = std::sqrt(sigma);
    }
};






