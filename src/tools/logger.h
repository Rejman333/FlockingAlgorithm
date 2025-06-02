#include <iostream>
#include <string>
#include <chrono>
#include <fstream>
#ifndef LOGGER_H
#define LOGGER_H


struct LogConfig
{
    std::string method_name; //chosen algorithm
    //std::string mode = "default";
    int number_of_boids;
};

class logger
{
private:
    LogConfig config;
    std::ofstream file;

    float fps_sum = 0.0;
    float fps_min = 0.0;
    int frame_count = 0;

    std::chrono::high_resolution_clock::time_point build_start;
    std::chrono::high_resolution_clock::time_point retrieval_start;

    double build_time = 0.0;
    double retrieval_time = 0.0;

public:
    explicit logger(const LogConfig& cfg);
    ~logger();

    void startBuildTimer();
    void stopBuildTimer();

    void startRetrievalTimer();
    void stopRetrievalTimer();

    void updateInfoFPS(float fps);
    void saveToFile();

    std::chrono::high_resolution_clock::time_point last_log_time;
};


#endif //LOGGER_H
