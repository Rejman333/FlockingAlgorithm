#pragma once

#include <iostream>
#include <string>
#include <chrono>
#include <fstream>


struct LogConfig {
    std::string method_name; //chosen algorithm
    int number_of_boids;
    double log_interval_seconds = 10.0;
};

class logger {
private:
    LogConfig config;
    std::ofstream file;

    float fps_sum = 0.0;
    float fps_min = std::numeric_limits<float>::max();
    int frame_count = 0;

    std::chrono::high_resolution_clock::time_point build_start;
    std::chrono::high_resolution_clock::time_point retrieval_start;

    double build_time = 0.0;
    double retrieval_time = 0.0;

public:
    explicit logger(const LogConfig &cfg);

    ~logger();

    void startBuildTimer();

    void stopBuildTimer();

    void startRetrievalTimer();

    void stopRetrievalTimer();

    void updateInfoFPS(float fps);

    void saveToFile();

    void tick(float fps);


    std::chrono::high_resolution_clock::time_point last_log_time;
};
