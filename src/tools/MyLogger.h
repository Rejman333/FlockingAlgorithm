#pragma once

#include <vector>
#include <chrono>
#include <fstream>
#include <string>
#include "methods.h"

class MyLogger {
public:
    MyLogger(int boidCount, METHOD method, const std::string& filename = "logs.csv");
    ~MyLogger();

    void tick(); // raz na klatkę
    void recordBuildTime(double milliseconds);
    void recordRetrievalTime(double milliseconds);

    void recordCheckTime(double microseconds);

    void record_k_meansTimesTime(double microseconds);

private:
    void report();
    double average(const std::vector<double>& v) const;

    int boidCount;
    METHOD method;
    std::string filename;
    std::ofstream logFile;

    int frameCount;
    double minFPS;
    std::vector<int> fpsHistory;
    std::chrono::steady_clock::time_point lastSecondTime;
    std::chrono::steady_clock::time_point lastReportTime;

    std::vector<double> buildTimes;
    std::vector<double> retrievalTimes;
    std::vector<double> checkTimes;
    std::vector<double> k_meansTimes;

};
