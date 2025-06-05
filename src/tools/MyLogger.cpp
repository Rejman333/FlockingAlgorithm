#include "MyLogger.h"
#include <iomanip>
#include <iostream>
#include <ctime>
#include <chrono>
#include <stdexcept>
#include "methods.h" // method_to_string

MyLogger::MyLogger(int boidCount, METHOD method, const std::string& filename)
    : boidCount(boidCount), method(method), filename(filename),
      frameCount(0), minFPS(1000.0)
{
    lastSecondTime = lastReportTime = std::chrono::steady_clock::now();
    logFile.open(filename); // nadpisuje plik

    if (!logFile.is_open()) {
        throw std::runtime_error("Failed to open log file: " + filename);
    }

    logFile << "Timestamp,Boids,Method,AvgFPS,MinFPS,AvgBuildTime(us),AvgRetrievalTime(us),AvgCheckTime(us)\n";
}

MyLogger::~MyLogger() {
    if (logFile.is_open()) {
        logFile.close();
    }
}

void MyLogger::tick() {
    using namespace std::chrono;

    frameCount++;
    auto now = steady_clock::now();
    duration<double> elapsed = now - lastSecondTime;

    if (elapsed.count() >= 1.0) {
        double fps = frameCount / elapsed.count();
        minFPS = std::min(minFPS, fps);
        fpsHistory.push_back(static_cast<int>(fps));
        frameCount = 0;
        lastSecondTime = now;
    }

    if (duration<double>(now - lastReportTime).count() >= 10.0) {
        report();
        lastReportTime = now;
        fpsHistory.clear();
        buildTimes.clear();
        retrievalTimes.clear();
        checkTimes.clear();
        minFPS = 1000.0;
    }
}

void MyLogger::recordBuildTime(double microseconds) {
    buildTimes.push_back(microseconds);
}

void MyLogger::recordRetrievalTime(double microseconds) {
    retrievalTimes.push_back(microseconds);
}

void MyLogger::recordCheckTime(double microseconds) {
    checkTimes.push_back(microseconds);
}

void MyLogger::record_k_meansTimesTime(double microseconds) {
    k_meansTimes.push_back(microseconds);
}

void MyLogger::report() {
    double avgFPS = fpsHistory.empty() ? 0.0 : average(std::vector<double>(fpsHistory.begin(), fpsHistory.end()));
    double avgBuild = average(buildTimes);
    double avgRetrieval = average(retrievalTimes);
    double avgCheck = average(checkTimes);

    // Timestamp
    std::time_t now = std::time(nullptr);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&now));

    logFile << buf << ","
            << boidCount << ","
            << method_to_string(method) << ","
            << std::fixed << std::setprecision(2)
            << avgFPS << ","
            << minFPS << ","
            << avgBuild << ","
            << avgRetrieval << ","
            << avgCheck << "\n";

    logFile.flush();
}

double MyLogger::average(const std::vector<double>& v) const {
    if (v.empty()) return 0.0;
    double sum = 0;
    for (double x : v) sum += x;
    return sum / v.size();
}
