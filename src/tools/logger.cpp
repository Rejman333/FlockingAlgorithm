#include <fstream>
#include <sstream>
#include "logger.h"

logger::logger(const LogConfig& cfg)
    : config(cfg),
      file("../logs/" + cfg.method_name + "_" + std::to_string(cfg.number_of_boids) + ".csv")
{
    file << "AvgFPS10s,LowestFPS,BuildTime(ms),RetrievalTime(ms)\n";
    last_log_time = std::chrono::high_resolution_clock::now();
}

logger::~logger() noexcept
{
    if (file.is_open())
    {
        file.close();
    }
}

void logger::startBuildTimer()
{
    build_start = std::chrono::high_resolution_clock::now();
}

void logger::stopBuildTimer()
{
    auto end_time = std::chrono::high_resolution_clock::now();
    build_time = std::chrono::duration<double, std::milli>(end_time - build_start).count(); //1 milli = 0,001sec
}

void logger::startRetrievalTimer()
{
    retrieval_start = std::chrono::high_resolution_clock::now();
}

void logger::stopRetrievalTimer()
{
    auto end_time = std::chrono::high_resolution_clock::now();
    retrieval_time = std::chrono::duration<double, std::milli>(end_time - retrieval_start).count();
}

void logger::updateInfoFPS(float fps)
{
    fps_sum += fps;
    frame_count++;
    if (fps > 0.0f && fps < fps_min)
    {
        fps_min = fps;
    }
}

void logger::saveToFile()
{
    if (frame_count == 0) return; // avoid div by zero and misuse because default value for frameCount is 0

    float avg_fps = fps_sum / frame_count;

    std::ostringstream data_line;
    data_line.precision(3);
    data_line << std::fixed; // force normal notation

    data_line << avg_fps << ","
        << fps_min << ","
        << build_time << ","
        << retrieval_time << "\n";

    file << data_line.str();
    fps_sum = 0.0;
    fps_min = std::numeric_limits<float>::max();
    frame_count = 0;
    last_log_time = std::chrono::high_resolution_clock::now();
}

void logger::tick(float fps)
{
    updateInfoFPS(fps);
    auto now = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(now - last_log_time).count();

    if (elapsed >= config.log_interval_seconds)
    {
        saveToFile();
    }
}