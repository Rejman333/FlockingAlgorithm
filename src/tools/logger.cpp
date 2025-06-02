
#include <fstream>
#include "logger.h"

logger::logger(const LogConfig& cfg)
    : config(cfg){
        std::string filename = "../logs/" + config.method_name + "_" + std::to_string(config.number_of_boids) + ".csv";
        file.open(filename);
    file << "AvgFPS10s,LowestFPS,BuildTime(ms),RetrievalTime(ms)\n";//headers CSV
    last_log_time = std::chrono::high_resolution_clock::now();
    }

logger::~logger() {
    if (file.is_open()) {
        file.close();
    }
}

void logger::startBuildTimer(){
    build_start = std::chrono::high_resolution_clock::now();
}
void logger::stopBuildTimer(){
    auto end_time = std::chrono::high_resolution_clock::now();
    build_time = std::chrono::duration<double, std::milli>(end_time - build_start).count(); //1 milli = 0,001sec
}

void logger::startRetrievalTimer() {
    retrieval_start = std::chrono::high_resolution_clock::now();
}

void logger::stopRetrievalTimer() {
    auto end_time = std::chrono::high_resolution_clock::now();
    retrieval_time = std::chrono::duration<double, std::milli>(end_time - retrieval_start).count();
}

void logger::updateInfoFPS(float fps){
    fps_sum += fps;
    frame_count++;
    if (frame_count == 1 || fps < fps_min) {
        fps_min = fps;
    }
}

void logger::saveToFile(){
    if (frame_count == 0) return; // avoid div by zero and misuse because default value for frameCount is 0

    float avg_fps = fps_sum / frame_count;

    file << avg_fps << ","
         << fps_min << ","
         << build_time << ","
         << retrieval_time << "\n";


    fps_sum = 0.0;
    fps_min = 0.0;
    frame_count = 0;
    last_log_time = std::chrono::high_resolution_clock::now();
}


