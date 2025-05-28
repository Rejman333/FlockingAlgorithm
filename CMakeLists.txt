cmake_minimum_required(VERSION 3.29)
project(Boids)

set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

FetchContent_Declare(
        raylib
        GIT_REPOSITORY https://github.com/raysan5/raylib.git
        GIT_BRANCH master
)


FetchContent_MakeAvailable(raylib)


add_executable(Boids src/main.cpp
        src/data_structures/hash_table.cpp
        src/data_structures/hash_table.h)
target_link_libraries(${PROJECT_NAME} raylib)
