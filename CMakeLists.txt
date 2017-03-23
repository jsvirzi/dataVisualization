cmake_minimum_required(VERSION 3.5)
project(DisplayFrm)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
    src/display_video.cpp)

execute_process(COMMAND pkg-config --libs opencv OUTPUT_VARIABLE OPENCVLIBS)
execute_process(COMMAND pkg-config --cflags opencv OUTPUT_VARIABLE OPENCVINCS)

include(${OPENCVINCS})
target_link_libraries(${OPENCVLIBS})

add_executable(DisplayFrm ${SOURCE_FILES})