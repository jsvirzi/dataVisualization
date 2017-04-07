cmake_minimum_required(VERSION 3.5)
project(dataVisualization)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -m64 -std=c++11 -g -Wall -Wextra -Wl,--no-as-needed -I/home/jsvirzi/root/include")

set(SOURCE_FILES src/main.cpp src/lidar.cpp)

# execute_process(COMMAND pkg-config --libs opencv OUTPUT_VARIABLE OPENCVLIBS)
# execute_process(COMMAND pkg-config --cflags opencv OUTPUT_VARIABLE OPENCVINCS)
# include(${OPENCVINCS})

include_directories(src)
include_directories(/usr/local/include/opencv)
include_directories(/usr/local/include)

# link_libraries(${OPENCVLIBS})

link_libraries(-L/usr/local/lib -lopencv_shape -lopencv_stitching -lopencv_objdetect -lopencv_superres -lopencv_videostab -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_video -lopencv_photo -lopencv_ml -lopencv_imgproc -lopencv_flann -lopencv_core)
link_libraries(-L/home/jsvirzi/root/lib -lCore -lCint -lRIO -lNet -lHist -lGraf -lGraf3d -lGpad -lTree -lRint -lPostscript -lMatrix -lPhysics -lMathCore -lThread -pthread -lm -ldl -rdynamic)

add_executable(dataVisualization src/main.cpp src/lidar.cpp src/pcapreader.cpp src/utils.cpp)
add_executable(daqToRoot src/daqToRoot.cpp src/lidar.cpp src/daqToRoot.cpp src/pcapreader.cpp src/utils.cpp src/gpsReferenceWeekConverter.cpp)