cmake_minimum_required(VERSION 3.20)
project(old)

set(CMAKE_CXX_STANDARD 20)

#add_executable(old main.cpp)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND ORB_SLAM_ROOT_DIR "/home/tgl/ORB_SLAM2")


set(PROJECT_HEADERS
        /home/tgl/ctello/include/ctello.h
        ~/ORB_SLAM2/include/System.h
        ~/opencv/modules/videoio/include/opencv2/videoio.hpp
        our_headers.h usefulFunctions.cpp)
set(PROJECT_SOURCES
        main.cpp
        /home/tgl/ctello/src/ctello.cpp
        ~/ORB_SLAM2/lib/libORB_SLAM2.so
        usefulFunctions.cpp)

include_directories(~/ORB_SLAM2/include)
include_directories(~/ORB_SLAM2)
include_directories(/usr/include/eigen3)
include_directories(/home/tgl/ctello/include)
include_directories(/usr/local/lib)
include_directories(~/ctello/src)
# include_directories(~/opencv/modules/videoio/include/opencv2/)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(ORB_SLAM REQUIRED)
#find_package(Pangolin REQUIRED)
add_executable(old ${PROJECT_SOURCES} ${PROJECT_HEADERS} usefulFunctions.cpp)
set(LIBS ${OpenCV_LIBS} ${OpenGL_LIBS} ${GLEW_LIBS}  ${Pangolin_LIBRARIES} ${/home/tgl/ORB_SLAM2/lib/libORB_SLAM2.so} ${ORB_SLAM_LIBRARIES} -lGLU -lGL)
target_link_libraries(old ${LIBS})

