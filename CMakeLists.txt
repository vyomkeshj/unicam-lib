cmake_minimum_required(VERSION 3.15)
project(libunicam)

set(CMAKE_CXX_STANDARD 14)


SET(MY_DIR ${libunicam_SOURCE_DIR})


message (${libunicam_SOURCE_DIR})

FIND_PACKAGE(PkgConfig)    # try find PKGConfig as it will be used if found
LIST(APPEND CMAKE_MODULE_PATH ${freenect2_ROOT_DIR}/cmake_modules) # FindGLFW3.cmake

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

FIND_PACKAGE(realsense2 REQUIRED)


find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV 4 REQUIRED)


include_directories(
        ${OpenCV_INCLUDE_DIRS}
        /home/rob-ot/Downloads/ASUS-Linux-x64-OpenNI2.2/Include
        /home/rob-ot/Documents/KinectLibs/libfreenect2/include
        ${realsense_INCLUDE_DIR}
)

set(LINK_LIBS  ${OpenCV_LIBRARIES} ${realsense2_LIBRARY}
        /home/rob-ot/Bin/
        /home/rob-ot/Bin/OpenNI2/Drivers/
        /home/rob-ot/Documents/KinectLibs/libfreenect2/cmake-build-debug/lib/
        )
link_directories(${LINK_LIBS})

add_executable(libunicam
        ./headers/async_buf.h
        ./headers/io_interface.h
        headers/RealsenseProvider.h
        headers/RealsenseDevice.h
        ./src/io_interface.cxx
        main.cpp headers/unicam/UnicamApi.h
        headers/unicam/UnicamCamera.h
        headers/unicam/UnicamDeviceProvider.h
        headers/unicam/UnicamDevices.h
        src/RealsenseProvider.cpp
        src/RealsenseDevice.cpp
        headers/RealsenseCamera.h
        headers/XtionCamera.h headers/XtionProvider.h
        src/XtionProvider.cpp
        src/XtionCamera.cpp
        src/CameraOrientationController.cpp
        headers/orientationControl/CameraOrientationController.h
        headers/freenectCamera.h
        headers/FreenectProvider.h
        src/freenectCamera.cpp
        src/FreenectProvider.cpp headers/frame_data.h)

target_link_libraries(libunicam ${LINK_LIBS}
                                libSenDuck.so
                                libOpenNI2.so
                                libOniFile.so
                                libfreenect2.so.0.2)
