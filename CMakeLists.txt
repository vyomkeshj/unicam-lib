cmake_minimum_required(VERSION 3.15)
project(libunicam)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_LIST_DIR}/cmake)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_LIST_DIR}/freenect2/libfreenect2)


SET(MY_DIR ${libunicam_SOURCE_DIR})

SET(freenect2_DIR ${MY_DIR}/freenect2/libfreenect2/cmake-build-debug/)

message (${libunicam_SOURCE_DIR})
# The example build system is standalone and will work out-of-tree with these files copied
SET(freenect2_ROOT_DIR ${MY_DIR}/freenect2/libfreenect2/)
SET(flextGL_SOURCES ${freenect2_ROOT_DIR}/src/flextGL.cpp)
SET(flextGL_INCLUDE_DIRS ${freenect2_ROOT_DIR}/src) # for flextGL.h

FIND_PACKAGE(PkgConfig)    # try find PKGConfig as it will be used if found
LIST(APPEND CMAKE_MODULE_PATH ${freenect2_ROOT_DIR}/cmake_modules) # FindGLFW3.cmake



FIND_PACKAGE(freenect2 REQUIRED)
FIND_PACKAGE(realsense2 REQUIRED)


find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV 4 REQUIRED)
find_package(OpenNI2 REQUIRED)


include_directories(
        ${PC_OPENNI2_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${PC_OPENNI2_INCLUDE_DIRS}
        ${realsense_INCLUDE_DIR}
)

set(LINK_LIBS ${PC_OPENNI2_LIBRARIES} ${OpenCV_LIBRARIES} ${realsense2_LIBRARY})


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
        src/XtionCamera.cpp headers/XtionCamera.h src/XtionProvider.cpp headers/XtionProvider.h)
target_link_libraries(libunicam ${LINK_LIBS})
