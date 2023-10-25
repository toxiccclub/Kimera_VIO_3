cmake_minimum_required (VERSION 3.5)
project(iCentral_Capture VERSION 1.0 LANGUAGES CXX)

set(CMAKE_VERBOSE_MAKEFILE ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/../../cmake)

if(NOT PEVENTS_INCLUDED)
add_subdirectory(${CMAKE_SOURCE_DIR}/../../3rd_party/neosmart ${CMAKE_CURRENT_BINARY_DIR}/neosmart)
endif()

include(FindPkgConfig)

find_package(iCentral REQUIRED)
find_package(Boost REQUIRED COMPONENTS atomic filesystem program_options)
find_package(fmt REQUIRED)
pkg_check_modules(MAGICKPP REQUIRED Magick++)

add_library(odometry-capture SHARED)

target_sources(odometry-capture
    PRIVATE
    ${CMAKE_SOURCE_DIR}/src/AcquisitionThread.cpp
    ${CMAKE_SOURCE_DIR}/src/CameraSettings.cpp
    ${CMAKE_SOURCE_DIR}/src/CaptureService.cpp
    ${CMAKE_SOURCE_DIR}/src/CaptureSettings.cpp
    ${CMAKE_SOURCE_DIR}/src/DataAccumulator.cpp
    ${CMAKE_SOURCE_DIR}/src/DataChannel.cpp
    ${CMAKE_SOURCE_DIR}/src/Device.cpp
    ${CMAKE_SOURCE_DIR}/src/DeviceIMU.cpp
    ${CMAKE_SOURCE_DIR}/src/DeviceList.cpp
    ${CMAKE_SOURCE_DIR}/src/log.cpp
    PUBLIC
    ${CMAKE_SOURCE_DIR}/include/AcquisitionThread.h
    ${CMAKE_SOURCE_DIR}/include/CameraSettings.h
    ${CMAKE_SOURCE_DIR}/include/CaptureService.h
    ${CMAKE_SOURCE_DIR}/include/DataAccumulator.h
    ${CMAKE_SOURCE_DIR}/include/DataChannel.h
    ${CMAKE_SOURCE_DIR}/include/Device.h
    ${CMAKE_SOURCE_DIR}/include/DeviceIMU.h
    ${CMAKE_SOURCE_DIR}/include/DeviceList.h
    ${CMAKE_SOURCE_DIR}/include/FrameHeader.h
    ${CMAKE_SOURCE_DIR}/include/log.h
    )

target_link_libraries(odometry-capture PUBLIC iCentral fmt pevents glog Boost::atomic Boost::filesystem)
target_include_directories(odometry-capture PUBLIC ${CMAKE_SOURCE_DIR}/include SYSTEM ${Boost_INCLUDE_DIR})

add_executable(capturer)
target_sources(capturer PRIVATE ${CMAKE_SOURCE_DIR}/tools/main_capture.cpp)
target_link_libraries(capturer odometry-capture Boost::program_options)


add_executable(enumerator)
target_sources(enumerator PRIVATE ${CMAKE_SOURCE_DIR}/tools/main_enumerate.cpp)
target_link_libraries(enumerator odometry-capture)

add_executable(decode)
target_sources(decode PRIVATE
${CMAKE_SOURCE_DIR}/tools/main_decode.cpp
${CMAKE_SOURCE_DIR}/tools/MagickppWrapper.cpp
${CMAKE_SOURCE_DIR}/tools/MagickppWrapper.h)
target_link_libraries(decode odometry-capture ${MAGICKPP_LIBRARIES} Boost::program_options)
target_include_directories(decode PRIVATE ${MAGICKPP_INCLUDE_DIRS})