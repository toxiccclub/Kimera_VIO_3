cmake_minimum_required (VERSION 3.5)
project(pevents VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(pevents SHARED)

target_sources(pevents
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/pevents/src/pevents.cpp
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/pevents/src/pevents.h
    )
target_compile_definitions(pevents PUBLIC WFMO)
target_include_directories(pevents PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/pevents/src)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)

set(tests "")

FILE(GLOB children RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/pevents/tests" "${CMAKE_CURRENT_SOURCE_DIR}/pevents/tests/*.cpp")
FOREACH(child ${children})
    LIST(APPEND tests ${child})
ENDFOREACH()

FOREACH(test ${tests})
    get_filename_component(test_target ${test} NAME_WLE)
  MESSAGE(STATUS "Add test ${CMAKE_CURRENT_SOURCE_DIR}/pevents/tests/${test} as ${test_target}")
  add_executable(${test_target} ${CMAKE_CURRENT_SOURCE_DIR}/pevents/tests/${test})
  target_link_libraries(${test_target} pevents)
ENDFOREACH()

set(tests "")

FILE(GLOB children RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/pevents/examples" "${CMAKE_CURRENT_SOURCE_DIR}/pevents/examples/*.cpp")
FOREACH(child ${children})
    LIST(APPEND tests ${child})
ENDFOREACH()

FOREACH(test ${tests})
    get_filename_component(test_target ${test} NAME_WLE)
  MESSAGE(STATUS "Add test ${CMAKE_CURRENT_SOURCE_DIR}/pevents/examples/${test} as ${test_target}")
  add_executable(${test_target} ${CMAKE_CURRENT_SOURCE_DIR}/pevents/examples/${test})
  target_link_libraries(${test_target} pevents)
ENDFOREACH()

else()
set(PEVENTS_INCLUDED ON)
endif()