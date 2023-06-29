cmake_minimum_required(VERSION 3.16)
project(iCentral_grab_demo)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)

#set(CMAKE_VERBOSE_MAKEFILE ON)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake")

find_package(iCentral REQUIRED)


if(EXISTS "${iCenral_Install_Dir}/share/C")

message(STATUS "Adding CLI C++ samples")

set(samples "")

FILE(GLOB children RELATIVE "${iCenral_Install_Dir}/share/C" "${iCenral_Install_Dir}/share/C/*")
FOREACH(child ${children})
  IF(IS_DIRECTORY "${iCenral_Install_Dir}/share/C/${child}")
    LIST(APPEND samples ${child})
  ENDIF()
ENDFOREACH()


FOREACH(sample ${samples})
  MESSAGE(STATUS "Add sample ${iCenral_Install_Dir}/share/C/${sample} as ${sample}")
  add_executable(${sample} "${iCenral_Install_Dir}/share/C/${sample}/${sample}.cpp")
  target_link_libraries(${sample} iCentral)
ENDFOREACH()

endif()

# if(EXISTS "${iCenral_Install_Dir}/share/QT" AND iCentral_WITH_GUI)
#   set(samples "")
#   message(STATUS "Adding QT5 C++ samples")

#   FILE(GLOB children RELATIVE "${iCenral_Install_Dir}/share/QT" "${iCenral_Install_Dir}/share/QT/*")
#     FOREACH(child ${children})
#     IF(IS_DIRECTORY "${iCenral_Install_Dir}/share/QT/${child}")
#       LIST(APPEND samples ${child})
#     ENDIF()
#   ENDFOREACH()

#   FOREACH(sample ${samples})
#     MESSAGE(STATUS "Add sample ${iCenral_Install_Dir}/share/QT/${sample} as ${sample}")
#     set(sources "")
#     FILE(GLOB children RELATIVE "${iCenral_Install_Dir}/share/QT/${sample}/src"
#       "${iCenral_Install_Dir}/share/QT/${sample}/src/*")
#     FOREACH(child ${children})
#       LIST(APPEND sources "${iCenral_Install_Dir}/share/QT/${sample}/src/${child}")
#     ENDFOREACH()
#     add_executable(${sample} ${sources})
#     target_link_libraries(${sample} iCentralGui)
#   ENDFOREACH()

# endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

MESSAGE(STATUS "Add sample ShowQImage as ShowQImage")
    set(sources "")
    FILE(GLOB children RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/ShowQImage/src"
      "ShowQImage/src/*")
      message(STATUS ${children})
    FOREACH(child ${children})
      LIST(APPEND sources "${CMAKE_CURRENT_SOURCE_DIR}/ShowQImage/src/${child}")
    ENDFOREACH()
    add_executable(ShowQImage ${sources})
    target_link_libraries(ShowQImage iCentralGui)
    target_include_directories(ShowQImage PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/ShowQImage)
