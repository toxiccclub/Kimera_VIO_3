# Find iCentral cmake module.
#
# This module will set the following variables:
#
# iCentral_FOUND
# iCentral_INCLUDE_DIRS
# iCentral_LIBRARIES
#
# This module tries to create the target "iCentral::iCentral" that may be used
# when building executables and libraries via target_link_libraries(my_module).
# In this case iCentral_INCLUDE_DIRS will be empty and iCentral_LIBRARIES will
# get the target name to be sure that target is activated properly.
# For old CMake versions iCentral_INCLUDE_DIRS and iCentral_LIBRARIES will have
# includes and libraries paths.
#

if (NOT iCentral_FOUND)

find_path(iCenral_Install_Dir
        NAMES "include/IMVApi.h"
        PATHS
            "$ENV{iCentral_DEV_ROOT}"
            "${iCentral_ROOT}" $ENV{iCentral_ROOT}
            "${CMAKE_INSTALL_PREFIX}"
            "/opt/iCentral/iCentral"
            "/opt/iCentral"
            PATH_SUFFIXES "include"
        )
#set(iCentral_INCLUDE_DIR "${iCenral_Install_Dir}/include")
find_path(iCentral_INCLUDE_DIR
        NAMES "IMVApi.h"
        PATHS
        "${iCenral_Install_Dir}/include"
        PATH_SUFFIXES "include"
        )

find_library(iCentral_MVSDK_LIBRARY
        NAMES "MVSDK"
        PATHS "${iCenral_Install_Dir}/lib"
        PATH_SUFFIXES "lib"
)

find_library(GenICam_GCBase_LIBRARY
            NAMES "GCBase_gcc421_v3_0"
            PATHS
            "${iCenral_Install_Dir}/lib/GenICam/bin"
            "${iCenral_Install_Dir}/lib/GenICam/bin/Linux64_x64"
            "${iCenral_Install_Dir}/lib/GenICam/bin/Linux32_i86"
            "${iCenral_Install_Dir}/lib/GenICam/bin/Linux32_ARM"
            PATH_SUFFIXES "lib"
            )

find_library(GenICam_GenApi_LIBRARY
            NAMES "GenApi_gcc421_v3_0"
            PATHS
            "${iCenral_Install_Dir}/lib/GenICam/bin"
            "${iCenral_Install_Dir}/lib/GenICam/bin/Linux64_x64"
            "${iCenral_Install_Dir}/lib/GenICam/bin/Linux32_i86"
            "${iCenral_Install_Dir}/lib/GenICam/bin/Linux32_ARM"
            PATH_SUFFIXES "lib"
            )

if (GenICam_GCBase_LIBRARY AND GenICam_GenApi_LIBRARY)
        set(GenICam_LIBRARY "${GenICam_GCBase_LIBRARY}")
        LIST(APPEND GenICam_LIBRARY "${GenICam_GenApi_LIBRARY}")
endif ()
if (iCentral_INCLUDE_DIR AND iCentral_MVSDK_LIBRARY AND GenICam_LIBRARY)
        message(STATUS "Found base ICentral SDK: ${iCenral_Install_Dir}")
        set(iCentral_FOUND TRUE)
else ()
        set(iCentral_FOUND FALSE)
endif ()

if (iCentral_FOUND)
set(iCentral_WITH_GUI FALSE)

find_package(Qt5 COMPONENTS Core DBus Network Widgets Xml QUIET)

if(Qt5_FOUND)
find_library(iCentral_GUI_LIBRARY
        NAMES "RecordVideo"
        PATHS "${iCenral_Install_Dir}/lib"
        PATH_SUFFIXES "lib")
find_library(iCentral_RECORDER_LIBRARY
        NAMES "RecordVideo"
        PATHS "${iCenral_Install_Dir}/lib"
        PATH_SUFFIXES "lib")
find_library(iCentral_RENDER_LIBRARY
        NAMES "VideoRender"
        PATHS "${iCenral_Install_Dir}/lib"
        PATH_SUFFIXES "lib")

if(iCentral_GUI_LIBRARY AND iCentral_RECORDER_LIBRARY AND iCentral_RENDER_LIBRARY)
set(iCentral_WITH_GUI TRUE)

endif (iCentral_GUI_LIBRARY AND iCentral_RECORDER_LIBRARY AND iCentral_RENDER_LIBRARY)

endif()
endif()


### Handle REQUIRED / QUIET optional arguments.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(iCentral DEFAULT_MSG
        iCentral_INCLUDE_DIR iCentral_MVSDK_LIBRARY GenICam_LIBRARY
        )
# Set standard CMake FindPackage variables if found.
if (iCentral_FOUND)
    message(STATUS "Configure iCentral variables")
    set(iCentral_INCLUDE_DIRS "${iCentral_INCLUDE_DIR}")
    set(iCentral_LIBRARIES "${iCentral_MVSDK_LIBRARY}" "${GenICam_LIBRARY}")
    if(iCentral_WITH_GUI)
        set(iCentral_GUI_LIBRARIES "${iCentral_GUI_LIBRARY}" "${iCentral_RECORDER_LIBRARY}" "${iCentral_RENDER_LIBRARY}")
    endif()
endif ()
### Create target.
if (CMAKE_VERSION VERSION_GREATER_EQUAL 3.0 AND iCentral_FOUND)
    message(STATUS "Creating interface iCentral target: '${iCentral_MVSDK_LIBRARY}'")
    add_library(iCentral INTERFACE)
    target_link_libraries(iCentral INTERFACE ${iCentral_LIBRARIES} rt pthread)
    target_include_directories(iCentral INTERFACE ${iCentral_INCLUDE_DIRS} )
    target_compile_definitions(iCentral INTERFACE "-DENABLE_iCentral")
    set(iCentral_INCLUDE_DIRS "")
    set(iCentral_LIBRARIES iCentral)

    if(iCentral_WITH_GUI)
        message(STATUS "Creating interface iCentralGui target: '${iCentral_MVSDK_LIBRARY}'")
        add_library(iCentralGui INTERFACE)
        target_link_libraries(iCentralGui INTERFACE iCentral
        "${iCentral_GUI_LIBRARY}" "${iCentral_RECORDER_LIBRARY}" "${iCentral_RENDER_LIBRARY}"
         Qt5::Core Qt5::DBus Qt5::Network Qt5::Widgets Qt5::Xml)
    endif()

    # Only mark internal variables as advanced if we found package, otherwise
    # leave them visible in the standard GUI for the user to set manually.
    mark_as_advanced(FORCE iCentral_INCLUDE_DIR iCentral_LIBRARY)
endif ()

endif (NOT iCentral_FOUND)
