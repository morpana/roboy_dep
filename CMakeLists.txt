##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(roboy_DEP)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0 -g")
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/../roboy_managing_node/cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../roboy_managing_node/cmake/options.cmake)
include_directories(
        ${OPLK_BASE_DIR}/stack/proj/linux/liboplkmndrv-pcap
        ${OPLK_BASE_DIR}/apps/common/objdicts/CiA302-4_MN
        ${CMAKE_CURRENT_SOURCE_DIR}/../roboy_powerlink/powerlink/output
        include
)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS qt_build roscpp pcl_ros roboy_communication_middleware cv_bridge tf tf_conversions
        roboy_managing_node common_utilities interactive_markers)
catkin_package(CATKIN_DEPENDS roboy_managing_node roboy_communication_middleware common_utilities)
include_directories(${catkin_INCLUDE_DIRS} ../common_utilities/include)

##############################################################################
# Sources
##############################################################################

SET(SOURCES
        src/main.cpp
        src/DEP.cpp
        src/DerMartiusController.cpp
        src/inspectable.cpp
        src/matrix.cpp
        src/stl_adds.cpp
)

##############################################################################
# Binaries
##############################################################################

add_executable(roboy_DEP ${SOURCES})
add_dependencies(roboy_DEP roboy_communication_middleware_gencpp)
target_link_libraries(roboy_DEP roboy_managing_node)

##############################################################################
# Installation
##############################################################################
install(TARGETS roboy_DEP
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
        package.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
