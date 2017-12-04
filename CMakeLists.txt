##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(roboy_dep)

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


find_package(catkin REQUIRED COMPONENTS qt_build roscpp pcl_ros roboy_communication_middleware cv_bridge tf tf_conversions roboy_managing_node common_utilities interactive_markers std_msgs message_generation)

##########################
## Declare ROS messages ##
##########################

add_message_files(FILES command.msg depParameters.msg cArray.msg depMatrix.msg transition.msg transition_start.msg linear_combination.msg brain_id.msg stop.msg trigger_ref.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS roboy_managing_node roboy_communication_middleware common_utilities std_msgs message_runtime)

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

##########
# Build ##
##########

add_executable(roboy_dep ${SOURCES})
add_dependencies(roboy_dep roboy_communication_middleware_gencpp roboy_dep_generate_messsages_cpp)
target_link_libraries(roboy_dep roboy_managing_node)

#add_executable(dep_interface src/dep_interface.cpp)
#add_dependencies(dep_interface ${catkin_EXPORTED_TARGETS} roboy_dep roboy_dep_generate_messsages_cpp)
#target_link_libraries(dep_interface ${catkin_LIBRARIES})

##############################################################################
# Installation
##############################################################################
install(TARGETS 
        roboy_dep
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


#install(TARGETS 
#        dep_interface
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)


install(FILES
        package.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
