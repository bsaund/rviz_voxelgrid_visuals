cmake_minimum_required(VERSION 2.8.3)
project(rviz_voxelgrid_visuals)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED
        rviz_voxelgrid_visuals_msgs
        rviz
        roscpp
        std_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

set(CMAKE_AUTOMOC ON)

set(SRC_FILES
        src/voxel_display.cpp
        src/voxel_visual.cpp
        include/rviz_voxelgrid_visuals/conversions.h src/conversions.cpp)

catkin_python_setup()


###################################
## catkin specific configuration ##
###################################
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES conversions
        #  CATKIN_DEPENDS other_catkin_pkg
        #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
        ${SRC_FILES}
        )

target_link_libraries(${PROJECT_NAME}
        ${QT_LIBRARIES}
        ${catkin_LIBRARES}
        )


add_library(conversions
        src/conversions.cpp)
target_link_libraries(conversions
        ${catkin_LIBRARIES}
        )


#############
## Install ##
#############

install(TARGETS
        ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(FILES
        plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

