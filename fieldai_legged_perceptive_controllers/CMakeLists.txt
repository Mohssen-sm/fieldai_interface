cmake_minimum_required(VERSION 3.10)
project(fieldai_legged_perceptive_controllers)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        controller_interface
        legged_perceptive_interface
        legged_controllers
        grid_map_ros
        )

catkin_package(
        CATKIN_DEPENDS
        roscpp
        controller_interface
        legged_perceptive_interface
        legged_controllers
        grid_map_ros
)

#############
## Install ##
#############

# Mark resource files for installation
install(
        DIRECTORY meshes urdf launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
