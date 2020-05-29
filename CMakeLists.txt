cmake_minimum_required(VERSION 2.8.3)
project(laserline)
add_compile_options(-std=c++11)

# find_package(Boost REQUIRED COMPONENTS system thread)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
    dynamic_reconfigure
    roscpp
    sensor_msgs
    rospy
    std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
catkin_package(
    INCLUDE_DIRS include
    # CATKIN_DEPENDS rosconsole
    # LIBRARIES libhokuyo
    # CATKIN_DEPENDS message_runtime
)

# find_package(PkgConfig REQUIRED)

# pkg_check_modules( ncurses REQUIRED ncurses)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

# include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ library
# add_library(laserline
#   src/${PROJECT_NAME}/laserline.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(laserline ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(laserline_node src/laserline_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(laserline_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(laserline_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS laserline laserline_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############
add_executable(laserline
    src/main.cpp
    src/laser_feature_ros.cpp
    src/line_feature.cpp)

target_link_libraries(laserline
    ${catkin_LIBRARIES}
    )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS laserline
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
