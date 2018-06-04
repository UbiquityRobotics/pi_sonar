cmake_minimum_required(VERSION 3.0.0)
project(ubiquity_sonar)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
   CATKIN_DEPENDS roscpp sensor_msgs
)

###########
## Build ##
###########


include_directories(pigpio ${catkin_INCLUDE_DIRS})

add_library(pigpio STATIC
  pigpio/pigpio.c
  pigpio/command.c 
)
target_compile_options(pigpio PRIVATE -std=c11 -fpic -03)
target_link_libraries(pigpio pthread)

add_executable(ubiquity_sonar src/ubiquity_sonar.cpp)
target_compile_options(ubiquity_sonar PRIVATE -std=c++11)
target_link_libraries(ubiquity_sonar ${catkin_LIBRARIES}
                      ${roscpp_LIBRARIES} pigpio pthread)

#############
## Install ##
#############

install(TARGETS ubiquity_sonar
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hercules_sonar_skirt.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
