cmake_minimum_required(VERSION 3.0.0)
project(pi_sonar)

FIND_LIBRARY(pigpio REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  roscpp
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
   CATKIN_DEPENDS diagnostic_updater roscpp sensor_msgs
)

###########
## Build ##
###########


include_directories(
  include
  pigpio_INCLUDES
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES arm*)

  add_executable(pi_sonar src/pi_sonar.cpp)
  target_compile_options(pi_sonar PRIVATE -std=c++11)
  target_link_libraries(pi_sonar ${catkin_LIBRARIES}
                        ${roscpp_LIBRARIES} pigpiod_if2 pthread)
endif()

#############
## Install ##
#############

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES arm*)
install(TARGETS pi_sonar
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    PERMISSIONS OWNER_WRITE OWNER_READ OWNER_EXECUTE
                GROUP_READ GROUP_EXECUTE
                WORLD_READ WORLD_EXECUTE
                SETUID SETGID
)
endif()

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
