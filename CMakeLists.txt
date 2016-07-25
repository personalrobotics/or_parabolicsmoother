cmake_minimum_required(VERSION 2.8.3)
project(or_parabolicsmoother)

find_package(catkin REQUIRED COMPONENTS openrave_catkin)
find_package(OpenRAVE REQUIRED)
find_package(Boost REQUIRED COMPONENTS chrono system)

###################################
## catkin specific configuration ##
###################################
catkin_python_setup()
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS openrave_catkin
  DEPENDS boost
)

###########
## Build ##
###########

include_directories(
  include/${PROJECT_NAME}
  ${OpenRAVE_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
link_directories(
  ${OpenRAVE_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME}
  src/DynamicPath.cpp
  src/ParabolicRamp.cpp
  src/Timer.cpp
  src/ParabolicSmoother.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${OpenRAVE_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

openrave_plugin(${PROJECT_NAME}_plugin
  src/ParabolicSmootherPlugin.cpp
)
target_link_libraries(${PROJECT_NAME}_plugin
  ${PROJECT_NAME}
)


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
